/*
 * DDC channelizer -- two-stage per-channel digital downconversion
 *
 * Architecture:
 *   Stage 1: Per-band NCO mix + coarse decimation from SDR rate to an
 *            intermediate rate (400 kHz for MSK/BPSK bands, 200 kHz for
 *            OQPSK bands).  One band per frequency cluster.
 *   Stage 2: Per-channel NCO mix (relative to band center) + fine
 *            decimation from intermediate rate to ~48 kHz output.
 *   Final:   127-tap cleanup filter at output rate (unchanged).
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "simd_kernels.h"
#include <stdio.h>
#include <complex.h>

#include "channelizer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Each decimation stage has a short FIR lowpass that anti-aliases
 * before downsampling. With decimation <= 16 per stage and 63 taps,
 * the filter works well. Stages cascade: e.g. 50x = 5 * 10.
 */
#define MAX_STAGES       4
#define STAGE_FIR_TAPS   63   /* both stage-1 and stage-2 FIR tap count */
#define CLEANUP_FIR_TAPS 127  /* final narrowband filter at output rate */
#define MAX_BANDS        8    /* maximum concurrent band groups */

/* ------------------------------------------------------------------ */
/* Decimation stage                                                     */
/* ------------------------------------------------------------------ */

typedef struct {
    int decimation;
    int count;
    float fir_taps[STAGE_FIR_TAPS];
    float complex fir_hist[STAGE_FIR_TAPS];
    int fir_idx;
} decim_stage_t;

/* ------------------------------------------------------------------ */
/* Per-band state (stage 1)                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    int active;
    double center_freq;       /* absolute Hz */
    double intermediate_rate; /* Hz after stage-1 decimation */

    /* Stage-1 NCO (offset from SDR center) */
    double nco_freq;
    float complex nco_phasor;
    float complex nco_current;
    int nco_renorm;

    /* Stage-1 cascaded decimation */
    decim_stage_t stages[MAX_STAGES];
    int num_stages;

    /* Intermediate sample buffer (output of stage 1) */
    float complex *inter_buf;
    int inter_len;
    int inter_cap;

    /* Which channel indices belong to this band */
    int channel_indices[MAX_CHANNELS];
    int num_channels;

    /* Modulation family — channels of different family never share a band
     * even if their frequency ranges are compatible. Prevents asymmetric
     * centroid placement when distinct spectral clusters are nearby. */
    int family;
} band_state_t;

/* ------------------------------------------------------------------ */
/* Per-channel state (stage 2 + cleanup)                               */
/* ------------------------------------------------------------------ */

typedef struct {
    int active;
    int channel_id;
    channel_type_t type;
    int band_idx;             /* index into channelizer->bands[] */

    /* Stage-2 NCO (offset from band center) */
    double nco_freq;
    float complex nco_phasor;
    float complex nco_current;
    int nco_renorm;

    /* Stage-2 cascaded decimation (intermediate_rate -> output_rate) */
    decim_stage_t stages[MAX_STAGES];
    int num_stages;

    /* Final cleanup filter */
    float cleanup_taps[CLEANUP_FIR_TAPS];
    float complex cleanup_hist[CLEANUP_FIR_TAPS * 2];
    int cleanup_idx;
    int has_cleanup;

    /* Per-channel digital gain (applied after cleanup filter) */
    float gain;

    /* Output buffer */
    float complex *out_buf;
    int out_len;
    int out_cap;
} channel_state_t;

/* ------------------------------------------------------------------ */
/* Channelizer struct                                                   */
/* ------------------------------------------------------------------ */

struct channelizer {
    double center_freq;
    double samp_rate;
    channel_cb_t cb;
    void *user;

    /* Stage-2 channels */
    channel_state_t channels[MAX_CHANNELS];
    int num_channels;

    /* Stage-1 bands */
    band_state_t bands[MAX_BANDS];
    int num_bands;
};

/* ------------------------------------------------------------------ */
/* FIR design                                                           */
/* ------------------------------------------------------------------ */

static void design_lowpass(float *taps, int num_taps, double cutoff) {
    int M = num_taps - 1;
    double sum = 0;

    for (int i = 0; i < num_taps; i++) {
        double n = i - M / 2.0;
        double h;
        if (fabs(n) < 1e-10)
            h = 2.0 * cutoff;
        else
            h = sin(2.0 * M_PI * cutoff * n) / (M_PI * n);

        double w = 0.42 - 0.5 * cos(2.0 * M_PI * i / M)
                        + 0.08 * cos(4.0 * M_PI * i / M);
        h *= w;
        sum += h;
        taps[i] = (float)h;
    }

    for (int i = 0; i < num_taps; i++)
        taps[i] /= (float)sum;
}

/* ------------------------------------------------------------------ */
/* Decimation stage processing                                          */
/* ------------------------------------------------------------------ */

static inline int decim_stage_process(decim_stage_t *st,
                                       float complex in,
                                       float complex *out) {
    st->fir_hist[st->fir_idx] = in;
    st->fir_idx = (st->fir_idx + 1) % STAGE_FIR_TAPS;

    if (++st->count < st->decimation)
        return 0;
    st->count = 0;

    float complex acc = 0;
    int idx = st->fir_idx;
    for (int t = 0; t < STAGE_FIR_TAPS; t++) {
        idx--;
        if (idx < 0) idx = STAGE_FIR_TAPS - 1;
        acc += st->fir_hist[idx] * st->fir_taps[t];
    }
    *out = acc;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Decimation planning                                                  */
/* ------------------------------------------------------------------ */

static int plan_decimation(int total, int max_per_stage,
                            int *decim, int max_stages) {
    int n = 0;
    int remaining = total;

    while (remaining > 1 && n < max_stages) {
        if (remaining <= max_per_stage) {
            decim[n++] = remaining;
            remaining = 1;
        } else {
            int best = 2;
            for (int d = max_per_stage; d >= 2; d--) {
                if (remaining % d == 0) {
                    best = d;
                    break;
                }
            }
            decim[n++] = best;
            remaining /= best;
        }
    }

    if (remaining > 1 && n < max_stages)
        decim[n++] = remaining;

    return n;
}

/* ------------------------------------------------------------------ */
/* Channel / band metadata helpers                                      */
/* ------------------------------------------------------------------ */

static double target_output_rate(channel_type_t type) {
    switch (type) {
    case CHAN_STDC_EGC:   return 1200.0 * 16.0;
    case CHAN_AERO_600:   return 48000.0;
    case CHAN_AERO_1200:  return 48000.0;
    case CHAN_AERO_10500: return 48000.0;
    case CHAN_AERO_8400:  return 48000.0;
    default: return 48000.0;
    }
}

/* Channels share a stage-1 band only if modulation family matches
 * (OQPSK 10500 and 8400 differ in RRC α so they're kept separate). */
static int modulation_family(channel_type_t type) {
    switch (type) {
    case CHAN_STDC_EGC:   return 0;  /* BPSK */
    case CHAN_AERO_600:
    case CHAN_AERO_1200:  return 1;  /* MSK */
    case CHAN_AERO_10500: return 2;  /* OQPSK α=1.0 */
    case CHAN_AERO_8400:  return 3;  /* OQPSK α=0.6 */
    default: return -1;
    }
}

/* Stage-1 intermediate rate: narrower for OQPSK, wider for MSK/BPSK. */
static double target_intermediate_rate(channel_type_t type) {
    switch (type) {
    case CHAN_AERO_10500:
    case CHAN_AERO_8400:
        return 200000.0;   /* 200 kHz for OQPSK */
    default:
        return 400000.0;   /* 400 kHz for BPSK / STD-C */
    }
}

/* Find the largest factor of n that is <= limit.
 * Used to maximize stage-1 decimation while keeping the intermediate
 * rate wide enough for all channels in the cluster. */
static int largest_factor_leq(int n, int limit) {
    if (n <= 1) return 1;
    int best = 1;
    for (int d = 2; d * d <= n; d++) {
        if (n % d != 0) continue;
        if (d <= limit && d > best) best = d;
        int other = n / d;
        if (other <= limit && other > best) best = other;
    }
    if (n <= limit && n > best) best = n;
    return best;
}

/* Per-channel digital gain. Unity: the demodulators' feedIQ path
 * already applies 5.0x scaling. */
static float channel_gain(channel_type_t type) {
    (void)type;
    return 1.0f;
}

static double signal_bandwidth(channel_type_t type) {
    switch (type) {
    case CHAN_STDC_EGC:   return 4800.0;
    case CHAN_AERO_600:   return 6000.0;
    case CHAN_AERO_1200:  return 6000.0;
    case CHAN_AERO_10500: return 21000.0;  /* α=1.0 → fb*(1+α) = 21 kHz */
    case CHAN_AERO_8400:  return 14000.0;   /* α=0.6 → fb*(1+α) = 13.4 kHz */
    default: return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Band initialisation                                                  */
/* ------------------------------------------------------------------ */

static int init_band(band_state_t *b, double center_freq,
                     double samp_rate, int s1_decim) {
    memset(b, 0, sizeof(*b));
    b->active = 1;
    b->center_freq = center_freq;
    if (s1_decim < 1) s1_decim = 1;
    b->intermediate_rate = samp_rate / s1_decim;

    b->nco_freq   = 0.0;   /* caller fills after SDR center is known */
    b->nco_phasor = 1.0f;
    b->nco_current = 1.0f;

    /* Plan stage-1 decimation cascade */
    int stage_decims[MAX_STAGES];
    b->num_stages = plan_decimation(s1_decim, 16, stage_decims, MAX_STAGES);

    /* Design per-stage anti-alias filters */
    double stage_rate = samp_rate;
    for (int i = 0; i < b->num_stages; i++) {
        decim_stage_t *st = &b->stages[i];
        st->decimation = stage_decims[i];
        st->count = 0;
        st->fir_idx = 0;
        memset(st->fir_hist, 0, sizeof(st->fir_hist));

        double cutoff = 0.4 / st->decimation;
        design_lowpass(st->fir_taps, STAGE_FIR_TAPS, cutoff);

        stage_rate /= st->decimation;
    }
    /* stage_rate should now equal b->intermediate_rate */

    /* Intermediate buffer: hold up to 0.1 s worth of samples */
    b->inter_cap = (int)(b->intermediate_rate * 0.1) + 512;
    b->inter_buf = malloc(b->inter_cap * sizeof(float complex));
    if (!b->inter_buf) return -1;
    b->inter_len = 0;
    b->num_channels = 0;

    return 0;
}

/* ------------------------------------------------------------------ */
/* Find or create a band that can absorb a new channel                  */
/* ------------------------------------------------------------------ */

/* Returns band index (>=0) or -1 on failure.
 * s1_decim is the stage-1 decimation factor chosen by the caller
 * to guarantee exact total decimation (s1 * s2 = total). */
static int find_or_create_band(channelizer_t *ch, double freq,
                                channel_type_t type,
                                int channel_slot, int s1_decim) {
    double actual_inter = ch->samp_rate / (double)s1_decim;
    int family = modulation_family(type);

    /* Tolerance: channel must fit within ±40% of intermediate BW
     * around the band center */
    double tol = actual_inter * 0.4;

    /* Search for an existing compatible band */
    for (int b = 0; b < ch->num_bands; b++) {
        band_state_t *bd = &ch->bands[b];
        if (!bd->active) continue;
        /* Channels of different modulation family never share a band */
        if (bd->family != family) continue;
        /* Check rate compatibility (must have same s1_decim) */
        if (fabs(bd->intermediate_rate - actual_inter) >
            actual_inter * 0.01) continue;
        /* Check frequency proximity */
        if (fabs(bd->center_freq - freq) <= tol &&
            bd->num_channels < MAX_CHANNELS) {
            bd->channel_indices[bd->num_channels++] = channel_slot;
            return b;
        }
    }

    /* No suitable band found — create a new one */
    if (ch->num_bands >= MAX_BANDS) {
        fprintf(stderr, "Channelizer: exceeded MAX_BANDS (%d)\n", MAX_BANDS);
        return -1;
    }

    int bidx = ch->num_bands++;
    band_state_t *bd = &ch->bands[bidx];

    if (init_band(bd, freq, ch->samp_rate, s1_decim) != 0)
        return -1;

    bd->family = family;

    /* Set NCO: shift band center to DC */
    bd->nco_freq = freq - ch->center_freq;
    double phase_inc = -2.0 * M_PI * bd->nco_freq / ch->samp_rate;
    bd->nco_phasor  = cosf((float)phase_inc) + sinf((float)phase_inc) * I;
    bd->nco_current = 1.0f;

    bd->channel_indices[bd->num_channels++] = channel_slot;

    extern int verbose;
    if (verbose)
        fprintf(stderr,
                "Channelizer: new band %d  center=%.3f MHz  "
                "s1_decim=%d  inter_rate=%.0f Hz\n",
                bidx, freq / 1e6, s1_decim, bd->intermediate_rate);

    return bidx;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

channelizer_t *channelizer_create(double center_freq, double samp_rate,
                                   channel_cb_t cb, void *user) {
    channelizer_t *ch = calloc(1, sizeof(*ch));
    if (!ch) return NULL;

    ch->center_freq = center_freq;
    ch->samp_rate   = samp_rate;
    ch->cb   = cb;
    ch->user = user;
    ch->num_channels = 0;
    ch->num_bands    = 0;

    return ch;
}

int channelizer_add_channel(channelizer_t *ch, double freq,
                             channel_type_t type, int channel_id) {
    if (ch->num_channels >= MAX_CHANNELS)
        return -1;

    int slot = ch->num_channels;
    channel_state_t *c = &ch->channels[slot];
    memset(c, 0, sizeof(*c));

    c->active     = 1;
    c->channel_id = channel_id;
    c->type       = type;

    /* Compute total decimation first to guarantee correct output rate.
     * Then factor into s1 * s2 with s1 close to target intermediate. */
    double out_rate = target_output_rate(type);
    int total_decim = (int)(ch->samp_rate / out_rate);
    if (total_decim < 1) total_decim = 1;

    double want_inter = target_intermediate_rate(type);
    int max_s1 = (int)(ch->samp_rate / want_inter);
    if (max_s1 < 1) max_s1 = 1;
    int s1_decim = largest_factor_leq(total_decim, max_s1);
    int s2_decim = total_decim / s1_decim;
    if (s2_decim < 1) s2_decim = 1;

    /* Find or create stage-1 band with exact s1_decim */
    int bidx = find_or_create_band(ch, freq, type, slot, s1_decim);
    if (bidx < 0) return -1;
    c->band_idx = bidx;

    band_state_t *bd = &ch->bands[bidx];
    double inter_rate = bd->intermediate_rate;

    /* Stage-2 NCO: mix channel center to DC relative to band center */
    c->nco_freq = freq - bd->center_freq;
    double phase_inc = -2.0 * M_PI * c->nco_freq / inter_rate;
    c->nco_phasor  = cosf((float)phase_inc) + sinf((float)phase_inc) * I;
    c->nco_current = 1.0f;
    c->nco_renorm  = 0;

    /* Stage-2 decimation: inter_rate -> output_rate */
    int stage_decims[MAX_STAGES];
    c->num_stages = plan_decimation(s2_decim, 16, stage_decims, MAX_STAGES);

    double stage_rate = inter_rate;
    for (int i = 0; i < c->num_stages; i++) {
        decim_stage_t *st = &c->stages[i];
        st->decimation = stage_decims[i];
        st->count      = 0;
        st->fir_idx    = 0;
        memset(st->fir_hist, 0, sizeof(st->fir_hist));

        double cutoff = 0.4 / st->decimation;
        design_lowpass(st->fir_taps, STAGE_FIR_TAPS, cutoff);

        stage_rate /= st->decimation;
    }
    /* stage_rate is the actual output rate */

    /* Final cleanup filter */
    double sig_bw = signal_bandwidth(type);
    c->cleanup_idx = 0;
    memset(c->cleanup_hist, 0, sizeof(c->cleanup_hist));
    if (sig_bw > 0 && sig_bw < stage_rate * 0.8) {
        double cleanup_cutoff = sig_bw / (2.0 * stage_rate);
        design_lowpass(c->cleanup_taps, CLEANUP_FIR_TAPS, cleanup_cutoff);
        c->has_cleanup = 1;
    } else {
        c->has_cleanup = 0;
    }

    c->gain = channel_gain(type);

    /* Output buffer */
    c->out_cap = (int)(stage_rate * 0.1) + 512;
    c->out_buf = malloc(c->out_cap * sizeof(float complex));
    if (!c->out_buf) return -1;
    c->out_len = 0;

    extern int verbose;
    if (verbose)
        fprintf(stderr,
                "Channelizer: ch%d  freq=%.3f MHz  type=%d  "
                "band=%d  s1=%d  s2=%d  total=%d  out_rate=%.0f Hz\n",
                channel_id, freq / 1e6, (int)type,
                bidx, s1_decim, s2_decim, total_decim, stage_rate);

    ch->num_channels++;
    return 0;
}

static void process_block(channelizer_t *ch, const float *samples,
                          int num_samples) {
    /*
     * Stage 1: for each input sample, NCO-mix and decimate into each
     * band's intermediate buffer.
     */
    for (int s = 0; s < num_samples; s++) {
        float complex input = samples[s * 2] + samples[s * 2 + 1] * I;

        for (int b = 0; b < ch->num_bands; b++) {
            band_state_t *bd = &ch->bands[b];
            if (!bd->active) continue;

            /* NCO mix to band center */
            float complex x = input * bd->nco_current;
            bd->nco_current *= bd->nco_phasor;

            if (++bd->nco_renorm >= 1024) {
                bd->nco_renorm = 0;
                float mag = cabsf(bd->nco_current);
                if (mag > 0.0f)
                    bd->nco_current /= mag;
            }

            /* Stage-1 decimation cascade */
            int produced = 1;
            for (int i = 0; i < bd->num_stages && produced; i++) {
                float complex out;
                produced = decim_stage_process(&bd->stages[i], x, &out);
                x = out;
            }
            if (!produced) continue;

            /* Append to intermediate buffer */
            if (bd->inter_len < bd->inter_cap)
                bd->inter_buf[bd->inter_len++] = x;
        }
    }

    /*
     * Stage 2: drain each band's intermediate buffer through per-channel
     * NCO + decimation + cleanup filter.
     */
    for (int b = 0; b < ch->num_bands; b++) {
        band_state_t *bd = &ch->bands[b];
        if (!bd->active || bd->inter_len == 0) continue;

        for (int s = 0; s < bd->inter_len; s++) {
            float complex isamp = bd->inter_buf[s];

            for (int ci = 0; ci < bd->num_channels; ci++) {
                int cidx = bd->channel_indices[ci];
                channel_state_t *cs = &ch->channels[cidx];
                if (!cs->active) continue;

                /* Stage-2 NCO mix relative to band center */
                float complex x = isamp * cs->nco_current;
                cs->nco_current *= cs->nco_phasor;

                if (++cs->nco_renorm >= 1024) {
                    cs->nco_renorm = 0;
                    float mag = cabsf(cs->nco_current);
                    if (mag > 0.0f)
                        cs->nco_current /= mag;
                }

                /* Stage-2 decimation cascade */
                int produced = 1;
                for (int i = 0; i < cs->num_stages && produced; i++) {
                    float complex out;
                    produced = decim_stage_process(&cs->stages[i], x, &out);
                    x = out;
                }
                if (!produced) continue;

                /* Cleanup filter (double-buffer trick, simd_fir_ccf) */
                if (cs->has_cleanup) {
                    cs->cleanup_hist[cs->cleanup_idx] = x;
                    cs->cleanup_hist[cs->cleanup_idx + CLEANUP_FIR_TAPS] = x;
                    cs->cleanup_idx = (cs->cleanup_idx + 1) % CLEANUP_FIR_TAPS;

                    float complex out;
                    simd_fir_ccf(cs->cleanup_taps, CLEANUP_FIR_TAPS,
                                 &cs->cleanup_hist[cs->cleanup_idx], &out, 1);
                    x = out;
                }

                /* Apply per-channel gain and accumulate */
                if (cs->out_len < cs->out_cap)
                    cs->out_buf[cs->out_len++] = x * cs->gain;
            }
        }

        bd->inter_len = 0;
    }

    /* Flush output buffers */
    int flush_threshold = 32;
    for (int c = 0; c < ch->num_channels; c++) {
        channel_state_t *cs = &ch->channels[c];
        if (cs->out_len >= flush_threshold) {
            if (ch->cb)
                ch->cb(cs->channel_id, cs->type, cs->out_buf,
                       cs->out_len, ch->user);
            cs->out_len = 0;
        }
    }
}

/* Intermediate/output buffers hold 0.1 s; feed at most 0.05 s per pass so
 * long input blocks (low-rate files) are not truncated. */
void channelizer_process(channelizer_t *ch, const float *samples,
                          int num_samples) {
    int max_block = (int)(ch->samp_rate * 0.05);
    if (max_block < 1) max_block = 1;
    for (int off = 0; off < num_samples; off += max_block) {
        int n = num_samples - off;
        if (n > max_block) n = max_block;
        process_block(ch, samples + (size_t)off * 2, n);
    }
}

void channelizer_process_i8(channelizer_t *ch, const int8_t *samples,
                             int num_samples) {
    int block = 4096;
    float complex cbuf[block];

    for (int off = 0; off < num_samples; off += block) {
        int n = num_samples - off;
        if (n > block) n = block;

        simd_convert_i8_cf(samples + off * 2, cbuf, n);
        channelizer_process(ch, (float *)cbuf, n);
    }
}

int channelizer_has_freq(channelizer_t *ch, double freq, double tolerance) {
    if (!ch) return 0;
    for (int c = 0; c < ch->num_channels; c++) {
        int bidx = ch->channels[c].band_idx;
        double ch_freq = ch->bands[bidx].center_freq
                       + ch->channels[c].nco_freq;
        if (fabs(ch_freq - freq) < tolerance)
            return 1;
    }
    return 0;
}

double channelizer_output_rate(channelizer_t *ch, int channel_id) {
    for (int c = 0; c < ch->num_channels; c++) {
        if (ch->channels[c].channel_id != channel_id) continue;
        channel_state_t *cs = &ch->channels[c];
        double rate = ch->bands[cs->band_idx].intermediate_rate;
        for (int i = 0; i < cs->num_stages; i++)
            rate /= cs->stages[i].decimation;
        return rate;
    }
    return 0;
}

void channelizer_adjust_center(channelizer_t *ch, double offset_hz) {
    if (!ch) return;

    /* Adjust band NCOs; channel NCOs are relative to band center so
     * they stay unchanged. */
    for (int b = 0; b < ch->num_bands; b++) {
        band_state_t *bd = &ch->bands[b];
        if (!bd->active) continue;
        bd->nco_freq += offset_hz;
        double phase_inc = -2.0 * M_PI * bd->nco_freq / ch->samp_rate;
        bd->nco_phasor  = cosf((float)phase_inc) + sinf((float)phase_inc) * I;
        /* Keep current phase — just change the rate */
    }

    ch->center_freq -= offset_hz;
    fprintf(stderr,
            "Channelizer: adjusted center by %.0f Hz (new: %.3f MHz)\n",
            offset_hz, ch->center_freq / 1e6);
}

void channelizer_finalize(channelizer_t *ch) {
    if (!ch) return;

    /* Recenter each band's NCO on the centroid of its member channels so
     * no channel sits at DC (where DC offset and 1/f noise hurt). */
    for (int b = 0; b < ch->num_bands; b++) {
        band_state_t *bd = &ch->bands[b];
        if (!bd->active || bd->num_channels == 0) continue;

        /* Compute centroid from member channels */
        double freq_sum = 0;
        double freq_lo = 1e18, freq_hi = 0;
        for (int ci = 0; ci < bd->num_channels; ci++) {
            int cidx = bd->channel_indices[ci];
            channel_state_t *cs = &ch->channels[cidx];
            double ch_freq = bd->center_freq + cs->nco_freq;
            freq_sum += ch_freq;
            if (ch_freq < freq_lo) freq_lo = ch_freq;
            if (ch_freq > freq_hi) freq_hi = ch_freq;
        }
        double centroid = freq_sum / bd->num_channels;
        double shift = centroid - bd->center_freq;
        if (fabs(shift) < 1.0) continue;  /* already centered */

        /* Update band NCO to mix the new centroid to DC */
        bd->nco_freq += shift;
        double phase_inc = -2.0 * M_PI * bd->nco_freq / ch->samp_rate;
        bd->nco_phasor  = cosf((float)phase_inc) + sinf((float)phase_inc) * I;
        bd->nco_current = 1.0f;
        bd->center_freq = centroid;

        /* Rebalance each channel's stage-2 NCO relative to new centroid */
        double inter_rate = bd->intermediate_rate;
        for (int ci = 0; ci < bd->num_channels; ci++) {
            int cidx = bd->channel_indices[ci];
            channel_state_t *cs = &ch->channels[cidx];
            cs->nco_freq -= shift;
            double cph_inc = -2.0 * M_PI * cs->nco_freq / inter_rate;
            cs->nco_phasor  = cosf((float)cph_inc) + sinf((float)cph_inc) * I;
            cs->nco_current = 1.0f;
        }

        extern int verbose;
        if (verbose)
            fprintf(stderr,
                    "Channelizer: band %d NCO centered at %.3f MHz "
                    "(mix point only; channels remain at their spec frequencies %.3f-%.3f MHz)\n",
                    b, centroid / 1e6, freq_lo / 1e6, freq_hi / 1e6);
    }
}

void channelizer_destroy(channelizer_t *ch) {
    if (!ch) return;

    for (int c = 0; c < ch->num_channels; c++)
        free(ch->channels[c].out_buf);

    for (int b = 0; b < ch->num_bands; b++)
        free(ch->bands[b].inter_buf);

    free(ch);
}
