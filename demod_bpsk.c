/*
 * Coherent BPSK demodulator for STD-C / EGC (1200 baud)
 *
 * FFT coarse acquisition + squaring FLL -> RRC matched filter -> AGC ->
 * Gardner timing recovery (linear interpolation, fractional sps) ->
 * decision-directed Costas loop at symbol rate. Parameters follow SatDump's
 * Inmarsat STD-C pipeline.
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <complex.h>

#include "demod_bpsk.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SOFT_BIT_BUF 512
#define RRC_ALPHA    0.6
#define RRC_SPAN     4       /* symbols each side */
/* Per-symbol rates; divided by sps so loop behavior is rate independent */
#define AGC_RATE     0.016
#define FLL_AVG      0.08
#define FLL_GAIN     0.0016
#define FLL_MAX_HZ   3000.0
#define ACQ_N        8192    /* coarse acquisition FFT size */
#define ACQ_AVG      4       /* blocks of power spectrum averaged per decision */
#define ACQ_SNR      8.0f    /* averaged peak / mean power to accept */
#define ACQ_MIN_HZ   150.0   /* residuals below this are left to the loops */
#define TIMING_BW    0.01
#define COSTAS_BW    0.03
#define SOFT_SCALE   0.7f

struct bpsk_demod {
    double sps;

    /* NCO driven by the FLL */
    float nco_phase, nco_freq, nco_max;
    float complex fll_acc, prev_sq;
    float complex *acq_buf, *acq_fft;
    float *acq_pwr;
    int acq_n, acq_blocks;
    int acq_locked;          /* FLL held until acquisition confirms frequency */
    float acq_min;

    /* RRC matched filter, doubled history for contiguous dot product */
    float *taps;
    int ntaps;
    float complex *hist;
    int hpos;

    float agc_pwr, agc_rate;
    float fll_avg, fll_gain;

    /* Gardner timing recovery */
    float complex prev_filt, mid, prev_sym;
    double t_next;           /* samples until next strobe */
    double rate_adj;         /* samples per symbol correction */
    double tim_alpha, tim_beta;
    int mid_strobe;

    /* Costas loop */
    float cphase, cfreq;
    float c_alpha, c_beta;
    float sym_mag;

    bpsk_bit_cb_t cb;
    void *user;
    float soft_bits[SOFT_BIT_BUF];
    int num_soft_bits;
};

/* Second-order loop gains, damping 0.707 */
static void loop_gains(double bw, double *alpha, double *beta)
{
    double zeta = 0.707;
    double denom = 1.0 + 2.0 * zeta * bw + bw * bw;
    *alpha = 4.0 * zeta * bw / denom;
    *beta = 4.0 * bw * bw / denom;
}

static double rrc(double t, double a)
{
    if (fabs(t) < 1e-9)
        return 1.0 - a + 4.0 * a / M_PI;
    if (fabs(fabs(t) - 1.0 / (4.0 * a)) < 1e-9)
        return a / sqrt(2.0) * ((1 + 2 / M_PI) * sin(M_PI / (4 * a)) +
                                (1 - 2 / M_PI) * cos(M_PI / (4 * a)));
    return (sin(M_PI * t * (1 - a)) + 4 * a * t * cos(M_PI * t * (1 + a))) /
           (M_PI * t * (1 - (4 * a * t) * (4 * a * t)));
}

static void fft(float complex *x, int n)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            float complex t = x[i];
            x[i] = x[j];
            x[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float complex w = cexpf(-2.0f * (float)M_PI * I / len);
        for (int i = 0; i < n; i += len) {
            float complex wk = 1.0f;
            for (int k = 0; k < len / 2; k++) {
                float complex u = x[i + k], v = x[i + k + len / 2] * wk;
                x[i + k] = u + v;
                x[i + k + len / 2] = u - v;
                wk *= w;
            }
        }
    }
}

static float wrap_phase(float p)
{
    while (p > (float)M_PI) p -= 2.0f * (float)M_PI;
    while (p < -(float)M_PI) p += 2.0f * (float)M_PI;
    return p;
}

bpsk_demod_t *bpsk_demod_create(double samp_rate, double symbol_rate,
                                 bpsk_bit_cb_t cb, void *user)
{
    bpsk_demod_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->sps = samp_rate / symbol_rate;
    d->nco_max = (float)(2.0 * M_PI * FLL_MAX_HZ / samp_rate);
    d->agc_rate = (float)(AGC_RATE / d->sps);
    d->fll_avg = (float)(FLL_AVG / d->sps);
    d->fll_gain = (float)(FLL_GAIN / d->sps);

    int half = (int)ceil(RRC_SPAN * d->sps);
    d->ntaps = 2 * half + 1;
    d->taps = malloc(sizeof(float) * d->ntaps);
    d->hist = calloc(2 * d->ntaps, sizeof(float complex));
    d->acq_buf = malloc(sizeof(float complex) * ACQ_N);
    d->acq_fft = malloc(sizeof(float complex) * ACQ_N);
    d->acq_pwr = calloc(ACQ_N, sizeof(float));
    d->acq_min = (float)(2.0 * M_PI * ACQ_MIN_HZ / samp_rate);
    if (!d->taps || !d->hist || !d->acq_buf || !d->acq_fft || !d->acq_pwr) {
        bpsk_demod_destroy(d);
        return NULL;
    }
    double energy = 0;
    for (int i = 0; i < d->ntaps; i++) {
        double v = rrc((i - half) / d->sps, RRC_ALPHA);
        d->taps[i] = (float)v;
        energy += v * v;
    }
    for (int i = 0; i < d->ntaps; i++)
        d->taps[i] /= (float)sqrt(energy);

    double a, b;
    loop_gains(TIMING_BW, &a, &b);
    d->tim_alpha = a * d->sps;
    d->tim_beta = b * d->sps;
    loop_gains(COSTAS_BW, &a, &b);
    d->c_alpha = (float)a;
    d->c_beta = (float)b;

    d->t_next = d->sps / 2.0;
    d->mid_strobe = 1;
    d->sym_mag = 1.0f;
    d->cb = cb;
    d->user = user;
    return d;
}

static void flush_bits(bpsk_demod_t *d)
{
    if (d->num_soft_bits > 0 && d->cb)
        d->cb(d->soft_bits, d->num_soft_bits, d->user);
    d->num_soft_bits = 0;
}

/* Squared BPSK carries a tone at twice the residual offset; jump the NCO
 * when it is beyond what the FLL can see through the RRC passband. */
static void coarse_acquire(bpsk_demod_t *d)
{
    memcpy(d->acq_fft, d->acq_buf, sizeof(float complex) * ACQ_N);
    fft(d->acq_fft, ACQ_N);
    for (int k = 0; k < ACQ_N; k++)
        d->acq_pwr[k] += crealf(d->acq_fft[k]) * crealf(d->acq_fft[k]) +
                         cimagf(d->acq_fft[k]) * cimagf(d->acq_fft[k]);
    if (++d->acq_blocks < ACQ_AVG)
        return;

    int peak = 0;
    float peak_pwr = 0, sum = 0;
    for (int k = 0; k < ACQ_N; k++) {
        sum += d->acq_pwr[k];
        if (d->acq_pwr[k] > peak_pwr) {
            peak_pwr = d->acq_pwr[k];
            peak = k;
        }
    }
    memset(d->acq_pwr, 0, sizeof(float) * ACQ_N);
    d->acq_blocks = 0;
    if (peak_pwr < ACQ_SNR * sum / ACQ_N)
        return;

    int k = peak > ACQ_N / 2 ? peak - ACQ_N : peak;
    float df = (float)M_PI * k / ACQ_N;   /* half of 2*pi*k/N */
    d->acq_locked = 1;
    if (fabsf(df) < d->acq_min)
        return;
    d->nco_freq += df;
    d->fll_acc = 0;
    d->cfreq = 0;
}

static void symbol_out(bpsk_demod_t *d, float complex y)
{
    /* Gardner TED; rotation invariant so it runs ahead of the Costas loop */
    float e = crealf((d->prev_sym - y) * conjf(d->mid));
    if (e > 1.0f) e = 1.0f;
    if (e < -1.0f) e = -1.0f;
    d->prev_sym = y;

    double max_adj = d->sps * 0.005;
    d->rate_adj += d->tim_beta * e;
    if (d->rate_adj > max_adj) d->rate_adj = max_adj;
    if (d->rate_adj < -max_adj) d->rate_adj = -max_adj;
    d->t_next += d->tim_alpha * e;

    float complex s = y * cexpf(-I * d->cphase);
    float re = crealf(s);
    float err = (re > 0 ? 1.0f : -1.0f) * cimagf(s);
    if (err > 1.0f) err = 1.0f;
    if (err < -1.0f) err = -1.0f;
    d->cfreq += d->c_beta * err;
    d->cphase = wrap_phase(d->cphase + d->cfreq + d->c_alpha * err);

    d->sym_mag += 0.01f * (fabsf(re) - d->sym_mag);
    if (d->num_soft_bits >= SOFT_BIT_BUF)
        flush_bits(d);
    d->soft_bits[d->num_soft_bits++] = re / (d->sym_mag + 1e-12f) * SOFT_SCALE;
}

void bpsk_demod_process(bpsk_demod_t *d, const float complex *samples,
                         int num_samples)
{
    for (int i = 0; i < num_samples; i++) {
        float complex x = samples[i] * cexpf(-I * d->nco_phase);
        d->nco_phase = wrap_phase(d->nco_phase + d->nco_freq);

        d->acq_buf[d->acq_n++] = x * x;
        if (d->acq_n == ACQ_N) {
            coarse_acquire(d);
            d->acq_n = 0;
        }

        d->hist[d->hpos] = x;
        d->hist[d->hpos + d->ntaps] = x;
        if (++d->hpos >= d->ntaps) d->hpos = 0;
        const float complex *h = &d->hist[d->hpos];
        float complex f = 0;
        for (int k = 0; k < d->ntaps; k++)
            f += d->taps[k] * h[k];

        float p = crealf(f) * crealf(f) + cimagf(f) * cimagf(f);
        if (d->agc_pwr == 0.0f) d->agc_pwr = p + 1e-20f;
        d->agc_pwr += d->agc_rate * (p - d->agc_pwr);
        f /= sqrtf(d->agc_pwr + 1e-20f);

        /* BPSK squared has a tone at twice the residual offset */
        float complex sq = f * f;
        d->fll_acc += d->fll_avg * (sq * conjf(d->prev_sq) - d->fll_acc);
        d->prev_sq = sq;
        if (d->acq_locked)
            d->nco_freq += d->fll_gain * 0.5f * cargf(d->fll_acc);
        if (d->nco_freq > d->nco_max) d->nco_freq = d->nco_max;
        if (d->nco_freq < -d->nco_max) d->nco_freq = -d->nco_max;

        d->t_next -= 1.0;
        while (d->t_next <= 0.0) {
            float frac = (float)(1.0 + d->t_next);
            float complex y = d->prev_filt + frac * (f - d->prev_filt);
            double half = (d->sps + d->rate_adj) / 2.0;
            if (d->mid_strobe) {
                d->mid = y;
                d->mid_strobe = 0;
                d->t_next += half;
            } else {
                d->mid_strobe = 1;
                d->t_next += half;
                symbol_out(d, y);
            }
        }
        d->prev_filt = f;
    }

    if (d->num_soft_bits >= SOFT_BIT_BUF / 2)
        flush_bits(d);
}

void bpsk_demod_destroy(bpsk_demod_t *d)
{
    if (!d) return;
    flush_bits(d);
    free(d->taps);
    free(d->hist);
    free(d->acq_buf);
    free(d->acq_fft);
    free(d->acq_pwr);
    free(d);
}
