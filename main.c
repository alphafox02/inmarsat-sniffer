/*
 * inmarsat-sniffer: Inmarsat L-band decoder
 * Decodes STD-C (EGC) and Aero (ACARS) from a single SDR
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE
#include <err.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <complex.h>
#include <unistd.h>

#ifdef HAVE_SOAPYSDR
#include "soapysdr.h"
#endif

#ifdef HAVE_SDRPLAY
#include "sdrplay.h"
#endif

#ifdef HAVE_ZMQ
#include "zmq_audio.h"
int zmq_enabled = 0;
int zmq_base_port = 6001;
#endif

#include "jaero_dsp/jaero_demod.h"
#define MAX_JAERO_DEMODS 32
#define AUDIO_CENTER_HZ 8000.0   /* OQPSK audio carrier (JAERO default) */
#define PMSK_AUDIO_HZ   1000.0   /* P-channel MSK audio carrier (JAERO default) */
#define AUDIO_GAIN 5.0

/* Per-channel ring buffer for parallel demodulation. Each channel gets
 * its own worker thread that pulls IQ samples from this ring and feeds
 * its demod. Channelizer callback pushes without blocking (drops on full). */
#define CHAN_RING_SIZE (1 << 18)  /* 262144 complex samples (~5.4s @ 48kHz) */

typedef struct {
    jaero_pmsk_demod_t       *pmsk;      /* continuous MSK demod for P-channel 600/1200 */
    jaero_oqpsk_demod_t      *oqpsk;     /* burst OQPSK for 8400 baud */
    jaero_oqpsk_cont_demod_t *oqpsk_cont; /* continuous OQPSK for 10500 baud forward link */
    int channel_id;
    int baud_rate;
    int channel_type;          /* CHAN_AERO_* */
    double sample_rate;
    double mixer_phase;
    double mixer_inc;

    /* Lockless-ish ring buffer (single producer, single consumer) */
    float complex *ring;       /* size CHAN_RING_SIZE */
    atomic_uint ring_head;     /* producer writes here */
    atomic_uint ring_tail;     /* consumer reads here */
    atomic_ulong drops;

    pthread_t thread;
    atomic_int thread_run;

    /* Per-channel stats for web dashboard */
    atomic_ulong msg_count;
    atomic_ulong burst_count;
    double last_msg_time;

} jaero_chan_t;

static jaero_chan_t jaero_chans[MAX_JAERO_DEMODS];
static int num_jaero_chans = 0;

/* Export channel stats for web dashboard. Called from web.c build_json. */
typedef struct {
    int channel_id;
    int baud_rate;
    unsigned long msg_count;
    unsigned long burst_count;
    double last_msg_time;
    unsigned long drops;
    double mse;    /* signal quality: lower = better, 0-1 range */
    double ebno;   /* Eb/No in dB: higher = better signal */
    int is_locked; /* demod sigstat — true = frame sync, independent of msgs */
} chan_web_info_t;

void web_get_channel_info(chan_web_info_t *out, int *n) {
    int count = num_jaero_chans;
    if (count > 32) count = 32;
    for (int i = 0; i < count; i++) {
        out[i].channel_id = jaero_chans[i].channel_id;
        out[i].baud_rate = jaero_chans[i].baud_rate;
        out[i].msg_count = atomic_load(&jaero_chans[i].msg_count);
        out[i].burst_count = atomic_load(&jaero_chans[i].burst_count);
        out[i].last_msg_time = jaero_chans[i].last_msg_time;
        out[i].drops = atomic_load(&jaero_chans[i].drops);
        /* Read MSE and Eb/No from whichever demod is active */
        double mse = 1.0, ebno = 0;
        int locked = 0;
        if (jaero_chans[i].pmsk) {
            mse = jaero_pmsk_get_mse(jaero_chans[i].pmsk);
            ebno = jaero_pmsk_get_ebno(jaero_chans[i].pmsk);
            locked = jaero_pmsk_is_locked(jaero_chans[i].pmsk);
        } else if (jaero_chans[i].oqpsk_cont) {
            mse = jaero_oqpsk_cont_get_mse(jaero_chans[i].oqpsk_cont);
            ebno = jaero_oqpsk_cont_get_ebno(jaero_chans[i].oqpsk_cont);
            locked = jaero_oqpsk_cont_is_locked(jaero_chans[i].oqpsk_cont);
        }
        out[i].mse = mse;
        out[i].ebno = ebno;
        out[i].is_locked = locked;
    }
    *n = count;
}

/* ---- Web spectrum / tune accessors ----
 * Used by /api/spectrum and /api/tune. Look up channel by id, call the
 * matching demod's get_spectrum / set_manual_tune / get_tune_info.
 * Returns 0 on success, -1 if channel not found or no active demod. */
int web_get_spectrum_by_channel(int channel_id, float *mags_db, int n_bins,
                                 double *mixer_hz, double *freq_center_hz,
                                 double *fs_hz, double *lockingbw_hz,
                                 int *baud_out, int *afc_on_out)
{
    for (int i = 0; i < num_jaero_chans; i++) {
        if (jaero_chans[i].channel_id != channel_id) continue;
        if (baud_out) *baud_out = jaero_chans[i].baud_rate;
        if (jaero_chans[i].pmsk) {
            jaero_pmsk_get_tune_info(jaero_chans[i].pmsk,
                                      mixer_hz, freq_center_hz, fs_hz);
            if (lockingbw_hz) *lockingbw_hz =
                jaero_pmsk_get_lockingbw(jaero_chans[i].pmsk);
            if (afc_on_out)   *afc_on_out =
                jaero_pmsk_is_afc(jaero_chans[i].pmsk);
            return jaero_pmsk_get_spectrum(jaero_chans[i].pmsk,
                                            mags_db, n_bins) ? 0 : -1;
        }
        if (jaero_chans[i].oqpsk_cont) {
            jaero_oqpsk_cont_get_tune_info(jaero_chans[i].oqpsk_cont,
                                            mixer_hz, freq_center_hz, fs_hz);
            if (lockingbw_hz) *lockingbw_hz =
                jaero_oqpsk_cont_get_lockingbw(jaero_chans[i].oqpsk_cont);
            if (afc_on_out)   *afc_on_out =
                jaero_oqpsk_cont_is_afc(jaero_chans[i].oqpsk_cont);
            return jaero_oqpsk_cont_get_spectrum(jaero_chans[i].oqpsk_cont,
                                                  mags_db, n_bins) ? 0 : -1;
        }
        return -1;  /* no spectrum-capable demod on this channel */
    }
    return -1;
}

int web_get_constellation_by_channel(int channel_id, double *iq_out, int max_pairs)
{
    for (int i = 0; i < num_jaero_chans; i++) {
        if (jaero_chans[i].channel_id != channel_id) continue;
        if (jaero_chans[i].pmsk)
            return jaero_pmsk_get_constellation(jaero_chans[i].pmsk, iq_out, max_pairs);
        if (jaero_chans[i].oqpsk_cont)
            return jaero_oqpsk_cont_get_constellation(jaero_chans[i].oqpsk_cont, iq_out, max_pairs);
        return 0;
    }
    return 0;
}

/* audio_hz < 0 means 'no tune change'; afc_action: 0=leave, 1=enable, -1=disable */
int web_set_tune_by_channel(int channel_id, double audio_hz, int afc_action)
{
    for (int i = 0; i < num_jaero_chans; i++) {
        if (jaero_chans[i].channel_id != channel_id) continue;
        if (jaero_chans[i].pmsk) {
            if (audio_hz >= 0)
                jaero_pmsk_set_manual_tune(jaero_chans[i].pmsk, audio_hz);
            if (afc_action > 0) jaero_pmsk_set_afc(jaero_chans[i].pmsk, 1);
            if (afc_action < 0) jaero_pmsk_set_afc(jaero_chans[i].pmsk, 0);
            return 0;
        }
        if (jaero_chans[i].oqpsk_cont) {
            if (audio_hz >= 0)
                jaero_oqpsk_cont_set_manual_tune(jaero_chans[i].oqpsk_cont, audio_hz);
            if (afc_action > 0) jaero_oqpsk_cont_set_afc(jaero_chans[i].oqpsk_cont, 1);
            if (afc_action < 0) jaero_oqpsk_cont_set_afc(jaero_chans[i].oqpsk_cont, 0);
            return 0;
        }
        return -1;
    }
    return -1;
}

/* Per-channel worker: pop IQ from ring, feed matching demod. Runs until
 * thread_run is cleared. */
static void *chan_worker_fn(void *arg)
{
    jaero_chan_t *jc = (jaero_chan_t *)arg;
    const int BATCH = 4096;
    float complex batch[BATCH];
    double iq_dbl[BATCH * 2];

    while (atomic_load(&jc->thread_run)) {
        unsigned head = atomic_load(&jc->ring_head);
        unsigned tail = atomic_load(&jc->ring_tail);
        unsigned avail = head - tail;
        if (avail == 0) {
            /* No samples — short sleep then retry */
            struct timespec ts = {0, 500 * 1000};  /* 500 us */
            nanosleep(&ts, NULL);
            continue;
        }
        unsigned take = avail;
        if (take > BATCH) take = BATCH;
        /* Copy out (handle wrap) */
        unsigned tail_mod = tail & (CHAN_RING_SIZE - 1);
        unsigned first = CHAN_RING_SIZE - tail_mod;
        if (first > take) first = take;
        memcpy(batch, &jc->ring[tail_mod], first * sizeof(float complex));
        if (take > first)
            memcpy(&batch[first], &jc->ring[0], (take - first) * sizeof(float complex));
        atomic_store(&jc->ring_tail, tail + take);

        /* Feed demod. PMSK: direct complex IQ (channelizer already gave us
         * analytic baseband). OQPSK: mix IQ → real audio at AUDIO_CENTER_HZ
         * and feed JAERO's audio path (unchanged from known-ok config). */
        if (jc->pmsk) {
            for (unsigned i = 0; i < take; i++) {
                iq_dbl[i*2]   = crealf(batch[i]);
                iq_dbl[i*2+1] = cimagf(batch[i]);
            }
            jaero_pmsk_feed_iq(jc->pmsk, iq_dbl, take);
        } else if (jc->oqpsk_cont) {
            /* Continuous OQPSK: feedIQ wrapper does IQ→audio internally. */
            for (unsigned i = 0; i < take; i++) {
                iq_dbl[i*2]   = crealf(batch[i]);
                iq_dbl[i*2+1] = cimagf(batch[i]);
            }
            jaero_oqpsk_cont_feed_iq(jc->oqpsk_cont, iq_dbl, take);
        } else if (jc->oqpsk) {
            /* Burst OQPSK (8400 baud): same audio conversion path. */
            int16_t pcm[BATCH];
            for (unsigned i = 0; i < take; i++) {
                double ca = cos(jc->mixer_phase);
                double sa = sin(jc->mixer_phase);
                double audio = creal((double complex)batch[i] * (ca + sa * I));
                jc->mixer_phase += jc->mixer_inc;
                if (jc->mixer_phase > 2.0 * M_PI) jc->mixer_phase -= 2.0 * M_PI;
                double scaled = audio * AUDIO_GAIN * 32768.0;
                if (scaled > 32767.0) scaled = 32767.0;
                if (scaled < -32768.0) scaled = -32768.0;
                pcm[i] = (int16_t)scaled;
            }
            jaero_oqpsk_feed(jc->oqpsk, pcm, take);
        }
    }
    return NULL;
}

/* Push samples into channel's ring (non-blocking). Drops on full. */
static void chan_push(jaero_chan_t *jc, const float complex *samples, int n)
{
    unsigned head = atomic_load(&jc->ring_head);
    unsigned tail = atomic_load(&jc->ring_tail);
    unsigned free = CHAN_RING_SIZE - (head - tail);
    if ((unsigned)n > free) {
        atomic_fetch_add(&jc->drops, 1);
        n = (int)free;  /* write what fits, drop rest */
        if (n <= 0) return;
    }
    unsigned head_mod = head & (CHAN_RING_SIZE - 1);
    unsigned first = CHAN_RING_SIZE - head_mod;
    if ((unsigned)first > (unsigned)n) first = (unsigned)n;
    memcpy(&jc->ring[head_mod], samples, first * sizeof(float complex));
    if ((unsigned)n > first)
        memcpy(&jc->ring[0], &samples[first], ((unsigned)n - first) * sizeof(float complex));
    atomic_store(&jc->ring_head, head + (unsigned)n);
}

static void chan_init_thread(jaero_chan_t *jc)
{
    jc->ring = (float complex *)malloc(CHAN_RING_SIZE * sizeof(float complex));
    if (!jc->ring) {
        fprintf(stderr, "chan_init_thread: OOM allocating ring for ch%d\n",
                jc->channel_id);
        return;
    }
    atomic_init(&jc->ring_head, 0);
    atomic_init(&jc->ring_tail, 0);
    atomic_init(&jc->drops, 0);
    atomic_init(&jc->thread_run, 1);
    pthread_create(&jc->thread, NULL, chan_worker_fn, jc);
}

#ifdef HAVE_RTLSDR
#include "rtlsdr.h"
#endif

#ifdef HAVE_HACKRF
#include "hackrf.h"
#endif

#ifdef HAVE_BLADERF
#include "bladerf.h"
#endif

#ifdef HAVE_UHD
#include "usrp.h"
#endif

#ifdef HAVE_AIRSPY
#include "airspy.h"
#endif

#include "sdr.h"
#include "inmarsat.h"
#include "satellites.h"
#include "options.h"
#include "channelizer.h"
#include "demod_dbpsk.h"
#include "stdc_decode.h"
#include "aero_decode.h"

#ifdef HAVE_LIBACARS
#include <libacars/libacars.h>
#include <libacars/acars.h>
#include <libacars/adsc.h>
#include <libacars/arinc.h>
#include <libacars/cpdlc.h>
#include <libacars/list.h>
#include <libacars/reassembly.h>
#include <libacars/vstring.h>
#include <libacars/json.h>
#include <libacars/version.h>
static la_reasm_ctx *acars_reasm_ctx = NULL;

/* Walk an ADS-C msg's tag list for basic-report (7/9/10/18/19/20) with a
 * plausible lat/lon, plus optional earth/air reference (tags 14/15) for
 * heading and groundspeed. Called on the whole proto tree via
 * la_proto_tree_find_adsc. */
static int extract_adsc_position(la_adsc_msg_t *adsc,
                                   double *lat, double *lon, int *alt_ft,
                                   double *gs_kts, double *track_deg) {
    if (!adsc || adsc->err) return 0;
    int found_pos = 0;
    *gs_kts = -1;
    *track_deg = -1;
    for (la_list *p = adsc->tag_list; p; p = p->next) {
        la_adsc_tag_t *t = (la_adsc_tag_t *)p->data;
        if (!t || !t->data) continue;
        if (!found_pos && (t->tag == 7 || t->tag == 9 || t->tag == 10 ||
                           t->tag == 18 || t->tag == 19 || t->tag == 20)) {
            la_adsc_basic_report_t *r = (la_adsc_basic_report_t *)t->data;
            if (r->lat >= -90 && r->lat <= 90 &&
                r->lon >= -180 && r->lon <= 180 &&
                !(r->lat == 0 && r->lon == 0)) {
                *lat = r->lat;
                *lon = r->lon;
                *alt_ft = r->alt;
                found_pos = 1;
            }
        }
        /* Earth/air reference: heading + groundspeed (knots). */
        if (t->tag == 14 || t->tag == 15) {
            la_adsc_earth_air_ref_t *r = (la_adsc_earth_air_ref_t *)t->data;
            if (!r->heading_invalid) {
                *track_deg = r->heading;
                *gs_kts = r->speed;
            }
        }
    }
    return found_pos;
}
#endif
#include "vita49.h"
#include "feed.h"
#include "web.h"
#include "basestation.h"
#include "aircraft_db.h"
#include "acars_position.h"
#include "waypoint_db.h"
#include "learned_waypoints.h"
#include "simd_kernels.h"

#define C_FEK_BLOCKING_QUEUE_IMPLEMENTATION
#define C_FEK_FAIR_LOCK_IMPLEMENTATION
#include "blocking_queue.h"

/* ---- Global configuration ---- */
double samp_rate = 0;         /* 0 = auto from satellite table */
double center_freq = 0;
double ppm_correction = 0;    /* user-supplied SDR PPM error; 0 = use auto-cal */
int verbose = 0;
int live = 0;
iq_format_t iq_format = FMT_CI8;
op_mode_t op_mode = MODE_AERO;   /* Aero is the verified-working path; STD-C
                                  * and full require explicit opt-in until
                                  * STD-C decode is confirmed on air. */
int skip_c_channel = 0;       /* --skip-c-channel: don't decode OQPSK 8400 C-channel */
double oqpsk_lockingbw = 0;   /* --oqpsk-lockingbw=HZ: override default 10500 AFC range */
char *satellite_name = NULL;

/* SDR selection */
#ifdef HAVE_SOAPYSDR
int soapy_num = -1;
char *soapy_args = NULL;
#define SOAPY_SETTINGS_MAX 8
char *soapy_setting_keys[SOAPY_SETTINGS_MAX];
char *soapy_setting_vals[SOAPY_SETTINGS_MAX];
int soapy_setting_count = 0;
#define SOAPY_GAINS_MAX 8
char *soapy_gain_elem_names[SOAPY_GAINS_MAX];
double soapy_gain_elem_vals[SOAPY_GAINS_MAX];
int soapy_gain_elem_count = 0;
#endif

double soapy_gain_val = 40.0;
int    soapy_gain_explicit = 0;  /* 1 once --soapy-gain is parsed */
int    airspy_gain_val = -1;     /* native linearity_gain 0..21; <0 = default */
int bias_tee = 0;

#ifdef HAVE_SDRPLAY
char *sdrplay_serial = NULL;
int sdrplay_gain_val = -1;  /* -1 = AGC enabled */
#endif
#ifdef HAVE_RTLSDR
int rtl_dev_index = -1;
#endif
#ifdef HAVE_AIRSPY
uint64_t airspy_serial = 0;  /* 0 = first device */
int airspy_selected = 0;     /* set when -i airspy[-SN] is passed */
#endif
#ifdef HAVE_HACKRF
char *hackrf_serial = NULL;
int hackrf_lna_gain = 40;  /* max LNA for weak L-band */
int hackrf_vga_gain = 40;  /* higher VGA for satellite signals (tested working) */
int hackrf_amp_enable = 0; /* off — enable with --hackrf-amp if no external LNA */
#endif
#ifdef HAVE_BLADERF
int bladerf_num = -1;
int bladerf_gain_val = 40;
#endif
#ifdef HAVE_UHD
char *usrp_serial = NULL;
int usrp_gain_val = 40;
#endif
int vita49_enabled = 0;
char *vita49_endpoint = NULL;
int web_enabled = 0;
int web_port = 8888;
int spectrum_enabled = 0;
int feed_enabled = 0;
int jaero_format_enabled = 0;
char *jaero_format_host = NULL;
int jaero_format_port = 0;
int agc_enabled = 0;
#define UDP_MAX 4
char *udp_hosts[UDP_MAX];
int udp_ports[UDP_MAX];
int udp_count = 0;
int mqtt_enabled = 0;
char *mqtt_host = NULL;
int mqtt_port = 1883;
char *mqtt_user = NULL;
char *mqtt_pass = NULL;
char *mqtt_topic = NULL;
int basestation_enabled = 0;
char *basestation_endpoint = NULL;
char *aircraft_db_path = NULL;
int update_db_flag = 0;
char *station_id = NULL;

/* Input file */
FILE *in_file = NULL;

/* Threading state */
volatile sig_atomic_t running = 1;
pid_t self_pid;

/* Queues */
#define SAMPLES_QUEUE_SIZE 2048
#define DECODED_QUEUE_SIZE 256
Blocking_Queue samples_queue;
Blocking_Queue decoded_queue;

/* Atomic stats counters */
atomic_ulong stat_samples_total = 0;
atomic_ulong stat_stdc_frames = 0;
atomic_ulong stat_drops = 0;
atomic_ulong stat_stdc_crc_ok = 0;
atomic_ulong stat_stdc_crc_fail = 0;
atomic_ulong stat_aero_crc_ok = 0;
atomic_ulong stat_aero_bursts = 0;
atomic_ulong stat_aero_msgs = 0;
atomic_ulong stat_pos_adsc = 0;
atomic_ulong stat_pos_text = 0;
atomic_ulong stat_pos_waypoint = 0;
atomic_ulong stat_stdc_ber_sum = 0;   /* fixed-point * 10000 */
atomic_ulong stat_stdc_ber_count = 0;
atomic_int stat_stdc_synced = 0;

/* ---- Sample buffer management ---- */

void push_samples(sample_buf_t *buf) {
    atomic_fetch_add(&stat_samples_total, buf->num);
    if (blocking_queue_add(&samples_queue, buf) == BQ_FULL) {
        atomic_fetch_add(&stat_drops, 1);
        free(buf);
    }
}

/* ---- Signal handler ---- */

static void sig_handler(int sig) {
    (void)sig;
    running = 0;
}

/* ---- File spewer thread ---- */

static void *spewer_thread(void *arg) {
    FILE *f = (FILE *)arg;
    size_t block = 32768;

    while (running) {
        sample_buf_t *s = NULL;
        size_t r = 0;

        switch (iq_format) {
        case FMT_CI8:
            s = malloc(sizeof(*s) + block * 2);
            if (!s) break;
            s->format = SAMPLE_FMT_INT8;
            r = fread(s->samples, 2, block, f);
            break;

        case FMT_CU8: {
            s = malloc(sizeof(*s) + block * 2);
            if (!s) break;
            s->format = SAMPLE_FMT_INT8;
            uint8_t *tmp = malloc(block * 2);
            if (!tmp) { free(s); s = NULL; break; }
            r = fread(tmp, 2, block, f);
            for (size_t i = 0; i < r * 2; i++)
                s->samples[i] = (int8_t)(tmp[i] - 128);
            free(tmp);
            break;
        }

        case FMT_CI16: {
            s = malloc(sizeof(*s) + block * 2);
            if (!s) break;
            s->format = SAMPLE_FMT_INT8;
            int16_t *tmp = malloc(block * 4);
            if (!tmp) { free(s); s = NULL; break; }
            r = fread(tmp, 4, block, f);
            for (size_t i = 0; i < r * 2; i++)
                s->samples[i] = (int8_t)(tmp[i] >> 8);
            free(tmp);
            break;
        }

        case FMT_CF32: {
            s = malloc(sizeof(*s) + block * 8);
            if (!s) break;
            s->format = SAMPLE_FMT_FLOAT;
            r = fread(s->samples, 8, block, f);
            break;
        }

        default:
            break;
        }

        if (!s) break;         /* OOM — bail out */
        if (r == 0) {
            free(s);
            break;
        }
        s->num = r;
        s->hw_timestamp_ns = 0;
        if (blocking_queue_put(&samples_queue, s) != 0) {
            free(s);
            break;
        }
    }

    /* Wait for queue to drain */
    while (running && samples_queue.queue_size > 0)
        usleep(10000);

    running = 0;
    kill(self_pid, SIGINT);
    return NULL;
}

/* ---- Status line ---- */

static void print_status(void) {
    unsigned long stdc = atomic_load(&stat_stdc_frames);
    unsigned long drops = atomic_load(&stat_drops);
    unsigned long feed_drops = feed_get_json_drops();
    unsigned long sc_ok = atomic_load(&stat_stdc_crc_ok);
    unsigned long sc_fail = atomic_load(&stat_stdc_crc_fail);
    unsigned long sb_sum = atomic_load(&stat_stdc_ber_sum);
    unsigned long sb_cnt = atomic_load(&stat_stdc_ber_count);
    int synced = atomic_load(&stat_stdc_synced);
    unsigned long bursts = atomic_load(&stat_aero_bursts);
    unsigned long msgs = atomic_load(&stat_aero_msgs);
    unsigned long ac_ok = atomic_load(&stat_aero_crc_ok);

    float stdc_ber = sb_cnt ? (float)sb_sum / (sb_cnt * 10000.0f) : 0;

    /* Format large counters with K/M suffixes */
    char burst_str[16];
    if (bursts >= 1000000)
        snprintf(burst_str, sizeof(burst_str), "%.1fM", bursts / 1e6);
    else if (bursts >= 1000)
        snprintf(burst_str, sizeof(burst_str), "%.1fK", bursts / 1e3);
    else
        snprintf(burst_str, sizeof(burst_str), "%lu", bursts);

    char fd_buf[24] = "";
    if (feed_drops > 0)
        snprintf(fd_buf, sizeof(fd_buf), " feed_drop:%lu", feed_drops);

    if (op_mode == MODE_AERO) {
        fprintf(stderr, "\r[Aero: %s bursts %lu msgs CRC:%lu | drop:%lu%s]   ",
                burst_str, msgs, ac_ok, drops, fd_buf);
    } else if (op_mode == MODE_STDC) {
        fprintf(stderr, "\r[STD-C: %lu %s BER:%.2f CRC:%lu/%lu | drop:%lu%s]   ",
                stdc, synced ? "SYNC" : "SRCH",
                stdc_ber, sc_ok, sc_ok + sc_fail, drops, fd_buf);
    } else {
        fprintf(stderr, "\r[STD-C: %lu %s BER:%.2f CRC:%lu/%lu | "
                "Aero: %s bursts %lu msgs CRC:%lu | drop:%lu%s]   ",
                stdc, synced ? "SYNC" : "SRCH",
                stdc_ber, sc_ok, sc_ok + sc_fail,
                burst_str, msgs, ac_ok, drops, fd_buf);
    }
}

/* ---- STD-C demod/decode chain ---- */

static dbpsk_demod_t *stdc_demod = NULL;
static stdc_decoder_t *stdc_decoder = NULL;
static channelizer_t *channelizer = NULL;
static int next_dynamic_channel_id = 100;

static const char *get_stdc_type_str(stdc_msg_type_t type) {
    switch (type) {
    case STDC_MSG_EGC_SINGLE:
    case STDC_MSG_EGC_DOUBLE_1:
    case STDC_MSG_EGC_DOUBLE_2:     return "EGC";
    case STDC_MSG_BULLETIN:          return "Bulletin";
    case STDC_MSG_ANNOUNCEMENT:      return "Announcement";
    case STDC_MSG_CHANNEL_CLEAR:     return "Chan Clear";
    case STDC_MSG_ACK_REQUEST:       return "Ack Req";
    case STDC_MSG_MSG_ACK:           return "Msg Ack";
    case STDC_MSG_CHAN_ASSIGNMENT:    return "Chan Assign";
    case STDC_MSG_LOGIN_ACK:         return "Login Ack";
    case STDC_MSG_MESSAGE_DATA:      return "Message";
    case STDC_MSG_NET_UPDATE:        return "Net Update";
    case STDC_MSG_LES_LIST:          return "LES List";
    case STDC_MSG_INDIVIDUAL_POLL:   return "Poll";
    case STDC_MSG_CONFIRMATION:      return "Confirm";
    default:                         return "STD-C";
    }
}

/* Try to add a dynamically discovered downlink channel */
static void try_add_dynamic_channel(double freq_mhz) {
    if (!channelizer || op_mode == MODE_STDC)
        return;

    double freq_hz = freq_mhz * 1e6;

    /* Sanity check -- must be in L-band downlink range */
    if (freq_hz < INMARSAT_L_BAND_LOW || freq_hz > INMARSAT_L_BAND_HIGH)
        return;

    /* Must be within the active SDR bandwidth */
    double offset = fabs(freq_hz - center_freq);
    if (offset > samp_rate / 2.0)
        return;

    /* Already have this frequency? */
    if (channelizer_has_freq(channelizer, freq_hz, 5000.0))
        return;

    /* Guess channel type from frequency region.
     * Aero 600/1200 are typically 1545.0-1545.5 MHz,
     * Aero 10500 around 1546.0 MHz, Aero 8400 around 1546.1+ MHz.
     * This is a rough heuristic -- the actual baud rate is determined
     * by the demodulator locking to the signal. */
    channel_type_t type = CHAN_AERO_1200;  /* safe default */
    if (freq_hz >= 1546050000.0)
        type = CHAN_AERO_8400;
    else if (freq_hz >= 1545800000.0)
        type = CHAN_AERO_10500;
    else if (freq_hz < 1545150000.0)
        type = CHAN_AERO_600;

    int ch_id = next_dynamic_channel_id++;
    if (channelizer_add_channel(channelizer, freq_hz, type, ch_id) == 0) {
        fprintf(stderr, "\n[Dynamic] Added channel %d at %.4f MHz\n",
                ch_id, freq_mhz);
    }
}

static void stdc_message_cb(const stdc_message_t *msg, void *user) {
    (void)user;

    const char *type_str = get_stdc_type_str(msg->type);
    int is_verbose_only = 0;

    /* Signalling/control packets only shown in verbose mode */
    switch (msg->type) {
    case STDC_MSG_CHANNEL_CLEAR:
    case STDC_MSG_ACK_REQUEST:
    case STDC_MSG_MSG_ACK:
    case STDC_MSG_CHAN_ASSIGNMENT:
    case STDC_MSG_LOGIN_ACK:
    case STDC_MSG_NET_UPDATE:
    case STDC_MSG_LES_LIST:
        is_verbose_only = 1;
        break;
    default:
        break;
    }

    /* Dynamic channel discovery from on-air frequency announcements */
    if (msg->downlink_mhz > 0)
        try_add_dynamic_channel(msg->downlink_mhz);

    if (!is_verbose_only || verbose)
        fprintf(stderr, "\n[%s] %s\n", type_str, msg->text);

    feed_stdc_message(msg);
    if (web_enabled)
        web_add_stdc(msg);
}

static void stdc_bits_cb(const float *soft_bits, int num_bits, void *user) {
    (void)user;
    if (stdc_decoder)
        stdc_decoder_feed(stdc_decoder, soft_bits, num_bits);
}

/* ---- Aero decode chain ----
 * AeroL owns Viterbi → descramble → RS → ISU → ACARS validation.
 * Validated ACARS userdata arrives in jaero_acars_data_cb below. */

/* C-channel assignment event: type bytes 0x31 distress, 0x32 safety,
 * 0x33 other safety, 0x34 non-safety. */
static void on_cassign(int channel_id, uint8_t type,
                        uint32_t aes_id, uint8_t ges_id,
                        double rx_mhz, double tx_mhz, void *user) {
    (void)user;
    const char *type_str;
    switch (type) {
    case 0x31: type_str = "DISTRESS"; break;
    case 0x32: type_str = "SAFETY";   break;
    case 0x33: type_str = "OTHER_SAFETY"; break;
    case 0x34: type_str = "NON_SAFETY"; break;
    default:   type_str = "UNKNOWN"; break;
    }
    fprintf(stderr, "\n[C-ASSIGN ch%d AES:%06X GES:%02X] %s  RX=%.4f MHz  TX=%.4f MHz\n",
            channel_id, aes_id, ges_id, type_str, rx_mhz, tx_mhz);
}

static void jaero_acars_data_cb(const uint8_t *data, int len,
                                  int channel_id,
                                  uint32_t aes_id, uint8_t ges_id,
                                  uint8_t qno, uint8_t refno, int downlink,
                                  void *user) {
    (void)user;
    atomic_fetch_add(&stat_aero_msgs, 1);
    /* AeroL passes only valid frames — count every event as CRC-OK. */
    atomic_fetch_add(&stat_aero_crc_ok, 1);

    /* Per-channel stats */
    for (int i = 0; i < num_jaero_chans; i++) {
        if (jaero_chans[i].channel_id == channel_id) {
            atomic_fetch_add(&jaero_chans[i].msg_count, 1);
            jaero_chans[i].last_msg_time = (double)time(NULL);
            break;
        }
    }

    /* Verbose-only raw dump: hex + 7-bit ASCII (ACARS parity-stripped). */
    if (verbose) {
        fprintf(stderr, "\n[JAERO-DECODED ch%d] %d bytes\n  hex: ", channel_id, len);
        for (int i = 0; i < len && i < 120; i++)
            fprintf(stderr, "%02X ", data[i]);
        if (len > 120) fprintf(stderr, "...");
        fprintf(stderr, "\n  txt: ");
        for (int i = 0; i < len && i < 120; i++) {
            uint8_t c = data[i] & 0x7F;
            fputc((c >= 0x20 && c < 0x7F) ? c : '.', stderr);
        }
        fprintf(stderr, "\n");
    }

#ifdef HAVE_LIBACARS
    /* ISU userdata framing: FF FF SOH <mode+hdr> STX <text> ETX <crc2> DEL.
     * libacars expects the bytes after SOH (0x01) — scan forward, skip. */
    int soh_idx = -1;
    for (int i = 0; i < len - 12 && i < 6; i++) {
        if ((data[i] & 0x7F) == 0x01) { soh_idx = i; break; }
    }
    if (soh_idx >= 0 && len - soh_idx > 13) {
        const uint8_t *acars_start = data + soh_idx + 1;
        int acars_len = len - soh_idx - 1;
        struct timeval tv;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        tv.tv_sec = ts.tv_sec;
        tv.tv_usec = ts.tv_nsec / 1000;

        la_proto_node *tree = la_acars_parse_and_reassemble(
            acars_start, acars_len, LA_MSG_DIR_AIR2GND, acars_reasm_ctx, tv);

        if (tree) {
            la_proto_node *acars_node = la_proto_tree_find_acars(tree);
            if (acars_node && acars_node->data) {
                la_acars_msg *amsg = (la_acars_msg *)acars_node->data;
                if (amsg->reasm_status != LA_REASM_IN_PROGRESS && !amsg->err) {
                    fprintf(stderr, "\n[ACARS ch%d AES:%06X GES:%02X] reg=%s label=%.2s blk=%c",
                            channel_id, aes_id, ges_id,
                            amsg->reg[0] ? amsg->reg : "?",
                            amsg->label, amsg->block_id ? amsg->block_id : '?');

                    /* Aircraft DB lookup by AES ID — shows description + operator */
                    aircraft_info_t info;
                    if (aircraft_db_lookup_by_aes(aes_id, &info)) {
                        const char *desc = info.description ? info.description : info.type;
                        if (desc && *desc) fprintf(stderr, "  %s", desc);
                        if (info.operator_ && *info.operator_)
                            fprintf(stderr, " / %s", info.operator_);
                    }

                    if (amsg->txt && amsg->txt[0])
                        fprintf(stderr, "\n  %s", amsg->txt);
                    fprintf(stderr, "\n");

                    if (verbose) {
                        la_vstring *vstr = la_proto_tree_format_text(NULL, tree);
                        if (vstr && vstr->str)
                            fprintf(stderr, "%s", vstr->str);
                        if (vstr) la_vstring_destroy(vstr, true);
                    }

                    /* Populate aero_message_t and hand to feed/web outputs. */
                    aero_message_t outmsg;
                    memset(&outmsg, 0, sizeof(outmsg));
                    outmsg.channel_id = channel_id;
                    outmsg.aes_id = aes_id;
                    outmsg.ges_id = ges_id;
                    outmsg.qno = qno;
                    outmsg.refno = refno;
                    outmsg.downlink = downlink;
                    outmsg.lat = NAN;
                    outmsg.lon = NAN;
                    outmsg.alt_ft = -1;
                    outmsg.has_position = 0;
                    outmsg.mode = amsg->mode;
                    outmsg.block_id = amsg->block_id;
                    outmsg.ack = amsg->ack;
                    strncpy(outmsg.reg, amsg->reg, sizeof(outmsg.reg) - 1);
                    strncpy(outmsg.flight, amsg->flight_id, sizeof(outmsg.flight) - 1);
                    strncpy(outmsg.label, amsg->label, sizeof(outmsg.label) - 1);
                    if (amsg->txt) {
                        int tl = (int)strlen(amsg->txt);
                        if (tl > (int)sizeof(outmsg.text) - 1)
                            tl = sizeof(outmsg.text) - 1;
                        memcpy(outmsg.text, amsg->txt, tl);
                        outmsg.text_len = tl;
                    }

                    /* Serialise the libacars proto tree as JSON so feed.c
                     * can emit it in the JSONdump "arinc622" sub-object.
                     * Only include it if there's something more than the
                     * plain ACARS wrapper (ARINC-622 application payload,
                     * ADS-C, CPDLC, etc). vstring lives on this stack,
                     * released after all consumers have run. */
                    la_vstring *arinc_vs = NULL;
                    if (la_proto_tree_find_protocol(tree, &la_DEF_arinc_message) ||
                        la_proto_tree_find_adsc(tree) ||
                        la_proto_tree_find_cpdlc(tree)) {
                        arinc_vs = la_vstring_new();
                        if (arinc_vs) {
                            la_proto_tree_format_json(arinc_vs, tree);
                            if (arinc_vs->str && arinc_vs->str[0])
                                outmsg.arinc622_json = arinc_vs->str;
                        }
                    }

                    /* Position extraction. Prefer structured ADS-C (ARINC
                     * 620 CR1 binary payload, tag-based lat/lon) over the
                     * text regex — the ADS-C path is lossless and unambiguous. */
                    double lat = 0, lon = 0;
                    double gs_kts = -1, track_deg = -1;
                    int alt_ft = -99999;
                    int have_pos = 0;
                    la_proto_node *adsc_node = la_proto_tree_find_adsc(tree);
                    if (adsc_node && adsc_node->data) {
                        la_adsc_msg_t *adsc = (la_adsc_msg_t *)adsc_node->data;
                        have_pos = extract_adsc_position(adsc, &lat, &lon,
                                                         &alt_ft,
                                                         &gs_kts, &track_deg);
                        if (have_pos) atomic_fetch_add(&stat_pos_adsc, 1);
                    }
                    if (!have_pos) {
                        have_pos = acars_extract_text_position(amsg->label,
                                                                amsg->txt,
                                                                &lat, &lon);
                        if (have_pos) atomic_fetch_add(&stat_pos_text, 1);
                    }
                    if (!have_pos) {
                        have_pos = acars_extract_waypoint_position(amsg->label,
                                                                    amsg->txt,
                                                                    &lat, &lon);
                        if (have_pos) atomic_fetch_add(&stat_pos_waypoint, 1);
                    }
                    /* If ADS-C didn't supply altitude, try pulling a flight
                     * level or raw feet out of the ACARS text (MDPOS etc.). */
                    if (alt_ft == -99999 && amsg->txt) {
                        int text_alt = 0;
                        if (acars_extract_text_altitude(amsg->txt, &text_alt))
                            alt_ft = text_alt;
                    }
                    if (have_pos) {
                        outmsg.lat = lat;
                        outmsg.lon = lon;
                        outmsg.alt_ft = alt_ft == -99999 ? -1 : alt_ft;
                        outmsg.has_position = 1;
                        fprintf(stderr, "  pos=%.4f,%.4f%s%d\n",
                                lat, lon,
                                alt_ft == -99999 ? "" : " alt=",
                                alt_ft == -99999 ? 0 : alt_ft);
                        if (basestation_enabled) {
                            struct timespec tsn;
                            clock_gettime(CLOCK_REALTIME, &tsn);
                            uint64_t ns = (uint64_t)tsn.tv_sec * 1000000000ULL
                                        + (uint64_t)tsn.tv_nsec;
                            basestation_send_position(amsg->reg,
                                                       amsg->flight_id,
                                                       lat, lon, alt_ft,
                                                       gs_kts, track_deg, ns);
                        }
                    }

                    /* Harvest waypoints from any Flight Plan (FPN) message.
                     * These carry WAYPOINT,COORD pairs that later position
                     * reports may reference by name only. */
                    if (amsg->txt && amsg->label[0] == 'H' &&
                        amsg->label[1] == '1' &&
                        strncmp(amsg->txt, "FPN", 3) == 0) {
                        learned_wp_parse_fpn(amsg->txt);
                    }

                    /* CPDLC: surface controller-pilot text messages too. */
                    la_proto_node *cpdlc_node = la_proto_tree_find_cpdlc(tree);
                    if (cpdlc_node) {
                        la_vstring *vstr = la_proto_tree_format_text(NULL, cpdlc_node);
                        if (vstr && vstr->str) {
                            fprintf(stderr, "[CPDLC ch%d %s]\n%s",
                                    channel_id, amsg->reg, vstr->str);
                        }
                        if (vstr) la_vstring_destroy(vstr, true);
                    }

                    feed_aero_message(&outmsg);
                    if (web_enabled)
                        web_add_aero(&outmsg);

                    if (arinc_vs) la_vstring_destroy(arinc_vs, true);
                }
            }
            la_proto_tree_destroy(tree);
        }
    }
#endif
}


/* Soft-bits callback. AeroL inside the wrapper owns full decode; this
 * just counts bursts for the status line. */
static void jaero_bits_cb(const unsigned char *bits, int num_bits,
                            int channel_id, void *user) {
    (void)user; (void)bits; (void)num_bits; (void)channel_id;
    atomic_fetch_add(&stat_aero_bursts, 1);
}

/* ---- Channel output callback ---- */

static void channel_output_cb(int channel_id, channel_type_t type,
                                float complex *samples, int num_samples,
                                void *user) {
    (void)user;

    /* Per-channel power measurement (verbose only) */
    if (verbose) {
        static double ch_pwr[32] = {0};
        static int ch_cnt[32] = {0};
        static int pwr_print = 0;
        int idx = -1;
        for (int i = 0; i < num_jaero_chans; i++) {
            if (jaero_chans[i].channel_id == channel_id) { idx = i; break; }
        }
        if (idx >= 0 && idx < 32) {
            double pwr = 0;
            for (int i = 0; i < num_samples; i++)
                pwr += crealf(samples[i]) * crealf(samples[i]) +
                       cimagf(samples[i]) * cimagf(samples[i]);
            ch_pwr[idx] += pwr;
            ch_cnt[idx] += num_samples;
        }
        if (++pwr_print >= 5000) {
            pwr_print = 0;
            fprintf(stderr, "\n[PWR]");
            for (int i = 0; i < num_jaero_chans && i < 32; i++) {
                if (ch_cnt[i] > 0) {
                    double rms = sqrt(ch_pwr[i] / ch_cnt[i]);
                    fprintf(stderr, " ch%d=%.4f", jaero_chans[i].channel_id, rms);
                    ch_pwr[i] = 0;
                    ch_cnt[i] = 0;
                }
            }
            fprintf(stderr, "\n");
        }
    }

#ifdef HAVE_ZMQ
    if (zmq_enabled) {
        double output_rate = channelizer_output_rate(channelizer, channel_id);
        /* Audio centers match JAERO/SDRReceiver defaults: 1 kHz MSK, 8 kHz OQPSK. */
        double audio_center = (type == CHAN_AERO_10500 ||
                                type == CHAN_AERO_8400) ? 8000.0 : 1000.0;
        if (output_rate > 0)
            zmq_audio_send(channel_id, samples, num_samples, output_rate, audio_center);
    }
#endif

    if (type == CHAN_STDC_EGC && stdc_demod) {
        dbpsk_demod_process(stdc_demod, samples, num_samples);
        return;
    }

    /* Startup-only PPM auto-cal from the first aero channel's carrier.
     * Skipped when --ppm is set — the user supplied the answer already. */
    if (ppm_correction == 0.0) {
        static float complex cal_buf[1024];
        static int cal_n = 0;
        static int cal_ch = -1;
        static int cal_done = 0;
        #define CAL_SIZE 1024

        if (!cal_done && cal_ch < 0 && (type == CHAN_AERO_600 || type == CHAN_AERO_1200)) {
            cal_ch = channel_id;
            cal_n = 0;
        }
        if (!cal_done && cal_ch >= 0 && channel_id == cal_ch) {
            int need = CAL_SIZE - cal_n;
            int take = num_samples < need ? num_samples : need;
            memcpy(&cal_buf[cal_n], samples, take * sizeof(float complex));
            cal_n += take;

            if (cal_n >= CAL_SIZE) {
                double max_pwr = 0;
                int max_bin = 0;
                for (int k = 0; k < CAL_SIZE; k++) {
                    double re = 0, im = 0;
                    for (int n = 0; n < CAL_SIZE; n++) {
                        double angle = -2.0 * M_PI * k * n / CAL_SIZE;
                        re += crealf(cal_buf[n]) * cos(angle) - cimagf(cal_buf[n]) * sin(angle);
                        im += crealf(cal_buf[n]) * sin(angle) + cimagf(cal_buf[n]) * cos(angle);
                    }
                    double pwr = re * re + im * im;
                    if (pwr > max_pwr) { max_pwr = pwr; max_bin = k; }
                }
                double output_rate = channelizer_output_rate(channelizer, cal_ch);
                int half = CAL_SIZE / 2;
                int offset_bin = max_bin > half ? max_bin - CAL_SIZE : max_bin;
                double offset_hz = offset_bin * output_rate / CAL_SIZE;

                if (fabs(offset_hz) > 50.0) {
                    fprintf(stderr, "Auto-cal: carrier offset %.0f Hz on ch%d, adjusting center freq\n",
                            offset_hz, cal_ch);
                    channelizer_adjust_center(channelizer, offset_hz);
                } else if (verbose) {
                    fprintf(stderr, "Auto-cal: centered (%.0f Hz), no adjustment\n", offset_hz);
                }
                cal_done = 1;
            }
        }
    }

    /* Aero MSK (600/1200 baud P-channel): JAERO MskDemodulator + AeroL, feedIQ path. */
    if (type == CHAN_AERO_600 || type == CHAN_AERO_1200) {

        int baud = (type == CHAN_AERO_1200) ? 1200 : 600;
        double output_rate = channelizer_output_rate(channelizer, channel_id);
        if (output_rate <= 0) return;

        jaero_chan_t *jc = NULL;
        for (int i = 0; i < num_jaero_chans; i++) {
            if (jaero_chans[i].channel_id == channel_id) {
                jc = &jaero_chans[i];
                break;
            }
        }
        if (!jc && num_jaero_chans < MAX_JAERO_DEMODS) {
            jc = &jaero_chans[num_jaero_chans++];
            jc->channel_id  = channel_id;
            jc->baud_rate   = baud;
            jc->pmsk        = NULL;
            jc->oqpsk       = NULL;
            jc->oqpsk_cont  = NULL;
            jc->mixer_phase = 0.0;
            /* Mix IQ to audio at 1000 Hz — the JAERO desktop default for
             * 600/1200 baud MSK. Matches SDRReceiver/ZMQ convention. */
            jc->mixer_inc = 2.0 * M_PI * PMSK_AUDIO_HZ / output_rate;

            jc->pmsk = jaero_pmsk_create(output_rate, (double)baud,
                                          channel_id, jaero_bits_cb, NULL);
            if (jc->pmsk) {
                jaero_pmsk_set_acars_callback(jc->pmsk,
                                               jaero_acars_data_cb, NULL);
                jaero_pmsk_set_cassign_callback(jc->pmsk,
                                                  on_cassign, NULL);
            }

            if (verbose)
                fprintf(stderr, "[PMSK ch%d] baud=%d rate=%.0f (continuous P-channel)\n",
                        channel_id, baud, output_rate);
            if (jc->pmsk)
                chan_init_thread(jc);
        }
        if (!jc || !jc->pmsk) return;

        chan_push(jc, samples, num_samples);
        return;
    }

    /* Aero 10500 baud OQPSK forward link: continuous OqpskDemodulator + AeroL. */
    if (type == CHAN_AERO_10500) {
        double output_rate = channelizer_output_rate(channelizer, channel_id);
        if (output_rate <= 0) return;

        jaero_chan_t *jc = NULL;
        for (int i = 0; i < num_jaero_chans; i++) {
            if (jaero_chans[i].channel_id == channel_id) {
                jc = &jaero_chans[i];
                break;
            }
        }
        if (!jc && num_jaero_chans < MAX_JAERO_DEMODS) {
            jc = &jaero_chans[num_jaero_chans++];
            jc->channel_id  = channel_id;
            jc->baud_rate   = 10500;
            jc->pmsk        = NULL;
            jc->oqpsk       = NULL;
            jc->oqpsk_cont  = NULL;
            jc->mixer_phase = 0.0;
            jc->mixer_inc   = 2.0 * M_PI * AUDIO_CENTER_HZ / output_rate;

            jc->oqpsk_cont = jaero_oqpsk_cont_create(output_rate, 10500.0,
                                                       channel_id, jaero_bits_cb, NULL);
            if (jc->oqpsk_cont)
                jaero_oqpsk_cont_set_acars_callback(jc->oqpsk_cont,
                                                     jaero_acars_data_cb, NULL);
            if (verbose)
                fprintf(stderr, "[OQPSK-CONT ch%d] baud=10500 rate=%.0f (continuous)\n",
                        channel_id, output_rate);
            if (jc->oqpsk_cont)
                chan_init_thread(jc);
        }
        if (!jc || !jc->oqpsk_cont) return;

        chan_push(jc, samples, num_samples);
        return;
    }

    /* Aero 8400 baud OQPSK C-channel (voice/data). */
    if (type == CHAN_AERO_8400) {
        double output_rate = channelizer_output_rate(channelizer, channel_id);
        if (output_rate <= 0) return;

        jaero_chan_t *jc = NULL;
        for (int i = 0; i < num_jaero_chans; i++) {
            if (jaero_chans[i].channel_id == channel_id) {
                jc = &jaero_chans[i];
                break;
            }
        }
        if (!jc && num_jaero_chans < MAX_JAERO_DEMODS) {
            jc = &jaero_chans[num_jaero_chans++];
            jc->channel_id  = channel_id;
            jc->baud_rate   = 8400;
            jc->pmsk        = NULL;
            jc->oqpsk       = NULL;
            jc->mixer_phase = 0.0;
            jc->mixer_inc   = 2.0 * M_PI * AUDIO_CENTER_HZ / output_rate;

            jc->oqpsk_cont = jaero_oqpsk_cont_create(output_rate, 8400.0,
                                                       channel_id, jaero_bits_cb, NULL);
            if (jc->oqpsk_cont)
                jaero_oqpsk_cont_set_acars_callback(jc->oqpsk_cont,
                                                     jaero_acars_data_cb, NULL);
            if (verbose)
                fprintf(stderr, "[OQPSK-CONT ch%d] baud=8400 rate=%.0f (continuous)\n",
                        channel_id, output_rate);
            if (jc->oqpsk_cont)
                chan_init_thread(jc);
        }
        if (!jc || !jc->oqpsk_cont) return;

        chan_push(jc, samples, num_samples);
        return;
    }
}

/* ---- Main ---- */

int main(int argc, char **argv) {
    self_pid = getpid();

    /* If stderr is a pipe, non-block it so fprintf() on the decode path
     * can't stall when a 2>&1 | consumer falls behind. Log lines get
     * dropped on EAGAIN — preferred over freezing decode. */
    {
        struct stat st;
        if (fstat(STDERR_FILENO, &st) == 0 && S_ISFIFO(st.st_mode)) {
            int flags = fcntl(STDERR_FILENO, F_GETFL, 0);
            if (flags >= 0)
                fcntl(STDERR_FILENO, F_SETFL, flags | O_NONBLOCK);
        }
    }

    parse_options(argc, argv);

    /* Detect CPU features and wire up SIMD kernel function pointers. */
    simd_init(0);

    /* Initialize feed output */
    feed_init();

#ifdef HAVE_MQTT
    if (mqtt_enabled) {
        extern int mqtt_init(const char *, int, const char *, const char *,
                              const char *);
        if (mqtt_init(mqtt_host, mqtt_port, mqtt_user, mqtt_pass,
                       mqtt_topic) != 0)
            errx(1, "Failed to initialize MQTT");
    }
#endif

    /* Update aircraft DB if requested, then exit (matches iridium-sniffer). */
    if (update_db_flag) {
        int rc = aircraft_db_update();
        return rc < 0 ? 1 : 0;
    }

    /* Start web dashboard */
    if (web_enabled) {
        if (web_init(web_port) != 0)
            errx(1, "Failed to start web dashboard");
    }

    /* Aircraft DB — always load if available. Used for:
     *   - AES → aircraft type/operator enrichment on ACARS output
     *   - Registration → ICAO hex lookup for SBS basestation output */
    {
        const char *dbpath = aircraft_db_path ? aircraft_db_path
                                               : aircraft_db_default_path();
        if (dbpath && aircraft_db_load(dbpath) < 0 && basestation_enabled) {
            fprintf(stderr, "aircraft_db: no database found\n"
                    "  Run: inmarsat-sniffer --update-db\n"
                    "  Or specify: --aircraft-db=PATH\n");
        }
    }
    if (basestation_enabled) {
        if (basestation_init(basestation_endpoint) != 0)
            errx(1, "Failed to start basestation output");
    }

    /* Optional waypoint DB (for fallback position extraction from ACARS
     * text that references named fixes rather than coordinates). */
    {
        char wp_path[512];
        ssize_t exe_len = readlink("/proc/self/exe", wp_path, sizeof(wp_path) - 1);
        if (exe_len > 0) {
            wp_path[exe_len] = '\0';
            char *slash = strrchr(wp_path, '/');
            if (slash) {
                snprintf(slash + 1, sizeof(wp_path) - (slash + 1 - wp_path),
                         "../data/waypoints.csv");
                if (waypoint_db_load(wp_path) < 0) {
                    snprintf(slash + 1, sizeof(wp_path) - (slash + 1 - wp_path),
                             "data/waypoints.csv");
                    if (waypoint_db_load(wp_path) < 0)
                        waypoint_db_load("/usr/share/inmarsat-sniffer/waypoints.csv");
                }
            }
        }
    }

#ifdef HAVE_ZMQ
    if (zmq_enabled) {
        if (zmq_audio_init(zmq_base_port) != 0)
            errx(1, "Failed to initialize ZMQ audio");
    }
#endif

    /* Look up satellite */
    const satellite_t *sat = NULL;
    if (satellite_name) {
        sat = satellite_lookup(satellite_name);
        if (!sat)
            errx(1, "Unknown satellite: %s (use --list-satellites)", satellite_name);

        fprintf(stderr, "Satellite: %s (%s, %+.1f%s)\n",
                sat->name, sat->region,
                fabs(sat->position), sat->position < 0 ? "W" : "E");

        /* Aero+C span on 4F3/3F5 exceeds RTL-SDR's ~2.88 MHz usable BW
         * because C-channels sit ~3 MHz below the aero cluster. Skip them. */
#ifdef HAVE_RTLSDR
        if (rtl_dev_index >= 0 && !skip_c_channel && samp_rate == 0 &&
            op_mode != MODE_STDC) {
            double aero_lo = 1e12, aero_hi = 0;
            for (int i = 0; i < sat->num_channels; i++) {
                const channel_def_t *cd = &sat->channels[i];
                if (cd->type == CHAN_STDC_EGC) continue;
                if (cd->frequency < aero_lo) aero_lo = cd->frequency;
                if (cd->frequency > aero_hi) aero_hi = cd->frequency;
            }
            /* RTL-SDR practical max ≈ 2.88 MHz; use 2.4 MHz for safety */
            if ((aero_hi - aero_lo) * 1.2 > 2400000.0) {
                fprintf(stderr,
                    "RTL-SDR: %s aero+C span is %.2f MHz, exceeds RTL-SDR's\n"
                    "         usable max. Auto-enabling --skip-c-channel to\n"
                    "         skip OQPSK 8400 C-channels (voice/data, rarely\n"
                    "         carry ACARS). Pass --skip-c-channel explicitly\n"
                    "         to silence this message.\n",
                    sat->name, (aero_hi - aero_lo) / 1e6);
                skip_c_channel = 1;
            }
        }
#endif

        /* Auto center/rate from the channels that will actually decode. */
        double lo = 1e12, hi = 0;
        for (int i = 0; i < sat->num_channels; i++) {
            const channel_def_t *cd = &sat->channels[i];
            if (op_mode == MODE_AERO && cd->type == CHAN_STDC_EGC) continue;
            if (op_mode == MODE_STDC && cd->type != CHAN_STDC_EGC) continue;
            if (skip_c_channel && cd->type == CHAN_AERO_8400) continue;
            if (cd->frequency < lo) lo = cd->frequency;
            if (cd->frequency > hi) hi = cd->frequency;
        }
        if (lo > hi) { lo = sat->freq_min; hi = sat->freq_max; }

        if (center_freq == 0) {
            center_freq = (lo + hi) / 2.0;
            if (verbose)
                fprintf(stderr, "Auto center freq: %.3f MHz\n", center_freq / 1e6);
        }
        if (samp_rate == 0) {
            double span = hi - lo;

            /* Prefer the known-good (satellite, SDR) rate from
             * SDRReceiver configs or live testing — set on satellite_t when
             * authoritative data exists. Falls back to max(span*1.2, floor). */
            double preferred = 0;
#ifdef HAVE_RTLSDR
            if (rtl_dev_index >= 0) preferred = sat->preferred_rate_rtl;
#endif
#ifdef HAVE_SDRPLAY
            if (sdrplay_serial != NULL) preferred = sat->preferred_rate_sdrplay;
#endif
#ifdef HAVE_HACKRF
            if (hackrf_serial != NULL) preferred = sat->preferred_rate_hackrf;
#endif

            if (preferred > 0 && preferred >= span) {
                samp_rate = preferred;
            } else {
                samp_rate = span * 1.2;
                double min_rate = 2400000;
#ifdef HAVE_RTLSDR
                if (rtl_dev_index >= 0) min_rate = 1536000; else
#endif
#ifdef HAVE_SDRPLAY
                if (sdrplay_serial != NULL) min_rate = 3072000; else
#endif
#ifdef HAVE_HACKRF
                if (hackrf_serial != NULL) min_rate = 6000000; else
#endif
                { /* default 2400000 */ }
                if (samp_rate < min_rate)
                    samp_rate = min_rate;
            }
            if (verbose)
                fprintf(stderr, "Auto sample rate: %.3f MHz (span %.3f MHz, %s)\n",
                        samp_rate / 1e6, span / 1e6,
                        (preferred > 0 && preferred >= span) ? "preferred" : "auto");
        }
    }

    if (samp_rate == 0) {
        samp_rate = 2400000;
#ifdef HAVE_RTLSDR
        if (rtl_dev_index >= 0) samp_rate = 1536000;
#endif
#ifdef HAVE_SDRPLAY
        if (sdrplay_serial != NULL) samp_rate = 3072000;
#endif
#ifdef HAVE_HACKRF
        if (hackrf_serial != NULL) samp_rate = 6000000;
#endif
    }
    if (center_freq == 0)
        center_freq = 1545100000.0;

    /* Auto mode selection based on available bandwidth */
    if (op_mode == MODE_AUTO && sat) {
        double bw = samp_rate;

        /* Count how many channel types are reachable */
        int have_stdc = 0, have_aero = 0;
        for (int i = 0; i < sat->num_channels; i++) {
            double offset = fabs(sat->channels[i].frequency - center_freq);
            if (offset > bw / 2.0)
                continue;
            if (sat->channels[i].type == CHAN_STDC_EGC)
                have_stdc = 1;
            else
                have_aero = 1;
        }

        if (have_stdc && have_aero) {
            op_mode = MODE_FULL;
            fprintf(stderr, "Auto mode: full (STD-C + Aero)\n");
        } else if (have_stdc) {
            op_mode = MODE_STDC;
            fprintf(stderr, "Auto mode: STD-C only\n");
        } else if (have_aero) {
            op_mode = MODE_AERO;
            fprintf(stderr, "Auto mode: Aero only\n");
        } else {
            fprintf(stderr, "Warning: no channels within SDR bandwidth\n");
        }
    }

    fprintf(stderr, "Center: %.3f MHz  Rate: %.3f MHz\n",
            center_freq / 1e6, samp_rate / 1e6);

    /* Set up signal handler */
    struct sigaction sa = { .sa_handler = sig_handler };
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* Initialize queues */
    blocking_queue_init(&samples_queue, SAMPLES_QUEUE_SIZE);
    blocking_queue_init(&decoded_queue, DECODED_QUEUE_SIZE);

    /* Start input thread */
    pthread_t input_tid;
    void *rtl_dev = NULL;
    if (vita49_enabled) {
        pthread_create(&input_tid, NULL, vita49_thread, NULL);
    } else if (live) {
#ifdef HAVE_RTLSDR
        if (rtl_dev_index >= 0) {
            rtl_dev = rtlsdr_backend_setup(rtl_dev_index);
            pthread_create(&input_tid, NULL, rtlsdr_stream_thread, rtl_dev);
        } else
#endif
#ifdef HAVE_HACKRF
        if (hackrf_serial) {
            void *dev = hackrf_backend_setup(hackrf_serial);
            pthread_create(&input_tid, NULL, hackrf_stream_thread, dev);
        } else
#endif
#ifdef HAVE_BLADERF
        if (bladerf_num >= 0) {
            void *dev = bladerf_backend_setup(bladerf_num);
            pthread_create(&input_tid, NULL, bladerf_stream_thread, dev);
        } else
#endif
#ifdef HAVE_UHD
        if (usrp_serial) {
            void *dev = usrp_backend_setup(usrp_serial);
            pthread_create(&input_tid, NULL, usrp_stream_thread, dev);
        } else
#endif
#ifdef HAVE_AIRSPY
        if (airspy_selected) {
            void *dev = airspy_backend_setup(airspy_serial);
            pthread_create(&input_tid, NULL, airspy_stream_thread, dev);
        } else
#endif
#ifdef HAVE_SDRPLAY
        if (sdrplay_serial) {
            void *ctx = sdrplay_setup(sdrplay_serial);
            pthread_create(&input_tid, NULL, sdrplay_stream_thread, ctx);
        } else
#endif
#ifdef HAVE_SOAPYSDR
        {
            SoapySDRDevice *device;
            if (soapy_args)
                device = soapy_setup(-1, soapy_args);
            else
                device = soapy_setup(soapy_num, NULL);
            pthread_create(&input_tid, NULL, soapy_stream_thread, device);
        }
#else
        errx(1, "No SDR backend available");
#endif
    } else {
        pthread_create(&input_tid, NULL, spewer_thread, in_file);
    }

    /* Set up channelizer */
    if (sat) {
        channelizer = channelizer_create(center_freq, samp_rate,
                                          channel_output_cb, NULL);
        if (!channelizer)
            errx(1, "Failed to create channelizer");

        int added = 0;
        for (int i = 0; i < sat->num_channels; i++) {
            const channel_def_t *cd = &sat->channels[i];

            /* Check if channel is within SDR bandwidth */
            double offset = fabs(cd->frequency - center_freq);
            if (offset > samp_rate / 2.0) {
                if (verbose)
                    fprintf(stderr, "Channel %d (%.3f MHz) outside usable bandwidth, skipping\n",
                            cd->channel_id, cd->frequency / 1e6);
                continue;
            }

            /* Mode filtering */
            if (op_mode == MODE_AERO && cd->type == CHAN_STDC_EGC)
                continue;
            if (op_mode == MODE_STDC && cd->type != CHAN_STDC_EGC)
                continue;
            if (skip_c_channel && cd->type == CHAN_AERO_8400)
                continue;
            /* OQPSK channels included for ZMQ audio output */

            if (channelizer_add_channel(channelizer, cd->frequency,
                                         cd->type, cd->channel_id) == 0) {
                added++;
            }
        }
        long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
        fprintf(stderr, "Active channels: %d (%ld CPU cores)\n",
                added, ncpu > 0 ? ncpu : 1);
        /* Per-channel summary — always on, short form */
        for (int i = 0; i < sat->num_channels; i++) {
            const channel_def_t *cd = &sat->channels[i];
            double offset = fabs(cd->frequency - center_freq);
            if (offset > samp_rate / 2.0) continue;
            if (op_mode == MODE_AERO && cd->type == CHAN_STDC_EGC) continue;
            if (op_mode == MODE_STDC && cd->type != CHAN_STDC_EGC) continue;
            if (skip_c_channel && cd->type == CHAN_AERO_8400) continue;
            const char *label;
            int baud = 0;
            switch (cd->type) {
            case CHAN_STDC_EGC:   label = "STD-C EGC";  baud = 1200; break;
            case CHAN_AERO_600:   label = "P-chan MSK"; baud = 600;  break;
            case CHAN_AERO_1200:  label = "P-chan MSK"; baud = 1200; break;
            case CHAN_AERO_10500: label = "Fwd OQPSK";  baud = 10500; break;
            case CHAN_AERO_8400:  label = "C-chan OQPSK"; baud = 8400; break;
            default:              label = "?";           break;
            }
            fprintf(stderr, "  ch%-3d  %-13s @ %.3f MHz  %5d baud\n",
                    cd->channel_id, label, cd->frequency / 1e6, baud);
        }

        /* Rebalance bands so no channel sits at DC (where offset/1/f noise hurt) */
        channelizer_finalize(channelizer);

        /* Universal --ppm shift: RTL-SDR tuned its TCXO at backend setup,
         * but every other backend hits the channelizer with an uncorrected
         * carrier. Apply ppm × center_freq as a software offset here so
         * --ppm works the same way on all SDRs. */
#ifdef HAVE_RTLSDR
        int rtl_active = (rtl_dev_index >= 0);
#else
        int rtl_active = 0;
#endif
        if (ppm_correction != 0.0 && !rtl_active) {
            double shift_hz = ppm_correction * center_freq * 1e-6;
            channelizer_adjust_center(channelizer, shift_hz);
            fprintf(stderr, "PPM correction: %.2f ppm (%.0f Hz at %.3f MHz)\n",
                    ppm_correction, shift_hz, center_freq / 1e6);
        }

        /* STD-C demod/decode chain if an EGC channel is active */
        for (int i = 0; i < sat->num_channels; i++) {
            if (sat->channels[i].type == CHAN_STDC_EGC &&
                (op_mode != MODE_AERO)) {
                double output_rate = channelizer_output_rate(channelizer, sat->channels[i].channel_id);
                if (output_rate <= 0) output_rate = 19200.0;
                stdc_decoder = stdc_decoder_create(stdc_message_cb, NULL);
                stdc_demod = dbpsk_demod_create(output_rate, 1200.0,
                                                  stdc_bits_cb, NULL);
                if (stdc_demod && stdc_decoder)
                    fprintf(stderr, "STD-C EGC decoder: active\n");
                break;
            }
        }

        /* Aero decoder if any Aero channel is active */
        if (op_mode != MODE_STDC) {
            int have_aero = 0;
            for (int i = 0; i < sat->num_channels; i++) {
                if (sat->channels[i].type == CHAN_AERO_600 ||
                    sat->channels[i].type == CHAN_AERO_1200 ||
                    sat->channels[i].type == CHAN_AERO_8400 ||
                    sat->channels[i].type == CHAN_AERO_10500) {
                    have_aero = 1;
                    break;
                }
            }
            if (have_aero) {
                fprintf(stderr, "Aero decoder: JAERO/AeroL embedded\n");
#ifdef HAVE_LIBACARS
                acars_reasm_ctx = la_reasm_ctx_new();
                if (acars_reasm_ctx)
                    fprintf(stderr, "libacars %s: ACARS reassembly active\n",
                            LA_VERSION);
#endif
            }
        }
    }

    /* Main processing loop */
    unsigned long status_interval = 0;
    while (running) {
        sample_buf_t *buf;
        int ret = blocking_queue_poll(&samples_queue, &buf);
        if (ret == BQ_CLOSED)
            break;
        if (ret != 0) {
            usleep(1000);
            continue;
        }

        if (channelizer) {
            /* Debug: print signal power every 500 buffers (verbose only) */
            static int dbg_cnt = 0;
            if (verbose && ++dbg_cnt == 500) {
                dbg_cnt = 0;
                double pwr = 0;
                if (buf->format == SAMPLE_FMT_FLOAT) {
                    float *f = (float *)buf->samples;
                    for (int i = 0; i < 100 && i < (int)buf->num * 2; i++)
                        pwr += f[i] * f[i];
                } else {
                    for (int i = 0; i < 100 && i < (int)buf->num * 2; i++)
                        pwr += buf->samples[i] * buf->samples[i];
                }
                fprintf(stderr, "\n[DBG] fmt=%d num=%u pwr=%.6f samples: ",
                        buf->format, buf->num, pwr);
                if (buf->format == SAMPLE_FMT_FLOAT) {
                    float *f = (float *)buf->samples;
                    for (int i = 0; i < 10; i++) fprintf(stderr, "%.4f ", f[i]);
                } else {
                    for (int i = 0; i < 10; i++) fprintf(stderr, "%d ", buf->samples[i]);
                }
                fprintf(stderr, "\n");
            }

            if (buf->format == SAMPLE_FMT_INT8)
                channelizer_process_i8(channelizer, buf->samples, buf->num);
            else
                channelizer_process(channelizer, (float *)buf->samples, buf->num);
        }

        free(buf);

        if (++status_interval % 100 == 0)
            print_status();
    }

    fprintf(stderr, "\nShutting down...\n");
    /* Three independent extraction paths, see README "Position sources". */
    fprintf(stderr, "Position fixes: %lu binary ADS-C, %lu text-coord, %lu waypoint-name\n",
            atomic_load(&stat_pos_adsc),
            atomic_load(&stat_pos_text),
            atomic_load(&stat_pos_waypoint));

#ifdef HAVE_RTLSDR
    /* Cancel async read so stream thread can exit */
    if (rtl_dev)
        rtlsdr_backend_close(rtl_dev);
#endif

    /* Cleanup */
    blocking_queue_close(&samples_queue);
    blocking_queue_close(&decoded_queue);

    pthread_join(input_tid, NULL);

    /* Destroy demods before decoders -- demod destroy flushes remaining
     * bits through the callback, which needs the decoder still alive. */
    dbpsk_demod_destroy(stdc_demod);
    /* Stop all per-channel worker threads first so the ring is quiesced
     * before freeing the demods they're feeding. */
    for (int i = 0; i < num_jaero_chans; i++) {
        if (jaero_chans[i].ring) {
            atomic_store(&jaero_chans[i].thread_run, 0);
            pthread_join(jaero_chans[i].thread, NULL);
        }
    }
    for (int i = 0; i < num_jaero_chans; i++) {
        if (jaero_chans[i].pmsk)
            jaero_pmsk_destroy(jaero_chans[i].pmsk);
        if (jaero_chans[i].oqpsk)
            jaero_oqpsk_destroy(jaero_chans[i].oqpsk);
        if (jaero_chans[i].oqpsk_cont)
            jaero_oqpsk_cont_destroy(jaero_chans[i].oqpsk_cont);
        free(jaero_chans[i].ring);
        jaero_chans[i].ring = NULL;
    }

    stdc_decoder_destroy(stdc_decoder);
    channelizer_destroy(channelizer);

    if (basestation_enabled)
        basestation_destroy();
    aircraft_db_destroy();

#ifdef HAVE_MQTT
    if (mqtt_enabled) {
        extern void mqtt_cleanup(void);
        mqtt_cleanup();
    }
#endif

#ifdef HAVE_ZMQ
    if (zmq_enabled)
        zmq_audio_cleanup();
#endif

    blocking_queue_destroy(&samples_queue);
    blocking_queue_destroy(&decoded_queue);

#ifdef HAVE_LIBACARS
    if (acars_reasm_ctx)
        la_reasm_ctx_destroy(acars_reasm_ctx);
#endif

#ifdef HAVE_SOAPYSDR
    /* device cleanup handled by stream thread */
#endif

    web_shutdown();
    feed_shutdown();

    if (in_file && in_file != stdin)
        fclose(in_file);

    return 0;
}
