/*
 * STD-C loopback test: synthesize NCS frames (bulletin board + EGC text),
 * BPSK-modulate with impairments, run through demod + decoder.
 *
 * Usage: test_stdc [options]
 *   -n FRAMES     frames to send (default 4)
 *   -s ESN0_DB    Es/N0 in dB (default 20)
 *   -f HZ         carrier offset (default 0)
 *   -r RATE       sample rate (default 19200)
 *   -t FRAC       timing offset in symbols (default 0.37)
 *   -b            bypass demod: feed ideal soft bits straight to decoder
 *   -o FILE       also write baseband as cf32
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE

#include <complex.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "demod_bpsk.h"
#include "stdc_decode.h"
#include "viterbi.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

atomic_ulong stat_stdc_frames = 0;
atomic_ulong stat_stdc_crc_ok = 0;
atomic_ulong stat_stdc_crc_fail = 0;
atomic_ulong stat_stdc_ber_sum = 0;
atomic_ulong stat_stdc_ber_count = 0;
atomic_int stat_stdc_synced = 0;

/* Must match stdc_decode.c */
static const uint8_t sync_word[64] = {
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0,
    1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0,
    0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1,
    0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0,
};

static const uint8_t scramble_seq[160] = {
    0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,0,0,0,0,0,1,
    1,0,0,1,0,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,0,0,1,0,1,0,1,1,0,1,1,
    0,1,0,1,1,0,0,1,0,1,1,0,0,0,0,1,1,1,1,1,0,1,1,0,1,1,1,1,0,1,0,1,
    1,1,0,1,0,0,0,1,0,0,0,0,1,1,0,1,1,0,0,0,1,1,1,1,0,0,1,1,1,0,0,1,
    1,0,0,0,1,0,1,1,0,1,0,0,1,0,0,0,1,0,1,0,0,1,0,1,0,1,0,0,1,1,1,0,
};

static const char *egc_text = "SECURITE NAVAREA I 123/26 TEST MESSAGE";

static int got_bulletins = 0;
static int got_egc = 0;
static int last_frame_no = -1;

static uint8_t reverse_bits(uint8_t b)
{
    b = ((b >> 1) & 0x55) | ((b & 0x55) << 1);
    b = ((b >> 2) & 0x33) | ((b & 0x33) << 2);
    b = ((b >> 4) & 0x0F) | ((b & 0x0F) << 4);
    return b;
}

static void put_crc(uint8_t *pkt, int len)
{
    short c0 = 0, c1 = 0;
    for (int i = 0; i < len; i++) {
        uint8_t b = (i < len - 2) ? pkt[i] : 0;
        c0 += b;
        c1 += c0;
    }
    pkt[len - 2] = (uint8_t)(c0 - c1);
    pkt[len - 1] = (uint8_t)(c1 - 2 * c0);
}

/* 640-byte NCS frame: bulletin board (0x7D) + EGC part 1 (0xB1) */
static void build_frame(uint8_t *f, int frame_no)
{
    memset(f, 0, STDC_FRAME_BYTES);
    int p = 0;

    uint8_t *bb = &f[p];
    bb[0] = 0x7D;
    bb[1] = 1;
    bb[2] = (uint8_t)(frame_no >> 8);
    bb[3] = (uint8_t)frame_no;
    bb[7] = (1 << 6) | 2;   /* AOR-E, LES 2 */
    put_crc(bb, 14);
    p += 14;

    int tlen = (int)strlen(egc_text);
    int addr_len = 4;       /* service 0x31 */
    int len = 8 + addr_len + tlen + 2;
    uint8_t *eg = &f[p];
    eg[0] = 0xB1;
    eg[1] = (uint8_t)(len - 2);
    eg[2] = 0x31;
    eg[3] = (1 << 5) | 1;   /* safety, repetition 1 */
    eg[4] = 0x12;
    eg[5] = 0x34;
    eg[6] = 1;
    eg[7] = 0;              /* IA5 */
    memcpy(&eg[8 + addr_len], egc_text, tlen);
    put_crc(eg, len);
}

/* Inverse of stdc_decode.c: scramble, encode, interleave, permute, add UW */
static void frame_to_symbols(const uint8_t *frame, uint8_t *out)
{
    uint8_t scr[STDC_FRAME_BYTES];
    for (int i = 0; i < 160; i++)
        for (int j = 0; j < 4; j++) {
            uint8_t b = frame[i * 4 + j];
            if (scramble_seq[i]) b ^= 0xFF;
            scr[i * 4 + j] = reverse_bits(b);
        }

    uint8_t enc[STDC_DATA_SIZE];
    viterbi_encode(scr, STDC_DATA_SIZE / 2, enc);

    for (int i = 0; i < STDC_FRAME_ROWS; i++) {
        uint8_t *row = &out[((i * 23) % 64) * STDC_FRAME_COLS];
        for (int col = 0; col < 160; col++)
            row[col + 2] = enc[col * STDC_FRAME_ROWS + i];
    }
    for (int r = 0; r < STDC_FRAME_ROWS; r++) {
        out[r * STDC_FRAME_COLS] = sync_word[r];
        out[r * STDC_FRAME_COLS + 1] = sync_word[r];
    }
}

static double rrc(double t, double alpha)
{
    if (fabs(t) < 1e-9)
        return 1.0 - alpha + 4.0 * alpha / M_PI;
    if (fabs(fabs(t) - 1.0 / (4.0 * alpha)) < 1e-9)
        return alpha / sqrt(2.0) *
               ((1 + 2 / M_PI) * sin(M_PI / (4 * alpha)) +
                (1 - 2 / M_PI) * cos(M_PI / (4 * alpha)));
    double num = sin(M_PI * t * (1 - alpha)) +
                 4 * alpha * t * cos(M_PI * t * (1 + alpha));
    double den = M_PI * t * (1 - (4 * alpha * t) * (4 * alpha * t));
    return num / den;
}

static double gauss(void)
{
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
    double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2 * M_PI * u2);
}

static void msg_cb(const stdc_message_t *msg, void *user)
{
    (void)user;
    if (msg->descriptor == 0x7D) {
        got_bulletins++;
        last_frame_no = msg->frame_number;
    } else if (msg->descriptor == 0xB1 && strstr(msg->text, egc_text)) {
        got_egc++;
    }
}

static stdc_decoder_t *decoder;
static float *rx_soft;
static long rx_n, rx_cap;

static void bits_cb(const float *soft, int n, void *user)
{
    (void)user;
    if (rx_n + n > rx_cap) {
        rx_cap = (rx_n + n) * 2;
        rx_soft = realloc(rx_soft, sizeof(float) * rx_cap);
    }
    memcpy(&rx_soft[rx_n], soft, sizeof(float) * n);
    rx_n += n;
    stdc_decoder_feed(decoder, soft, n);
}

/* Raw symbol error rate over the second half of the run (after settling),
 * best over small lags and both polarities. */
static double raw_ser(const uint8_t *bits, long nsym)
{
    double best = 1.0;
    for (int lag = -20; lag <= 20; lag++) {
        long errs[2] = {0, 0}, n = 0;
        for (long i = rx_n / 2; i < rx_n; i++) {
            long k = i + lag;
            if (k < 0 || k >= nsym) continue;
            int hard = rx_soft[i] > 0;
            errs[0] += hard != bits[k];
            errs[1] += hard == bits[k];
            n++;
        }
        for (int p = 0; p < 2 && n; p++)
            if ((double)errs[p] / n < best)
                best = (double)errs[p] / n;
    }
    return best;
}

int main(int argc, char **argv)
{
    int frames = 4, bypass = 0;
    double esn0_db = 20.0, foff = 0.0, fs = 19200.0, toff = 0.37;
    const char *out_path = NULL;
    int opt;

    while ((opt = getopt(argc, argv, "n:s:f:r:t:bo:")) != -1) {
        switch (opt) {
        case 'n': frames = atoi(optarg); break;
        case 's': esn0_db = atof(optarg); break;
        case 'f': foff = atof(optarg); break;
        case 'r': fs = atof(optarg); break;
        case 't': toff = atof(optarg); break;
        case 'b': bypass = 1; break;
        case 'o': out_path = optarg; break;
        default:
            fprintf(stderr, "usage: %s [-n frames] [-s esn0_db] [-f hz] "
                    "[-r rate] [-t frac] [-b] [-o out.cf32]\n", argv[0]);
            return 2;
        }
    }
    srand(1);

    /* One extra leading frame so the receiver can settle before sync */
    int total_frames = frames + 1;
    int nsym = total_frames * STDC_ENCODED_SIZE;
    uint8_t *bits = malloc(nsym);
    for (int k = 0; k < total_frames; k++) {
        uint8_t frame[STDC_FRAME_BYTES];
        build_frame(frame, 1000 + k);
        frame_to_symbols(frame, &bits[k * STDC_ENCODED_SIZE]);
    }

    decoder = stdc_decoder_create(msg_cb, NULL);
    double esn0 = pow(10.0, esn0_db / 10.0);

    if (bypass) {
        float soft[256];
        double sigma = sqrt(1.0 / (2.0 * esn0));
        for (int i = 0; i < nsym; i += 256) {
            int n = nsym - i < 256 ? nsym - i : 256;
            for (int j = 0; j < n; j++)
                soft[j] = (float)((bits[i + j] ? 1.0 : -1.0) + sigma * gauss()) * 0.7f;
            stdc_decoder_feed(decoder, soft, n);
        }
    } else {
        const double sym_rate = 1200.0, alpha = 0.6;
        double sps = fs / sym_rate;
        int span = 8;
        long nsamp = (long)((nsym + span) * sps);
        float complex *iq = malloc(sizeof(*iq) * nsamp);

        /* Unit symbol energy: sum of p^2 over one symbol's samples = sps */
        double pnorm = 0;
        for (int i = -span * (int)ceil(sps); i <= span * (int)ceil(sps); i++)
            pnorm += pow(rrc(i / sps, alpha), 2);
        pnorm = sqrt(sps / pnorm);
        double sigma = sqrt(sps / esn0 / 2.0);
        double phase0 = 1.1;

        for (long n = 0; n < nsamp; n++) {
            double t = n / sps - toff;          /* in symbols */
            int k0 = (int)floor(t) - span;
            double acc = 0;
            for (int k = k0; k <= k0 + 2 * span; k++) {
                if (k < 0 || k >= nsym) continue;
                acc += (bits[k] ? 1.0 : -1.0) * rrc(t - k, alpha);
            }
            double ph = phase0 + 2 * M_PI * foff * n / fs;
            iq[n] = (float complex)(acc * pnorm * cexp(I * ph) +
                                    sigma * (gauss() + I * gauss()));
        }

        if (out_path) {
            FILE *fo = fopen(out_path, "wb");
            if (fo) {
                fwrite(iq, sizeof(*iq), nsamp, fo);
                fclose(fo);
            }
        }

        bpsk_demod_t *demod = bpsk_demod_create(fs, sym_rate, bits_cb, NULL);
        for (long n = 0; n < nsamp; n += 1024) {
            int len = nsamp - n < 1024 ? (int)(nsamp - n) : 1024;
            bpsk_demod_process(demod, &iq[n], len);
        }
        bpsk_demod_destroy(demod);
        free(iq);
        printf("raw symbol error rate %.4f (theory %.4f)\n", raw_ser(bits, nsym),
               0.5 * erfc(sqrt(esn0)));
    }

    unsigned long nframes = atomic_load(&stat_stdc_frames);
    unsigned long bc = atomic_load(&stat_stdc_ber_count);
    double ber = bc ? atomic_load(&stat_stdc_ber_sum) / (bc * 10000.0) : 0;
    printf("Es/N0 %.1f dB  foff %.0f Hz  rate %.1f  %s\n", esn0_db, foff, fs,
           bypass ? "(bits only)" : "");
    printf("frames %lu/%d  bulletins %d  egc %d  crc ok %lu fail %lu  "
           "channel ber %.4f  last frame %d\n",
           nframes, frames, got_bulletins, got_egc,
           atomic_load(&stat_stdc_crc_ok), atomic_load(&stat_stdc_crc_fail),
           ber, last_frame_no);

    stdc_decoder_destroy(decoder);
    free(bits);
    free(rx_soft);

    int pass = got_bulletins >= frames && got_egc >= frames;
    printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
