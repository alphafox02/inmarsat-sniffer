/*
 * Coherent BPSK demodulator for STD-C / EGC (1200 baud)
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DEMOD_BPSK_H
#define DEMOD_BPSK_H

#include <complex.h>
#include <stdint.h>

/* Soft bits: positive = 1, negative = 0, nominal magnitude ~0.7.
 * Polarity is ambiguous (180 deg); the frame sync resolves it. */
typedef void (*bpsk_bit_cb_t)(const float *soft_bits, int num_bits,
                               void *user);

typedef struct bpsk_demod bpsk_demod_t;

/* samp_rate need not be an integer multiple of symbol_rate. */
bpsk_demod_t *bpsk_demod_create(double samp_rate, double symbol_rate,
                                 bpsk_bit_cb_t cb, void *user);

void bpsk_demod_process(bpsk_demod_t *d, const float complex *samples,
                         int num_samples);

void bpsk_demod_destroy(bpsk_demod_t *d);

#endif
