/*
 * Viterbi decoder for rate 1/2, constraint length 7 convolutional code
 *
 * Used by both STD-C and Aero decoders.
 * Polynomials: G1 = 109 (0x6D), G2 = 79 (0x4F)
 * This is the standard NASA/CCSDS convolutional code.
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef VITERBI_H
#define VITERBI_H

#include <stdint.h>

/* Number of states for k=7 (2^(k-1) = 64) */
#define VITERBI_STATES 64

/* Viterbi decoder state */
typedef struct {
    uint32_t path_metric[VITERBI_STATES];
    uint32_t path_metric_new[VITERBI_STATES];
    uint64_t *path_history;  /* path history for traceback */
    int traceback_len;       /* traceback length */
    int num_bits;            /* number of output bits */
} viterbi_t;

/* Initialize the Viterbi decoder.
 * num_bits: number of decoded output bits expected. */
void viterbi_init(viterbi_t *v, int num_bits);

/* Decode soft symbols to bits.
 * soft_syms: input soft symbols (pairs: G1, G2), length = num_bits * 2
 *   Positive values favor 1, negative favor 0.
 * bits_out: output decoded bits packed into bytes (MSB first).
 *   Length = ceil(num_bits / 8) bytes.
 * Returns the best path metric (lower = more confident). */
uint32_t viterbi_decode(viterbi_t *v, const int8_t *soft_syms,
                         uint8_t *bits_out);

/* Free decoder resources. */
void viterbi_free(viterbi_t *v);

/* Encode packed bits (MSB first) to 0/1 symbol pairs (G1, G2). */
void viterbi_encode(const uint8_t *bits, int num_bits, uint8_t *syms_out);

/* Channel symbol error rate: re-encode decoded bits and compare against
 * hard decisions of the received soft symbols. Independent of soft scale. */
float viterbi_ber(const uint8_t *bits, int num_bits, const int8_t *soft_syms);

#endif
