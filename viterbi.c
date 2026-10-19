/*
 * Viterbi decoder for rate 1/2, constraint length 7 convolutional code
 *
 * Polynomials: G1 = 109 (0b1101101), G2 = 79 (0b1001111)
 * Standard NASA/CCSDS code used in Inmarsat STD-C and Aero.
 *
 * This is a basic soft-decision Viterbi decoder with full traceback.
 * Not optimized -- correctness over speed.
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "viterbi.h"

#define G1 0x6D  /* 109 = 1101101 -- matches JAERO polys[0]=109 */
#define G2 0x4F  /* 79 = 1001111 -- matches JAERO polys[1]=79 */
#define K  7
#define RATE 2

/* Count bits set in a byte (popcount) */
static int parity(unsigned int x) {
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return x & 1;
}

/* Compute expected output symbols for a given state transition.
 * state: current encoder state (6 bits for k=7)
 * input_bit: 0 or 1
 * Returns packed symbol pair: (g1 << 1) | g2 */
static int encode_sym(int state, int input_bit) {
    int reg = (input_bit << (K - 1)) | state;
    int g1 = parity(reg & G1);
    int g2 = parity(reg & G2);
    return (g1 << 1) | g2;
}

void viterbi_init(viterbi_t *v, int num_bits) {
    v->num_bits = num_bits;
    v->traceback_len = num_bits + K;  /* extra for tail */

    /* Allocate path history: one uint64_t per state, per time step.
     * Each bit in the uint64_t represents the decision at that step.
     * For long sequences we use a sliding window approach. */
    v->path_history = calloc((size_t)VITERBI_STATES * v->traceback_len,
                              sizeof(uint64_t));

    /* Initialize path metrics -- state 0 is the starting state */
    for (int i = 0; i < VITERBI_STATES; i++)
        v->path_metric[i] = (i == 0) ? 0 : 0x7FFFFFFF;
}

/* Branch metric: distance between received soft symbols and expected.
 * soft_g1, soft_g2: received soft values (positive = 1, negative = 0)
 * exp_g1, exp_g2: expected hard bits (0 or 1)
 * Returns metric (lower = better match) */
static uint32_t branch_metric(int8_t soft_g1, int8_t soft_g2,
                                int exp_g1, int exp_g2) {
    /* Convert expected to soft: 0 -> -127, 1 -> +127 */
    int e1 = exp_g1 ? 127 : -127;
    int e2 = exp_g2 ? 127 : -127;

    /* Euclidean-ish distance (Manhattan for speed) */
    int d1 = abs(soft_g1 - e1);
    int d2 = abs(soft_g2 - e2);
    return (uint32_t)(d1 + d2);
}

uint32_t viterbi_decode(viterbi_t *v, const int8_t *soft_syms,
                         uint8_t *bits_out) {
    int T = v->num_bits;

    /* Forward pass */
    for (int t = 0; t < T; t++) {
        int8_t s0 = soft_syms[t * 2];      /* G1 */
        int8_t s1 = soft_syms[t * 2 + 1];  /* G2 */

        for (int s = 0; s < VITERBI_STATES; s++)
            v->path_metric_new[s] = 0xFFFFFFFF;

        for (int s = 0; s < VITERBI_STATES; s++) {
            if (v->path_metric[s] >= 0x7FFFFFFF)
                continue;

            for (int bit = 0; bit < 2; bit++) {
                int next_state = (s >> 1) | (bit << (K - 2));
                int expected = encode_sym(s, bit);
                int exp_g1 = (expected >> 1) & 1;
                int exp_g2 = expected & 1;

                uint32_t bm = branch_metric(s0, s1, exp_g1, exp_g2);
                uint32_t pm = v->path_metric[s] + bm;

                if (pm < v->path_metric_new[next_state]) {
                    v->path_metric_new[next_state] = pm;
                    /* Store the input bit that led to this state */
                    v->path_history[(size_t)t * VITERBI_STATES + next_state] =
                        ((uint64_t)s << 1) | bit;
                }
            }
        }

        memcpy(v->path_metric, v->path_metric_new,
               VITERBI_STATES * sizeof(uint32_t));
    }

    /* Find best final state */
    uint32_t best_metric = 0xFFFFFFFF;
    int best_state = 0;
    for (int s = 0; s < VITERBI_STATES; s++) {
        if (v->path_metric[s] < best_metric) {
            best_metric = v->path_metric[s];
            best_state = s;
        }
    }

    /* Traceback */
    uint8_t *decoded = calloc(T, sizeof(uint8_t));
    int state = best_state;
    for (int t = T - 1; t >= 0; t--) {
        uint64_t entry = v->path_history[(size_t)t * VITERBI_STATES + state];
        int bit = entry & 1;
        int prev_state = (int)(entry >> 1);
        decoded[t] = bit;
        state = prev_state;
    }

    /* Pack bits into bytes (MSB first) */
    if (T <= 0) { free(decoded); return best_metric; }
    memset(bits_out, 0, (size_t)((T + 7) / 8));
    for (int i = 0; i < T; i++) {
        if (decoded[i])
            bits_out[i / 8] |= (0x80 >> (i % 8));
    }

    free(decoded);
    return best_metric;
}

void viterbi_free(viterbi_t *v) {
    free(v->path_history);
    v->path_history = NULL;
}

void viterbi_encode(const uint8_t *bits, int num_bits, uint8_t *syms_out) {
    int state = 0;
    for (int i = 0; i < num_bits; i++) {
        int bit = (bits[i / 8] >> (7 - i % 8)) & 1;
        int sym = encode_sym(state, bit);
        syms_out[i * 2]     = (sym >> 1) & 1;
        syms_out[i * 2 + 1] = sym & 1;
        state = (state >> 1) | (bit << (K - 2));
    }
}

float viterbi_ber(const uint8_t *bits, int num_bits, const int8_t *soft_syms) {
    if (num_bits <= 0) return 0.5f;
    uint8_t *syms = malloc((size_t)num_bits * 2);
    if (!syms) return 0.5f;
    viterbi_encode(bits, num_bits, syms);
    int errors = 0;
    for (int i = 0; i < num_bits * 2; i++)
        if ((soft_syms[i] > 0) != syms[i])
            errors++;
    free(syms);
    return (float)errors / (num_bits * 2);
}
