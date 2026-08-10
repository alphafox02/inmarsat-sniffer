/*
 * ZMQ raw IQ output.
 *
 * Frame: [topic "IQ"] [uint32 sample_rate_hz] [uint64 sequence]
 *        [complex64 samples]
 *
 * Samples are always little-endian-ish host float complex pairs on the wire:
 * float32 I, float32 Q, repeated. Existing integer/file/SDR input formats are
 * normalized here so downstream tools can subscribe without knowing the SDR
 * backend that produced the stream.
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zmq.h>

#include "zmq_iq.h"

static void *iq_ctx = NULL;
static void *iq_pub = NULL;

int zmq_iq_init(int port)
{
    iq_ctx = zmq_ctx_new();
    if (!iq_ctx) return -1;

    iq_pub = zmq_socket(iq_ctx, ZMQ_PUB);
    if (!iq_pub) return -1;

    int hwm = 8;
    zmq_setsockopt(iq_pub, ZMQ_SNDHWM, &hwm, sizeof(hwm));

    char addr[64];
    snprintf(addr, sizeof(addr), "tcp://0.0.0.0:%d", port);
    if (zmq_bind(iq_pub, addr) != 0) {
        fprintf(stderr, "ZMQ IQ: failed to bind %s\n", addr);
        return -1;
    }

    fprintf(stderr, "ZMQ IQ: tcp://127.0.0.1:%d "
            "(topic IQ, cf32 payload)\n", port);
    return 0;
}

void zmq_iq_send(const sample_buf_t *buf, double samp_rate,
                 uint64_t sequence)
{
    if (!iq_pub || !buf || buf->num == 0) return;

    float *out = malloc((size_t)buf->num * 2 * sizeof(float));
    if (!out) return;

    if (buf->format == SAMPLE_FMT_FLOAT) {
        memcpy(out, buf->samples, (size_t)buf->num * 2 * sizeof(float));
    } else {
        const int8_t *s = buf->samples;
        for (unsigned i = 0; i < buf->num * 2; i++)
            out[i] = (float)s[i] / 128.0f;
    }

    uint32_t rate = (uint32_t)(samp_rate + 0.5);
    zmq_send(iq_pub, "IQ", 2, ZMQ_SNDMORE | ZMQ_DONTWAIT);
    zmq_send(iq_pub, &rate, sizeof(rate), ZMQ_SNDMORE | ZMQ_DONTWAIT);
    zmq_send(iq_pub, &sequence, sizeof(sequence), ZMQ_SNDMORE | ZMQ_DONTWAIT);
    zmq_send(iq_pub, out, (size_t)buf->num * 2 * sizeof(float), ZMQ_DONTWAIT);

    free(out);
}

void zmq_iq_cleanup(void)
{
    if (iq_pub) {
        zmq_close(iq_pub);
        iq_pub = NULL;
    }
    if (iq_ctx) {
        zmq_ctx_destroy(iq_ctx);
        iq_ctx = NULL;
    }
}
