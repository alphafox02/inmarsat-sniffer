/*
 * ZMQ raw IQ output
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef ZMQ_IQ_H
#define ZMQ_IQ_H

#include <stdint.h>
#include "sdr.h"

int zmq_iq_init(int port);
void zmq_iq_send(const sample_buf_t *buf, double samp_rate,
                 uint64_t sequence);
void zmq_iq_cleanup(void);

#endif
