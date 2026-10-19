/*
 * STD-C frame sync, deinterleaver, descrambler, and packet parser
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef STDC_DECODE_H
#define STDC_DECODE_H

#include <stdint.h>

/*
 * STD-C frame parameters:
 *
 * Frame structure: 64 rows x 162 columns = 10368 encoded bits
 * - 2 sync bits per row (positions 0,1) = 128 sync bits
 * - 160 data bits per row = 10240 data bits
 * - 10240 data bits / 2 (rate 1/2) = 5120 decoded bits = 640 bytes
 *
 * Processing order:
 * 1. Frame sync (correlate sync word across 64 rows)
 * 2. Depermute (row reordering: row i -> row (i*23)%64)
 * 3. Deinterleave (column readout)
 * 4. Viterbi decode (rate 1/2, k=7)
 * 5. Descramble (XOR with scrambling sequence + bit reversal)
 * 6. Parse packets
 */

#define STDC_FRAME_ROWS     64
#define STDC_FRAME_COLS     162
#define STDC_ENCODED_SIZE   10368   /* 64 * 162 */
#define STDC_DATA_SIZE      10240   /* 64 * 160 (no sync bits) */
#define STDC_FRAME_BYTES    640     /* decoded frame size */

/* Packet descriptor types (from inmarsatc) */
#define STDC_PKT_ACK_REQUEST        0x08
#define STDC_PKT_CHANNEL_CLEAR      0x27
#define STDC_PKT_MSG_ACK            0x2A
#define STDC_PKT_SIGNALLING_CH      0x6C
#define STDC_PKT_BULLETIN_BOARD     0x7D
#define STDC_PKT_ANNOUNCEMENT       0x81
#define STDC_PKT_CHAN_ASSIGNMENT     0x83
#define STDC_PKT_DISTRESS_ACK       0x91
#define STDC_PKT_LOGIN_ACK          0x92
#define STDC_PKT_DATA_REPORT_ACK    0x9A
#define STDC_PKT_DISTRESS_TEST      0xA0
#define STDC_PKT_INDIVIDUAL_POLL    0xA3
#define STDC_PKT_CONFIRMATION       0xA8
#define STDC_PKT_MESSAGE            0xAA
#define STDC_PKT_LES_LIST           0xAB
#define STDC_PKT_REQUEST_STATUS     0xAC
#define STDC_PKT_TEST_RESULT        0xAD
#define STDC_PKT_EGC_DOUBLE_1       0xB1
#define STDC_PKT_EGC_DOUBLE_2       0xB2
#define STDC_PKT_MULTIFRAME_START   0xBD
#define STDC_PKT_MULTIFRAME_CONT    0xBE

/* Legacy enum kept for feed.c / web.c compatibility */
typedef enum {
    STDC_MSG_EGC_SINGLE     = 0x30,
    STDC_MSG_EGC_DOUBLE_1   = 0xB1,
    STDC_MSG_EGC_DOUBLE_2   = 0xB2,
    STDC_MSG_ANNOUNCEMENT   = 0x81,
    STDC_MSG_BULLETIN       = 0x7D,
    STDC_MSG_DISTRESS_ACK   = 0x91,
    STDC_MSG_MESSAGE_DATA   = 0xAA,
    STDC_MSG_NET_UPDATE     = 0x6C,
    STDC_MSG_MULTIFRAME_START = 0xBD,
    STDC_MSG_MULTIFRAME_CONT  = 0xBE,
    STDC_MSG_CHANNEL_CLEAR  = 0x27,
    STDC_MSG_ACK_REQUEST    = 0x08,
    STDC_MSG_MSG_ACK        = 0x2A,
    STDC_MSG_CHAN_ASSIGNMENT = 0x83,
    STDC_MSG_LOGIN_ACK      = 0x92,
    STDC_MSG_INDIVIDUAL_POLL = 0xA3,
    STDC_MSG_CONFIRMATION   = 0xA8,
    STDC_MSG_LES_LIST       = 0xAB,
} stdc_msg_type_t;

/* Decoded STD-C message */
typedef struct {
    stdc_msg_type_t type;
    uint8_t descriptor;         /* raw packet descriptor byte */

    /* Satellite / LES identification */
    int sat_id;                 /* 0=AOR-W, 1=AOR-E, 2=POR, 3=IOR */
    int les_id;
    const char *sat_name;
    const char *les_name;

    /* EGC fields */
    int service_code;
    int priority;               /* 0=Routine, 1=Safety, 2=Urgency, 3=Distress */
    int repetition;
    int message_id;             /* EGC message sequence number */
    int continuation;           /* 1 = more parts follow */
    int packet_no;

    /* Channel info */
    int logical_channel;
    double uplink_mhz;
    double downlink_mhz;

    /* MES identity */
    int mes_id;

    /* Presentation layer */
    int presentation;           /* 0=IA5, 6=ITA2, 7=Binary */

    /* Frame timing */
    int frame_number;

    /* Message text */
    char text[4096];
    int text_len;

    /* Position (from SafetyNET address decoding) */
    double lat, lon;
    int has_position;

    uint64_t timestamp;

    /* CRC status */
    int crc_ok;
} stdc_message_t;

/* Callback for decoded STD-C messages */
typedef void (*stdc_msg_cb_t)(const stdc_message_t *msg, void *user);

/* STD-C decoder state (opaque) */
typedef struct stdc_decoder stdc_decoder_t;

/* Create a STD-C decoder.
 * cb: callback for decoded messages
 * user: passed to callback */
stdc_decoder_t *stdc_decoder_create(stdc_msg_cb_t cb, void *user);

/* Feed soft bits from the BPSK demodulator.
 * The decoder handles frame sync, deinterleaving, Viterbi decode,
 * descrambling, and message parsing internally. */
void stdc_decoder_feed(stdc_decoder_t *d, const float *soft_bits,
                        int num_bits);

/* Destroy the decoder. */
void stdc_decoder_destroy(stdc_decoder_t *d);

#endif
