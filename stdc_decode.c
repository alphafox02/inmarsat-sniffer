/*
 * STD-C frame sync, deinterleaver, descrambler, and packet parser
 *
 * Frame processing pipeline:
 * soft bits -> frame sync -> depermute -> deinterleave -> Viterbi -> descramble -> parse
 *
 * Packet parsing based on protocol analysis of cropinghigh's inmarsatc library
 * and SatDump's Inmarsat-C support module.
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <time.h>

#include "stdc_decode.h"
#include "viterbi.h"

/* Sync word -- 64 bits, one per frame row.
 * Each row starts with 2 bits: sync[i] at position [i*162+0] and [i*162+1]. */
static const uint8_t sync_word[64] = {
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0,
    1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0,
    0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1,
    0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0,
};

/* Descrambling sequence -- 160 elements, one per data column */
static const uint8_t scramble_seq[160] = {
    0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,0,0,0,0,0,1,
    1,0,0,1,0,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,0,0,1,0,1,0,1,1,0,1,1,
    0,1,0,1,1,0,0,1,0,1,1,0,0,0,0,1,1,1,1,1,0,1,1,0,1,1,1,1,0,1,0,1,
    1,1,0,1,0,0,0,1,0,0,0,0,1,1,0,1,1,0,0,0,1,1,1,1,0,0,1,1,1,0,0,1,
    1,0,0,0,1,0,1,1,0,1,0,0,1,0,0,0,1,0,1,0,0,1,0,1,0,1,0,0,1,1,1,0,
};

/* Reverse bits in a byte */
static uint8_t reverse_bits(uint8_t b) {
    b = ((b >> 1) & 0x55) | ((b & 0x55) << 1);
    b = ((b >> 2) & 0x33) | ((b & 0x33) << 2);
    b = ((b >> 4) & 0x0F) | ((b & 0x0F) << 4);
    return b;
}

extern atomic_ulong stat_stdc_frames;
extern atomic_ulong stat_stdc_crc_ok;
extern atomic_ulong stat_stdc_crc_fail;
extern atomic_ulong stat_stdc_ber_sum;
extern atomic_ulong stat_stdc_ber_count;
extern atomic_int stat_stdc_synced;

/* ---- Satellite / LES lookup tables ---- */

static const char *get_sat_name(int sat) {
    switch (sat) {
    case 0: return "AOR-W";
    case 1: return "AOR-E";
    case 2: return "POR";
    case 3: return "IOR";
    case 9: return "All Regions";
    default: return "Unknown";
    }
}

static const char *get_les_name(int sat, int les_id) {
    int v = les_id + sat * 100;
    switch (v) {
    case 1: case 101: case 201: case 301:
        return "Vizada-Telenor, USA";
    case 2: case 102: case 302:
        return "Stratos Global (Burum-2), Netherlands";
    case 202:
        return "Stratos Global (Auckland), New Zealand";
    case 3: case 103: case 203: case 303:
        return "KDDI, Japan";
    case 4: case 104: case 204: case 304:
        return "Vizada-Telenor, Norway";
    case 44: case 144: case 244: case 344:
        return "NCS";
    case 105: case 335:
        return "Telecom, Italy";
    case 305: case 120:
        return "OTESTAT, Greece";
    case 306:
        return "VSNL, India";
    case 110: case 310:
        return "Turk Telecom, Turkey";
    case 211: case 311:
        return "Beijing MCN, China";
    case 12: case 112: case 212: case 312:
        return "Stratos Global (Burum), Netherlands";
    case 114:
        return "Embratel, Brazil";
    case 116: case 316:
        return "TP, Poland";
    case 117: case 217: case 317:
        return "Morsviazsputnik, Russia";
    case 21: case 121: case 221: case 321:
        return "Vizada (FT), France";
    case 127: case 327:
        return "Bezeq, Israel";
    case 210: case 328:
        return "Singapore Telecom, Singapore";
    case 330:
        return "VISHIPEL, Vietnam";
    default:
        return "Unknown";
    }
}

static const char *get_service_name(int code) {
    switch (code) {
    case 0x00: return "All ships";
    case 0x02: return "FleetNET group call";
    case 0x04: return "SafetyNET rect area warning";
    case 0x11: return "Inmarsat system message";
    case 0x13: return "Coastal warning";
    case 0x14: return "Distress alert (circular)";
    case 0x23: return "EGC system message";
    case 0x24: return "SafetyNET circular area warning";
    case 0x31: return "NAVAREA/METAREA warning";
    case 0x33: return "Download group identity";
    case 0x34: return "SAR coordination (rect)";
    case 0x44: return "SAR coordination (circular)";
    case 0x72: return "FleetNET chart correction";
    case 0x73: return "SafetyNET chart correction";
    default:   return "Unknown";
    }
}

static const char *get_priority_name(int p) {
    switch (p) {
    case 0: return "Routine";
    case 1: return "Safety";
    case 2: return "Urgency";
    case 3: return "Distress";
    default: return "Unknown";
    }
}

static int get_address_length(int msg_type) {
    switch (msg_type) {
    case 0x00: return 3;
    case 0x11: case 0x31: return 4;
    case 0x02: case 0x72: return 5;
    case 0x13: case 0x23: case 0x33: case 0x73: return 6;
    case 0x04: case 0x14: case 0x24: case 0x34: case 0x44: return 7;
    default: return 3;
    }
}

static const char *get_descriptor_name(uint8_t d) {
    switch (d) {
    case 0x08: return "Ack Request";
    case 0x27: return "Channel Clear";
    case 0x2A: return "Message Ack";
    case 0x6C: return "Signalling Channel";
    case 0x7D: return "Bulletin Board";
    case 0x81: return "Announcement";
    case 0x83: return "Channel Assignment";
    case 0x91: return "Distress Alert Ack";
    case 0x92: return "Login Ack";
    case 0x9A: return "Data Report Ack";
    case 0xA0: return "Distress Test Request";
    case 0xA3: return "Individual Poll";
    case 0xA8: return "Confirmation";
    case 0xAA: return "Message";
    case 0xAB: return "LES List";
    case 0xAC: return "Request Status";
    case 0xAD: return "Test Result";
    case 0xB1: return "EGC Part 1";
    case 0xB2: return "EGC Part 2";
    case 0xBD: return "Multiframe Start";
    case 0xBE: return "Multiframe Continue";
    default:   return "Unknown";
    }
}

/* ---- CRC-16 (Fletcher-style, from inmarsatc) ---- */

static int compute_crc(const uint8_t *data, int pos, int length) {
    short c0 = 0, c1 = 0;
    for (int i = 0; i < length; i++) {
        uint8_t b = (i < length - 2) ? data[pos + i] : 0;
        c0 += b;
        c1 += c0;
    }
    uint8_t cb1 = (uint8_t)(c0 - c1);
    uint8_t cb2 = (uint8_t)(c1 - 2 * c0);
    return (cb1 << 8) | cb2;
}

/* ---- Multiframe reassembly state ---- */

typedef struct {
    int active;
    uint8_t data[4096];
    int total_len;          /* expected total length of encapsulated packet */
    int filled;             /* bytes filled so far */
} multiframe_state_t;

/* ---- EGC reassembly state ---- */

#define EGC_MAX_REASSEMBLY 16
#define EGC_TIMEOUT_SEC    60

typedef struct {
    int active;
    int message_id;
    int presentation;
    int service_code;
    int priority;
    int repetition;
    uint8_t parts[8192];
    int part_lens[64];      /* length of each part */
    int max_pkt_no;
    int complete;           /* got final continuation=0 */
    time_t last_seen;
} egc_reassembly_t;

/* ---- Decoder state ---- */

struct stdc_decoder {
    stdc_msg_cb_t cb;
    void *user;

    /* Soft bit shift register for frame sync */
    int8_t shift_reg[STDC_ENCODED_SIZE];
    int shift_pos;
    int synced;
    int frame_count;

    /* Working buffers */
    int8_t depermuted[STDC_ENCODED_SIZE];
    int8_t deinterleaved[STDC_DATA_SIZE];
    uint8_t decoded[STDC_FRAME_BYTES];

    /* Multiframe packet reassembly */
    multiframe_state_t mfp;

    /* EGC reassembly slots */
    egc_reassembly_t egc_slots[EGC_MAX_REASSEMBLY];
};

stdc_decoder_t *stdc_decoder_create(stdc_msg_cb_t cb, void *user) {
    stdc_decoder_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->cb = cb;
    d->user = user;
    return d;
}

/* ---- Frame sync ---- */

static int check_sync(const int8_t *buf, int *inverted) {
    int match_nrm = 0;
    int match_inv = 0;

    for (int i = 0; i < STDC_FRAME_ROWS; i++) {
        int pos = i * STDC_FRAME_COLS;
        int bit0 = buf[pos] > 0 ? 1 : 0;
        int bit1 = buf[pos + 1] > 0 ? 1 : 0;

        if (bit0 == sync_word[i]) match_nrm++;
        else match_inv++;

        if (bit1 == sync_word[i]) match_nrm++;
        else match_inv++;
    }

    if (match_inv > match_nrm) {
        *inverted = 1;
        return match_inv;
    }
    *inverted = 0;
    return match_nrm;
}

/* Depermute: reorder rows. Row i in output <- row ((i*23) % 64) in input */
static void depermute(const int8_t *in, int8_t *out) {
    for (int i = 0; i < STDC_FRAME_ROWS; i++)
        memcpy(&out[i * STDC_FRAME_COLS],
               &in[((i * 23) % 64) * STDC_FRAME_COLS],
               STDC_FRAME_COLS);
}

/* Deinterleave: extract data bits (skip sync), column-major readout */
static void deinterleave(const int8_t *in, int8_t *out) {
    for (int row = 0; row < STDC_FRAME_ROWS; row++)
        for (int col = 0; col < 160; col++)
            out[col * STDC_FRAME_ROWS + row] = in[row * STDC_FRAME_COLS + col + 2];
}

/* Descramble decoded bytes */
static void descramble(uint8_t *pkt) {
    for (int i = 0; i < 160; i++) {
        for (int j = 0; j < 4; j++) {
            pkt[i * 4 + j] = reverse_bits(pkt[i * 4 + j]);
            if (scramble_seq[i])
                pkt[i * 4 + j] ^= 0xFF;
        }
    }
}

/* ---- Packet length from descriptor ---- */

static int get_packet_length(const uint8_t *frame, int pos, int frame_len) {
    uint8_t desc = frame[pos];

    if ((desc >> 7) == 0) {
        /* Short descriptor: length in low 4 bits, doesn't include byte 0 */
        return (desc & 0x0F) + 1;
    } else if ((desc >> 6) == 0x02) {
        /* Medium descriptor: length in next byte, doesn't include first 2 bytes */
        if (pos + 1 >= frame_len) return frame_len - pos;
        return frame[pos + 1] + 2;
    }
    /* Long or unknown -- consume rest of frame as safety fallback */
    return frame_len - pos;
}

/* ---- Extract IA5 text from payload bytes ---- */

static int extract_ia5_text(char *out, int max_out,
                            const uint8_t *data, int len) {
    int tlen = 0;
    for (int i = 0; i < len && tlen < max_out - 1; i++) {
        char c = data[i] & 0x7F;
        if (c >= 0x20 || c == '\n' || c == '\r' || c == '\t')
            out[tlen++] = c;
        else if (c == 0x03) /* ETX */
            break;
    }
    out[tlen] = '\0';
    return tlen;
}

/* ---- Emit a message through the callback ---- */

static void emit_message(stdc_decoder_t *d, stdc_message_t *msg) {
    if (d->cb)
        d->cb(msg, d->user);
}

/* ---- EGC reassembly ---- */

static egc_reassembly_t *find_egc_slot(stdc_decoder_t *d, int msg_id) {
    time_t now = time(NULL);

    /* Find existing slot */
    for (int i = 0; i < EGC_MAX_REASSEMBLY; i++) {
        if (d->egc_slots[i].active && d->egc_slots[i].message_id == msg_id)
            return &d->egc_slots[i];
    }

    /* Find empty or expired slot */
    for (int i = 0; i < EGC_MAX_REASSEMBLY; i++) {
        if (!d->egc_slots[i].active ||
            (now - d->egc_slots[i].last_seen > EGC_TIMEOUT_SEC)) {
            memset(&d->egc_slots[i], 0, sizeof(d->egc_slots[i]));
            d->egc_slots[i].active = 1;
            d->egc_slots[i].message_id = msg_id;
            d->egc_slots[i].last_seen = now;
            return &d->egc_slots[i];
        }
    }

    /* Evict oldest */
    int oldest = 0;
    for (int i = 1; i < EGC_MAX_REASSEMBLY; i++) {
        if (d->egc_slots[i].last_seen < d->egc_slots[oldest].last_seen)
            oldest = i;
    }
    memset(&d->egc_slots[oldest], 0, sizeof(d->egc_slots[oldest]));
    d->egc_slots[oldest].active = 1;
    d->egc_slots[oldest].message_id = msg_id;
    d->egc_slots[oldest].last_seen = now;
    return &d->egc_slots[oldest];
}

static void try_emit_egc(stdc_decoder_t *d, egc_reassembly_t *slot) {
    if (!slot->complete) return;

    /* Reassemble all parts in order */
    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_EGC_DOUBLE_1;
    msg.descriptor = STDC_PKT_EGC_DOUBLE_1;
    msg.service_code = slot->service_code;
    msg.priority = slot->priority;
    msg.repetition = slot->repetition;
    msg.message_id = slot->message_id;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;
    msg.crc_ok = 1;

    /* Unused return values -- use lookup tables for coverage */
    (void)get_service_name(slot->service_code);
    (void)get_priority_name(slot->priority);

    int total = 0;
    for (int i = 0; i <= slot->max_pkt_no && total < (int)sizeof(msg.text) - 1; i++) {
        int off = i * 128;  /* max part size */
        int plen = slot->part_lens[i];
        if (plen <= 0) continue;
        if (plen > 128) plen = 128;

        if (slot->presentation == 0) {
            /* IA5 */
            int n = extract_ia5_text(msg.text + total,
                                     sizeof(msg.text) - total,
                                     slot->parts + off, plen);
            total += n;
        } else {
            /* Binary -- copy raw */
            int n = plen;
            if (total + n >= (int)sizeof(msg.text)) n = sizeof(msg.text) - total - 1;
            memcpy(msg.text + total, slot->parts + off, n);
            total += n;
        }
    }
    msg.text[total] = '\0';
    msg.text_len = total;

    if (total > 0)
        emit_message(d, &msg);

    slot->active = 0;
}

/* ---- Individual packet decoders ---- */

static void decode_channel_clear(stdc_decoder_t *d, const uint8_t *frame,
                                  int pos, int pkt_len) {
    /* 0x27: Logical Channel Clear */
    if (pkt_len < 8) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_CHANNEL_CLEAR;
    msg.descriptor = 0x27;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.mes_id = (frame[pos + 1] << 16) | (frame[pos + 2] << 8) | frame[pos + 3];
    msg.sat_id = (frame[pos + 4] >> 6) & 0x03;
    msg.les_id = frame[pos + 4] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);
    msg.logical_channel = frame[pos + 5];

    snprintf(msg.text, sizeof(msg.text),
             "Channel Clear: MES=%d LES=%s ch=%d",
             msg.mes_id, msg.les_name, msg.logical_channel);
    msg.text_len = strlen(msg.text);

    emit_message(d, &msg);
}

static void decode_msg_ack(stdc_decoder_t *d, const uint8_t *frame,
                            int pos, int pkt_len) {
    /* 0x2A: Inbound Message Ack */
    if (pkt_len < 9) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_MSG_ACK;
    msg.descriptor = 0x2A;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.mes_id = (frame[pos + 1] << 16) | (frame[pos + 2] << 8) | frame[pos + 3];
    msg.sat_id = (frame[pos + 4] >> 6) & 0x03;
    msg.les_id = frame[pos + 4] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);
    msg.logical_channel = frame[pos + 5];

    snprintf(msg.text, sizeof(msg.text),
             "Message Ack: MES=%d LES=%s ch=%d",
             msg.mes_id, msg.les_name, msg.logical_channel);
    msg.text_len = strlen(msg.text);

    emit_message(d, &msg);
}

static void decode_ack_request(stdc_decoder_t *d, const uint8_t *frame,
                                int pos, int pkt_len) {
    /* 0x08: Acknowledgement Request */
    if (pkt_len < 6) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_ACK_REQUEST;
    msg.descriptor = 0x08;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.sat_id = (frame[pos + 1] >> 6) & 0x03;
    msg.les_id = frame[pos + 1] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);
    msg.logical_channel = frame[pos + 2];
    msg.uplink_mhz = ((frame[pos + 3] << 8 | frame[pos + 4]) - 6000) * 0.0025 + 1626.5;

    snprintf(msg.text, sizeof(msg.text),
             "Ack Request: LES=%s ch=%d uplink=%.4f MHz",
             msg.les_name, msg.logical_channel, msg.uplink_mhz);
    msg.text_len = strlen(msg.text);

    emit_message(d, &msg);
}

static void decode_signalling(stdc_decoder_t *d, const uint8_t *frame,
                               int pos, int pkt_len) {
    /* 0x6C: Signalling Channel */
    if (pkt_len < 11) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_NET_UPDATE;
    msg.descriptor = 0x6C;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.uplink_mhz = ((frame[pos + 2] << 8 | frame[pos + 3]) - 6000) * 0.0025 + 1626.5;

    snprintf(msg.text, sizeof(msg.text),
             "Signalling Channel: uplink=%.4f MHz", msg.uplink_mhz);
    msg.text_len = strlen(msg.text);

    emit_message(d, &msg);
}

static void decode_bulletin_board(stdc_decoder_t *d, const uint8_t *frame,
                                   int pos, int pkt_len) {
    /* 0x7D: Bulletin Board */
    if (pkt_len < 14) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_BULLETIN;
    msg.descriptor = 0x7D;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    int nv = frame[pos + 1];
    msg.frame_number = (frame[pos + 2] << 8) | frame[pos + 3];

    /* Frame timestamp: frame_number * 8.64 seconds into the day */
    double ts_sec = msg.frame_number * 8.64;
    int ts_h = (int)(ts_sec / 3600.0);
    int ts_m = ((int)ts_sec % 3600) / 60;
    int ts_s = (int)ts_sec % 60;

    msg.sat_id = (frame[pos + 7] >> 6) & 0x03;
    msg.les_id = frame[pos + 7] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);

    snprintf(msg.text, sizeof(msg.text),
             "Bulletin Board: LES=%s sat=%s frame=%d time=%02d:%02d:%02d ver=%d [%s]",
             msg.les_name, msg.sat_name, msg.frame_number,
             ts_h, ts_m, ts_s, nv, get_descriptor_name(0x7D));
    msg.text_len = strlen(msg.text);

    emit_message(d, &msg);
}

static void decode_announcement(stdc_decoder_t *d, const uint8_t *frame,
                                 int pos, int pkt_len) {
    /* 0x81: Announcement */
    if (pkt_len < 17) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_ANNOUNCEMENT;
    msg.descriptor = 0x81;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.mes_id = (frame[pos + 2] << 16) | (frame[pos + 3] << 8) | frame[pos + 4];
    msg.sat_id = (frame[pos + 5] >> 6) & 0x03;
    msg.les_id = frame[pos + 5] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);
    msg.logical_channel = frame[pos + 9];
    msg.downlink_mhz = ((frame[pos + 6] << 8 | frame[pos + 7]) - 8000) * 0.0025 + 1530.5;
    msg.presentation = frame[pos + 14];

    snprintf(msg.text, sizeof(msg.text),
             "Announcement: MES=%d LES=%s ch=%d downlink=%.4f MHz pres=%d",
             msg.mes_id, msg.les_name, msg.logical_channel,
             msg.downlink_mhz, msg.presentation);
    msg.text_len = strlen(msg.text);

    emit_message(d, &msg);
}

static void decode_chan_assignment(stdc_decoder_t *d, const uint8_t *frame,
                                   int pos, int pkt_len) {
    /* 0x83: Logical Channel Assignment */
    if (pkt_len < 16) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_CHAN_ASSIGNMENT;
    msg.descriptor = 0x83;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.mes_id = (frame[pos + 2] << 16) | (frame[pos + 3] << 8) | frame[pos + 4];
    msg.sat_id = (frame[pos + 5] >> 6) & 0x03;
    msg.les_id = frame[pos + 5] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);
    msg.logical_channel = frame[pos + 7];
    msg.downlink_mhz = ((frame[pos + 10] << 8 | frame[pos + 11]) - 8000) * 0.0025 + 1530.5;
    msg.uplink_mhz = ((frame[pos + 12] << 8 | frame[pos + 13]) - 6000) * 0.0025 + 1626.5;

    snprintf(msg.text, sizeof(msg.text),
             "Channel Assignment: MES=%d LES=%s ch=%d down=%.4f up=%.4f MHz",
             msg.mes_id, msg.les_name, msg.logical_channel,
             msg.downlink_mhz, msg.uplink_mhz);
    msg.text_len = strlen(msg.text);

    emit_message(d, &msg);
}

static void decode_login_ack(stdc_decoder_t *d, const uint8_t *frame,
                              int pos, int pkt_len) {
    /* 0x92: Login Ack */
    if (pkt_len < 8) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_LOGIN_ACK;
    msg.descriptor = 0x92;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.downlink_mhz = ((frame[pos + 5] << 8 | frame[pos + 6]) - 8000) * 0.0025 + 1530.5;

    snprintf(msg.text, sizeof(msg.text),
             "Login Ack: downlink=%.4f MHz", msg.downlink_mhz);
    msg.text_len = strlen(msg.text);

    emit_message(d, &msg);
}

static void decode_individual_poll(stdc_decoder_t *d, const uint8_t *frame,
                                    int pos, int pkt_len) {
    /* 0xA3: Individual Poll */
    if (pkt_len < 8) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_INDIVIDUAL_POLL;
    msg.descriptor = 0xA3;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.mes_id = (frame[pos + 2] << 16) | (frame[pos + 3] << 8) | frame[pos + 4];
    msg.sat_id = (frame[pos + 5] >> 6) & 0x03;
    msg.les_id = frame[pos + 5] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);

    /* Check for short message in longer packets */
    if (pkt_len >= 38) {
        int tlen = 0;
        for (int i = pos + 13; i < pos + pkt_len - 2 && tlen < (int)sizeof(msg.text) - 1; i++) {
            char c = frame[i] & 0x7F;
            if (c >= 0x20 || c == '\n' || c == '\r')
                msg.text[tlen++] = c;
        }
        msg.text[tlen] = '\0';
        msg.text_len = tlen;
    } else {
        snprintf(msg.text, sizeof(msg.text),
                 "Individual Poll: MES=%d LES=%s",
                 msg.mes_id, msg.les_name);
        msg.text_len = strlen(msg.text);
    }

    emit_message(d, &msg);
}

static void decode_confirmation(stdc_decoder_t *d, const uint8_t *frame,
                                 int pos, int pkt_len) {
    /* 0xA8: Confirmation */
    if (pkt_len < 9) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_CONFIRMATION;
    msg.descriptor = 0xA8;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.mes_id = (frame[pos + 2] << 16) | (frame[pos + 3] << 8) | frame[pos + 4];
    msg.sat_id = (frame[pos + 5] >> 6) & 0x03;
    msg.les_id = frame[pos + 5] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);

    /* Short message if present */
    int sm_len = frame[pos + 9];
    if (sm_len > 2 && pos + 11 < pos + pkt_len - 2) {
        int tlen = 0;
        for (int i = pos + 11; i < pos + pkt_len - 2 && tlen < (int)sizeof(msg.text) - 1; i++) {
            char c = frame[i] & 0x7F;
            if (c >= 0x20 || c == '\n' || c == '\r')
                msg.text[tlen++] = c;
        }
        msg.text[tlen] = '\0';
        msg.text_len = tlen;
    } else {
        snprintf(msg.text, sizeof(msg.text),
                 "Confirmation: MES=%d LES=%s",
                 msg.mes_id, msg.les_name);
        msg.text_len = strlen(msg.text);
    }

    emit_message(d, &msg);
}

static void decode_message(stdc_decoder_t *d, const uint8_t *frame,
                            int pos, int pkt_len) {
    /* 0xAA: Message Data (payload on logical channel) */
    if (pkt_len < 7) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_MESSAGE_DATA;
    msg.descriptor = 0xAA;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    msg.sat_id = (frame[pos + 2] >> 6) & 0x03;
    msg.les_id = frame[pos + 2] & 0x3F;
    msg.sat_name = get_sat_name(msg.sat_id);
    msg.les_name = get_les_name(msg.sat_id, msg.les_id);
    msg.logical_channel = frame[pos + 3];
    msg.packet_no = frame[pos + 4];

    /* Extract payload (bytes after header, before CRC) */
    int payload_start = pos + 5;
    int payload_len = pkt_len - 5 - 2; /* minus header minus CRC */
    if (payload_len < 0) payload_len = 0;

    msg.text_len = extract_ia5_text(msg.text, sizeof(msg.text),
                                     &frame[payload_start], payload_len);

    if (msg.text_len > 0)
        emit_message(d, &msg);
}

static void decode_les_list(stdc_decoder_t *d, const uint8_t *frame,
                             int pos, int pkt_len) {
    /* 0xAB: LES List */
    if (pkt_len < 6) return;

    stdc_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STDC_MSG_LES_LIST;
    msg.descriptor = 0xAB;
    msg.crc_ok = 1;
    msg.has_position = 0;
    msg.lat = NAN;
    msg.lon = NAN;

    int station_count = frame[pos + 3];
    int tlen = 0;
    tlen += snprintf(msg.text + tlen, sizeof(msg.text) - tlen,
                     "LES List (%d stations):", station_count);

    int j = pos + 4;
    for (int i = 0; i < station_count && j + 5 < pos + pkt_len - 2; i++) {
        int sat = (frame[j] >> 6) & 0x03;
        int lid = frame[j] & 0x3F;
        double dl = ((frame[j + 4] << 8 | frame[j + 5]) - 8000) * 0.0025 + 1530.5;
        tlen += snprintf(msg.text + tlen, sizeof(msg.text) - tlen,
                         " %s/%s@%.4f",
                         get_sat_name(sat), get_les_name(sat, lid), dl);
        j += 6;
    }
    msg.text[tlen] = '\0';
    msg.text_len = tlen;

    emit_message(d, &msg);
}

/* ---- EGC packet decoders (0xB1, 0xB2) ---- */

static void decode_egc(stdc_decoder_t *d, const uint8_t *frame,
                        int pos, int pkt_len, int is_part2) {
    /* 0xB1/0xB2: EGC double header part 1 / part 2 */
    if (pkt_len < 10) return;

    int msg_type = frame[pos + 2];          /* service code / address type */
    int continuation = (frame[pos + 3] >> 7) & 1;
    int priority = (frame[pos + 3] >> 5) & 3;
    int repetition = frame[pos + 3] & 0x1F;
    int msg_id = (frame[pos + 4] << 8) | frame[pos + 5];
    int pkt_no = frame[pos + 6];
    int presentation = frame[pos + 7];

    int addr_len = get_address_length(msg_type);
    int hdr_size = 8 + addr_len;
    if (hdr_size > pkt_len - 2) hdr_size = pkt_len - 2;

    int payload_start = pos + hdr_size;
    int payload_len = pkt_len - 2 - hdr_size;
    if (payload_len < 0) payload_len = 0;

    /* Find or create reassembly slot */
    egc_reassembly_t *slot = find_egc_slot(d, msg_id);
    slot->service_code = msg_type;
    slot->priority = priority;
    slot->repetition = repetition;
    slot->presentation = presentation;
    slot->last_seen = time(NULL);

    /* Store this part's payload.
     * Part index: pkt_no*2 + is_part2 gives ordering */
    int part_idx = (pkt_no - 1) * 2 + is_part2;
    if (part_idx < 0) part_idx = 0;
    if (part_idx >= 64) part_idx = 63;

    int off = part_idx * 128;
    if (payload_len > 128) payload_len = 128;
    if (off + payload_len <= (int)sizeof(slot->parts)) {
        memcpy(slot->parts + off, &frame[payload_start], payload_len);
        slot->part_lens[part_idx] = payload_len;
    }

    if (part_idx > slot->max_pkt_no)
        slot->max_pkt_no = part_idx;

    if (!continuation && is_part2)
        slot->complete = 1;

    /* Also try emitting as a standalone message for immediate display */
    if (payload_len > 0) {
        stdc_message_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.type = is_part2 ? STDC_MSG_EGC_DOUBLE_2 : STDC_MSG_EGC_DOUBLE_1;
        msg.descriptor = is_part2 ? 0xB2 : 0xB1;
        msg.service_code = msg_type;
        msg.priority = priority;
        msg.repetition = repetition;
        msg.message_id = msg_id;
        msg.packet_no = pkt_no;
        msg.continuation = continuation;
        msg.presentation = presentation;
        msg.crc_ok = 1;
        msg.has_position = 0;
        msg.lat = NAN;
        msg.lon = NAN;

        if (presentation == 0) {
            msg.text_len = extract_ia5_text(msg.text, sizeof(msg.text),
                                            &frame[payload_start], payload_len);
        } else {
            int n = payload_len;
            if (n >= (int)sizeof(msg.text)) n = sizeof(msg.text) - 1;
            memcpy(msg.text, &frame[payload_start], n);
            msg.text[n] = '\0';
            msg.text_len = n;
        }

        if (msg.text_len > 0)
            emit_message(d, &msg);
    }

    /* If reassembly complete, emit the full message */
    if (slot->complete)
        try_emit_egc(d, slot);
}

/* ---- Parse packets from a single decoded frame ---- */

static void parse_packets(stdc_decoder_t *d, const uint8_t *frame,
                           int frame_len);

static void decode_multiframe_start(stdc_decoder_t *d, const uint8_t *frame,
                                     int pos, int pkt_len) {
    /* 0xBD: Multiframe Packet Start */
    if (pkt_len < 5) return;

    multiframe_state_t *mfp = &d->mfp;

    /* The encapsulated packet descriptor starts at pos+2 */
    uint8_t inner_desc = frame[pos + 2];
    int inner_len;
    if ((inner_desc >> 7) == 0) {
        inner_len = (inner_desc & 0x0F) + 1;
    } else if ((inner_desc >> 6) == 0x02) {
        inner_len = frame[pos + 3] + 2;
    } else {
        inner_len = 256;
    }

    mfp->active = 1;
    mfp->total_len = inner_len;
    mfp->filled = 0;
    memset(mfp->data, 0, sizeof(mfp->data));

    /* Copy first chunk: from pos+2 to end of packet minus CRC */
    int first_chunk = pkt_len - 2 - 2; /* minus header(2) minus CRC(2)... */
    /* Actually: payload starts at pos+2, CRC is last 2 bytes */
    first_chunk = pkt_len - 2 - 2;
    if (first_chunk < 0) first_chunk = 0;
    if (first_chunk > (int)sizeof(mfp->data)) first_chunk = sizeof(mfp->data);

    memcpy(mfp->data, &frame[pos + 2], first_chunk);
    mfp->filled = first_chunk;
}

static void decode_multiframe_cont(stdc_decoder_t *d, const uint8_t *frame,
                                    int pos, int pkt_len) {
    /* 0xBE: Multiframe Packet Continue */
    multiframe_state_t *mfp = &d->mfp;
    if (!mfp->active) return;

    /* Payload starts at pos+2, CRC is last 2 bytes */
    int payload_len = pkt_len - 2 - 2;
    if (payload_len < 0) payload_len = 0;

    int space = (int)sizeof(mfp->data) - mfp->filled;
    if (payload_len > space) payload_len = space;

    memcpy(mfp->data + mfp->filled, &frame[pos + 2], payload_len);
    mfp->filled += payload_len;

    /* Check if we've collected the full encapsulated packet (minus its CRC) */
    if (mfp->filled >= mfp->total_len - 2) {
        /* Recursively parse the encapsulated packet */
        parse_packets(d, mfp->data, mfp->total_len);
        mfp->active = 0;
    }
}

/* ---- Main packet parser ---- */

static void parse_packets(stdc_decoder_t *d, const uint8_t *frame,
                           int frame_len) {
    int pos = 0;

    while (pos < frame_len - 2) {
        uint8_t desc = frame[pos];

        /* End of data marker */
        if (desc == 0x00)
            break;

        /* Skip padding bytes */
        if (desc == 0xFF) {
            pos++;
            continue;
        }

        /* Get packet length */
        int pkt_len = get_packet_length(frame, pos, frame_len);
        if (pkt_len <= 0 || pos + pkt_len > frame_len)
            break;

        /* CRC check */
        int pkt_crc = (frame[pos + pkt_len - 2] << 8) | frame[pos + pkt_len - 1];
        int calc_crc = compute_crc(frame, pos, pkt_len);
        int crc_ok = (pkt_crc == 0) || (pkt_crc == calc_crc);

        if (crc_ok)
            atomic_fetch_add(&stat_stdc_crc_ok, 1);
        else {
            atomic_fetch_add(&stat_stdc_crc_fail, 1);
            pos += pkt_len;
            continue;
        }

        /* Dispatch by packet descriptor */
        switch (desc) {
        case STDC_PKT_ACK_REQUEST:
            decode_ack_request(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_CHANNEL_CLEAR:
            decode_channel_clear(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_MSG_ACK:
            decode_msg_ack(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_SIGNALLING_CH:
            decode_signalling(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_BULLETIN_BOARD:
            decode_bulletin_board(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_ANNOUNCEMENT:
            decode_announcement(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_CHAN_ASSIGNMENT:
            decode_chan_assignment(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_LOGIN_ACK:
            decode_login_ack(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_INDIVIDUAL_POLL:
            decode_individual_poll(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_CONFIRMATION:
            decode_confirmation(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_MESSAGE:
            decode_message(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_LES_LIST:
            decode_les_list(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_EGC_DOUBLE_1:
            decode_egc(d, frame, pos, pkt_len, 0);
            break;
        case STDC_PKT_EGC_DOUBLE_2:
            decode_egc(d, frame, pos, pkt_len, 1);
            break;
        case STDC_PKT_MULTIFRAME_START:
            decode_multiframe_start(d, frame, pos, pkt_len);
            break;
        case STDC_PKT_MULTIFRAME_CONT:
            decode_multiframe_cont(d, frame, pos, pkt_len);
            break;

        /* Known but not decoded yet -- skip silently */
        case STDC_PKT_DISTRESS_ACK:
        case STDC_PKT_DATA_REPORT_ACK:
        case STDC_PKT_DISTRESS_TEST:
        case STDC_PKT_REQUEST_STATUS:
        case STDC_PKT_TEST_RESULT:
            break;

        default:
            /* Unknown descriptor -- skip */
            break;
        }

        pos += pkt_len;
    }
}

/* Process a complete frame from the shift register */
static void process_frame(stdc_decoder_t *d, int inverted) {
    int8_t *buf = d->shift_reg;

    /* Invert if needed */
    if (inverted) {
        for (int i = 0; i < STDC_ENCODED_SIZE; i++)
            buf[i] = -buf[i];
    }

    /* Depermute rows */
    depermute(buf, d->depermuted);

    /* Deinterleave */
    deinterleave(d->depermuted, d->deinterleaved);

    /* Viterbi decode */
    viterbi_t vit;
    int decoded_bits = STDC_DATA_SIZE / 2;  /* rate 1/2 */
    viterbi_init(&vit, decoded_bits);
    viterbi_decode(&vit, d->deinterleaved, d->decoded);
    viterbi_free(&vit);

    float ber = viterbi_ber(d->decoded, decoded_bits, d->deinterleaved);
    atomic_fetch_add(&stat_stdc_ber_sum, (unsigned long)(ber * 10000));
    atomic_fetch_add(&stat_stdc_ber_count, 1);
    if (ber > 0.15f) {
        /* Beyond what rate 1/2 k=7 can correct */
        return;
    }

    /* Descramble */
    descramble(d->decoded);

    atomic_fetch_add(&stat_stdc_frames, 1);
    d->frame_count++;

    /* Parse packets in the decoded frame */
    parse_packets(d, d->decoded, STDC_FRAME_BYTES);
}

void stdc_decoder_feed(stdc_decoder_t *d, const float *soft_bits,
                        int num_bits) {
    for (int i = 0; i < num_bits; i++) {
        /* Convert float soft bit to int8 */
        float v = soft_bits[i] * 127.0f;
        if (v > 127.0f) v = 127.0f;
        if (v < -127.0f) v = -127.0f;
        int8_t sb = (int8_t)v;

        /* Shift into register */
        memmove(d->shift_reg, d->shift_reg + 1, STDC_ENCODED_SIZE - 1);
        d->shift_reg[STDC_ENCODED_SIZE - 1] = sb;
        d->shift_pos++;

        /* Check for frame sync periodically */
        if (d->synced) {
            /* Already synced -- process at expected frame boundary */
            if (d->shift_pos >= STDC_ENCODED_SIZE) {
                int inv = 0;
                int score = check_sync(d->shift_reg, &inv);
                if (score >= 100) {  /* good sync */
                    process_frame(d, inv);
                    d->shift_pos = 0;
                } else {
                    /* Lost sync */
                    d->synced = 0;
                    atomic_store(&stat_stdc_synced, 0);
                }
            }
        } else {
            /* Searching for sync */
            if (d->shift_pos >= STDC_ENCODED_SIZE) {
                int inv = 0;
                int score = check_sync(d->shift_reg, &inv);
                if (score >= 110) {  /* require strong initial sync */
                    d->synced = 1;
                    atomic_store(&stat_stdc_synced, 1);
                    process_frame(d, inv);
                    d->shift_pos = 0;
                }
            }
        }
    }
}

void stdc_decoder_destroy(stdc_decoder_t *d) {
    free(d);
}
