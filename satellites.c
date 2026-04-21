/*
 * Built-in satellite channel frequency tables
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "satellites.h"

/*
 * Channel tables for known Inmarsat satellites.
 * Frequencies are from publicly available L-band surveys.
 * These are relatively static -- Inmarsat rarely changes allocations.
 *
 * Aero channel frequencies for 4F3 (98W, AORW) are well-documented.
 * Other satellites have partial tables -- will be filled in as data
 * becomes available.
 */

/* I4-F3 (98W, AORW) -- Americas region
 * Frequencies from SDRReceiver 98W config (verified against live signals). */
static const channel_def_t channels_4f3[] = {
    /* STD-C EGC (NCS common) -- channel 0 by convention */
    { 1537700000.0, CHAN_STDC_EGC,   0 },

    /* Aero 600 baud channels */
    { 1545021000.0, CHAN_AERO_600,  1 },
    { 1545051000.0, CHAN_AERO_600,  2 },
    { 1545061000.0, CHAN_AERO_600,  3 },
    { 1545066000.0, CHAN_AERO_600,  4 },
    { 1545081000.0, CHAN_AERO_600,  5 },
    { 1545086000.0, CHAN_AERO_600,  6 },
    { 1545091000.0, CHAN_AERO_600,  7 },
    { 1545101000.0, CHAN_AERO_600,  8 },
    { 1545111000.0, CHAN_AERO_600,  9 },
    { 1545171000.0, CHAN_AERO_600, 10 },
    { 1545176000.0, CHAN_AERO_600, 11 },

    /* Aero 1200 baud channels */
    { 1545076000.0, CHAN_AERO_1200, 12 },

    /* Aero 10500 baud channels */
    { 1545995000.0, CHAN_AERO_10500, 13 },
    { 1546010000.0, CHAN_AERO_10500, 14 },
    { 1546055000.0, CHAN_AERO_10500, 15 },
    { 1546070000.0, CHAN_AERO_10500, 16 },

    /* Aero 8400 baud (C-channel, voice + data) */
    { 1546135300.0, CHAN_AERO_8400, 17 },
    { 1546140500.0, CHAN_AERO_8400, 18 },
    { 1546145700.0, CHAN_AERO_8400, 19 },
    { 1546150300.0, CHAN_AERO_8400, 20 },
    { 1546155500.0, CHAN_AERO_8400, 21 },
    { 1546160600.0, CHAN_AERO_8400, 22 },
    { 1546166300.0, CHAN_AERO_8400, 23 },
    { 1546171430.0, CHAN_AERO_8400, 24 },
    { 1546176430.0, CHAN_AERO_8400, 25 },
    { 1546181430.0, CHAN_AERO_8400, 26 },
    { 1546186430.0, CHAN_AERO_8400, 27 },
};

/* I3-F5 (54W, AORE) -- Atlantic East
 * Frequencies from SDRReceiver 54W config. */
static const channel_def_t channels_3f5[] = {
    { 1541450000.0, CHAN_STDC_EGC,   0 },

    /* Aero 600 baud channels */
    { 1545014429.0, CHAN_AERO_600,  1 },
    { 1545029412.0, CHAN_AERO_600,  2 },
    { 1545134635.0, CHAN_AERO_600,  3 },
    { 1545194731.0, CHAN_AERO_600,  4 },

    /* Aero 10500 baud channels */
    { 1546045422.0, CHAN_AERO_10500, 5 },
    { 1546061717.0, CHAN_AERO_10500, 6 },

    /* Aero 8400 baud channels */
    { 1546817935.0, CHAN_AERO_8400,  7 },
    { 1546823426.0, CHAN_AERO_8400,  8 },
    { 1546828110.0, CHAN_AERO_8400,  9 },
    { 1546833112.0, CHAN_AERO_8400, 10 },
    { 1546838105.0, CHAN_AERO_8400, 11 },
    { 1546842770.0, CHAN_AERO_8400, 12 },
    { 1546848155.0, CHAN_AERO_8400, 13 },
    { 1546853237.0, CHAN_AERO_8400, 14 },
};

/* Alphasat / Inmarsat-4A F4 (25E, EMEA/IOR)
 * Aero frequencies from SDRReceiver 25E config.
 * STD-C EGC from thebaldgeek.github.io/stdc.html (1537.10 MHz). */
static const channel_def_t channels_af1[] = {
    { 1537100000.0, CHAN_STDC_EGC,   0 },

    /* Aero 600 baud channels */
    { 1545005146.0, CHAN_AERO_600,  1 },
    { 1545214573.0, CHAN_AERO_600,  2 },
    { 1545219706.0, CHAN_AERO_600,  3 },
    { 1545224996.0, CHAN_AERO_600,  4 },
    { 1545114134.0, CHAN_AERO_600,  5 },
    { 1545119063.0, CHAN_AERO_600,  6 },
    { 1545129563.0, CHAN_AERO_600,  7 },
    { 1545159288.0, CHAN_AERO_600,  8 },
    { 1545164682.0, CHAN_AERO_600,  9 },
    { 1545183905.0, CHAN_AERO_600, 10 },
    { 1545189244.0, CHAN_AERO_600, 11 },

    /* Aero 1200 baud channels */
    { 1545124261.0, CHAN_AERO_1200, 12 },

    /* Aero 10500 baud channels */
    { 1546005300.0, CHAN_AERO_10500, 13 },
    { 1546019800.0, CHAN_AERO_10500, 14 },
    { 1546034700.0, CHAN_AERO_10500, 15 },
    { 1546084600.0, CHAN_AERO_10500, 16 },
    { 1546099900.0, CHAN_AERO_10500, 17 },
    { 1546114200.0, CHAN_AERO_10500, 18 },

    /* Aero 8400 baud channels */
    { 1546137300.0, CHAN_AERO_8400, 19 },
    { 1546142500.0, CHAN_AERO_8400, 20 },
    { 1546147700.0, CHAN_AERO_8400, 21 },
    { 1546152300.0, CHAN_AERO_8400, 22 },
    { 1546157500.0, CHAN_AERO_8400, 23 },
    { 1546162600.0, CHAN_AERO_8400, 24 },
    { 1546168300.0, CHAN_AERO_8400, 25 },
    { 1546173430.0, CHAN_AERO_8400, 26 },
    { 1546178430.0, CHAN_AERO_8400, 27 },
};

/* I4-AF2 (84E, MOR/IOR-East) -- Middle East / Asia-Pacific fringe
 * Frequencies from SDRReceiver 84E config. No STD-C EGC confirmed --
 * placeholder freq used; verify against live signal. */
static const channel_def_t channels_af2[] = {
    /* STD-C EGC -- UNVERIFIED placeholder, confirm before use */
    { 1537950000.0, CHAN_STDC_EGC,   0 },

    /* Aero 600 baud channels */
    { 1545005000.0, CHAN_AERO_600,  1 },
    { 1545080000.0, CHAN_AERO_600,  2 },
    { 1545085000.0, CHAN_AERO_600,  3 },
    { 1545090000.0, CHAN_AERO_600,  4 },
    { 1545160000.0, CHAN_AERO_600,  5 },
    { 1545165000.0, CHAN_AERO_600,  6 },
    { 154518500.0, CHAN_AERO_600,  7 },
    { 1545190000.0, CHAN_AERO_600,  8 },

    /* Aero 10500 baud channels */
    { 1546045000.0, CHAN_AERO_10500,  9 },
    { 1546095000.0, CHAN_AERO_10500, 10 },
    { 1546110000.0, CHAN_AERO_10500, 11 },
    { 1546125000.0, CHAN_AERO_10500, 12 },
};

/* I4-F1 (143.5E, POR) -- Pacific Ocean
 * Frequencies from live-measured pipeline config (inmarsat_aero_6/12/105).
 * Updated from round-number placeholders; three additional 600 baud channels
 * (1545160300, 1545165300, 1545200350) and two additional 10500 baud channels
 * (1546050000, 1546085000) added vs previous table. */
static const channel_def_t channels_f1[] = {
    { 1541450000.0, CHAN_STDC_EGC,   0 },

    /* Aero 600 baud channels */
    { 1545005200.0, CHAN_AERO_600,  1 },
    { 1545025170.0, CHAN_AERO_600,  2 },
    { 1545030100.0, CHAN_AERO_600,  3 },
    { 1545035135.0, CHAN_AERO_600,  4 },
    { 1545040200.0, CHAN_AERO_600,  5 },
    { 1545045100.0, CHAN_AERO_600,  6 },
    { 1545095200.0, CHAN_AERO_600,  7 },
    { 1545135200.0, CHAN_AERO_600,  8 },
    { 1545140380.0, CHAN_AERO_600,  9 },
    { 1545145360.0, CHAN_AERO_600, 10 },
    { 1545150360.0, CHAN_AERO_600, 11 },
    { 1545155300.0, CHAN_AERO_600, 12 },
    { 1545160300.0, CHAN_AERO_600, 13 },
    { 1545165300.0, CHAN_AERO_600, 14 },
    { 1545180440.0, CHAN_AERO_600, 15 },
    { 1545200350.0, CHAN_AERO_600, 16 },
    { 1545210200.0, CHAN_AERO_600, 17 },

    /* Aero 1200 baud channels */
    { 1545070250.0, CHAN_AERO_1200, 18 },

    /* Aero 10500 baud channels */
    { 1546005000.0, CHAN_AERO_10500, 19 },
    { 1546035000.0, CHAN_AERO_10500, 20 },
    { 1546050000.0, CHAN_AERO_10500, 21 },
    { 1546070000.0, CHAN_AERO_10500, 22 },
    { 1546085000.0, CHAN_AERO_10500, 23 },
};

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static const satellite_t satellites[] = {
    {
        .name = "I4-F3",
        .designator = "4F3",
        .position = -98.0,
        .region = "AORW",
        /* STD-C NCS is on a different sub-band (~1537 MHz). --mode=full
         * covers both; Aero-only modes ignore this. */
        .stdc_egc_freq = 1537700000.0,
        .channels = channels_4f3,
        .num_channels = ARRAY_SIZE(channels_4f3),
        .freq_min = 1537700000.0,
        .freq_max = 1546186430.0,
        /* RTL-SDR: SDRReceiver sdr_98W.ini; proven live here.
         * SDRplay:  extrapolated from the AF1 SDRplay config (same Inmarsat
         *           generation and L-band span); proven live here.
         * HackRF:   proven live here. */
        .preferred_rate_rtl     = 1536000.0,
        .preferred_rate_sdrplay = 3072000.0,
        .preferred_rate_hackrf  = 6000000.0,
    },
    {
        .name = "I3-F5",
        .designator = "3F5",
        .position = -54.0,
        .region = "AORE",
        .stdc_egc_freq = 1541450000.0,
        .channels = channels_3f5,
        .num_channels = ARRAY_SIZE(channels_3f5),
        .freq_min = 1541450000.0,
        .freq_max = 1546853237.0,
        /* RTL-SDR: SDRReceiver sdr_54W_all.ini — 1.92 MHz covers the wider
         *          3F5 span (1.84 MHz) on RTL-SDR's native-clean rate ladder. */
        .preferred_rate_rtl     = 1920000.0,
        .preferred_rate_sdrplay = 3072000.0,
        .preferred_rate_hackrf  = 6000000.0,
    },
    {
        .name = "Alphasat (I-4A F4)",
        .designator = "AF1",
        .position = 25.0,
        .region = "EMEA",
        .stdc_egc_freq = 1537100000.0,
        .channels = channels_af1,
        .num_channels = ARRAY_SIZE(channels_af1),
        .freq_min = 1537100000.0,
        .freq_max = 1546178430.0,
        /* RTL-SDR: SDRReceiver sdr_25E.ini.
         * SDRplay:  SDRReceiver sdr_25E_sdrplay.ini. */
        .preferred_rate_rtl     = 1536000.0,
        .preferred_rate_sdrplay = 3072000.0,
        .preferred_rate_hackrf  = 6000000.0,
    },
    {
        .name = "I4-AF2",
        .designator = "AF2",
        .position = 84.0,
        .region = "MOR",
        .stdc_egc_freq = 1537950000.0,   /* UNVERIFIED placeholder */
        .channels = channels_af2,
        .num_channels = ARRAY_SIZE(channels_af2),
        .freq_min = 1537950000.0,
        .freq_max = 1546123000.0,
    },
    {
        .name = "I4-F1",
        .designator = "F1",
        .position = 143.5,
        .region = "POR",
        .stdc_egc_freq = 1541450000.0,
        .channels = channels_f1,
        .num_channels = ARRAY_SIZE(channels_f1),
        .freq_min = 1541450000.0,
        .freq_max = 1546085000.0,
    },
};

const satellite_t *satellite_lookup(const char *designator) {
    /* Primary designator match */
    for (size_t i = 0; i < ARRAY_SIZE(satellites); i++) {
        if (strcasecmp(satellites[i].designator, designator) == 0)
            return &satellites[i];
    }
    /* Geographic position + common-name aliases */
    struct { const char *alias; const char *designator; } aliases[] = {
        { "98W",      "4F3" },  /* Inmarsat 4-F3 Americas */
        { "54W",      "3F5" },  /* Inmarsat 3-F5 Atlantic */
        { "25E",      "AF1" },  /* Alphasat / I-4A F4 EMEA */
        { "AF4",      "AF1" },  /* canonical -- Inmarsat designates as I-4A F4 */
        { "4AF4",     "AF1" },
        { "alphasat", "AF1" },
        { "84E",      "AF2" },  /* Inmarsat 4-AF2 Middle East */
        { "143E",     "F1"  },  /* Inmarsat 4-F1 Pacific */
        { "143.5E",   "F1"  },
        { "4F1",      "F1"  },
    };
    for (size_t i = 0; i < ARRAY_SIZE(aliases); i++) {
        if (strcasecmp(aliases[i].alias, designator) == 0) {
            for (size_t j = 0; j < ARRAY_SIZE(satellites); j++) {
                if (strcasecmp(satellites[j].designator, aliases[i].designator) == 0)
                    return &satellites[j];
            }
        }
    }
    return NULL;
}

void satellite_list(void) {
    fprintf(stderr, "Known Inmarsat satellites (--satellite=DESIGNATOR):\n\n");
    fprintf(stderr, "  %-12s %-8s %-6s %-8s %s\n",
            "Satellite", "Pos", "Region", "Channels", "STD-C EGC");
    fprintf(stderr, "  %-12s %-8s %-6s %-8s %s\n",
            "---------", "---", "------", "--------", "---------");

    for (size_t i = 0; i < ARRAY_SIZE(satellites); i++) {
        const satellite_t *s = &satellites[i];
        int aero_count = 0;
        int stdc_count = 0;
        for (int j = 0; j < s->num_channels; j++) {
            if (s->channels[j].type == CHAN_STDC_EGC)
                stdc_count++;
            else
                aero_count++;
        }
        fprintf(stderr, "  %-12s %+.1f%s  %-6s %d aero   %.3f MHz\n",
                s->name, s->position,
                s->position < 0 ? "W" : "E",
                s->region, aero_count,
                s->stdc_egc_freq / 1e6);
    }
}
