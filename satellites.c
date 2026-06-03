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

/* I4-F3 (98W, AORW/AMER) -- Americas region
 * Validated against 98W references using an Airspy R2 and a commercial L-Band dish */
static const channel_def_t channels_4f3[] = {
    /* STD-C EGC (NCS common) -- channel 0 by convention */
    { 1537700000.0, CHAN_STDC_EGC,   0 },   /* AORW, NCS,        Laurentides CA */

    /* Aero-L 600 baud P-channels / Psmc */
    { 1545020000.0, CHAN_AERO_600,  1 },    /* AORW, SITA,       Laurentides CA */
    { 1545050000.0, CHAN_AERO_600,  2 },    /* AORW, SITA,       Laurentides CA */
    { 1545060000.0, CHAN_AERO_600,  3 },    /* AMER, ARINC/SITA, Paumalu HI     */
    { 1545065000.0, CHAN_AERO_600,  4 },    /* AMER, ARINC/SITA, Paumalu HI     */
    { 1545080000.0, CHAN_AERO_600,  5 },    /* AORW, SITA,       Laurentides CA */
    { 1545085000.0, CHAN_AERO_600,  6 },    /* AMER, ARINC/SITA, Paumalu HI     */
    { 1545090000.0, CHAN_AERO_600,  7 },    /* AORW, SITA,       Laurentides CA */
    { 1545100000.0, CHAN_AERO_600,  8 },    /* AORW, ARINC,      Laurentides CA */
    { 1545110000.0, CHAN_AERO_600,  9 },    /* AMER, ARINC/SITA, Paumalu HI     */
    { 1545170000.0, CHAN_AERO_600, 10 },    /* AORW, ARINC,      Laurentides CA */
    { 1545175000.0, CHAN_AERO_600, 11 },    /* AORW, ARINC,      Laurentides CA */
    { 1545205000.0, CHAN_AERO_600, 12 },    /* AORW, ARINC,      Laurentides CA */

    /* Aero-L 1200 baud */
    { 1545075000.0, CHAN_AERO_1200, 13 },   /* AMER, ARINC/SITA, Paumalu HI     */

    /* Aero-H+ 10500 baud */
    { 1546005000.0, CHAN_AERO_10500, 14 },  /* AMER, ARINC/SITA, Paumalu HI     */
    { 1546020000.0, CHAN_AERO_10500, 15 },  /* AMER, ARINC/SITA, Paumalu HI     */
    { 1546062500.0, CHAN_AERO_10500, 16 },  /* AORW, ARINC/SITA, Laurentides CA */
    { 1546077500.0, CHAN_AERO_10500, 17 },  /* AORW, ARINC/SITA, Laurentides CA */

    /* Aero 8400 baud C-channel (AMBE voice + data) */
    { 1542937500.0, CHAN_AERO_8400, 18 },   /* AMER, ARINC/SITA, Paumalu HI     */
    { 1542942500.0, CHAN_AERO_8400, 19 },   /* AMER, ARINC/SITA, Paumalu HI     */
    { 1542947500.0, CHAN_AERO_8400, 20 },   /* AMER, ARINC/SITA, Paumalu HI     */
    { 1542952500.0, CHAN_AERO_8400, 21 },   /* AMER, ARINC/SITA, Paumalu HI     */
    { 1542957500.0, CHAN_AERO_8400, 22 },   /* AMER, ARINC/SITA, Paumalu HI     */
    { 1542977500.0, CHAN_AERO_8400, 23 },   /* AORW, ARINC/SITA, Laurentides CA */
    { 1542982500.0, CHAN_AERO_8400, 24 },   /* AORW, ARINC/SITA, Laurentides CA */
    { 1542987500.0, CHAN_AERO_8400, 25 },   /* AORW, ARINC/SITA, Laurentides CA */
    { 1542992500.0, CHAN_AERO_8400, 26 },   /* AORW, ARINC/SITA, Laurentides CA */
};

/* I-3 F5 (54W, AORE) -- Atlantic East
 * Frequencies from the authoritative INMARSAT L-band frequency list.
 * Global-beam only — spot-beam 8400 regions (N-America, Mexico, Europe,
 * Brazil, S-America, Africa) are listed in the reference but serve
 * different coverage areas and can be added later if needed. */
static const channel_def_t channels_3f5[] = {
    /* STD-C EGC NCS (Burum NL) */
    { 1541450000.0, CHAN_STDC_EGC,   0 },

    /* Aero-L 600 P-channels (Burum NL) */
    { 1545025000.0, CHAN_AERO_600,  1 },
    { 1545030000.0, CHAN_AERO_600,  2 },
    { 1545035000.0, CHAN_AERO_600,  3 },
    { 1545040000.0, CHAN_AERO_600,  4 },

    /* Aero-H+ 10500 P-channels */
    { 1546055000.0, CHAN_AERO_10500, 5 },
    { 1546070000.0, CHAN_AERO_10500, 6 },

    /* Aero 8400 AMBE C-channels, global beam */
    { 1542975000.0, CHAN_AERO_8400,  7 },
    { 1542980000.0, CHAN_AERO_8400,  8 },
    { 1542985000.0, CHAN_AERO_8400,  9 },
};

/* Alphasat / Inmarsat-4A F4 (25E, EMEA/IOR)
 * Frequencies from the authoritative INMARSAT L-band frequency list PDF,
 * independently verified live by lmb56 in Denmark (issue #13) using
 * AirSpy R2. Provider is ARINC & SITA; ground station Fucino, IT. */
static const channel_def_t channels_af1[] = {
    /* STD-C EGC NCS */
    { 1537100000.0, CHAN_STDC_EGC,   0 },

    /* Aero-L 600 P-channels (Fucino IT) — ch12 is the PSMC1 management
     * channel; report from #21 (lmb56) confirms it carries 600-baud P-ch. */
    { 1545115000.0, CHAN_AERO_600, 12 },
    { 1545120000.0, CHAN_AERO_600,  1 },
    { 1545130000.0, CHAN_AERO_600,  2 },

    /* Aero-L 1200 P-channel */
    { 1545125000.0, CHAN_AERO_1200, 3 },

    /* Aero-H+ 10500 P-channels */
    { 1546012500.0, CHAN_AERO_10500, 4 },
    { 1546027500.0, CHAN_AERO_10500, 5 },

    /* Aero 8400 AMBE C-channels, global beam */
    { 1546142500.0, CHAN_AERO_8400,  6 },
    { 1546147500.0, CHAN_AERO_8400,  7 },
    { 1546152500.0, CHAN_AERO_8400,  8 },
    { 1546182500.0, CHAN_AERO_8400,  9 },
    { 1546187500.0, CHAN_AERO_8400, 10 },
    { 1546192500.0, CHAN_AERO_8400, 11 },
};

/* I-6 F1 (83.5E, IOE) -- Indian Ocean East
 * Frequencies from the authoritative INMARSAT L-band frequency list PDF.
 * NOTE: our previous "F1" config claimed position 143.5°E (POR) but the
 * PDF has no Inmarsat satellite at that position. I-6 F1 at 83.5°E is
 * what's operational. Provider: ARINC & SITA; ground station Perth, AU.
 * STD-C NCS is not documented on I-6 F1 in the reference.
 * Untested on-air. */
static const channel_def_t channels_f1[] = {
    /* Aero-L 600 P-channels (Perth AU) */
    { 1545160000.0, CHAN_AERO_600,  1 },
    { 1545165000.0, CHAN_AERO_600,  2 },
    { 1545185000.0, CHAN_AERO_600,  3 },
    { 1545215000.0, CHAN_AERO_600,  4 },
    { 1545220000.0, CHAN_AERO_600,  5 },
    { 1545225000.0, CHAN_AERO_600,  6 },

    /* Aero-H+ 10500 P-channels */
    { 1546042500.0, CHAN_AERO_10500, 7 },
    { 1546092500.0, CHAN_AERO_10500, 8 },
    { 1546107500.0, CHAN_AERO_10500, 9 },
    { 1546122500.0, CHAN_AERO_10500, 10 },

    /* Aero 8400 AMBE C-channels, global beam */
    { 1546157500.0, CHAN_AERO_8400, 11 },
    { 1546162500.0, CHAN_AERO_8400, 12 },
    { 1546167500.0, CHAN_AERO_8400, 13 },
    { 1546172500.0, CHAN_AERO_8400, 14 },
    { 1546177500.0, CHAN_AERO_8400, 15 },
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
        .freq_max = 1546070000.0,
        /* RTL-SDR: 1.92 MHz covers the wider 3F5 span on RTL-SDR's clean rates. */
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
        .freq_max = 1546192500.0,
        /* RTL-SDR/SDRplay rates from SDRReceiver 25E configs (unchanged —
         * the aero cluster span is similar enough to 4F3). */
        .preferred_rate_rtl     = 1536000.0,
        .preferred_rate_sdrplay = 3072000.0,
        .preferred_rate_hackrf  = 6000000.0,
    },
    {
        .name = "I-6 F1",
        .designator = "F1",
        .position = 83.5,
        .region = "IOE",
        /* STD-C NCS is not documented on I-6 F1 in the reference PDF;
         * may be absent on this satellite. 0 = no STD-C in aero-only mode. */
        .stdc_egc_freq = 0.0,
        .channels = channels_f1,
        .num_channels = ARRAY_SIZE(channels_f1),
        .freq_min = 1545160000.0,
        .freq_max = 1546177500.0,
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
        { "AF4",      "AF1" },  /* canonical — Inmarsat designates as I-4A F4 */
        { "4AF4",     "AF1" },
        { "alphasat", "AF1" },
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
