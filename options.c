/*
 * Command-line option parsing
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <err.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "inmarsat.h"
#include "sdr.h"
#include "satellites.h"

#ifdef HAVE_SOAPYSDR
#include "soapysdr.h"
#endif

#ifdef HAVE_SDRPLAY
#include "sdrplay.h"
#endif

#ifdef HAVE_RTLSDR
#include "rtlsdr.h"
#endif

extern double samp_rate;
extern double center_freq;
extern int verbose;
extern int live;
extern iq_format_t iq_format;
extern FILE *in_file;
extern op_mode_t op_mode;
extern int skip_c_channel;
extern double oqpsk_lockingbw;
extern char *satellite_name;

#ifdef HAVE_SOAPYSDR
extern int soapy_num;
extern char *soapy_args;
#define SOAPY_SETTINGS_MAX 8
extern char *soapy_setting_keys[SOAPY_SETTINGS_MAX];
extern char *soapy_setting_vals[SOAPY_SETTINGS_MAX];
extern int soapy_setting_count;
#define SOAPY_GAINS_MAX 8
extern char *soapy_gain_elem_names[SOAPY_GAINS_MAX];
extern double soapy_gain_elem_vals[SOAPY_GAINS_MAX];
extern int soapy_gain_elem_count;
#endif

extern double soapy_gain_val;
extern int bias_tee;
extern double ppm_correction;

#ifdef HAVE_SDRPLAY
extern char *sdrplay_serial;
extern int sdrplay_gain_val;
#endif
#ifdef HAVE_RTLSDR
extern int rtl_dev_index;
#endif
extern int vita49_enabled;
extern char *vita49_endpoint;
extern int web_enabled;
#ifdef HAVE_ZMQ
extern int zmq_enabled;
extern int zmq_base_port;
extern int iq_zmq_enabled;
extern int iq_zmq_port;
#endif
extern int web_port;
extern int feed_enabled;
extern int jaero_format_enabled;
extern char *jaero_format_host;
extern int jaero_format_port;
extern int agc_enabled;
#define UDP_MAX 4
extern char *udp_hosts[UDP_MAX];
extern int udp_ports[UDP_MAX];
extern int udp_count;
extern int basestation_enabled;
extern char *basestation_endpoint;
extern char *aircraft_db_path;
extern int update_db_flag;
extern char *station_id;
#ifdef HAVE_HACKRF
extern char *hackrf_serial;
extern int hackrf_lna_gain;
extern int hackrf_vga_gain;
extern int hackrf_amp_enable;
#endif
#ifdef HAVE_BLADERF
extern int bladerf_num;
extern int bladerf_gain_val;
#endif
#ifdef HAVE_UHD
extern char *usrp_serial;
extern int usrp_gain_val;
#endif
extern int mqtt_enabled;
extern char *mqtt_host;
extern int mqtt_port;
extern char *mqtt_user;
extern char *mqtt_pass;
extern char *mqtt_topic;

static void usage(int exitcode) {
    fprintf(stderr,
"Usage: inmarsat-sniffer <-f FILE | -i IFACE | --vita49> [options]\n"
"Inmarsat L-band decoder. Decodes STD-C (EGC) and Aero (ACARS)\n"
"simultaneously from a single SDR.\n"
"\n"
"Input (one required):\n"
"    -f, --file=FILE         read IQ samples from file\n"
"    -l, --live              capture live from SDR (implied by -i)\n"
"    --vita49[=IP:PORT]      receive IQ via VITA 49 (VRT) UDP (default: 0.0.0.0:4991)\n"
"    --format=FMT            IQ format: ci8 (default), cu8 (rtl_sdr), ci16, cf32\n"
"\n"
"SDR options:\n"
"    -i, --interface=IFACE   SDR to use (see --list for available devices):\n"
"                             rtl-N, soapy-N, soapy:driver=X, sdrplay[-SERIAL],\n"
"                             hackrf[-SERIAL], bladerfN, usrp-PRODUCT-SERIAL,\n"
"                             airspy[-HEXSN]\n"
"    -r, --sample-rate=HZ    sample rate in Hz (default: auto from satellite)\n"
"    -B, --bias-tee          enable bias tee power\n"
"    -p, --ppm=N.NN          SDR frequency error in PPM. RTL-SDR tunes its\n"
"                             TCXO; other backends apply it as a software\n"
"                             shift at the channelizer. Disables auto-cal.\n"
"    --agc                   enable RTL-SDR AGC mode\n"
"    --soapy-gain=GAIN       SoapySDR overall gain in dB (default: 40)\n"
"    --soapy-gain-element=NAME:VAL  per-element gain (repeatable)\n"
"    --soapy-setting=K:V     SoapySDR device setting (repeatable)\n"
"    --hackrf-lna=DB         HackRF LNA gain 0-40 in 8dB steps (default: 40)\n"
"    --hackrf-vga=DB         HackRF VGA gain 0-62 in 2dB steps (default: 20)\n"
"    --hackrf-amp            enable HackRF 14dB RF amplifier\n"
"    --bladerf-gain=DB       BladeRF RX gain in dB (default: 40)\n"
"    --usrp-gain=DB          USRP RX gain in dB (default: 40)\n"
"    --airspy-gain=N         Airspy native linearity_gain 0-21 (default: 19).\n"
"                             Overrides --soapy-gain on Airspy.\n"
"\n"
"Satellite:\n"
"    --satellite=SAT         satellite designator: 4F3, 3F5, AF1, F1\n"
"                             (required for live, determines frequencies)\n"
"    --mode=MODE             operating mode: aero (default), stdc, full, auto\n"
"                             stdc/full are experimental — STD-C decode not yet\n"
"                             verified on-air. auto picks based on SDR bandwidth.\n"
"    --skip-c-channel        skip OQPSK 8400 C-channel demods (voice/data bursts,\n"
"                             rarely carry ACARS; saves ~50%% CPU on low-power hosts)\n"
"    --oqpsk-lockingbw=HZ    AFC search/pull-in bandwidth for OQPSK 10500 (default 10500).\n"
"                             Try 20000-30000 if you see SDRReceiver+JAERO locking carriers\n"
"                             that we don't (means drifted from nominal channel centres).\n"
"                             Wider = more carriers caught but may lock onto spurs.\n"
"\n"
"Output:\n"
"    --web[=PORT]            enable live web dashboard (default port: 8888)\n"
"    --spectrum              also show the Spectrum tab (waterfall + constellation,\n"
"                             click-to-tune). Implies --web.\n"
"    --feed                  output JSON lines to stdout\n"
"    --jaero-format[=HOST:PORT] JAERO text format 3 (stderr, or UDP if endpoint given)\n"
"    --iq-zmq[=PORT]         publish raw tuned IQ as ZMQ cf32 (default port 5555)\n"
"    --udp=HOST:PORT         send JSON messages via UDP (repeatable, max 4)\n"
"    --basestation[=ENDPOINT] SBS (MSG,3) aircraft feed — PORT for server\n"
"                             (default 30003), HOST:PORT to push to remote\n"
"    --mqtt=HOST[:PORT]      publish ACARS JSON to MQTT broker (default port 1883)\n"
"    --mqtt-user=USER        MQTT authentication username\n"
"    --mqtt-pass=PASS        MQTT authentication password\n"
"    --mqtt-topic=TOPIC      MQTT topic (default: inmarsat-sniffer/acars)\n"
"    --station-id=ID         station identifier for JSON feed output\n"
"    --aircraft-db=PATH      tar1090-db aircraft.csv (for reg→ICAO hex)\n"
"    --update-db             download/refresh aircraft DB and exit\n"
"    -v, --verbose           verbose output to stderr\n"
"    -h, --help              show this help\n"
"    --list                  list available SDR interfaces\n"
"    --list-satellites       list known satellite channel tables\n"
"\n"
"Examples:\n"
"    inmarsat-sniffer -i soapy-0 --satellite=4F3\n"
"    inmarsat-sniffer -i soapy-0 --satellite=4F3 --web --feed\n"
"    inmarsat-sniffer -f recording.cf32 --format=cf32 --satellite=4F3\n"
"    inmarsat-sniffer --vita49 --format=cf32 -r 2400000 --satellite=4F3\n"
    );
    exit(exitcode);
}

static void list_interfaces(void) {
    printf("Available SDR interfaces (-i VALUE):\n");
#ifdef HAVE_RTLSDR
    rtlsdr_backend_list();
#endif
#ifdef HAVE_HACKRF
    {
        extern void hackrf_backend_list(void);
        hackrf_backend_list();
    }
#endif
#ifdef HAVE_BLADERF
    {
        extern void bladerf_backend_list(void);
        bladerf_backend_list();
    }
#endif
#ifdef HAVE_UHD
    {
        extern void usrp_backend_list(void);
        usrp_backend_list();
    }
#endif
#ifdef HAVE_AIRSPY
    {
        extern void airspy_backend_list(void);
        airspy_backend_list();
    }
#endif
#ifdef HAVE_SDRPLAY
    sdrplay_list();
#endif
#ifdef HAVE_SOAPYSDR
    soapy_list();
#endif
    fflush(stdout);
    _exit(0);
}

void parse_options(int argc, char **argv) {
    int ch;
    int format_explicit = 0;
    const char *in_filename = NULL;

    enum {
        OPT_FORMAT = 0x100,
        OPT_LIST,
        OPT_LIST_SATS,
        OPT_SATELLITE,
        OPT_MODE,
        OPT_WEB,
        OPT_FEED,
        OPT_UDP,
        OPT_SOAPY_GAIN,
        OPT_SOAPY_GAIN_ELEM,
        OPT_SOAPY_SETTING,
        OPT_SDRPLAY_GAIN,
        OPT_VITA49,
        OPT_ZMQ,
        OPT_IQ_ZMQ,
        OPT_BASESTATION,
        OPT_AIRCRAFT_DB,
        OPT_UPDATE_DB,
        OPT_STATION_ID,
        OPT_HACKRF_LNA,
        OPT_HACKRF_VGA,
        OPT_HACKRF_AMP,
        OPT_BLADERF_GAIN,
        OPT_USRP_GAIN,
        OPT_AIRSPY_GAIN,
        OPT_MQTT,
        OPT_MQTT_USER,
        OPT_MQTT_PASS,
        OPT_MQTT_TOPIC,
        OPT_JAERO_FORMAT,
        OPT_AGC,
        OPT_SKIP_C_CHANNEL,
        OPT_OQPSK_LOCKINGBW,
        OPT_SPECTRUM,
    };

    static const struct option longopts[] = {
        { "file",               required_argument, NULL, 'f' },
        { "live",               no_argument,       NULL, 'l' },
        { "interface",          required_argument, NULL, 'i' },
        { "sample-rate",        required_argument, NULL, 'r' },
        { "center-freq",        required_argument, NULL, 'c' },
        { "bias-tee",           no_argument,       NULL, 'B' },
        { "format",             required_argument, NULL, OPT_FORMAT },
        { "verbose",            no_argument,       NULL, 'v' },
        { "help",               no_argument,       NULL, 'h' },
        { "list",               no_argument,       NULL, OPT_LIST },
        { "list-satellites",    no_argument,       NULL, OPT_LIST_SATS },
        { "satellite",          required_argument, NULL, OPT_SATELLITE },
        { "mode",               required_argument, NULL, OPT_MODE },
        { "web",                optional_argument, NULL, OPT_WEB },
        { "spectrum",           no_argument,       NULL, OPT_SPECTRUM },
        { "feed",               no_argument,       NULL, OPT_FEED },
        { "zmq",                optional_argument, NULL, OPT_ZMQ },
        { "iq-zmq",             optional_argument, NULL, OPT_IQ_ZMQ },
        { "udp",                required_argument, NULL, OPT_UDP },
        { "soapy-gain",         required_argument, NULL, OPT_SOAPY_GAIN },
        { "soapy-gain-element", required_argument, NULL, OPT_SOAPY_GAIN_ELEM },
        { "soapy-setting",      required_argument, NULL, OPT_SOAPY_SETTING },
        { "sdrplay-gain",       required_argument, NULL, OPT_SDRPLAY_GAIN },
        { "vita49",             optional_argument, NULL, OPT_VITA49 },
        { "basestation",        optional_argument, NULL, OPT_BASESTATION },
        { "aircraft-db",        required_argument, NULL, OPT_AIRCRAFT_DB },
        { "update-db",          no_argument,       NULL, OPT_UPDATE_DB },
        { "station-id",         required_argument, NULL, OPT_STATION_ID },
#ifdef HAVE_HACKRF
        { "hackrf-lna",         required_argument, NULL, OPT_HACKRF_LNA },
        { "hackrf-vga",         required_argument, NULL, OPT_HACKRF_VGA },
        { "hackrf-amp",         no_argument,       NULL, OPT_HACKRF_AMP },
#endif
#ifdef HAVE_BLADERF
        { "bladerf-gain",       required_argument, NULL, OPT_BLADERF_GAIN },
#endif
#ifdef HAVE_UHD
        { "usrp-gain",          required_argument, NULL, OPT_USRP_GAIN },
        { "airspy-gain",        required_argument, NULL, OPT_AIRSPY_GAIN },
#endif
        { "mqtt",               required_argument, NULL, OPT_MQTT },
        { "mqtt-user",          required_argument, NULL, OPT_MQTT_USER },
        { "mqtt-pass",          required_argument, NULL, OPT_MQTT_PASS },
        { "mqtt-topic",         required_argument, NULL, OPT_MQTT_TOPIC },
        { "jaero-format",       optional_argument, NULL, OPT_JAERO_FORMAT },
        { "agc",                no_argument,       NULL, OPT_AGC },
        { "skip-c-channel",     no_argument,       NULL, OPT_SKIP_C_CHANNEL },
        { "oqpsk-lockingbw",    required_argument, NULL, OPT_OQPSK_LOCKINGBW },
        { "ppm",                required_argument, NULL, 'p' },
        { NULL, 0, NULL, 0 },
    };

    while ((ch = getopt_long(argc, argv, "f:li:r:c:p:Bvh", longopts, NULL)) != -1) {
        switch (ch) {
        case 'f':
            in_filename = optarg;
            break;

        case 'l':
            live = 1;
            break;

        case 'i':
            live = 1;
#ifdef HAVE_RTLSDR
            if (strncmp(optarg, "rtl-", 4) == 0) {
                rtl_dev_index = atoi(optarg + 4);
                break;
            }
#endif
#ifdef HAVE_HACKRF
            if (strncmp(optarg, "hackrf-", 7) == 0) {
                hackrf_serial = strdup(optarg + 7);
                break;
            } else if (strcmp(optarg, "hackrf") == 0) {
                hackrf_serial = strdup("");
                break;
            }
#endif
#ifdef HAVE_BLADERF
            if (strncmp(optarg, "bladerf", 7) == 0) {
                bladerf_num = atoi(optarg + 7);
                break;
            }
#endif
#ifdef HAVE_UHD
            if (strncmp(optarg, "usrp-", 5) == 0) {
                extern char *usrp_get_serial(const char *);
                usrp_serial = strdup(usrp_get_serial(optarg));
                break;
            }
#endif
#ifdef HAVE_AIRSPY
            if (strncmp(optarg, "airspy-", 7) == 0) {
                extern uint64_t airspy_serial;
                extern int airspy_selected;
                airspy_serial = strtoull(optarg + 7, NULL, 16);
                airspy_selected = 1;
                break;
            } else if (strcmp(optarg, "airspy") == 0) {
                extern int airspy_selected;
                airspy_selected = 1;
                break;
            }
#endif
#ifdef HAVE_SDRPLAY
            if (strncmp(optarg, "sdrplay-", 8) == 0) {
                sdrplay_serial = strdup(optarg + 8);
                break;
            } else if (strcmp(optarg, "sdrplay") == 0) {
                sdrplay_serial = strdup("");
                break;
            }
#endif
#ifdef HAVE_SOAPYSDR
            if (strncmp(optarg, "soapy:", 6) == 0) {
                soapy_args = strdup(optarg + 6);
            } else if (strncmp(optarg, "soapy-", 6) == 0) {
                soapy_num = atoi(optarg + 6);
            } else {
                errx(1, "Unknown interface: %s (use rtl-N, hackrf[-SERIAL], bladerfN, usrp-PRODUCT-SERIAL, airspy[-HEXSN], soapy-N, soapy:args, or sdrplay[-SERIAL])", optarg);
            }
#else
#ifndef HAVE_SDRPLAY
#ifndef HAVE_RTLSDR
            errx(1, "No SDR backend compiled in");
#endif
#endif
#endif
            break;

        case 'r':
            /* Strip a leading '=' so both "-r=10000000" and "-r 10000000" work.
             * Otherwise optarg="=10000000" -> atof() returns 0 and forces auto */
            samp_rate = atof(optarg[0] == '=' ? optarg + 1 : optarg);
            break;

        case 'c':
            center_freq = strtod(optarg, NULL);
            break;

        case 'p':
            ppm_correction = strtod(optarg, NULL);
            break;

        case 'B':
            bias_tee = 1;
            break;

        case OPT_FORMAT:
            format_explicit = 1;
            if (strcasecmp(optarg, "ci8") == 0 || strcasecmp(optarg, "cs8") == 0)
                iq_format = FMT_CI8;
            else if (strcasecmp(optarg, "cu8") == 0)
                iq_format = FMT_CU8;
            else if (strcasecmp(optarg, "ci16") == 0 || strcasecmp(optarg, "cs16") == 0)
                iq_format = FMT_CI16;
            else if (strcasecmp(optarg, "cf32") == 0)
                iq_format = FMT_CF32;
            else
                errx(1, "Unknown format: %s (use ci8, cu8, ci16, cf32)", optarg);
            break;

        case 'v':
            verbose = 1;
            break;

        case 'h':
            usage(0);
            break;

        case OPT_LIST:
            list_interfaces();
            break;

        case OPT_LIST_SATS:
            satellite_list();
            exit(0);
            break;

        case OPT_SATELLITE:
            satellite_name = strdup(optarg);
            break;

        case OPT_MODE:
            if (strcasecmp(optarg, "auto") == 0)
                op_mode = MODE_AUTO;
            else if (strcasecmp(optarg, "aero") == 0)
                op_mode = MODE_AERO;
            else if (strcasecmp(optarg, "stdc") == 0)
                op_mode = MODE_STDC;
            else if (strcasecmp(optarg, "full") == 0)
                op_mode = MODE_FULL;
            else
                errx(1, "Unknown mode: %s (use auto, aero, stdc, full)", optarg);
            break;

        case OPT_WEB:
            web_enabled = 1;
            if (optarg)
                web_port = atoi(optarg);
            break;

        case OPT_SPECTRUM:
            /* Implies --web: the Spectrum tab lives in the web UI. */
            web_enabled = 1;
            {
                extern int spectrum_enabled;
                spectrum_enabled = 1;
            }
            break;

        case OPT_ZMQ:
#ifdef HAVE_ZMQ
            zmq_enabled = 1;
            if (optarg)
                zmq_base_port = atoi(optarg);
#else
            errx(1, "ZMQ support not compiled in (install libzmq-dev)");
#endif
            break;

        case OPT_IQ_ZMQ:
#ifdef HAVE_ZMQ
            iq_zmq_enabled = 1;
            if (optarg)
                iq_zmq_port = atoi(optarg);
#else
            errx(1, "ZMQ support not compiled in (install libzmq-dev)");
#endif
            break;

        case OPT_FEED:
            feed_enabled = 1;
            break;

        case OPT_BASESTATION:
            basestation_enabled = 1;
            if (optarg)
                basestation_endpoint = strdup(optarg);
            break;

        case OPT_AIRCRAFT_DB:
            aircraft_db_path = strdup(optarg);
            break;

        case OPT_UPDATE_DB:
            update_db_flag = 1;
            break;

        case OPT_STATION_ID:
            station_id = strdup(optarg);
            break;

#ifdef HAVE_HACKRF
        case OPT_HACKRF_LNA:
            hackrf_lna_gain = atoi(optarg);
            break;

        case OPT_HACKRF_VGA:
            hackrf_vga_gain = atoi(optarg);
            break;

        case OPT_HACKRF_AMP:
            hackrf_amp_enable = 1;
            break;
#endif

#ifdef HAVE_BLADERF
        case OPT_BLADERF_GAIN:
            bladerf_gain_val = atoi(optarg);
            break;
#endif

#ifdef HAVE_UHD
        case OPT_USRP_GAIN:
            usrp_gain_val = atoi(optarg);
            break;
#endif

#ifdef HAVE_AIRSPY
        case OPT_AIRSPY_GAIN: {
            extern int airspy_gain_val;
            int g = atoi(optarg);
            if (g < 0)  g = 0;
            if (g > 21) g = 21;
            airspy_gain_val = g;
            break;
        }
#endif

        case OPT_MQTT: {
#ifdef HAVE_MQTT
            mqtt_enabled = 1;
            char *ep = strdup(optarg);
            char *colon = strrchr(ep, ':');
            if (colon) {
                *colon = '\0';
                mqtt_host = strdup(ep);
                mqtt_port = atoi(colon + 1);
            } else {
                mqtt_host = strdup(ep);
                mqtt_port = 1883;
            }
            free(ep);
#else
            errx(1, "MQTT support not compiled in (install libmosquitto-dev)");
#endif
            break;
        }
        case OPT_MQTT_USER:
            mqtt_user = strdup(optarg);
            break;
        case OPT_MQTT_PASS:
            mqtt_pass = strdup(optarg);
            break;
        case OPT_MQTT_TOPIC:
            mqtt_topic = strdup(optarg);
            break;

        case OPT_JAERO_FORMAT: {
            jaero_format_enabled = 1;
            if (optarg) {
                char *ep = strdup(optarg);
                char *colon = strrchr(ep, ':');
                if (colon) {
                    *colon = '\0';
                    jaero_format_host = strdup(ep);
                    jaero_format_port = atoi(colon + 1);
                } else {
                    jaero_format_host = strdup("127.0.0.1");
                    jaero_format_port = atoi(ep);
                }
                free(ep);
            }
            /* No argument = stderr output only */
            break;
        }

        case OPT_AGC:
            agc_enabled = 1;
            break;

        case OPT_SKIP_C_CHANNEL:
            skip_c_channel = 1;
            break;

        case OPT_OQPSK_LOCKINGBW:
            oqpsk_lockingbw = atof(optarg);
            if (oqpsk_lockingbw < 1000 || oqpsk_lockingbw > 40000)
                errx(1, "--oqpsk-lockingbw must be between 1000 and 40000 Hz");
            break;

        case OPT_UDP: {
            if (udp_count >= UDP_MAX)
                errx(1, "Too many --udp endpoints (max %d)", UDP_MAX);
            char *colon = strrchr(optarg, ':');
            if (!colon)
                errx(1, "Invalid --udp format, use HOST:PORT");
            *colon = '\0';
            udp_hosts[udp_count] = strdup(optarg);
            udp_ports[udp_count] = atoi(colon + 1);
            if (udp_ports[udp_count] < 1 || udp_ports[udp_count] > 65535)
                errx(1, "Invalid port: %s", colon + 1);
            udp_count++;
            break;
        }

        case OPT_SOAPY_GAIN: {
            extern int soapy_gain_explicit;
            soapy_gain_val = atof(optarg);
            soapy_gain_explicit = 1;
            break;
        }

#ifdef HAVE_SDRPLAY
        case OPT_SDRPLAY_GAIN:
            sdrplay_gain_val = atoi(optarg);
            break;
#endif

#ifdef HAVE_SOAPYSDR
        case OPT_SOAPY_GAIN_ELEM: {
            if (soapy_gain_elem_count >= SOAPY_GAINS_MAX)
                errx(1, "Too many --soapy-gain-element (max %d)", SOAPY_GAINS_MAX);
            char *sep = strchr(optarg, ':');
            if (!sep)
                errx(1, "Invalid --soapy-gain-element format, use NAME:VALUE");
            *sep = '\0';
            soapy_gain_elem_names[soapy_gain_elem_count] = strdup(optarg);
            soapy_gain_elem_vals[soapy_gain_elem_count] = atof(sep + 1);
            soapy_gain_elem_count++;
            break;
        }

        case OPT_SOAPY_SETTING: {
            if (soapy_setting_count >= SOAPY_SETTINGS_MAX)
                errx(1, "Too many --soapy-setting (max %d)", SOAPY_SETTINGS_MAX);
            char *sep = strchr(optarg, ':');
            if (!sep)
                errx(1, "Invalid --soapy-setting format, use KEY:VALUE");
            *sep = '\0';
            soapy_setting_keys[soapy_setting_count] = strdup(optarg);
            soapy_setting_vals[soapy_setting_count] = strdup(sep + 1);
            soapy_setting_count++;
            break;
        }
#endif

        case OPT_VITA49:
            vita49_enabled = 1;
            if (optarg)
                vita49_endpoint = strdup(optarg);
            break;

        default:
            usage(1);
            break;
        }
    }

    /* Validate inputs — but --update-db is a standalone action. */
    if (update_db_flag)
        return;
    if (!in_filename && !live && !vita49_enabled)
        errx(1, "No input specified. Use -f FILE, -i IFACE, or --vita49");

    if (vita49_enabled && (live || in_filename))
        errx(1, "--vita49 cannot be combined with -f or -i");

    if (in_filename && live)
        errx(1, "Cannot use both -f and -i");

    if (live && !satellite_name) {
#ifdef HAVE_ZMQ
        if (!(iq_zmq_enabled && center_freq > 0 && samp_rate > 0))
#endif
            errx(1, "--satellite is required for live capture unless "
                    "--iq-zmq is used with -c and -r");
    }

#ifdef HAVE_ZMQ
    if (iq_zmq_enabled && samp_rate <= 0 && !satellite_name)
        errx(1, "--iq-zmq without --satellite needs -r/--sample-rate");
#endif

    /* Open input file */
    if (in_filename) {
        if (strcmp(in_filename, "-") == 0) {
            in_file = stdin;
        } else {
            in_file = fopen(in_filename, "rb");
            if (!in_file)
                err(1, "Cannot open %s", in_filename);

            /* Auto-detect format from extension */
            if (!format_explicit) {
                const char *ext = strrchr(in_filename, '.');
                if (ext) {
                    if (strcasecmp(ext, ".cf32") == 0 || strcasecmp(ext, ".fc32") == 0)
                        iq_format = FMT_CF32;
                    else if (strcasecmp(ext, ".ci16") == 0 || strcasecmp(ext, ".cs16") == 0)
                        iq_format = FMT_CI16;
                    else if (strcasecmp(ext, ".cu8") == 0)
                        iq_format = FMT_CU8;
                    else if (strcasecmp(ext, ".ci8") == 0 || strcasecmp(ext, ".cs8") == 0)
                        iq_format = FMT_CI8;
                }
            }
        }
    }
}
