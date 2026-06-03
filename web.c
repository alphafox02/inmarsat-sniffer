/*
 * Embedded web dashboard with live map
 *
 * Serves a Leaflet-based map showing decoded STD-C and Aero messages.
 * Uses Server-Sent Events (SSE) for live updates.
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include "inmarsat.h"
#include <time.h>
#include <unistd.h>

#include "web.h"

extern volatile sig_atomic_t running;

/* ---- Ring buffer storage ---- */

#define MAX_STDC_MSGS   200
#define MAX_AIRCRAFT    256
#define MAX_FIXES       8

typedef struct {
    double timestamp;
    double lat, lon;
    int has_position;
    int service_code;
    int msg_type;
    char text[512];
    int text_len;
} stdc_entry_t;

typedef struct {
    char reg[16];
    char flight[16];
    char label[4];
    double fix_lat[MAX_FIXES];
    double fix_lon[MAX_FIXES];
    int fix_alt[MAX_FIXES];
    double fix_t[MAX_FIXES];
    int n_fixes;
    double last_seen;
    char last_text[256];
    int channel_id;
} aircraft_entry_t;

static struct {
    pthread_mutex_t lock;

    stdc_entry_t stdc[MAX_STDC_MSGS];
    int stdc_head;
    int stdc_count;

    aircraft_entry_t aircraft[MAX_AIRCRAFT];
    int num_aircraft;

    /* Cumulative ACARS messages since startup (exposed via JSON). */
    unsigned long total_acars;
} state;

/* ---- Time ---- */

static double now_unix(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* ---- State update functions ---- */

void web_add_stdc(const stdc_message_t *msg) {
    pthread_mutex_lock(&state.lock);

    stdc_entry_t *e = &state.stdc[state.stdc_head];
    e->timestamp = now_unix();
    e->lat = msg->lat;
    e->lon = msg->lon;
    e->has_position = msg->has_position;
    e->service_code = msg->service_code;
    e->msg_type = msg->type;

    int tlen = msg->text_len;
    if (tlen > (int)sizeof(e->text) - 1)
        tlen = (int)sizeof(e->text) - 1;
    memcpy(e->text, msg->text, tlen);
    e->text[tlen] = '\0';
    e->text_len = tlen;

    state.stdc_head = (state.stdc_head + 1) % MAX_STDC_MSGS;
    if (state.stdc_count < MAX_STDC_MSGS)
        state.stdc_count++;

    pthread_mutex_unlock(&state.lock);
}

void web_add_aero(const aero_message_t *msg) {
    pthread_mutex_lock(&state.lock);

    state.total_acars++;
    double now = now_unix();

    /* Find existing aircraft by registration */
    aircraft_entry_t *ac = NULL;
    for (int i = 0; i < state.num_aircraft; i++) {
        if (strcmp(state.aircraft[i].reg, msg->reg) == 0) {
            ac = &state.aircraft[i];
            break;
        }
    }

    if (!ac) {
        /* Create new entry */
        if (state.num_aircraft < MAX_AIRCRAFT) {
            ac = &state.aircraft[state.num_aircraft++];
            memset(ac, 0, sizeof(*ac));
            strncpy(ac->reg, msg->reg, sizeof(ac->reg) - 1);
        } else {
            /* Evict oldest */
            double oldest_t = 1e18;
            int oldest_i = 0;
            for (int i = 0; i < state.num_aircraft; i++) {
                if (state.aircraft[i].last_seen < oldest_t) {
                    oldest_t = state.aircraft[i].last_seen;
                    oldest_i = i;
                }
            }
            ac = &state.aircraft[oldest_i];
            memset(ac, 0, sizeof(*ac));
            strncpy(ac->reg, msg->reg, sizeof(ac->reg) - 1);
        }
    }

    strncpy(ac->flight, msg->flight, sizeof(ac->flight) - 1);
    strncpy(ac->label, msg->label, sizeof(ac->label) - 1);
    ac->last_seen = now;
    ac->channel_id = msg->channel_id;

    if (msg->text_len > 0) {
        int tlen = msg->text_len;
        if (tlen > (int)sizeof(ac->last_text) - 1)
            tlen = (int)sizeof(ac->last_text) - 1;
        memcpy(ac->last_text, msg->text, tlen);
        ac->last_text[tlen] = '\0';
    }

    /* Add position fix if available */
    if (msg->has_position && !isnan(msg->lat) && !isnan(msg->lon)) {
        if (ac->n_fixes >= MAX_FIXES) {
            memmove(ac->fix_lat, ac->fix_lat + 1, (MAX_FIXES - 1) * sizeof(double));
            memmove(ac->fix_lon, ac->fix_lon + 1, (MAX_FIXES - 1) * sizeof(double));
            memmove(ac->fix_alt, ac->fix_alt + 1, (MAX_FIXES - 1) * sizeof(int));
            memmove(ac->fix_t, ac->fix_t + 1, (MAX_FIXES - 1) * sizeof(double));
            ac->n_fixes = MAX_FIXES - 1;
        }
        ac->fix_lat[ac->n_fixes] = msg->lat;
        ac->fix_lon[ac->n_fixes] = msg->lon;
        ac->fix_alt[ac->n_fixes] = msg->alt_ft;
        ac->fix_t[ac->n_fixes] = now;
        ac->n_fixes++;
    }

    pthread_mutex_unlock(&state.lock);
}

/* ---- JSON serialization ---- */

static int json_escape_str(char *out, int maxlen, const char *in, int inlen) {
    int pos = 0;
    for (int i = 0; i < inlen && pos < maxlen - 6; i++) {
        char c = in[i];
        switch (c) {
        case '"':  out[pos++] = '\\'; out[pos++] = '"'; break;
        case '\\': out[pos++] = '\\'; out[pos++] = '\\'; break;
        case '\n': out[pos++] = '\\'; out[pos++] = 'n'; break;
        case '\r': out[pos++] = '\\'; out[pos++] = 'r'; break;
        case '\t': out[pos++] = '\\'; out[pos++] = 't'; break;
        default:
            if (c >= 0x20)
                out[pos++] = c;
            break;
        }
    }
    out[pos] = '\0';
    return pos;
}

#define JSON_BUF_SIZE 524288  /* 512 KB — room for 512 aircraft + 200 STD-C */

static int build_json(char *buf, int maxlen) {
    pthread_mutex_lock(&state.lock);

    int pos = 0;
    /* Expose op_mode so the JS can hide the STD-C tab in aero-only mode. */
    extern op_mode_t op_mode;
    extern int spectrum_enabled;
    int stdc_enabled = (op_mode != MODE_AERO);
    extern unsigned long feed_get_json_drops(void);
    pos += snprintf(buf + pos, maxlen - pos,
        "{\"t\":%.3f,\"stdc_enabled\":%s,\"spectrum_enabled\":%s,"
        "\"total_acars\":%lu,\"feed_drops\":%lu,",
        now_unix(), stdc_enabled ? "true" : "false",
        spectrum_enabled ? "true" : "false",
        state.total_acars, feed_get_json_drops());

    /* STD-C messages */
    pos += snprintf(buf + pos, maxlen - pos, "\"stdc\":[");
    int first = 1;
    for (int i = 0; i < state.stdc_count && pos < maxlen - 1024; i++) {
        int idx = (state.stdc_head - state.stdc_count + i + MAX_STDC_MSGS) % MAX_STDC_MSGS;
        stdc_entry_t *e = &state.stdc[idx];

        char escaped[1024];
        int elen = e->text_len;
        if (elen > 400) elen = 400;
        json_escape_str(escaped, sizeof(escaped), e->text, elen);

        if (!first) buf[pos++] = ',';
        first = 0;

        if (e->has_position) {
            pos += snprintf(buf + pos, maxlen - pos,
                "{\"t\":%.3f,\"svc\":%d,\"type\":%d,"
                "\"lat\":%.6f,\"lon\":%.6f,\"text\":\"%s\"}",
                e->timestamp, e->service_code, e->msg_type,
                e->lat, e->lon, escaped);
        } else {
            pos += snprintf(buf + pos, maxlen - pos,
                "{\"t\":%.3f,\"svc\":%d,\"type\":%d,\"text\":\"%s\"}",
                e->timestamp, e->service_code, e->msg_type, escaped);
        }
    }
    pos += snprintf(buf + pos, maxlen - pos, "],");

    /* Aircraft — only send entries seen in the last 10 minutes */
    double cutoff = now_unix() - 600.0;
    pos += snprintf(buf + pos, maxlen - pos, "\"aircraft\":[");
    first = 1;
    for (int i = 0; i < state.num_aircraft && pos < maxlen - 2048; i++) {
        aircraft_entry_t *ac = &state.aircraft[i];
        if (ac->last_seen < cutoff) continue;
        if (!first) buf[pos++] = ',';
        first = 0;

        char escaped_text[512];
        json_escape_str(escaped_text, sizeof(escaped_text),
                         ac->last_text, (int)strlen(ac->last_text));

        pos += snprintf(buf + pos, maxlen - pos,
            "{\"reg\":\"%s\",\"flight\":\"%s\",\"label\":\"%s\","
            "\"last_seen\":%.3f,\"ch\":%d,\"text\":\"%s\",\"fixes\":[",
            ac->reg, ac->flight, ac->label,
            ac->last_seen, ac->channel_id, escaped_text);

        for (int j = 0; j < ac->n_fixes; j++) {
            if (j > 0) buf[pos++] = ',';
            pos += snprintf(buf + pos, maxlen - pos,
                "[%.6f,%.6f,%d,%.3f]",
                ac->fix_lat[j], ac->fix_lon[j],
                ac->fix_alt[j], ac->fix_t[j]);
        }

        pos += snprintf(buf + pos, maxlen - pos, "]}");
    }
    pos += snprintf(buf + pos, maxlen - pos, "],");

    /* Per-channel status (from jaero_chans in main.c) */
    {
        extern int num_jaero_chans;
        typedef struct {
            int channel_id;
            int baud_rate;
            unsigned long msg_count;
            unsigned long burst_count;
            double last_msg_time;
            unsigned long drops;
            double mse;
            double ebno;
            int is_locked;
        } chan_web_info_t;

        /* Gather channel info without holding main's lock */
        extern void web_get_channel_info(chan_web_info_t *out, int *n);
        chan_web_info_t chinfo[32];
        int nch = 0;
        web_get_channel_info(chinfo, &nch);

        pos += snprintf(buf + pos, maxlen - pos, "\"channels\":[");
        for (int i = 0; i < nch && pos < maxlen - 256; i++) {
            if (i > 0) buf[pos++] = ',';
            double age = now_unix() - chinfo[i].last_msg_time;
            if (chinfo[i].last_msg_time < 1) age = -1;
            pos += snprintf(buf + pos, maxlen - pos,
                "{\"ch\":%d,\"baud\":%d,\"msgs\":%lu,\"age\":%.0f,\"mse\":%.3f,\"ebno\":%.1f,\"lock\":%d}",
                chinfo[i].channel_id, chinfo[i].baud_rate,
                chinfo[i].msg_count, age, chinfo[i].mse, chinfo[i].ebno,
                chinfo[i].is_locked);
        }
        pos += snprintf(buf + pos, maxlen - pos, "]");
    }

    pos += snprintf(buf + pos, maxlen - pos, "}");

    pthread_mutex_unlock(&state.lock);
    return pos;
}

/* ---- Embedded HTML page ---- */

static const char HTML_PAGE[] =
"<!DOCTYPE html>\n"
"<html><head>\n"
"<meta charset=\"utf-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
"<title>inmarsat-sniffer</title>\n"
"<link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\">\n"
"<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>\n"
"<style>\n"
"*{margin:0;padding:0;box-sizing:border-box}\n"
"body{font-family:system-ui,-apple-system,sans-serif;background:#0f172a;color:#e2e8f0}\n"
"#map{width:100vw;height:calc(100vh - 44px)}\n"
"#bar{height:44px;background:#1e293b;color:#e2e8f0;display:flex;\n"
"  align-items:center;padding:0 16px;gap:20px;font-size:13px;\n"
"  border-bottom:1px solid #334155}\n"
"#bar .title{font-weight:600;color:#f8fafc;letter-spacing:0.5px}\n"
".stat{color:#94a3b8}\n"
".val{color:#38bdf8;font-weight:600;font-variant-numeric:tabular-nums}\n"
"#status{margin-left:auto;font-size:12px}\n"
".leaflet-popup-content-wrapper{max-width:480px}\n"
".leaflet-popup-content{font-family:'SF Mono',Consolas,monospace;\n"
"  font-size:12px;line-height:1.6;word-break:break-word;\n"
"  white-space:pre-wrap;margin:10px 12px;max-height:420px;\n"
"  overflow-y:auto}\n"
".popup-title{font-weight:700;font-size:13px;margin-bottom:4px;\n"
"  padding-bottom:4px;border-bottom:1px solid #e2e8f0}\n"
".popup-ac{color:#38bdf8;font-weight:600}\n"
".popup-egc{color:#4ade80;font-weight:600}\n"
".leaflet-container{background:#0f172a}\n"
".leaflet-control-layers{background:rgba(15,23,42,0.92)!important;\n"
"  color:#e2e8f0!important;border:1px solid #334155!important}\n"
".leaflet-control-layers label{color:#e2e8f0}\n"
"#side{position:absolute;right:0;top:44px;bottom:0;width:320px;\n"
"  background:rgba(15,23,42,0.92);border-left:1px solid #334155;\n"
"  font-size:11px;display:flex;flex-direction:column;\n"
"  font-family:'SF Mono',Consolas,monospace;color:#cbd5e1;z-index:400;\n"
"  transition:width 0.15s ease-out}\n"
"#side.wide{width:min(720px,75vw)}\n"
"#tabs{display:flex;border-bottom:1px solid #334155;flex-shrink:0}\n"
".tab{flex:1;padding:6px 0;text-align:center;font-size:10px;cursor:pointer;\n"
"  color:#64748b;text-transform:uppercase;letter-spacing:1px;font-weight:600;\n"
"  border-bottom:2px solid transparent}\n"
".tab.active{color:#38bdf8;border-bottom-color:#38bdf8}\n"
".tab:hover{color:#94a3b8}\n"
".tab-content{flex:1;overflow-y:auto;padding:8px;display:none}\n"
".tab-content.active{display:block}\n"
".msg{background:#1e293b;border-left:2px solid #38bdf8;margin:3px 0;\n"
"  padding:4px 6px;word-wrap:break-word;border-radius:2px}\n"
".msg.egc{border-color:#4ade80}\n"
".msg .hdr{color:#38bdf8;font-weight:600;font-size:11px}\n"
".msg .ts{color:#64748b;font-size:10px}\n"
".msg .txt{color:#cbd5e1;font-size:11px;margin-top:2px}\n"
"@media(max-width:900px){#side{display:none}#map{width:100vw}}\n"
/* ---- Light theme overrides (gated by html.light, dark stays default) ---- */
"html.light body{background:#f8fafc;color:#1e293b}\n"
"html.light #bar{background:#ffffff;color:#1e293b;border-bottom-color:#cbd5e1}\n"
"html.light #bar .title{color:#0f172a}\n"
"html.light .stat{color:#475569}\n"
"html.light .val{color:#0284c7}\n"
"html.light .leaflet-container{background:#f8fafc}\n"
"html.light .leaflet-control-layers{background:rgba(255,255,255,0.95)!important;\n"
"  color:#1e293b!important;border-color:#cbd5e1!important}\n"
"html.light .leaflet-control-layers label{color:#1e293b}\n"
"html.light #side{background:rgba(255,255,255,0.96);color:#1e293b;\n"
"  border-left-color:#cbd5e1}\n"
"html.light #tabs{border-bottom-color:#cbd5e1}\n"
"html.light .tab{color:#94a3b8}\n"
"html.light .tab.active{color:#0284c7;border-bottom-color:#0284c7}\n"
"html.light .tab:hover{color:#475569}\n"
"html.light .msg{background:#f1f5f9;border-left-color:#0284c7}\n"
"html.light .msg.egc{border-color:#16a34a}\n"
"html.light .msg .hdr{color:#0284c7}\n"
"html.light .msg .ts{color:#64748b}\n"
"html.light .msg .txt{color:#1e293b}\n"
"html.light .popup-ac{color:#0284c7}\n"
"html.light .popup-egc{color:#16a34a}\n"
"html.light .popup-title{border-bottom-color:#cbd5e1}\n"
"html.light #btn-export{background:#e2e8f0;color:#1e293b;border-color:#cbd5e1}\n"
"html.light #spec-ch{background:#ffffff;color:#1e293b;border-color:#cbd5e1}\n"
"html.light #spec-info{color:#475569}\n"
"html.light #spec-auto{background:#e2e8f0;color:#1e293b;border-color:#cbd5e1}\n"
"html.light #spec-canvas,html.light #const-canvas{background:#ffffff;border-color:#cbd5e1}\n"
"#theme-toggle{background:transparent;color:inherit;border:1px solid currentColor;\n"
"  border-radius:4px;padding:0 8px;cursor:pointer;font-size:14px;line-height:24px;\n"
"  height:26px;opacity:0.6}\n"
"#theme-toggle:hover{opacity:1}\n"
"</style></head><body>\n"
"<div id=\"bar\">\n"
"  <span class=\"title\">inmarsat-sniffer</span>\n"
"  <span class=\"stat\">Locked <span id=\"n-ac\" class=\"val\">0</span></span>\n"
"  <span class=\"stat\" title=\"Unique aircraft heard in the last 10 minutes (includes those without position reports)\">Aircraft (10m) <span id=\"n-aircraft\" class=\"val\">0</span></span>\n"
"  <span class=\"stat\">Positions <span id=\"n-pos\" class=\"val\">0</span></span>\n"
"  <span class=\"stat stdc-ui\">STD-C <span id=\"n-stdc\" class=\"val\">0</span></span>\n"
"  <button id=\"btn-export\" onclick=\"exportAircraft()\" style=\"background:#334155;color:#e2e8f0;border:1px solid #475569;border-radius:4px;padding:2px 8px;cursor:pointer;font-size:11px;margin-left:4px\">Export CSV</button>\n"
"  <button id=\"theme-toggle\" onclick=\"toggleTheme()\" title=\"Toggle light/dark\">\\u263E</button>\n"
"  <span id=\"status\" style=\"color:#64748b\">connecting...</span>\n"
"</div>\n"
"<div id=\"map\"></div>\n"
"<div id=\"side\">\n"
"  <div id=\"tabs\">\n"
"    <div class=\"tab active\" onclick=\"switchTab('acars')\">ACARS</div>\n"
"    <div class=\"tab stdc-ui\" onclick=\"switchTab('stdc')\">STD-C</div>\n"
"    <div class=\"tab\" onclick=\"switchTab('channels')\">Channels</div>\n"
"    <div class=\"tab spectrum-ui\" onclick=\"switchTab('spectrum')\">Spectrum</div>\n"
"  </div>\n"
"  <div id=\"tab-acars\" class=\"tab-content active\"><div id=\"aero-list\"></div></div>\n"
"  <div id=\"tab-stdc\" class=\"tab-content\"><div id=\"stdc-list\"></div></div>\n"
"  <div id=\"tab-channels\" class=\"tab-content\"><div id=\"ch-panel\" style=\"font-size:10px;line-height:1.6\"></div></div>\n"
"  <div id=\"tab-spectrum\" class=\"tab-content spectrum-ui\">\n"
"    <div style=\"display:flex;gap:8px;align-items:center;margin-bottom:8px;font-size:12px\">\n"
"      <label>Channel: <select id=\"spec-ch\" style=\"background:#0f172a;color:#e2e8f0;border:1px solid #334155;border-radius:3px;padding:2px\"></select></label>\n"
"      <span id=\"spec-info\" style=\"color:#94a3b8\"></span>\n"
"      <button id=\"spec-auto\" onclick=\"resetTune()\" style=\"background:#334155;color:#e2e8f0;border:1px solid #475569;border-radius:3px;padding:2px 8px;cursor:pointer;font-size:11px;margin-left:auto\">Auto (AFC)</button>\n"
"    </div>\n"
"    <div style=\"font-size:10px;color:#64748b;margin-bottom:4px\">Waterfall \\u2014 newest at top, time scrolls down. Click to retune (disables AFC). Triangle marks the current demod tune.</div>\n"
"    <canvas id=\"spec-canvas\" width=\"1024\" height=\"180\" style=\"width:100%;height:180px;background:#0b1220;border:1px solid #334155;border-radius:4px;cursor:crosshair;display:block\"></canvas>\n"
"    <div style=\"font-size:10px;color:#64748b;margin:10px 0 4px 0\">Constellation \\u2014 post-matched-filter I/Q points. Tight clusters = locked, smear = not.</div>\n"
"    <canvas id=\"const-canvas\" width=\"260\" height=\"260\" style=\"width:100%;max-width:260px;aspect-ratio:1/1;background:#0b1220;border:1px solid #334155;border-radius:4px;display:block;margin:0 auto\"></canvas>\n"
"  </div>\n"
"</div>\n"
"<script>"
"function switchTab(name){"
"  var tabs=document.querySelectorAll('.tab');"
"  var panes=document.querySelectorAll('.tab-content');"
"  tabs.forEach(function(t){t.classList.remove('active')});"
"  panes.forEach(function(p){p.classList.remove('active')});"
"  document.querySelector('.tab[onclick*=\"'+name+'\"]').classList.add('active');"
"  document.getElementById('tab-'+name).classList.add('active');"
/* Widen the side panel only while the Spectrum tab is active so narrow
 * MSK signals get enough horizontal pixels to be clickable. */
"  var side=document.getElementById('side');"
"  if(side){if(name==='spectrum')side.classList.add('wide');else side.classList.remove('wide')}"
"  if(name==='spectrum'){startSpectrum()}else{stopSpectrum()}"
"}\n"

/* Spectrum tab: waterfall (time × freq × dB) + constellation, 4 Hz poll. */
"var specTimer=null,specFs=0,specLastCh=null;"
"function startSpectrum(){"
"  if(specTimer)return;"
"  pollSpectrum();"
"  specTimer=setInterval(pollSpectrum,250);"  /* 4 Hz = 1 pixel row every 250 ms */
"}"
"function stopSpectrum(){"
"  if(specTimer){clearInterval(specTimer);specTimer=null}"
"}"
"function currentSpecCh(){"
"  var sel=document.getElementById('spec-ch');"
"  return sel&&sel.value?parseInt(sel.value,10):null;"
"}"
"function pollSpectrum(){"
"  var ch=currentSpecCh();if(ch===null)return;"
"  fetch('/api/spectrum?ch='+ch+'&bins=512')"
"    .then(function(r){return r.json()})"
"    .then(function(d){drawWaterfall(d)})"
"    .catch(function(){});"
"  fetch('/api/constellation?ch='+ch)"
"    .then(function(r){return r.json()})"
"    .then(function(d){drawConstellation(d)})"
"    .catch(function(){});"
"}"
"function drawConstellation(d){"
"  var cv=document.getElementById('const-canvas');if(!cv)return;"
"  var ctx=cv.getContext('2d');"
"  var W=cv.width,H=cv.height;"
"  ctx.fillStyle='#0b1220';ctx.fillRect(0,0,W,H);"
/* Axes through the middle */
"  ctx.strokeStyle='#1e293b';ctx.lineWidth=1;ctx.beginPath();"
"  ctx.moveTo(0,H/2);ctx.lineTo(W,H/2);"
"  ctx.moveTo(W/2,0);ctx.lineTo(W/2,H);ctx.stroke();"
"  if(!d||!d.ok||!d.points||!d.points.length){"
"    ctx.fillStyle='#64748b';ctx.font='10px system-ui';ctx.textAlign='center';"
"    ctx.fillText('no constellation',W/2,H/2-6);"
"    return;"
"  }"
/* Auto-scale to the largest magnitude seen in this batch, padded. */
"  var mx=0.01;"
"  for(var i=0;i<d.points.length;i++){"
"    var ax=Math.abs(d.points[i][0]),ay=Math.abs(d.points[i][1]);"
"    if(ax>mx)mx=ax;if(ay>mx)mx=ay;"
"  }"
"  var scale=(W/2-10)/mx;"
/* Unit circle reference at the auto-scale radius (so locked signals sit on the circle). */
"  ctx.strokeStyle='#334155';ctx.lineWidth=1;ctx.beginPath();"
"  ctx.arc(W/2,H/2,(W/2-10)*0.7,0,Math.PI*2);ctx.stroke();"
/* Points */
"  ctx.fillStyle='rgba(56,189,248,0.7)';"
"  for(var k=0;k<d.points.length;k++){"
"    var px=W/2+d.points[k][0]*scale;"
"    var py=H/2-d.points[k][1]*scale;"
"    ctx.fillRect(px-1,py-1,2,2);"
"  }"
"}"
/* dB → RGB magma ramp over -35..0 dB. Narrow range makes signal peaks
 * stand out against a dark noise floor. */
"function dbColor(db){"
"  var t=(db+35)/35;if(t<0)t=0;if(t>1)t=1;"
"  var r,g,b;"
"  if(t<0.25){r=t*4*40;g=0;b=t*4*80}"              /* black -> purple */
"  else if(t<0.5){var u=(t-0.25)*4;r=40+u*(180-40);g=0;b=80-u*40}"  /* purple -> red */
"  else if(t<0.75){var v=(t-0.5)*4;r=180+v*(255-180);g=v*140;b=40-v*40}" /* red -> orange */
"  else{var w=(t-0.75)*4;r=255;g=140+w*(230-140);b=w*60}"          /* orange -> yellow */
"  return[Math.round(r),Math.round(g),Math.round(b)]"
"}"
"function drawWaterfall(d){"
"  var cv=document.getElementById('spec-canvas');"
"  var ctx=cv.getContext('2d');"
"  var W=cv.width,H=cv.height;"
"  if(!d||!d.ok||!d.mags_db){"
"    ctx.fillStyle='#0b1220';ctx.fillRect(0,0,W,H);"
"    ctx.fillStyle='#64748b';ctx.font='12px system-ui';"
"    ctx.fillText('no data (channel unavailable)',10,20);"
"    document.getElementById('spec-info').textContent='';"
"    return;"
"  }"
"  specFs=d.fs||0;"
/* If channel changed, clear history so we don't show the previous channel's data */
"  if(d.ch!==specLastCh){"
"    ctx.fillStyle='#0b1220';ctx.fillRect(0,0,W,H);"
"    specLastCh=d.ch;"
"  }"
/* Scroll existing pixels down by 1 row (preserve rows 0..H-2 at y=1..H-1).
 * Row 0 (the old marker row) gets overwritten by the new data row below,
 * and the new marker row is drawn last. */
"  var prev=ctx.getImageData(0,0,W,H-1);"
"  ctx.putImageData(prev,0,1);"
/* Write the newest row at y=0. Each output pixel maps to a source FFT bin. */
"  var n=d.mags_db.length;"
"  var row=ctx.createImageData(W,1);"
"  for(var x=0;x<W;x++){"
"    var bin=Math.floor(x*n/W);if(bin>=n)bin=n-1;"
"    var c=dbColor(d.mags_db[bin]);"
"    row.data[x*4]=c[0];row.data[x*4+1]=c[1];row.data[x*4+2]=c[2];row.data[x*4+3]=255;"
"  }"
"  ctx.putImageData(row,0,0);"
/* Tune marker: triangle at the current mixer_hz, yellow=AFC, red=manual.
 * Drawn on top of the fresh row each frame. */
"  if(d.fs>0){"
"    var mx=(d.mixer_hz/(d.fs/2))*W;"
"    var mkColor=d.afc?'#fbbf24':'#ef4444';"
"    ctx.fillStyle=mkColor;"
"    ctx.beginPath();ctx.moveTo(mx-5,0);ctx.lineTo(mx+5,0);ctx.lineTo(mx,7);ctx.closePath();ctx.fill();"
"  }"
/* Compact info in the channel-picker row (short summary) */
"  var state=d.afc?'<span style=\"color:#fbbf24\">AFC</span>':'<span style=\"color:#ef4444\">Manual</span>';"
"  var info='baud '+d.baud+'  tune '+d.mixer_hz.toFixed(0)+' Hz  ['+state+']';"
"  document.getElementById('spec-info').innerHTML=info;"
"  var btn=document.getElementById('spec-auto');"
"  if(btn){btn.disabled=d.afc?true:false;btn.style.opacity=d.afc?'0.5':'1'}"
"}"
/* Populate channel dropdown from latest state snapshot */
"function updateSpecChannels(channels){"
"  var sel=document.getElementById('spec-ch');if(!sel||!channels)return;"
"  var have={};for(var i=0;i<sel.options.length;i++)have[sel.options[i].value]=1;"
"  var cur=sel.value;"
"  channels.forEach(function(c){"
"    if(!have[c.ch]){"
"      var o=document.createElement('option');"
"      o.value=c.ch;o.textContent='ch'+c.ch+' ('+c.baud+' baud)';"
"      sel.appendChild(o);"
"    }"
"  });"
"  if(!cur&&channels.length)sel.value=channels[0].ch;"
"}"
/* Canvas click → /api/tune at the clicked audio Hz (auto-disables AFC server-side) */
"document.addEventListener('DOMContentLoaded',function(){"
"  var cv=document.getElementById('spec-canvas');if(!cv)return;"
"  cv.addEventListener('click',function(ev){"
"    var ch=currentSpecCh();if(ch===null||!specFs)return;"
"    var rect=cv.getBoundingClientRect();"
"    var frac=(ev.clientX-rect.left)/rect.width;"
"    var hz=frac*(specFs/2);"
"    fetch('/api/tune?ch='+ch+'&hz='+hz.toFixed(1))"
"      .then(function(){pollSpectrum()}).catch(function(){});"
"  });"
"  var sel=document.getElementById('spec-ch');"
"  if(sel)sel.addEventListener('change',function(){pollSpectrum()});"
"});"
/* Auto button re-enables AFC on the current channel */
"function resetTune(){"
"  var ch=currentSpecCh();if(ch===null)return;"
"  fetch('/api/tune?ch='+ch+'&afc=1')"
"    .then(function(){pollSpectrum()}).catch(function(){});"
"}"
"var map=L.map('map',{center:[20,0],zoom:3});"
"var tileLayer=null;"
"function setTheme(name){"
"  var html=document.documentElement;"
"  if(name==='light'){html.classList.add('light')}else{html.classList.remove('light')}"
"  try{localStorage.setItem('theme',name)}catch(e){}"
"  if(tileLayer)map.removeLayer(tileLayer);"
"  var url=(name==='light')?'https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png'"
"                          :'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png';"
"  tileLayer=L.tileLayer(url,{attribution:'CartoDB',maxZoom:19}).addTo(map);"
"  var btn=document.getElementById('theme-toggle');"
"  if(btn)btn.textContent=(name==='light')?'\\u263C':'\\u263E';"
"}"
"function toggleTheme(){"
"  var cur=document.documentElement.classList.contains('light')?'light':'dark';"
"  setTheme(cur==='light'?'dark':'light');"
"}"
"setTheme((function(){try{return localStorage.getItem('theme')||'dark'}catch(e){return 'dark'}})());"
""
"var stdc_layer=L.layerGroup().addTo(map);"
"var ac_layer=L.layerGroup().addTo(map);"
"var trail_layer=L.layerGroup().addTo(map);"
"L.control.layers(null,{'STD-C':stdc_layer,'Aircraft':ac_layer,"
"'Trails':trail_layer},{position:'bottomleft'}).addTo(map);"
""
"var ac_markers={};"
"var allAcars=[];"
"var acarsKeys={};"
"function exportAircraft(){"
"  if(allAcars.length===0){alert('No ACARS messages collected yet.');return;}"
"  var csv='timestamp,reg,flight,label,channel,lat,lon,text\\n';"
"  allAcars.forEach(function(a){"
"    var d=new Date(a.t*1000);"
"    var txt=(a.text||'').replace(/[\"\\n\\r,]/g,' ').substring(0,200);"
"    csv+=d.toISOString()+','+a.reg+','+a.flight+','+a.label+','+a.ch"
"      +','+(a.lat||'')+','+(a.lon||'')+',\"'+txt+'\"\\n';"
"  });"
"  var blob=new Blob([csv],{type:'text/csv'});"
"  var a=document.createElement('a');"
"  a.href=URL.createObjectURL(blob);"
"  a.download='inmarsat_acars_'+Date.now()+'.csv';"
"  a.click();"
"}"
"var stdc_markers=[];"
""
"function fmtTime(ts){"
"var d=new Date(ts*1000);"
"return d.toUTCString().slice(17,25)+'Z'}"
""
"function update(d){"
"var now=d.t;"
"var nPos=0;"
"if(d.aircraft){d.aircraft.forEach(function(a){if(a.fixes&&a.fixes.length)nPos++})}"
"document.getElementById('n-ac').textContent=d.aircraft?d.aircraft.length:0;"
"document.getElementById('n-aircraft').textContent=d.aircraft?d.aircraft.length:0;"
"document.getElementById('n-pos').textContent=nPos;"
"document.getElementById('n-stdc').textContent=d.stdc?d.stdc.length:0;"
"/* Hide STD-C tab and counter when not in a mode that decodes it */"
"var stdcUI=document.querySelectorAll('.stdc-ui');"
"for(var i=0;i<stdcUI.length;i++)stdcUI[i].style.display=d.stdc_enabled?'':'none';"
"var specUI=document.querySelectorAll('.spectrum-ui');"
"for(var j=0;j<specUI.length;j++)specUI[j].style.display=d.spectrum_enabled?'':'none';"
"document.getElementById('status').style.color='#22c55e';"
"document.getElementById('status').textContent='live';"
"if(d.channels){"
"  var cp=document.getElementById('ch-panel');"
"  var locked=0;"
"  var html='';"
"  function fmtN(n){return n>=10000?(n/1000).toFixed(1)+'k':n>=1000?(n/1000).toFixed(1)+'k':''+n}"
"  d.channels.forEach(function(c){"
"    var hasRecent=c.msgs>0&&c.age>=0&&c.age<120;"
"    var active=!!c.lock||hasRecent;"
"    if(active)locked++;"
"    var baud=c.baud>=8400?(c.baud/1000+'k OQPSK'):(c.baud+' MSK');"
"    var dot=active?'\\u25CF':'\\u25CB';"
"    var color=active?'#38bdf8':'#475569';"
"    var msgs=c.msgs>0?fmtN(c.msgs):'\\u2014';"
"    var eb=c.ebno.toFixed(1);"
"    var ebc=c.msgs>0?'#38bdf8':'#64748b';"
"    html+='<div style=\"color:'+color+';display:flex;gap:6px;padding:1px 0;align-items:center\">'"
"      +'<span>'+dot+'</span>'"
"      +'<span style=\"min-width:32px\">ch'+c.ch+'</span>'"
"      +'<span style=\"min-width:78px\">'+baud+'</span>'"
"      +'<span style=\"min-width:38px;text-align:right\">'+msgs+'</span>'"
"      +'<span style=\"min-width:55px;color:'+ebc+'\">'+eb+' dB</span>'"
"      +'</div>';"
"  });"
"  document.getElementById('n-ac').textContent=locked+'/'+d.channels.length;"
"  cp.innerHTML=html;"
"  updateSpecChannels(d.channels);"
"}"
"if(d.aircraft){d.aircraft.forEach(function(a){"
"  var key=a.reg+'|'+a.last_seen.toFixed(1);"
"  if(!acarsKeys[key]){"
"    acarsKeys[key]=1;"
"    allAcars.push({t:a.last_seen,reg:a.reg,flight:a.flight,"
"      label:a.label,ch:a.ch,text:a.text,"
"      lat:a.fixes&&a.fixes.length?a.fixes[a.fixes.length-1][0]:null,"
"      lon:a.fixes&&a.fixes.length?a.fixes[a.fixes.length-1][1]:null});"
"    if(allAcars.length>5000){allAcars=allAcars.slice(-2500);acarsKeys={};allAcars.forEach(function(e){acarsKeys[e.reg+'|'+e.t.toFixed(1)]=1;});}"
"  }"
"})}"
""
"/* STD-C messages */"
"var sl=document.getElementById('stdc-list');"
"sl.innerHTML='';"
"stdc_layer.clearLayers();"
"var recent=d.stdc.slice(-50).reverse();"
"for(var i=0;i<recent.length;i++){"
"var m=recent[i];"
"var div=document.createElement('div');"
"div.className='msg egc';"
"div.innerHTML='<div class=\"ts\">'+fmtTime(m.t)+'</div>'"
"+m.text.substring(0,200);"
"sl.appendChild(div);"
"if(m.lat!==undefined){"
"L.circleMarker([m.lat,m.lon],{radius:5,color:'#0f0',"
"fillColor:'#0f0',fillOpacity:0.7}).addTo(stdc_layer)"
".bindPopup('<b>STD-C</b><br>'+m.text);"
"}}"
""
"/* Aircraft list — cap at 50 most recent to avoid DOM bloat */"
"var al=document.getElementById('aero-list');"
"al.innerHTML='';"
"var seen={};"
"var sorted=d.aircraft.slice().sort(function(a,b){return b.last_seen-a.last_seen;});"
"var maxList=Math.min(sorted.length,50);"
"for(var i=0;i<sorted.length;i++){"
"var ac=sorted[i];"
"seen[ac.reg]=1;"
"if(i<maxList){"
"var div=document.createElement('div');"
"div.className='msg acars';"
"div.innerHTML='<div class=\"hdr\">'+ac.reg+' '+ac.flight"
"+'</div><div class=\"ts\">'+fmtTime(ac.last_seen)"
"+'</div>'+ac.text.substring(0,150);"
"al.appendChild(div);"
"}"
""
"if(ac.fixes.length>0){"
"var last=ac.fixes[ac.fixes.length-1];"
"var latlng=[last[0],last[1]];"
"var m=ac_markers[ac.reg];"
"if(m){"
"m.setLatLng(latlng);"
"m._popupText='<b>'+ac.reg+'</b><br>'+ac.flight+'<br>Alt: '+last[2]+' ft<br>'+ac.text;"
"}else{"
"m=L.circleMarker(latlng,{radius:6,color:'#38bdf8',fillColor:'#38bdf8',fillOpacity:0.8}).addTo(ac_layer);"
"m._popupText='<b>'+ac.reg+'</b><br>'+ac.flight+'<br>Alt: '+last[2]+' ft<br>'+ac.text;"
"m.bindPopup(function(){return m._popupText;});"
"ac_markers[ac.reg]=m;"
"}"
"}}"
""
"/* Remove stale markers */"
"for(var r in ac_markers){"
"if(!seen[r]){ac_layer.removeLayer(ac_markers[r]);"
"delete ac_markers[r];}}"
""
"/* Rebuild trails (clear first to avoid layer accumulation) */"
"trail_layer.clearLayers();"
"for(var i=0;i<d.aircraft.length;i++){"
"var ac=d.aircraft[i];"
"if(ac.fixes.length>1){"
"var pts=ac.fixes.map(function(f){return[f[0],f[1]]});"
"L.polyline(pts,{color:'#38bdf8',weight:1,opacity:0.5,"
"dashArray:'4 4'}).addTo(trail_layer);}}"
"}"
""
"var base=location.protocol+'//'+location.host+'/';"
"var es=new EventSource(base+'api/events');"
"es.addEventListener('update',function(e){"
"try{update(JSON.parse(e.data))}catch(err){}});"
"es.onerror=function(){"
"document.getElementById('stats').innerHTML='Reconnecting...';};"
"</script></body></html>";

/* ---- HTTP server ---- */

static int server_fd = -1;
static pthread_t server_tid;

static void send_response(int fd, const char *status, const char *content_type,
                            const char *body, int body_len) {
    char header[512];
    int hlen = snprintf(header, sizeof(header),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n\r\n",
        status, content_type, body_len);

    struct iovec iov[2];
    iov[0].iov_base = header;
    iov[0].iov_len = hlen;
    iov[1].iov_base = (void *)body;
    iov[1].iov_len = body_len;
    if (writev(fd, iov, 2) < 0) { /* ignore broken pipe */ }
}

static void handle_sse(int fd) {
    const char *header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "X-Accel-Buffering: no\r\n"
        "Access-Control-Allow-Origin: *\r\n\r\n";

    if (write(fd, header, strlen(header)) < 0) {
        close(fd);
        return;
    }

    char *json = malloc(JSON_BUF_SIZE);
    if (!json) { close(fd); return; }

    while (running) {
        usleep(1000000);  /* 1 Hz updates */

        int jlen = build_json(json, JSON_BUF_SIZE - 64);

        char prefix[32];
        int plen = snprintf(prefix, sizeof(prefix), "event: update\ndata: ");

        struct iovec iov[3];
        iov[0].iov_base = prefix;
        iov[0].iov_len = plen;
        iov[1].iov_base = json;
        iov[1].iov_len = jlen;
        iov[2].iov_base = (void *)"\n\n";
        iov[2].iov_len = 2;

        if (writev(fd, iov, 3) < 0)
            break;
    }

    free(json);
    close(fd);
}

static void *client_thread(void *arg) {
    int fd = (int)(intptr_t)arg;

    char req[4096];
    int rlen = read(fd, req, sizeof(req) - 1);
    if (rlen <= 0) {
        close(fd);
        return NULL;
    }
    req[rlen] = '\0';

    /* Parse request path */
    char *path = NULL;
    if (strncmp(req, "GET ", 4) == 0) {
        path = req + 4;
        char *end = strchr(path, ' ');
        if (end) *end = '\0';
    }

    if (!path) {
        send_response(fd, "400 Bad Request", "text/plain", "Bad Request", 11);
        close(fd);
        return NULL;
    }

    extern int spectrum_enabled;

    if (strcmp(path, "/") == 0) {
        send_response(fd, "200 OK", "text/html",
                       HTML_PAGE, (int)sizeof(HTML_PAGE) - 1);
        close(fd);
    } else if (strcmp(path, "/api/events") == 0) {
        handle_sse(fd);  /* blocks until disconnect */
    } else if (strcmp(path, "/api/state") == 0) {
        char *json = malloc(JSON_BUF_SIZE);
        if (json) {
            int jlen = build_json(json, JSON_BUF_SIZE - 64);
            send_response(fd, "200 OK", "application/json", json, jlen);
            free(json);
        }
        close(fd);
    } else if (strncmp(path, "/api/spectrum", 13) == 0 && spectrum_enabled) {
        /* /api/spectrum?ch=N&bins=512
         * Returns mag-dB array 0..Fs/2 + tune info + lockingbw + AFC state. */
        extern int web_get_spectrum_by_channel(int, float *, int,
                                               double *, double *, double *,
                                               double *, int *, int *);
        int ch = -1, n_bins = 512;
        const char *q = strchr(path, '?');
        if (q) {
            const char *p = q + 1;
            while (*p) {
                if (!strncmp(p, "ch=", 3))   ch     = atoi(p + 3);
                if (!strncmp(p, "bins=", 5)) n_bins = atoi(p + 5);
                const char *amp = strchr(p, '&');
                if (!amp) break;
                p = amp + 1;
            }
        }
        if (n_bins < 32)   n_bins = 32;
        if (n_bins > 1024) n_bins = 1024;

        float *mags = (float *)malloc(n_bins * sizeof(float));
        double mixer = 0, fc = 0, fs = 0, lockbw = 0;
        int baud = 0, afc_on = 0;
        int ok = (ch >= 0 && mags &&
                  web_get_spectrum_by_channel(ch, mags, n_bins,
                                              &mixer, &fc, &fs,
                                              &lockbw, &baud, &afc_on) == 0);
        char *body = (char *)malloc(n_bins * 8 + 512);
        if (!body) { free(mags); close(fd); return NULL; }
        int pos = 0;
        if (!ok) {
            pos = snprintf(body, 512,
                "{\"ok\":false,\"ch\":%d,\"reason\":\"channel unavailable\"}", ch);
        } else {
            pos = snprintf(body, 512,
                "{\"ok\":true,\"ch\":%d,\"baud\":%d,"
                "\"mixer_hz\":%.2f,\"freq_center_hz\":%.2f,\"fs\":%.2f,"
                "\"lockingbw\":%.2f,\"afc\":%s,"
                "\"bins\":%d,\"mags_db\":[",
                ch, baud, mixer, fc, fs, lockbw,
                afc_on ? "true" : "false", n_bins);
            for (int i = 0; i < n_bins; i++) {
                pos += snprintf(body + pos, n_bins * 8 + 512 - pos,
                                "%s%.1f", i ? "," : "", mags[i]);
            }
            pos += snprintf(body + pos, n_bins * 8 + 512 - pos, "]}");
        }
        send_response(fd, "200 OK", "application/json", body, pos);
        free(mags);
        free(body);
        close(fd);
    } else if (strncmp(path, "/api/constellation", 18) == 0 && spectrum_enabled) {
        /* /api/constellation?ch=N — returns up to 300 I/Q points from
         * the demod's post-matched-filter ring buffer. Used by the
         * Spectrum tab scatter plot. */
        extern int web_get_constellation_by_channel(int, double *, int);
        int ch = -1;
        const char *q = strchr(path, '?');
        if (q) {
            const char *p = q + 1;
            while (*p) {
                if (!strncmp(p, "ch=", 3)) ch = atoi(p + 3);
                const char *amp = strchr(p, '&');
                if (!amp) break;
                p = amp + 1;
            }
        }
        const int MAX_PAIRS = 300;
        double *iq = (double *)malloc(MAX_PAIRS * 2 * sizeof(double));
        int n = (ch >= 0 && iq) ? web_get_constellation_by_channel(ch, iq, MAX_PAIRS) : 0;
        char *body = (char *)malloc(MAX_PAIRS * 24 + 128);
        if (!body) { free(iq); close(fd); return NULL; }
        int pos = snprintf(body, 128,
            "{\"ok\":%s,\"ch\":%d,\"points\":[",
            n > 0 ? "true" : "false", ch);
        for (int i = 0; i < n; i++) {
            pos += snprintf(body + pos, MAX_PAIRS * 24 + 128 - pos,
                            "%s[%.4f,%.4f]", i ? "," : "", iq[i * 2], iq[i * 2 + 1]);
        }
        pos += snprintf(body + pos, MAX_PAIRS * 24 + 128 - pos, "]}");
        send_response(fd, "200 OK", "application/json", body, pos);
        free(iq);
        free(body);
        close(fd);
    } else if (strncmp(path, "/api/tune", 9) == 0 && spectrum_enabled) {
        /* /api/tune?ch=N[&hz=1234.5][&afc=0|1]
         * Setting hz automatically disables AFC so the tune sticks
         * (unless afc=1 also passed). Sending only afc=1 re-enables AFC. */
        extern int web_set_tune_by_channel(int, double, int);
        int ch = -1;
        double hz = -1.0;
        int afc_action = 0;  /* 0 leave, 1 enable, -1 disable */
        int got_hz = 0, got_afc = 0;
        const char *q = strchr(path, '?');
        if (q) {
            const char *p = q + 1;
            while (*p) {
                if (!strncmp(p, "ch=", 3)) ch = atoi(p + 3);
                if (!strncmp(p, "hz=", 3)) { hz = atof(p + 3); got_hz = 1; }
                if (!strncmp(p, "afc=", 4)) {
                    got_afc = 1;
                    afc_action = (atoi(p + 4) > 0) ? 1 : -1;
                }
                const char *amp = strchr(p, '&');
                if (!amp) break;
                p = amp + 1;
            }
        }
        /* If hz was set and afc wasn't explicitly toggled, auto-disable AFC */
        if (got_hz && !got_afc) afc_action = -1;
        char body[160];
        int rc = (ch >= 0 && (got_hz || got_afc))
                 ? web_set_tune_by_channel(ch, got_hz ? hz : -1.0, afc_action)
                 : -1;
        int blen = snprintf(body, sizeof(body),
            "{\"ok\":%s,\"ch\":%d,\"hz\":%.2f,\"afc\":%s}",
            rc == 0 ? "true" : "false", ch, hz,
            afc_action > 0 ? "true" : (afc_action < 0 ? "false" : "null"));
        send_response(fd, "200 OK", "application/json", body, blen);
        close(fd);
    } else {
        send_response(fd, "404 Not Found", "text/plain", "Not Found", 9);
        close(fd);
    }

    return NULL;
}

static void *server_thread(void *arg) {
    (void)arg;

    while (running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr *)&client_addr,
                                &client_len);
        if (client_fd < 0)
            continue;

        pthread_t tid;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        pthread_create(&tid, &attr, client_thread,
                        (void *)(intptr_t)client_fd);
        pthread_attr_destroy(&attr);
    }

    return NULL;
}

int web_init(int port) {
    memset(&state, 0, sizeof(state));
    pthread_mutex_init(&state.lock, NULL);

    /* Ignore SIGPIPE for broken SSE connections */
    signal(SIGPIPE, SIG_IGN);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("web: socket");
        return -1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = INADDR_ANY,
    };

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("web: bind");
        close(server_fd);
        server_fd = -1;
        return -1;
    }

    if (listen(server_fd, 8) < 0) {
        perror("web: listen");
        close(server_fd);
        server_fd = -1;
        return -1;
    }

    fprintf(stderr, "Web dashboard: http://localhost:%d/\n", port);

    pthread_create(&server_tid, NULL, server_thread, NULL);
    return 0;
}

void web_shutdown(void) {
    if (server_fd >= 0) {
        close(server_fd);
        server_fd = -1;
    }
    pthread_mutex_destroy(&state.lock);
}
