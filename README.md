# inmarsat-sniffer

A standalone Inmarsat L-band decoder written in C. Decodes STD-C (Enhanced Group Call maritime safety messages) and Aero (aviation ACARS, ADS-C, CPDLC) simultaneously from a single SDR receiver. No GNU Radio, no Python, no Java runtime -- just one binary.

Supports HackRF, BladeRF, USRP (UHD), RTL-SDR, SDRplay (native API v3), Airspy R2/Mini, and any SoapySDR device for live capture, plus VITA 49 (VRT) UDP input and IQ file playback. Built-in web dashboard (`--web`) provides a real-time Leaflet.js map with aircraft positions and decoded messages. Outputs include JSON feed (`--feed`, `--udp`), SBS/BaseStation (`--basestation`) for tar1090/VRS, and MQTT (`--mqtt`) -- all with JAERO-compatible field names for drop-in integration with existing tools.

The Aero decode chain uses JAERO's proven DSP code (MskDemodulator, OqpskDemodulator, plus BurstMskDemodulator / BurstOqpskDemodulator for future burst-channel work) ported from [jontio/JAERO](https://github.com/jontio/JAERO), Qt-stripped to pure C++. When [libacars-2](https://github.com/szpajder/libacars) is installed, ARINC-622 application payloads (ADS-C position reports, CPDLC controller-pilot messages) are fully decoded and reassembled.

Sister project to [iridium-sniffer](https://github.com/alphafox02/iridium-sniffer) for Iridium L-band.

## Features

- Simultaneous STD-C EGC + Aero ACARS decode from one SDR
- Up to 27-channel parallel demodulation with per-channel worker threads (varies by satellite: 4F3/AF1=27, F1=17, 3F5=14)
- Aero 600/1200 baud MSK (P-channel continuous, via JAERO MskDemodulator + AeroL)
- Aero 10500/8400 baud OQPSK (continuous OqpskDemodulator + AeroL)
- STD-C EGC: DBPSK demod, Viterbi k=7 FEC, frame sync and message parsing
- ADS-C position extraction from binary ARINC 620 payloads (tags 7/9/10/14/15/18/19/20)
- CPDLC (controller-pilot datalink) message surfacing via libacars
- Three-tier position extraction: ADS-C binary, coordinate regex, waypoint DB (125k fixes)
- Learned waypoints harvested from FPN (flight plan) messages at runtime
- SBS/BaseStation output (`--basestation`) for tar1090, VRS, PlanePlotter
- MQTT output (`--mqtt`) with configurable host, user/pass, topic
- JSON feed with station-id (`--feed`, `--udp`, `--station-id`)
- JAERO text format 3 output (`--jaero-format=HOST:PORT`) via UDP for legacy script compatibility
- AES/GES identifiers from ISU layer in all outputs
- Built-in web dashboard with dark theme, aircraft markers, signal quality bars, trail history, CSV export, and per-channel lock indicator driven by demod signal status (not just message recency)
- Optional Spectrum tab (`--spectrum`) with scrolling waterfall, I/Q constellation, and click-to-tune per channel; runs only while the tab is open
- Non-blocking I/O on every decode-path sink (stdout JSON, stderr, UDP, ZMQ, MQTT) so a stalled downstream consumer drops messages instead of freezing decode
- Two-stage channelizer with per-channel digital gain
- 125-tap Hilbert USB demod on both internal decode and ZMQ output paths (matches SDRReceiver's `vfo::usb_demod()` math; inner loop optimised from 125 to 31 multiplies via antisymmetric zero-tap pairing)
- Startup auto-calibration for SDR crystal offset (measures carrier error on first active channel, adjusts center freq)
- Aircraft database (568k entries from tar1090-db) for AES/registration-to-ICAO-hex lookup and type/operator enrichment in ACARS output
- AVX2, SSE4.2, and NEON SIMD kernels with automatic runtime detection
- `--skip-c-channel` flag to disable 11 OQPSK 8400 demods (~27% CPU savings, Pi-friendly)
- RTL-SDR AGC mode (`--agc`) for weak signal setups
- ZMQ audio output (`--zmq`) for JAERO-compatible per-channel streaming (same USB-demod audio SDRReceiver produces)
- VITA 49 (VRT) UDP input for remote/distributed SDR setups
- HackRF, BladeRF, USRP, RTL-SDR, SDRplay, Airspy R2/Mini, and SoapySDR native backends
- MacOS (Homebrew) build support on Intel and Apple Silicon

## What it decodes

- **STD-C / EGC**: NAVAREA warnings, METAREA weather, distress alerts, SafetyNET
- **Aero ACARS**: 600/1200 baud MSK P-channel + 8400/10500 baud OQPSK C-channel
- **ADS-C**: Binary position reports with lat/lon/alt/heading/groundspeed
- **CPDLC**: Controller-pilot text messages (uplink clearances, downlink reports)

All decoded simultaneously from a single wideband capture.

## Installation

### DragonOS Noble

DragonOS has the SDR libraries and libacars pre-installed. Just clone and build:

```bash
git clone https://github.com/alphafox02/inmarsat-sniffer.git
cd inmarsat-sniffer
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install   # installs as /usr/local/bin/inmarsat-sniffer; skip and use ./inmarsat-sniffer if you'd rather not install system-wide
```

CMake auto-detects available backends. All should show "enabled".

### Ubuntu / Debian

```bash
git clone https://github.com/alphafox02/inmarsat-sniffer.git
cd inmarsat-sniffer

# Core dependencies
sudo apt install build-essential cmake pkg-config


# SDR libraries (install what you have)
sudo apt install librtlsdr-dev       # RTL-SDR native
sudo apt install libhackrf-dev       # HackRF One
sudo apt install libbladerf-dev      # BladeRF
sudo apt install libuhd-dev          # USRP (B2x0, N2x0, etc.)
sudo apt install libsoapysdr-dev     # SoapySDR (any device)
sudo apt install libairspy-dev       # Airspy R2 / Mini
# SDRplay native API: install from https://www.sdrplay.com/api/

# Optional: ACARS ARINC-622/ADS-C/CPDLC — libacars-2 is NOT packaged on
# most Debian/Ubuntu releases. Build from source:
sudo apt install autoconf automake libtool pkg-config
git clone https://github.com/szpajder/libacars.git /tmp/libacars
cd /tmp/libacars && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
sudo ldconfig
cd -  # back to inmarsat-sniffer

# Optional: ZMQ audio output for external JAERO
sudo apt install libzmq3-dev

# Optional: MQTT broker publishing
sudo apt install libmosquitto-dev

mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install   # installs as /usr/local/bin/inmarsat-sniffer; skip and use ./inmarsat-sniffer if you'd rather not install system-wide
```

### macOS (Homebrew)

```bash
brew install cmake librtlsdr hackrf libbladerf uhd soapysdr mosquitto zmq airspy

# libacars isn't in Homebrew — build from source:
git clone https://github.com/szpajder/libacars.git /tmp/libacars
cd /tmp/libacars && mkdir build && cd build
cmake .. && make -j$(sysctl -n hw.ncpu) && sudo make install
cd -

git clone https://github.com/alphafox02/inmarsat-sniffer.git
cd inmarsat-sniffer && mkdir build && cd build
cmake .. && make -j$(sysctl -n hw.ncpu)
```

### SDRplay users

Install the SDRplay API v3 from [sdrplay.com](https://www.sdrplay.com/api/) first. CMake finds it automatically.

### Build output

```
-- SoapySDR: enabled
-- SDRplay: enabled
-- RTL-SDR: enabled
-- HackRF: enabled
-- BladeRF: enabled
-- USRP (UHD): enabled
-- Airspy: enabled
-- SIMD: SSE4.2 + AVX2+FMA kernels enabled (runtime-detected)
-- MQTT: enabled
-- ZMQ: enabled
-- libacars: enabled
```

## Supported SDR hardware

Native backends (no SoapySDR needed):

| Flag | Device | Notes |
|------|--------|-------|
| `-i rtl-0` | RTL-SDR Blog V3/V4 | Cheapest, 2.4 MHz BW, aero-only mode |
| `-i hackrf[-SERIAL]` | HackRF One | 20 MHz BW, good for full mode |
| `-i bladerf0` | BladeRF x40/xA4/micro | Up to 56 MHz BW |
| `-i usrp-PRODUCT-SERIAL` | Ettus USRP | B205mini, B210, N210, X310 |
| `-i sdrplay[-SERIAL]` | SDRplay RSP family | RSPdx recommended, 10 MHz BW, bias tee |
| `-i airspy[-HEXSN]` | Airspy R2 / Mini | 12-bit ADC, fixed 2.5/10 MSPS rates auto-selected |
| `-i soapy-N` or `soapy:args` | Any SoapySDR device | LimeSDR, PlutoSDR, etc. |
| `--vita49[=IP:PORT]` | VITA 49 (VRT) UDP | Remote SDR via network |

You need an antenna covering 1525-1559 MHz. A modified GPS patch, small helix, or L-band patch works. Point at the satellite -- Inmarsat birds are geostationary.

## Usage

### Quick start

```bash
# SDRplay, decode Aero ACARS on Inmarsat 4F3 (Americas, 98W)
inmarsat-sniffer -i sdrplay --satellite=4F3

# RTL-SDR (defaults to 1.536 MHz — matches SDRReceiver for best 8-bit SNR)
inmarsat-sniffer -i rtl-0 --satellite=4F3 --mode=aero

# HackRF (defaults to 6 MHz, LNA=40, VGA=20 — tested decoding MSK + OQPSK)
inmarsat-sniffer -i hackrf --satellite=4F3 --mode=aero --web -B

# Raspberry Pi friendly (RTL-SDR, skip C-channels)
inmarsat-sniffer -i rtl-0 --satellite=4F3 --mode=aero --skip-c-channel
```

### With web dashboard + SBS feed

```bash
inmarsat-sniffer -i sdrplay --satellite=4F3 --web --basestation=30003
# Web map: http://localhost:8888
# SBS feed: connect tar1090/VRS to localhost:30003
```

### Spectrum tab (waterfall + constellation + click-to-tune)

Opt-in via `--spectrum` (implies `--web`). Adds a Spectrum tab to the
dashboard with a scrolling waterfall per channel, an I/Q constellation
scatter, and click-to-tune that disables AFC so the manual frequency
holds. Not enabled by default — runs FFTs only while the tab is open.

```bash
inmarsat-sniffer -i sdrplay --satellite=4F3 --spectrum
```

### Push SBS to a remote aggregator

```bash
inmarsat-sniffer -i sdrplay --satellite=4F3 --basestation=myhost.net:2226
```

### JSON feed

```bash
# To stdout
inmarsat-sniffer -i rtl-0 --satellite=4F3 --feed --station-id=MY-STATION

# Via UDP (up to 4 endpoints)
inmarsat-sniffer -i rtl-0 --satellite=4F3 --udp=127.0.0.1:5555
```

### JAERO text format 3

```bash
# Send JAERO-compatible text format 3 via UDP (drop-in for existing JAERO scripts)
inmarsat-sniffer -i sdrplay --satellite=4F3 --jaero-format=127.0.0.1:5554
```

### MQTT

```bash
inmarsat-sniffer -i sdrplay --satellite=4F3 \
    --mqtt=broker.local:1883 --mqtt-user=user --mqtt-pass=pass \
    --mqtt-topic=inmarsat/acars
```

### Aircraft database

```bash
# Download tar1090-db aircraft.csv (568k entries, ~33 MB)
inmarsat-sniffer --update-db

# Use for reg-to-ICAO-hex in SBS output
inmarsat-sniffer -i sdrplay --satellite=4F3 --basestation=30003
```

### IQ recording for development

If you have good satellite reception and want to contribute a test recording:

```bash
# RTL-SDR, Inmarsat 4F3 (Americas), with bias tee LNA
./tools/capture_iq.sh --sdr=rtl --satellite=4F3 --bias-tee

# SDRplay, Inmarsat 4F3, 6 MHz capture (covers MSK + OQPSK)
./tools/capture_iq.sh --sdr=sdrplay --satellite=4F3

# Playback
inmarsat-sniffer -f capture_4F3_rtl_20260418.cu8 --format=cu8 --satellite=4F3
```

Recordings are capped at 500 MB (~1-2 minutes depending on sample rate). RTL-SDR saves as `.cu8` (unsigned 8-bit IQ), SDRplay saves as `.cs16` (signed 16-bit IQ).

### Mode selection

```bash
--mode=aero    # Aero channels only (default) — the verified-working path
--mode=stdc    # STD-C EGC only — experimental, STD-C decode not yet confirmed on-air
--mode=full    # Both — experimental, needs wider SDR (~9 MHz)
--mode=auto    # Auto-select based on SDR bandwidth (may enable STD-C silently)
```

### Low-power hosts (Raspberry Pi, SBCs)

The 11 OQPSK 8400 C-channel demods (ch17-27 on 4F3/AF1) carry burst
voice/data sessions that rarely emit ACARS. Skipping them cuts total
CPU by ~27% with no impact on P-channel ACARS or OQPSK 10500 decodes:

```bash
inmarsat-sniffer -i rtl-0 --satellite=4F3 --mode=aero --skip-c-channel
```

Rough CPU on an i7-11800H (SDRplay @ 3.072 MHz):
- All 27 channels: ~290% (2.9 cores)
- `--skip-c-channel`: ~150% (1.5 cores)
- RTL-SDR @ 1.536 MHz + `--skip-c-channel`: comfortable on Pi 5; Pi 4 probably too slow

## Satellites

| Flag | Name | Position | Region | Notes |
|------|------|----------|--------|-------|
| `4F3` | Inmarsat 4-F3 | 98.0W | Americas (AORW) | Best from North/South America |
| `3F5` | Inmarsat 3-F5 | 54.0W | Atlantic (AORE) | Eastern US and Europe |
| `AF1` | Alphasat (I-4A F4) | 25.0E | EMEA | Europe, Africa, Middle East. Aliases: `AF4`, `4AF4`, `alphasat`, `25E` |
| `F1`  | Inmarsat 4-F1 | 143.5E | Pacific (POR) | Asia-Pacific, Australia. Aliases: `4F1`, `143E` |

## Architecture

```
SDR/file/VITA49 --> channelizer (two-stage DDC per channel, SIMD-accelerated)
                        |
                        +-- STD-C EGC --> DBPSK demod --> Viterbi k=7 --> frame parser
                        |
                        +-- Aero 600/1200 --> Hilbert USB --> JAERO MskDemodulator --+
                        |                                    (continuous MSK, AFC)   |
                        +-- Aero 10500 ------> Hilbert USB --> JAERO OqpskDemod ---->+
                        |                                    (continuous OQPSK, AFC) |
                        +-- Aero 8400 -------> Hilbert USB --> JAERO OqpskDemod ---->+--> AeroL
                                                                                     |
                                                                      libacars (optional)
                                                                      ADS-C, CPDLC, ACARS
                                                                                     |
                                                               +---------+-----------+----------+
                                                               |         |           |          |
                                                          JSON feed    SBS/BS       MQTT    ZMQ audio
                                                          (--feed)   (--basestation) (--mqtt)  (--zmq)
                                                               |
                                                          Web dashboard (--web :8888)
```

## Bandwidth and SDR selection

| SDR bandwidth | Channels covered | Recommended hardware |
|---------------|-----------------|---------------------|
| ~2.4 MHz | 12 MSK P-channels (aero-only) | RTL-SDR Blog V3/V4 |
| ~3.2 MHz | 12 MSK + some OQPSK | RTL-SDR Blog V4 (higher rate) |
| ~6-10 MHz | Full 27-channel plan (MSK + OQPSK) | SDRplay RSPdx, Airspy R2 |
| ~10+ MHz | Full + STD-C (--mode=full) | SDRplay RSPdx, BladeRF, USRP |

Channels are automatically filtered based on your SDR's actual bandwidth -- the channelizer only adds channels whose center frequency falls within the captured spectrum. Channels outside the bandwidth are skipped (visible with `-v`). This means an RTL-SDR at 2.4 MHz simply decodes fewer channels, not incorrectly -- no wasted CPU on out-of-band noise. Center frequency and sample rate are auto-computed from the satellite table unless you override with `-c` and `-r`.

## Position sources

Aircraft positions on the map come from three independent extractors
applied in order to every decoded Aero ACARS message. The shutdown
banner prints all three counters separately:

```
Position fixes: 12 binary ADS-C, 47 text-coord, 18 waypoint-name
```

1. **Binary ADS-C** -- ARINC-622 contract reports decoded by libacars
   (`la_proto_tree_find_adsc`). Returns a structured tag list with
   explicit lat/lon/alt/heading/groundspeed. This is the most precise
   source and the closest to "ADS-C" as the term is used in industry,
   but it requires the aircraft to have an active ADS-C contract on
   Inmarsat -- many carriers route ADS-C over VDL-2 or HF instead and
   use Inmarsat only for general ACARS. **Zero ADS-C fixes in a session
   doesn't mean the decoder is broken** -- just that no aircraft in
   beam happened to be sending binary contracts during that window.

2. **Text-coord** -- regex-extracted coordinates from human-readable
   FANS-1/A H1 message bodies (`acars_extract_text_position()`). Two
   formats are recognised: `POS[NS]ddddd[EW]dddddd` (Inmarsat POS
   prefix, degrees * 1000) and `[NS]dddmm[EW]dddmm` (degrees + tenths
   of a minute, e.g. `N33521W084123` -> 33d52.1'N, 84d12.3'W). Many
   carrier-specific encodings exist beyond these two; if you see H1
   traffic that consistently doesn't extract, an additional parser may
   be worth adding.

3. **Waypoint-name** -- looks up 5-letter ICAO fix identifiers found in
   the message body against `data/waypoints.csv` (~125k FAA + global
   fixes) and runtime-learned waypoints harvested from FPN flight-plan
   messages. Approximate -- the aircraft is somewhere near that fix,
   not on it -- but useful when the message body has no explicit
   coordinates and just names the next waypoint (common on NAT tracks).

Each message tries the extractors in order and stops at the first hit,
so the three counters never overlap. Many H1 messages aren't position
reports at all (REQPOS, REQPRG, AT1 logon, AFN handshake) and produce
zero fixes -- that's expected.

## Frequency correction (`--ppm` and auto-cal)

Every SDR has a small TCXO offset -- typically tens to a few hundred Hz at L-band. The decoder corrects for it two ways:

**Auto-cal (default, runs once at startup)**

If `--ppm` is *not* set, the decoder measures the carrier offset of the first aero channel against where the satellite says it should be, then shifts the channelizer's mix point in software to match. The startup banner prints something like:

```
Auto-cal: carrier offset 120 Hz on ch1, adjusting center freq
Channelizer: adjusted center by 120 Hz (new: 1545.620 MHz)
```

Triggers only when the offset exceeds 50 Hz (smaller than that is treated as already centered). Runs once per session -- there's no periodic recalibration.

A "good" SDR at L-band typically reads **100-150 Hz** of offset. Up to about a kilohertz is normal; multi-kHz offsets suggest either a poorly-trimmed crystal or that the auto-cal locked onto a noise spike, in which case the manual `--ppm` knob below is the answer.

**Manual `--ppm=N.NN`**

If you already know your SDR's PPM error (from `rtl_test`, `kalibrate-rtl`, or repeated runs of auto-cal), pass it explicitly. This:

- On **RTL-SDR**, retunes the device's hardware TCXO scaling via `rtlsdr_set_freq_correction()` -- best long-term stability since every retune benefits, including thermal drift compensation.
- On **HackRF, BladeRF, USRP, SDRplay, Airspy, SoapySDR**, applies the same value as a software shift at the channelizer (`offset_hz = ppm * center_freq / 1e6`). The hardware is left alone but the resulting baseband is correct at L-band.
- **Disables auto-cal** for that run -- if you've supplied the answer, the decoder won't re-measure and risk fighting your value.

PPM and a fixed Hz offset are equivalent at our single observation frequency: `1 ppm ≈ 1545 Hz` at L-band, so a 100 Hz observed offset is roughly `0.065 ppm`.

```bash
inmarsat-sniffer -i sdrplay --satellite=4F3 --ppm=0.07
```

Pick `--ppm` if you want repeatable behavior across runs (auto-cal varies by ±50 Hz between sessions because it depends on which sample is locked first), or if you operate on a satellite/band where auto-cal struggles to find a clean carrier.

## Current status

**Satellites verified live:**

- **4F3 (98°W, AOR-W)** -- primary test satellite; thousands of MSK P-channel ACARS, hundreds of aircraft, plus OQPSK 10500 ch13-16 on SDRplay/RTL-SDR/B210
- **3F5 (54°W, AOR-E)** -- verified on-air with RTL-SDR at auto-selected 1.92 MHz. MSK ACARS, CPDLC, and aircraft DB enrichment all working. Mix of transatlantic military (USAF C-17A) and South American/Canadian commercial traffic
- AF1 / Alphasat (25°E) and F1 (143°E) are not reachable from North America; AF1 STD-C carrier corrected to 1537.100 MHz per external reference but awaits on-air test from Europe

**Working and verified live:**

- 600/1200 baud MSK P-channel ACARS decode (thousands of messages across hundreds of aircraft, 100% CRC pass)
- 10500 baud OQPSK forward link -- continuous OqpskDemodulator with Hilbert USB demod, aircraft decoded across ch13-16 on both SDRplay and RTL-SDR
- ADS-C position extraction (oceanic aircraft tracked across North/South Atlantic, Americas)
- CPDLC controller-pilot messages decoded via libacars (incl. UNABLE/WILCO responses)
- SBS basestation feed verified with remote aggregator
- Per-channel threading with zero drops over multi-hour runs
- Web dashboard with live aircraft markers, trail history, sigstat-driven lock indicator
- ZMQ audio output -- external JAERO locks and decodes the exact same audio our internal demod consumes (SDRReceiver-compatible Hilbert USB wiring)
- Aircraft DB enrichment -- registration, type, and operator shown alongside ACARS output
- RTL-SDR Blog V4, SDRplay RSPdx/RSP1A, and Ettus USRP B210 all tested live
- Capture replay via `-f FILE` matches live decode counts

**Plumbed, awaiting traffic or verification:**

- 8400 baud OQPSK C-channel -- continuous OqpskDemodulator wired, but 8400 channels are burst voice/data sessions and may be silent for extended periods. Skip with `--skip-c-channel` on low-power hosts
- STD-C EGC decode -- DBPSK/Viterbi path active; hasn't synced in testing on 4F3 at author's location. May need stronger signal or different satellite. AF1 (25E) STD-C carrier corrected to 1537.100 MHz but untested on-air from here
- F1 (143°E) channel plan -- came from a forum listing, not cross-referenced against an authoritative source; no confirmation available from North America
- HackRF on 4F3 -- decoding at default gains with bias tee; still characterising optimal gain staging

**Not implemented:**

- Voice decoding (Inmarsat Aero carries AMBE-encoded voice on C-channel slots; would require mbelib or similar AMBE codec, same approach as DSD/OP25)
- R/T burst channel frequencies (not in satellite tables; would need frequency survey. BurstMskDemodulator / BurstOqpskDemodulator are ported and ready to wire when frequencies are known)
- C-band feeder link reception (BurstOqpskDemodulator is the intended demod; needs C-band dish + downconverter + frequency entries)

## Related projects

- [iridium-sniffer](https://github.com/alphafox02/iridium-sniffer) -- Sister project for Iridium L-band
- [JAERO](https://github.com/jontio/JAERO) -- Aero ACARS decoder (Qt GUI), DSP code ported here
- [libacars](https://github.com/szpajder/libacars) -- ACARS/ARINC-622 message decoder library
- [SatDump](https://github.com/SatDump/SatDump) -- Multi-satellite decoder
- [sdrpp-inmarsatc-demodulator](https://github.com/cropinghigh/sdrpp-inmarsatc-demodulator) -- SDR++ Inmarsat-C plugin
- [inmarsatc](https://github.com/cropinghigh/inmarsatc) -- Inmarsat-C decoder library
- [stdcdec](https://github.com/cropinghigh/stdcdec) -- Standalone STD-C decoder
- [gr-JAERO](https://github.com/muaddib1984/gr-JAERO) -- GNU Radio Inmarsat Aero RF front-end
- [SDRReceiver](https://github.com/jeroenbeijer/SDRReceiver) -- Multi-VFO receiver; our Hilbert USB path uses its `vfo::usb_demod()` math for JAERO compatibility
- [thebaldgeek STD-C](https://thebaldgeek.github.io/stdc.html) -- Cross-referenced satellite STD-C frequencies

## License

GPL-3.0-or-later. Copyright (c) 2026 CEMAXECUTER LLC.

DSP code in `jaero_dsp/` is derived from [JAERO](https://github.com/jontio/JAERO) by Jonathan Olds, MIT license.
