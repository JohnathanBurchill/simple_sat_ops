/*

   Simple Satellite Operations  utils/tx_frame_sdr.c

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

// Single-shot AX100 frame transmitter via the USRP B210.
//
// Pipeline:
//   payload (ascii / hex)
//     -> CSP v1 packet
//     -> AX100 frame (preamble + ASM + Golay len + scrambler + RS + tail)
//     -> Gaussian-shaped BPSK baseband PCM (modem.c, S16_LE, real-valued)
//     -> software FM modulator (deviation = peak ±dev_hz, default
//        bit_rate/4 → 2.4 kHz at 9600 baud → modulation index h = 0.5)
//     -> complex int16 IQ at the TX sample rate
//     -> UHD TX streamer at the configured center frequency / gain
//
// This is the SDR-side equivalent of utils/tx_frame.c. tx_frame goes
// PCM -> ALSA -> SignaLink -> FT-991A -> RF; tx_frame_sdr replaces the
// ALSA-and-radio half with software FM modulation + UHD streaming, so
// it builds on Mac (no ALSA, no SGP4, no ncurses required).
//
// One frame, one burst. The streamer is brought up, the IQ buffer is
// pushed in chunks of max_samps_per_buff with end-of-burst on the last
// chunk, and then the streamer is torn down. No retries, no retuning.
//
// CLI defaults match the FrontierSat carrier (436.150 MHz, the same
// default as radio_ctl). FrontierSat is simplex — uplink and downlink
// share 436.150 MHz — so the same default works for both transmitting
// to the satellite and bench-looping back to the FT-991A on the
// operator desk. Override with --freq-hz= for any other test frequency.

#include "csp.h"
#include "ax100.h"
#include "modem.h"
#include "hmac_keyfile.h"
#include "frontiersat.h"  // FRONTIERSAT_CARRIER_HZ for help text only
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_operator.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include <uhd/types/tune_request.h>
#include <uhd/types/metadata.h>
#include <uhd/error.h>

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Parses tolerant hex: ignores whitespace and ':' separators.
static ssize_t parse_payload_hex(const char *hex, uint8_t *out, size_t out_cap)
{
    size_t n = 0;
    int hi = -1;
    for (const char *p = hex; *p != '\0'; p++) {
        if (isspace((unsigned char)*p) || *p == ':') continue;
        int d = hex_digit(*p);
        if (d < 0) {
            fprintf(stderr, "tx_frame_sdr: bad hex char '%c'\n", *p);
            return -1;
        }
        if (hi < 0) hi = d;
        else {
            if (n >= out_cap) { fprintf(stderr, "tx_frame_sdr: hex too long\n"); return -1; }
            out[n++] = (uint8_t)((hi << 4) | d);
            hi = -1;
        }
    }
    if (hi >= 0) { fprintf(stderr, "tx_frame_sdr: odd number of hex digits\n"); return -1; }
    return (ssize_t)n;
}

static void usage(FILE *f, const char *argv0)
{
    fprintf(f,
        "usage: %s (--payload-hex=<HEX> | --payload-ascii=<STR>) [options]\n"
        "\n"
        "Build a CSP+AX100 frame and transmit it via a USRP B210. The frame\n"
        "is FM-modulated in software (peak deviation = bit_rate/4 by default,\n"
        "so 2.4 kHz at 9600 baud — modulation index h = 0.5 to match the\n"
        "gr-satellites fsk_demodulator the gold-reference receiver uses) and\n"
        "pushed to the B210's TX streamer at --tx-rate.\n"
        "\n"
        "Payload (exactly one):\n"
        "  --payload-hex=<HEX>         Payload as hex (whitespace/':' ignored)\n"
        "  --payload-ascii=<STR>       Payload as literal ASCII bytes\n"
        "\n"
        "CSP / framing (defaults match cts_send to FrontierSat OBC):\n"
        "  --src=<n> --dst=<n>         CSP src/dst (defaults 10 = GCS, 1 = OBC)\n"
        "  --dport=<n> --sport=<n>     CSP dport/sport (defaults 7 = CTS1 cmd, 16)\n"
        "  --prio=<n>                  CSP priority 0..3 (default 2 = normal)\n"
        "  --no-hmac                   Disable HMAC (default ON, matches uplink)\n"
        "  --keyfile=<path>            HMAC keyfile (default $HOME/%s)\n"
        "  --no-reed-solomon           Disable RS(255,223) (default ON)\n"
        "\n"
        "Modem / FM:\n"
        "  --bit-rate=<bps>            Default 9600\n"
        "  --tx-rate=<sps>             B210 TX sample rate (default 480000;\n"
        "                              must be a multiple of --bit-rate so\n"
        "                              modem can render directly at this rate)\n"
        "  --deviation-hz=<hz>         FM peak deviation in Hz (default\n"
        "                              bit_rate/4 = 2400 Hz at 9600 baud)\n"
        "  --gauss-bt=<f>              Gaussian BT product (default 0 = no\n"
        "                              filter; rectangular NRZ matches the\n"
        "                              gr-satellites fsk_demodulator)\n"
        "  --gauss-span=<n>            Gaussian symbol span (default 4)\n"
        "\n"
        "Radio:\n"
        "  --freq-hz=<hz>              Carrier center freq (default %.0f Hz =\n"
        "                              FrontierSat simplex carrier — uplink\n"
        "                              and downlink share this frequency)\n"
        "  --gain-db=<n>               B210 TX gain dB, 0..89.75 (default 30)\n"
        "  --device-args=<str>         Accepted but ignored (offline IQ tool;\n"
        "                              opens no device)\n"
        "  --repeat=<n>                Send the same frame N times back-to-back\n"
        "                              with a small gap (default 1)\n"
        "  --gap-ms=<n>                Gap between repeats in ms (default 200)\n"
        "  --ramp-ms=<f>               Raised-cosine envelope ramp at the\n"
        "                              start and end of each modulated burst,\n"
        "                              in milliseconds (default 1.0). Smooths\n"
        "                              the on/off keying transition so the\n"
        "                              wide-band click that would otherwise\n"
        "                              splatter energy outside the receiver's\n"
        "                              IF passband doesn't trip its AGC.\n"
        "                              Set to 0 for instantaneous keying.\n"
        "  --joke                      Replace --repeat/--gap-ms with a\n"
        "                              \"da   da-da da  da, dah   dah!\"\n"
        "                              rhythm — five short bursts and two\n"
        "                              long ones (held carrier tail) with\n"
        "                              the gap timing scaled to the frame's\n"
        "                              on-air duration. Don't use for real\n"
        "                              telecommands. Plays once.\n"
        "  --preroll-ms=<n>            Modulated 0xAA tone in front of the\n"
        "                              AX100 frame (default 100). Acts as\n"
        "                              extra symbol-clock training tone for\n"
        "                              the receiver and lets the FPGA TX\n"
        "                              FIFO fully buffer the burst so a one-\n"
        "                              time host stall doesn't eat the\n"
        "                              real frame's preamble. Quantized to\n"
        "                              whole bytes so it stays bit-aligned\n"
        "                              with the frame's 0xAA prefill.\n"
        "  --postroll-ms=<n>           Constant-carrier pad after the last burst,\n"
        "                              before EOB (default 50).\n"
        "  --start-delay-s=<f>         Schedule the burst to start at this many\n"
        "                              seconds after the device clock is reset\n"
        "                              (default 0.5). Adds latency from invocation\n"
        "                              to first sample on-air; raise if you still\n"
        "                              see mid-burst underruns.\n"
        "\n"
        "Safety / dry-run:\n"
        "  --allow-tx                  Required to actually key the streamer\n"
        "                              (no upstream radio_t inhibit here, but\n"
        "                              we still ask for explicit consent)\n"
        "  --dump-iq=<path>            Dump the IQ buffer to a file instead\n"
        "                              of streaming (sc16 interleaved, no\n"
        "                              header). Useful with tx_samples_from_file\n"
        "                              for offline replay.\n"
        "  --dry-run                   Build the frame, print sizes, exit.\n"
        "  --help                      This message.\n",
        argv0, HMAC_KEYFILE_DEFAULT_RELPATH, FRONTIERSAT_CARRIER_HZ);
}

// FM-modulate a real-valued PCM input (S16_LE, normalized to [-1, +1]
// by /32767) to complex int16 IQ at the same sample rate. Phase is
// integrated continuously across the call so multiple chunks stitch
// without phase discontinuity (we only ever call this once per frame
// here, but keeping phi as a static state lets repeats stitch cleanly).
static void fm_modulate_pcm_to_iq(const int16_t *pcm, size_t n_pcm,
                                  double dev_hz, double fs,
                                  int16_t *iq_out)
{
    static double phi = 0.0;
    const double k = 2.0 * M_PI * dev_hz / fs;
    const double inv_full_scale = 1.0 / 32767.0;
    for (size_t i = 0; i < n_pcm; i++) {
        double x = (double)pcm[i] * inv_full_scale;
        phi += k * x;
        // Wrap phase to [-pi, pi] so it never overflows.
        if (phi >  M_PI) phi -= 2.0 * M_PI;
        if (phi < -M_PI) phi += 2.0 * M_PI;
        double ci = cos(phi);
        double cq = sin(phi);
        // Scale to ~70% of full scale to leave headroom for the
        // baseband DAC / IQ-imbalance compensation. 32767 * 0.7 ≈ 22937.
        iq_out[2 * i + 0] = (int16_t)lround(ci * 22937.0);
        iq_out[2 * i + 1] = (int16_t)lround(cq * 22937.0);
    }
}

// Apply a raised-cosine envelope ramp to a modulated burst already
// written into the IQ buffer. The first ramp_n samples are scaled by
// 0.5*(1 - cos(pi*i/ramp_n)) (smooth 0->1 ramp); the last ramp_n
// samples are scaled by 0.5*(1 + cos(pi*i/ramp_n)) (smooth 1->0).
// Without this, the IQ envelope steps from 0 to unit magnitude at
// burst start and back to 0 at burst end, producing wide-band keying
// clicks that splatter energy outside the receiver's IF passband and
// trip its AGC. ramp_n is clamped to half the burst length so the
// two ramps don't overlap.
static void apply_envelope_ramp(int16_t *iq_burst, size_t n_samps, size_t ramp_n)
{
    if (ramp_n == 0) return;
    if (ramp_n > n_samps / 2) ramp_n = n_samps / 2;
    for (size_t i = 0; i < ramp_n; i++) {
        double env_in  = 0.5 * (1.0 - cos(M_PI * (double)i / (double)ramp_n));
        double env_out = 0.5 * (1.0 + cos(M_PI * (double)i / (double)ramp_n));
        // Ramp up at the start of the burst.
        iq_burst[2 * i + 0] = (int16_t)lround((double)iq_burst[2 * i + 0] * env_in);
        iq_burst[2 * i + 1] = (int16_t)lround((double)iq_burst[2 * i + 1] * env_in);
        // Ramp down at the end of the burst.
        size_t k = n_samps - ramp_n + i;
        iq_burst[2 * k + 0] = (int16_t)lround((double)iq_burst[2 * k + 0] * env_out);
        iq_burst[2 * k + 1] = (int16_t)lround((double)iq_burst[2 * k + 1] * env_out);
    }
}

// "da   da-da da  da, dah   dah!" — encoded as a list of (is_dah, gap)
// pairs. Each "da" is a single frame burst; each "dah" is a frame
// burst followed by held-carrier samples (a steady CW tone at the LO
// frequency, by clamping the IQ at its last value). gap_after_units
// is in units of one frame's on-air duration so the rhythm naturally
// scales with payload size — a longer payload makes each "da" longer
// and the gaps between them stretch by the same factor.
static const struct {
    int is_dah;
    double gap_after_units;
} JOKE_PATTERN[] = {
    {0, 3.0},   // "da" + 3-space gap
    {0, 0.2},   // "da" (joined by '-' to the next; very short gap)
    {0, 1.0},   // "da" + 1-space gap
    {0, 2.0},   // "da" + 2-space gap
    {0, 3.0},   // "da" + ", " (comma + space)
    {1, 3.0},   // "dah" + 3-space gap
    {1, 0.0},   // "dah!" (final, no trailing gap)
};
#define JOKE_N (sizeof JOKE_PATTERN / sizeof JOKE_PATTERN[0])
#define JOKE_DAH_HOLD_UNITS 2.0   // dah = 1 frame + this many frame-durations of CW tail

int main(int argc, char **argv)
{
    const char *payload_hex = NULL;
    const char *payload_ascii = NULL;
    const char *keyfile_path = NULL;
    const char *dump_iq_path = NULL;
    double freq_hz = FRONTIERSAT_CARRIER_HZ;
    double gain_db = 30.0;
    double tx_rate = 480000.0;
    // Default = bit_rate / 4 = 2400 Hz at 9600 bps. This produces an FM
    // modulation index h = 2*fdev/baud = 0.5 (MSK-like), which is what
    // the gold-reference gr-satellites fsk_demodulator in
    // gnu_radio/usrp_b210_gnu_radio/radio_ax100.py expects. h above 0.5
    // (e.g. 3000 Hz at 9600 baud → h = 0.625) widens the eye in a way
    // the demodulator does not match-filter for and reduces SNR margin.
    double deviation_hz = -1.0;
    int use_hmac = 1;
    int use_rs = 1;
    int allow_tx = 0;
    int dry_run = 0;
    // Operator gating: tx_frame_sdr keys the PA, so it refuses to run
    // unless simple_sat_ops is in operator mode with $USER == that
    // operator's user. --no-control-check skips the gate (dev/test
    // only; logged as such in runs.log).
    int no_control_check = 0;
    int repeat = 1;
    int gap_ms = 200;
    int preroll_ms = 100;
    int postroll_ms = 50;
    int joke = 0;
    double start_delay_s = 0.5;
    double ramp_ms = 1.0;

    // Defaults match the cts_send gold reference in
    // CalgaryToSpace/CTS-SAT-1-Ground-Station/gnu_radio/supporting_demo_scripts/pycsp_tx.py:
    // src = GCS_ADDR (10), dst = OBC_ADDR (1), dport = 7 (CTS1 command
    // handler), sport = 16, prio = norm. CSP-layer flags stay 0 — the
    // HMAC trailer is added at the AX100 layer, not inside the packet.
    csp_v1_header_t csp_hdr = {
        .prio = CSP_PRIO_NORM,
        .src = 10, .dst = 1,
        .dport = 7, .sport = 16,
        .flags = 0,
    };
    modem_params_t mp;
    modem_params_defaults(&mp);
    // Render baseband directly at the B210 TX rate so we don't need a
    // separate upsampling step. modem.c requires samp_rate be a multiple
    // of bit_rate; the default tx_rate = 480000 / bit_rate = 9600 → 50
    // samples per symbol. Plenty for the Gaussian filter to do its job.
    mp.samp_rate = (int)tx_rate;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0)                   { usage(stdout, argv[0]); return 0; }
        else if (starts_with(a, "--payload-hex="))      payload_hex   = a + 14;
        else if (starts_with(a, "--payload-ascii="))    payload_ascii = a + 16;
        else if (starts_with(a, "--keyfile="))          keyfile_path  = a + 10;
        else if (strcmp(a, "--no-hmac") == 0)           use_hmac = 0;
        else if (strcmp(a, "--reed-solomon") == 0)      use_rs = 1;
        else if (strcmp(a, "--no-reed-solomon") == 0)   use_rs = 0;
        else if (starts_with(a, "--src="))     csp_hdr.src   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dst="))     csp_hdr.dst   = (uint8_t)atoi(a + 6);
        else if (starts_with(a, "--dport="))   csp_hdr.dport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--sport="))   csp_hdr.sport = (uint8_t)atoi(a + 8);
        else if (starts_with(a, "--prio="))    csp_hdr.prio  = (uint8_t)atoi(a + 7);
        else if (starts_with(a, "--bit-rate=")) {
            int bps = atoi(a + 11);
            if (bps <= 0) { fprintf(stderr, "--bit-rate must be positive\n"); return 1; }
            mp.bit_rate = bps;
        }
        else if (starts_with(a, "--tx-rate="))          tx_rate      = atof(a + 10);
        else if (starts_with(a, "--deviation-hz="))     deviation_hz = atof(a + 15);
        else if (starts_with(a, "--gauss-bt="))         mp.gauss_bt  = atof(a + 11);
        else if (starts_with(a, "--gauss-span="))       mp.gauss_symbol_span = atoi(a + 13);
        else if (starts_with(a, "--freq-hz="))          freq_hz      = atof(a + 10);
        else if (starts_with(a, "--gain-db="))          gain_db      = atof(a + 10);
        else if (starts_with(a, "--device-args="))      { /* accepted but ignored: this tool renders IQ offline and never opens the B210 */ }
        else if (starts_with(a, "--repeat="))           repeat       = atoi(a + 9);
        else if (starts_with(a, "--gap-ms="))           gap_ms       = atoi(a + 9);
        else if (starts_with(a, "--preroll-ms="))       preroll_ms   = atoi(a + 13);
        else if (starts_with(a, "--postroll-ms="))      postroll_ms  = atoi(a + 14);
        else if (starts_with(a, "--start-delay-s="))    start_delay_s = atof(a + 16);
        else if (starts_with(a, "--ramp-ms="))           ramp_ms      = atof(a + 10);
        else if (starts_with(a, "--dump-iq="))          dump_iq_path = a + 10;
        else if (strcmp(a, "--joke") == 0)              joke = 1;
        else if (strcmp(a, "--allow-tx") == 0)          allow_tx = 1;
        else if (strcmp(a, "--dry-run") == 0)           dry_run = 1;
        else if (strcmp(a, "--no-control-check") == 0)  no_control_check = 1;
        else { fprintf(stderr, "tx_frame_sdr: unknown arg '%s'\n", a); usage(stderr, argv[0]); return 1; }
    }

    if ((payload_hex == NULL) == (payload_ascii == NULL)) {
        fprintf(stderr, "tx_frame_sdr: pass exactly one of --payload-hex or --payload-ascii\n");
        return 1;
    }

    // Live RF streaming moved to b210_rx_tx (driven by simple_sat_ops's
    // TX compose modal). tx_frame_sdr is now an offline IQ-rendering
    // tool: --dry-run prints frame sizes, --dump-iq writes the sc16 IQ
    // buffer to disk. The streamer path is intentionally disabled so
    // tx_frame_sdr can't race b210_rx_tx for the B210 device.
    if (!dry_run && dump_iq_path == NULL) {
        fprintf(stderr,
            "tx_frame_sdr: live streaming has moved to b210_rx_tx.\n"
            "  Use simple_sat_ops's TX compose modal (press 't' while the\n"
            "  keyboard is unlocked) to send a frame on the air.\n"
            "  This tool now requires --dry-run or --dump-iq=<path>.\n");
        return 2;
    }
    mp.samp_rate = (int)tx_rate;
    if (deviation_hz < 0.0) {
        deviation_hz = (double)mp.bit_rate / 4.0;
    }
    if (mp.samp_rate <= 0 || mp.bit_rate <= 0 || mp.samp_rate % mp.bit_rate != 0) {
        fprintf(stderr, "tx_frame_sdr: --tx-rate (%d) must be a positive multiple of "
                        "--bit-rate (%d)\n", mp.samp_rate, mp.bit_rate);
        return 1;
    }
    if (repeat < 1) repeat = 1;
    if (gap_ms < 0) gap_ms = 0;

    // tx_frame_sdr is now offline-only — no operator gate. Audit so
    // the runs.log still records dump-iq / dry-run invocations.
    sso_audit_start("tx_frame_sdr",
                    dump_iq_path != NULL ? "dump-iq" : "dry-run");
    (void) no_control_check;  // --no-control-check is a no-op now

    // --- Build the wire frame ---

    uint8_t payload[2048];
    ssize_t payload_len = 0;
    if (payload_hex != NULL) {
        payload_len = parse_payload_hex(payload_hex, payload, sizeof payload);
        if (payload_len < 0) return 1;
    } else {
        size_t n = strlen(payload_ascii);
        if (n > sizeof payload) {
            fprintf(stderr, "tx_frame_sdr: payload too long\n"); return 1;
        }
        memcpy(payload, payload_ascii, n);
        payload_len = (ssize_t)n;
    }

    uint8_t hmac_key[128];
    ssize_t hmac_key_len = 0;
    if (use_hmac) {
        char default_path[512];
        const char *path = keyfile_path;
        if (path == NULL) {
            if (hmac_keyfile_default_path(default_path, sizeof default_path) != 0) {
                fprintf(stderr, "tx_frame_sdr: HOME unset; pass --keyfile=<path>\n");
                return 1;
            }
            path = default_path;
        }
        hmac_key_len = hmac_keyfile_load(path, hmac_key, sizeof hmac_key);
        if (hmac_key_len < 0) return 1;
    }

    uint8_t csp_packet[4096];
    ssize_t csp_len = csp_v1_encode(&csp_hdr, payload, (size_t)payload_len,
                                    csp_packet, sizeof csp_packet);
    if (csp_len < 0) { fprintf(stderr, "csp_v1_encode failed\n"); return 1; }

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (use_hmac) {
        opts.hmac_key = hmac_key;
        opts.hmac_key_len = (size_t)hmac_key_len;
    }
    opts.reed_solomon = use_rs;
    uint8_t frame[4200];
    ssize_t frame_len = ax100_frame(csp_packet, (size_t)csp_len, &opts, frame, sizeof frame);
    if (frame_len < 0) { fprintf(stderr, "ax100_frame failed\n"); return 1; }

    int sps = mp.samp_rate / mp.bit_rate;
    size_t n_pcm = (size_t)frame_len * 8u * (size_t)sps;
    int16_t *pcm = (int16_t *)malloc(n_pcm * sizeof(int16_t));
    if (pcm == NULL) { fprintf(stderr, "tx_frame_sdr: out of memory for PCM\n"); return 1; }
    ssize_t got = modem_bytes_to_pcm16(frame, (size_t)frame_len, &mp, pcm, n_pcm);
    if (got < 0) { fprintf(stderr, "modem_bytes_to_pcm16 failed\n"); free(pcm); return 1; }

    // Buffer layout: [preroll][mod_1][gap][mod_2][gap]...[mod_N][postroll]
    //
    // The pre-roll is filled with modulated 0xAA bytes — same alternating
    // 1010... bit pattern as the AX100 frame's own 32-byte 0xAA prefill,
    // just longer. This serves two purposes at once: it gives the FPGA
    // TX FIFO time to fully buffer the burst (so a one-time host
    // scheduling stall at burst start can't eat the actual frame), AND
    // it gives the receiver's symbol-clock recovery a long training
    // tone to lock onto before the ASM arrives. The earlier "silent
    // pre-roll" version filled this region with IQ=0, which buffers the
    // FIFO but does nothing for clock recovery — and at the gold-
    // reference modulation index h=0.5, the AX100 frame's 32 prefill
    // bytes alone (~3.3 ms at 9600 baud) is right on the edge of
    // adequate.
    //
    // The post-roll stays silent — it just gives the AD9361 a clean
    // settle before EOB drains the FIFO; the receiver is done with the
    // frame by then.
    //
    // The FM modulator state (phi) is static, so calling it once per
    // burst stitches phase across the gap — no glitch when the next
    // burst starts.
    //
    // --joke replaces the simple repeat-and-gap layout with a
    // "da   da-da da  da, dah   dah!" rhythm. Each "da" is one frame;
    // each "dah" is a frame plus held-carrier (steady CW tone) tail.

    // Quantize the pre-roll window to a whole number of bit periods so
    // we can generate it by running the modem on N bytes of 0xAA. Each
    // byte modulates to (8 * sps) PCM samples; rounding *down* keeps
    // pre-roll <= the user-requested duration.
    size_t preroll_bytes = ((size_t)preroll_ms * (size_t)mp.samp_rate / 1000)
                            / (8 * (size_t)sps);
    size_t preroll_samps  = preroll_bytes * 8 * (size_t)sps;
    size_t postroll_samps = (size_t)((double)mp.samp_rate * (double)postroll_ms / 1000.0);
    size_t ramp_samps     = (size_t)((double)mp.samp_rate * ramp_ms             / 1000.0);

    // Pre-roll PCM: N bytes of 0xAA run through the same modem. Phase
    // continuity with the burst PCM that follows is provided by the
    // static phi state inside fm_modulate_pcm_to_iq — calling it for
    // the pre-roll first, then for the burst, is seamless.
    int16_t *preroll_pcm = NULL;
    if (preroll_samps > 0) {
        uint8_t *preroll_bytes_buf = (uint8_t *)malloc(preroll_bytes);
        if (preroll_bytes_buf == NULL) {
            fprintf(stderr, "tx_frame_sdr: out of memory for preroll bytes\n");
            free(pcm); return 1;
        }
        memset(preroll_bytes_buf, 0xAA, preroll_bytes);
        preroll_pcm = (int16_t *)malloc(preroll_samps * sizeof(int16_t));
        if (preroll_pcm == NULL) {
            fprintf(stderr, "tx_frame_sdr: out of memory for preroll PCM\n");
            free(preroll_bytes_buf); free(pcm); return 1;
        }
        ssize_t pgot = modem_bytes_to_pcm16(preroll_bytes_buf, preroll_bytes,
                                            &mp, preroll_pcm, preroll_samps);
        free(preroll_bytes_buf);
        if (pgot < 0) {
            fprintf(stderr, "tx_frame_sdr: preroll modem_bytes_to_pcm16 failed\n");
            free(preroll_pcm); free(pcm); return 1;
        }
    }

    size_t n_iq_total;
    int16_t *iq = NULL;

    if (joke) {
        // Pre-compute total samples needed for the rhythm pattern.
        size_t unit_samps = n_pcm;  // one rhythmic unit = one frame on-air
        size_t hold_samps = (size_t)(JOKE_DAH_HOLD_UNITS * (double)unit_samps);
        size_t joke_samps = 0;
        for (size_t i = 0; i < JOKE_N; i++) {
            joke_samps += n_pcm;                                   // frame
            if (JOKE_PATTERN[i].is_dah) joke_samps += hold_samps;  // CW tail
            joke_samps += (size_t)(JOKE_PATTERN[i].gap_after_units * (double)unit_samps);
        }
        n_iq_total = preroll_samps + joke_samps + postroll_samps;
        iq = (int16_t *)calloc(n_iq_total * 2, sizeof(int16_t));
        if (iq == NULL) {
            fprintf(stderr, "tx_frame_sdr: out of memory for %zu IQ samples\n", n_iq_total);
            free(preroll_pcm); free(pcm); return 1;
        }

        // Modulate the pre-roll 0xAA tone into the IQ buffer first so the
        // first burst's envelope ramp covers the pre-roll → frame
        // transition without a discontinuity.
        if (preroll_pcm != NULL) {
            fm_modulate_pcm_to_iq(preroll_pcm, preroll_samps, deviation_hz,
                                  (double)mp.samp_rate, iq);
        }
        size_t off = preroll_samps;
        for (size_t i = 0; i < JOKE_N; i++) {
            // For the first burst, extend the ramp region back through
            // the pre-roll so the keying ramp-up sweeps the whole 0xAA
            // tone-and-frame as one continuous burst.
            size_t burst_start = (i == 0) ? 0 : off;
            // Modulate the frame.
            fm_modulate_pcm_to_iq(pcm, n_pcm, deviation_hz,
                                  (double)mp.samp_rate, iq + off * 2);
            off += n_pcm;
            // For "dah", clamp the next hold_samps IQ entries to the
            // last (I, Q) value the modulator emitted. Since the
            // modulator's phi is static and undriven (audio = 0)
            // would mean no further phase change, holding the last
            // sample produces a clean CW tone at the LO frequency
            // for the duration of the hold.
            if (JOKE_PATTERN[i].is_dah) {
                int16_t last_i = iq[(off - 1) * 2 + 0];
                int16_t last_q = iq[(off - 1) * 2 + 1];
                for (size_t k = 0; k < hold_samps; k++) {
                    iq[(off + k) * 2 + 0] = last_i;
                    iq[(off + k) * 2 + 1] = last_q;
                }
                off += hold_samps;
            }
            // Apply the envelope ramp across the whole burst (frame
            // plus any held-carrier tail) so on/off transitions don't
            // splatter energy outside the receiver's IF.
            apply_envelope_ramp(iq + burst_start * 2, off - burst_start,
                                ramp_samps);
            // Gap region stays zero (silence) from calloc.
            off += (size_t)(JOKE_PATTERN[i].gap_after_units * (double)unit_samps);
        }
    } else {
        size_t gap_samps = (size_t)((double)mp.samp_rate * (double)gap_ms / 1000.0);
        size_t n_iq_per_rep = n_pcm + gap_samps;
        n_iq_total = preroll_samps + n_iq_per_rep * (size_t)repeat + postroll_samps;
        iq = (int16_t *)calloc(n_iq_total * 2, sizeof(int16_t));
        if (iq == NULL) {
            fprintf(stderr, "tx_frame_sdr: out of memory for %zu IQ samples\n", n_iq_total);
            free(preroll_pcm); free(pcm); return 1;
        }
        // Modulate the pre-roll 0xAA tone in front of the first burst.
        if (preroll_pcm != NULL) {
            fm_modulate_pcm_to_iq(preroll_pcm, preroll_samps, deviation_hz,
                                  (double)mp.samp_rate, iq);
        }
        for (int r = 0; r < repeat; r++) {
            size_t off = preroll_samps + (size_t)r * n_iq_per_rep;
            fm_modulate_pcm_to_iq(pcm, n_pcm, deviation_hz, (double)mp.samp_rate,
                                  iq + off * 2);
            // Smooth the on/off transition so wide-band keying clicks
            // don't trip the receiver's AGC. For the first burst, sweep
            // the ramp back through the pre-roll so the entire pre-roll +
            // frame turns on as one continuous burst.
            size_t burst_start_samp = (r == 0) ? 0 : off;
            size_t burst_n_samps    = (r == 0) ? (preroll_samps + n_pcm) : n_pcm;
            apply_envelope_ramp(iq + burst_start_samp * 2, burst_n_samps,
                                ramp_samps);
            // Gap region stays zero (silence) from calloc.
        }
    }

    double tx_seconds = (double)n_iq_total / (double)mp.samp_rate;
    fprintf(stderr,
            "tx_frame_sdr: payload=%zd csp=%zd frame=%zd bytes; "
            "PCM=%zu samples (%.0fHz); IQ=%zu samples (preroll=%zu, "
            "%s, postroll=%zu); %.3f s on-air @ %.3f MHz, "
            "gain %.1f dB, dev %.0f Hz, start_delay=%.3f s\n",
            payload_len, csp_len, frame_len, n_pcm, (double)mp.samp_rate,
            n_iq_total, preroll_samps,
            joke ? "joke rhythm: 5×da + 2×dah" : (repeat == 1 ? "1×burst" : "N×burst+gap"),
            postroll_samps,
            tx_seconds, freq_hz / 1e6, gain_db, deviation_hz,
            start_delay_s);

    if (dry_run) {
        fprintf(stderr, "tx_frame_sdr: --dry-run, not opening device\n");
        free(iq); free(preroll_pcm); free(pcm); return 0;
    }

    if (dump_iq_path != NULL) {
        FILE *fp = fopen(dump_iq_path, "wb");
        if (fp == NULL) { perror("tx_frame_sdr: fopen --dump-iq"); free(iq); free(preroll_pcm); free(pcm); return 1; }
        size_t wrote = fwrite(iq, sizeof(int16_t) * 2, n_iq_total, fp);
        fclose(fp);
        if (wrote != n_iq_total) {
            fprintf(stderr, "tx_frame_sdr: short write to %s\n", dump_iq_path);
            free(iq); free(preroll_pcm); free(pcm); return 1;
        }
        fprintf(stderr, "tx_frame_sdr: wrote %zu IQ samples to %s (sc16 interleaved)\n",
                n_iq_total, dump_iq_path);
        free(iq); free(preroll_pcm); free(pcm);
        return 0;
    }

    // Live streaming has moved to b210_rx_tx (driven by
    // simple_sat_ops's TX compose modal). The earlier --dry-run /
    // --dump-iq gate already short-circuited any path that would
    // reach this point; the unreachable streamer code that used to
    // live here was removed as dead. tx_frame_sdr now exists purely
    // for offline IQ rendering / replay.
    (void) allow_tx;  // accepted on the CLI for backwards compatibility
    free(iq); free(preroll_pcm); free(pcm);
    return 0;
}
