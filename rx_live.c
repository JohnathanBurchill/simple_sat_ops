/*

    Simple Satellite Operations  rx_live.c

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Live AX100 decoder over ALSA capture. Reads audio continuously from
// the SignaLink (default) or radio's native USB CODEC, runs the rx_decode
// pipeline on sliding overlapping windows, and prints each decoded frame
// to stdout with a UTC timestamp. Optionally appends frames to a log
// file and tees the raw audio to a .raw for post-mortem rx_decode.
//
// Sibling of rx_decode (offline, file-based) and rx_capture (capture
// only, no decode). Decode latency is approximately window_s; throughput
// is gated by ALSA, so the loop runs in real time.

#include "audio.h"
#include "ax100.h"
#include "csp.h"
#include "hmac_keyfile.h"
#include "modem.h"

#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void usage(FILE *dest, const char *name)
{
    fprintf(dest,
        "usage: %s [options]\n"
        "\n"
        "Live AX100 decoder over ALSA capture. Reads audio continuously\n"
        "from the SignaLink (default) or radio's native USB CODEC, runs\n"
        "the rx_decode pipeline on sliding overlapping windows, and\n"
        "prints each decoded frame to stdout with a UTC timestamp.\n"
        "Sibling of rx_decode (offline) and rx_capture (capture only).\n"
        "\n"
        "Audio source:\n"
        "  --radio-audio            (default) Auto-detect radio's native\n"
        "                           USB CODEC. The FT-991A's discriminator\n"
        "                           digital tap is the path that decodes\n"
        "                           cleanly at 9600 baud.\n"
        "  --signalink-audio        Auto-detect SignaLink USB (TI Burr-\n"
        "                           Brown PCM2901/PCM2904). Doesn't work\n"
        "                           reliably for 9600-baud RX through the\n"
        "                           FT-991A's rear DATA jack — use the\n"
        "                           radio's USB CODEC instead.\n"
        "  --audio-device=<name>    Explicit ALSA PCM device, e.g. plughw:1,0.\n"
        "  --backend-hint=<id>      yaesu|icom for --radio-audio.\n"
        "\n"
        "Decoder:\n"
        "  --bit-rate=<bps>         Default 9600.\n"
        "  --window-s=<seconds>     Decoder window size (default 1.5).\n"
        "                           Must comfortably contain a full frame\n"
        "                           plus preamble (~300 ms at 9600 bps).\n"
        "  --slide-s=<seconds>      How far to slide between decode\n"
        "                           attempts (default 0.5; smaller =\n"
        "                           catches more edge-straddling frames,\n"
        "                           costs CPU).\n"
        "  --sync-threshold=<0..8>  Max ASM bit errors (default 4).\n"
        "  --hmac                   Enable HMAC verification. AX100\n"
        "                           downlink frames do NOT use HMAC, so\n"
        "                           this is OFF by default. Use only when\n"
        "                           round-tripping uplink_test output.\n"
        "  --keyfile=<path>         HMAC keyfile (only relevant with\n"
        "                           --hmac; default $HOME/%s).\n"
        "  --reed-solomon           RS(255,223) decode (DEFAULT).\n"
        "  --no-reed-solomon        Skip RS decode (use when decoding\n"
        "                           downlink-style CRC frames).\n"
        "  --no-dc-block            Skip the modem's DC-block IIR. Useful\n"
        "                           on radio digital taps (FT-991A USB\n"
        "                           CODEC) where there's no DC offset.\n"
        "  --csp-crc32              Validate + strip a trailing CSP zlib\n"
        "                           CRC32 (libcsp CRC mode). Off by\n"
        "                           default. AX100 frames in either\n"
        "                           direction don't necessarily carry\n"
        "                           one (uplink has HMAC; downlink\n"
        "                           depends on firmware). Enable only\n"
        "                           when you know the TX side appends\n"
        "                           a CRC; otherwise frames will fail\n"
        "                           validation and be silently dropped.\n"
        "\n"
        "Output (defaults: ON, auto-named\n"
        "        rx_live_UT=YYYYMMDDTHHMMSS.sss.{log,raw,wav,png} in CWD):\n"
        "  --log=<path>             Override log path. Re-opened per frame\n"
        "                           so log-rotation works (mv the log\n"
        "                           mid-run, next frame creates a fresh\n"
        "                           file).\n"
        "  --raw=<path>             Override headerless S16_LE raw path.\n"
        "  --wav=<path>             Override WAV path.\n"
        "  --no-log                 Skip the decode log.\n"
        "  --no-raw                 Skip the .raw companion.\n"
        "  --no-wav                 Skip the .wav companion (also skips\n"
        "                           the spectrogram).\n"
        "  --no-spectrogram         Skip the end-of-session spectrogram\n"
        "                           PNG (rendered from the WAV via\n"
        "                           ffmpeg showspectrumpic on Ctrl-C).\n"
        "  --quiet                  Skip stdout output (log-only mode).\n"
        "  --help                   Show this help.\n",
        name, HMAC_KEYFILE_DEFAULT_RELPATH);
}

static void fmt_utc(char *buf, size_t cap)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm utc;
    gmtime_r(&tv.tv_sec, &utc);
    snprintf(buf, cap, "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
             utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
             utc.tm_hour, utc.tm_min, utc.tm_sec,
             (long)(tv.tv_usec / 1000));
}

// Multi-hypothesis sync search on a single window of mono int16 samples.
// Same logic as rx_decode.c's main decode loop — try sync, on failure
// advance min_offset past the bad match and try again. With HMAC enabled,
// HMAC mismatch is treated as a false-positive sync (advance + retry).
// Returns 1 on a successful decode (packet/packet_len/* filled), 0 if no
// frame found. The caller may loop on this function with min_offset_in
// advanced past the previously-returned sync_off to extract multiple
// frames per window (e.g. when bursts arrive faster than the slide
// interval); *out_sync_off receives the offset of the accepted sync
// match so the caller can compute the next min_offset_in.
//
// rs_errs sentinels (out_rs_errs):
//   >= 0  RS-corrected; value is the count of corrected byte errors
//   -1    RS off / not exercised
//   -2    RS uncorrectable, descrambled-but-uncorrected bytes returned
//         via packet/packet_len so the operator can still see the data
//         (only happens when allow_partial_rs is on and HMAC is off).
static int try_decode_window(const int16_t *samples, size_t n_samples,
                             const modem_params_t *mp,
                             const ax100_opts_t *opts,
                             int sync_max_ham, int use_hmac,
                             int allow_partial_rs,
                             size_t min_offset_in,
                             uint8_t *bits_scratch, size_t bits_cap,
                             uint8_t *bytes_scratch, size_t bytes_cap,
                             uint8_t *packet, size_t packet_cap,
                             ssize_t *out_packet_len,
                             int *out_golay_errs, int *out_hmac_ok,
                             int *out_rs_errs, int *out_used_golay_len,
                             size_t *out_sync_off)
{
    // Was 32; bumped to 256 because the cross-phase best-Hamming search
    // can flounder through many low-HD noise candidates between real
    // frames in a long-packet window before finding the next real ASM.
    const int MAX_ATTEMPTS = 256;
    size_t min_offset = min_offset_in;
    int attempts = 0;

    while (attempts < MAX_ATTEMPTS) {
        size_t n_bits = 0, sync_off = 0;
        int polarity_used = -1;
        int rc = modem_pcm16_to_bits(samples, n_samples, mp,
                                     0, sync_max_ham, min_offset,
                                     bits_scratch, &n_bits,
                                     &sync_off, &polarity_used);
        if (rc != 0) break;
        ++attempts;
        if (n_bits == 0) {
            min_offset = sync_off + 1;
            continue;
        }
        size_t need_bytes = (n_bits + 7) / 8;
        if (need_bytes > bytes_cap) {
            // Too much data after sync — clamp by advancing past this match
            // rather than overflowing scratch.
            min_offset = sync_off + 1;
            continue;
        }
        size_t n_bytes = modem_bits_to_bytes(bits_scratch, n_bits,
                                             bytes_scratch);
        ssize_t plen = ax100_unframe(bytes_scratch, n_bytes, opts,
                                     packet, packet_cap,
                                     out_golay_errs, out_hmac_ok,
                                     out_rs_errs, out_used_golay_len);
        (void)polarity_used;
        if (plen < 0) {
            // Partial-decode rescue: RS(255,223) gives up at >16 byte
            // errors, but for long bursts on a marginal link the
            // descrambled bytes are still mostly readable. When the
            // Golay header decoded perfectly (errs==0) we trust the
            // sync was real; retry with RS disabled to recover the
            // uncorrected payload, mark it rs=-2 (UNCORRECTABLE), and
            // hand it up so the operator sees "Happy birthday..." with
            // a few flipped bytes instead of nothing at all. Off in
            // HMAC mode (no integrity gate would mean we'd emit
            // garbage at every false sync hit) and never overrides a
            // good RS decode.
            if (allow_partial_rs && !use_hmac && opts->reed_solomon
                && *out_golay_errs == 0) {
                ax100_opts_t partial_opts = *opts;
                partial_opts.reed_solomon = 0;
                int p_golay = 0, p_hmac = -1, p_rs = -1, p_lensrc = -1;
                ssize_t pp = ax100_unframe(bytes_scratch, n_bytes,
                                           &partial_opts,
                                           packet, packet_cap,
                                           &p_golay, &p_hmac,
                                           &p_rs, &p_lensrc);
                // Strip the trailing 32-byte RS parity (it's noise to
                // the operator and would push the apparent length up
                // by 32 vs. the as-transmitted packet). Anything <=32
                // would underflow; treat as no usable bytes.
                if (pp > 32) {
                    *out_packet_len = pp - 32;
                    *out_golay_errs = p_golay;
                    *out_hmac_ok = -1;
                    *out_rs_errs = -2;
                    *out_used_golay_len = p_lensrc;
                    if (out_sync_off) *out_sync_off = sync_off;
                    return 1;
                }
            }
            min_offset = sync_off + 1;
            continue;
        }
        if (use_hmac && *out_hmac_ok == 0) {
            min_offset = sync_off + 1;
            continue;
        }
        *out_packet_len = plen;
        if (out_sync_off) *out_sync_off = sync_off;
        return 1;
    }
    return 0;
}

// Append-only log line writer. Re-opens for each frame so log rotation
// works (mv the log mid-run, next frame creates a fresh file). Mirrored
// to stdout if not quiet.
static void emit_frame(const char *log_path, int quiet, const char *ts,
                       const uint8_t *packet, size_t packet_len,
                       int golay_errs, int hmac_ok, int use_hmac,
                       int rs_errs, int used_golay_len,
                       int crc_status,
                       uint32_t crc_computed, uint32_t crc_le, uint32_t crc_be)
{
    char rs_buf[32];
    if (rs_errs == -2)      snprintf(rs_buf, sizeof rs_buf, "UNCORRECTABLE");
    else if (rs_errs < 0)   snprintf(rs_buf, sizeof rs_buf, "off");
    else                    snprintf(rs_buf, sizeof rs_buf, "corrected=%d", rs_errs);

    csp_v1_header_t hdr = {0};
    int csp_ok = (packet_len >= 4) && (csp_v1_decode(packet, &hdr) == 0);
    const uint8_t *payload = csp_ok ? packet + 4 : NULL;
    size_t payload_len = csp_ok ? packet_len - 4 : 0;

    FILE *streams[2] = { quiet ? NULL : stdout, NULL };
    FILE *log_fp = NULL;
    if (log_path != NULL) {
        log_fp = fopen(log_path, "a");
        if (log_fp != NULL) streams[1] = log_fp;
    }

    const char *len_src =
        used_golay_len == 1 ? "golay-header"
        : used_golay_len == 0 ? "brute-forced" : "(n/a)";
    for (int s = 0; s < 2; s++) {
        FILE *fp = streams[s];
        if (fp == NULL) continue;
        if (use_hmac) {
            fprintf(fp, "[%s] AX100: golay_errors=%d hmac=%s rs=%s len=%s "
                    "len_bytes=%zu\n",
                    ts, golay_errs,
                    hmac_ok == 1 ? "ok" : hmac_ok == 0 ? "MISMATCH" : "(off)",
                    rs_buf, len_src, packet_len);
        } else {
            fprintf(fp, "[%s] AX100: golay_errors=%d rs=%s len=%s "
                    "len_bytes=%zu\n",
                    ts, golay_errs, rs_buf, len_src, packet_len);
        }
        // CRC notice (only when --csp-crc32 was active for this frame).
        // Mismatch never drops the frame; the trailer stays in payload
        // so the operator can inspect what was received.
        if (crc_status == 1) {
            fprintf(fp, "[%s] csp_crc32: ok (0x%08x, 4-byte trailer "
                    "stripped)\n", ts, crc_computed);
        } else if (crc_status == 0) {
            fprintf(fp, "[%s] csp_crc32: MISMATCH "
                    "(computed=0x%08x, trailer LE=0x%08x BE=0x%08x; "
                    "trailer kept in payload)\n",
                    ts, crc_computed, crc_le, crc_be);
        }
        if (csp_ok) {
            fprintf(fp, "[%s] CSP v1: src=%u dst=%u dport=%u sport=%u "
                    "prio=%u flags=0x%02x\n",
                    ts, hdr.src, hdr.dst, hdr.dport, hdr.sport, hdr.prio,
                    hdr.flags);
            fprintf(fp, "[%s] hex:   ", ts);
            for (size_t i = 0; i < payload_len; i++) {
                fprintf(fp, "%02x", payload[i]);
            }
            fputc('\n', fp);
            fprintf(fp, "[%s] ascii: ", ts);
            for (size_t i = 0; i < payload_len; i++) {
                char c = (payload[i] >= 0x20 && payload[i] < 0x7F)
                    ? (char)payload[i] : '.';
                fputc(c, fp);
            }
            fputc('\n', fp);
        }
        fflush(fp);
    }

    if (log_fp != NULL) fclose(log_fp);
}

int main(int argc, char **argv)
{
    const char *audio_device = NULL;
    char audio_device_buf[64] = {0};
    int audio_pick = 2;  // default to --radio-audio (FT-991A USB CODEC)
    const char *backend_hint = NULL;
    const char *log_path = NULL;
    const char *raw_path = NULL;
    const char *wav_path = NULL;
    const char *keyfile_path = NULL;
    int bit_rate = 9600;
    double window_s = 1.5;
    double slide_s = 0.5;
    int sync_max_ham = 4;
    int use_hmac = 0;  // AX100 downlink does not use HMAC; opt in with --hmac
    int use_rs = 1;
    int no_dc_block = 0;
    int csp_crc32 = 0;  // opt-in via --csp-crc32 (some firmwares use it)
    int quiet = 0;
    int want_log = 1;
    int want_raw = 1;
    int want_wav = 1;
    int want_spectrogram = 1;
    char auto_log_path[300] = {0};
    char auto_raw_path[300] = {0};
    char auto_wav_path[300] = {0};

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strncmp("--audio-device=", a, 15) == 0) {
            if (strlen(a) < 16) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            audio_device = a + 15;
            audio_pick = 0;
        } else if (strcmp("--signalink-audio", a) == 0) {
            audio_pick = 1;
        } else if (strcmp("--radio-audio", a) == 0) {
            audio_pick = 2;
        } else if (strncmp("--backend-hint=", a, 15) == 0) {
            if (strlen(a) < 16) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            backend_hint = a + 15;
        } else if (strncmp("--bit-rate=", a, 11) == 0) {
            if (strlen(a) < 12) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            bit_rate = atoi(a + 11);
        } else if (strncmp("--window-s=", a, 11) == 0) {
            if (strlen(a) < 12) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            window_s = atof(a + 11);
            if (window_s < 0.5) window_s = 0.5;
            if (window_s > 30.0) window_s = 30.0;
        } else if (strncmp("--slide-s=", a, 10) == 0) {
            if (strlen(a) < 11) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            slide_s = atof(a + 10);
            if (slide_s < 0.05) slide_s = 0.05;
        } else if (strncmp("--sync-threshold=", a, 17) == 0) {
            if (strlen(a) < 18) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            sync_max_ham = atoi(a + 17);
            if (sync_max_ham < 0 || sync_max_ham > 8) {
                fprintf(stderr, "rx_live: --sync-threshold out of range [0,8]\n");
                return EXIT_FAILURE;
            }
        } else if (strncmp("--keyfile=", a, 10) == 0) {
            if (strlen(a) < 11) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            keyfile_path = a + 10;
        } else if (strcmp("--hmac", a) == 0) {
            use_hmac = 1;
        } else if (strcmp("--no-hmac", a) == 0) {
            // Default; kept as a no-op so existing scripts don't break.
            use_hmac = 0;
        } else if (strcmp("--reed-solomon", a) == 0) {
            use_rs = 1;
        } else if (strcmp("--no-reed-solomon", a) == 0) {
            use_rs = 0;
        } else if (strcmp("--no-dc-block", a) == 0) {
            no_dc_block = 1;
        } else if (strcmp("--csp-crc32", a) == 0) {
            csp_crc32 = 1;
        } else if (strcmp("--no-csp-crc32", a) == 0) {
            // Default; kept as a no-op for any existing scripts.
            csp_crc32 = 0;
        } else if (strncmp("--log=", a, 6) == 0) {
            if (strlen(a) < 7) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            log_path = a + 6;
            want_log = 1;
        } else if (strcmp("--no-log", a) == 0) {
            want_log = 0;
        } else if (strncmp("--raw=", a, 6) == 0) {
            if (strlen(a) < 7) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            raw_path = a + 6;
            want_raw = 1;
        } else if (strcmp("--no-raw", a) == 0) {
            want_raw = 0;
        } else if (strncmp("--wav=", a, 6) == 0) {
            if (strlen(a) < 7) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            wav_path = a + 6;
            want_wav = 1;
        } else if (strcmp("--no-wav", a) == 0) {
            want_wav = 0;
        } else if (strcmp("--no-spectrogram", a) == 0) {
            want_spectrogram = 0;
        } else if (strcmp("--quiet", a) == 0) {
            quiet = 1;
        } else if (strcmp("--help", a) == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else {
            fprintf(stderr, "rx_live: unknown option '%s'\n", a);
            usage(stderr, argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (slide_s > window_s) slide_s = window_s;

    // Audio device autodetect.
    if (audio_device == NULL) {
        int rc;
        if (audio_pick == 2) {
            rc = audio_find_radio_device(backend_hint, audio_device_buf,
                                         sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "rx_live: --radio-audio: not found "
                        "(rc=%d)\n", rc);
                return EXIT_FAILURE;
            }
        } else {
            rc = audio_find_signalink_device(audio_device_buf,
                                              sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "rx_live: --signalink-audio: SignaLink not "
                        "found (rc=%d). Pass --audio-device= or "
                        "--radio-audio.\n", rc);
                return EXIT_FAILURE;
            }
        }
        audio_device = audio_device_buf;
        fprintf(stderr, "rx_live: auto-detected audio device %s\n",
                audio_device);
    }

    int samp_rate = AUDIO_RATE_HZ;
    int channels = AUDIO_CHANNELS;
    if (samp_rate <= 0 || bit_rate <= 0 || (samp_rate % bit_rate) != 0) {
        fprintf(stderr, "rx_live: samp_rate (%d) must be a multiple of "
                "bit_rate (%d)\n", samp_rate, bit_rate);
        return EXIT_FAILURE;
    }

    // Auto-name log / raw / wav using a single UTC-stamped session basename
    // when the operator didn't pass an explicit path. All three end up
    // sharing the same basename so post-mortem inspection (.raw, .wav,
    // .log, .png) groups together in directory listings.
    if ((want_log && log_path == NULL) ||
        (want_raw && raw_path == NULL) ||
        (want_wav && wav_path == NULL)) {
        struct timeval tv;
        struct tm utc;
        if (gettimeofday(&tv, NULL) != 0 || gmtime_r(&tv.tv_sec, &utc) == NULL) {
            fprintf(stderr, "rx_live: could not read wall clock for "
                    "auto session names\n");
            return EXIT_FAILURE;
        }
        char base[200];
        snprintf(base, sizeof base,
                 "rx_live_UT=%04d%02d%02dT%02d%02d%02d.%03ld",
                 utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                 utc.tm_hour, utc.tm_min, utc.tm_sec,
                 (long)(tv.tv_usec / 1000));
        if (want_log && log_path == NULL) {
            snprintf(auto_log_path, sizeof auto_log_path, "%s.log", base);
            log_path = auto_log_path;
        }
        if (want_raw && raw_path == NULL) {
            snprintf(auto_raw_path, sizeof auto_raw_path, "%s.raw", base);
            raw_path = auto_raw_path;
        }
        if (want_wav && wav_path == NULL) {
            snprintf(auto_wav_path, sizeof auto_wav_path, "%s.wav", base);
            wav_path = auto_wav_path;
        }
    }
    if (!want_log)  log_path = NULL;
    if (!want_raw)  raw_path = NULL;
    if (!want_wav)  wav_path = NULL;

    // HMAC key.
    uint8_t hmac_key[128];
    ssize_t hmac_key_len = 0;
    if (use_hmac) {
        char default_path[512];
        const char *path = keyfile_path;
        if (path == NULL) {
            if (hmac_keyfile_default_path(default_path,
                                          sizeof default_path) != 0) {
                fprintf(stderr, "rx_live: HOME unset; pass --keyfile=<path>\n");
                return EXIT_FAILURE;
            }
            path = default_path;
        }
        hmac_key_len = hmac_keyfile_load(path, hmac_key, sizeof hmac_key);
        if (hmac_key_len < 0) return EXIT_FAILURE;
    }

    // Open ALSA capture.
    snd_pcm_t *capture = NULL;
    int arc = audio_capture_open(&capture, audio_device,
                                 (unsigned)samp_rate, (unsigned)channels);
    if (arc != AUDIO_OK || capture == NULL) {
        fprintf(stderr, "rx_live: audio_capture_open(%s) failed (rc=%d)\n",
                audio_device, arc);
        return EXIT_FAILURE;
    }

    // Raw and WAV tees. Both are optional via --no-raw / --no-wav; default
    // ON with auto-named paths (set just above) for post-mortem analysis.
    FILE *raw_fp = NULL;
    if (raw_path != NULL) {
        raw_fp = fopen(raw_path, "wb");
        if (raw_fp == NULL) {
            fprintf(stderr, "rx_live: fopen(%s): %s\n",
                    raw_path, strerror(errno));
            audio_capture_close(capture);
            return EXIT_FAILURE;
        }
    }
    audio_wav_writer_t *wav_w = NULL;
    if (wav_path != NULL) {
        wav_w = audio_wav_writer_open(wav_path, (unsigned)samp_rate,
                                      (unsigned)channels);
        if (wav_w == NULL) {
            fprintf(stderr, "rx_live: could not open WAV %s\n", wav_path);
            if (raw_fp) fclose(raw_fp);
            audio_capture_close(capture);
            return EXIT_FAILURE;
        }
    }

    // Sliding window of mono int16 (channel 0 only).
    size_t window_samples = (size_t)(window_s * (double)samp_rate);
    size_t slide_samples = (size_t)(slide_s * (double)samp_rate);
    if (slide_samples == 0) slide_samples = 1;
    if (slide_samples > window_samples) slide_samples = window_samples;

    int16_t *window = calloc(window_samples, sizeof(int16_t));
    if (window == NULL) {
        fprintf(stderr, "rx_live: out of memory for window (%zu samples)\n",
                window_samples);
        if (raw_fp) fclose(raw_fp);
        if (wav_w) audio_wav_writer_close(wav_w);
        audio_capture_close(capture);
        return EXIT_FAILURE;
    }
    size_t window_filled = 0;

    // Chunk read buffer (interleaved across all channels).
    const snd_pcm_uframes_t chunk_frames = 2048;
    int16_t *chunk = malloc(chunk_frames * (size_t)channels * sizeof(int16_t));
    if (chunk == NULL) {
        free(window);
        if (raw_fp) fclose(raw_fp);
        if (wav_w) audio_wav_writer_close(wav_w);
        audio_capture_close(capture);
        return EXIT_FAILURE;
    }

    // Decoder scratch — pre-allocated, reused per window.
    int sps = samp_rate / bit_rate;
    size_t bits_cap = window_samples / (size_t)sps + 16;
    size_t bytes_cap = bits_cap / 8 + 16;
    uint8_t *bits_scratch = malloc(bits_cap);
    uint8_t *bytes_scratch = malloc(bytes_cap);
    uint8_t packet[4100];
    if (bits_scratch == NULL || bytes_scratch == NULL) {
        free(bits_scratch); free(bytes_scratch); free(chunk); free(window);
        if (raw_fp) fclose(raw_fp);
        if (wav_w) audio_wav_writer_close(wav_w);
        audio_capture_close(capture);
        return EXIT_FAILURE;
    }

    modem_params_t mp;
    modem_params_defaults(&mp);
    mp.samp_rate = samp_rate;
    mp.bit_rate = bit_rate;
    mp.rx_disable_dc_block = no_dc_block;

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (use_hmac) {
        opts.hmac_key = hmac_key;
        opts.hmac_key_len = (size_t)hmac_key_len;
    }
    opts.reed_solomon = use_rs;

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (use_hmac) {
        fprintf(stderr, "rx_live: %s window=%.2fs slide=%.2fs sync_thr=%d "
                "hmac=on rs=%s log=%s raw=%s\n",
                audio_device, window_s, slide_s, sync_max_ham,
                use_rs ? "on" : "off",
                log_path ? log_path : "(none)",
                raw_path ? raw_path : "(none)");
    } else {
        fprintf(stderr, "rx_live: %s window=%.2fs slide=%.2fs sync_thr=%d "
                "rs=%s log=%s raw=%s\n",
                audio_device, window_s, slide_s, sync_max_ham,
                use_rs ? "on" : "off",
                log_path ? log_path : "(none)",
                raw_path ? raw_path : "(none)");
    }

    // Dedup by absolute sample position (quantised). The same burst is
    // visible in every window whose [start, end] contains it; with
    // window=1.5s slide=0.5s that's up to 4 windows per burst, so we
    // need to suppress 3-4 redundant emits per real frame. Using
    // *position* (not content hash) means:
    //   1. Partial-RS emits, whose descrambled bytes drift slightly
    //      across windows due to DC-block IIR state, still dedup.
    //   2. Legitimate retransmissions of the same payload at
    //      different times all emit (the old FNV ring suppressed
    //      every retransmission of an already-seen content hash for
    //      the rest of the session, which is the "rapid 0.2 s bursts
    //      misses some" symptom).
    // Quantisation: 100 ms (4800 samples @ 48 kHz). Bursts ~200 ms
    // apart map to distinct keys; a single burst seen across slides
    // maps to the same key (the absolute ASM sample is invariant
    // since slide_samples is a multiple of sps).
    enum { DEDUP_RING_SZ = 64 };
    enum { DEDUP_QUANT_SAMPLES = 4800 };
    uint64_t recent_pos_quant[DEDUP_RING_SZ] = {0};
    int recent_idx = 0;
    int recent_count = 0;
    // Running total of channel-0 samples appended to `window`. Used
    // with sync_off_local to recover the absolute sample index of the
    // ASM at emit time.
    uint64_t total_window_samples = 0;

    while (!g_stop) {
        snd_pcm_sframes_t got = snd_pcm_readi(capture, chunk, chunk_frames);
        if (got == -EPIPE) {
            snd_pcm_prepare(capture);
            continue;
        }
        if (got < 0) {
            fprintf(stderr, "rx_live: snd_pcm_readi: %s\n",
                    snd_strerror((int)got));
            break;
        }

        if (raw_fp != NULL) {
            (void)fwrite(chunk, sizeof(int16_t),
                         (size_t)got * (size_t)channels, raw_fp);
        }
        if (wav_w != NULL) {
            audio_wav_writer_append(wav_w, chunk,
                                    (size_t)got * (size_t)channels
                                    * sizeof(int16_t));
        }

        // Append channel 0 to the window, decode whenever the window fills.
        for (snd_pcm_sframes_t i = 0; i < got; i++) {
            window[window_filled++] = chunk[i * channels];
            ++total_window_samples;
            if (window_filled < window_samples) continue;

            // Inner loop: pull every decodable frame out of this window
            // before sliding. With bursts arriving at 200 ms intervals
            // and window=1.5 s, a single window can hold 6+ frames; the
            // old "first decode then slide" path emitted only one and
            // missed the rest until the slide caught up.
            size_t inner_min_offset = 0;
            for (;;) {
                ssize_t plen = -1;
                int golay_errs = 0, hmac_ok = -1;
                int rs_errs = -1, used_golay_len = -1;
                size_t sync_off_local = 0;
                if (!try_decode_window(window, window_samples, &mp, &opts,
                                       sync_max_ham, use_hmac,
                                       /*allow_partial_rs=*/1,
                                       inner_min_offset,
                                       bits_scratch, bits_cap,
                                       bytes_scratch, bytes_cap,
                                       packet, sizeof packet,
                                       &plen, &golay_errs, &hmac_ok,
                                       &rs_errs, &used_golay_len,
                                       &sync_off_local)) {
                    break;
                }
                inner_min_offset = sync_off_local + 1;
                // Sanity bound: must fit a CSP header at minimum, and must
                // not exceed the packet buffer. Was 256 — too tight; AX100
                // frames at the format max go up to 255 bytes pre-CRC and
                // ax100_unframe's brute-force length search can come back
                // with values close to that. Use the buffer size as the
                // upper bound; the remaining filter is the CSP CRC32
                // validation a few lines below.
                if (plen < 4 || (size_t)plen > sizeof packet) continue;
                // CSP v1 downlink frames carry a trailing 32-bit zlib CRC
                // over the entire packet (header + payload). libcsp peers
                // emit it little-endian on the wire on most builds, but
                // we accept either endianness so the same code works
                // against unknown-endian satellite firmwares. Skip when
                // HMAC mode is on (uplink frames carry a 32-byte SHA-256
                // tag, not a CRC).
                int crc_status = -1;
                uint32_t crc_computed = 0, crc_le = 0, crc_be = 0;
                if (!use_hmac && csp_crc32 && plen >= 8) {
                    crc_computed = csp_crc32_zlib(packet, (size_t)(plen - 4));
                    crc_le = (uint32_t)packet[plen - 4]
                           | ((uint32_t)packet[plen - 3] << 8)
                           | ((uint32_t)packet[plen - 2] << 16)
                           | ((uint32_t)packet[plen - 1] << 24);
                    crc_be = ((uint32_t)packet[plen - 4] << 24)
                           | ((uint32_t)packet[plen - 3] << 16)
                           | ((uint32_t)packet[plen - 2] <<  8)
                           |  (uint32_t)packet[plen - 1];
                    if (crc_computed == crc_le || crc_computed == crc_be) {
                        crc_status = 1;
                        plen -= 4;  // strip the trailer for output
                    } else {
                        crc_status = 0;
                        // Mismatch: keep the frame and the trailer; emit_frame
                        // will print a 'csp_crc32: MISMATCH' line for the operator.
                    }
                }
                // Recover the absolute sample index of the ASM. The
                // window currently spans the most recent
                // window_samples appended to `window`, so the window
                // start in absolute sample units is
                // total_window_samples - window_samples. sync_off_local
                // is the bit offset (within whichever phase the modem
                // picked) at which the ASM was found; *sps maps it
                // back to a sample offset within the window. We don't
                // know the exact phase here (modem doesn't return it)
                // but the ambiguity is ≤ sps-1 samples — well below
                // DEDUP_QUANT_SAMPLES, so the dedup key is stable
                // across overlapping windows.
                uint64_t window_start_abs =
                    total_window_samples - (uint64_t)window_samples;
                uint64_t asm_abs_sample = window_start_abs
                    + (uint64_t)sync_off_local * (uint64_t)sps
                    + (uint64_t)(sps / 2);
                uint64_t pos_quant = asm_abs_sample / DEDUP_QUANT_SAMPLES;
                int seen = 0;
                int ring_n = recent_count < DEDUP_RING_SZ
                    ? recent_count : DEDUP_RING_SZ;
                for (int r = 0; r < ring_n; r++) {
                    if (recent_pos_quant[r] == pos_quant) { seen = 1; break; }
                }
                if (seen) continue;
                recent_pos_quant[recent_idx] = pos_quant;
                recent_idx = (recent_idx + 1) % DEDUP_RING_SZ;
                if (recent_count < DEDUP_RING_SZ) recent_count++;
                char ts[64];
                fmt_utc(ts, sizeof ts);
                emit_frame(log_path, quiet, ts,
                           packet, (size_t)plen,
                           golay_errs, hmac_ok, use_hmac,
                           rs_errs, used_golay_len,
                           crc_status, crc_computed, crc_le, crc_be);
            }

            // Slide the window.
            memmove(window, window + slide_samples,
                    (window_samples - slide_samples) * sizeof(int16_t));
            window_filled = window_samples - slide_samples;
        }
    }

    if (g_stop) {
        fprintf(stderr, "rx_live: stopped on signal\n");
    }

    free(bits_scratch);
    free(bytes_scratch);
    free(chunk);
    free(window);
    audio_capture_close(capture);
    if (raw_fp) fclose(raw_fp);
    if (wav_w) {
        // Close WAV (patches header sizes) before invoking ffmpeg so
        // showspectrumpic reads a complete file. Spectrogram is
        // best-effort: if ffmpeg isn't on PATH, audio_generate_spectrogram
        // logs and returns non-zero but rx_live still exits cleanly.
        audio_wav_writer_close(wav_w);
        if (want_spectrogram && wav_path != NULL) {
            audio_generate_spectrogram(wav_path);
        }
    }
    return 0;
}
