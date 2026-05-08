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
#include "decode_loop.h"
#include "hmac_keyfile.h"
#include "modem.h"
#include "packet_db.h"
#include "rx_tui.h"

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
    rx_tui_request_quit();
}

// REPL handler used in --ui mode. Recognises the shared
// decode_loop_try_command verbs (packetheaders / ph) plus a `quit`
// command — the TUI disables its q-as-quit shortcut once a handler
// is registered, so the operator needs an in-REPL exit besides Ctrl-C.
static void rx_live_cmd_handler(const char *cmd, void *ctx)
{
    (void)ctx;
    char status[128];
    if (decode_loop_try_command(cmd, status, sizeof status)) {
        rx_tui_set_status(status);
        return;
    }
    // Strip leading whitespace before the verb-by-strcmp checks below.
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "q") == 0
        || strcmp(cmd, "exit") == 0) {
        g_stop = 1;
        rx_tui_request_quit();
        return;
    }
    if (cmd[0] != '\0') {
        char m[160];
        snprintf(m, sizeof m,
                 "unknown: `%.80s` — try `packetheaders on|off` or `quit`",
                 cmd);
        rx_tui_set_status(m);
    }
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
        "  --ui                     Switch from streaming text to a curses\n"
        "                           panel display (latest beacon broken\n"
        "                           out by subsystem, scrolling list of\n"
        "                           recent telecommand responses, frame\n"
        "                           counters). File logging continues.\n"
        "                           Press q to quit. Default off.\n"
        "  --packet-headers         Show the AX100 framing line, CSP v1\n"
        "                           header line, and per-frame hex/ascii\n"
        "                           dumps. Default off — only the\n"
        "                           interpreted body (beacon/tcmd/log/\n"
        "                           bulk_file) and error conditions print.\n"
        "                           In --ui mode, the REPL also accepts\n"
        "                           `packetheaders on|off` (or `ph on|off`)\n"
        "                           to flip the toggle mid-pass.\n"
        "  --no-packet-headers      Default; kept as a no-op for scripts.\n"
        "  --db=<path>              SQLite store for decoded packets.\n"
        "                           Default: $SSO_PACKET_DB or\n"
        "                           $HOME/.local/share/simple_sat_ops/\n"
        "                           packets.db. Append-only across runs.\n"
        "  --no-db                  Skip DB writes.\n"
        "  --source-run=<id>        Override the per-launch run-id used\n"
        "                           for dedup of re-decoded captures.\n"
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
    int use_tui = 0;
    int show_packet_headers = 0;
    const char *db_path = NULL;
    const char *source_run_override = NULL;
    int no_db = 0;
    const char *ref_hex_arg = NULL;
    uint8_t ref_buf[4100];
    size_t ref_buf_len = 0;
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
        } else if (strncmp("--ref-hex=", a, 10) == 0) {
            if (strlen(a) < 11) { fprintf(stderr, "Unable to parse %s\n", a); return EXIT_FAILURE; }
            ref_hex_arg = a + 10;
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
        } else if (strcmp("--ui", a) == 0) {
            use_tui = 1;
        } else if (strcmp("--packet-headers", a) == 0) {
            show_packet_headers = 1;
        } else if (strcmp("--no-packet-headers", a) == 0) {
            show_packet_headers = 0;
        } else if (strncmp("--db=", a, 5) == 0) {
            db_path = a + 5;
        } else if (strcmp("--no-db", a) == 0) {
            no_db = 1;
        } else if (strncmp("--source-run=", a, 13) == 0) {
            source_run_override = a + 13;
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

    if (ref_hex_arg != NULL) {
        size_t n = 0;
        int high = -1;
        for (const char *p = ref_hex_arg; *p; ++p) {
            char c = *p;
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == ':') continue;
            int v = (c >= '0' && c <= '9') ? c - '0'
                  : (c >= 'a' && c <= 'f') ? c - 'a' + 10
                  : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
            if (v < 0) {
                fprintf(stderr, "rx_live: --ref-hex: bad char '%c'\n", c);
                return EXIT_FAILURE;
            }
            if (high < 0) {
                high = v;
            } else {
                if (n >= sizeof ref_buf) {
                    fprintf(stderr, "rx_live: --ref-hex too long (>%zu bytes)\n",
                            sizeof ref_buf);
                    return EXIT_FAILURE;
                }
                ref_buf[n++] = (uint8_t)((high << 4) | v);
                high = -1;
            }
        }
        if (high >= 0) {
            fprintf(stderr, "rx_live: --ref-hex: odd number of hex digits\n");
            return EXIT_FAILURE;
        }
        if (n == 0) {
            fprintf(stderr, "rx_live: --ref-hex: empty\n");
            return EXIT_FAILURE;
        }
        ref_buf_len = n;
    }

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

    // Touch the log at startup so it exists before the first decoded
    // frame. emit_frame's fopen("a") would otherwise only create the
    // file on first hit, leaving `tail -F rx_live_*.log` with nothing
    // to follow during AOS wait. One-line session header makes the
    // file's purpose obvious if found on disk later.
    if (log_path != NULL) {
        FILE *lf = fopen(log_path, "a");
        if (lf == NULL) {
            fprintf(stderr, "rx_live: fopen(%s, a): %s\n",
                    log_path, strerror(errno));
        } else {
            char ts[64];
            fmt_utc(ts, sizeof ts);
            fprintf(lf,
                    "[%s] rx_live: session start audio=%s window=%.2fs "
                    "slide=%.2fs sync_thr=%d rs=%s hmac=%s\n",
                    ts, audio_device, window_s, slide_s, sync_max_ham,
                    use_rs ? "on" : "off",
                    use_hmac ? "on" : "off");
            fclose(lf);
        }
    }

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

    decode_loop_set_show_headers(show_packet_headers);

    char db_run_id[24];
    packet_db_t *db = packet_db_setup(db_path, no_db,
                                      db_run_id, sizeof db_run_id);
    if (source_run_override != NULL) {
        snprintf(db_run_id, sizeof db_run_id, "%s", source_run_override);
    }
    decode_loop_set_packet_db(db, "rx_live", db_run_id);

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    // In --ui mode the banner becomes the TUI title bar instead of a
    // stderr line that would scribble over the curses screen. quiet=1
    // is also forced so emit_frame doesn't write to stdout (ncurses
    // owns it). File logging continues either way.
    // Sized to comfortably hold an auto-named log path (up to 299
    // chars from auto_log_path) plus the rest of the format. The TUI
    // truncates to terminal width when rendering, so an oversized
    // header is harmless.
    char tui_header[512];
    if (use_tui) {
        if (rx_tui_init() != 0) {
            fprintf(stderr, "rx_live: --ui requested but ncurses is not "
                    "built in (rebuild with libncurses-dev installed).\n");
            return EXIT_FAILURE;
        }
        snprintf(tui_header, sizeof tui_header,
                 "rx_live | %s | win=%.2fs slide=%.2fs sync_thr=%d rs=%s hmac=%s | log=%s",
                 audio_device, window_s, slide_s, sync_max_ham,
                 use_rs ? "on" : "off",
                 use_hmac ? "on" : "off",
                 log_path ? log_path : "(none)");
        rx_tui_set_header(tui_header);
        rx_tui_set_command_handler(rx_live_cmd_handler, NULL);
        quiet = 1;
    } else if (use_hmac) {
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
        if (use_tui && rx_tui_tick()) g_stop = 1;
        snd_pcm_sframes_t got = snd_pcm_readi(capture, chunk, chunk_frames);
        if (got == -EPIPE) {
            snd_pcm_prepare(capture);
            continue;
        }
        if (got < 0) {
            if (use_tui) rx_tui_close();
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
                int rs_locs[32];
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
                                       &sync_off_local,
                                       rs_locs)) {
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
                           crc_status, crc_computed, crc_le, crc_be,
                           rs_locs,
                           ref_buf_len > 0 ? ref_buf : NULL, ref_buf_len,
                           /*force_beacon=*/0);
                if (use_tui) {
                    rx_tui_observe_frame(ts, packet, (size_t)plen,
                                         golay_errs, hmac_ok, use_hmac,
                                         rs_errs, crc_status);
                }
            }

            // Slide the window.
            memmove(window, window + slide_samples,
                    (window_samples - slide_samples) * sizeof(int16_t));
            window_filled = window_samples - slide_samples;
        }
    }

    if (use_tui) rx_tui_close();
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
