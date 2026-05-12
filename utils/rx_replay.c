/*

    Simple Satellite Operations  utils/rx_replay.c

    Offline equivalent of b210_rx_live: same sliding-window AX100
    decoder, same partial-RS rescue, same position-quantised dedup — but
    the samples come from a WAV or headerless S16_LE PCM file rather
    than live IQ. Use it to re-process the .wav companion b210_rx_live
    leaves on disk after a pass, or to debug the decoder against a
    capture from rtl_fm / a different SDR pipeline.

    Differs from rx_decode (the single-frame offline decoder):
      - Walks the entire file at window/slide cadence, not just the
        first sync match.
      - Emits a UNCORRECTABLE-marked line for long bursts whose
        Golay header decoded but RS(255,223) couldn't correct the
        in-band errors (the descrambled bytes are recovered).
      - Dedups bursts seen across overlapping windows by absolute
        sample position; legitimate retransmissions of the same
        content at different times all emit.

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

#include "ax100.h"
#include "csp.h"
#include "decode_loop.h"
#include "hmac_keyfile.h"
#include "modem.h"
#include "packet_db.h"
#include "rx_tui.h"
#include "sso_audit.h"
#include "wav_read.h"

#ifdef WITH_SGP4SDP4
#include "prediction.h"
#endif

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <libgen.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
    // Also nudge the TUI in case we're sitting in rx_tui_hold_until_quit.
    // The stub for non-ncurses builds is a no-op.
    rx_tui_request_quit();
}

// REPL handler used in --ui mode. See rx_live.c for rationale; the
// implementation is identical (registering this disables the
// q-as-quit shortcut, so a `quit` verb is wired in alongside).
static void rx_replay_cmd_handler(const char *cmd, void *ctx)
{
    (void)ctx;
    char status[128];
    if (decode_loop_try_command(cmd, status, sizeof status)) {
        rx_tui_set_status(status);
        return;
    }
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

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

// Read three lines (name + line1 + line2) for the requested satellite
// out of a TLE file. sat_prefix may be NULL — in that case the first
// non-comment 3-line block is returned (useful when the file is a
// single-sat TLE the user dropped next to the WAV). Returns 0 on
// success, -1 on miss / I/O error.
static int read_tle_lines(const char *path, const char *sat_prefix,
                          char *out_name,  size_t name_n,
                          char *out_line1, size_t line1_n,
                          char *out_line2, size_t line2_n)
{
    if (path == NULL) return -1;
    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;
    char a[128], b[128], c[128];
    int rc = -1;
    while (fgets(a, sizeof a, f) != NULL) {
        size_t n = strlen(a);
        while (n > 0 && (a[n-1] == '\n' || a[n-1] == '\r')) a[--n] = '\0';
        if (n == 0 || a[0] == '#') continue;
        if (sat_prefix != NULL
            && strncmp(a, sat_prefix, strlen(sat_prefix)) != 0) continue;
        if (fgets(b, sizeof b, f) == NULL) break;
        n = strlen(b);
        while (n > 0 && (b[n-1] == '\n' || b[n-1] == '\r')) b[--n] = '\0';
        if (fgets(c, sizeof c, f) == NULL) break;
        n = strlen(c);
        while (n > 0 && (c[n-1] == '\n' || c[n-1] == '\r')) c[--n] = '\0';
        if (b[0] != '1' || c[0] != '2') break;
        snprintf(out_name,  name_n,  "%s", a);
        snprintf(out_line1, line1_n, "%s", b);
        snprintf(out_line2, line2_n, "%s", c);
        rc = 0;
        break;
    }
    fclose(f);
    return rc;
}

// If the input file's directory contains exactly one *.tle file,
// return its absolute-or-relative path in `out`. Returns 0 on hit, -1
// on zero or multiple matches.
static int autodiscover_tle_in_dir(const char *dir, char *out, size_t outn)
{
    if (dir == NULL) return -1;
    DIR *d = opendir(dir);
    if (d == NULL) return -1;
    struct dirent *de;
    char hit[1024] = {0};
    int count = 0;
    while ((de = readdir(d)) != NULL) {
        size_t nlen = strlen(de->d_name);
        if (nlen < 4) continue;
        if (strcmp(de->d_name + nlen - 4, ".tle") != 0) continue;
        snprintf(hit, sizeof hit, "%s/%s", dir, de->d_name);
        count++;
        if (count > 1) break;
    }
    closedir(d);
    if (count == 1) {
        snprintf(out, outn, "%s", hit);
        return 0;
    }
    return -1;
}

// Parse "...UT=YYYYMMDDTHHMMSS.sss..." out of a filename and return
// the seconds-since-epoch of that timestamp into *out_unix_ms.
// Returns 0 on success, -1 if the pattern isn't found.
static int parse_ut_from_filename(const char *path, double *out_unix_seconds)
{
    if (path == NULL) return -1;
    char buf[1024];
    snprintf(buf, sizeof buf, "%s", path);
    const char *base = basename(buf);
    const char *p = strstr(base, "UT=");
    if (p == NULL) return -1;
    p += 3;
    int yr, mo, d, h, mi, s, ms = 0;
    int got = sscanf(p, "%4d%2d%2dT%2d%2d%2d.%3d",
                     &yr, &mo, &d, &h, &mi, &s, &ms);
    if (got < 6) return -1;
    struct tm utc = {0};
    utc.tm_year = yr - 1900;
    utc.tm_mon  = mo - 1;
    utc.tm_mday = d;
    utc.tm_hour = h;
    utc.tm_min  = mi;
    utc.tm_sec  = s;
    time_t epoch = timegm(&utc);
    if (epoch == (time_t)-1) return -1;
    *out_unix_seconds = (double)epoch + ms / 1000.0;
    return 0;
}

// Format seconds-since-epoch (UTC) as ISO-8601.
static void unix_seconds_to_iso(double t, char *out, size_t outn)
{
    time_t sec = (time_t)t;
    int ms = (int)((t - sec) * 1000.0 + 0.5) % 1000;
    struct tm utc;
    gmtime_r(&sec, &utc);
    // Masks bound each field so gcc's -Wformat-truncation= can prove
    // the formatted result fits the caller's buffer.
    int yr = (utc.tm_year + 1900) % 10000;
    int mo = (utc.tm_mon + 1) % 100;
    int da = utc.tm_mday % 100;
    int hh = utc.tm_hour % 100;
    int mm = utc.tm_min  % 100;
    int ss = utc.tm_sec  % 100;
    snprintf(out, outn, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
             yr, mo, da, hh, mm, ss, ms);
}

// Parse "YYYY-MM-DDTHH:MM:SS[.fff]Z" -> unix seconds. Returns 0 / -1.
static int parse_iso_to_unix_seconds(const char *iso, double *out)
{
    if (iso == NULL) return -1;
    int yr, mo, d, h, mi, s, ms = 0;
    int got = sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2d.%3d",
                     &yr, &mo, &d, &h, &mi, &s, &ms);
    if (got < 6) return -1;
    struct tm utc = {0};
    utc.tm_year = yr - 1900;
    utc.tm_mon  = mo - 1;
    utc.tm_mday = d;
    utc.tm_hour = h;
    utc.tm_min  = mi;
    utc.tm_sec  = s;
    time_t epoch = timegm(&utc);
    if (epoch == (time_t)-1) return -1;
    *out = (double)epoch + ms / 1000.0;
    return 0;
}

// Read a raw S16_LE file into a freshly-malloc'd int16 buffer. Returns
// 0 on success (caller frees *out_samples), -1 on error.
static int read_raw_pcm16(const char *path, int16_t **out_samples, size_t *out_n)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        fprintf(stderr, "rx_replay: open(%s): %s\n", path, strerror(errno));
        return -1;
    }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return -1; }
    long sz = ftell(f);
    if (sz < 0) { fclose(f); return -1; }
    if ((sz & 1) != 0) {
        fprintf(stderr, "rx_replay: %s has odd byte count (%ld); expected S16_LE\n",
                path, sz);
        fclose(f);
        return -1;
    }
    rewind(f);
    size_t n = (size_t)sz / 2u;
    int16_t *buf = (int16_t *)malloc(n * sizeof(int16_t));
    if (buf == NULL) { fclose(f); return -1; }
    if (fread(buf, sizeof(int16_t), n, f) != n) {
        fprintf(stderr, "rx_replay: read(%s): short read\n", path);
        free(buf); fclose(f); return -1;
    }
    fclose(f);
    *out_samples = buf;
    *out_n = n;
    return 0;
}

static void usage(FILE *dest, const char *name)
{
    fprintf(dest,
        "usage: %s <path> [options]\n"
        "\n"
        "Offline AX100 sliding-window decoder. Reads a WAV or headerless\n"
        "S16_LE raw PCM recording (e.g. b210_rx_live's .wav companion)\n"
        "and applies the same window/slide/decode loop b210_rx_live runs\n"
        "on live IQ capture.\n"
        "\n"
        "Input:\n"
        "  --raw                    Treat <path> as headerless S16_LE PCM\n"
        "                           (auto-enabled when path ends in '.raw').\n"
        "  --rate=<hz>              Sample rate for --raw (default 48000).\n"
        "  --channels=<n>           Channels for --raw (default 2; ch 0 used).\n"
        "                           Pass --channels=1 for rtl_fm captures.\n"
        "\n"
        "Decoder (same defaults as b210_rx_live):\n"
        "  --bit-rate=<bps>         Default 9600.\n"
        "  --window-s=<seconds>     Decoder window size (default 1.5).\n"
        "  --slide-s=<seconds>      Slide between decode attempts (default 0.5).\n"
        "  --sync-threshold=<0..8>  Max ASM bit errors (default 4).\n"
        "  --hmac                   Enable HMAC verification (off by default;\n"
        "                           AX100 downlink frames don't carry HMAC).\n"
        "  --keyfile=<path>         HMAC keyfile (default $HOME/%s).\n"
        "  --reed-solomon           RS(255,223) decode (DEFAULT).\n"
        "  --no-reed-solomon        Skip RS decode.\n"
        "  --no-dc-block            Skip the modem's DC-block IIR (use on\n"
        "                           radio digital taps with no DC offset).\n"
        "  --csp-crc32              Validate + strip a trailing CSP zlib\n"
        "                           CRC32 (off by default; opt-in only when\n"
        "                           the TX side is known to append one).\n"
        "  --no-partial-rs          Disable the partial-RS rescue. With it\n"
        "                           on (default) a frame whose Golay\n"
        "                           header decoded with 0 errors but whose\n"
        "                           RS(255,223) was uncorrectable still\n"
        "                           emits, marked rs=UNCORRECTABLE.\n"
        "  --ref-hex=<hex>          Compare each decoded packet (or payload,\n"
        "                           auto-detected by length) against this\n"
        "                           reference and print byte positions that\n"
        "                           differ. Whitespace and ':' in the hex\n"
        "                           string are ignored.\n"
        "  --force-beacon           Pad each decoded payload with zeros up\n"
        "                           to 130 bytes and print it as a beacon\n"
        "                           regardless of length or dispatch result.\n"
        "                           Last-ditch view of heavily corrupted\n"
        "                           frames. Ignored in --ui mode.\n"
        "\n"
        "Output:\n"
        "  --log=<path>             Append decode lines to <path> in\n"
        "                           addition to stdout. Re-opened per frame\n"
        "                           so log rotation works.\n"
        "  --quiet                  Skip stdout output (log-only mode).\n"
        "  --ui                     Switch from streaming text to a curses\n"
        "                           panel display (latest beacon broken\n"
        "                           out by subsystem, scrolling list of\n"
        "                           recent telecommand responses, frame\n"
        "                           counters). After the file is replayed\n"
        "                           the screen holds until you press q.\n"
        "                           File logging continues. Default off.\n"
        "  --packet-headers         Show the AX100 framing line, CSP v1\n"
        "                           header line, and per-frame hex/ascii\n"
        "                           dumps. Default off — only the\n"
        "                           interpreted body and error conditions\n"
        "                           print. In --ui mode, the REPL accepts\n"
        "                           `packetheaders on|off` (or `ph on|off`)\n"
        "                           to flip the toggle mid-replay.\n"
        "  --no-packet-headers      Default; kept as a no-op for scripts.\n"
        "  --db=<path>              SQLite store for decoded packets.\n"
        "                           Default: $SSO_PACKET_DB or\n"
        "                           $HOME/.local/share/simple_sat_ops/\n"
        "                           packets.db. Append-only across runs;\n"
        "                           re-running the same input is dedup'd\n"
        "                           (pass --source-run=<id> to force a\n"
        "                           fresh row group).\n"
        "  --no-db                  Skip DB writes.\n"
        "  --source-run=<id>        Override the per-launch run-id.\n"
        "  --capture-origin=<name>  Tag rows with the audio's provenance,\n"
        "                           e.g. cts_ground or satnogs. Distinct\n"
        "                           from --source-run (which identifies\n"
        "                           one launch) and source_tool (the\n"
        "                           decoder). Same packet captured at\n"
        "                           multiple sites keeps one row per\n"
        "                           origin so cross-site decodes are\n"
        "                           visible. Default: unset (NULL).\n"
        "  --help                   Show this help.\n",
        name, HMAC_KEYFILE_DEFAULT_RELPATH);
}

int main(int argc, char **argv)
{
    const char *input_path = NULL;
    const char *log_path = NULL;
    const char *keyfile_path = NULL;
    int raw_mode = 0;
    int raw_mode_explicit = 0;
    int raw_rate = 48000;
    int raw_channels = 2;
    int bit_rate = 9600;
    double window_s = 1.5;
    double slide_s = 0.5;
    int sync_max_ham = 4;
    int use_hmac = 0;
    int use_rs = 1;
    int no_dc_block = 0;
    int csp_crc32 = 0;
    int allow_partial_rs = 1;
    int quiet = 0;
    int use_tui = 0;
    const char *ref_hex_arg = NULL;
    uint8_t ref_buf[4100];
    size_t ref_buf_len = 0;
    int force_beacon = 0;
    int show_packet_headers = 0;
    const char *db_path = NULL;
    const char *source_run_override = NULL;
    int no_db = 0;
    const char *tle_path = NULL;
    const char *sat_arg  = NULL;
    const char *start_utc_arg = NULL;
    const char *session_dir_arg = NULL;
    const char *capture_origin = NULL;
    int update_mode = 0;
    double obs_lat_deg = 50.8688;   // RAO defaults; overridden by flags
    double obs_lon_deg = -114.2910;
    double obs_alt_m   = 1279.0;
    double nominal_freq_hz = 436150000.0; // FrontierSat carrier

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (strcmp(a, "--raw") == 0) { raw_mode = 1; raw_mode_explicit = 1; }
        else if (starts_with(a, "--rate="))      raw_rate = atoi(a + 7);
        else if (starts_with(a, "--channels="))  raw_channels = atoi(a + 11);
        else if (starts_with(a, "--bit-rate="))  bit_rate = atoi(a + 11);
        else if (starts_with(a, "--window-s=")) {
            window_s = atof(a + 11);
            if (window_s < 0.5) window_s = 0.5;
            if (window_s > 30.0) window_s = 30.0;
        } else if (starts_with(a, "--slide-s=")) {
            slide_s = atof(a + 10);
            if (slide_s < 0.05) slide_s = 0.05;
        } else if (starts_with(a, "--sync-threshold=")) {
            sync_max_ham = atoi(a + 17);
            if (sync_max_ham < 0 || sync_max_ham > 8) {
                fprintf(stderr, "rx_replay: --sync-threshold out of range [0,8]\n");
                return 1;
            }
        }
        else if (strcmp(a, "--hmac") == 0)             use_hmac = 1;
        else if (strcmp(a, "--no-hmac") == 0)          use_hmac = 0;
        else if (strcmp(a, "--reed-solomon") == 0)     use_rs = 1;
        else if (strcmp(a, "--no-reed-solomon") == 0)  use_rs = 0;
        else if (strcmp(a, "--no-dc-block") == 0)      no_dc_block = 1;
        else if (strcmp(a, "--csp-crc32") == 0)        csp_crc32 = 1;
        else if (strcmp(a, "--no-csp-crc32") == 0)     csp_crc32 = 0;
        else if (strcmp(a, "--no-partial-rs") == 0)    allow_partial_rs = 0;
        else if (strcmp(a, "--quiet") == 0)            quiet = 1;
        else if (strcmp(a, "--ui") == 0)               use_tui = 1;
        else if (starts_with(a, "--keyfile="))         keyfile_path = a + 10;
        else if (starts_with(a, "--log="))             log_path = a + 6;
        else if (starts_with(a, "--ref-hex="))         ref_hex_arg = a + 10;
        else if (strcmp(a, "--force-beacon") == 0)     force_beacon = 1;
        else if (strcmp(a, "--packet-headers") == 0)   show_packet_headers = 1;
        else if (strcmp(a, "--no-packet-headers") == 0) show_packet_headers = 0;
        else if (starts_with(a, "--db="))              db_path = a + 5;
        else if (strcmp(a, "--no-db") == 0)            no_db = 1;
        else if (starts_with(a, "--source-run="))      source_run_override = a + 13;
        else if (starts_with(a, "--tle="))             tle_path = a + 6;
        else if (starts_with(a, "--satellite="))       sat_arg = a + 12;
        else if (starts_with(a, "--start-utc="))       start_utc_arg = a + 12;
        else if (starts_with(a, "--session-dir="))     session_dir_arg = a + 14;
        else if (starts_with(a, "--capture-origin="))  capture_origin = a + 17;
        else if (strcmp(a, "--update") == 0)           update_mode = 1;
        else if (starts_with(a, "--lat="))             obs_lat_deg = atof(a + 6);
        else if (starts_with(a, "--lon="))             obs_lon_deg = atof(a + 6);
        else if (starts_with(a, "--alt="))             obs_alt_m   = atof(a + 6);
        else if (starts_with(a, "--carrier-mhz="))     nominal_freq_hz = atof(a + 14) * 1e6;
        else if (a[0] == '-') {
            fprintf(stderr, "rx_replay: unknown option '%s'\n", a);
            usage(stderr, argv[0]);
            return 1;
        } else if (input_path == NULL) {
            input_path = a;
        } else {
            fprintf(stderr, "rx_replay: unexpected positional '%s'\n", a);
            return 1;
        }
    }
    if (input_path == NULL) {
        fprintf(stderr, "rx_replay: missing <path>\n");
        usage(stderr, argv[0]);
        return 1;
    }
    sso_audit_start("rx_replay", input_path);
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
                fprintf(stderr, "rx_replay: --ref-hex: bad char '%c'\n", c);
                return 1;
            }
            if (high < 0) {
                high = v;
            } else {
                if (n >= sizeof ref_buf) {
                    fprintf(stderr, "rx_replay: --ref-hex too long (>%zu bytes)\n",
                            sizeof ref_buf);
                    return 1;
                }
                ref_buf[n++] = (uint8_t)((high << 4) | v);
                high = -1;
            }
        }
        if (high >= 0) {
            fprintf(stderr, "rx_replay: --ref-hex: odd number of hex digits\n");
            return 1;
        }
        if (n == 0) {
            fprintf(stderr, "rx_replay: --ref-hex: empty\n");
            return 1;
        }
        ref_buf_len = n;
    }
    if (slide_s > window_s) slide_s = window_s;
    if (!raw_mode_explicit) {
        size_t plen = strlen(input_path);
        if (plen >= 4 && strcmp(input_path + plen - 4, ".raw") == 0) raw_mode = 1;
    }
    if (raw_channels < 1 || raw_channels > 8) {
        fprintf(stderr, "rx_replay: --channels out of range [1,8]\n");
        return 1;
    }

    // Load samples (interleaved if multi-channel) and reduce to ch 0.
    int16_t *samples = NULL;
    size_t n_samples = 0;
    int samp_rate = 0;
    int channels = 1;
    if (raw_mode) {
        if (read_raw_pcm16(input_path, &samples, &n_samples) != 0) return 1;
        samp_rate = raw_rate;
        channels = raw_channels;
    } else {
        if (wav_read_pcm16(input_path, &samples, &n_samples,
                           &samp_rate, &channels) != 0) {
            return 1;
        }
    }
    size_t n_frames = (channels > 1) ? (n_samples / (size_t)channels) : n_samples;
    if (channels > 1) {
        for (size_t f = 0; f < n_frames; ++f) {
            samples[f] = samples[f * (size_t)channels];
        }
    }

    if (samp_rate <= 0 || bit_rate <= 0 || (samp_rate % bit_rate) != 0) {
        fprintf(stderr, "rx_replay: samp_rate (%d) must be a multiple of "
                "bit_rate (%d)\n", samp_rate, bit_rate);
        free(samples);
        return 1;
    }
    int sps = samp_rate / bit_rate;

    // HMAC key.
    uint8_t hmac_key[128];
    ssize_t hmac_key_len = 0;
    if (use_hmac) {
        char default_path[512];
        const char *path = keyfile_path;
        if (path == NULL) {
            if (hmac_keyfile_default_path(default_path, sizeof default_path) != 0) {
                fprintf(stderr, "rx_replay: HOME unset; pass --keyfile=<path>\n");
                free(samples);
                return 1;
            }
            path = default_path;
        }
        hmac_key_len = hmac_keyfile_load(path, hmac_key, sizeof hmac_key);
        if (hmac_key_len < 0) { free(samples); return 1; }
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

    size_t window_samples = (size_t)(window_s * (double)samp_rate);
    size_t slide_samples  = (size_t)(slide_s  * (double)samp_rate);
    if (slide_samples == 0) slide_samples = 1;
    if (slide_samples > window_samples) slide_samples = window_samples;
    size_t bits_cap = window_samples / (size_t)sps + 16;
    size_t bytes_cap = bits_cap / 8 + 16;
    uint8_t *bits_scratch  = (uint8_t *)malloc(bits_cap);
    uint8_t *bytes_scratch = (uint8_t *)malloc(bytes_cap);
    uint8_t packet[4100];
    if (bits_scratch == NULL || bytes_scratch == NULL) {
        free(bits_scratch); free(bytes_scratch); free(samples);
        return 1;
    }

    decode_loop_set_show_headers(show_packet_headers);

    char db_run_id[24];
    packet_db_t *db = packet_db_setup(db_path, no_db,
                                      db_run_id, sizeof db_run_id);
    if (source_run_override != NULL) {
        snprintf(db_run_id, sizeof db_run_id, "%s", source_run_override);
    }
    // In --update mode, suppress emit_frame's INSERT — we manually
    // call packet_db_update_observer per packet so existing rows
    // (typically from the original b210_rx_live capture) get the
    // observer / TLE / session_dir gaps filled in without piling up
    // duplicate rx_replay rows in the DB.
    decode_loop_set_packet_db(update_mode ? NULL : db, "rx_replay", db_run_id);

    // Auto-discover TLE in the pass folder if --tle wasn't given.
    // Prefer --session-dir when the caller set it (decode_passes.sh
    // does, even when the input file is a temp WAV from an ogg
    // conversion that sits in /tmp), and fall back to the input file's
    // own directory for the b210_rx_live single-TLE-per-folder layout.
    char tle_path_buf[1024];
    char search_dir_buf[1024];
    const char *search_dir;
    if (session_dir_arg != NULL) {
        search_dir = session_dir_arg;
    } else {
        snprintf(search_dir_buf, sizeof search_dir_buf, "%s", input_path);
        search_dir = dirname(search_dir_buf);
    }
    if (tle_path == NULL && db != NULL) {
        if (autodiscover_tle_in_dir(search_dir, tle_path_buf,
                                    sizeof tle_path_buf) == 0) {
            tle_path = tle_path_buf;
            fprintf(stderr, "rx_replay: auto-discovered TLE %s\n", tle_path);
        }
    }

    long long tle_id = 0;
    char tle_name[128]  = "";
    char tle_line1[128] = "";
    char tle_line2[128] = "";
    if (db != NULL && tle_path != NULL) {
        if (read_tle_lines(tle_path, sat_arg,
                           tle_name,  sizeof tle_name,
                           tle_line1, sizeof tle_line1,
                           tle_line2, sizeof tle_line2) == 0) {
            tle_id = packet_db_register_tle(db, tle_name, tle_line1, tle_line2);
            if (tle_id > 0) decode_loop_set_tle_id(tle_id);
        } else {
            fprintf(stderr, "rx_replay: TLE %s did not yield a 3-line block "
                    "for %s — observer state will be NULL\n",
                    tle_path, sat_arg ? sat_arg : "(first sat in file)");
        }
    }

    // Determine the absolute UTC of audio_offset_s=0 — needed to
    // propagate SGP4 per-packet. Order: --start-utc, then UT= in the
    // input filename, then file mtime as a last resort.
    double start_utc_seconds = 0.0;
    int have_start_utc = 0;
    if (start_utc_arg != NULL) {
        if (parse_iso_to_unix_seconds(start_utc_arg, &start_utc_seconds) == 0) {
            have_start_utc = 1;
        } else {
            fprintf(stderr, "rx_replay: bad --start-utc='%s' "
                    "(want YYYY-MM-DDTHH:MM:SS[.fff]Z)\n", start_utc_arg);
        }
    }
    if (!have_start_utc) {
        if (parse_ut_from_filename(input_path, &start_utc_seconds) == 0) {
            have_start_utc = 1;
        }
    }
    if (!have_start_utc) {
        struct stat st;
        if (stat(input_path, &st) == 0) {
            start_utc_seconds = (double)st.st_mtime;
            have_start_utc = 1;
            fprintf(stderr,
                    "rx_replay: no UT stamp in filename and --start-utc "
                    "not set; falling back to file mtime\n");
        }
    }
    // Plumb the anchor into decode_loop so emit_frame's "t=NN.NNNs"
    // entry stamps ts_received with the actual transmission UTC
    // (anchor + offset) rather than wall-clock-of-decode. NaN tells
    // record_packet "no anchor — fall back to wall clock."
    decode_loop_set_audio_clock_anchor(have_start_utc
                                       ? start_utc_seconds
                                       : (0.0 / 0.0));

    // Session dir — explicit flag wins, else dirname of input file.
    static char session_dir_buf[1024];
    if (session_dir_arg != NULL) {
        snprintf(session_dir_buf, sizeof session_dir_buf, "%s", session_dir_arg);
    } else {
        char tmp[1024];
        snprintf(tmp, sizeof tmp, "%s", input_path);
        snprintf(session_dir_buf, sizeof session_dir_buf, "%s", dirname(tmp));
    }
    if (db != NULL) decode_loop_set_session_dir(session_dir_buf);
    if (db != NULL) decode_loop_set_capture_origin(capture_origin);

#ifdef WITH_SGP4SDP4
    // SGP4 propagation state, only used when --tle was given (or
    // auto-discovered) and a start-utc could be resolved.
    prediction_t pred;
    int have_pred = 0;
    if (tle_id > 0 && have_start_utc) {
        memset(&pred, 0, sizeof pred);
        pred.observer_ephem.position_geodetic.lat = obs_lat_deg * (M_PI / 180.0);
        pred.observer_ephem.position_geodetic.lon = obs_lon_deg * (M_PI / 180.0);
        pred.observer_ephem.position_geodetic.alt = obs_alt_m / 1000.0;
        pred.tles_filename = (char *)tle_path;
        pred.satellite_ephem.name = (char *)(sat_arg ? sat_arg : tle_name);
        if (load_tle(&pred) == 0) {
            ClearFlag(ALL_FLAGS);
            select_ephemeris(&pred.satellite_ephem.tle);
            have_pred = 1;
        } else {
            fprintf(stderr, "rx_replay: load_tle failed; observer state "
                    "will be NULL\n");
        }
    }
#else
    int have_pred = 0;
    (void)obs_lat_deg; (void)obs_lon_deg; (void)obs_alt_m;
#endif

    if (update_mode && db == NULL) {
        fprintf(stderr, "rx_replay: --update needs a writable DB; "
                "remove --no-db or pass --db=<path>\n");
        return 1;
    }
    if (update_mode && tle_id == 0) {
        fprintf(stderr, "rx_replay: --update without a working TLE — "
                "the only thing to backfill would be session_dir\n");
    }

    // Position-quantised dedup ring (mirrors rx_live).
    enum { DEDUP_RING_SZ = 64 };
    enum { DEDUP_QUANT_SAMPLES = 4800 };
    uint64_t recent_pos_quant[DEDUP_RING_SZ] = {0};
    int recent_idx = 0;
    int recent_count = 0;

    // Same sizing rationale as rx_live: input_path can be long; the
    // TUI truncates to terminal width on render, so oversized is fine.
    char tui_header[512];
    if (use_tui) {
        if (rx_tui_init() != 0) {
            fprintf(stderr, "rx_replay: --ui requested but ncurses is not "
                    "built in (rebuild with libncurses-dev installed).\n");
            free(bits_scratch); free(bytes_scratch); free(samples);
            return 1;
        }
        snprintf(tui_header, sizeof tui_header,
                 "rx_replay | %s | rate=%dHz win=%.2fs slide=%.2fs "
                 "rs=%s hmac=%s | dur=%.1fs",
                 input_path, samp_rate, window_s, slide_s,
                 use_rs ? "on" : "off",
                 use_hmac ? "on" : "off",
                 (double)n_frames / (double)samp_rate);
        rx_tui_set_header(tui_header);
        rx_tui_set_command_handler(rx_replay_cmd_handler, NULL);
        quiet = 1;
    } else {
        fprintf(stderr,
                "rx_replay: %s  rate=%d Hz  channels=%d  bit_rate=%d  "
                "window=%.2fs  slide=%.2fs  sync_thr=%d  rs=%s  hmac=%s  "
                "partial=%s  duration=%.3fs\n",
                input_path, samp_rate, channels, bit_rate,
                window_s, slide_s, sync_max_ham,
                use_rs ? "on" : "off",
                use_hmac ? "on" : "off",
                allow_partial_rs ? "on" : "off",
                (double)n_frames / (double)samp_rate);
    }

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int n_emitted = 0;
    for (size_t window_start = 0;
         window_start + window_samples <= n_frames && !g_stop;
         window_start += slide_samples)
    {
        const int16_t *win = samples + window_start;
        size_t inner_min_offset = 0;
        for (;;) {
            ssize_t plen = -1;
            int golay_errs = 0, hmac_ok = -1;
            int rs_errs = -1, used_golay_len = -1;
            int rs_locs[32];
            size_t sync_off_local = 0;
            if (!try_decode_window(win, window_samples, &mp, &opts,
                                   sync_max_ham, use_hmac,
                                   allow_partial_rs,
                                   inner_min_offset,
                                   bits_scratch, bits_cap,
                                   bytes_scratch, bytes_cap,
                                   packet, sizeof packet,
                                   &plen, &golay_errs, &hmac_ok,
                                   &rs_errs, &used_golay_len,
                                   &sync_off_local,
                                   rs_locs)) break;
            inner_min_offset = sync_off_local + 1;
            if (plen < 4 || (size_t)plen > sizeof packet) continue;

            // CSP zlib CRC32 trailer (opt-in via --csp-crc32; mirrors
            // rx_live so the same bytes produce the same output).
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
                    plen -= 4;
                } else {
                    crc_status = 0;
                }
            }

            uint64_t asm_abs_sample = (uint64_t)window_start
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

            char ts[32];
            double t_sec = (double)asm_abs_sample / (double)samp_rate;
            snprintf(ts, sizeof ts, "t=%.3fs", t_sec);

            // Observer state for this packet. SGP4-propagate to the
            // moment in absolute UTC when this burst's ASM was
            // detected (start_utc + audio_offset_s). Skipped (and the
            // setter cleared back to NaN) when no TLE / no start_utc
            // was available.
            double az_deg = (0.0/0.0), el_deg = (0.0/0.0);
            double range_km = (0.0/0.0), range_rate_km_s = (0.0/0.0);
            double doppler_hz = (0.0/0.0);
#ifdef WITH_SGP4SDP4
            if (have_pred && have_start_utc) {
                double abs_t = start_utc_seconds + t_sec;
                time_t epoch = (time_t)abs_t;
                struct tm utc;
                gmtime_r(&epoch, &utc);
                struct timeval tv;
                tv.tv_sec = epoch;
                tv.tv_usec = (long)((abs_t - epoch) * 1e6);
                double jul_utc = Julian_Date(&utc, &tv);
                update_satellite_position(&pred, jul_utc);
                az_deg = pred.satellite_ephem.azimuth;
                el_deg = pred.satellite_ephem.elevation;
                range_km = pred.satellite_ephem.range_km;
                range_rate_km_s = pred.satellite_ephem.range_rate_km_s;
                doppler_hz = nominal_freq_hz
                           * (-range_rate_km_s / 299792.458);
            }
#endif
            decode_loop_set_observer(az_deg, el_deg, range_km,
                                     range_rate_km_s, doppler_hz);

            emit_frame(log_path, quiet, ts,
                       packet, (size_t)plen,
                       golay_errs, hmac_ok, use_hmac,
                       rs_errs, used_golay_len,
                       crc_status, crc_computed, crc_le, crc_be,
                       rs_locs,
                       ref_buf_len > 0 ? ref_buf : NULL, ref_buf_len,
                       force_beacon);

            // --update mode: emit_frame's INSERT was suppressed (db
            // passed as NULL above), so backfill the existing rows
            // for this payload — typically the b210_rx_live row from
            // the original capture — with whatever observer state we
            // have now.
            if (update_mode && db != NULL && plen >= 4) {
                size_t pl = (size_t)plen;
                packet_db_update_observer(db, packet + 4, pl - 4,
                                          az_deg, el_deg, range_km,
                                          range_rate_km_s, doppler_hz,
                                          tle_id,
                                          session_dir_buf,
                                          /*force=*/0);
                // Rewrite ts_received on existing rx_replay rows that
                // were stamped with wall-clock-of-decode. Scoped to
                // source_tool='rx_replay' inside the update so live
                // rows (correctly stamped at reception) don't get
                // trampled. Only meaningful when we have a UT anchor.
                if (have_start_utc) {
                    double abs_t = start_utc_seconds + t_sec;
                    time_t epoch = (time_t)floor(abs_t);
                    double frac = abs_t - (double)epoch;
                    long ms_long = (long)(frac * 1000.0 + 0.5);
                    if (ms_long >= 1000) { ms_long = 0; epoch += 1; }
                    struct tm utc;
                    gmtime_r(&epoch, &utc);
                    char ts_iso[40];
                    snprintf(ts_iso, sizeof ts_iso,
                             "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                             (utc.tm_year + 1900) % 10000,
                             (utc.tm_mon + 1) % 100,
                             utc.tm_mday % 100,
                             utc.tm_hour % 100,
                             utc.tm_min  % 100,
                             utc.tm_sec  % 100,
                             (int)(ms_long % 1000));
                    packet_db_update_replay_ts(db, packet + 4, pl - 4,
                                               ts_iso, t_sec);
                }
            }
            if (use_tui) {
                rx_tui_observe_frame(ts, packet, (size_t)plen,
                                     golay_errs, hmac_ok, use_hmac,
                                     rs_errs, crc_status);
            }
            ++n_emitted;
        }
        if (use_tui && rx_tui_tick()) goto done;
    }

done:
    if (use_tui) {
        // Replay finished but the operator may want to read the panels;
        // hold the screen until they press q (skipped if a SIGINT
        // already asked us to quit). Tear down ncurses before printing
        // the final stderr summary so it lands in the shell rather
        // than getting clobbered by curses cleanup.
        if (!g_stop) rx_tui_hold_until_quit();
        rx_tui_close();
    }
    fprintf(stderr, "rx_replay: %d frame(s) emitted.\n", n_emitted);
    free(bits_scratch);
    free(bytes_scratch);
    free(samples);
    return 0;
}
