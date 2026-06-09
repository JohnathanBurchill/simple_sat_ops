/*

    Simple Satellite Operations  utils/rx_replay.c

    Offline equivalent of b210_rx_tx: same sliding-window AX100
    decoder, same partial-RS rescue, same position-quantised dedup — but
    the samples come from a WAV or headerless S16_LE PCM file rather
    than live IQ. Use it to re-process the .wav companion b210_rx_tx
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

#include "argparse.h"
#include "ax100.h"
#include "csp.h"
#include "decode_loop.h"
#include "hmac_keyfile.h"
#include "iq_burst.h"
#include "modem.h"
#include "modem_fsk.h"
#include "modem_iq.h"
#include "packet_db.h"
#include "rx_tui.h"
#include "sso_audit.h"
#include "sw_nco.h"
#include "tle_csv.h"
#include "wav_read.h"

#ifdef HAVE_SNDFILE
#include <sndfile.h>
#endif

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
        // Cap the dir + name widths so the "/" + NUL always fit in
        // the 1024-byte buffer. NAME_MAX is 255 on every filesystem
        // we'll encounter; that leaves plenty of dir headroom.
        snprintf(hit, sizeof hit, "%.512s/%.510s", dir, de->d_name);
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

// (unix_seconds_to_iso was the inverse of parse_iso_to_unix_seconds and
// got removed when the timestamp-formatting code moved to the worker
// chain. Resurrect from git if rx_replay ever needs to print ISO times
// directly again.)

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

// Read just the sample rate from a 16-bit PCM RIFF/WAVE file's header,
// without loading the data. Used to auto-detect the IQ rate of a .iq
// sidecar from its companion .wav. Returns the rate in Hz on success,
// or -1 if the file can't be opened / isn't a valid RIFF/WAVE header.
static int read_wav_samp_rate(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) return -1;
    uint8_t hdr[44];
    size_t n = fread(hdr, 1, sizeof hdr, f);
    fclose(f);
    if (n < sizeof hdr) return -1;
    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0
        || memcmp(hdr + 12, "fmt ", 4) != 0) {
        return -1;
    }
    uint32_t rate = (uint32_t) hdr[24]
                  | ((uint32_t) hdr[25] << 8)
                  | ((uint32_t) hdr[26] << 16)
                  | ((uint32_t) hdr[27] << 24);
    if (rate < 8000 || rate > 4000000) return -1;
    return (int) rate;
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

#ifdef HAVE_SNDFILE
// Decode a compressed audio recording (a SatNOGS .ogg, which is the
// receiver's FM-discriminated audio — the same kind of PCM as the .wav
// path, just Vorbis-compressed) to mono int16 in memory via libsndfile.
// Multi-channel input is downmixed to channel 0. Returns 0 with
// *out_samples / *out_n / *out_rate, -1 on error. Caller frees samples.
static int read_audio_sndfile(const char *path, int16_t **out_samples,
                              size_t *out_n, int *out_rate)
{
    SF_INFO info;
    memset(&info, 0, sizeof info);
    SNDFILE *sf = sf_open(path, SFM_READ, &info);
    if (sf == NULL) {
        fprintf(stderr, "rx_replay: sf_open(%s): %s\n", path, sf_strerror(NULL));
        return -1;
    }
    if (info.frames <= 0 || info.channels < 1) {
        fprintf(stderr, "rx_replay: %s has no decodable audio\n", path);
        sf_close(sf);
        return -1;
    }
    size_t ch     = (size_t) info.channels;
    size_t frames = (size_t) info.frames;
    // Read as float and saturate to int16 ourselves. libsndfile's
    // sf_readf_short WRAPS samples whose source float exceeds +/-1.0
    // (overdriven SatNOGS recordings do) into garbage instead of
    // clipping, which corrupts exactly the loud signal and kills the
    // decode; SFC_SET_CLIPPING does not fix this path. Float + clamp
    // matches what ffmpeg's s16 conversion produces, sample-for-sample.
    float   *inter = (float *)   malloc(frames * ch * sizeof(float));
    int16_t *mono  = (int16_t *) malloc(frames * sizeof(int16_t));
    if (inter == NULL || mono == NULL) {
        fprintf(stderr, "rx_replay: out of memory decoding %s\n", path);
        free(inter); free(mono); sf_close(sf);
        return -1;
    }
    sf_count_t got = sf_readf_float(sf, inter, (sf_count_t) frames);
    sf_close(sf);
    if (got <= 0) { free(inter); free(mono); return -1; }
    size_t nf = (size_t) got;
    for (size_t f = 0; f < nf; ++f) {
        double s = (double) inter[f * ch] * 32768.0;  // keep ch 0
        if (s >  32767.0) s =  32767.0;
        if (s < -32768.0) s = -32768.0;
        mono[f] = (int16_t) lround(s);
    }
    free(inter);
    *out_samples = mono;
    *out_n       = nf;
    *out_rate    = info.samplerate;
    return 0;
}
#endif

// Parsed command-line configuration. parse_args() fills this; main() copies
// the fields out into working locals so the (large) decode body is unchanged.
typedef struct {
    const char *input_path;
    const char *log_path;
    const char *keyfile_path;
    int raw_mode;
    int raw_mode_explicit;
    int raw_rate;
    int raw_rate_explicit;
    int raw_channels;
    int iq_mode;
    int iq_mode_explicit;
    double lo_shift_hz;
    int use_viterbi;
    int two_pass;
    int bit_rate;
    double window_s;
    double slide_s;
    int sync_max_ham;
    int use_hmac;
    int use_rs;
    int no_dc_block;
    int csp_crc32;
    int allow_partial_rs;
    int quiet;
    int use_tui;
    const char *ref_hex_arg;
    int force_beacon;
    int show_packet_headers;
    const char *db_path;
    const char *source_run_override;
    int no_db;
    const char *tle_path;
    const char *sat_arg;
    const char *start_utc_arg;
    const char *session_dir_arg;
    const char *capture_origin;
    int update_mode;
    const char *burst_csv_arg;
    int burst_csv_suppress;
    int burst_bins_threshold;
    int burst_min_quiet;
    int burst_merge_ms;
    double obs_lat_deg;
    double obs_lon_deg;
    double obs_alt_m;
    int no_observer;
    double nominal_freq_hz;
    const char *anchor_csv_arg;
    double anchor_window_s;
    double anchor_pre_s;
} rxr_args_t;

// Option column width: the widest label below ("--burst-bins-threshold=<n>") +
// a small margin. See src/cli/argparse.h for the parse_args convention.
#define OPTW 28

// Parse argv into *a (help == 0), or print one right-aligned help line per
// option and return (help != 0). Each option is one self-contained block whose
// test carries "|| help", so help mode falls through and prints them all.
static int parse_args(rxr_args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // <path>: first non-option token. A lone "-" counts as a positional.
        // Declared first so it lists above the --options in help.
        if ((a->input_path == NULL && (arg[0] != '-' || strcmp(arg, "-") == 0)) || help) {
            if (help) parse_help_line(OPTW, "<path>", "WAV, headerless S16_LE PCM, or SatNOGS .ogg recording");
            else a->input_path = arg;
            matched = 1;
        }
        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "--raw") == 0 || help) {
            if (help) parse_help_line(OPTW, "--raw", "treat <path> as headerless S16_LE PCM (auto for .raw)");
            else { a->raw_mode = 1; a->raw_mode_explicit = 1; }
            matched = 1;
        }
        if (strcmp(arg, "--iq") == 0 || help) {
            if (help) parse_help_line(OPTW, "--iq", "treat <path> as headerless int16 I,Q pairs (auto for .iq)");
            else { a->iq_mode = 1; a->iq_mode_explicit = 1; }
            matched = 1;
        }
        if (starts_with(arg, "--burst-csv=") || help) {
            if (help) parse_help_line(OPTW, "--burst-csv=<path>", "burst.csv output path (--iq; default <input>.burst.csv)");
            else a->burst_csv_arg = arg + 12;
            matched = 1;
        }
        if (strcmp(arg, "--no-burst-csv") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-burst-csv", "suppress the burst.csv output");
            else a->burst_csv_suppress = 1;
            matched = 1;
        }
        if (starts_with(arg, "--burst-bins-threshold=") || help) {
            if (help) parse_help_line(OPTW, "--burst-bins-threshold=<n>", "bright-bin count to declare a burst (default 16)");
            else {
                a->burst_bins_threshold = atoi(arg + 23);
                if (a->burst_bins_threshold < 1) a->burst_bins_threshold = 1;
            }
            matched = 1;
        }
        if (starts_with(arg, "--burst-min-quiet=") || help) {
            if (help) parse_help_line(OPTW, "--burst-min-quiet=<n>", "quiet snapshots needed to end a burst (default 5)");
            else {
                a->burst_min_quiet = atoi(arg + 18);
                if (a->burst_min_quiet < 1) a->burst_min_quiet = 1;
            }
            matched = 1;
        }
        if (starts_with(arg, "--burst-merge-ms=") || help) {
            if (help) parse_help_line(OPTW, "--burst-merge-ms=<ms>", "merge bursts closer than this gap (default 400)");
            else {
                a->burst_merge_ms = atoi(arg + 17);
                if (a->burst_merge_ms < 0) a->burst_merge_ms = 0;
            }
            matched = 1;
        }
        if (starts_with(arg, "--lo-shift-khz=") || help) {
            if (help) parse_help_line(OPTW, "--lo-shift-khz=<N>", "NCO-shift the loaded IQ by -N kHz first (--iq; default 0)");
            else a->lo_shift_hz = atof(arg + 15) * 1000.0;
            matched = 1;
        }
        if (strcmp(arg, "--viterbi") == 0 || help) {
            if (help) parse_help_line(OPTW, "--viterbi", "force the Viterbi MLSE in --iq mode (implies --no-two-pass)");
            else a->use_viterbi = 1;
            matched = 1;
        }
        if (strcmp(arg, "--no-viterbi") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-viterbi", "disable the Viterbi MLSE; pass-1 slicer only");
            else a->use_viterbi = 0;
            matched = 1;
        }
        if (strcmp(arg, "--two-pass") == 0 || help) {
            if (help) parse_help_line(OPTW, "--two-pass", "enable two-pass decode (default in --iq mode)");
            else a->two_pass = 1;
            matched = 1;
        }
        if (strcmp(arg, "--no-two-pass") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-two-pass", "single pass with whichever chain --viterbi selects");
            else a->two_pass = 0;
            matched = 1;
        }
        if (starts_with(arg, "--rate=") || help) {
            if (help) parse_help_line(OPTW, "--rate=<hz>", "sample rate for --raw / --iq (default 48000)");
            else { a->raw_rate = atoi(arg + 7); a->raw_rate_explicit = 1; }
            matched = 1;
        }
        if (starts_with(arg, "--channels=") || help) {
            if (help) parse_help_line(OPTW, "--channels=<n>", "channels for --raw (default 2; ch 0 used; --iq ignores)");
            else a->raw_channels = atoi(arg + 11);
            matched = 1;
        }
        if (starts_with(arg, "--bit-rate=") || help) {
            if (help) parse_help_line(OPTW, "--bit-rate=<bps>", "modem bit rate (default 9600)");
            else a->bit_rate = atoi(arg + 11);
            matched = 1;
        }
        if (starts_with(arg, "--window-s=") || help) {
            if (help) parse_help_line(OPTW, "--window-s=<seconds>", "decoder window size (default 1.5; clamped 0.5..30)");
            else {
                a->window_s = atof(arg + 11);
                if (a->window_s < 0.5) a->window_s = 0.5;
                if (a->window_s > 30.0) a->window_s = 30.0;
            }
            matched = 1;
        }
        if (starts_with(arg, "--slide-s=") || help) {
            if (help) parse_help_line(OPTW, "--slide-s=<seconds>", "slide between decode attempts (default 0.5; min 0.05)");
            else {
                a->slide_s = atof(arg + 10);
                if (a->slide_s < 0.05) a->slide_s = 0.05;
            }
            matched = 1;
        }
        if (starts_with(arg, "--sync-threshold=") || help) {
            if (help) parse_help_line(OPTW, "--sync-threshold=<0..8>", "max ASM bit errors (default 4)");
            else {
                a->sync_max_ham = atoi(arg + 17);
                if (a->sync_max_ham < 0 || a->sync_max_ham > 8) {
                    fprintf(stderr, "rx_replay: --sync-threshold out of range [0,8]\n");
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strcmp(arg, "--hmac") == 0 || help) {
            if (help) parse_help_line(OPTW, "--hmac", "enable HMAC verification (off by default)");
            else a->use_hmac = 1;
            matched = 1;
        }
        if (strcmp(arg, "--no-hmac") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-hmac", "disable HMAC verification (the default)");
            else a->use_hmac = 0;
            matched = 1;
        }
        if (strcmp(arg, "--reed-solomon") == 0 || help) {
            if (help) parse_help_line(OPTW, "--reed-solomon", "RS(255,223) decode (DEFAULT)");
            else a->use_rs = 1;
            matched = 1;
        }
        if (strcmp(arg, "--no-reed-solomon") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-reed-solomon", "skip RS decode");
            else a->use_rs = 0;
            matched = 1;
        }
        if (strcmp(arg, "--no-dc-block") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-dc-block", "skip the modem's DC-block IIR on RX");
            else a->no_dc_block = 1;
            matched = 1;
        }
        if (strcmp(arg, "--csp-crc32") == 0 || help) {
            if (help) parse_help_line(OPTW, "--csp-crc32", "validate + strip a trailing CSP zlib CRC32");
            else a->csp_crc32 = 1;
            matched = 1;
        }
        if (strcmp(arg, "--no-csp-crc32") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-csp-crc32", "do not check a CSP CRC32 (the default)");
            else a->csp_crc32 = 0;
            matched = 1;
        }
        if (strcmp(arg, "--no-partial-rs") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-partial-rs", "disable the partial-RS rescue path");
            else a->allow_partial_rs = 0;
            matched = 1;
        }
        if (strcmp(arg, "--quiet") == 0 || help) {
            if (help) parse_help_line(OPTW, "--quiet", "skip stdout output (log-only mode)");
            else a->quiet = 1;
            matched = 1;
        }
        if (strcmp(arg, "--ui") == 0 || help) {
            if (help) parse_help_line(OPTW, "--ui", "curses panel display instead of streaming text");
            else a->use_tui = 1;
            matched = 1;
        }
        if (starts_with(arg, "--keyfile=") || help) {
            if (help) parse_help_line(OPTW, "--keyfile=<path>", "HMAC keyfile (default $HOME/" HMAC_KEYFILE_DEFAULT_RELPATH ")");
            else a->keyfile_path = arg + 10;
            matched = 1;
        }
        if (starts_with(arg, "--log=") || help) {
            if (help) parse_help_line(OPTW, "--log=<path>", "append decode lines to <path> as well as stdout");
            else a->log_path = arg + 6;
            matched = 1;
        }
        if (starts_with(arg, "--ref-hex=") || help) {
            if (help) parse_help_line(OPTW, "--ref-hex=<hex>", "diff each decoded packet/payload against this reference hex");
            else a->ref_hex_arg = arg + 10;
            matched = 1;
        }
        if (strcmp(arg, "--force-beacon") == 0 || help) {
            if (help) parse_help_line(OPTW, "--force-beacon", "pad each payload to 130 bytes and print as a beacon");
            else a->force_beacon = 1;
            matched = 1;
        }
        if (strcmp(arg, "--packet-headers") == 0 || help) {
            if (help) parse_help_line(OPTW, "--packet-headers", "show AX100/CSP header lines and per-frame hex/ascii");
            else a->show_packet_headers = 1;
            matched = 1;
        }
        if (strcmp(arg, "--no-packet-headers") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-packet-headers", "default; hide header lines (kept as a no-op for scripts)");
            else a->show_packet_headers = 0;
            matched = 1;
        }
        if (starts_with(arg, "--db=") || help) {
            if (help) parse_help_line(OPTW, "--db=<path>", "SQLite store for decoded packets");
            else a->db_path = arg + 5;
            matched = 1;
        }
        if (strcmp(arg, "--no-db") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-db", "skip DB writes");
            else a->no_db = 1;
            matched = 1;
        }
        if (starts_with(arg, "--source-run=") || help) {
            if (help) parse_help_line(OPTW, "--source-run=<id>", "override the per-launch run-id used for dedup");
            else a->source_run_override = arg + 13;
            matched = 1;
        }
        if (starts_with(arg, "--tle=") || help) {
            if (help) parse_help_line(OPTW, "--tle=<path>", "TLE file for observer geometry (else auto-discovered)");
            else a->tle_path = tle_path_resolve(arg + 6);
            matched = 1;
        }
        if (starts_with(arg, "--satellite=") || help) {
            if (help) parse_help_line(OPTW, "--satellite=<name>", "satellite name prefix to pick out of the TLE file");
            else a->sat_arg = arg + 12;
            matched = 1;
        }
        if (starts_with(arg, "--start-utc=") || help) {
            if (help) parse_help_line(OPTW, "--start-utc=<iso>", "UTC of sample 0 (YYYY-MM-DDTHH:MM:SS[.fff]Z)");
            else a->start_utc_arg = arg + 12;
            matched = 1;
        }
        if (starts_with(arg, "--session-dir=") || help) {
            if (help) parse_help_line(OPTW, "--session-dir=<path>", "pass folder for TLE auto-discovery and DB rows");
            else a->session_dir_arg = arg + 14;
            matched = 1;
        }
        if (starts_with(arg, "--capture-origin=") || help) {
            if (help) parse_help_line(OPTW, "--capture-origin=<name>", "tag rows with the audio's provenance (e.g. satnogs)");
            else a->capture_origin = arg + 17;
            matched = 1;
        }
        if (strcmp(arg, "--update") == 0 || help) {
            if (help) parse_help_line(OPTW, "--update", "backfill existing DB rows; suppress new INSERTs");
            else a->update_mode = 1;
            matched = 1;
        }
        if (starts_with(arg, "--lat=") || help) {
            if (help) parse_help_line(OPTW, "--lat=<deg>", "observer latitude (default RAO 50.8688)");
            else a->obs_lat_deg = atof(arg + 6);
            matched = 1;
        }
        if (starts_with(arg, "--lon=") || help) {
            if (help) parse_help_line(OPTW, "--lon=<deg>", "observer longitude (default RAO -114.2910)");
            else a->obs_lon_deg = atof(arg + 6);
            matched = 1;
        }
        if (starts_with(arg, "--alt=") || help) {
            if (help) parse_help_line(OPTW, "--alt=<m>", "observer altitude in metres (default RAO 1279)");
            else a->obs_alt_m = atof(arg + 6);
            matched = 1;
        }
        if (strcmp(arg, "--no-observer") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-observer", "leave geometry NULL instead of using --lat/--lon/--alt");
            else a->no_observer = 1;
            matched = 1;
        }
        if (starts_with(arg, "--carrier-mhz=") || help) {
            if (help) parse_help_line(OPTW, "--carrier-mhz=<mhz>", "nominal carrier for Doppler (default 436.150)");
            else a->nominal_freq_hz = atof(arg + 14) * 1e6;
            matched = 1;
        }
        if (starts_with(arg, "--anchor-csv=") || help) {
            if (help) parse_help_line(OPTW, "--anchor-csv=<path>", "decode at external (time, freq) burst.csv anchors (--iq)");
            else a->anchor_csv_arg = arg + 13;
            matched = 1;
        }
        if (starts_with(arg, "--anchor-window-s=") || help) {
            if (help) parse_help_line(OPTW, "--anchor-window-s=<f>", "tight window around each anchor (default 0.40)");
            else a->anchor_window_s = atof(arg + 18);
            matched = 1;
        }
        if (starts_with(arg, "--anchor-pre-s=") || help) {
            if (help) parse_help_line(OPTW, "--anchor-pre-s=<f>", "pre-anchor cushion for M&M settling (default 0.05)");
            else a->anchor_pre_s = atof(arg + 14);
            matched = 1;
        }

        if (!matched && !help) {
            if (arg[0] == '-' && strcmp(arg, "-") != 0)
                fprintf(stderr, "rx_replay: unknown option '%s'\n", arg);
            else
                fprintf(stderr, "rx_replay: unexpected positional '%s'\n", arg);
            return PARSE_ERROR;
        }
    }
    return PARSE_OK;
}

// Bundle of per-run state the emit helper needs. Built once in main()
// and shared by pass-1 (slicer sliding window) and pass-2 (anchored
// Viterbi) so the same dedup ring, observer geometry, and packet-DB
// hookups apply uniformly to both.
typedef struct {
    int             samp_rate;
    int             sps;
    int             csp_crc32;
    int             use_hmac;
    int             update_mode;
    int             have_start_utc;
    double          start_utc_seconds;
    int             have_pred;
#ifdef WITH_SGP4SDP4
    prediction_t   *pred;
#endif
    double          nominal_freq_hz;
    long long       tle_id;
    const char     *session_dir;
    packet_db_t    *db;
    const char     *log_path;
    int             quiet;
    int             use_tui;
    int             force_beacon;
    const uint8_t  *ref_buf;
    size_t          ref_buf_len;
    uint64_t       *recent_pos_quant;
    int            *recent_idx;
    int            *recent_count;
    int             dedup_ring_sz;
    uint64_t        dedup_quant_samples;
    int            *n_emitted_p;
} rx_emit_ctx_t;

// Post-decode pipeline: optional CSP zlib CRC32 trim, dedup by absolute
// sample position, SGP4 observer state, emit_frame, --update DB writes,
// TUI mirror, dedup ring update, and n_emitted bump. Used by both
// pass-1 (slicer sliding window) and pass-2 (anchored Viterbi).
// Returns 1 if emitted, 0 if skipped by dedup or length check.
static int rx_emit_decoded(rx_emit_ctx_t *ctx,
                           uint64_t asm_abs_sample,
                           uint8_t *packet, ssize_t plen,
                           int golay_errs, int hmac_ok,
                           int rs_errs, int used_golay_len,
                           const int *rs_locs)
{
    if (plen < 4) return 0;

    int crc_status = -1;
    uint32_t crc_computed = 0, crc_le = 0, crc_be = 0;
    if (!ctx->use_hmac && ctx->csp_crc32 && plen >= 8) {
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

    uint64_t pos_quant = asm_abs_sample / ctx->dedup_quant_samples;
    int ring_n = *ctx->recent_count < ctx->dedup_ring_sz
        ? *ctx->recent_count : ctx->dedup_ring_sz;
    for (int r = 0; r < ring_n; ++r) {
        if (ctx->recent_pos_quant[r] == pos_quant) return 0;
    }
    ctx->recent_pos_quant[*ctx->recent_idx] = pos_quant;
    *ctx->recent_idx = (*ctx->recent_idx + 1) % ctx->dedup_ring_sz;
    if (*ctx->recent_count < ctx->dedup_ring_sz) (*ctx->recent_count)++;

    char ts[32];
    double t_sec = (double)asm_abs_sample / (double)ctx->samp_rate;
    snprintf(ts, sizeof ts, "t=%.3fs", t_sec);

    double az_deg = (0.0/0.0), el_deg = (0.0/0.0);
    double range_km = (0.0/0.0), range_rate_km_s = (0.0/0.0);
    double doppler_hz = (0.0/0.0);
#ifdef WITH_SGP4SDP4
    if (ctx->have_pred && ctx->have_start_utc && ctx->pred != NULL) {
        double abs_t = ctx->start_utc_seconds + t_sec;
        time_t epoch = (time_t)abs_t;
        struct tm utc;
        gmtime_r(&epoch, &utc);
        struct timeval tv;
        tv.tv_sec = epoch;
        tv.tv_usec = (long)((abs_t - epoch) * 1e6);
        double jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(ctx->pred, jul_utc);
        az_deg = ctx->pred->satellite_ephem.azimuth;
        el_deg = ctx->pred->satellite_ephem.elevation;
        range_km = ctx->pred->satellite_ephem.range_km;
        range_rate_km_s = ctx->pred->satellite_ephem.range_rate_km_s;
        doppler_hz = ctx->nominal_freq_hz
                   * (-range_rate_km_s / 299792.458);
        int finite_ok = isfinite(az_deg) && isfinite(el_deg)
                     && isfinite(range_km)
                     && isfinite(range_rate_km_s)
                     && isfinite(doppler_hz);
        int bounds_ok = (range_km >= 0.0 && range_km < 5.0e5)
                     && (el_deg >= -90.0 && el_deg <= 90.0)
                     && (az_deg >= -360.0 && az_deg <= 720.0);
        if (!finite_ok || !bounds_ok) {
            fprintf(stderr,
                "rx_replay: implausible geometry (az=%.2f "
                "el=%.2f range=%.1f rate=%.3f) — dropping\n",
                az_deg, el_deg, range_km, range_rate_km_s);
            az_deg = el_deg = range_km = (0.0/0.0);
            range_rate_km_s = doppler_hz = (0.0/0.0);
        }
    }
#endif
    decode_loop_set_observer(az_deg, el_deg, range_km,
                             range_rate_km_s, doppler_hz);

    emit_frame(ctx->log_path, ctx->quiet, ts,
               packet, (size_t)plen,
               golay_errs, hmac_ok, ctx->use_hmac,
               rs_errs, used_golay_len,
               crc_status, crc_computed, crc_le, crc_be,
               rs_locs,
               ctx->ref_buf_len > 0 ? ctx->ref_buf : NULL, ctx->ref_buf_len,
               ctx->force_beacon);

    if (ctx->update_mode && ctx->db != NULL && plen >= 4) {
        size_t pl = (size_t)plen;
        packet_db_update_observer(ctx->db, packet + 4, pl - 4,
                                  az_deg, el_deg, range_km,
                                  range_rate_km_s, doppler_hz,
                                  ctx->tle_id, ctx->session_dir,
                                  /*force=*/0);
        if (ctx->have_start_utc) {
            double abs_t = ctx->start_utc_seconds + t_sec;
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
            packet_db_update_replay_ts(ctx->db, packet + 4, pl - 4,
                                       ts_iso, t_sec);
        }
    }
    if (ctx->use_tui) {
        rx_tui_observe_frame(ts, packet, (size_t)plen,
                             golay_errs, hmac_ok, ctx->use_hmac,
                             rs_errs, crc_status);
    }
    (*ctx->n_emitted_p)++;
    return 1;
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "rx_replay")) return 0;
    rxr_args_t cfg = {
        .raw_rate = 48000,
        .raw_channels = 2,
        .two_pass = 1,
        .bit_rate = 9600,
        .window_s = 1.5,
        .slide_s = 0.5,
        .sync_max_ham = 4,
        .use_rs = 1,
        .allow_partial_rs = 1,
        .burst_bins_threshold = 16,
        .burst_min_quiet = 5,
        .burst_merge_ms = 400,
        .obs_lat_deg = 50.8688,   // RAO defaults; overridden by flags
        .obs_lon_deg = -114.2910,
        .obs_alt_m   = 1279.0,
        .nominal_freq_hz = 436150000.0, // FrontierSat carrier
        .anchor_window_s = 0.40,     // tight window around each anchor
        .anchor_pre_s    = 0.05,     // pre-anchor cushion for M&M lock
    };
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }
    if (cfg.input_path == NULL) {
        fprintf(stderr, "rx_replay: missing <path> (try --help)\n");
        return 1;
    }

    // Copy parsed config into the working locals the body below uses.
    const char *input_path = cfg.input_path;
    const char *log_path = cfg.log_path;
    const char *keyfile_path = cfg.keyfile_path;
    int raw_mode = cfg.raw_mode;
    int raw_mode_explicit = cfg.raw_mode_explicit;
    // Auto-detected from a .ogg extension: a SatNOGS audio recording,
    // decoded to PCM via libsndfile and run through the FM-audio chain.
    int ogg_mode = 0;
    int raw_rate = cfg.raw_rate;
    int raw_rate_explicit = cfg.raw_rate_explicit;
    int raw_channels = cfg.raw_channels;
    // IQ-file mode: treat the input as headerless interleaved int16
    // I,Q pairs at samp_rate (the format simple_sat_ops writes as the
    // .iq sidecar next to each pass WAV). Enabled by --iq or auto-
    // detected from a .iq extension. When on, the decoder runs through
    // modem_iq / modem_viterbi instead of modem_pcm16, which on noisy
    // captures pulls more frames than the FM-discriminated WAV path.
    int iq_mode = cfg.iq_mode;
    int iq_mode_explicit = cfg.iq_mode_explicit;
    // Optional pre-demod NCO shift (Hz). Current simple_sat_ops
    // captures land at DC because the live receiver cancels both the
    // operator's lo_offset and the UHD tune residual before the IQ
    // tap — so the default of 0 is right and no flag is needed for
    // anything captured after that change. Legacy .iq files that
    // still carry the +lo_offset baseband (LO offset NOT cancelled
    // before the tap) need --lo-shift-khz=N where N equals the
    // operator's lo_offset_hz in kHz (e.g. 25 for the historical
    // -25 kHz default). Mis-applying the flag on a centered file
    // pushes the carrier outside the 12 kHz LPF and breaks decode.
    double lo_shift_hz = cfg.lo_shift_hz;
    // Viterbi default off pending a fix for the FrontierSat downlink
    // modulation. Empirically (RAO captures, 2026-05-15 pass) the
    // symbol-spaced differential phase histogram peaks near ±π — that
    // is the GFSK h≈1 signature, not the MSK h=0.5 the current 4-state
    // CPM trellis is built for. The 2nd-power bias estimator then
    // latches near ±π/2 across the whole pass (independent of Doppler)
    // and the Viterbi reports no syncs. The slicer survives because
    // differential decoding doesn't care about the absolute trellis
    // structure. Pass --viterbi to force it on (e.g. for synthetic
    // h=0.5 tests).
    int use_viterbi = cfg.use_viterbi;
    // Two-pass IQ decoding. Pass 1: slicer (modem_iq_to_bits) on the
    // wide sliding window — fast, finds easy frames, populates the
    // dedup ring. Pass 2: rewalk the file collecting sync candidates
    // with the slicer, then anchor a tight (~0.4 s) window on each
    // unseen candidate and try the Viterbi MLSE. Currently a no-op on
    // FrontierSat downlinks (see use_viterbi note above) but cheap
    // and harmless; keep on so the wiring stays exercised. Revert
    // with --no-two-pass.
    int two_pass = cfg.two_pass;
    int bit_rate = cfg.bit_rate;
    double window_s = cfg.window_s;
    double slide_s = cfg.slide_s;
    int sync_max_ham = cfg.sync_max_ham;
    int use_hmac = cfg.use_hmac;
    int use_rs = cfg.use_rs;
    int no_dc_block = cfg.no_dc_block;
    int csp_crc32 = cfg.csp_crc32;
    int allow_partial_rs = cfg.allow_partial_rs;
    int quiet = cfg.quiet;
    int use_tui = cfg.use_tui;
    const char *ref_hex_arg = cfg.ref_hex_arg;
    uint8_t ref_buf[4100];
    size_t ref_buf_len = 0;
    int force_beacon = cfg.force_beacon;
    int show_packet_headers = cfg.show_packet_headers;
    const char *db_path = cfg.db_path;
    const char *source_run_override = cfg.source_run_override;
    int no_db = cfg.no_db;
    const char *tle_path = cfg.tle_path;
    const char *sat_arg  = cfg.sat_arg;
    const char *start_utc_arg = cfg.start_utc_arg;
    const char *session_dir_arg = cfg.session_dir_arg;
    const char *capture_origin = cfg.capture_origin;
    int update_mode = cfg.update_mode;
    // Burst detector (iq_burst) output — same writer as the live
    // rx_session's burst.csv. Default path is "<input>.burst.csv";
    // --burst-csv= overrides; --no-burst-csv suppresses.
    const char *burst_csv_arg     = cfg.burst_csv_arg;
    int         burst_csv_suppress = cfg.burst_csv_suppress;
    int         burst_bins_threshold = cfg.burst_bins_threshold;
    int         burst_min_quiet      = cfg.burst_min_quiet;
    // Collapse adjacent burst events into one when the gap between
    // burst_end[k] and burst_start[k+1] is shorter than this. Tracks
    // the operator's intuitive "beacon arrival" rather than the
    // detector's finer-grained start/end transitions inside one
    // packet (FSK modulation produces brief mid-packet dropouts).
    int         burst_merge_ms       = cfg.burst_merge_ms;
    double obs_lat_deg = cfg.obs_lat_deg;
    double obs_lon_deg = cfg.obs_lon_deg;
    double obs_alt_m   = cfg.obs_alt_m;
    // --no-observer leaves geometry NULL in the DB instead of falling
    // back to obs_lat/lon/alt. Used by decode_passes.sh on satnogs obs
    // whose recording-station coords aren't known — better to record
    // "we don't know where it was heard from" than to silently
    // attribute the pass to RAO.
    int no_observer = cfg.no_observer;
    double nominal_freq_hz = cfg.nominal_freq_hz;
    // Anchored-decode mode: read (time, freq) anchors from a CSV (same
    // schema gen_waterfall --show-tm consumes — burst_start rows with
    // the optional 6th freq_hz field). For each anchor we mix the IQ
    // to DC at the given freq and run the FSK chain on a tight window
    // around the anchor time. Useful when the beacon sits far off DC
    // and the wide sliding-window pass can't see it through the
    // matched filter's ±10 kHz passband.
    const char *anchor_csv_arg = cfg.anchor_csv_arg;
    double anchor_window_s = cfg.anchor_window_s;     // tight window around each anchor
    double anchor_pre_s    = cfg.anchor_pre_s;        // pre-anchor cushion for M&M lock

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
    if (!iq_mode_explicit) {
        size_t plen = strlen(input_path);
        if (plen >= 3 && strcmp(input_path + plen - 3, ".iq") == 0) iq_mode = 1;
    }
    // .ogg (SatNOGS audio) -> decode to PCM and run the FM-audio chain.
    // Only when not already raw/iq.
    if (!raw_mode && !iq_mode) {
        size_t plen = strlen(input_path);
        if (plen >= 4 && strcmp(input_path + plen - 4, ".ogg") == 0) ogg_mode = 1;
    }
    // When given a .iq without an explicit --rate, look for a companion
    // .wav alongside it (simple_sat_ops writes them in pairs) and lift
    // the sample rate from its header. Saves the caller from having
    // to remember --rate=96000 on captures from the post-Gardner-bump
    // simple_sat_ops, while keeping 48000 as the fallback for old
    // recordings and any .iq written without a .wav peer.
    if (iq_mode && !raw_rate_explicit) {
        size_t plen = strlen(input_path);
        if (plen >= 3 && strcmp(input_path + plen - 3, ".iq") == 0) {
            char wav_path[1024];
            if (plen < sizeof wav_path - 1) {
                snprintf(wav_path, sizeof wav_path, "%s", input_path);
                wav_path[plen - 2] = 'w';
                wav_path[plen - 1] = 'a';
                wav_path[plen] = 'v';
                wav_path[plen + 1] = '\0';
                int wav_rate = read_wav_samp_rate(wav_path);
                if (wav_rate > 0) {
                    if (wav_rate != raw_rate) {
                        fprintf(stderr,
                            "rx_replay: auto-detected IQ rate %d Hz "
                            "from companion %s\n",
                            wav_rate, wav_path);
                    }
                    raw_rate = wav_rate;
                }
            }
        }
    }
    if (raw_channels < 1 || raw_channels > 8) {
        fprintf(stderr, "rx_replay: --channels out of range [1,8]\n");
        return 1;
    }

    // Load samples and reduce to mono (PCM mode) or keep interleaved
    // I,Q pairs (IQ mode). For IQ the file is headerless int16 pairs,
    // already in the format try_decode_window_iq / _viterbi expect.
    int16_t *samples = NULL;
    size_t n_samples = 0;
    int samp_rate = 0;
    int channels = 1;
    if (iq_mode) {
        if (read_raw_pcm16(input_path, &samples, &n_samples) != 0) return 1;
        samp_rate = raw_rate;
        if ((n_samples & 1u) != 0) {
            fprintf(stderr,
                    "rx_replay: --iq file has odd int16 count (%zu); "
                    "not interleaved I,Q?\n", n_samples);
            free(samples);
            return 1;
        }
        // Apply --lo-shift-khz BEFORE the decode loop. sw_nco_apply
        // rotates by exp(-j 2π f · n/fs), so positive lo_shift_hz
        // moves a signal at +lo_shift_hz baseband down to DC. Default
        // is 0 because current captures already land at DC — the live
        // receiver folds the operator's lo_offset AND the UHD tune
        // residual into its second NCO before the IQ tap. The shift
        // is one-shot across the whole captured file; no need for
        // phase persistence across chunks.
        if (lo_shift_hz != 0.0) {
            sw_nco_t nco;
            sw_nco_init(&nco, (double) samp_rate);
            sw_nco_set_freq(&nco, lo_shift_hz);
            size_t n_pairs = n_samples / 2u;
            sw_nco_apply(&nco, samples, n_pairs);
            fprintf(stderr,
                "rx_replay: applied --lo-shift-khz=%g (sw NCO over %zu IQ pairs)\n",
                lo_shift_hz / 1000.0, n_pairs);
        }
    } else if (raw_mode) {
        if (read_raw_pcm16(input_path, &samples, &n_samples) != 0) return 1;
        samp_rate = raw_rate;
        channels = raw_channels;
    } else if (ogg_mode) {
#ifdef HAVE_SNDFILE
        if (read_audio_sndfile(input_path, &samples, &n_samples,
                               &samp_rate) != 0) {
            return 1;
        }
        channels = 1;  // already downmixed to mono
        if (raw_rate_explicit) samp_rate = raw_rate;  // allow override
        fprintf(stderr,
            "rx_replay: decoded %s via libsndfile (%d Hz, %zu samples)\n",
            input_path, samp_rate, n_samples);
#else
        fprintf(stderr,
            "rx_replay: %s is a .ogg but this build has no libsndfile. "
            "Rebuild with libsndfile (brew install libsndfile / "
            "apt install libsndfile1-dev), or convert it to .wav first.\n",
            input_path);
        return 1;
#endif
    } else {
        if (wav_read_pcm16(input_path, &samples, &n_samples,
                           &samp_rate, &channels) != 0) {
            return 1;
        }
    }
    size_t n_frames;
    if (iq_mode) {
        n_frames = n_samples / 2u;        // pairs
    } else {
        n_frames = (channels > 1) ? (n_samples / (size_t)channels) : n_samples;
        if (channels > 1) {
            for (size_t f = 0; f < n_frames; ++f) {
                samples[f] = samples[f * (size_t)channels];
            }
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
    // (typically from the original b210_rx_tx capture) get the
    // observer / TLE / session_dir gaps filled in without piling up
    // duplicate rx_replay rows in the DB.
    decode_loop_set_packet_db(update_mode ? NULL : db, "rx_replay", db_run_id);

    // Auto-discover TLE in the pass folder if --tle wasn't given.
    // Prefer --session-dir when the caller set it (decode_passes.sh
    // does, even when the input file is a temp WAV from an ogg
    // conversion that sits in /tmp), and fall back to the input file's
    // own directory for the b210_rx_tx single-TLE-per-folder layout.
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
            tle_path = tle_path_resolve(tle_path_buf);
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

    // Burst-detect pass (iq_mode only). Pushes the captured IQ
    // through iq_burst in n_fft-sample chunks, runs the same
    // start/end debounce state machine the live rx_session writes,
    // and produces a burst.csv that's bit-for-bit format-compatible
    // with the live file. Done BEFORE the sliding-window decode
    // loop so the burst.csv exists even if the decode pass crashes
    // or is interrupted with Ctrl-C.
    if (iq_mode && !burst_csv_suppress) {
        char burst_path[1024];
        if (burst_csv_arg != NULL) {
            snprintf(burst_path, sizeof burst_path, "%s", burst_csv_arg);
        } else {
            snprintf(burst_path, sizeof burst_path, "%s.burst.csv",
                     input_path);
        }
        FILE *bfp = fopen(burst_path, "w");
        if (bfp == NULL) {
            fprintf(stderr,
                "rx_replay: --burst-csv: cannot open %s: %s (skipping)\n",
                burst_path, strerror(errno));
        } else {
            const unsigned BURST_NFFT = 512u;
            iq_burst_t *bdet = iq_burst_new(BURST_NFFT, (double) samp_rate,
                                            /*threshold_db=*/10.0,
                                            /*floor_tau_s=*/2.0);
            if (bdet == NULL) {
                fprintf(stderr, "rx_replay: iq_burst_new failed; "
                                "skipping burst.csv\n");
                fclose(bfp);
            } else {
                fputs("# event,unix_time_ms,bright_bins,peak_excess_db,"
                      "duration_ms\n",
                      bfp);
                long long start_unix_ms =
                    have_start_utc
                    ? (long long)(start_utc_seconds * 1000.0 + 0.5) : 0;
                size_t n_pairs = n_samples / 2u;
                int  in_burst = 0;
                int  quiet    = 0;
                int  peak_bins = 0;
                double peak_db = 0.0;
                long long start_ms = 0;
                // burst_merge_ms collapse: a burst_end is "pending"
                // until either (a) the next burst_start arrives within
                // merge_ms, in which case we silently absorb the gap
                // and the new sub-burst into the current one, or (b)
                // merge_ms elapses without a new start, in which case
                // we flush the pending end as a real event.
                int       pending_end       = 0;
                long long pending_end_ms    = 0;
                int       pending_peak_bins = 0;
                double    pending_peak_db   = 0.0;
                long long pending_start_ms  = 0;  // start of the merged group
                for (size_t off = 0; off < n_pairs; off += BURST_NFFT) {
                    size_t take = n_pairs - off;
                    if (take > BURST_NFFT) take = BURST_NFFT;
                    iq_burst_push(bdet, samples + off * 2, take);
                    int    bins   = iq_burst_bright_bins(bdet);
                    double excess = iq_burst_peak_excess_db(bdet);
                    long long t_ms = start_unix_ms
                        + (long long)((double)(off + take)
                                       / (double) samp_rate * 1000.0);
                    // Flush a pending end if its merge window has
                    // expired with no new start — i.e. this snapshot
                    // is below threshold AND we're past pending_end +
                    // merge_ms.
                    if (pending_end && !in_burst
                        && (t_ms - pending_end_ms) >= burst_merge_ms) {
                        fprintf(bfp,
                            "burst_end,%lld,%d,%.2f,%lld\n",
                            pending_end_ms, pending_peak_bins,
                            pending_peak_db,
                            pending_end_ms - pending_start_ms);
                        pending_end = 0;
                    }
                    if (bins >= burst_bins_threshold) {
                        if (!in_burst) {
                            if (pending_end
                                && (t_ms - pending_end_ms) < burst_merge_ms) {
                                // Resume the previous merged group —
                                // suppress this burst_start row and
                                // accumulate into the existing peaks.
                                in_burst   = 1;
                                start_ms   = pending_start_ms;
                                peak_bins  = pending_peak_bins;
                                peak_db    = pending_peak_db;
                                pending_end = 0;
                                if (bins > peak_bins) peak_bins = bins;
                                if (excess > peak_db) peak_db = excess;
                                quiet = 0;
                            } else {
                                in_burst   = 1;
                                start_ms   = t_ms;
                                peak_bins  = bins;
                                peak_db    = excess;
                                quiet      = 0;
                                pending_start_ms = t_ms;
                                fprintf(bfp,
                                    "burst_start,%lld,%d,%.2f,\n",
                                    t_ms, bins, excess);
                            }
                        } else {
                            if (bins > peak_bins) peak_bins = bins;
                            if (excess > peak_db) peak_db = excess;
                            quiet = 0;
                        }
                    } else if (in_burst) {
                        quiet++;
                        if (quiet >= burst_min_quiet) {
                            // Stash as pending — flushed later if no
                            // new start within merge_ms.
                            pending_end       = 1;
                            pending_end_ms    = t_ms;
                            pending_peak_bins = peak_bins;
                            pending_peak_db   = peak_db;
                            in_burst          = 0;
                        }
                    }
                }
                // End-of-file: drain any in-flight or pending burst.
                long long file_end_ms = start_unix_ms
                    + (long long)((double) n_pairs
                                   / (double) samp_rate * 1000.0);
                if (in_burst) {
                    fprintf(bfp, "burst_end,%lld,%d,%.2f,%lld\n",
                            file_end_ms, peak_bins, peak_db,
                            file_end_ms - start_ms);
                } else if (pending_end) {
                    fprintf(bfp, "burst_end,%lld,%d,%.2f,%lld\n",
                            pending_end_ms, pending_peak_bins,
                            pending_peak_db,
                            pending_end_ms - pending_start_ms);
                }
                fflush(bfp);
                iq_burst_free(bdet);
                fclose(bfp);
                fprintf(stderr, "rx_replay: burst.csv -> %s\n",
                        burst_path);
            }
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
    // auto-discovered), a start-utc could be resolved, and the caller
    // didn't explicitly opt out with --no-observer.
    prediction_t pred;
    int have_pred = 0;
    if (!no_observer && tle_id > 0 && have_start_utc) {
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
    // Candidate frames handed to rx_emit_decoded across all passes,
    // before its position dedup. The gap to n_emitted is how many were
    // dropped as same-position duplicates.
    long raw_decodes = 0;
    decode_loop_reset_stats();

    // Build the emit context once: shared by both passes.
    rx_emit_ctx_t ectx = {
        .samp_rate = samp_rate,
        .sps = sps,
        .csp_crc32 = csp_crc32,
        .use_hmac = use_hmac,
        .update_mode = update_mode,
        .have_start_utc = have_start_utc,
        .start_utc_seconds = start_utc_seconds,
        .have_pred = have_pred,
#ifdef WITH_SGP4SDP4
        .pred = have_pred ? &pred : NULL,
#endif
        .nominal_freq_hz = nominal_freq_hz,
        .tle_id = tle_id,
        .session_dir = session_dir_buf,
        .db = db,
        .log_path = log_path,
        .quiet = quiet,
        .use_tui = use_tui,
        .force_beacon = force_beacon,
        .ref_buf = ref_buf_len > 0 ? ref_buf : NULL,
        .ref_buf_len = ref_buf_len,
        .recent_pos_quant = recent_pos_quant,
        .recent_idx = &recent_idx,
        .recent_count = &recent_count,
        .dedup_ring_sz = DEDUP_RING_SZ,
        .dedup_quant_samples = DEDUP_QUANT_SAMPLES,
        .n_emitted_p = &n_emitted,
    };

    // Pass 1: the original sliding-window decode. In iq_mode the
    // default chain is modem_iq (complex-differential slicer with
    // 2nd-power bias removal) — same chain the live rx_session uses,
    // so "rx_replay on the .iq" reproduces live decodes by default.
    // FrontierSat's modulation is MSK h=0.5 (dev=2400 Hz at 9600 baud)
    // — modem_iq's MSK-tuned arg(z[n]·conj(z[n-sps])) front-end is the
    // right match. Set RX_REPLAY_USE_FSK=1 to switch to modem_fsk's
    // FM-discriminator chain (modulation-index-agnostic; preferred
    // when the bird is not MSK). When two_pass is on, pass-1 always
    // uses the slicer family — the Viterbi runs in pass 2.
    //
    // Skipped entirely when --anchor-csv is set: anchored-decode mode
    // is "decode only the boxed parts", not "boxed parts in addition
    // to a full sweep".
    int pass1_use_viterbi = (iq_mode && two_pass) ? 0 : use_viterbi;
    int pass1_use_fsk = iq_mode && getenv("RX_REPLAY_USE_FSK") != NULL
                        && !pass1_use_viterbi;
    int anchored_only = (anchor_csv_arg != NULL);
    for (size_t window_start = 0;
         window_start + window_samples <= n_frames && !g_stop
         && !anchored_only;
         window_start += slide_samples)
    {
        // PCM windows index by sample; IQ windows index by pair, where
        // each pair occupies 2 int16s in the underlying buffer.
        const int16_t *win = iq_mode
            ? samples + window_start * 2u
            : samples + window_start;
        size_t inner_min_offset = 0;
        for (;;) {
            ssize_t plen = -1;
            int golay_errs = 0, hmac_ok = -1;
            int rs_errs = -1, used_golay_len = -1;
            int rs_locs[32];
            size_t sync_off_local = 0;
            int decoded;
            if (iq_mode && pass1_use_viterbi) {
                decoded = try_decode_window_viterbi(
                    win, window_samples, &mp, &opts,
                    sync_max_ham, use_hmac, allow_partial_rs,
                    inner_min_offset,
                    bits_scratch, bits_cap,
                    bytes_scratch, bytes_cap,
                    packet, sizeof packet,
                    &plen, &golay_errs, &hmac_ok,
                    &rs_errs, &used_golay_len,
                    &sync_off_local, rs_locs);
            } else if (iq_mode && pass1_use_fsk) {
                decoded = try_decode_window_fsk(
                    win, window_samples, &mp, &opts,
                    sync_max_ham, use_hmac, allow_partial_rs,
                    inner_min_offset,
                    bits_scratch, bits_cap,
                    bytes_scratch, bytes_cap,
                    packet, sizeof packet,
                    &plen, &golay_errs, &hmac_ok,
                    &rs_errs, &used_golay_len,
                    &sync_off_local, rs_locs);
            } else if (iq_mode) {
                decoded = try_decode_window_iq(
                    win, window_samples, &mp, &opts,
                    sync_max_ham, use_hmac, allow_partial_rs,
                    inner_min_offset,
                    bits_scratch, bits_cap,
                    bytes_scratch, bytes_cap,
                    packet, sizeof packet,
                    &plen, &golay_errs, &hmac_ok,
                    &rs_errs, &used_golay_len,
                    &sync_off_local, rs_locs);
            } else {
                decoded = try_decode_window(
                    win, window_samples, &mp, &opts,
                    sync_max_ham, use_hmac, allow_partial_rs,
                    inner_min_offset,
                    bits_scratch, bits_cap,
                    bytes_scratch, bytes_cap,
                    packet, sizeof packet,
                    &plen, &golay_errs, &hmac_ok,
                    &rs_errs, &used_golay_len,
                    &sync_off_local, rs_locs);
            }
            if (!decoded) break;
            inner_min_offset = sync_off_local + 1;
            if (plen < 4 || (size_t)plen > sizeof packet) continue;

            uint64_t asm_abs_sample = (uint64_t)window_start
                + (uint64_t)sync_off_local * (uint64_t)sps
                + (uint64_t)(sps / 2);
            raw_decodes++;
            rx_emit_decoded(&ectx, asm_abs_sample, packet, plen,
                            golay_errs, hmac_ok, rs_errs,
                            used_golay_len, rs_locs);
        }
        if (use_tui && rx_tui_tick()) goto done;
    }
    int pass1_n_emitted = n_emitted;

    // Pass 2: anchored Viterbi on candidates the slicer found but
    // pass-1 didn't already emit. Only meaningful in iq_mode with
    // --no-two-pass not set. Tight window so the Viterbi's 4th-power
    // φ_0 estimate is dominated by signal rather than the seconds of
    // silence around each burst.
    int p2_attempts = 0, p2_emitted = 0;
    if (iq_mode && two_pass && !g_stop && !anchored_only) {
        // Tight Viterbi window: one max-length AX100 frame plus
        // pre-ASM cushion for M&M timing-loop settling.
        // Max AX100 payload = ~256 B RS-coded + framing ≈ 2300 bits
        // ≈ 240 ms @ 9600 baud. Pre-ASM cushion = 50 ms.
        const double p2_window_s = 0.40;
        const double p2_pre_anchor_s = 0.05;
        const size_t p2_window_pairs =
            (size_t)(p2_window_s * (double)samp_rate);
        const size_t p2_pre_pairs    =
            (size_t)(p2_pre_anchor_s * (double)samp_rate);

        for (size_t window_start = 0;
             window_start + window_samples <= n_frames && !g_stop;
             window_start += slide_samples)
        {
            const int16_t *win = samples + window_start * 2u;
            size_t inner_min = 0;
            for (int tries = 0; tries < 64; ++tries) {
                size_t n_bits_sym = 0, sync_off_bits = 0;
                int polarity_used = -1;
                int rc = modem_fsk_iq_to_bits(win, window_samples, &mp,
                                          0, sync_max_ham, inner_min,
                                          bits_scratch, &n_bits_sym,
                                          &sync_off_bits, &polarity_used);
                if (rc != 0) break;
                inner_min = sync_off_bits + 1;

                uint64_t asm_abs_sample = (uint64_t)window_start
                    + (uint64_t)sync_off_bits * (uint64_t)sps
                    + (uint64_t)(sps / 2);
                uint64_t pos_quant = asm_abs_sample / DEDUP_QUANT_SAMPLES;
                int seen = 0;
                int ring_n = recent_count < DEDUP_RING_SZ
                    ? recent_count : DEDUP_RING_SZ;
                for (int r = 0; r < ring_n; ++r) {
                    if (recent_pos_quant[r] == pos_quant) { seen = 1; break; }
                }
                if (seen) continue;

                // Anchor a tight window around the candidate ASM.
                uint64_t tight_start = (asm_abs_sample > (uint64_t)p2_pre_pairs)
                    ? (asm_abs_sample - (uint64_t)p2_pre_pairs) : 0;
                if (tight_start + p2_window_pairs > n_frames) {
                    tight_start = (n_frames > p2_window_pairs)
                        ? (n_frames - p2_window_pairs) : 0;
                }
                size_t tw_pairs = (tight_start + p2_window_pairs <= n_frames)
                    ? p2_window_pairs : (n_frames - (size_t)tight_start);
                if (tw_pairs < (size_t)(64 * sps)) continue;
                const int16_t *tw = samples + tight_start * 2u;

                ssize_t plen2 = -1;
                int golay2 = 0, hmac2 = -1, rs2 = -1, glen2 = -1;
                int rs_locs2[32];
                size_t sync_off2 = 0;
                // Pass-2 retry uses the FSK chain on a tight window
                // centered on the sync candidate. The tight window
                // lets AGC + HPF settle on signal-dominated samples
                // rather than mostly-noise — sometimes recovers a
                // frame that pass-1's wide-window FSK partial-RS'd
                // or that pass-1 missed entirely. The Viterbi MLSE
                // is reserved for h=0.5 AWGN tests where its 2 dB
                // coherent gain matters; on FrontierSat's h≈2/3 FSK
                // its 4-state trellis is wrong and finds zero syncs.
                int dec2 = try_decode_window_fsk(
                    tw, tw_pairs, &mp, &opts,
                    sync_max_ham, use_hmac, allow_partial_rs,
                    0,
                    bits_scratch, bits_cap,
                    bytes_scratch, bytes_cap,
                    packet, sizeof packet,
                    &plen2, &golay2, &hmac2,
                    &rs2, &glen2,
                    &sync_off2, rs_locs2);
                ++p2_attempts;
                if (!dec2) continue;
                if (plen2 < 4 || (size_t)plen2 > sizeof packet) continue;

                uint64_t asm_abs_v = tight_start
                    + (uint64_t)sync_off2 * (uint64_t)sps
                    + (uint64_t)(sps / 2);
                raw_decodes++;
                if (rx_emit_decoded(&ectx, asm_abs_v, packet, plen2,
                                    golay2, hmac2, rs2, glen2, rs_locs2)) {
                    ++p2_emitted;
                }
            }
            if (use_tui && rx_tui_tick()) goto done;
        }
    }

    // Anchored decode at external (time, freq) anchors. Reads a CSV in
    // the same format gen_waterfall --show-tm consumes: lines starting
    // with "burst_start," with the optional 6th freq_hz field. For each
    // anchor we mix the IQ to DC at freq_hz over a tight window around
    // the anchor time and run the FSK chain on the mixed buffer. Lets
    // us decode packets that sit far off DC (e.g. the LO-offset
    // baseband signal at +25 kHz) where the sliding-window pass can't
    // reach them through the matched filter's ±10 kHz passband.
    int anc_attempts = 0, anc_emitted = 0;
    if (iq_mode && anchor_csv_arg != NULL && !g_stop) {
        FILE *afp = fopen(anchor_csv_arg, "r");
        if (afp == NULL) {
            fprintf(stderr,
                "rx_replay: --anchor-csv: cannot open %s: %s\n",
                anchor_csv_arg, strerror(errno));
        } else if (!have_start_utc) {
            fprintf(stderr,
                "rx_replay: --anchor-csv requires a UT start time "
                "(parse filename / --start-utc / mtime fallback)\n");
            fclose(afp);
        } else {
            const long long start_unix_ms_ll =
                (long long)(start_utc_seconds * 1000.0 + 0.5);
            const size_t anc_window_pairs =
                (size_t)(anchor_window_s * (double)samp_rate);
            const size_t anc_pre_pairs =
                (size_t)(anchor_pre_s    * (double)samp_rate);
            int16_t *mix_buf = (int16_t *)
                malloc(anc_window_pairs * 2 * sizeof(int16_t));
            if (mix_buf == NULL) {
                fprintf(stderr, "rx_replay: --anchor-csv: oom\n");
                fclose(afp);
            } else {
                char line[512];
                while (fgets(line, sizeof line, afp) != NULL) {
                    if (line[0] == '#' || line[0] == '\n'
                                       || line[0] == '\0'
                                       || line[0] == '\r') continue;
                    if (strncmp(line, "burst_start,", 12) != 0) continue;
                    long long u_ms = 0;
                    int  bins = 0;
                    double db = 0.0, dur = 0.0, fhz = 0.0;
                    int got = sscanf(line + 12,
                                     "%lld,%d,%lf,%lf,%lf",
                                     &u_ms, &bins, &db, &dur, &fhz);
                    if (got < 5) continue;
                    double t_s = (u_ms - start_unix_ms_ll) / 1000.0;
                    if (t_s < 0.0) continue;
                    uint64_t anc_pos = (uint64_t)
                        (t_s * (double)samp_rate + 0.5);
                    if (anc_pos >= n_frames) continue;
                    uint64_t tight_start =
                        (anc_pos > (uint64_t)anc_pre_pairs)
                            ? (anc_pos - (uint64_t)anc_pre_pairs) : 0;
                    if (tight_start + anc_window_pairs > n_frames) {
                        tight_start = (n_frames > anc_window_pairs)
                            ? (n_frames - anc_window_pairs) : 0;
                    }
                    size_t tw_pairs =
                        (tight_start + anc_window_pairs <= n_frames)
                            ? anc_window_pairs
                            : (n_frames - (size_t)tight_start);
                    if (tw_pairs < (size_t)(64 * sps)) continue;

                    // Copy and NCO-mix to DC at fhz. sw_nco_apply
                    // rotates by exp(-j 2π · f · n/fs), so a signal
                    // at +fhz lands at DC.
                    memcpy(mix_buf,
                           samples + tight_start * 2u,
                           tw_pairs * 2 * sizeof(int16_t));
                    sw_nco_t nco;
                    sw_nco_init(&nco, (double)samp_rate);
                    sw_nco_set_freq(&nco, fhz);
                    sw_nco_apply(&nco, mix_buf, tw_pairs);

                    ssize_t plen_a = -1;
                    int golay_a = 0, hmac_a = -1, rs_a = -1, glen_a = -1;
                    int rs_locs_a[32];
                    size_t sync_off_a = 0;
                    int dec_a = try_decode_window_fsk(
                        mix_buf, tw_pairs, &mp, &opts,
                        sync_max_ham, use_hmac, allow_partial_rs,
                        0,
                        bits_scratch, bits_cap,
                        bytes_scratch, bytes_cap,
                        packet, sizeof packet,
                        &plen_a, &golay_a, &hmac_a,
                        &rs_a, &glen_a,
                        &sync_off_a, rs_locs_a);
                    ++anc_attempts;
                    if (!dec_a) continue;
                    if (plen_a < 4 || (size_t)plen_a > sizeof packet)
                        continue;

                    uint64_t asm_abs_a = tight_start
                        + (uint64_t)sync_off_a * (uint64_t)sps
                        + (uint64_t)(sps / 2);
                    raw_decodes++;
                    if (rx_emit_decoded(&ectx, asm_abs_a,
                                        packet, plen_a,
                                        golay_a, hmac_a, rs_a, glen_a,
                                        rs_locs_a)) {
                        ++anc_emitted;
                    }
                    if (use_tui && rx_tui_tick()) {
                        free(mix_buf);
                        fclose(afp);
                        goto done;
                    }
                }
                free(mix_buf);
                fclose(afp);
                fprintf(stderr,
                    "rx_replay: --anchor-csv: %d anchor(s) tried, "
                    "%d emitted from %s\n",
                    anc_attempts, anc_emitted, anchor_csv_arg);
            }
        }
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
    const char *chain;
    if (anchored_only) {
        chain = "anchored-csv (fsk)";
    } else if (!iq_mode) {
        chain = "fm-audio";
    } else if (two_pass) {
        chain = pass1_use_fsk ? "iq+fsk+anchored-fsk"
                              : "iq+slicer+anchored-fsk";
    } else if (pass1_use_viterbi) {
        chain = "iq+viterbi";
    } else if (pass1_use_fsk) {
        chain = "iq+fsk";
    } else {
        chain = "iq+slicer";
    }
    decode_loop_stats_t st;
    decode_loop_get_stats(&st);
    fprintf(stderr, "rx_replay: decode summary (chain=%s):\n", chain);
    fprintf(stderr, "  candidate frames (pre-dedup)   : %ld\n", raw_decodes);
    fprintf(stderr, "  detected (after position dedup): %d  "
            "— all recorded to the DB\n", n_emitted);
    fprintf(stderr, "    valid CSP header             : %ld\n", st.csp_ok);
    fprintf(stderr, "    RS corrected / uncorrectable : %ld / %ld\n",
            st.rs_corrected, st.rs_uncorrectable);
    if (use_hmac) {
        fprintf(stderr, "    HMAC ok / bad                : %ld / %ld\n",
                st.hmac_ok, st.hmac_bad);
    }
    fprintf(stderr, "    recognized / unrecognized    : %ld / %ld\n",
            st.recognized, st.unrecognized);
    fprintf(stderr, "    recognized by type           : "
            "beacon %ld, tcmd_response %ld, log %ld, bulk_file %ld\n",
            st.beacon, st.tcmd_response, st.log_message, st.bulk_file);
    fprintf(stderr, "  the DB keeps one row per distinct payload, "
            "so repeats collapse.\n");
    if (iq_mode && two_pass && !anchored_only) {
        fprintf(stderr,
                "rx_replay: pass-1 emitted %d, pass-2 (anchored FSK) "
                "tried %d candidate(s) and rescued %d.\n",
                pass1_n_emitted, p2_attempts, p2_emitted);
    }
    free(bits_scratch);
    free(bytes_scratch);
    free(samples);
    return 0;
}
