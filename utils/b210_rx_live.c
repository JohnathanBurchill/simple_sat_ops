/*

   Simple Satellite Operations  utils/b210_rx_live.c

   Live B210 RX + AX100 decoder + Doppler retune driver. Captures IQ
   from a USRP B210 via libuhd, FM-demodulates, runs the same sliding-
   window decode loop as rx_live and rx_replay, and (when built with
   SGP4) re-tunes the SDR every ~1 s based on range-rate from the
   tracked TLE.

   Single-process owner of the B210 — the device cannot be shared with
   another UHD program. simple_sat_ops continues to drive the FT-991A
   and rotator; b210_rx_live is the parallel scientific receiver.

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
#include "b210_rx_core.h"
#include "csp.h"
#include "decode_loop.h"
#include "hmac_keyfile.h"
#include "modem.h"
#include "monitor_squelch.h"
#include "radio.h"
#include "rx_tui.h"

#ifdef WITH_SGP4SDP4
#include "prediction.h"
#endif

#ifdef WITH_ALSA
#include <alsa/asoundlib.h>
#endif

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
    rx_tui_request_quit();
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

static double monotonic_seconds(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// Spawn ffmpeg to render a showspectrumpic PNG from a finished WAV.
// PNG path = WAV path with .wav stripped + .png appended. Blocks
// until ffmpeg exits (a multi-minute pass typically renders in a few
// seconds). Same parameters as rx_live's audio_generate_spectrogram
// in audio.c, lifted here so the B210 path doesn't have to link the
// libasound-dependent module just to call execvp.
//
// Note on rate: the WAV is the post-decim FM-demoded PCM (default 48
// kHz), so the spectrogram's y-axis spans 0..24 kHz and 9600-baud GFSK
// bursts occupy most of the visible band. With --no-decim the WAV is
// at the UHD input rate (typ. 240 kHz) and bursts crowd into the
// bottom ~4 % — useful for diagnostics but harder to read.
static int generate_spectrogram(const char *wav_path)
{
    if (wav_path == NULL) return -1;
    size_t len = strlen(wav_path);
    char png_path[512];
    int n;
    if (len >= 4 && strcmp(wav_path + len - 4, ".wav") == 0) {
        n = snprintf(png_path, sizeof png_path, "%.*s.png",
                     (int)(len - 4), wav_path);
    } else {
        n = snprintf(png_path, sizeof png_path, "%s.png", wav_path);
    }
    if (n <= 0 || (size_t)n >= sizeof png_path) return -1;

    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "b210_rx_live: fork() for ffmpeg failed: %s\n",
                strerror(errno));
        return -1;
    }
    if (pid == 0) {
        char *args[] = {
            "ffmpeg",
            "-hide_banner", "-loglevel", "error", "-y",
            "-i", (char *)wav_path,
            "-lavfi",
            "showspectrumpic=s=1920x1080:mode=combined:color=intensity:legend=1",
            png_path,
            NULL,
        };
        execvp("ffmpeg", args);
        fprintf(stderr, "b210_rx_live: ffmpeg not on PATH; "
                "skipping spectrogram for %s\n", wav_path);
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return -1;
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        fprintf(stderr, "b210_rx_live: spectrogram -> %s\n", png_path);
        return 0;
    }
    return -1;
}

// Snapshot N seconds of the master WAV (which is still being appended
// to in the main thread) and render a spectrogram PNG from it. Runs in
// its own pthread so the decode loop keeps ticking. The caller fixes
// start_sample / n_samples at schedule time so the worker only reads
// bytes that were already on disk before the job was created — there's
// no race with the writer.
typedef struct spectrum_job {
    pthread_t       thr;
    int             active;          // 1 when thread has been launched
    volatile int    done;            // worker sets to 1 just before return
    char            wav_in[512];     // master WAV (read-only)
    int             sample_rate;
    long            start_sample;    // first sample to include
    long            n_samples;       // count
    char            png_out[600];
    char            status_msg[1024]; // shown briefly in the TUI status row
} spectrum_job_t;

static void *spectrum_worker(void *arg)
{
    spectrum_job_t *j = (spectrum_job_t *)arg;
    char tmp_wav[700];
    snprintf(tmp_wav, sizeof tmp_wav, "%s.tmp.wav", j->png_out);

    FILE *fin = fopen(j->wav_in, "rb");
    if (fin == NULL) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: open %s failed: %s", j->wav_in, strerror(errno));
        j->done = 1;
        return NULL;
    }
    if (fseek(fin, 44 + j->start_sample * 2, SEEK_SET) != 0) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: seek failed: %s", strerror(errno));
        fclose(fin); j->done = 1; return NULL;
    }
    FILE *fout = fopen(tmp_wav, "wb");
    if (fout == NULL) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: open %s failed: %s", tmp_wav, strerror(errno));
        fclose(fin); j->done = 1; return NULL;
    }
    uint32_t sr   = (uint32_t)j->sample_rate;
    uint32_t bps  = sr * 2;
    uint32_t bcnt = (uint32_t)(j->n_samples * 2);
    uint32_t fsz  = bcnt + 36;
    uint8_t hdr[44] = {
        'R','I','F','F',
        (uint8_t)fsz,(uint8_t)(fsz>>8),(uint8_t)(fsz>>16),(uint8_t)(fsz>>24),
        'W','A','V','E', 'f','m','t',' ', 16,0,0,0, 1,0, 1,0,
        (uint8_t)sr, (uint8_t)(sr>>8), (uint8_t)(sr>>16), (uint8_t)(sr>>24),
        (uint8_t)bps,(uint8_t)(bps>>8),(uint8_t)(bps>>16),(uint8_t)(bps>>24),
        2,0, 16,0,
        'd','a','t','a',
        (uint8_t)bcnt,(uint8_t)(bcnt>>8),(uint8_t)(bcnt>>16),(uint8_t)(bcnt>>24),
    };
    if (fwrite(hdr, 1, 44, fout) != 44) {
        snprintf(j->status_msg, sizeof j->status_msg, "spectrum: header write failed");
        fclose(fin); fclose(fout); unlink(tmp_wav); j->done = 1; return NULL;
    }
    int16_t buf[4096];
    long remaining = j->n_samples;
    while (remaining > 0) {
        size_t want = remaining > (long)(sizeof buf / sizeof buf[0])
                    ? sizeof buf / sizeof buf[0] : (size_t)remaining;
        size_t got = fread(buf, sizeof buf[0], want, fin);
        if (got == 0) break;
        fwrite(buf, sizeof buf[0], got, fout);
        remaining -= (long)got;
    }
    fclose(fin); fclose(fout);

    pid_t pid = fork();
    int rc = -1;
    if (pid == 0) {
        char *args[] = {
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-i", tmp_wav,
            "-lavfi",
            "showspectrumpic=s=1920x1080:mode=combined:color=intensity:legend=1",
            j->png_out, NULL,
        };
        execvp("ffmpeg", args);
        _exit(127);
    } else if (pid > 0) {
        int status;
        if (waitpid(pid, &status, 0) >= 0 && WIFEXITED(status))
            rc = WEXITSTATUS(status);
    }
    unlink(tmp_wav);

    if (rc == 0) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: wrote %s (%.1fs)",
                 j->png_out, (double)j->n_samples / (double)j->sample_rate);
    } else {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: ffmpeg failed (rc=%d) for %s", rc, j->png_out);
    }
    j->done = 1;
    return NULL;
}

// Best-effort mkdir -p $HOME/.local/state/simple_sat_ops. Errors are
// silently ignored — callers downstream will get a clean fopen error
// if the directory really can't be created.
static void ensure_state_dir(const char *home)
{
    char path[512];
    snprintf(path, sizeof path, "%s/.local",                       home); (void)mkdir(path, 0755);
    snprintf(path, sizeof path, "%s/.local/state",                 home); (void)mkdir(path, 0755);
    snprintf(path, sizeof path, "%s/.local/state/simple_sat_ops",  home); (void)mkdir(path, 0700);
}

// Tiny streaming WAV writer (header at open, samples appended, sizes
// patched at close). Independent of audio.c so this binary doesn't pull
// in ALSA. Mono S16_LE only — that's all the demod produces. Forward-
// declared here so cmd_handler can read .n_samples for `spectrum N`.
typedef struct {
    FILE  *fp;
    size_t n_samples;
    int    sample_rate;
} wav_w_t;

// Carrier-presence squelch (see monitor_squelch.h). Old FM-noise
// gate replaced — the new logic gates on the same data-band /
// noise-band ratio that beacon_detect uses post-pass.

// Format the current post-FIR IQ level into " peak=±X.XdBFS rms=±X.XdBFS".
// `peak` is the fast-attack / slow-release envelope kept inside the core
// — rises instantly on incoming RF, decays at ~0.5 s. `rms` is the 30 ms-
// smoothed magnitude. Both are referenced to sc16 full-scale (±32767).
// The B210 has no internal RSSI calibration, so this is dBFS not dBm —
// to read it as dBm, subtract --gain-db plus the LNA gain in the chain.
static void iq_level_format(const b210_rx_core_t *core,
                            char *buf, size_t cap)
{
    double peak = 0.0, rms_sq = 0.0;
    if (core == NULL
        || b210_rx_core_iq_levels(core, &peak, &rms_sq) != 0) {
        snprintf(buf, cap, "n/a");
        return;
    }
    double rms = sqrt(rms_sq);
    double peak_dbfs = (peak < 1.0) ? -90.0 : 20.0 * log10(peak / 32768.0);
    double rms_dbfs  = (rms  < 1.0) ? -90.0 : 20.0 * log10(rms  / 32768.0);
    snprintf(buf, cap, "peak=%+5.1f rms=%+5.1f dBFS", peak_dbfs, rms_dbfs);
}

// REPL handler context. The TUI calls cmd_handler() inside its tick,
// which runs synchronously in the main thread between decode-window
// passes — so mutating opts.reed_solomon directly is safe.
typedef struct b210_cmd_ctx {
    ax100_opts_t      *opts;
    int               *force_beacon;        // toggleable mid-run via REPL
    int               *want_status_refresh; // bumped to force status redraw
    // Live-spectrum REPL command: snapshots the master WAV (assumed
    // open in the main thread), spawns a worker to render a PNG. Single
    // job slot — the operator can only run one render at a time.
    wav_w_t           *wav;
    const char        *wav_path;            // master WAV path (NULL if --no-wav)
    spectrum_job_t    *spec_job;
    monitor_squelch_t *sq;                  // NULL if --monitor wasn't requested
} b210_cmd_ctx_t;

// Trim leading/trailing ASCII whitespace in place. Returns the new
// length so the caller can decide on empty-after-trim handling.
static size_t str_trim(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
    size_t i = 0;
    while (s[i] == ' ' || s[i] == '\t') i++;
    if (i > 0) memmove(s, s + i, n - i + 1);
    return n - i;
}

// g_stop is declared static at the top of this file; the cmd_handler
// is also file-static so it sees the same symbol.
static void cmd_handler(const char *cmd, void *ctx_v)
{
    b210_cmd_ctx_t *ctx = (b210_cmd_ctx_t *)ctx_v;
    char buf[256];
    snprintf(buf, sizeof buf, "%s", cmd);
    str_trim(buf);
    if (buf[0] == '\0') return;

    char ph_status[128];
    if (decode_loop_try_command(buf, ph_status, sizeof ph_status)) {
        rx_tui_set_status(ph_status);
        if (ctx->want_status_refresh) *ctx->want_status_refresh = 1;
        return;
    }

    if (strcmp(buf, "rs on") == 0) {
        ctx->opts->reed_solomon = 1;
        if (ctx->want_status_refresh) *ctx->want_status_refresh = 1;
    } else if (strcmp(buf, "rs off") == 0) {
        ctx->opts->reed_solomon = 0;
        if (ctx->want_status_refresh) *ctx->want_status_refresh = 1;
    } else if (strcmp(buf, "force-beacon on") == 0
               || strcmp(buf, "fb on") == 0) {
        if (ctx->force_beacon) *ctx->force_beacon = 1;
        if (ctx->want_status_refresh) *ctx->want_status_refresh = 1;
    } else if (strcmp(buf, "force-beacon off") == 0
               || strcmp(buf, "fb off") == 0) {
        if (ctx->force_beacon) *ctx->force_beacon = 0;
        if (ctx->want_status_refresh) *ctx->want_status_refresh = 1;
    } else if (strncmp(buf, "sq", 2) == 0
               && (buf[2] == ' ' || buf[2] == '\t')) {
        // sq auto | sq off | sq <number>
        if (ctx->sq == NULL) {
            rx_tui_set_status("sq: --monitor not enabled this run");
            return;
        }
        const char *arg = buf + 2;
        while (*arg == ' ' || *arg == '\t') arg++;
        if (strcmp(arg, "auto") == 0) {
            monitor_squelch_set_auto(ctx->sq);
            if (ctx->want_status_refresh) *ctx->want_status_refresh = 1;
            rx_tui_set_status("sq: auto — re-bootstrapping noise floor");
        } else if (strcmp(arg, "off") == 0) {
            monitor_squelch_set_off(ctx->sq);
            if (ctx->want_status_refresh) *ctx->want_status_refresh = 1;
            rx_tui_set_status("sq: off (passing all audio)");
        } else {
            char *endp = NULL;
            double th = strtod(arg, &endp);
            if (endp == arg) {
                rx_tui_set_status("sq: usage `sq auto | sq off | sq <dB>`");
                return;
            }
            monitor_squelch_set_fixed_db(ctx->sq, th);
            if (ctx->want_status_refresh) *ctx->want_status_refresh = 1;
            char m[128];
            snprintf(m, sizeof m, "sq: fixed thr=%+.1f dB (sig/noise ratio)", th);
            rx_tui_set_status(m);
        }
    } else if (strncmp(buf, "spectrum", 8) == 0
               || strncmp(buf, "spec",     4) == 0) {
        const char *arg = buf;
        if (strncmp(arg, "spectrum", 8) == 0) arg += 8;
        else                                  arg += 4;
        while (*arg == ' ' || *arg == '\t') arg++;
        char *endp = NULL;
        double duration_s = strtod(arg, &endp);
        if (endp == arg || duration_s <= 0.0) {
            rx_tui_set_status("spectrum: usage `spectrum <seconds>` (1..600)");
            return;
        }
        if (duration_s > 600.0) duration_s = 600.0;
        if (duration_s < 1.0)   duration_s = 1.0;
        if (ctx->wav == NULL || ctx->wav_path == NULL) {
            rx_tui_set_status("spectrum: WAV not enabled (run without --no-wav)");
            return;
        }
        if (ctx->spec_job == NULL) {
            rx_tui_set_status("spectrum: no job slot (internal error)");
            return;
        }
        if (ctx->spec_job->active && !ctx->spec_job->done) {
            rx_tui_set_status("spectrum: a render is already in progress");
            return;
        }
        // Reap the previous job if the main loop hasn't gotten to it yet.
        if (ctx->spec_job->active && ctx->spec_job->done) {
            pthread_join(ctx->spec_job->thr, NULL);
            ctx->spec_job->active = 0;
        }
        // Flush stdio so bytes the worker is about to read are on disk.
        if (ctx->wav->fp) fflush(ctx->wav->fp);

        long total       = (long)ctx->wav->n_samples;
        long want        = (long)(duration_s * (double)ctx->wav->sample_rate);
        long start       = total - want;
        if (start < 0) { start = 0; want = total; }
        if (want <= 0) {
            rx_tui_set_status("spectrum: no samples captured yet");
            return;
        }

        // Filename: strip the .wav extension and append a local-time range.
        char base[600];
        size_t plen = strlen(ctx->wav_path);
        if (plen >= 4 && strcmp(ctx->wav_path + plen - 4, ".wav") == 0) {
            snprintf(base, sizeof base, "%.*s",
                     (int)(plen - 4), ctx->wav_path);
        } else {
            snprintf(base, sizeof base, "%s", ctx->wav_path);
        }
        struct timeval tv_end; gettimeofday(&tv_end, NULL);
        time_t t_end   = tv_end.tv_sec;
        time_t t_start = t_end - (time_t)llround(duration_s);
        struct tm lt_start, lt_end;
        localtime_r(&t_start, &lt_start);
        localtime_r(&t_end,   &lt_end);
        char ts_start[32], ts_end[32];
        strftime(ts_start, sizeof ts_start, "%Y-%m-%d_%H-%M-%S", &lt_start);
        strftime(ts_end,   sizeof ts_end,   "%H-%M-%S",          &lt_end);

        spectrum_job_t *j = ctx->spec_job;
        memset(j, 0, sizeof *j);
        snprintf(j->wav_in,  sizeof j->wav_in,  "%s", ctx->wav_path);
        // Cap the base path's contribution so -Wformat-truncation can
        // see the output fits png_out[600]. 500 + ts/literals (~50)
        // leaves comfortable headroom and is way more than any sane
        // auto-named WAV path will be.
        snprintf(j->png_out, sizeof j->png_out,
                 "%.500s_LOCAL_%s_to_%s.png", base, ts_start, ts_end);
        j->sample_rate  = ctx->wav->sample_rate;
        j->start_sample = start;
        j->n_samples    = want;

        if (pthread_create(&j->thr, NULL, spectrum_worker, j) != 0) {
            snprintf(j->status_msg, sizeof j->status_msg,
                     "spectrum: pthread_create failed: %s", strerror(errno));
            rx_tui_set_status(j->status_msg);
            return;
        }
        j->active = 1;
        char msg[1024];
        snprintf(msg, sizeof msg,
                 "spectrum: rendering %.1fs (%s..%s) → %s",
                 (double)want / (double)j->sample_rate,
                 ts_start, ts_end, j->png_out);
        rx_tui_set_status(msg);
    } else if (strcmp(buf, "q") == 0 || strcmp(buf, "quit") == 0
               || strcmp(buf, "exit") == 0) {
        g_stop = 1;
        rx_tui_request_quit();
    }
    // Unknown commands silently no-op — the TUI owns stdout/stderr.
    // A `help` command that pokes at a help panel is a future add.
}

// wav_w_t is defined further up so cmd_handler can read it. The open/
// append/close trio implements the streaming write side.
static int wav_w_open(wav_w_t *w, const char *path, int sample_rate)
{
    memset(w, 0, sizeof *w);
    w->fp = fopen(path, "wb");
    if (w->fp == NULL) return -1;
    uint32_t sr   = (uint32_t)sample_rate;
    uint32_t bps  = sr * 2;            // bytes/sec = rate * mono * 2 bytes
    uint8_t hdr[44] = {
        'R','I','F','F', 0,0,0,0, 'W','A','V','E',
        'f','m','t',' ', 16,0,0,0, 1,0, 1,0,
        (uint8_t)(sr      & 0xFF), (uint8_t)((sr >> 8)  & 0xFF),
        (uint8_t)((sr>>16)& 0xFF), (uint8_t)((sr >> 24) & 0xFF),
        (uint8_t)(bps     & 0xFF), (uint8_t)((bps >> 8) & 0xFF),
        (uint8_t)((bps>>16)& 0xFF),(uint8_t)((bps >> 24) & 0xFF),
        2,0, 16,0,
        'd','a','t','a', 0,0,0,0,
    };
    if (fwrite(hdr, 1, 44, w->fp) != 44) { fclose(w->fp); w->fp = NULL; return -1; }
    w->sample_rate = sample_rate;
    return 0;
}

static void wav_w_append(wav_w_t *w, const int16_t *s, size_t n)
{
    if (w->fp == NULL || n == 0) return;
    if (fwrite(s, sizeof(int16_t), n, w->fp) == n) {
        w->n_samples += n;
    }
}

static void wav_w_close(wav_w_t *w)
{
    if (w->fp == NULL) return;
    uint32_t data_sz = (uint32_t)(w->n_samples * 2u);
    uint32_t riff_sz = data_sz + 36u;
    if (fseek(w->fp, 4, SEEK_SET) == 0)  (void)fwrite(&riff_sz, 4, 1, w->fp);
    if (fseek(w->fp, 40, SEEK_SET) == 0) (void)fwrite(&data_sz, 4, 1, w->fp);
    fclose(w->fp);
    w->fp = NULL;
}

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static void usage(FILE *dest, const char *name)
{
    fprintf(dest,
        "usage: %s [<tle_path> <satellite_name>] [options]\n"
        "\n"
        "Live AX100 decoder over a USRP B210, with optional Doppler retuning\n"
        "from a TLE-tracked satellite. Single-process owner of the B210.\n"
        "Sibling of rx_live (FT-991A audio path) and b210_rx_capture\n"
        "(record-only).\n"
        "\n"
        "When the positional <tle_path> <satellite_name> are given (and the\n"
        "binary is built with SGP4SDP4), the tool re-tunes the B210 every\n"
        "~1 s as range-rate moves the carrier. Without them, or with\n"
        "--no-doppler, the freq stays at --nominal-freq for the run.\n"
        "\n"
        "Tracking:\n"
        "  --lat=<deg>                 Observer latitude (default RAO %.4f)\n"
        "  --lon=<deg>                 Observer longitude (default %.4f)\n"
        "  --alt=<m>                   Observer altitude (default %.0f m)\n"
        "  --nominal-freq=<hz>         Rest-frame carrier (default %.0f)\n"
        "  --doppler-threshold-hz=<hz> Min freq delta to trigger retune (default 200)\n"
        "  --no-doppler                Freeze freq at --nominal-freq, skip prediction\n"
        "\n"
        "Capture (B210):\n"
        "  --rate=<sps>                UHD input sample rate (default 240000)\n"
        "  --gain-db=<n>               RX gain dB (default 50)\n"
        "  --bw-hz=<hz>                Analog filter bw (default = rate)\n"
        "  --rx-antenna=<name>         Antenna port (default \"RX2\" = RF A RX;\n"
        "                              alternative: \"TX/RX\" = RF A TX/RX jack)\n"
        "  --device-args=<str>         UHD device args (default \"type=b200\")\n"
        "  --fm-fullscale-hz=<hz>      PCM scale: ±this Hz → ±32767 (default 25000)\n"
        "\n"
        "IQ decimator (narrows IQ before the FM discriminator — without\n"
        "this stage a 9600-baud GMSK carrier is buried in 230+ kHz of\n"
        "out-of-band noise and never decodes):\n"
        "  --decim-factor=<int>        Integer decimation factor (default 5;\n"
        "                              240 kHz → 48 kHz, 5 sps at 9600 baud,\n"
        "                              matching rx_live's audio path)\n"
        "  --no-decim                  Disable decimation (==--decim-factor=1).\n"
        "                              Useful for diagnostics; not for decode.\n"
        "  --decim-cutoff-hz=<hz>      FIR -6 dB cutoff at the input rate\n"
        "                              (default 18000)\n"
        "  --decim-taps=<int>          FIR tap count (default 96; transition\n"
        "                              ~3.3 * input_rate / taps wide)\n"
        "\n"
        "Decoder:\n"
        "  --bit-rate=<bps>            Default 9600\n"
        "  --window-s=<seconds>        Decoder window (default 1.5; min 0.5, max 30)\n"
        "  --slide-s=<seconds>         Slide between attempts (default 0.5)\n"
        "  --sync-threshold=<0..8>     Max ASM bit errors (default 4)\n"
        "  --hmac / --no-hmac          Verify HMAC (default off — downlink frames\n"
        "                              don't carry one)\n"
        "  --keyfile=<path>            HMAC keyfile (default $HOME/%s)\n"
        "  --reed-solomon              RS(255,223) decode (DEFAULT)\n"
        "  --no-reed-solomon           Skip RS decode\n"
        "  --partial-rs                Emit RS-uncorrectable bytes for inspection\n"
        "                              (default; same as rx_live)\n"
        "  --dc-block / --no-dc-block  α=0.995 IIR HP on the FM-demoded PCM\n"
        "                              (~40 Hz cutoff at 48 kHz). DEFAULT ON,\n"
        "                              matching rx_live. Disable only if you\n"
        "                              know the discriminator output has no\n"
        "                              DC offset and you want to skip the IIR\n"
        "                              startup transient.\n"
        "  --csp-crc32                 Validate + strip a trailing CSP zlib CRC32\n"
        "  --force-beacon              Pad each decoded payload with zeros up\n"
        "                              to 130 bytes and print as a beacon\n"
        "                              regardless of length or dispatch result.\n"
        "                              Same flag as rx_replay/rx_decode. Toggle\n"
        "                              live in --ui mode with `fb on`/`fb off`.\n"
        "  --packet-headers            Show the AX100 framing line, CSP v1\n"
        "                              header line, and per-frame hex/ascii\n"
        "                              dumps. Default off — only the\n"
        "                              interpreted body and error conditions\n"
        "                              print. Toggle live with\n"
        "                              `packetheaders on|off` (or `ph on|off`).\n"
        "  --no-packet-headers         Default; kept as a no-op for scripts.\n"
        "\n"
        "Live monitor (Linux only — requires this binary be built with ALSA):\n"
        "  --monitor                  Pipe the FM-demoded audio to ALSA so\n"
        "                             you hear it in real time. Decode and\n"
        "                             WAV write are upstream and never gated.\n"
        "  --monitor-device=<name>    ALSA device for --monitor (default\n"
        "                             \"default\")\n"
        "  --monitor-squelch=<a|off|N>  Squelch mode for the monitor audio.\n"
        "                             Carrier-presence gate: opens when\n"
        "                             the (4500–5100 Hz)/(8000–22000 Hz)\n"
        "                             power ratio crosses threshold (same\n"
        "                             detector beacon_detect uses).\n"
        "                             auto = bootstrap mean ratio from\n"
        "                                    first ~1 s, then open when\n"
        "                                    ratio > mean + offset (DEFAULT)\n"
        "                             off  = pass everything through\n"
        "                             N    = fixed dB threshold on the\n"
        "                                    sig/noise ratio (negative is\n"
        "                                    fine; typical bootstrap mean\n"
        "                                    is around −9 dB on noise-only)\n"
        "  --monitor-auto-offset-db=<dB>  Threshold offset above bootstrap\n"
        "                             mean for auto mode (default 3.0)\n"
        "\n"
        "Output:\n"
        "  --log=<path>                Append decoded-frame summaries (default:\n"
        "                              auto-name b210_rx_live_UT=...log)\n"
        "  --no-log                    Skip the decode log\n"
        "  --wav=<path>                Tee demoded PCM to a WAV. Default: ON,\n"
        "                              auto-named b210_rx_live_UT=...wav in\n"
        "                              CWD so every run leaves a replayable\n"
        "                              breadcrumb. Header rate = post-decim\n"
        "                              rate (~5.7 MB/min at the default 48 kHz).\n"
        "  --no-wav                    Skip the WAV.\n"
        "  --csv=<path>                Per-second CSV with timestamp,\n"
        "                              freq, Doppler offset, range_rate,\n"
        "                              azimuth, elevation, IQ peak/RMS\n"
        "                              dBFS, frame counts. Default: ON,\n"
        "                              auto-named alongside the WAV. For\n"
        "                              post-pass triage in a spreadsheet\n"
        "                              without re-running Python on the\n"
        "                              WAV. Az/el blank without --doppler.\n"
        "  --no-csv                    Skip the CSV.\n"
        "  --no-spectrogram            Skip the post-run spectrogram PNG\n"
        "                              (rendered from the WAV via\n"
        "                              ffmpeg showspectrumpic at exit).\n"
        "  --quiet                     Drop stdout (log only)\n"
        "  --ui                        ncurses panel TUI (forces --quiet).\n"
        "                              ON BY DEFAULT — pass --no-ui to drop\n"
        "                              back to streaming stderr lines.\n"
        "  --no-ui                     Disable the ncurses TUI; print\n"
        "                              retunes and decoded frames to stderr\n"
        "                              instead.\n"
        "  --duration-s=<f>            Stop after N seconds (default: run until\n"
        "                              Ctrl-C or q in --ui mode)\n"
        "  --help                      This message.\n"
        "\n"
        "REPL (--ui mode, type into the bottom input row):\n"
        "  rs on | rs off              Toggle Reed-Solomon decode mid-pass\n"
        "  fb on | fb off              Toggle force-beacon mid-pass\n"
        "  packetheaders on | off      Show/hide AX100, CSP, hex, ascii lines\n"
        "  ph on | ph off              Shorthand for packetheaders\n"
        "  sq auto                     Re-bootstrap the monitor squelch from\n"
        "                              the next ~1 s of audio\n"
        "  sq off                      Disable squelch (pass all monitor audio)\n"
        "  sq <dB>                     Set a fixed squelch threshold on\n"
        "                              the sig/noise ratio (e.g. `sq -5`)\n"
        "  spectrum <seconds>          Render a spectrogram PNG of the last N\n"
        "                              seconds of WAV samples in a worker\n"
        "                              thread (1..600 s; needs ffmpeg). PNG\n"
        "                              filename includes the local-time range.\n"
        "  q | quit | exit             Exit cleanly (Ctrl-C also works)\n",
        name,
#ifdef WITH_SGP4SDP4
        (double)RAO_LATITUDE, (double)RAO_LONGITUDE, (double)RAO_ALTITUDE,
#else
        0.0, 0.0, 0.0,
#endif
        FRONTIERSAT_CARRIER_HZ,
        HMAC_KEYFILE_DEFAULT_RELPATH);
}

// load_tle in prediction.c handles the file-scan + name-prefix-match +
// SGP4 conversion already; we just plug in the name and path. It does
// case-sensitive prefix matching, same convention as simple_sat_ops.

int main(int argc, char **argv)
{
    // Defaults.
    const char *tle_path     = NULL;
    const char *sat_name     = NULL;
    double obs_lat_deg       =
#ifdef WITH_SGP4SDP4
        RAO_LATITUDE;
#else
        0.0;
#endif
    double obs_lon_deg       =
#ifdef WITH_SGP4SDP4
        RAO_LONGITUDE;
#else
        0.0;
#endif
    double obs_alt_m         =
#ifdef WITH_SGP4SDP4
        RAO_ALTITUDE;
#else
        0.0;
#endif
    double nominal_freq_hz   = FRONTIERSAT_CARRIER_HZ;
    double doppler_thr_hz    = 200.0;
    int    no_doppler        = 0;

    double rate_hz           = 240000.0;
    double gain_db           = 50.0;
    double bw_hz             = -1.0;
    const char *rx_antenna   = "RX2";
    const char *device_args  = "type=b200";
    double fm_fullscale_hz   = 25000.0;

    // IQ decimator (digital narrowband filter inside b210_rx_core).
    // Default: 240 kHz UHD → 48 kHz post-decim → 5 sps at 9600 baud,
    // matching rx_live's audio path. Without this stage the FM
    // discriminator runs on raw 240 kHz IQ and drowns a ~10 kHz-wide
    // GMSK signal in 230+ kHz of out-of-band noise.
    unsigned decim_factor    = 5u;
    double   decim_cutoff_hz = 18000.0;
    unsigned decim_taps      = 96u;

    // Live ALSA monitor + FM-noise squelch (Linux only; --monitor is a
    // no-op on Mac builds). The squelch operates on the FM-demoded PCM
    // stream — a high-pass extracts noise above the audio band, running
    // Carrier-presence squelch on the audio sent to ALSA. Same
    // detector as beacon_detect: ratio of in-band (default 4500–5100
    // Hz, the GMSK preamble tone at half the bit rate) to out-of-band
    // noise (default 8000 Hz to fs/2). Gate OPENS for ~0.5 s every
    // time the dB ratio crosses threshold. Decode and WAV write are
    // never gated; only the audio sent to the speaker.
    //
    // Squelch modes (selectable with --monitor-squelch=, toggleable
    // mid-run via the REPL `sq auto|off|<dB>`):
    //   auto    bootstrap mean ratio from first ~1 s, then open when
    //           ratio > mean + auto_offset_db (default +3 dB)
    //   off     pass everything through
    //   <dB>    fixed: open when ratio > <dB>
    int         want_monitor      = 0;
    const char *monitor_device    = "default";
    msq_mode_t  monitor_sq_init   = MSQ_AUTO_BOOTSTRAPPING;
    double      monitor_sq_user_db = 0.0;     // for MSQ_FIXED
    double      monitor_auto_offset_db = 3.0;

    int    bit_rate          = 9600;
    double window_s          = 1.5;
    double slide_s           = 0.5;
    int    sync_max_ham      = 4;
    int    use_hmac          = 0;
    int    use_rs            = 1;
    int    no_dc_block       = 0;     // matches rx_live's default (DC block ON)
    int    csp_crc32         = 0;
    int    force_beacon      = 0;
    int    show_packet_headers = 0;
    const char *keyfile_path = NULL;

    const char *log_path     = NULL;
    int         want_log     = 1;
    char        auto_log_path[300] = {0};
    const char *wav_path     = NULL;
    int         want_wav     = 1;
    char        auto_wav_path[300] = {0};
    const char *csv_path     = NULL;
    int         want_csv     = 1;
    char        auto_csv_path[300] = {0};
    int         want_spectrogram = 1;
    int         quiet        = 0;
    // ncurses TUI is the default — the operator wants to see the live
    // Doppler-tracked frequency in a stable on-screen location during a
    // pass. Pass --no-ui to drop back to plain stderr lines (for piping
    // / scripted runs). use_tui_explicit lets us tell the operator
    // unambiguously when a build without ncurses falls back to text.
    int         use_tui          = 1;
    int         use_tui_explicit = 0;
    double      duration_s   = 0.0;   // 0 = run until signal/q

    // Parse positional args first (TLE path + satellite name). If the
    // first non-flag arg looks like a path, treat it as the TLE; the
    // next non-flag is the satellite name. Both are optional; without
    // them we run --no-doppler implicitly.
    int posn = 0;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        if (a[0] == '-' && a[1] == '-') continue;
        if (posn == 0) tle_path = a;
        else if (posn == 1) sat_name = a;
        else {
            fprintf(stderr, "b210_rx_live: unexpected positional arg '%s'\n", a);
            usage(stderr, argv[0]);
            return EXIT_FAILURE;
        }
        posn++;
    }
    if (tle_path != NULL && sat_name == NULL) {
        // Default the satellite name to FrontierSat — the canonical
        // target for this tool. Operator can override by passing it as
        // a second positional arg.
        sat_name = "FrontierSat";
        fprintf(stderr, "b210_rx_live: defaulting satellite name to '%s'\n",
                sat_name);
    }

    // Flag args.
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (a[0] != '-' || a[1] != '-') continue;  // positional (already handled)
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }

        else if (starts_with(a, "--lat="))                  obs_lat_deg = atof(a + 6);
        else if (starts_with(a, "--lon="))                  obs_lon_deg = atof(a + 6);
        else if (starts_with(a, "--alt="))                  obs_alt_m   = atof(a + 6);
        else if (starts_with(a, "--nominal-freq="))         nominal_freq_hz = atof(a + 15);
        else if (starts_with(a, "--doppler-threshold-hz=")) doppler_thr_hz  = atof(a + 23);
        else if (strcmp(a, "--no-doppler") == 0)            no_doppler = 1;

        else if (starts_with(a, "--rate="))                 rate_hz   = atof(a + 7);
        else if (starts_with(a, "--gain-db="))              gain_db   = atof(a + 10);
        else if (starts_with(a, "--bw-hz="))                bw_hz     = atof(a + 8);
        else if (starts_with(a, "--rx-antenna="))           rx_antenna = a + 13;
        else if (starts_with(a, "--device-args="))          device_args = a + 14;
        else if (starts_with(a, "--fm-fullscale-hz="))      fm_fullscale_hz = atof(a + 18);

        else if (starts_with(a, "--decim-factor=")) {
            int v = atoi(a + 15);
            if (v < 1) v = 1;
            decim_factor = (unsigned)v;
        }
        else if (strcmp(a, "--no-decim") == 0)              decim_factor = 1u;
        else if (starts_with(a, "--decim-cutoff-hz="))      decim_cutoff_hz = atof(a + 18);
        else if (starts_with(a, "--decim-taps=")) {
            int v = atoi(a + 13);
            if (v < 8) v = 8;
            decim_taps = (unsigned)v;
        }

        else if (strcmp(a, "--monitor") == 0)               want_monitor = 1;
        else if (starts_with(a, "--monitor-device="))       monitor_device = a + 17;
        else if (starts_with(a, "--monitor-squelch=")) {
            const char *v = a + 18;
            if (strcmp(v, "auto") == 0)      monitor_sq_init = MSQ_AUTO_BOOTSTRAPPING;
            else if (strcmp(v, "off") == 0)  monitor_sq_init = MSQ_OFF;
            else {
                char *endp = NULL;
                double th = strtod(v, &endp);
                if (endp == v) {
                    fprintf(stderr, "b210_rx_live: --monitor-squelch must be "
                                    "'auto', 'off', or a number (sig/noise dB) "
                                    "(got '%s')\n", v);
                    return EXIT_FAILURE;
                }
                monitor_sq_init    = MSQ_FIXED;
                monitor_sq_user_db = th;
            }
        }
        else if (starts_with(a, "--monitor-auto-offset-db=")) {
            monitor_auto_offset_db = atof(a + 25);
        }

        else if (starts_with(a, "--bit-rate="))             bit_rate = atoi(a + 11);
        else if (starts_with(a, "--window-s=")) {
            window_s = atof(a + 11);
            if (window_s < 0.5)  window_s = 0.5;
            if (window_s > 30.0) window_s = 30.0;
        }
        else if (starts_with(a, "--slide-s=")) {
            slide_s = atof(a + 10);
            if (slide_s < 0.05) slide_s = 0.05;
        }
        else if (starts_with(a, "--sync-threshold=")) {
            sync_max_ham = atoi(a + 17);
            if (sync_max_ham < 0 || sync_max_ham > 8) {
                fprintf(stderr, "b210_rx_live: --sync-threshold out of range [0,8]\n");
                return EXIT_FAILURE;
            }
        }
        else if (strcmp(a, "--hmac") == 0)                  use_hmac = 1;
        else if (strcmp(a, "--no-hmac") == 0)               use_hmac = 0;
        else if (strcmp(a, "--reed-solomon") == 0)          use_rs = 1;
        else if (strcmp(a, "--no-reed-solomon") == 0)       use_rs = 0;
        else if (strcmp(a, "--partial-rs") == 0)            { /* default ON below */ }
        else if (strcmp(a, "--dc-block") == 0)              no_dc_block = 0;
        else if (strcmp(a, "--no-dc-block") == 0)           no_dc_block = 1;
        else if (strcmp(a, "--csp-crc32") == 0)             csp_crc32 = 1;
        else if (strcmp(a, "--force-beacon") == 0)          force_beacon = 1;
        else if (strcmp(a, "--packet-headers") == 0)        show_packet_headers = 1;
        else if (strcmp(a, "--no-packet-headers") == 0)     show_packet_headers = 0;
        else if (starts_with(a, "--keyfile="))              keyfile_path = a + 10;

        else if (starts_with(a, "--log="))                  { log_path = a + 6; want_log = 1; }
        else if (strcmp(a, "--no-log") == 0)                want_log = 0;
        else if (starts_with(a, "--wav="))                  { wav_path = a + 6; want_wav = 1; }
        else if (strcmp(a, "--no-wav") == 0)                want_wav = 0;
        else if (starts_with(a, "--csv="))                  { csv_path = a + 6; want_csv = 1; }
        else if (strcmp(a, "--no-csv") == 0)                want_csv = 0;
        else if (strcmp(a, "--no-spectrogram") == 0)        want_spectrogram = 0;
        else if (strcmp(a, "--quiet") == 0)                 quiet = 1;
        else if (strcmp(a, "--ui") == 0)                  { use_tui = 1; use_tui_explicit = 1; }
        else if (strcmp(a, "--no-ui") == 0)                 use_tui = 0;
        else if (starts_with(a, "--duration-s="))           duration_s = atof(a + 13);

        else {
            fprintf(stderr, "b210_rx_live: unknown option '%s'\n", a);
            usage(stderr, argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (slide_s > window_s) slide_s = window_s;
    if (bit_rate <= 0) {
        fprintf(stderr, "b210_rx_live: --bit-rate must be > 0\n");
        return EXIT_FAILURE;
    }

#ifndef WITH_SGP4SDP4
    if (tle_path != NULL && !no_doppler) {
        fprintf(stderr, "b210_rx_live: this build has no SGP4SDP4 — Doppler\n"
                        "tracking is unavailable. Re-run with --no-doppler,\n"
                        "or rebuild on a host with libsgp4sdp4 installed.\n");
        return EXIT_FAILURE;
    }
#endif

    int do_doppler = (tle_path != NULL && sat_name != NULL && !no_doppler);

#ifdef WITH_SGP4SDP4
    prediction_t pred;
    if (do_doppler) {
        memset(&pred, 0, sizeof pred);
        pred.observer_ephem.position_geodetic.lat = obs_lat_deg * (M_PI / 180.0);
        pred.observer_ephem.position_geodetic.lon = obs_lon_deg * (M_PI / 180.0);
        pred.observer_ephem.position_geodetic.alt = obs_alt_m / 1000.0;
        pred.tles_filename = (char *)tle_path;
        pred.satellite_ephem.name = (char *)sat_name;
        if (load_tle(&pred) != 0) {
            return EXIT_FAILURE;
        }
        // SGP4 internal state init: clear all bookkeeping flags, then
        // let the library pick SGP4 vs SDP4 from the TLE's mean motion.
        // simple_sat_ops's main.c does the same right after load_tle.
        // Without this the propagator reads stale flag state and
        // range_rate comes back as 0, which is why a freshly-built
        // b210_rx_live would tune to nominal and never budge.
        ClearFlag(ALL_FLAGS);
        select_ephemeris(&pred.satellite_ephem.tle);
    }
#endif

    // Auto-name log + wav + csv with a shared UTC stamp so they group
    // together in directory listings. Same convention as rx_live.
    if ((want_log && log_path == NULL) ||
        (want_wav && wav_path == NULL) ||
        (want_csv && csv_path == NULL)) {
        struct timeval tv;
        struct tm utc;
        if (gettimeofday(&tv, NULL) == 0 && gmtime_r(&tv.tv_sec, &utc) != NULL) {
            char base[200];
            snprintf(base, sizeof base,
                     "b210_rx_live_UT=%04d%02d%02dT%02d%02d%02d.%03ld",
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec,
                     (long)(tv.tv_usec / 1000));
            if (want_log && log_path == NULL) {
                snprintf(auto_log_path, sizeof auto_log_path, "%s.log", base);
                log_path = auto_log_path;
            }
            if (want_wav && wav_path == NULL) {
                snprintf(auto_wav_path, sizeof auto_wav_path, "%s.wav", base);
                wav_path = auto_wav_path;
            }
            if (want_csv && csv_path == NULL) {
                snprintf(auto_csv_path, sizeof auto_csv_path, "%s.csv", base);
                csv_path = auto_csv_path;
            }
        }
    }
    if (!want_log) log_path = NULL;
    if (!want_wav) wav_path = NULL;
    if (!want_csv) csv_path = NULL;

    // HMAC key load (only if requested).
    uint8_t hmac_key[128];
    ssize_t hmac_key_len = 0;
    if (use_hmac) {
        char default_path[512];
        const char *path = keyfile_path;
        if (path == NULL) {
            if (hmac_keyfile_default_path(default_path,
                                          sizeof default_path) != 0) {
                fprintf(stderr, "b210_rx_live: HOME unset; pass --keyfile=<path>\n");
                return EXIT_FAILURE;
            }
            path = default_path;
        }
        hmac_key_len = hmac_keyfile_load(path, hmac_key, sizeof hmac_key);
        if (hmac_key_len < 0) return EXIT_FAILURE;
    }

    // Open the B210 — also tunes to the (possibly Doppler-corrected)
    // initial freq. If Doppler is on we'd ideally compute the very first
    // freq from the prediction here; the simpler thing is to start at
    // nominal and let the first 1 s tick correct it.
    b210_rx_core_params_t cp = {
        .freq_hz         = nominal_freq_hz,
        .rate_hz         = rate_hz,
        .gain_db         = gain_db,
        .bw_hz           = bw_hz,
        .fm_fullscale_hz = fm_fullscale_hz,
        .device_args     = device_args,
        .rx_antenna      = rx_antenna,
        .decim_factor    = decim_factor,
        .decim_cutoff_hz = decim_cutoff_hz,
        .decim_taps      = decim_taps,
    };
    b210_rx_core_t *core = NULL;
    if (b210_rx_core_open(&cp, &core) != 0) {
        return EXIT_FAILURE;
    }
    double actual_rate = b210_rx_core_actual_rate(core);
    double input_rate  = b210_rx_core_input_rate(core);
    double actual_freq = b210_rx_core_actual_freq(core);
    int    samp_rate   = (int)actual_rate;
    if (samp_rate <= 0 || (samp_rate % bit_rate) != 0) {
        fprintf(stderr, "b210_rx_live: post-decim rate %d S/s isn't a multiple "
                        "of bit_rate %d — adjust --rate or --decim-factor so "
                        "(rate / decim_factor) divides bit_rate cleanly\n",
                samp_rate, bit_rate);
        b210_rx_core_close(core);
        return EXIT_FAILURE;
    }

    // Touch the log so `tail -F` works pre-AOS, with a session header
    // recording the run parameters.
    if (log_path != NULL) {
        FILE *lf = fopen(log_path, "a");
        if (lf == NULL) {
            fprintf(stderr, "b210_rx_live: fopen(%s, a): %s\n",
                    log_path, strerror(errno));
        } else {
            char ts[64];
            fmt_utc(ts, sizeof ts);
            fprintf(lf,
                    "[%s] b210_rx_live: session start freq=%.0f "
                    "input_rate=%.0f decim=%u (post-decim rate=%d) "
                    "decim_cutoff=%.0f decim_taps=%u "
                    "antenna=%s gain=%.1f window=%.2fs slide=%.2fs "
                    "sync_thr=%d rs=%s hmac=%s dc_block=%s doppler=%s\n",
                    ts, actual_freq,
                    input_rate, decim_factor, samp_rate,
                    decim_cutoff_hz, decim_taps,
                    rx_antenna, gain_db, window_s, slide_s,
                    sync_max_ham,
                    use_rs ? "on" : "off",
                    use_hmac ? "on" : "off",
                    no_dc_block ? "off" : "on",
                    do_doppler ? sat_name : "off");
            fclose(lf);
        }
    }

    // Optional WAV tee. Sized at the post-decim rate (matches what the
    // decoder sees and what rx_replay expects). At the default 48 kHz
    // mono int16 the WAV grows ~5.7 MB/min — small enough to leave on by
    // default for every run.
    wav_w_t wav = {0};
    if (wav_path != NULL) {
        if (wav_w_open(&wav, wav_path, samp_rate) != 0) {
            fprintf(stderr, "b210_rx_live: fopen(%s): %s\n",
                    wav_path, strerror(errno));
            b210_rx_core_close(core);
            return EXIT_FAILURE;
        }
    }

    // Optional per-second CSV companion. Each row: timestamp + freq +
    // Doppler + IQ peak/RMS dBFS + decoded-frame counts. Designed for
    // post-pass triage in a spreadsheet: see when the level meter
    // moved, when retunes happened, when decodes hit. One row per
    // second is plenty — beacon cadence is 20 s.
    FILE *csv = NULL;
    if (csv_path != NULL) {
        csv = fopen(csv_path, "w");
        if (csv == NULL) {
            fprintf(stderr, "b210_rx_live: fopen(%s): %s\n",
                    csv_path, strerror(errno));
            if (wav.fp) wav_w_close(&wav);
            b210_rx_core_close(core);
            return EXIT_FAILURE;
        }
        fprintf(csv,
                "# b210_rx_live session start utc=%s nominal_freq_hz=%.0f "
                "input_rate=%.0f post_rate=%d decim=%u decim_cutoff_hz=%.0f "
                "antenna=%s gain_db=%.1f window_s=%.2f slide_s=%.2f "
                "sync_thr=%d rs=%s hmac=%s dc_block=%s doppler=%s\n",
                "(see column utc_iso for per-row timestamps)",
                nominal_freq_hz, input_rate, samp_rate, decim_factor,
                decim_cutoff_hz,
                rx_antenna, gain_db, window_s, slide_s, sync_max_ham,
                use_rs ? "on" : "off",
                use_hmac ? "on" : "off",
                no_dc_block ? "off" : "on",
                do_doppler ? sat_name : "off");
        fprintf(csv,
                "utc_iso,t_s,freq_hz,doppler_offset_hz,range_rate_km_s,"
                "azimuth_deg,elevation_deg,"
                "iq_peak_dbfs,iq_rms_dbfs,frames_total,frames_in_window\n");
        fflush(csv);
    }

    // Optional ALSA monitor + carrier-presence squelch. Linux only;
    // on Mac builds (no WITH_ALSA) --monitor prints a notice and
    // continues without it. Squelch state is allocated regardless so
    // the REPL `sq` command always has somewhere to point — the
    // gating just skips snd_pcm_writei when alsa==NULL.
    monitor_squelch_t sq = {0};
    {
        monitor_squelch_params_t mp = {
            .rate_hz         = actual_rate,
            .auto_offset_db  = monitor_auto_offset_db,
            .init_mode       = monitor_sq_init,
            .init_thresh_db  = monitor_sq_user_db,
        };
        monitor_squelch_init(&sq, &mp);
    }
#ifdef WITH_ALSA
    snd_pcm_t *alsa = NULL;
    int16_t   *monitor_chunk = NULL;
    if (want_monitor) {
        int aerr = snd_pcm_open(&alsa, monitor_device,
                                SND_PCM_STREAM_PLAYBACK, 0);
        if (aerr < 0) {
            fprintf(stderr, "b210_rx_live: snd_pcm_open(%s): %s "
                            "— monitor disabled\n",
                    monitor_device, snd_strerror(aerr));
            alsa = NULL;
        } else {
            // 200 ms latency: forgiving of irregular UHD chunk arrival
            // without giving the operator a noticeable lag. soft_resample
            // = 1 lets ALSA's plug layer convert from 48 kHz to whatever
            // the playback HW prefers.
            unsigned int latency_us = 200000;
            aerr = snd_pcm_set_params(alsa,
                                      SND_PCM_FORMAT_S16_LE,
                                      SND_PCM_ACCESS_RW_INTERLEAVED,
                                      1,                    // mono
                                      (unsigned int)samp_rate,
                                      1,                    // soft_resample
                                      latency_us);
            if (aerr < 0) {
                fprintf(stderr, "b210_rx_live: snd_pcm_set_params"
                                "(%d Hz mono S16_LE): %s — monitor disabled\n",
                        samp_rate, snd_strerror(aerr));
                snd_pcm_close(alsa);
                alsa = NULL;
            } else {
                size_t mc = b210_rx_core_max_chunk(core);
                if (mc == 0) mc = 2040;
                monitor_chunk = (int16_t *)malloc(mc * sizeof(int16_t));
                if (monitor_chunk == NULL) {
                    fprintf(stderr, "b210_rx_live: out of memory for "
                                    "monitor buffer — monitor disabled\n");
                    snd_pcm_close(alsa);
                    alsa = NULL;
                } else {
                    char sqdesc[128];
                    monitor_squelch_status(&sq, sqdesc, sizeof sqdesc);
                    fprintf(stderr,
                            "b210_rx_live: monitor on %s @ %d Hz mono "
                            "(squelch=%s)\n",
                            monitor_device, samp_rate, sqdesc);
                }
            }
        }
    }
#else
    if (want_monitor) {
        fprintf(stderr, "b210_rx_live: --monitor requires ALSA support; "
                        "this binary was built without it.\n");
    }
#endif

    // Sliding window (mono int16). At 48 kHz post-decim with default
    // 1.5 s window = 72k samples = 144 KB. Same shape as rx_live.
    size_t window_samples = (size_t)(window_s * (double)samp_rate);
    size_t slide_samples  = (size_t)(slide_s  * (double)samp_rate);
    if (slide_samples == 0) slide_samples = 1;
    if (slide_samples > window_samples) slide_samples = window_samples;

    int16_t *window = calloc(window_samples, sizeof(int16_t));
    if (window == NULL) {
        fprintf(stderr, "b210_rx_live: out of memory for window (%zu samples)\n",
                window_samples);
        if (wav.fp) wav_w_close(&wav);
        if (csv != NULL) fclose(csv);
#ifdef WITH_ALSA
        if (alsa != NULL) snd_pcm_close(alsa);
        free(monitor_chunk);
#endif
        b210_rx_core_close(core);
        return EXIT_FAILURE;
    }
    size_t window_filled = 0;

    // Pump-output buffer sized to UHD's max recv chunk so each pump fits
    // in one call.
    size_t   max_chunk = b210_rx_core_max_chunk(core);
    if (max_chunk == 0) max_chunk = 2040;
    int16_t *pcm_chunk = malloc(max_chunk * sizeof(int16_t));
    if (pcm_chunk == NULL) {
        free(window);
        if (wav.fp) wav_w_close(&wav);
        if (csv != NULL) fclose(csv);
#ifdef WITH_ALSA
        if (alsa != NULL) snd_pcm_close(alsa);
        free(monitor_chunk);
#endif
        b210_rx_core_close(core);
        return EXIT_FAILURE;
    }

    // Decoder scratch.
    int    sps       = samp_rate / bit_rate;
    size_t bits_cap  = window_samples / (size_t)sps + 16;
    size_t bytes_cap = bits_cap / 8 + 16;
    uint8_t *bits_scratch  = malloc(bits_cap);
    uint8_t *bytes_scratch = malloc(bytes_cap);
    uint8_t  packet[4100];
    if (bits_scratch == NULL || bytes_scratch == NULL) {
        free(bits_scratch); free(bytes_scratch);
        free(pcm_chunk); free(window);
        if (wav.fp) wav_w_close(&wav);
        if (csv != NULL) fclose(csv);
#ifdef WITH_ALSA
        if (alsa != NULL) snd_pcm_close(alsa);
        free(monitor_chunk);
#endif
        b210_rx_core_close(core);
        return EXIT_FAILURE;
    }

    modem_params_t mp;
    modem_params_defaults(&mp);
    mp.samp_rate           = samp_rate;
    mp.bit_rate            = bit_rate;
    // Match rx_live's default: keep the modem's α=0.995 IIR DC-block
    // ON. At 48 kHz post-decim its corner is ~40 Hz, which removes
    // any sub-audio drift in the discriminator output (e.g. residual
    // carrier offset becoming a constant frequency bias) before the
    // bit slicer. Operator can disable with --no-dc-block.
    mp.rx_disable_dc_block = no_dc_block;

    ax100_opts_t opts;
    ax100_opts_defaults(&opts);
    if (use_hmac) {
        opts.hmac_key     = hmac_key;
        opts.hmac_key_len = (size_t)hmac_key_len;
    }
    opts.reed_solomon = use_rs;

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    // TUI banner / stderr banner. The live RX freq is intentionally
    // NOT in the header — it goes in the status row below the header
    // so it can be refreshed on every Doppler tick without rewriting
    // the whole title bar.
    char tui_header[512];
    char tui_status[320];
    if (use_tui) {
        int tui_rc = rx_tui_init();
        if (tui_rc != 0) {
            // Default-on mode: fall back to text. Only error out when
            // the operator explicitly asked for --ui and the build
            // can't honour it.
            if (use_tui_explicit) {
                fprintf(stderr, "b210_rx_live: --ui requested but ncurses is "
                        "not built in (rebuild with libncurses-dev "
                        "installed).\n");
                free(bits_scratch); free(bytes_scratch);
                free(pcm_chunk); free(window);
                if (wav.fp) wav_w_close(&wav);
                if (csv != NULL) fclose(csv);
#ifdef WITH_ALSA
                if (alsa != NULL) snd_pcm_close(alsa);
                free(monitor_chunk);
#endif
                b210_rx_core_close(core);
                return EXIT_FAILURE;
            }
            fprintf(stderr, "b210_rx_live: ncurses not built in, falling "
                    "back to streaming stderr output (pass --no-ui to "
                    "silence this notice).\n");
            use_tui = 0;
        }
    }
    int want_status_refresh = 0;
    spectrum_job_t spec_job = {0};
    int monitor_active = 0;
    decode_loop_set_show_headers(show_packet_headers);
#ifdef WITH_ALSA
    monitor_active = (alsa != NULL);
#endif
    b210_cmd_ctx_t cmd_ctx = {
        .opts                = &opts,
        .force_beacon        = &force_beacon,
        .want_status_refresh = &want_status_refresh,
        .wav                 = wav.fp ? &wav      : NULL,
        .wav_path            = wav.fp ? wav_path  : NULL,
        .spec_job            = &spec_job,
        .sq                  = monitor_active ? &sq : NULL,
    };

    if (use_tui) {
        // Header carries the static-ish run params. Reed-Solomon is
        // dropped from the header (it's now toggleable mid-run via the
        // REPL) and shown in the live status row instead so the
        // operator sees it flip the moment they type `rs on/off`.
        snprintf(tui_header, sizeof tui_header,
                 "b210_rx_live | %.0f→%d S/s (M=%u) | doppler=%s | "
                 "win=%.2fs slide=%.2fs sync_thr=%d hmac=%s | log=%s",
                 input_rate, samp_rate, decim_factor,
                 do_doppler ? sat_name : "off",
                 window_s, slide_s, sync_max_ham,
                 use_hmac ? "on" : "off",
                 log_path ? log_path : "(none)");
        rx_tui_set_header(tui_header);
        char sqsuffix[80] = {0};
        if (monitor_active) {
            char sqdesc[64];
            monitor_squelch_status(&sq, sqdesc, sizeof sqdesc);
            snprintf(sqsuffix, sizeof sqsuffix, " | sq=%s", sqdesc);
        }
        char lvldesc[48];
        iq_level_format(core, lvldesc, sizeof lvldesc);
        if (do_doppler) {
            snprintf(tui_status, sizeof tui_status,
                     "freq=%.6f MHz (waiting for first Doppler tick) | "
                     "nominal=%.6f MHz | rs=%s fb=%s%s | %s | sat=%s",
                     actual_freq / 1e6, nominal_freq_hz / 1e6,
                     opts.reed_solomon ? "on" : "off",
                     force_beacon ? "on" : "off", sqsuffix, lvldesc, sat_name);
        } else {
            snprintf(tui_status, sizeof tui_status,
                     "freq=%.6f MHz | rs=%s fb=%s%s | %s | no-doppler",
                     actual_freq / 1e6,
                     opts.reed_solomon ? "on" : "off",
                     force_beacon ? "on" : "off", sqsuffix, lvldesc);
        }
        rx_tui_set_status(tui_status);
        // Persistent REPL history: $HOME/.local/state/simple_sat_ops/
        // b210_rx_live_history. mkdir -p the parent best-effort.
        const char *home = getenv("HOME");
        if (home != NULL && home[0] != '\0') {
            ensure_state_dir(home);
            char hist_path[512];
            snprintf(hist_path, sizeof hist_path,
                     "%s/.local/state/simple_sat_ops/b210_rx_live_history",
                     home);
            rx_tui_set_history_path(hist_path);
        }
        rx_tui_set_command_handler(cmd_handler, &cmd_ctx);
        quiet = 1;
    } else {
        char sqdesc_init[64] = "n/a";
        if (monitor_active) {
            monitor_squelch_status(&sq, sqdesc_init, sizeof sqdesc_init);
        }
        fprintf(stderr,
                "b210_rx_live: freq=%.6f MHz input_rate=%.0f decim=%u "
                "post=%d gain=%.1f bw=%.0f "
                "window=%.2fs slide=%.2fs sync_thr=%d rs=%s hmac=%s "
                "dc_block=%s force-beacon=%s monitor=%s sq=%s doppler=%s log=%s\n",
                actual_freq / 1e6, input_rate, decim_factor, samp_rate,
                gain_db, bw_hz > 0 ? bw_hz : input_rate,
                window_s, slide_s, sync_max_ham,
                use_rs   ? "on" : "off",
                use_hmac ? "on" : "off",
                no_dc_block ? "off" : "on",
                force_beacon ? "on" : "off",
                monitor_active ? "on" : "off",
                sqdesc_init,
                do_doppler ? sat_name : "off",
                log_path ? log_path : "(none)");
    }

    // Dedup ring (same parameters as rx_live).
    enum { DEDUP_RING_SZ = 64 };
    enum { DEDUP_QUANT_SAMPLES_AT_48K = 4800 };  // 100 ms @ 48 kHz
    // Scale to actual rate so the 100 ms window holds across rates.
    uint64_t dedup_quant = (uint64_t)(0.1 * (double)samp_rate);
    if (dedup_quant == 0) dedup_quant = DEDUP_QUANT_SAMPLES_AT_48K;
    uint64_t recent_pos_quant[DEDUP_RING_SZ] = {0};
    int      recent_idx   = 0;
    int      recent_count = 0;
    uint64_t total_window_samples = 0;

    double t_start            = monotonic_seconds();
    double last_doppler_tick  = -1e9;  // force a Doppler tick on the first iteration
    // Level-meter cadence. The TUI status row redraws when
    // want_status_refresh is set, so to make the iq=peak/rms line move
    // visibly we bump the flag every ~100 ms; the underlying envelope
    // is updated every UHD pump regardless. In non-TUI mode the same
    // timer drives a periodic stderr line at a saner cadence.
    double last_level_redraw  = -1e9;
    double last_level_print   = -1e9;
    // CSV row cadence + decoded-frame counters. frames_total persists
    // across rows; frames_in_window counts decodes since the last CSV
    // write and resets after each row.
    double   last_csv_tick      = -1e9;
    uint64_t frames_total       = 0;
    unsigned frames_in_window   = 0;
    // Cache the most recent Doppler-tick output so CSV rows can include
    // range_rate / az / el without forcing a new SGP4 evaluation on a
    // non-doppler run. NaN sentinel indicates "never computed" — the
    // CSV writer leaves those fields blank for those rows.
    double   last_range_rate_km_s = (0.0/0.0);
    double   last_az_deg          = (0.0/0.0);
    double   last_el_deg          = (0.0/0.0);

    while (!g_stop) {
        if (use_tui && rx_tui_tick()) g_stop = 1;
        // Cmd handler may have toggled rs since the last Doppler tick.
        // Refresh the status row immediately so the operator sees the
        // change without waiting up to a second for the next tick.
        // The next Doppler tick overwrites this with the full freq +
        // range_rate string.
        if (use_tui && want_status_refresh) {
            char sqsuffix[80] = {0};
            if (monitor_active) {
                char sqdesc[64];
                monitor_squelch_status(&sq, sqdesc, sizeof sqdesc);
                snprintf(sqsuffix, sizeof sqsuffix, " | sq=%s", sqdesc);
            }
            char lvldesc[48];
            iq_level_format(core, lvldesc, sizeof lvldesc);
            snprintf(tui_status, sizeof tui_status,
                     "freq=%.6f MHz | rs=%s fb=%s%s | %s%s%s",
                     actual_freq / 1e6,
                     opts.reed_solomon ? "on" : "off",
                     force_beacon ? "on" : "off",
                     sqsuffix, lvldesc,
                     do_doppler ? " | sat="     : " | no-doppler",
                     do_doppler ? sat_name      : "");
            rx_tui_set_status(tui_status);
            want_status_refresh = 0;
        }
        // Reap a finished spectrum render and surface its status.
        // The next Doppler tick overwrites the status row a second
        // later, which is enough time to see "wrote /path/to/png" go by.
        if (spec_job.active && spec_job.done) {
            pthread_join(spec_job.thr, NULL);
            spec_job.active = 0;
            if (use_tui) rx_tui_set_status(spec_job.status_msg);
            else fprintf(stderr, "b210_rx_live: %s\n", spec_job.status_msg);
        }
        if (duration_s > 0.0 && (monotonic_seconds() - t_start) >= duration_s) break;

        // Pump one UHD chunk.
        ssize_t n = b210_rx_core_pump(core, pcm_chunk, max_chunk);
        if (n < 0) {
            fprintf(stderr, "b210_rx_live: b210_rx_core_pump fatal\n");
            break;
        }
        if (n == 0) {
            // Transient (overflow / timeout). Still service Doppler if
            // due so a stalled UHD doesn't freeze the retune loop.
            goto doppler_tick;
        }

        if (wav.fp != NULL) {
            wav_w_append(&wav, pcm_chunk, (size_t)n);
        }

        // Drive the level-meter display. The envelope itself was
        // updated inside b210_rx_core_pump on the post-FIR IQ stream;
        // here we just decide when to redraw. 100 ms refresh in TUI
        // mode is fast enough to see beacon-burst rises; 1 s stderr
        // cadence in non-TUI mode keeps logs readable.
        {
            double t_lvl = monotonic_seconds();
            if (use_tui) {
                if (t_lvl - last_level_redraw >= 0.1) {
                    want_status_refresh = 1;
                    last_level_redraw   = t_lvl;
                }
            } else if (!quiet) {
                if (t_lvl - last_level_print >= 1.0) {
                    char lvldesc[48], ts[64];
                    iq_level_format(core, lvldesc, sizeof lvldesc);
                    fmt_utc(ts, sizeof ts);
                    fprintf(stderr,
                            "[%s] b210_rx_live: %s\n",
                            ts, lvldesc);
                    last_level_print = t_lvl;
                }
            }
            // Per-second CSV row. Independent of level-print cadence so
            // the CSV stays consistent across TUI / non-TUI / quiet runs.
            if (csv != NULL && t_lvl - last_csv_tick >= 1.0) {
                double peak = 0.0, rms_sq = 0.0;
                b210_rx_core_iq_levels(core, &peak, &rms_sq);
                double rms = sqrt(rms_sq);
                double peak_dbfs = (peak < 1.0) ? -90.0 : 20.0 * log10(peak / 32768.0);
                double rms_dbfs  = (rms  < 1.0) ? -90.0 : 20.0 * log10(rms  / 32768.0);
                char ts[64];
                fmt_utc(ts, sizeof ts);
                double rel_t = t_lvl - t_start;
                double off_hz = actual_freq - nominal_freq_hz;
                if (do_doppler && last_range_rate_km_s == last_range_rate_km_s) {
                    fprintf(csv,
                            "%s,%.3f,%.0f,%+.1f,%+.4f,"
                            "%.2f,%+.2f,%+.2f,%+.2f,%llu,%u\n",
                            ts, rel_t, actual_freq, off_hz,
                            last_range_rate_km_s,
                            last_az_deg, last_el_deg,
                            peak_dbfs, rms_dbfs,
                            (unsigned long long)frames_total,
                            frames_in_window);
                } else {
                    // Empty fields for doppler_offset_hz / range_rate_km_s
                    // / az / el when Doppler isn't being tracked (or
                    // hasn't ticked yet). Spreadsheet importers handle
                    // the gaps fine.
                    fprintf(csv,
                            "%s,%.3f,%.0f,,,,,%+.2f,%+.2f,%llu,%u\n",
                            ts, rel_t, actual_freq, peak_dbfs, rms_dbfs,
                            (unsigned long long)frames_total,
                            frames_in_window);
                }
                fflush(csv);
                frames_in_window = 0;
                last_csv_tick    = t_lvl;
            }
        }

#ifdef WITH_ALSA
        // Live monitor: squelch + ALSA playback. Bootstrap completion
        // bumps the status row so the operator sees "auto(boot)" flip
        // to "auto(thr=...)". snd_pcm_recover handles XRUNs silently.
        if (alsa != NULL && monitor_chunk != NULL) {
            msq_mode_t prev_mode = sq.mode;
            monitor_squelch_process(&sq, pcm_chunk, monitor_chunk, (size_t)n);
            if (prev_mode == MSQ_AUTO_BOOTSTRAPPING
                && sq.mode == MSQ_AUTO_ENGAGED) {
                want_status_refresh = 1;
            }
            snd_pcm_sframes_t w = snd_pcm_writei(alsa, monitor_chunk, (size_t)n);
            if (w < 0) {
                int rerr = snd_pcm_recover(alsa, (int)w, /*silent=*/1);
                if (rerr < 0) {
                    fprintf(stderr, "b210_rx_live: snd_pcm_writei recover: %s\n",
                            snd_strerror(rerr));
                }
            }
        }
#endif

        // Append to the sliding window, decode whenever the window fills.
        for (ssize_t i = 0; i < n; i++) {
            window[window_filled++] = pcm_chunk[i];
            ++total_window_samples;
            if (window_filled < window_samples) continue;

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
                if (plen < 4 || (size_t)plen > sizeof packet) continue;

                int       crc_status   = -1;
                uint32_t  crc_computed = 0, crc_le = 0, crc_be = 0;
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

                // Dedup by quantised absolute ASM sample index. Same
                // logic as rx_live; window_start_abs =
                // total_window_samples - window_samples.
                uint64_t window_start_abs =
                    total_window_samples - (uint64_t)window_samples;
                uint64_t asm_abs_sample = window_start_abs
                    + (uint64_t)sync_off_local * (uint64_t)sps
                    + (uint64_t)(sps / 2);
                uint64_t pos_quant = asm_abs_sample / dedup_quant;
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
                           NULL, 0,
                           force_beacon);
                frames_total++;
                if (frames_in_window < UINT_MAX) frames_in_window++;
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

doppler_tick:
        // Doppler retune at most once per second. The 1 s cadence
        // matches simple_sat_ops; tighter would just spam UHD without
        // changing anything since range_rate moves the carrier on the
        // order of tens of Hz per second at LEO UHF.
#ifdef WITH_SGP4SDP4
        if (do_doppler) {
            double now = monotonic_seconds();
            if (now - last_doppler_tick >= 1.0) {
                // Use UTC_Calendar_Now, not gettimeofday + gmtime_r:
                // sgp4sdp4's Julian_Date wants tm_year as the absolute
                // year (e.g. 2026) and tm_mon in 1..12. Standard POSIX
                // gmtime_r returns tm_year=126, tm_mon=0..11 — feed
                // that into Julian_Date and you get a date roughly two
                // millennia off, SGP4 returns garbage, range_rate bounces
                // between bogus values. UTC_Calendar_Now does the +1900
                // / +1 fixup the lib expects.
                struct tm utc;
                struct timeval tv;
                UTC_Calendar_Now(&utc, &tv);
                {
                    double jul_utc = Julian_Date(&utc, &tv);
                    update_satellite_position(&pred, jul_utc);
                    double range_rate_km_s =
                        pred.satellite_ephem.range_rate_km_s;
                    last_range_rate_km_s = range_rate_km_s;
                    last_az_deg          = pred.satellite_ephem.azimuth;
                    last_el_deg          = pred.satellite_ephem.elevation;
                    double doppler_freq = nominal_freq_hz
                        * (1.0 - range_rate_km_s / 299792.458);
                    if (fabs(doppler_freq - actual_freq) >= doppler_thr_hz) {
                        double prev_freq = actual_freq;
                        if (b210_rx_core_set_freq(core, doppler_freq) == 0) {
                            actual_freq = b210_rx_core_actual_freq(core);
                            // Both target (what we asked UHD for) and
                            // actual (what UHD coerced to) — useful for
                            // diagnosing flip-flop where UHD's coerced
                            // value disagrees with our request enough to
                            // re-trip the threshold next tick.
                            if (!quiet && !use_tui) {
                                char ts[64];
                                fmt_utc(ts, sizeof ts);
                                fprintf(stderr,
                                        "[%s] b210_rx_live: retune target=%.6f "
                                        "actual=%.6f MHz (was %.6f, coerce=%+.0f Hz, "
                                        "range_rate=%+.3f km/s)\n",
                                        ts, doppler_freq / 1e6,
                                        actual_freq / 1e6, prev_freq / 1e6,
                                        actual_freq - doppler_freq,
                                        range_rate_km_s);
                            }
                        }
                    }
                    // Refresh the TUI status row every tick (whether
                    // or not we retuned) so the operator sees range_rate
                    // moving as the pass progresses. Outside --ui mode
                    // this is a no-op.
                    if (use_tui) {
                        double off_hz = actual_freq - nominal_freq_hz;
                        char sqsuffix[80] = {0};
                        if (monitor_active) {
                            char sqdesc[64];
                            monitor_squelch_status(&sq, sqdesc, sizeof sqdesc);
                            snprintf(sqsuffix, sizeof sqsuffix,
                                     " | sq=%s", sqdesc);
                        }
                        char lvldesc[48];
                        iq_level_format(core, lvldesc, sizeof lvldesc);
                        snprintf(tui_status, sizeof tui_status,
                                 "freq=%.6f MHz (Δ=%+.0f Hz from %.6f MHz) | "
                                 "range_rate=%+.3f km/s | rs=%s fb=%s%s | "
                                 "%s | sat=%s",
                                 actual_freq / 1e6, off_hz,
                                 nominal_freq_hz / 1e6, range_rate_km_s,
                                 opts.reed_solomon ? "on" : "off",
                                 force_beacon ? "on" : "off",
                                 sqsuffix, lvldesc, sat_name);
                        rx_tui_set_status(tui_status);
                        want_status_refresh = 0;
                    }
                }
                last_doppler_tick = now;
            }
        }
#endif
    }

    if (use_tui) rx_tui_close();
    if (g_stop) {
        fprintf(stderr, "b210_rx_live: stopped on signal\n");
    }

    free(bits_scratch);
    free(bytes_scratch);
    free(pcm_chunk);
    free(window);
#ifdef WITH_ALSA
    if (alsa != NULL) {
        snd_pcm_drain(alsa);
        snd_pcm_close(alsa);
    }
    free(monitor_chunk);
#endif
    // Wait for any in-flight `spectrum N` render to finish before we
    // tear down the WAV — the worker is still reading bytes from it.
    if (spec_job.active) {
        pthread_join(spec_job.thr, NULL);
        spec_job.active = 0;
        if (spec_job.status_msg[0]) {
            fprintf(stderr, "b210_rx_live: %s\n", spec_job.status_msg);
        }
    }
    if (wav.fp) {
        // Close (patches the WAV header sizes) before invoking ffmpeg
        // so showspectrumpic reads a complete file. Best-effort: if
        // ffmpeg isn't on PATH the helper logs and returns non-zero
        // and we still exit cleanly.
        wav_w_close(&wav);
        if (want_spectrogram && wav_path != NULL) {
            (void)generate_spectrogram(wav_path);
        }
    }
    if (csv != NULL) fclose(csv);
    b210_rx_core_close(core);
    return 0;
}
