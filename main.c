/*

   Simple Satellite Operations  main.c

   Copyright (C) 2025  Johnathan K Burchill

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

#define _GNU_SOURCE

#include "antenna_rotator.h"
#include "state.h"
#include "prediction.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_ipc_paths.h"
#include "sso_paths.h"
#include "tle_csv.h"
#include "frontiersat.h"

#ifdef WITH_USRP_B210
#include "b210_rx_tx_core.h"
#include "rx_session.h"
#include "tx_burst.h"
#endif

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <locale.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <ncurses.h>
#include <unistd.h>

// Carrier defaults (used only for display + IPC publication; the
// actual radio is now driven externally by tx_frame_sdr / b210_rx_tx
// over the sso_ipc socket).
#define UPLINK_FREQ_MHZ   145.150000
#define DOWNLINK_FREQ_MHZ 436.150000

// Doppler retune threshold (Hz). Frequencies update on the display
// when the residual drifts past this. 200 Hz keeps the residual offset
// well inside the 9600 GFSK clean-eye band (~±3 kHz) even at the peak
// ~100 Hz/s slew near TCA.
#define DOPPLER_SHIFT_RESOLUTION_KHZ 0.2

// Antenna rotator max angle from target before a new SET is issued.
#define MAX_DELTA_AZIMUTH_DEGREES 1.0
#define MAX_DELTA_ELEVATION_DEGREES 1.0

#define WARN_DAYS_SINCE_EPOCH 1.0
#define MAX_MINUTES_TO_PREDICT ((7 * 1440))

#define UPDATE_INTERVAL_MICROSEC 500000

#define SSO_IPC_MAX_CLIENTS_FOR_ROSTER 16

// --- Operator-mode IPC bookkeeping ---------------------------------
// Set by apply_args when --control is passed. When set, main() opens
// the sso_ipc server on /run/sso/simple_sat_ops.sock and fans out a
// state event on every UI tick. Other operator-aware tools
// (b210_rx_tx --control, tx_frame_sdr) verify the operator's Unix
// user matches their own via this socket.
static int g_control_mode = 0;
static int g_viewer_mode = 0;  // bare invocation found a running operator
static sso_ipc_server_t *g_ipc = NULL;
static const char *g_operator_user = NULL;

// Signal ribbon: 60-second 1 Hz rolling window of RX peak dBFS rendered
// as a UTF-8 block-character strip in the RX panel. Oldest sample on
// the left, newest on the right. Cheap fixed-size; the sampler is
// gated by monotonic seconds in the main loop.
// Signal ribbon: 1 Hz timeline of "I am alive" marks rendered as a
// vertical strip on the right side of the screen. Each char represents
// one second; the most recent sample sits at the bottom of the strip
// and older samples sit above. Plain ASCII ('.' / '-') so the display
// works on minimal SSH sessions / TTYs without UTF-8 fonts.
//
// Tick semantics: every 20 seconds in absolute push-time gets a bold
// '-' instead of '.'. Because the tick is keyed to absolute push count
// (not to a fixed visual position), the tick crawls upward by one row
// per second — the eye reads the timeline progressing even when the
// signal is flat.
#define RIBBON_LEN 60
static double g_ribbon_peak[RIBBON_LEN];
static int    g_ribbon_count       = 0;  // number of valid samples (caps at RIBBON_LEN)
static int    g_ribbon_head        = 0;  // next write index (circular)
static double g_ribbon_last_t      = 0.0;
static long   g_ribbon_push_count  = 0;  // total pushes since startup; drives ticks

static void ribbon_push(double peak_dbfs)
{
    g_ribbon_peak[g_ribbon_head] = peak_dbfs;
    g_ribbon_head = (g_ribbon_head + 1) % RIBBON_LEN;
    if (g_ribbon_count < RIBBON_LEN) g_ribbon_count++;
    g_ribbon_push_count++;
}

// Low-disk warning. statvfs on the pass folder filesystem; rendered in
// the RX panel when free space dips below LOW_DISK_BYTES. Re-checked
// every LOW_DISK_PERIOD_S seconds so we don't statvfs on every redraw.
#define LOW_DISK_BYTES   ((uint64_t)10 * 1000 * 1000 * 1000)
#define LOW_DISK_PERIOD_S 30.0
static char   g_low_disk_msg[80] = "";
static double g_low_disk_last_t  = 0.0;
// Tentative declaration so low_disk_refresh can reference g_pass_folder
// without reordering the whole file; the definition with an initialiser
// lives further down.
static char   g_pass_folder[256];

// Returns bytes available to a non-privileged user on the filesystem
// hosting `path`. Returns (uint64_t) -1 on error or when path is empty.
static uint64_t free_disk_bytes(const char *path)
{
    if (path == NULL || path[0] == '\0') return (uint64_t) -1;
    struct statvfs s;
    if (statvfs(path, &s) != 0) return (uint64_t) -1;
    return (uint64_t) s.f_bavail * (uint64_t) s.f_frsize;
}

// Refresh g_low_disk_msg if the period has elapsed. Empty message
// means "above threshold" — render_rx_panel skips the row in that case.
static void low_disk_refresh(double t_now)
{
    if ((t_now - g_low_disk_last_t) < LOW_DISK_PERIOD_S
        && g_low_disk_last_t != 0.0) return;
    g_low_disk_last_t = t_now;
    const char *probe = g_pass_folder[0] ? g_pass_folder : ".";
    uint64_t avail = free_disk_bytes(probe);
    if (avail == (uint64_t) -1) {
        g_low_disk_msg[0] = '\0';
        return;
    }
    if (avail >= LOW_DISK_BYTES) {
        g_low_disk_msg[0] = '\0';
        return;
    }
    double gb = (double) avail / 1.0e9;
    // Cap path width so the message fits in the 80-byte buffer: prefix
    // is ~26 chars ("LOW DISK: 12.34 GB free at "), leaving 50 for the
    // path. GCC -Wformat-truncation would otherwise flag the unbounded
    // %s against a 255-byte g_pass_folder.
    snprintf(g_low_disk_msg, sizeof g_low_disk_msg,
             "LOW DISK: %.2f GB free at %.50s", gb, probe);
}

// Spectrogram render job. The `:spectrum N` REPL command snapshots the
// last N seconds of the live WAV (which the rx_session worker is still
// appending to), copies them into a temporary WAV, and shells out to
// ffmpeg's showspectrumpic. Runs on its own pthread so the main loop
// keeps ticking. Single slot — only one render at a time.
//
// Caveat: the WAV is FM-demoded mono PCM, not IQ. That puts a hard
// ceiling on how SatNOGS-like these spectrograms can look — see the
// commentary in b210_rx_tx_core.c (line ~303) about the FM-discriminator
// noise floor dropping on carrier capture. For a SatNOGS-style waterfall
// against a flat thermal floor we'd need to tap the IQ stream before
// the discriminator; that's a separate feature.
typedef struct spectrum_job {
    pthread_t       thr;
    int             active;          // 1 once the thread has been launched
    volatile int    done;            // worker sets to 1 just before return
    // Source — pick one. When iq_in[0] is non-empty the worker renders
    // a SatNOGS-style waterfall via gen_waterfall(1) on the IQ slice;
    // otherwise it falls back to the FM-demod WAV slice through ffmpeg.
    char            wav_in[512];
    int             sample_rate;
    long            start_sample;
    long            n_samples;
    char            iq_in[512];
    int             iq_sample_rate;
    long             iq_start_pair;
    long             iq_pairs;
    char            png_out[640];
    char            status_msg[1024];
} spectrum_job_t;

static spectrum_job_t g_spec_job;

// Render a full IQ recording with gen_waterfall — SatNOGS-style
// viridis waterfall, no ffmpeg dependency, signals pop against a
// flat median-subtracted noise floor. Returns 0 on success, -1 on
// fork/exec failure or non-zero gen_waterfall exit.
static int generate_full_iq_waterfall(const char *iq_path, int rate_hz,
                                      char *png_out, size_t png_cap)
{
    if (iq_path == NULL || rate_hz <= 0) return -1;
    size_t len = strlen(iq_path);
    char png[640];
    int n;
    if (len >= 3 && strcmp(iq_path + len - 3, ".iq") == 0) {
        n = snprintf(png, sizeof png, "%.*s_waterfall.png",
                     (int)(len - 3), iq_path);
    } else {
        n = snprintf(png, sizeof png, "%s_waterfall.png", iq_path);
    }
    if (n <= 0 || (size_t) n >= sizeof png) return -1;
    char rate_buf[16];
    snprintf(rate_buf, sizeof rate_buf, "%d", rate_hz);

    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        char *args[] = {
            "gen_waterfall",
            (char *) iq_path, rate_buf, png,
            NULL,
        };
        execvp("gen_waterfall", args);
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return -1;
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        if (png_out && png_cap) snprintf(png_out, png_cap, "%s", png);
        return 0;
    }
    return -1;
}

// Render a finished WAV directly (no slicing) via ffmpeg. Blocks until
// ffmpeg exits. Returns 0 on success, -1 on fork/exec/exit failure.
// Used at end-of-pass on the final closed WAV.
static int generate_full_spectrogram(const char *wav_path, char *png_out, size_t png_cap)
{
    if (wav_path == NULL) return -1;
    size_t len = strlen(wav_path);
    char png[640];
    int n;
    if (len >= 4 && strcmp(wav_path + len - 4, ".wav") == 0) {
        n = snprintf(png, sizeof png, "%.*s.png", (int)(len - 4), wav_path);
    } else {
        n = snprintf(png, sizeof png, "%s.png", wav_path);
    }
    if (n <= 0 || (size_t) n >= sizeof png) return -1;

    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        char *args[] = {
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-i", (char *) wav_path,
            "-lavfi",
            "showspectrumpic=s=1920x1080:mode=combined:color=viridis:scale=log:legend=1:saturation=1.5",
            png, NULL,
        };
        execvp("ffmpeg", args);
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return -1;
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        if (png_out && png_cap) snprintf(png_out, png_cap, "%s", png);
        return 0;
    }
    return -1;
}

// IQ-slice branch of spectrum_worker. Snapshots `j->iq_pairs` pairs
// starting at `j->iq_start_pair` from the live .iq file, then shells
// out gen_waterfall on the slice. Returns 0 on success, -1 on failure
// (the worker fills j->status_msg in either case).
static int spectrum_worker_iq(spectrum_job_t *j)
{
    char tmp_iq[700];
    snprintf(tmp_iq, sizeof tmp_iq, "%s.tmp.iq", j->png_out);

    FILE *fin = fopen(j->iq_in, "rb");
    if (fin == NULL) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: open %s failed: %s", j->iq_in, strerror(errno));
        return -1;
    }
    if (fseek(fin, j->iq_start_pair * 4, SEEK_SET) != 0) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: iq seek failed: %s", strerror(errno));
        fclose(fin); return -1;
    }
    FILE *fout = fopen(tmp_iq, "wb");
    if (fout == NULL) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: open %s failed: %s", tmp_iq, strerror(errno));
        fclose(fin); return -1;
    }
    int16_t buf[4096];     // 1024 pairs per read
    long remaining = j->iq_pairs;
    while (remaining > 0) {
        long want_pairs = remaining > (long)(sizeof buf / 4)
                        ? (long)(sizeof buf / 4) : remaining;
        size_t int16_count = (size_t)(want_pairs * 2);
        size_t got = fread(buf, sizeof(int16_t), int16_count, fin);
        if (got == 0) break;
        fwrite(buf, sizeof(int16_t), got, fout);
        remaining -= (long)(got / 2);
    }
    fclose(fin); fclose(fout);

    char rate_buf[16];
    snprintf(rate_buf, sizeof rate_buf, "%d", j->iq_sample_rate);
    pid_t pid = fork();
    int rc = -1;
    if (pid == 0) {
        char *args[] = {
            "gen_waterfall",
            tmp_iq, rate_buf, j->png_out,
            NULL,
        };
        execvp("gen_waterfall", args);
        _exit(127);
    } else if (pid > 0) {
        int status;
        if (waitpid(pid, &status, 0) >= 0 && WIFEXITED(status))
            rc = WEXITSTATUS(status);
    }
    unlink(tmp_iq);
    if (rc == 0) {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: wrote %s (%.1fs IQ)",
                 j->png_out,
                 (double) j->iq_pairs / (double) j->iq_sample_rate);
        return 0;
    }
    snprintf(j->status_msg, sizeof j->status_msg,
             "spectrum: gen_waterfall failed (rc=%d) for %s", rc, j->png_out);
    return -1;
}

static void *spectrum_worker(void *arg)
{
    spectrum_job_t *j = (spectrum_job_t *) arg;
    // Prefer the IQ path — gives a real SatNOGS-style waterfall. The
    // WAV fallback below stays in place for runs where the IQ sidecar
    // didn't open (e.g., disk full mid-pass).
    if (j->iq_in[0] && j->iq_pairs > 0 && j->iq_sample_rate > 0) {
        (void) spectrum_worker_iq(j);
        j->done = 1;
        return NULL;
    }
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
    uint32_t sr   = (uint32_t) j->sample_rate;
    uint32_t bps  = sr * 2;
    uint32_t bcnt = (uint32_t)(j->n_samples * 2);
    uint32_t fsz  = bcnt + 36;
    uint8_t hdr[44] = {
        'R','I','F','F',
        (uint8_t) fsz,(uint8_t)(fsz>>8),(uint8_t)(fsz>>16),(uint8_t)(fsz>>24),
        'W','A','V','E', 'f','m','t',' ', 16,0,0,0, 1,0, 1,0,
        (uint8_t) sr, (uint8_t)(sr>>8), (uint8_t)(sr>>16), (uint8_t)(sr>>24),
        (uint8_t) bps,(uint8_t)(bps>>8),(uint8_t)(bps>>16),(uint8_t)(bps>>24),
        2,0, 16,0,
        'd','a','t','a',
        (uint8_t) bcnt,(uint8_t)(bcnt>>8),(uint8_t)(bcnt>>16),(uint8_t)(bcnt>>24),
    };
    if (fwrite(hdr, 1, 44, fout) != 44) {
        snprintf(j->status_msg, sizeof j->status_msg, "spectrum: header write failed");
        fclose(fin); fclose(fout); unlink(tmp_wav); j->done = 1; return NULL;
    }
    int16_t buf[4096];
    long remaining = j->n_samples;
    while (remaining > 0) {
        size_t want = remaining > (long)(sizeof buf / sizeof buf[0])
                    ? sizeof buf / sizeof buf[0] : (size_t) remaining;
        size_t got = fread(buf, sizeof buf[0], want, fin);
        if (got == 0) break;
        fwrite(buf, sizeof buf[0], got, fout);
        remaining -= (long) got;
    }
    fclose(fin); fclose(fout);

    pid_t pid = fork();
    int rc = -1;
    if (pid == 0) {
        char *args[] = {
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-i", tmp_wav,
            "-lavfi",
            "showspectrumpic=s=1920x1080:mode=combined:color=viridis:scale=log:legend=1:saturation=1.5",
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
                 j->png_out, (double) j->n_samples / (double) j->sample_rate);
    } else {
        snprintf(j->status_msg, sizeof j->status_msg,
                 "spectrum: ffmpeg failed (rc=%d) for %s", rc, j->png_out);
    }
    j->done = 1;
    return NULL;
}

// Reap a finished spectrum job so the slot is free for the next request.
// Called both from cmd_dispatch (so the operator can retry) and from the
// shutdown path (so we don't leak the worker thread).
static void spectrum_job_reap(void)
{
    if (g_spec_job.active && g_spec_job.done) {
        pthread_join(g_spec_job.thr, NULL);
        g_spec_job.active = 0;
    }
}

// Command line: vi-style ":" prompt at the bottom of the screen for
// runtime actions. While g_cmd_active, every key is routed through the
// command handler instead of the main key switch.
#define CMD_BUF_SIZE 128
static int  g_cmd_active = 0;
static char g_cmd_buf[CMD_BUF_SIZE];
static int  g_cmd_len = 0;
static int  g_cmd_cursor = 0;            // 0..g_cmd_len; insert position
static char g_cmd_status[160] = "";
// cmd-preview debounce state. cmd_dirty is set every time the buffer is
// edited (or :  is entered fresh); the main loop broadcasts a preview
// event once the buffer has been idle for cmd_debounce_ns. Mirrors how
// the TX compose modal debounces its tx-preview events.
static int    g_cmd_dirty        = 0;
static long   g_cmd_last_edit_ns = 0;
static long   g_cmd_debounce_ns  = 150000000L;  // 150 ms

static long cmd_now_ns(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (long) ts.tv_sec * 1000000000L + (long) ts.tv_nsec;
}

static void cmd_enter(void)
{
    g_cmd_active = 1;
    g_cmd_buf[0] = '\0';
    g_cmd_len = 0;
    g_cmd_cursor = 0;
    // Force an immediate preview broadcast so viewers see the ":" prompt
    // appear the moment the operator opens it.
    g_cmd_dirty = 1;
    g_cmd_last_edit_ns = 0;
}

static void cmd_set_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_cmd_status, sizeof g_cmd_status, fmt, ap);
    va_end(ap);
}

// Forward decls so cmd_dispatch can call the existing action helpers,
// which live further down in the file.
static void tx_compose_open(void);
static void auto_tcmd_open(void);
void start_tracking(state_t *state);
void stop_tracking(state_t *state);
int  point_to_stationary_target(state_t *state, double azimuth, double elevation);
// g_rx_session is referenced here in cmd_dispatch but its definition
// sits with the rest of the B210 globals further down. Forward-declare
// it so the compiler doesn't reject the references; the symbol resolves
// at link time to the static definition below.
#ifdef WITH_USRP_B210
static rx_session_t *g_rx_session;
#endif
// Forward-decl the auto-tcmd file path for the same reason — its
// definition lives next to the rest of the modal state further down,
// but cmd_dispatch needs to check it.
static char g_auto_tcmd_file_path[512];

// Dispatch the typed command. state may be touched by tracking-related
// commands; nothing else needs it. Returns nothing -- result lands in
// g_cmd_status for the next redraw.
static void cmd_dispatch(state_t *state)
{
    char buf[CMD_BUF_SIZE];
    snprintf(buf, sizeof buf, "%s", g_cmd_buf);
    // Trim leading whitespace; an empty command is a no-op.
    char *p = buf;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '\0') { cmd_set_status(""); return; }

    char *save = NULL;
    char *cmd  = strtok_r(p, " \t", &save);
    char *arg1 = strtok_r(NULL, " \t", &save);

    if (cmd == NULL) {
        cmd_set_status("");
        return;
    }
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        cmd_set_status("commands: help tx track stop home quit "
                       "freq <MHz> rs on|off spectrum <sec>");
    } else if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "q") == 0
               || strcmp(cmd, "exit") == 0) {
        state->running = 0;
        cmd_set_status("quitting");
    } else if (strcmp(cmd, "tx") == 0) {
        // Defer the modal until after we leave command-mode so the
        // bottom prompt doesn't bleed under the modal box.
        cmd_set_status("opening TX compose...");
        g_cmd_active = 0;
        tx_compose_open();
    } else if (strcmp(cmd, "auto") == 0) {
        if (g_auto_tcmd_file_path[0] == '\0') {
            cmd_set_status("auto: no --tc-file=<path> given on the cmdline");
        } else {
            cmd_set_status("opening auto-tcmd...");
            g_cmd_active = 0;
            auto_tcmd_open();
        }
    } else if (strcmp(cmd, "track") == 0) {
        start_tracking(state);
        cmd_set_status("tracking on");
    } else if (strcmp(cmd, "stop") == 0) {
        stop_tracking(state);
        cmd_set_status("tracking stopped");
    } else if (strcmp(cmd, "home") == 0) {
        stop_tracking(state);
        point_to_stationary_target(state, 0.0, 0.0);
        cmd_set_status("home: az=0 el=0");
    } else if (strcmp(cmd, "freq") == 0) {
        if (arg1 == NULL) {
            cmd_set_status("freq: missing argument (MHz)");
        } else {
#ifdef WITH_USRP_B210
            double v = atof(arg1);
            double hz = (v < 1e6) ? v * 1e6 : v;   // accept MHz or Hz
            if (hz < 1e6 || hz > 6e9) {
                cmd_set_status("freq: %g out of [1 MHz, 6 GHz]", hz);
            } else if (g_rx_session == NULL) {
                cmd_set_status("freq: no RX session");
            } else {
                rx_session_request_freq(g_rx_session, hz);
                cmd_set_status("freq -> %.6f MHz", hz / 1e6);
            }
#else
            cmd_set_status("freq: this build has no USRP support");
#endif
        }
    } else if (strcmp(cmd, "rs") == 0) {
        // Reed-Solomon toggle isn't a runtime knob yet -- rx_session
        // sets reed_solomon at open() time. Flag this clearly instead
        // of silently no-op'ing.
        if (arg1 == NULL) {
            cmd_set_status("rs: usage: rs on|off (not yet runtime-toggleable)");
        } else {
            cmd_set_status("rs %s: NOT YET WIRED -- rx_session_open params only",
                           arg1);
        }
    } else if (strcmp(cmd, "spectrum") == 0 || strcmp(cmd, "spec") == 0) {
#ifdef WITH_USRP_B210
        if (arg1 == NULL) {
            cmd_set_status("spectrum: usage `spectrum <seconds>` (1..600)");
        } else if (g_rx_session == NULL) {
            cmd_set_status("spectrum: no RX session");
        } else {
            double duration_s = atof(arg1);
            if (duration_s <= 0.0) {
                cmd_set_status("spectrum: invalid duration '%s'", arg1);
            } else {
                if (duration_s > 600.0) duration_s = 600.0;
                if (duration_s < 1.0)   duration_s = 1.0;

                spectrum_job_reap();
                if (g_spec_job.active) {
                    cmd_set_status("spectrum: a render is already in progress");
                } else {
                    char wav_path[512];
                    long n_samples = 0;
                    int  sample_rate = 0;
                    int  wav_active = 0;
                    rx_session_wav_snapshot(g_rx_session,
                                            wav_path, sizeof wav_path,
                                            &n_samples, &sample_rate, &wav_active);
                    if (wav_path[0] == '\0' || sample_rate <= 0) {
                        cmd_set_status("spectrum: no WAV (recording not started yet)");
                    } else {
                        long want  = (long)(duration_s * (double) sample_rate);
                        long start = n_samples - want;
                        if (start < 0) { start = 0; want = n_samples; }
                        if (want <= 0) {
                            cmd_set_status("spectrum: no samples captured yet");
                        } else {
                            // Filename: strip .wav and append a local-time stamp range.
                            char base[512];
                            size_t plen = strlen(wav_path);
                            if (plen >= 4 && strcmp(wav_path + plen - 4, ".wav") == 0) {
                                snprintf(base, sizeof base, "%.*s",
                                         (int)(plen - 4), wav_path);
                            } else {
                                snprintf(base, sizeof base, "%s", wav_path);
                            }
                            time_t t_end = time(NULL);
                            time_t t_start = t_end - (time_t) llround(duration_s);
                            struct tm lt_start, lt_end;
                            localtime_r(&t_start, &lt_start);
                            localtime_r(&t_end,   &lt_end);
                            char ts_start[32], ts_end[32];
                            strftime(ts_start, sizeof ts_start, "%Y-%m-%d_%H-%M-%S", &lt_start);
                            strftime(ts_end,   sizeof ts_end,   "%H-%M-%S",          &lt_end);

                            memset(&g_spec_job, 0, sizeof g_spec_job);
                            snprintf(g_spec_job.wav_in, sizeof g_spec_job.wav_in,
                                     "%s", wav_path);
                            snprintf(g_spec_job.png_out, sizeof g_spec_job.png_out,
                                     "%.480s_LOCAL_%s_to_%s.png",
                                     base, ts_start, ts_end);
                            g_spec_job.sample_rate  = sample_rate;
                            g_spec_job.start_sample = start;
                            g_spec_job.n_samples    = want;

                            // Pair the WAV slice with an IQ slice if the
                            // sidecar exists — worker prefers IQ and only
                            // falls back to the WAV+ffmpeg path when iq_in
                            // is empty.
                            char iq_path[512] = "";
                            long iq_pairs = 0;
                            int  iq_rate  = 0;
                            rx_session_iq_snapshot(g_rx_session,
                                                   iq_path, sizeof iq_path,
                                                   &iq_pairs, &iq_rate);
                            if (iq_path[0] && iq_pairs > 0 && iq_rate > 0) {
                                long want_p  = (long)(duration_s * (double) iq_rate);
                                long start_p = iq_pairs - want_p;
                                if (start_p < 0) { start_p = 0; want_p = iq_pairs; }
                                if (want_p > 0) {
                                    snprintf(g_spec_job.iq_in,
                                             sizeof g_spec_job.iq_in, "%s", iq_path);
                                    g_spec_job.iq_sample_rate = iq_rate;
                                    g_spec_job.iq_start_pair  = start_p;
                                    g_spec_job.iq_pairs       = want_p;
                                }
                            }

                            if (pthread_create(&g_spec_job.thr, NULL,
                                               spectrum_worker, &g_spec_job) != 0) {
                                cmd_set_status("spectrum: pthread_create failed: %s",
                                               strerror(errno));
                            } else {
                                g_spec_job.active = 1;
                                cmd_set_status("spectrum: rendering %.1fs (%s) -> %s",
                                               (double) want / (double) sample_rate,
                                               g_spec_job.iq_in[0] ? "iq" : "wav",
                                               g_spec_job.png_out);
                            }
                        }
                    }
                }
            }
        }
#else
        cmd_set_status("spectrum: this build has no USRP support");
#endif
    } else {
        cmd_set_status("unknown command '%s' (try :help)", cmd);
    }
}

// Mirror the operator's ":" prompt to viewers. cmd-preview carries the
// live buffer (debounced in the main loop); cmd-executed carries the
// dispatched command + the resulting status string. Both helpers no-op
// when g_ipc isn't open (e.g., --no-control).
static void cmd_broadcast_preview(void)
{
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_CMD_PREVIEW);
    snprintf(evt.from, sizeof evt.from, "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(evt.cmd_text, sizeof evt.cmd_text, "%s", g_cmd_buf);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }
}

static void cmd_broadcast_executed(const char *executed_cmd)
{
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_CMD_EXECUTED);
    snprintf(evt.from, sizeof evt.from, "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(evt.cmd_text,   sizeof evt.cmd_text,   "%s",
             executed_cmd ? executed_cmd : "");
    snprintf(evt.cmd_status, sizeof evt.cmd_status, "%s", g_cmd_status);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }
}

// Apply a single editing action to the cmd buffer. Factored so both
// the keypad-translated codepath (KEY_LEFT, KEY_RIGHT, KEY_DC, etc.)
// and the manual escape-sequence fallback (`\x1b[D` and friends) feed
// the same logic.
typedef enum {
    CMD_ACTION_NONE = 0,
    CMD_ACTION_LEFT,
    CMD_ACTION_RIGHT,
    CMD_ACTION_HOME,
    CMD_ACTION_END,
    CMD_ACTION_BACKSPACE,
    CMD_ACTION_DEL,
} cmd_action_t;

static int cmd_apply_action(cmd_action_t a)
{
    switch (a) {
        case CMD_ACTION_LEFT:
            if (g_cmd_cursor > 0) g_cmd_cursor--;
            return 1;
        case CMD_ACTION_RIGHT:
            if (g_cmd_cursor < g_cmd_len) g_cmd_cursor++;
            return 1;
        case CMD_ACTION_HOME:
            g_cmd_cursor = 0;
            return 1;
        case CMD_ACTION_END:
            g_cmd_cursor = g_cmd_len;
            return 1;
        case CMD_ACTION_BACKSPACE:
            if (g_cmd_cursor > 0) {
                memmove(&g_cmd_buf[g_cmd_cursor - 1],
                        &g_cmd_buf[g_cmd_cursor],
                        (size_t)(g_cmd_len - g_cmd_cursor + 1));
                g_cmd_len--;
                g_cmd_cursor--;
                g_cmd_dirty = 1;
                g_cmd_last_edit_ns = cmd_now_ns();
            }
            return 1;
        case CMD_ACTION_DEL:
            if (g_cmd_cursor < g_cmd_len) {
                memmove(&g_cmd_buf[g_cmd_cursor],
                        &g_cmd_buf[g_cmd_cursor + 1],
                        (size_t)(g_cmd_len - g_cmd_cursor));
                g_cmd_len--;
                g_cmd_dirty = 1;
                g_cmd_last_edit_ns = cmd_now_ns();
            }
            return 1;
        default:
            return 0;
    }
}

// When keypad/terminfo doesn't translate arrow keys (some minimal
// $TERM values, or a tmux/screen pane that strips function-key info)
// the raw escape sequence arrives byte-by-byte instead of as a single
// KEY_*. The first byte is Esc (27); if we treat that as cancel the
// rest of the sequence falls into the main keyboard switch and gets
// silently dropped. Peek for follow-on bytes; only fall through to
// the Esc-as-cancel path when nothing else is queued.
static cmd_action_t cmd_drain_csi(void)
{
    // After Esc we expect '['; otherwise it's some other sequence we
    // don't recognise and we just swallow the lookahead.
    int b1 = getch();
    if (b1 == ERR) return CMD_ACTION_NONE;
    if (b1 != '[') return CMD_ACTION_NONE;
    int b2 = getch();
    if (b2 == ERR) return CMD_ACTION_NONE;
    if (b2 == 'D') return CMD_ACTION_LEFT;
    if (b2 == 'C') return CMD_ACTION_RIGHT;
    if (b2 == 'H') return CMD_ACTION_HOME;
    if (b2 == 'F') return CMD_ACTION_END;
    // VT-style sequences: ESC [ <digits> ~ . We only care about a few.
    if (b2 >= '0' && b2 <= '9') {
        int b3 = getch();
        if (b3 == '~') {
            switch (b2) {
                case '1': return CMD_ACTION_HOME;   // some terminals
                case '3': return CMD_ACTION_DEL;
                case '4': return CMD_ACTION_END;    // some terminals
                case '7': return CMD_ACTION_HOME;
                case '8': return CMD_ACTION_END;
                default:  return CMD_ACTION_NONE;
            }
        }
    }
    return CMD_ACTION_NONE;
}

// Returns 1 if key was consumed by the command line, 0 to fall through.
// Supports left/right cursor movement, mid-line insert + delete, Home/
// End jumps, and Enter from any cursor position. The viewer's cmd_text
// wire field carries the buffer verbatim; cursor position stays local.
static int cmd_handle_key(int key, state_t *state)
{
    if (!g_cmd_active) return 0;
    if (key == ERR) return 1;
    if (key == 27 /* Esc OR start of a CSI sequence */) {
        cmd_action_t a = cmd_drain_csi();
        if (a != CMD_ACTION_NONE) {
            cmd_apply_action(a);
            return 1;
        }
        // Truly bare Esc — cancel.
        g_cmd_active = 0;
        g_cmd_status[0] = '\0';
        g_cmd_buf[0] = '\0';
        g_cmd_len = 0;
        g_cmd_cursor = 0;
        cmd_broadcast_executed("");
        g_cmd_dirty = 0;
        return 1;
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        char executed[CMD_BUF_SIZE];
        snprintf(executed, sizeof executed, "%s", g_cmd_buf);
        g_cmd_active = 0;
        cmd_dispatch(state);
        // After dispatch, g_cmd_status holds the result string. Mirror
        // both to viewers so they see exactly what the operator sees.
        cmd_broadcast_executed(executed);
        g_cmd_dirty = 0;
        return 1;
    }
    if (key == KEY_LEFT)  return cmd_apply_action(CMD_ACTION_LEFT);
    if (key == KEY_RIGHT) return cmd_apply_action(CMD_ACTION_RIGHT);
    if (key == KEY_HOME || key == 1 /* Ctrl-A */)
        return cmd_apply_action(CMD_ACTION_HOME);
    if (key == KEY_END  || key == 5 /* Ctrl-E */)
        return cmd_apply_action(CMD_ACTION_END);
    if (key == KEY_BACKSPACE || key == 127 || key == 8)
        return cmd_apply_action(CMD_ACTION_BACKSPACE);
    if (key == KEY_DC || key == 4 /* Ctrl-D */)
        return cmd_apply_action(CMD_ACTION_DEL);
    if (key >= 32 && key < 127 && g_cmd_len < (int) sizeof g_cmd_buf - 1) {
        // Insert at cursor: shift the tail right by one, drop char in.
        memmove(&g_cmd_buf[g_cmd_cursor + 1],
                &g_cmd_buf[g_cmd_cursor],
                (size_t)(g_cmd_len - g_cmd_cursor + 1));  // include nul
        g_cmd_buf[g_cmd_cursor] = (char) key;
        g_cmd_len++;
        g_cmd_cursor++;
        g_cmd_dirty = 1;
        g_cmd_last_edit_ns = cmd_now_ns();
    }
    return 1;
}

// Render the command prompt (or last-result status) on the bottom row.
// Cursor is drawn as a reverse-video block on the char at g_cmd_cursor
// — or on a trailing space when the cursor is at end-of-line. The
// surrounding text is plain so cursor position is unambiguous.
static void cmd_render(void)
{
    int row = LINES - 1;
    if (g_cmd_active) {
        move(row, 0);
        addch(':');
        for (int i = 0; i < g_cmd_len; ++i) {
            if (i == g_cmd_cursor) {
                addch(((unsigned char) g_cmd_buf[i]) | A_REVERSE);
            } else {
                addch((unsigned char) g_cmd_buf[i]);
            }
        }
        if (g_cmd_cursor == g_cmd_len) {
            addch(' ' | A_REVERSE);
        }
        clrtoeol();
        // Park the hardware cursor on the same cell the A_REVERSE
        // block highlights. The layered refresh below will curs_set(1)
        // when an editable context is active so the operator sees a
        // visible blinking cursor on top of the inverse block.
        move(row, 1 + g_cmd_cursor);
        return;
    } else if (g_cmd_status[0]) {
        mvprintw(row, 0, "%s", g_cmd_status);
    } else {
        move(row, 0);
    }
    clrtoeol();
}
// /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/ for the upcoming
// pass — created in main() once the AOS prediction is in, then
// broadcast on every STATE event so b210_rx_tx and tx_frame_sdr
// can drop their captures/logs in the same spot. Empty until set.
static char g_pass_folder[256] = "";
// SIGUSR1 sets this — used by the force-claim takeover path to nudge
// the operator-mode loop into a graceful exit. (Full in-place
// demotion is a follow-up; for now SIGUSR1 = quit.)
static volatile sig_atomic_t g_yield_requested = 0;
static void on_sigusr1(int sig) {
    (void) sig;
    g_yield_requested = 1;
}

// Loop pacing helper. With the B210 attached, the main loop runs at
// UHD-chunk cadence (~120 Hz at 240 kHz / 2040-sample chunks); slow-
// cadence work (IPC broadcast, ncurses redraw) is timestamp-gated so
// it stays at its historical 2 Hz / 10 Hz rates.
static double monotonic_seconds(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

// B210 ownership lives here now — simple_sat_ops is the single process
// that opens the SDR. --without-b210 (or a non-WITH_USRP_B210 build)
// leaves g_rx_session NULL and the loop falls through cleanly.
static int  g_without_b210 = 0;
#ifdef WITH_USRP_B210
// rx_session owns the b210 core + the worker thread. main.c only
// keeps a local handle long enough to open the device and hand it
// over.
static rx_session_t      *g_rx_session  = NULL;
static tx_request_slot_t  g_tx_request  = {0};
#endif

// CLI gate: --no-tx blocks the compose modal from actually committing
// a burst. Typing + preview broadcast still work so the operator can
// rehearse / get advice from viewers without keying the PA.
static int g_no_tx = 0;

// TX log ring buffer — last few PREVIEW/SENT/ACK events for display.
// Shared by operator and viewer renderers.
typedef struct {
    sso_event_type_t kind;     // PREVIEW | TX_COMMAND_SENT | TX_ACK
    char             ts[16];   // HH:MM:SS
    char             ascii[160];
    char             tx_ack_status[24];
} tx_log_entry_t;
#define TX_LOG_SIZE 8
static tx_log_entry_t g_tx_log[TX_LOG_SIZE];
static size_t         g_tx_log_count = 0;

// Persistent on-disk TX log. JSONL — one encoded sso_event_t per line.
// Opened lazily on the first event after g_pass_folder is set, kept
// open for the rest of the process, fflushed after every line so a
// crash mid-pass doesn't lose the last command sent. Captures every
// preview / commit / ack — the same events that drive the in-memory
// ring above and the viewer-side mirror.
static FILE *g_tx_log_fp = NULL;
static char  g_tx_log_path[512] = "";

// Pull "HH:MM:SS" out of an event's ISO ts ("2026-05-14T13:22:01.450Z").
// Falls back to local clock if the event ts is empty/garbled.
static void tx_log_ts_from_event(const sso_event_t *evt,
                                 char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    if (evt && evt->ts[0]) {
        const char *t = strchr(evt->ts, 'T');
        if (t && strlen(t) >= 9) {
            size_t n = 8;
            if (n >= out_size) n = out_size - 1;
            memcpy(out, t + 1, n);
            out[n] = '\0';
            return;
        }
    }
    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    snprintf(out, out_size, "%02d:%02d:%02d",
             lt.tm_hour, lt.tm_min, lt.tm_sec);
}

// Append one event to <pass_folder>/tx.log as a JSON line. Opens the
// file lazily; no-op when g_pass_folder isn't set yet (so events that
// arrive before pass-folder bring-up land in the in-memory ring but
// aren't dropped silently — they just don't reach disk until the
// folder exists). fflush after every write so a SIGKILL mid-pass
// preserves the last command sent.
static void tx_log_file_append(const sso_event_t *evt)
{
    if (!evt) return;
    if (g_tx_log_fp == NULL) {
        if (g_pass_folder[0] == '\0') return;
        char path[512];
        snprintf(path, sizeof path, "%.500s/tx.log", g_pass_folder);
        FILE *fp = fopen(path, "a");
        if (!fp) return;
        snprintf(g_tx_log_path, sizeof g_tx_log_path, "%s", path);
        g_tx_log_fp = fp;
    }
    char buf[2048];
    if (sso_event_encode(evt, buf, sizeof buf) != 0) return;
    // sso_event_encode already terminates with "}\n" — don't add another.
    fputs(buf, g_tx_log_fp);
    fflush(g_tx_log_fp);
}

// Push an event into the ring. PREVIEW events overwrite a trailing
// PREVIEW entry (live cursor-style update). SENT promotes a trailing
// PREVIEW to SENT, or appends a fresh entry. ACK appends with the
// status string filled in for rendering.
static void tx_log_push(const sso_event_t *evt)
{
    if (!evt) return;
    if (evt->type != SSO_EVT_TX_COMMAND_PREVIEW
     && evt->type != SSO_EVT_TX_COMMAND_SENT
     && evt->type != SSO_EVT_TX_ACK) return;

    tx_log_file_append(evt);

    tx_log_entry_t entry;
    memset(&entry, 0, sizeof entry);
    entry.kind = evt->type;
    tx_log_ts_from_event(evt, entry.ts, sizeof entry.ts);
    snprintf(entry.ascii, sizeof entry.ascii, "%s", evt->ascii);
    snprintf(entry.tx_ack_status, sizeof entry.tx_ack_status, "%s",
             evt->tx_ack_status);

    if (evt->type == SSO_EVT_TX_COMMAND_PREVIEW
        && g_tx_log_count > 0
        && g_tx_log[g_tx_log_count - 1].kind == SSO_EVT_TX_COMMAND_PREVIEW) {
        g_tx_log[g_tx_log_count - 1] = entry;
        return;
    }
    if (evt->type == SSO_EVT_TX_COMMAND_SENT
        && g_tx_log_count > 0
        && g_tx_log[g_tx_log_count - 1].kind == SSO_EVT_TX_COMMAND_PREVIEW) {
        // Promote the trailing draft in-place.
        g_tx_log[g_tx_log_count - 1] = entry;
        return;
    }
    if (g_tx_log_count < TX_LOG_SIZE) {
        g_tx_log[g_tx_log_count++] = entry;
    } else {
        memmove(&g_tx_log[0], &g_tx_log[1],
                sizeof(g_tx_log[0]) * (TX_LOG_SIZE - 1));
        g_tx_log[TX_LOG_SIZE - 1] = entry;
    }
}

// Render the TX log at rows [start_row .. start_row + (TX_LOG_SIZE+1)).
// Caller picks the column. Title line + one row per entry. Newest at
// the bottom; PREVIEW lines render with A_BOLD, SENT/ACK with A_DIM.
static void render_tx_log_panel(int start_row, int col)
{
    int row = start_row;
    mvprintw(row++, col, "TX log");
    clrtoeol();
    for (size_t i = 0; i < g_tx_log_count; ++i) {
        const tx_log_entry_t *e = &g_tx_log[i];
        const char *tag = "sent>  ";
        int attr = A_DIM;
        if (e->kind == SSO_EVT_TX_COMMAND_PREVIEW) {
            tag = "draft> ";
            attr = A_BOLD;
        } else if (e->kind == SSO_EVT_TX_ACK) {
            tag = "ack>   ";
            attr = A_DIM;
        }
        attron(attr);
        if (e->kind == SSO_EVT_TX_ACK && e->tx_ack_status[0]) {
            mvprintw(row++, col, "%s  %s %.40s  [%s]",
                     e->ts, tag, e->ascii, e->tx_ack_status);
        } else {
            mvprintw(row++, col, "%s  %s %.60s",
                     e->ts, tag, e->ascii);
        }
        clrtoeol();
        attroff(attr);
    }
}

// Latest broadcast snapshot, kept so a newly-connecting viewer gets
// state in its WELCOME response without having to wait up to 500 ms
// for the next periodic STATE broadcast.
static int    g_last_state_valid     = 0;
static char   g_last_state_sat[64]   = "";
static double g_last_state_az        = 0.0;
static double g_last_state_el        = 0.0;
static long   g_last_state_freq_hz   = 0;
static double g_last_state_doppler   = 0.0;
static char   g_last_state_tle[256]  = "";
static double g_last_state_tgt_az    = 0.0;
static double g_last_state_tgt_el    = 0.0;
static int    g_last_state_flip      = 0;
static int    g_last_state_in_pass   = 0;
static int    g_last_state_tracking  = 0;
static int    g_last_state_has_rot   = 0;
static double g_last_state_jul       = 0.0;
static char   g_last_state_idesg[9]  = "";
static double g_last_state_epoch_min   = 0.0;
static double g_last_state_min_visible = 0.0;
static double g_last_state_min_above_0 = 0.0;
static double g_last_state_min_above_30 = 0.0;
static double g_last_state_max_el      = 0.0;
static double g_last_state_pred_az     = 0.0;
static double g_last_state_pred_el     = 0.0;
static double g_last_state_alt_km      = 0.0;
static double g_last_state_lat_deg     = 0.0;
static double g_last_state_lon_deg     = 0.0;
static double g_last_state_speed_kms   = 0.0;
static double g_last_state_range_km    = 0.0;
static double g_last_state_rrate_kms   = 0.0;

// Snapshot the operator's live RX panel data into a self-contained
// struct so the same renderer can be driven by either the operator
// (reading rx_session + g_ribbon_*) or the viewer (filling it from a
// STATE event). RX_PT_COUNT comes from rx_session.h. ribbon is a
// nul-terminated string of glyph-index chars (' ' or '0'..'7'); empty
// when no samples have arrived yet.
//
// RX_PT_COUNT is gated by WITH_USRP_B210 (it lives in rx_session.h).
// On builds without B210 we still want the viewer to draw the panel
// from broadcast data, so define a fallback so the struct compiles
// everywhere.
#ifdef WITH_USRP_B210
#define RX_PANEL_PT_COUNT RX_PT_COUNT
#define RX_PANEL_PAYLOAD_MAX RX_LAST_PAYLOAD_MAX
#else
#define RX_PANEL_PT_COUNT 6
#define RX_PANEL_PAYLOAD_MAX 64
#endif

// Labels for the six RX packet-type slots, in the same order as the
// RX_PT_* enum (rx_session.h). Defined here so the viewer build —
// which doesn't link rx_session.c — can render the panel without
// pulling in the WITH_USRP_B210 codepath.
static const char *rx_panel_pt_label(int slot)
{
    static const char *labels[RX_PANEL_PT_COUNT] = {
        "beacon", "periph", "log", "tcmd", "bulk", "other",
    };
    if (slot < 0 || slot >= RX_PANEL_PT_COUNT) return "?";
    return labels[slot];
}

typedef struct {
    int        have_session;
    int        rec_active;
    double     rx_freq_hz;
    double     peak_dbfs;
    double     rms_dbfs;
    uint64_t   frames_total;
    char       last_frame_summary[80]; // "<ts>  N bytes" or empty
    double     age_s;                  // <0 = no frame yet
    uint64_t   pt_count[RX_PANEL_PT_COUNT];
    int        pt_payload_len[RX_PANEL_PT_COUNT];
    uint8_t    pt_payload[RX_PANEL_PT_COUNT][RX_PANEL_PAYLOAD_MAX];
    int        ribbon_n;
    char       ribbon[RIBBON_LEN + 1];
    // Parallel array: peak dBFS for the i-th second back. Clamped into
    // int8 (dBFS is naturally -90..0, well inside int8's range).
    int8_t     ribbon_peak[RIBBON_LEN];
    // Shadow IQ-demod frame count (modem_iq.c). Reported alongside
    // frames_total so the operator can A/B the IQ vs FM-audio chains.
    uint64_t   frames_iq;
    // Shadow Viterbi-MLSE frame count (modem_viterbi.c). Third A/B
    // counter on the same IQ window.
    uint64_t   frames_vit;
    // Optional warning row (e.g., low-disk). Empty when no warning.
    char       warning[80];
} rx_panel_data_t;

// Operator-side collector. Reads the live rx_session + g_ribbon globals
// into the struct. On non-B210 builds, only have_session=0 is filled.
static void rx_panel_collect_local(rx_panel_data_t *d)
{
    memset(d, 0, sizeof *d);
#ifdef WITH_USRP_B210
    d->have_session = (g_rx_session != NULL);
    if (!d->have_session) return;
    d->rec_active = rx_session_wav_active(g_rx_session);
    char last[sizeof d->last_frame_summary] = "";
    rx_session_snapshot(g_rx_session,
                        &d->frames_total,
                        &d->peak_dbfs,
                        &d->rms_dbfs,
                        &d->rx_freq_hz,
                        last, sizeof last);
    snprintf(d->last_frame_summary, sizeof d->last_frame_summary,
             "%s", last);
    d->frames_iq  = rx_session_iq_frames(g_rx_session);
    d->frames_vit = rx_session_viterbi_frames(g_rx_session);
    rx_packet_type_stats_t pts[RX_PT_COUNT];
    rx_session_stats_snapshot(g_rx_session, pts, &d->age_s);
    for (int s = 0; s < RX_PT_COUNT; ++s) {
        d->pt_count[s]       = pts[s].count;
        d->pt_payload_len[s] = pts[s].last_payload_len;
        int copy = pts[s].last_payload_len;
        if (copy < 0) copy = 0;
        if (copy > RX_PANEL_PAYLOAD_MAX) copy = RX_PANEL_PAYLOAD_MAX;
        memcpy(d->pt_payload[s], pts[s].last_payload, (size_t) copy);
    }
    // Wire format: ribbon[0] is the newest sample, ribbon[ribbon_n-1]
    // is the oldest. Each char is '.' for a normal second or '-' for
    // a 20 s tick. Tick test uses absolute push count (P) rather than
    // a fixed visual position so the tick row visibly walks upward at
    // 1 row/s; without that the strip looked frozen once it filled.
    // ribbon_peak[i] is the peak dBFS that was pushed at that same
    // second, clamped into int8 — rendered alongside the marker so
    // the operator sees the signal level on every row.
    int n = g_ribbon_count;
    if (n > (int) sizeof d->ribbon - 1) n = (int) sizeof d->ribbon - 1;
    long P = g_ribbon_push_count;
    for (int i = 0; i < n; ++i) {
        long abs_t = P - (long) i;
        if (abs_t > 0 && (abs_t % 20) == 0) d->ribbon[i] = '-';
        else                                 d->ribbon[i] = '.';
        int idx = (g_ribbon_head - 1 - i + RIBBON_LEN) % RIBBON_LEN;
        double dbfs = g_ribbon_peak[idx];
        long lr = lround(dbfs);
        if (lr > 127)  lr = 127;
        if (lr < -127) lr = -127;
        d->ribbon_peak[i] = (int8_t) lr;
    }
    d->ribbon[n] = '\0';
    d->ribbon_n  = n;
#endif
    // Warning row is filled regardless of the B210 build — even without
    // an SDR the operator could be running short on disk for logs.
    snprintf(d->warning, sizeof d->warning, "%s", g_low_disk_msg);
}

// Render the RX panel from a snapshot. Compiles even without UHD —
// the viewer feeds this from broadcast STATE so it can draw what the
// operator sees.
static void render_rx_panel(const rx_panel_data_t *d,
                            int *print_row, int print_col)
{
    if (print_row == NULL || d == NULL) return;
    int row = *print_row;
    int col = print_col;
    mvprintw(row++, col, "%15s   %s%s", "B210",
             d->have_session ? "active" : "(offline)",
             d->rec_active ? "  [REC]" : "");
    clrtoeol();
    if (d->warning[0]) {
        // Red attribute pair 1 was initialised in init_window; fall
        // back gracefully if colors aren't available.
        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(row++, col, "%15s   %s", "WARNING", d->warning);
        attroff(COLOR_PAIR(1) | A_BOLD);
        clrtoeol();
    }
    if (!d->have_session) {
        *print_row = row;
        return;
    }
    mvprintw(row++, col, "%15s   %.6f MHz", "RX freq", d->rx_freq_hz / 1e6);
    clrtoeol();
    mvprintw(row++, col, "%15s   peak %+5.1f  rms %+5.1f dBFS",
             "level", d->peak_dbfs, d->rms_dbfs);
    clrtoeol();
    mvprintw(row++, col, "%15s   %llu  (iq=%llu vit=%llu)", "frames",
             (unsigned long long) d->frames_total,
             (unsigned long long) d->frames_iq,
             (unsigned long long) d->frames_vit);
    clrtoeol();
    if (d->last_frame_summary[0]) {
        mvprintw(row++, col, "%15s   %s", "last frame", d->last_frame_summary);
        clrtoeol();
    }
    if (d->age_s >= 0.0) {
        mvprintw(row++, col, "%15s   %.1f s ago", "last frame T+", d->age_s);
        clrtoeol();
    }
    char by_type[160] = {0};
    size_t bt_len = 0;
    for (int s = 0; s < RX_PANEL_PT_COUNT; ++s) {
        int n = snprintf(by_type + bt_len, sizeof by_type - bt_len,
                         "%s%s=%llu",
                         (bt_len ? "  " : ""),
                         rx_panel_pt_label(s),
                         (unsigned long long) d->pt_count[s]);
        if (n < 0 || (size_t) n >= sizeof by_type - bt_len) break;
        bt_len += (size_t) n;
    }
    mvprintw(row++, col, "%15s   %s", "by type", by_type);
    clrtoeol();
    for (int s = 0; s < RX_PANEL_PT_COUNT; ++s) {
        if (d->pt_count[s] == 0 || d->pt_payload_len[s] <= 0) continue;
        char hexbuf[3 * 24 + 16];
        size_t hex_len = 0;
        int n_show = d->pt_payload_len[s];
        if (n_show > 24) n_show = 24;
        for (int b = 0; b < n_show; ++b) {
            int w = snprintf(hexbuf + hex_len, sizeof hexbuf - hex_len,
                             "%s%02X",
                             (b ? " " : ""),
                             d->pt_payload[s][b]);
            if (w < 0 || (size_t) w >= sizeof hexbuf - hex_len) break;
            hex_len += (size_t) w;
        }
        mvprintw(row++, col, "%15s   %s%s", rx_panel_pt_label(s), hexbuf,
                 d->pt_payload_len[s] > n_show ? " ..." : "");
        clrtoeol();
    }
    // No in-panel ribbon: the timeline lives in its own vertical strip
    // on the right of the screen (render_ribbon_vertical), so the panel
    // body never wraps onto a second line.
    *print_row = row;
}

// Vertical ribbon strip. Bottom row (`bot_row`) holds the newest
// sample; older samples climb upward toward `top_row`. Each row is
// rendered as "%3d <marker>" — right-aligned peak dBFS followed by
// space + '.' or '-'. '-' ticks render bold. Rows past the available
// data are blanked so the strip stays a clean column.
//
// `col` is the column of the marker char. The integer occupies the 4
// columns immediately to its left (col-4 .. col-1).
static void render_ribbon_vertical(const rx_panel_data_t *d,
                                   int top_row, int bot_row, int col)
{
    if (d == NULL || bot_row <= top_row) return;
    if (col < 4) return;
    int int_col = col - 4;
    int max_rows = bot_row - top_row + 1;
    for (int i = 0; i < max_rows; ++i) {
        int row = bot_row - i;
        if (i < d->ribbon_n) {
            int peak = (int) d->ribbon_peak[i];
            mvprintw(row, int_col, "%3d ", peak);
            char c = d->ribbon[i];
            if (c == '\0') c = ' ';
            if (c == '-') {
                attron(A_BOLD);
                mvaddch(row, col, c);
                attroff(A_BOLD);
            } else {
                mvaddch(row, col, c);
            }
        } else {
            // No data yet for this row — leave it blank.
            mvprintw(row, int_col, "    ");
            mvaddch(row, col, ' ');
        }
    }
}

// Snapshot the operator's RX panel into the wire-side fields of an
// event. Called for both STATE broadcasts and WELCOME replies so a
// newly-connecting viewer sees the same panel state everyone else does.
static void ipc_fill_rx_panel(sso_event_t *evt)
{
    rx_panel_data_t d;
    rx_panel_collect_local(&d);
    evt->rx_have_session = d.have_session;
    // Warning row is operator-wide (e.g. low disk), not gated on the
    // SDR — fill it before the have_session early-return.
    snprintf(evt->rx_warning, sizeof evt->rx_warning, "%s", d.warning);
    if (!d.have_session) return;
    evt->rx_rec_active   = d.rec_active;
    evt->rx_freq_hz      = d.rx_freq_hz;
    evt->rx_peak_dbfs    = d.peak_dbfs;
    evt->rx_rms_dbfs     = d.rms_dbfs;
    evt->rx_frames_total = (long) d.frames_total;
    evt->rx_frames_iq    = (long) d.frames_iq;
    evt->rx_frames_vit   = (long) d.frames_vit;
    snprintf(evt->rx_last_frame_summary,
             sizeof evt->rx_last_frame_summary, "%s", d.last_frame_summary);
    evt->rx_age_s = d.age_s;
    int slots = RX_PANEL_PT_COUNT < SSO_RX_PT_SLOTS
              ? RX_PANEL_PT_COUNT : SSO_RX_PT_SLOTS;
    for (int s = 0; s < slots; ++s) {
        evt->rx_pt_count[s]       = (long) d.pt_count[s];
        int pl = d.pt_payload_len[s];
        if (pl < 0) pl = 0;
        int wire_pl = pl;
        if (wire_pl > SSO_RX_PT_PAYLOAD_MAX) wire_pl = SSO_RX_PT_PAYLOAD_MAX;
        evt->rx_pt_payload_len[s] = pl;
        memcpy(evt->rx_pt_payload[s], d.pt_payload[s], (size_t) wire_pl);
    }
    int rn = d.ribbon_n;
    if (rn > SSO_RIBBON_MAX) rn = SSO_RIBBON_MAX;
    evt->rx_ribbon_n = rn;
    memcpy(evt->rx_ribbon, d.ribbon, (size_t) rn);
    evt->rx_ribbon[rn] = '\0';
    memcpy(evt->rx_ribbon_peak, d.ribbon_peak,
           (size_t) rn * sizeof evt->rx_ribbon_peak[0]);
}

static void ipc_broadcast_state(state_t *s,
                                  double az, double el,
                                  double downlink_freq,
                                  double doppler_delta_dl,
                                  double jul_utc) {
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_STATE);
    snprintf(evt.from, sizeof(evt.from), "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(evt.operator_user, sizeof(evt.operator_user), "%s",
             g_operator_user ? g_operator_user : "?");
    evt.has_state = 1;
    if (s->prediction.satellite_ephem.name) {
        snprintf(evt.satellite, sizeof(evt.satellite), "%s",
                 s->prediction.satellite_ephem.name);
    }
    evt.az = az;
    evt.el = el;
    evt.freq_hz = (long) downlink_freq;
    evt.doppler_hz = doppler_delta_dl;
    if (g_pass_folder[0]) {
        snprintf(evt.pass_folder, sizeof(evt.pass_folder), "%s",
                 g_pass_folder);
    }
    if (s->prediction.tles_filename) {
        snprintf(evt.tle_path, sizeof(evt.tle_path), "%s",
                 s->prediction.tles_filename);
    }
    evt.target_az = s->antenna_rotator.target_azimuth;
    evt.target_el = s->antenna_rotator.target_elevation;
    evt.flip      = s->antenna_rotator.flip_mode_pass;
    evt.in_pass   = s->in_pass;
    evt.tracking  = s->antenna_rotator.tracking;
    evt.has_rotator = s->have_antenna_rotator;
    evt.jul_utc   = jul_utc;

    // Prediction snapshot — viewer renders these verbatim.
    snprintf(evt.idesg, sizeof evt.idesg, "%s",
             s->prediction.satellite_ephem.tle.idesg);
    evt.epoch_min      = s->prediction.minutes_since_epoch;
    evt.min_visible    = s->prediction.predicted_minutes_until_visible;
    evt.min_above_0    = s->prediction.predicted_minutes_above_0_degrees;
    evt.min_above_30   = s->prediction.predicted_minutes_above_30_degrees;
    evt.max_el         = s->prediction.predicted_max_elevation;
    evt.pred_az        = s->prediction.satellite_ephem.azimuth;
    evt.pred_el        = s->prediction.satellite_ephem.elevation;
    evt.alt_km         = s->prediction.satellite_ephem.altitude_km;
    evt.lat_deg        = s->prediction.satellite_ephem.latitude;
    evt.lon_deg        = s->prediction.satellite_ephem.longitude;
    evt.speed_kms      = s->prediction.satellite_ephem.speed_km_s;
    evt.range_km       = s->prediction.satellite_ephem.range_km;
    evt.range_rate_kms = s->prediction.satellite_ephem.range_rate_km_s;
    sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
    size_t n = 0;
    if (n < sizeof(entries) / sizeof(entries[0])) {
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 g_operator_user ? g_operator_user : "?");
        snprintf(entries[n].role, sizeof(entries[n].role), "operator");
        entries[n].since[0] = '\0';
        n++;
    }
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char user[64], role[16], since[40];
    while (n < sizeof(entries) / sizeof(entries[0])
           && sso_ipc_server_next_client(g_ipc, &it, &cid,
                                          user, sizeof(user),
                                          role, sizeof(role),
                                          since, sizeof(since)) == 0) {
        if (!user[0]) continue;
        snprintf(entries[n].user, sizeof(entries[n].user), "%s", user);
        snprintf(entries[n].role, sizeof(entries[n].role), "%s",
                 role[0] ? role : "viewer");
        snprintf(entries[n].since, sizeof(entries[n].since), "%s", since);
        n++;
    }
    sso_event_set_roster(&evt, entries, n);
    ipc_fill_rx_panel(&evt);
    char buf[4096];
    if (sso_event_encode(&evt, buf, sizeof(buf)) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }

    // Cache for WELCOME replies so a viewer doesn't have to wait for
    // the next periodic broadcast to see anything.
    snprintf(g_last_state_sat, sizeof g_last_state_sat, "%s", evt.satellite);
    snprintf(g_last_state_tle, sizeof g_last_state_tle, "%s", evt.tle_path);
    g_last_state_az      = evt.az;
    g_last_state_el      = evt.el;
    g_last_state_freq_hz = evt.freq_hz;
    g_last_state_doppler = evt.doppler_hz;
    g_last_state_tgt_az  = evt.target_az;
    g_last_state_tgt_el  = evt.target_el;
    g_last_state_flip    = evt.flip;
    g_last_state_in_pass = evt.in_pass;
    g_last_state_tracking= evt.tracking;
    g_last_state_has_rot = evt.has_rotator;
    g_last_state_jul     = evt.jul_utc;
    snprintf(g_last_state_idesg, sizeof g_last_state_idesg, "%s", evt.idesg);
    g_last_state_epoch_min    = evt.epoch_min;
    g_last_state_min_visible  = evt.min_visible;
    g_last_state_min_above_0  = evt.min_above_0;
    g_last_state_min_above_30 = evt.min_above_30;
    g_last_state_max_el       = evt.max_el;
    g_last_state_pred_az      = evt.pred_az;
    g_last_state_pred_el      = evt.pred_el;
    g_last_state_alt_km       = evt.alt_km;
    g_last_state_lat_deg      = evt.lat_deg;
    g_last_state_lon_deg      = evt.lon_deg;
    g_last_state_speed_kms    = evt.speed_kms;
    g_last_state_range_km     = evt.range_km;
    g_last_state_rrate_kms    = evt.range_rate_kms;
    g_last_state_valid   = 1;
}

static void ipc_on_event(sso_ipc_server_t *srv, sso_client_id_t id,
                         const sso_event_t *evt, void *user) {
    (void) user;
    if (evt->type != SSO_EVT_HELLO) return;
    sso_event_t welcome;
    sso_event_init(&welcome, SSO_EVT_WELCOME);
    snprintf(welcome.from, sizeof(welcome.from), "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(welcome.operator_user, sizeof(welcome.operator_user), "%s",
             g_operator_user ? g_operator_user : "?");
    if (g_pass_folder[0]) {
        snprintf(welcome.pass_folder, sizeof(welcome.pass_folder), "%s",
                 g_pass_folder);
    }
    if (g_last_state_valid) {
        welcome.has_state   = 1;
        snprintf(welcome.satellite, sizeof welcome.satellite,
                 "%s", g_last_state_sat);
        snprintf(welcome.tle_path, sizeof welcome.tle_path,
                 "%s", g_last_state_tle);
        welcome.az          = g_last_state_az;
        welcome.el          = g_last_state_el;
        welcome.freq_hz     = g_last_state_freq_hz;
        welcome.doppler_hz  = g_last_state_doppler;
        welcome.target_az   = g_last_state_tgt_az;
        welcome.target_el   = g_last_state_tgt_el;
        welcome.flip        = g_last_state_flip;
        welcome.in_pass     = g_last_state_in_pass;
        welcome.tracking    = g_last_state_tracking;
        welcome.has_rotator = g_last_state_has_rot;
        welcome.jul_utc     = g_last_state_jul;
        snprintf(welcome.idesg, sizeof welcome.idesg, "%s", g_last_state_idesg);
        welcome.epoch_min      = g_last_state_epoch_min;
        welcome.min_visible    = g_last_state_min_visible;
        welcome.min_above_0    = g_last_state_min_above_0;
        welcome.min_above_30   = g_last_state_min_above_30;
        welcome.max_el         = g_last_state_max_el;
        welcome.pred_az        = g_last_state_pred_az;
        welcome.pred_el        = g_last_state_pred_el;
        welcome.alt_km         = g_last_state_alt_km;
        welcome.lat_deg        = g_last_state_lat_deg;
        welcome.lon_deg        = g_last_state_lon_deg;
        welcome.speed_kms      = g_last_state_speed_kms;
        welcome.range_km       = g_last_state_range_km;
        welcome.range_rate_kms = g_last_state_rrate_kms;
        // Roster — operator first, then the existing clients we know
        // of. The newly-connecting client is already in the slot table
        // (slot_dispatch_line ran first) but its role isn't populated
        // until HELLO is processed; that's why we iterate via
        // sso_ipc_server_next_client, which surfaces what HELLO set.
        sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
        size_t n = 0;
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 g_operator_user ? g_operator_user : "?");
        snprintf(entries[n].role, sizeof(entries[n].role), "operator");
        entries[n].since[0] = '\0';
        n++;
        sso_ipc_iter_t it = {0};
        sso_client_id_t cid;
        char ruser[64], rrole[16], rsince[40];
        while (n < sizeof(entries) / sizeof(entries[0])
               && sso_ipc_server_next_client(srv, &it, &cid,
                                              ruser, sizeof(ruser),
                                              rrole, sizeof(rrole),
                                              rsince, sizeof(rsince)) == 0) {
            if (!ruser[0]) continue;
            snprintf(entries[n].user,  sizeof(entries[n].user),  "%s", ruser);
            snprintf(entries[n].role,  sizeof(entries[n].role),  "%s",
                     rrole[0] ? rrole : "viewer");
            snprintf(entries[n].since, sizeof(entries[n].since), "%s", rsince);
            n++;
        }
        sso_event_set_roster(&welcome, entries, n);
        ipc_fill_rx_panel(&welcome);
    }
    char buf[4096];
    if (sso_event_encode(&welcome, buf, sizeof(buf)) == 0) {
        sso_ipc_server_send(srv, id, buf);
    }
}

// --- TX compose modal (operator side) -----------------------------
//
// Opened with `t` from the operator's main UI when the keyboard is
// unlocked. As the operator edits a field the modal broadcasts a
// debounced SSO_EVT_TX_COMMAND_PREVIEW so viewers see the draft and
// can call out typos. On Enter the modal stashes the parsed request in
// g_tx_request; the main loop runs it inline via tx_burst_run (no IPC
// round-trip, since the B210 now lives in this process). ACK +
// COMMAND_SENT events are published locally for viewer fan-out.

// Compose modal is intentionally minimal: a single payload line
// (always ASCII, prefilled with "CTS1+"), a TX-power-in-dB field,
// and the --allow-tx checkbox. CSP src/dst/dport/sport/prio, freq,
// repeat/gap, and the secondary allow-flags are hard-coded to the
// FrontierSat defaults inside tx_compose_fill_event.
typedef enum {
    TXF_PAYLOAD = 0,
    TXF_POWER,
    TXF_ALLOW_TX,
    TXF_COUNT,
} tx_field_t;

typedef struct {
    char payload[160];
    char power[12];           // TX power in dB
    int  allow_tx;
    tx_field_t focus;
    int  preview_dirty;
    struct timespec last_edit;
    char status_msg[160];
    // Per-field text cursor (only meaningful for the text fields —
    // payload, power). 0..strlen(buf). Bumped/clamped by every edit
    // helper below.
    int  cursors[TXF_COUNT];
    // Payload-history navigation state. history_idx == -1 means
    // "editing the current draft" (the live payload buffer); 0..N-1
    // points at g_tx_history[i] (newest at 0). When stepping into
    // history we stash the live draft into history_saved_edit so
    // DOWN can restore it.
    int  history_idx;
    char history_saved_edit[160];
} tx_compose_t;

// Survives Esc / commit so the operator can reopen and pick up the
// previous typed string. First open seeds it with "CTS1+" — the OBC's
// CTS1 telecommand prefix.
static char g_tx_last_payload[160] = "CTS1+";
static char g_tx_last_power[12]    = "30.0";
// Same idea for the --allow-tx checkbox: operators commonly send a
// series of commands during a pass and would rather not re-arm the
// safety gate between every one. Survives Esc + commit; cleared by
// process exit. Per-session, intentionally not persisted on disk.
static int  g_tx_last_allow_tx     = 0;

// Payload-only history ring (newest at index 0). Push happens on a
// successful commit; Esc-cancelled drafts don't enter history.
#define TX_HISTORY_MAX 32
static char g_tx_history[TX_HISTORY_MAX][160];
static int  g_tx_history_count = 0;

// Non-blocking modal state. The TX compose modal used to run a
// dedicated event loop inside run_tx_compose(), which froze the
// main loop's antenna control, screen redraws, and viewer broadcast
// for as long as the operator had the modal open. State now lives at
// file scope; the main loop ticks the modal alongside everything
// else, so tracking + rotator commands + IPC fanout keep flowing
// during composition. The modal window is drawn on top via a layered
// refresh helper.
static int           g_tx_compose_active        = 0;
static WINDOW       *g_tx_compose_win           = NULL;
static tx_compose_t  g_tx_compose_state;
static long          g_tx_compose_last_edit_ns  = 0;
static const long    g_tx_compose_debounce_ns   = 200000000L;

// --- Auto-TCMD modal ----------------------------------------------
//
// Drives a file of ASCII telecommands through the TX path automatically.
// Loaded via --tc-file=<path>; opened with 'A' (or `:auto`) from the
// operator UI. Each non-blank, non-comment line in the file is one
// CTS1+ telecommand. The operator picks TX power, how many times each
// command should be sent, and the inter-send delay. Once started the
// modal's tick runs alongside the main loop — non-blocking, like the
// TX compose modal — and queues one g_tx_request per shot, advancing
// through the file. Stops automatically when the satellite drops
// below the horizon (LOS) so an unattended run can't keep TXing after
// the pass. Every send goes through emit_tx_event_local, so the
// existing tx.log + viewer fanout capture all of them.
typedef enum {
    AUTO_F_POWER = 0,
    AUTO_F_REPEATS,
    AUTO_F_DELAY,
    AUTO_F_ALLOW_TX,
    AUTO_F_COUNT,
} auto_tcmd_field_t;

typedef enum {
    AUTO_STATE_SETUP = 0,
    AUTO_STATE_RUNNING,
    AUTO_STATE_STOPPED,    // user stopped, file not exhausted
    AUTO_STATE_DONE,       // file exhausted
    AUTO_STATE_PASS_OVER,  // LOS hit while running
} auto_tcmd_state_t;

typedef struct {
    // Commands loaded from --tc-file. commands[i] is one CTS1+ line,
    // trimmed of leading/trailing whitespace; comment lines (#...) and
    // blank lines are dropped at load.
    char **commands;
    int    n_commands;
    char   file_path[256];

    // Editable fields (text-edit semantics shared with TX compose).
    char power[12];
    char repeats[8];
    char delay_s[12];
    int  allow_tx;
    auto_tcmd_field_t focus;
    int               cursors[AUTO_F_COUNT];

    // Run state.
    auto_tcmd_state_t state;
    int    cmd_idx;        // index into commands[]
    int    repeat_idx;     // how many sends of commands[cmd_idx] so far
    int    repeats_total;  // parsed from repeats at start
    double delay_s_val;    // parsed from delay_s at start
    long   next_send_ns;
    int    sends_total;    // running tally — every queued burst
    char   last_sent[160];
    char   status_msg[160];
} auto_tcmd_t;

static int          g_auto_tcmd_active           = 0;
static WINDOW      *g_auto_tcmd_win              = NULL;
static auto_tcmd_t  g_auto_tcmd;
// Path captured from --tc-file. The modal reads this lazily so the
// CLI can be parsed before all the modal infrastructure is up.
static char         g_auto_tcmd_file_path[512]   = "";

// Trim leading and trailing whitespace in place; returns the (possibly
// advanced) start pointer. Used by the auto-tcmd file loader so the
// stored commands are clean for the wire.
static char *str_trim_inplace(char *s) {
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t'
                     || s[n - 1] == '\r' || s[n - 1] == '\n')) {
        s[--n] = '\0';
    }
    return s;
}

// Read commands from path; one per line. Comments (#...) and blank
// lines after trim are dropped. Returns 0 on success; allocates and
// stores in *out_commands / *out_n on success. Caller owns the
// allocation.
static int auto_tcmd_load_file(const char *path,
                               char ***out_commands, int *out_n)
{
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;
    int cap = 16, n = 0;
    char **arr = (char **) malloc((size_t) cap * sizeof(char *));
    if (!arr) { fclose(fp); return -1; }
    char line[512];
    while (fgets(line, sizeof line, fp)) {
        char *t = str_trim_inplace(line);
        if (t[0] == '\0' || t[0] == '#') continue;
        if (n == cap) {
            int new_cap = cap * 2;
            char **new_arr = (char **) realloc(arr,
                (size_t) new_cap * sizeof(char *));
            if (!new_arr) {
                for (int i = 0; i < n; ++i) free(arr[i]);
                free(arr); fclose(fp); return -1;
            }
            arr = new_arr; cap = new_cap;
        }
        arr[n] = strdup(t);
        if (!arr[n]) {
            for (int i = 0; i < n; ++i) free(arr[i]);
            free(arr); fclose(fp); return -1;
        }
        n++;
    }
    fclose(fp);
    *out_commands = arr;
    *out_n        = n;
    return 0;
}

static void auto_tcmd_free_commands(char **commands, int n) {
    if (!commands) return;
    for (int i = 0; i < n; ++i) free(commands[i]);
    free(commands);
}

static void tx_history_push(const char *payload) {
    if (payload == NULL || payload[0] == '\0') return;
    if (g_tx_history_count > 0
        && strcmp(g_tx_history[0], payload) == 0) {
        return;  // suppress trivial duplicates of the most-recent entry
    }
    int keep = g_tx_history_count < TX_HISTORY_MAX - 1
             ? g_tx_history_count : TX_HISTORY_MAX - 1;
    for (int i = keep; i > 0; --i) {
        memcpy(g_tx_history[i], g_tx_history[i - 1], sizeof g_tx_history[0]);
    }
    snprintf(g_tx_history[0], sizeof g_tx_history[0], "%s", payload);
    if (g_tx_history_count < TX_HISTORY_MAX) g_tx_history_count++;
}

static void tx_compose_init(tx_compose_t *c) {
    memset(c, 0, sizeof *c);
    snprintf(c->payload, sizeof c->payload, "%s", g_tx_last_payload);
    snprintf(c->power,   sizeof c->power,   "%s", g_tx_last_power);
    c->allow_tx                = g_tx_last_allow_tx;
    c->cursors[TXF_PAYLOAD]    = (int) strlen(c->payload);
    c->cursors[TXF_POWER]      = (int) strlen(c->power);
    c->history_idx             = -1;
    snprintf(c->status_msg, sizeof c->status_msg,
             "edit; viewers see drafts ~200 ms after you stop typing");
}

static void tx_compose_remember(const tx_compose_t *c) {
    snprintf(g_tx_last_payload, sizeof g_tx_last_payload, "%s", c->payload);
    snprintf(g_tx_last_power,   sizeof g_tx_last_power,   "%s", c->power);
    g_tx_last_allow_tx = c->allow_tx;
}

static void tx_compose_summary(const tx_compose_t *c, char *out, size_t out_size) {
    if (out_size == 0) return;
    snprintf(out, out_size, "%s", c->payload[0] ? c->payload : "(empty)");
}

static void tx_compose_fill_event(const tx_compose_t *c, sso_event_t *evt) {
    snprintf(evt->tx_payload_kind, sizeof evt->tx_payload_kind, "ascii");
    snprintf(evt->tx_payload, sizeof evt->tx_payload, "%s", c->payload);
    // CSP defaults match cts_send -> FrontierSat OBC (CTS1 cmd handler).
    evt->tx_csp_src   = 10;
    evt->tx_csp_dst   = 1;
    evt->tx_csp_dport = 7;
    evt->tx_csp_sport = 16;
    evt->tx_csp_prio  = 2;
    evt->tx_freq_hz   = (long) FRONTIERSAT_CARRIER_HZ;
    evt->tx_gain_db   = atof(c->power);
    evt->tx_repeat    = 1;
    evt->tx_gap_ms    = 200;
    evt->tx_allow_tx         = c->allow_tx;
    evt->tx_allow_high_power = 0;
    evt->tx_allow_hf_tx      = 0;
    char summary[160];
    tx_compose_summary(c, summary, sizeof summary);
    snprintf(evt->ascii, sizeof evt->ascii, "%s", summary);
    snprintf(evt->from, sizeof evt->from, "%s",
             g_operator_user ? g_operator_user : "?");
}

static int tx_field_is_text(tx_field_t f) { return f == TXF_PAYLOAD; }
static int tx_field_is_decimal(tx_field_t f) { return f == TXF_POWER; }
static int tx_field_is_toggle(tx_field_t f) { return f == TXF_ALLOW_TX; }

static char *tx_field_buf(tx_compose_t *c, tx_field_t f, size_t *cap) {
    switch (f) {
        case TXF_PAYLOAD: *cap = sizeof c->payload; return c->payload;
        case TXF_POWER:   *cap = sizeof c->power;   return c->power;
        default:          *cap = 0; return NULL;
    }
}

// Clamp the per-field cursor into [0, strlen(buf)]. Called after any
// op that might leave the cursor past the end (focus change, history
// recall) so subsequent insert/delete don't run off the buffer.
static void tx_field_clamp_cursor(tx_compose_t *c, tx_field_t f) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, f, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (c->cursors[f] < 0) c->cursors[f] = 0;
    if (c->cursors[f] > n) c->cursors[f] = n;
}

static void tx_field_insert(tx_compose_t *c, int ch) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (n + 1 >= (int) cap) return;
    int accept = 0;
    if (tx_field_is_text(c->focus)) {
        accept = (ch >= 32 && ch < 127);
    } else if (tx_field_is_decimal(c->focus)) {
        accept = (ch >= '0' && ch <= '9') || ch == '.' || ch == '-';
    }
    if (!accept) return;
    int cur = c->cursors[c->focus];
    if (cur < 0) cur = 0;
    if (cur > n) cur = n;
    // Shift the tail (including the existing nul) right by one.
    memmove(buf + cur + 1, buf + cur, (size_t)(n - cur + 1));
    buf[cur] = (char) ch;
    c->cursors[c->focus] = cur + 1;
    c->preview_dirty = 1;
    // Any edit cancels history navigation — the operator is now off
    // the recalled string, the next UP should walk history fresh.
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_backspace(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = c->cursors[c->focus];
    if (cur <= 0 || n == 0) return;
    memmove(buf + cur - 1, buf + cur, (size_t)(n - cur + 1));
    c->cursors[c->focus] = cur - 1;
    c->preview_dirty = 1;
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_delete(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = c->cursors[c->focus];
    if (cur >= n) return;
    memmove(buf + cur, buf + cur + 1, (size_t)(n - cur));
    c->preview_dirty = 1;
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_kill_to_end(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = c->cursors[c->focus];
    if (cur >= n) return;
    buf[cur] = '\0';
    c->preview_dirty = 1;
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_left(tx_compose_t *c) {
    size_t cap = 0;
    if (!tx_field_buf(c, c->focus, &cap)) return;
    if (c->cursors[c->focus] > 0) c->cursors[c->focus]--;
}

static void tx_field_right(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (c->cursors[c->focus] < n) c->cursors[c->focus]++;
}

static void tx_field_home(tx_compose_t *c) {
    size_t cap = 0;
    if (!tx_field_buf(c, c->focus, &cap)) return;
    c->cursors[c->focus] = 0;
}

static void tx_field_end(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    c->cursors[c->focus] = (int) strlen(buf);
}

// direction = -1 (UP, older) or +1 (DOWN, newer). No-op when focus
// is not on the payload field, when history is empty, or at the edge.
static void tx_history_recall(tx_compose_t *c, int direction) {
    if (c->focus != TXF_PAYLOAD) return;
    if (g_tx_history_count == 0) return;
    int step = (direction < 0) ? +1 : -1;
    int new_idx = c->history_idx + step;
    if (new_idx < -1) return;
    if (new_idx >= g_tx_history_count) return;
    if (c->history_idx == -1 && new_idx >= 0) {
        snprintf(c->history_saved_edit, sizeof c->history_saved_edit,
                 "%s", c->payload);
    }
    if (new_idx == -1) {
        snprintf(c->payload, sizeof c->payload, "%s",
                 c->history_saved_edit);
    } else {
        snprintf(c->payload, sizeof c->payload, "%s",
                 g_tx_history[new_idx]);
    }
    c->cursors[TXF_PAYLOAD] = (int) strlen(c->payload);
    c->history_idx          = new_idx;
    c->preview_dirty        = 1;
}

// Tiny CSI fallback parser for terminals where ncurses' keypad mode
// doesn't translate arrow / nav keys into KEY_* (notably some tmux
// configurations). Same idea as cmd_drain_csi in the `:` prompt.
// Returns a KEY_* code on success, or -1 if the lookahead isn't a
// CSI we recognise.
static int tx_drain_csi(WINDOW *w) {
    int b1 = wgetch(w);
    if (b1 == ERR || b1 != '[') return -1;
    int b2 = wgetch(w);
    if (b2 == ERR) return -1;
    switch (b2) {
        case 'A': return KEY_UP;
        case 'B': return KEY_DOWN;
        case 'C': return KEY_RIGHT;
        case 'D': return KEY_LEFT;
        case 'H': return KEY_HOME;
        case 'F': return KEY_END;
        default: break;
    }
    if (b2 >= '0' && b2 <= '9') {
        int b3 = wgetch(w);
        if (b3 == '~') {
            switch (b2) {
                case '1': return KEY_HOME;
                case '3': return KEY_DC;
                case '4': return KEY_END;
                case '7': return KEY_HOME;
                case '8': return KEY_END;
                default: break;
            }
        }
    }
    return -1;
}

static void tx_field_toggle(tx_compose_t *c) {
    if (c->focus == TXF_ALLOW_TX) {
        c->allow_tx = !c->allow_tx;
        c->preview_dirty = 1;
    }
}

// Single-line redraw helper: prints a label + value, applies A_REVERSE
// on the value when this field is focused. Used for the allow-tx
// checkbox where there's no cursor.
static void tx_draw_field(WINDOW *w, int row, int col, int focused,
                          const char *label, const char *value) {
    mvwprintw(w, row, col, "%s", label);
    if (focused) wattron(w, A_REVERSE);
    wprintw(w, "%s", value);
    if (focused) wattroff(w, A_REVERSE);
    wclrtoeol(w);
}

// Cursor-aware text-field renderer for payload + power. The value is
// drawn cell-by-cell across value_w columns; the cursor cell (if
// focused) is inverted, the remainder normal, and any space past the
// value is filled with plain spaces. When the cursor sits past the
// visible window the viewport scrolls so it stays at the right edge.
static void tx_draw_text_field(WINDOW *w, int row, int col,
                               const char *label, const char *value,
                               int value_w, int focused, int cursor)
{
    mvwprintw(w, row, col, "%s", label);
    int x = col + (int) strlen(label);
    int n = (int) strlen(value);
    int start = 0;
    if (focused && cursor > value_w - 1) start = cursor - value_w + 1;
    for (int i = 0; i < value_w; ++i) {
        int idx = start + i;
        char ch = (idx < n) ? value[idx] : ' ';
        chtype out = (chtype)(unsigned char) ch;
        if (focused && idx == cursor) {
            wattron(w, A_REVERSE);
            mvwaddch(w, row, x + i, out);
            wattroff(w, A_REVERSE);
        } else {
            mvwaddch(w, row, x + i, out);
        }
    }
    wclrtoeol(w);
}

static void tx_compose_draw(WINDOW *w, const tx_compose_t *c) {
    werase(w);
    box(w, 0, 0);
    int width = getmaxx(w);
    int payload_w = width - 14;
    if (payload_w < 32) payload_w = 32;
    if (payload_w > (int) sizeof c->payload - 1)
        payload_w = (int) sizeof c->payload - 1;

    mvwprintw(w, 0, 2, " TX compose (operator: %s)%s ",
              g_operator_user ? g_operator_user : "?",
              g_no_tx ? "  [--no-tx]" : "");
#ifdef WITH_USRP_B210
    mvwprintw(w, 1, 2,
              "B210: %s",
              g_rx_session ? "in-process (this binary)" : "(offline)");
#else
    mvwprintw(w, 1, 2, "B210: (this build has no UHD)");
#endif
    wclrtoeol(w);

    char buf[256];
    // Payload spans most of the modal width so a long telecommand
    // doesn't scroll off the right edge.
    tx_draw_text_field(w, 3, 2, "Payload  ",
                       c->payload, payload_w,
                       c->focus == TXF_PAYLOAD,
                       c->cursors[TXF_PAYLOAD]);

    tx_draw_text_field(w, 5, 2, "TX power ",
                       c->power, 8,
                       c->focus == TXF_POWER,
                       c->cursors[TXF_POWER]);
    mvwprintw(w, 5, 24, "dB  (B210 TX gain; 0..89.75)");
    wclrtoeol(w);

    snprintf(buf, sizeof buf, "[%c]", c->allow_tx ? 'x' : ' ');
    tx_draw_field(w, 7, 2, c->focus == TXF_ALLOW_TX,
                  "", buf);
    mvwprintw(w, 7, 7, "allow-tx  (required to key the PA)");
    wclrtoeol(w);

    char summary[160];
    tx_compose_summary(c, summary, sizeof summary);
    mvwprintw(w, 9, 2,  "Preview: %.*s",
              width - 12, summary);
    wclrtoeol(w);
    mvwprintw(w, 10, 2, "Status:  %.*s",
              width - 12, c->status_msg);
    wclrtoeol(w);

    mvwprintw(w, 12, 2,
              "Tab focus  Space toggle  Up/Down history  ^A/^E home/end  ^K kill  Enter send  Esc cancel");
    wclrtoeol(w);

    // Park the hardware cursor over the focused field's text cell so
    // the main loop's curs_set(1) lands a visible blinking cursor on
    // top of the A_REVERSE block. Toggle field has no cursor.
    if (c->focus == TXF_PAYLOAD) {
        int cur = c->cursors[TXF_PAYLOAD];
        int vis = (cur > payload_w - 1) ? (payload_w - 1) : cur;
        wmove(w, 3, 2 + 9 + vis);  // "Payload  " is 9 chars
    } else if (c->focus == TXF_POWER) {
        int cur = c->cursors[TXF_POWER];
        int vis = (cur > 7) ? 7 : cur;
        wmove(w, 5, 2 + 9 + vis);  // "TX power " is 9 chars
    }
    wrefresh(w);
}

static int tx_compose_validate(const tx_compose_t *c, char *err, size_t err_size) {
    if (!c->payload[0]) {
        snprintf(err, err_size, "empty payload");
        return -1;
    }
    if (!c->allow_tx) {
        snprintf(err, err_size, "--allow-tx is off; tick it before commit");
        return -1;
    }
    double db = atof(c->power);
    if (db < 0.0 || db > 89.75) {
        snprintf(err, err_size,
                 "TX power %.1f dB out of B210 range 0..89.75", db);
        return -1;
    }
    return 0;
}

static void tx_compose_broadcast_preview(const tx_compose_t *c) {
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_TX_COMMAND_PREVIEW);
    tx_compose_fill_event(c, &evt);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }
    // Mirror into our own ring buffer so the operator's TX log shows
    // the same draft line viewers are seeing.
    tx_log_push(&evt);
}

static int tx_compose_commit(const tx_compose_t *c, char *err, size_t err_size) {
#ifdef WITH_USRP_B210
    if (g_no_tx) {
        snprintf(err, err_size,
                 "TX disabled by --no-tx (preview still goes to viewers)");
        return -1;
    }
    if (g_tx_request.pending) {
        snprintf(err, err_size, "previous burst still in flight");
        return -1;
    }
    size_t n = strlen(c->payload);
    if (n == 0) {
        snprintf(err, err_size, "empty payload");
        return -1;
    }
    if (n > sizeof g_tx_request.payload) n = sizeof g_tx_request.payload;
    memcpy(g_tx_request.payload, c->payload, n);
    g_tx_request.payload_len  = n;
    g_tx_request.is_hex       = 0;  // always ascii in the simplified modal
    g_tx_request.csp_hdr      = (csp_v1_header_t){
        .prio  = 2, .src = 10, .dst = 1, .dport = 7, .sport = 16, .flags = 0,
    };
    g_tx_request.tx_freq_hz       = (long) FRONTIERSAT_CARRIER_HZ;
    g_tx_request.tx_gain_db       = atof(c->power);
    g_tx_request.repeat           = 1;
    g_tx_request.gap_ms           = 200;
    g_tx_request.allow_high_power = 0;
    g_tx_request.allow_hf_tx      = 0;
    tx_compose_summary(c, g_tx_request.summary, sizeof g_tx_request.summary);
    g_tx_request.pending = 1;
    return 0;
#else
    (void) c;
    snprintf(err, err_size, "this build has no B210 support");
    return -1;
#endif
}

#ifdef WITH_USRP_B210
// Emit a TX event locally: push into the operator's own TX log and
// broadcast to viewers via the IPC server.
static void emit_tx_event_local(sso_event_type_t type,
                                 const char *summary,
                                 const char *ack_status)
{
    sso_event_t evt;
    sso_event_init(&evt, type);
    snprintf(evt.from, sizeof evt.from, "%s",
             g_operator_user ? g_operator_user : "?");
    if (summary && summary[0]) {
        snprintf(evt.ascii, sizeof evt.ascii, "%s", summary);
    }
    if (ack_status && ack_status[0]) {
        snprintf(evt.tx_ack_status, sizeof evt.tx_ack_status, "%s", ack_status);
    }
    tx_log_push(&evt);
    if (g_ipc) {
        char buf[2048];
        if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
            sso_ipc_server_broadcast(g_ipc, buf);
        }
    }
}

#endif

static long ts_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long) ts.tv_sec * 1000000000L + (long) ts.tv_nsec;
}

// Open the modal — allocate the window, seed the compose state, draw
// once, and flip g_tx_compose_active so the main loop starts ticking
// it. Idempotent: re-opening while already active is a no-op.
static void tx_compose_open(void) {
    if (!g_ipc) return;
    if (g_tx_compose_active) return;
    if (g_auto_tcmd_active) return;  // one modal at a time
    int h = 14, ww = 120;
    if (h > LINES) h = LINES;
    if (ww > COLS) ww = COLS;
    if (ww < 60)  ww = (COLS < 60) ? COLS : 60;
    g_tx_compose_win = newwin(h, ww, (LINES - h) / 2, (COLS - ww) / 2);
    if (!g_tx_compose_win) return;
    keypad(g_tx_compose_win, TRUE);
    nodelay(g_tx_compose_win, TRUE);
    tx_compose_init(&g_tx_compose_state);
    tx_compose_draw(g_tx_compose_win, &g_tx_compose_state);
    g_tx_compose_last_edit_ns = ts_now_ns();
    g_tx_compose_active = 1;
}

// Tear the modal down. Touchwin + refresh paints stdscr's cells back
// into the area the modal occupied so the operator's normal panels
// become visible again.
static void tx_compose_close(void) {
    if (g_tx_compose_win) {
        delwin(g_tx_compose_win);
        g_tx_compose_win = NULL;
    }
    g_tx_compose_active = 0;
    touchwin(stdscr);
    refresh();
}

// Consume one key (from stdscr's getch, which the main loop is doing).
// Returns 1 to keep the modal open, 0 when the operator's Enter or
// Esc closed it — the caller invokes tx_compose_close() in that case.
static int tx_compose_handle_key(int key) {
    if (!g_tx_compose_active) return 0;
    if (key == ERR) return 1;
    tx_compose_t *c = &g_tx_compose_state;
    WINDOW *w = g_tx_compose_win;
    int changed = 1;
    // Esc may be a bare cancel OR the start of a CSI sequence (arrow
    // keys, Home/End, Delete) when keypad mode can't translate them.
    if (key == 27) {
        int translated = tx_drain_csi(w);
        if (translated >= 0) {
            key = translated;
        } else {
            tx_compose_remember(c);
            return 0;
        }
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        char err[120];
        if (tx_compose_validate(c, err, sizeof err) != 0) {
            snprintf(c->status_msg, sizeof c->status_msg,
                     "rejected: %.*s",
                     (int)(sizeof c->status_msg - 16), err);
        } else if (tx_compose_commit(c, err, sizeof err) != 0) {
            snprintf(c->status_msg, sizeof c->status_msg,
                     "commit failed: %.*s",
                     (int)(sizeof c->status_msg - 20), err);
        } else {
            tx_history_push(c->payload);
            tx_compose_remember(c);
            return 0;
        }
    } else if (key == '\t') {
        c->focus = (tx_field_t) ((c->focus + 1) % TXF_COUNT);
        tx_field_clamp_cursor(c, c->focus);
    } else if (key == KEY_BTAB) {
        c->focus = (tx_field_t) ((c->focus + TXF_COUNT - 1) % TXF_COUNT);
        tx_field_clamp_cursor(c, c->focus);
    } else if (key == KEY_BACKSPACE || key == 127 || key == 8) {
        tx_field_backspace(c);
    } else if (key == KEY_DC || key == 4 /* Ctrl-D */) {
        tx_field_delete(c);
    } else if (key == 11 /* Ctrl-K */) {
        tx_field_kill_to_end(c);
    } else if (key == KEY_LEFT) {
        tx_field_left(c);
    } else if (key == KEY_RIGHT) {
        tx_field_right(c);
    } else if (key == KEY_HOME || key == 1 /* Ctrl-A */) {
        tx_field_home(c);
    } else if (key == KEY_END  || key == 5 /* Ctrl-E */) {
        tx_field_end(c);
    } else if (key == KEY_UP) {
        tx_history_recall(c, -1);
    } else if (key == KEY_DOWN) {
        tx_history_recall(c, +1);
    } else if (key == ' ' && tx_field_is_toggle(c->focus)) {
        tx_field_toggle(c);
    } else if (key >= 32 && key < 127) {
        tx_field_insert(c, key);
    } else {
        changed = 0;
    }
    if (changed) {
        g_tx_compose_last_edit_ns = ts_now_ns();
        tx_compose_draw(w, c);
    }
    return 1;
}

// Per-tick housekeeping. Pumps the debounced preview broadcast and
// re-renders if the broadcast fired (so the mirror line refreshes).
// Called every main-loop iteration when active.
static void tx_compose_pump(void) {
    if (!g_tx_compose_active) return;
    tx_compose_t *c = &g_tx_compose_state;
    if (c->preview_dirty
        && (ts_now_ns() - g_tx_compose_last_edit_ns) >= g_tx_compose_debounce_ns) {
        tx_compose_broadcast_preview(c);
        c->preview_dirty = 0;
        tx_compose_draw(g_tx_compose_win, c);
    }
}

// --- Auto-TCMD modal helpers --------------------------------------

static const char *auto_tcmd_state_label(auto_tcmd_state_t s) {
    switch (s) {
        case AUTO_STATE_SETUP:     return "idle";
        case AUTO_STATE_RUNNING:   return "running";
        case AUTO_STATE_STOPPED:   return "stopped";
        case AUTO_STATE_DONE:      return "done";
        case AUTO_STATE_PASS_OVER: return "pass-over";
    }
    return "?";
}

static int auto_field_is_text(auto_tcmd_field_t f) {
    return f == AUTO_F_POWER || f == AUTO_F_REPEATS || f == AUTO_F_DELAY;
}
static int auto_field_is_toggle(auto_tcmd_field_t f) {
    return f == AUTO_F_ALLOW_TX;
}

static char *auto_field_buf(auto_tcmd_t *a, auto_tcmd_field_t f, size_t *cap) {
    switch (f) {
        case AUTO_F_POWER:   *cap = sizeof a->power;   return a->power;
        case AUTO_F_REPEATS: *cap = sizeof a->repeats; return a->repeats;
        case AUTO_F_DELAY:   *cap = sizeof a->delay_s; return a->delay_s;
        default:             *cap = 0; return NULL;
    }
}

static int auto_field_char_ok(auto_tcmd_field_t f, int ch) {
    if (f == AUTO_F_POWER || f == AUTO_F_DELAY) {
        return (ch >= '0' && ch <= '9') || ch == '.' || ch == '-';
    }
    if (f == AUTO_F_REPEATS) {
        return (ch >= '0' && ch <= '9');
    }
    return 0;
}

static void auto_field_clamp_cursor(auto_tcmd_t *a, auto_tcmd_field_t f) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, f, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (a->cursors[f] < 0) a->cursors[f] = 0;
    if (a->cursors[f] > n) a->cursors[f] = n;
}

static void auto_field_insert(auto_tcmd_t *a, int ch) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (n + 1 >= (int) cap) return;
    if (!auto_field_char_ok(a->focus, ch)) return;
    int cur = a->cursors[a->focus];
    if (cur < 0) cur = 0;
    if (cur > n) cur = n;
    memmove(buf + cur + 1, buf + cur, (size_t)(n - cur + 1));
    buf[cur] = (char) ch;
    a->cursors[a->focus] = cur + 1;
}
static void auto_field_backspace(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = a->cursors[a->focus];
    if (cur <= 0 || n == 0) return;
    memmove(buf + cur - 1, buf + cur, (size_t)(n - cur + 1));
    a->cursors[a->focus] = cur - 1;
}
static void auto_field_delete(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = a->cursors[a->focus];
    if (cur >= n) return;
    memmove(buf + cur, buf + cur + 1, (size_t)(n - cur));
}
static void auto_field_kill_to_end(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    int cur = a->cursors[a->focus];
    if (cur >= n) return;
    buf[cur] = '\0';
}
static void auto_field_left(auto_tcmd_t *a) {
    size_t cap = 0;
    if (!auto_field_buf(a, a->focus, &cap)) return;
    if (a->cursors[a->focus] > 0) a->cursors[a->focus]--;
}
static void auto_field_right(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    int n = (int) strlen(buf);
    if (a->cursors[a->focus] < n) a->cursors[a->focus]++;
}
static void auto_field_home(auto_tcmd_t *a) {
    size_t cap = 0;
    if (!auto_field_buf(a, a->focus, &cap)) return;
    a->cursors[a->focus] = 0;
}
static void auto_field_end(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    a->cursors[a->focus] = (int) strlen(buf);
}
static void auto_field_toggle(auto_tcmd_t *a) {
    if (a->focus == AUTO_F_ALLOW_TX) a->allow_tx = !a->allow_tx;
}

// Render helper — single inverse-cursor text-field cell, same shape as
// the TX compose renderer.
static void auto_draw_text_field(WINDOW *w, int row, int col,
                                 const char *label, const char *value,
                                 int value_w, int focused, int cursor)
{
    mvwprintw(w, row, col, "%s", label);
    int x = col + (int) strlen(label);
    int n = (int) strlen(value);
    int start = 0;
    if (focused && cursor > value_w - 1) start = cursor - value_w + 1;
    for (int i = 0; i < value_w; ++i) {
        int idx = start + i;
        char ch = (idx < n) ? value[idx] : ' ';
        chtype out = (chtype)(unsigned char) ch;
        if (focused && idx == cursor) {
            wattron(w, A_REVERSE);
            mvwaddch(w, row, x + i, out);
            wattroff(w, A_REVERSE);
        } else {
            mvwaddch(w, row, x + i, out);
        }
    }
    wclrtoeol(w);
}

static void auto_tcmd_draw(void) {
    if (!g_auto_tcmd_active || !g_auto_tcmd_win) return;
    WINDOW *w = g_auto_tcmd_win;
    auto_tcmd_t *a = &g_auto_tcmd;
    werase(w);
    box(w, 0, 0);
    int width = getmaxx(w);

    mvwprintw(w, 0, 2, " Auto-TCMD (operator: %s)%s ",
              g_operator_user ? g_operator_user : "?",
              g_no_tx ? "  [--no-tx]" : "");

    mvwprintw(w, 1, 2, "File:    %.*s  (%d commands)",
              width - 28, a->file_path[0] ? a->file_path : "(none)",
              a->n_commands);
    wclrtoeol(w);

    int running_ro = (a->state == AUTO_STATE_RUNNING);

    auto_draw_text_field(w, 3, 2, "TX power ",
                         a->power, 8,
                         !running_ro && a->focus == AUTO_F_POWER,
                         a->cursors[AUTO_F_POWER]);
    mvwprintw(w, 3, 24, "dB  (B210 TX gain; 0..89.75)");
    wclrtoeol(w);

    auto_draw_text_field(w, 4, 2, "Repeats  ",
                         a->repeats, 6,
                         !running_ro && a->focus == AUTO_F_REPEATS,
                         a->cursors[AUTO_F_REPEATS]);
    mvwprintw(w, 4, 24, "per command (TCM1 N×, then TCM2 N×, ...)");
    wclrtoeol(w);

    auto_draw_text_field(w, 5, 2, "Delay    ",
                         a->delay_s, 8,
                         !running_ro && a->focus == AUTO_F_DELAY,
                         a->cursors[AUTO_F_DELAY]);
    mvwprintw(w, 5, 24, "s between every send");
    wclrtoeol(w);

    char tg[8];
    snprintf(tg, sizeof tg, "[%c]", a->allow_tx ? 'x' : ' ');
    mvwprintw(w, 7, 2, "%s", "");
    if (!running_ro && a->focus == AUTO_F_ALLOW_TX) wattron(w, A_REVERSE);
    mvwprintw(w, 7, 2, "%s", tg);
    if (!running_ro && a->focus == AUTO_F_ALLOW_TX) wattroff(w, A_REVERSE);
    mvwprintw(w, 7, 7, "allow-tx  (required to key the PA)");
    wclrtoeol(w);

    mvwprintw(w, 9, 2, "State:    %s", auto_tcmd_state_label(a->state));
    wclrtoeol(w);
    if (a->n_commands > 0) {
        int rt = a->repeats_total > 0 ? a->repeats_total : 0;
        mvwprintw(w, 10, 2,
                  "Progress: cmd %d/%d   send %d/%d   total sent: %d",
                  a->cmd_idx + (a->state == AUTO_STATE_RUNNING ? 1 : 0),
                  a->n_commands,
                  a->repeat_idx, rt,
                  a->sends_total);
    } else {
        mvwprintw(w, 10, 2, "Progress: (no commands loaded)");
    }
    wclrtoeol(w);
    mvwprintw(w, 11, 2, "Last sent: %.*s",
              width - 14, a->last_sent[0] ? a->last_sent : "-");
    wclrtoeol(w);
    mvwprintw(w, 12, 2, "Status:    %.*s",
              width - 14, a->status_msg[0] ? a->status_msg : "-");
    wclrtoeol(w);

    if (a->state == AUTO_STATE_RUNNING) {
        mvwprintw(w, 14, 2,
                  "Running — s stops   Esc closes (and stops)");
    } else {
        mvwprintw(w, 14, 2,
                  "Tab focus  Space toggle  Enter start  Esc cancel");
    }
    wclrtoeol(w);

    // Park the hardware cursor on the focused text field's cell. The
    // toggle field has no cursor; running mode is read-only so we
    // also skip cursor placement there.
    if (a->state != AUTO_STATE_RUNNING) {
        if (a->focus == AUTO_F_POWER) {
            int cur = a->cursors[AUTO_F_POWER];
            int vis = (cur > 7) ? 7 : cur;
            wmove(w, 3, 2 + 9 + vis);   // "TX power " is 9 chars
        } else if (a->focus == AUTO_F_REPEATS) {
            int cur = a->cursors[AUTO_F_REPEATS];
            int vis = (cur > 5) ? 5 : cur;
            wmove(w, 4, 2 + 9 + vis);   // "Repeats  " is 9 chars
        } else if (a->focus == AUTO_F_DELAY) {
            int cur = a->cursors[AUTO_F_DELAY];
            int vis = (cur > 7) ? 7 : cur;
            wmove(w, 5, 2 + 9 + vis);   // "Delay    " is 9 chars
        }
    }
    wrefresh(w);
}

// Open the modal. Refuses if the TX compose modal is already up — at
// most one modal owns the screen at a time. Lazily loads the file if
// --tc-file was passed and we haven't loaded yet.
static void auto_tcmd_open(void) {
    if (!g_ipc) return;
    if (g_tx_compose_active) return;
    if (g_auto_tcmd_active) return;
    if (g_auto_tcmd_file_path[0] == '\0') return;

    // (Re)load on open — file may have been edited since last open.
    if (g_auto_tcmd.commands) {
        auto_tcmd_free_commands(g_auto_tcmd.commands, g_auto_tcmd.n_commands);
        g_auto_tcmd.commands = NULL;
        g_auto_tcmd.n_commands = 0;
    }
    char **cmds = NULL;
    int    nc   = 0;
    if (auto_tcmd_load_file(g_auto_tcmd_file_path, &cmds, &nc) != 0) {
        return;  // silent — operator will notice via the absent modal
    }

    memset(&g_auto_tcmd, 0, sizeof g_auto_tcmd);
    g_auto_tcmd.commands   = cmds;
    g_auto_tcmd.n_commands = nc;
    snprintf(g_auto_tcmd.file_path, sizeof g_auto_tcmd.file_path,
             "%.*s", (int)(sizeof g_auto_tcmd.file_path - 1),
             g_auto_tcmd_file_path);
    snprintf(g_auto_tcmd.power,   sizeof g_auto_tcmd.power,   "30.0");
    snprintf(g_auto_tcmd.repeats, sizeof g_auto_tcmd.repeats, "3");
    snprintf(g_auto_tcmd.delay_s, sizeof g_auto_tcmd.delay_s, "2.0");
    g_auto_tcmd.allow_tx = 0;
    g_auto_tcmd.focus    = AUTO_F_POWER;
    g_auto_tcmd.cursors[AUTO_F_POWER]   = (int) strlen(g_auto_tcmd.power);
    g_auto_tcmd.cursors[AUTO_F_REPEATS] = (int) strlen(g_auto_tcmd.repeats);
    g_auto_tcmd.cursors[AUTO_F_DELAY]   = (int) strlen(g_auto_tcmd.delay_s);
    g_auto_tcmd.state    = AUTO_STATE_SETUP;
    snprintf(g_auto_tcmd.status_msg, sizeof g_auto_tcmd.status_msg,
             "loaded %d command(s). Set fields, then Enter to start.",
             nc);

    int h = 17, ww = 110;
    if (h > LINES) h = LINES;
    if (ww > COLS) ww = COLS;
    if (ww < 60)  ww = (COLS < 60) ? COLS : 60;
    g_auto_tcmd_win = newwin(h, ww, (LINES - h) / 2, (COLS - ww) / 2);
    if (!g_auto_tcmd_win) {
        auto_tcmd_free_commands(g_auto_tcmd.commands, g_auto_tcmd.n_commands);
        g_auto_tcmd.commands = NULL;
        g_auto_tcmd.n_commands = 0;
        return;
    }
    keypad(g_auto_tcmd_win, TRUE);
    nodelay(g_auto_tcmd_win, TRUE);
    g_auto_tcmd_active = 1;
    auto_tcmd_draw();
}

static void auto_tcmd_close(void) {
    if (g_auto_tcmd_win) {
        delwin(g_auto_tcmd_win);
        g_auto_tcmd_win = NULL;
    }
    g_auto_tcmd_active = 0;
    if (g_auto_tcmd.commands) {
        auto_tcmd_free_commands(g_auto_tcmd.commands, g_auto_tcmd.n_commands);
        g_auto_tcmd.commands = NULL;
        g_auto_tcmd.n_commands = 0;
    }
    touchwin(stdscr);
    refresh();
}

// Validate the setup fields and move to RUNNING. Returns 0 on success,
// fills status_msg + returns -1 on failure.
static int auto_tcmd_start(void) {
    auto_tcmd_t *a = &g_auto_tcmd;
    if (a->n_commands == 0) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: file has no commands");
        return -1;
    }
    if (!a->allow_tx) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: allow-tx is off");
        return -1;
    }
    double power = atof(a->power);
    if (power < 0.0 || power > 89.75) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: TX power %.1f dB out of B210 range 0..89.75",
                 power);
        return -1;
    }
    int repeats = atoi(a->repeats);
    if (repeats < 1) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: repeats must be >= 1");
        return -1;
    }
    double delay = atof(a->delay_s);
    if (delay < 0.0) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: delay must be >= 0");
        return -1;
    }
    a->repeats_total = repeats;
    a->delay_s_val   = delay;
    a->cmd_idx       = 0;
    a->repeat_idx    = 0;
    a->sends_total   = 0;
    a->next_send_ns  = ts_now_ns();  // first send fires immediately
    a->state         = AUTO_STATE_RUNNING;
    snprintf(a->status_msg, sizeof a->status_msg,
             "running: %d cmds × %d repeats, %.2f s delay",
             a->n_commands, repeats, delay);
    return 0;
}

// Pause / cancel without closing the modal so the operator can see the
// final progress numbers.
static void auto_tcmd_stop(const char *reason) {
    auto_tcmd_t *a = &g_auto_tcmd;
    if (a->state != AUTO_STATE_RUNNING) return;
    a->state = AUTO_STATE_STOPPED;
    snprintf(a->status_msg, sizeof a->status_msg, "stopped: %s",
             reason ? reason : "user");
}

static int auto_tcmd_handle_key(int key) {
    if (!g_auto_tcmd_active) return 0;
    if (key == ERR) return 1;
    auto_tcmd_t *a = &g_auto_tcmd;
    int changed = 1;
    // Esc-as-CSI same fallback the TX modal uses.
    if (key == 27) {
        int translated = tx_drain_csi(g_auto_tcmd_win);
        if (translated >= 0) {
            key = translated;
        } else {
            return 0;  // Esc closes (and stops via close path below)
        }
    }
    if (a->state == AUTO_STATE_RUNNING) {
        // Run mode: only stop / close commands are honoured. Field
        // edits are blocked so an operator can't change power mid-run.
        if (key == 's' || key == 'S') {
            auto_tcmd_stop("user");
            auto_tcmd_draw();
            return 1;
        }
        return 1;
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        if (auto_tcmd_start() == 0) {
            auto_tcmd_draw();
        } else {
            auto_tcmd_draw();
        }
        return 1;
    } else if (key == '\t') {
        a->focus = (auto_tcmd_field_t) ((a->focus + 1) % AUTO_F_COUNT);
        auto_field_clamp_cursor(a, a->focus);
    } else if (key == KEY_BTAB) {
        a->focus = (auto_tcmd_field_t) ((a->focus + AUTO_F_COUNT - 1)
                                         % AUTO_F_COUNT);
        auto_field_clamp_cursor(a, a->focus);
    } else if (key == KEY_BACKSPACE || key == 127 || key == 8) {
        auto_field_backspace(a);
    } else if (key == KEY_DC || key == 4) {
        auto_field_delete(a);
    } else if (key == 11) {
        auto_field_kill_to_end(a);
    } else if (key == KEY_LEFT) {
        auto_field_left(a);
    } else if (key == KEY_RIGHT) {
        auto_field_right(a);
    } else if (key == KEY_HOME || key == 1) {
        auto_field_home(a);
    } else if (key == KEY_END || key == 5) {
        auto_field_end(a);
    } else if (key == ' ' && auto_field_is_toggle(a->focus)) {
        auto_field_toggle(a);
    } else if (key >= 32 && key < 127) {
        auto_field_insert(a, key);
    } else {
        changed = 0;
    }
    if (changed) auto_tcmd_draw();
    return 1;
}

// Per-tick burst driver. When running, queues one g_tx_request when
// (a) the previous burst has cleared, and (b) the inter-send delay
// has elapsed. Stops automatically on LOS so an unattended run won't
// keep TXing after the pass. emit_tx_event_local fires from the main
// loop's burst-handler the same way it does for the manual TX
// compose path, so tx.log + viewer fanout capture every shot.
static void auto_tcmd_tick(state_t *state) {
    if (!g_auto_tcmd_active) return;
    auto_tcmd_t *a = &g_auto_tcmd;
    if (a->state != AUTO_STATE_RUNNING) return;

    // LOS guard. We consider the pass over once the elevation has
    // gone negative AND the predictor has rolled the next pass into
    // the future. Sitting on a freshly-loaded prediction during AOS
    // ambiguity (elevation < 0 but next pass not yet predicted) is
    // not enough to abort; the running flag stays so a momentary
    // numerical wobble can't kill an active session.
    double el = state->prediction.satellite_ephem.elevation;
    if (el < 0.0
        && state->prediction.predicted_minutes_until_visible > 0.5) {
        a->state = AUTO_STATE_PASS_OVER;
        snprintf(a->status_msg, sizeof a->status_msg,
                 "stopped: pass over (elevation %.1f deg)", el);
        auto_tcmd_draw();
        return;
    }

    if (a->cmd_idx >= a->n_commands) {
        a->state = AUTO_STATE_DONE;
        snprintf(a->status_msg, sizeof a->status_msg,
                 "done: sent all %d command(s)", a->n_commands);
        auto_tcmd_draw();
        return;
    }

#ifdef WITH_USRP_B210
    long now = ts_now_ns();
    if (now < a->next_send_ns) return;
    if (g_tx_request.pending)  return;  // prior burst still inflight

    const char *cmd = a->commands[a->cmd_idx];
    size_t n = strlen(cmd);
    if (n > sizeof g_tx_request.payload) n = sizeof g_tx_request.payload;
    memcpy(g_tx_request.payload, cmd, n);
    g_tx_request.payload_len  = n;
    g_tx_request.is_hex       = 0;
    g_tx_request.csp_hdr      = (csp_v1_header_t){
        .prio  = 2, .src = 10, .dst = 1, .dport = 7, .sport = 16, .flags = 0,
    };
    g_tx_request.tx_freq_hz       = (long) FRONTIERSAT_CARRIER_HZ;
    g_tx_request.tx_gain_db       = atof(a->power);
    g_tx_request.repeat           = 1;
    g_tx_request.gap_ms           = 200;
    // No g_tx_request.allow_tx field — the TX-inhibit gate is enforced
    // at auto_tcmd_start time (refuses to enter RUNNING unless allow_tx
    // is ticked), same way tx_compose_validate handles it before commit.
    g_tx_request.allow_high_power = 0;
    g_tx_request.allow_hf_tx      = 0;
    snprintf(g_tx_request.summary, sizeof g_tx_request.summary,
             "auto[%d/%d %d/%d]: %s",
             a->cmd_idx + 1, a->n_commands,
             a->repeat_idx + 1, a->repeats_total,
             cmd);
    g_tx_request.pending = 1;
    snprintf(a->last_sent, sizeof a->last_sent, "%s", cmd);
    a->sends_total++;

    a->repeat_idx++;
    if (a->repeat_idx >= a->repeats_total) {
        a->cmd_idx++;
        a->repeat_idx = 0;
    }

    a->next_send_ns = now + (long)(a->delay_s_val * 1e9);
    auto_tcmd_draw();
#else
    (void) state;
    a->state = AUTO_STATE_STOPPED;
    snprintf(a->status_msg, sizeof a->status_msg,
             "stopped: this build has no B210 (WITH_USRP_B210=OFF)");
    auto_tcmd_draw();
#endif
}

// --- Forward decls -------------------------------------------------

void start_tracking(state_t *state);
void stop_tracking(state_t *state);
int  point_to_stationary_target(state_t *state, double azimuth, double elevation);
void update_doppler_shifted_frequencies(state_t *state, double uplink_freq, double downlink_freq);
int  apply_args(state_t *state, int argc, char **argv, double jul_utc);

// --- TLE auto-discovery helpers (used by --control with no
//     positional satellite name) ------------------------------------

// Recursively scan `dir` for *.tle files, return the path with the
// newest mtime via out_path. Returns 0 on success, -1 if dir is
// unreadable or no .tle file exists in the tree. Caller-allocated
// buffer must be at least PATH_MAX-ish.
static int find_newest_tle_recursive(const char *dir,
                                     char *out_path, size_t out_cap,
                                     time_t *out_mtime)
{
    DIR *d = opendir(dir);
    if (d == NULL) return -1;
    int found = 0;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char child[1024];
        int n = snprintf(child, sizeof child, "%s/%s", dir, de->d_name);
        if (n < 0 || (size_t)n >= sizeof child) continue;
        struct stat st;
        if (stat(child, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            char nested[1024];
            time_t nested_mtime = 0;
            if (find_newest_tle_recursive(child, nested, sizeof nested,
                                          &nested_mtime) == 0) {
                if (!found || nested_mtime > *out_mtime) {
                    snprintf(out_path, out_cap, "%s", nested);
                    *out_mtime = nested_mtime;
                    found = 1;
                }
            }
        } else if (S_ISREG(st.st_mode)) {
            size_t nlen = strlen(de->d_name);
            if (nlen < 4) continue;
            if (strcmp(de->d_name + nlen - 4, ".tle") != 0) continue;
            if (!found || st.st_mtime > *out_mtime) {
                snprintf(out_path, out_cap, "%s", child);
                *out_mtime = st.st_mtime;
                found = 1;
            }
        }
    }
    closedir(d);
    return found ? 0 : -1;
}

// Pull the satellite name out of a 3-line TLE — the first non-blank
// line that doesn't begin with "1 " or "2 ". Trims trailing
// whitespace. Returns 0 on success, -1 if the file is unreadable or
// has no name line.
static int read_tle_name(const char *tle_path,
                         char *out_name, size_t out_cap)
{
    FILE *f = fopen(tle_path, "r");
    if (f == NULL) return -1;
    char line[256];
    int rc = -1;
    while (fgets(line, sizeof line, f) != NULL) {
        size_t n = strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'
                      || line[n - 1] == ' '  || line[n - 1] == '\t')) {
            line[--n] = '\0';
        }
        if (n == 0) continue;
        if ((line[0] == '1' || line[0] == '2') && line[1] == ' ') continue;
        snprintf(out_name, out_cap, "%s", line);
        rc = 0;
        break;
    }
    fclose(f);
    return rc;
}

// mkdir -p for the parent of `path`. Used before copy_file to make
// sure ~/.local/state/simple_sat_ops/ exists.
static int mkdir_p_for_file(const char *path)
{
    char buf[1024];
    snprintf(buf, sizeof buf, "%s", path);
    for (char *p = buf + 1; *p; p++) {
        if (*p != '/') continue;
        *p = '\0';
        if (mkdir(buf, 0755) != 0 && errno != EEXIST) {
            *p = '/';
            return -1;
        }
        *p = '/';
    }
    return 0;
}

// Plain byte-copy. Returns 0 on success, -1 on any I/O error.
static int copy_file(const char *src, const char *dst)
{
    FILE *in = fopen(src, "rb");
    if (in == NULL) return -1;
    if (mkdir_p_for_file(dst) != 0) { fclose(in); return -1; }
    FILE *out = fopen(dst, "wb");
    if (out == NULL) { fclose(in); return -1; }
    char buf[4096];
    size_t n;
    int rc = 0;
    while ((n = fread(buf, 1, sizeof buf, in)) > 0) {
        if (fwrite(buf, 1, n, out) != n) { rc = -1; break; }
    }
    if (ferror(in)) rc = -1;
    fclose(in);
    if (fclose(out) != 0) rc = -1;
    return rc;
}

// --- Pass folder for the upcoming pass -----------------------------

// Julian Date -> Unix epoch seconds. Reference: JD 2440587.5 is
// 1970-01-01 00:00:00 UTC, which is what time_t counts from.
static time_t jul_to_unix(double jd)
{
    return (time_t)((jd - 2440587.5) * 86400.0 + 0.5);
}

// Refresh /FrontierSat/Operations/current so it points at `target`.
// Atomic-ish: unlink the old link, symlink the new one. If the
// symlink call fails we log and carry on — the pass folder itself is
// still created and broadcast over IPC, the symlink is just a
// convenience.
static void update_operations_current_symlink(const char *target)
{
    const char *link = sso_operations_current_symlink();
    if (link == NULL || link[0] == '\0') return;
    // Make sure /FrontierSat/Operations/ exists for the symlink slot.
    sso_mkdir_p_for_file(link);
    unlink(link);
    if (symlink(target, link) != 0) {
        fprintf(stderr,
                "simple_sat_ops: symlink %s -> %s failed: %s "
                "(non-fatal; pass folder still set)\n",
                link, target, strerror(errno));
    }
}

// Compute a fresh AOS prediction off `state`'s current position and
// build /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/. Stashes the
// result in g_pass_folder so ipc_broadcast_state can publish it on
// every tick.
static void setup_pass_folder(state_t *state, double jul_utc_now)
{
    // Handoff case: --pass-folder seeded g_pass_folder before we got
    // here. Honour it — make sure the dir exists, refresh the
    // "current" symlink, and skip AOS-discovery entirely.
    if (g_pass_folder[0]) {
        if (sso_mkdir_p(g_pass_folder) != 0) {
            fprintf(stderr,
                "simple_sat_ops: mkdir -p %s failed: %s\n",
                g_pass_folder, strerror(errno));
        }
        update_operations_current_symlink(g_pass_folder);
        fprintf(stderr, "simple_sat_ops: using inherited pass folder %s\n",
                g_pass_folder);
        return;
    }
    minutes_until_visible(&state->prediction, jul_utc_now,
                          jul_utc_now + MAX_MINUTES_TO_PREDICT / 1440.0,
                          1.0);
    // minutes_until_visible sets predicted_minutes_until_visible and
    // uses -9999.0 as the "no AOS in this window" sentinel. Positive
    // values are minutes until AOS; negatives are minutes since AOS
    // when we started mid-pass — either way (now + N) lands on the
    // current pass's AOS, which is what we want for the folder name.
    double minutes = state->prediction.predicted_minutes_until_visible;
    if (minutes <= -9000.0) {
        fprintf(stderr,
                "simple_sat_ops: no AOS in the next %d minutes — "
                "pass folder not created\n",
                MAX_MINUTES_TO_PREDICT);
        return;
    }
    double aos_jul = jul_utc_now + minutes / 1440.0;
    time_t aos = jul_to_unix(aos_jul);
    struct tm aos_local;
    localtime_r(&aos, &aos_local);
    char folder[256];
    int n = snprintf(folder, sizeof folder,
                     "%s/%04d%02d%02d/%02d%02dLT",
                     sso_operations_dir(),
                     aos_local.tm_year + 1900,
                     aos_local.tm_mon + 1,
                     aos_local.tm_mday,
                     aos_local.tm_hour,
                     aos_local.tm_min);
    if (n <= 0 || (size_t)n >= sizeof folder) {
        fprintf(stderr,
                "simple_sat_ops: pass folder name too long; skipping\n");
        return;
    }
    if (sso_mkdir_p(folder) != 0) {
        fprintf(stderr,
                "simple_sat_ops: mkdir -p %s failed: %s\n",
                folder, strerror(errno));
        return;
    }

    // Pin the TLE the operator just loaded into the pass folder so
    // any post-pass analysis (rx_replay, packet_browser session-dir
    // lookups, whoever shows up later) can find the exact ephemeris
    // that was being tracked, even if active.tle gets rewritten on
    // the next --control startup.
    if (state->prediction.tles_filename
        && state->prediction.tles_filename[0]) {
        const char *src = state->prediction.tles_filename;
        const char *base = strrchr(src, '/');
        base = (base != NULL) ? base + 1 : src;
        char dst[512];
        int rc = snprintf(dst, sizeof dst, "%s/%s", folder, base);
        if (rc > 0 && (size_t)rc < sizeof dst) {
            if (copy_file(src, dst) != 0) {
                fprintf(stderr,
                        "simple_sat_ops: warning: TLE copy %s -> %s "
                        "failed: %s\n",
                        src, dst, strerror(errno));
            } else {
                fprintf(stderr, "simple_sat_ops: pinned TLE %s\n", dst);
            }
        }
    }

    snprintf(g_pass_folder, sizeof g_pass_folder, "%s", folder);
    update_operations_current_symlink(folder);
    fprintf(stderr, "simple_sat_ops: pass folder %s\n", folder);
}

// Sample the upcoming pass on a local prediction_t copy and render a
// polar az/el plot to pass_folder/az_el_plot.png via gnuplot. Two
// traces: the satellite's sky position and the rotator boom's beam
// direction (which match for non-flip passes and diverge near apex on
// flip passes -- a visual sanity check of the flip mapping). The
// raw TSV and the gnuplot script are left in the pass folder so the
// operator can rerun or tweak the plot offline.
static void generate_pass_plot(state_t *state, const char *pass_folder,
                               double jul_utc_now)
{
    if (!pass_folder || !pass_folder[0]) {
        return;
    }

    // Work on a local copy: update_pass_predictions / update_satellite_position
    // both mutate the prediction's satellite_ephem and aggregate fields.
    prediction_t pred = state->prediction;

    // Defensive: handoff case (setup_pass_folder used inherited
    // g_pass_folder) leaves predicted_minutes_until_visible stale.
    // Re-run the search so aos_jul below is well defined.
    minutes_until_visible(&pred, jul_utc_now,
                          jul_utc_now + 180.0 / 1440.0, 1.0);
    if (pred.predicted_minutes_until_visible <= -9000.0) {
        fprintf(stderr,
                "simple_sat_ops: no AOS in the next 3 h — skipping plot\n");
        return;
    }
    double aos_jul = jul_utc_now + pred.predicted_minutes_until_visible / 1440.0;

    // Populate predicted_max_elevation / predicted_ascension_azimuth /
    // predicted_pass_duration_minutes on the local copy.
    update_pass_predictions(&pred, aos_jul, 0.1);

    int flip_mode = 0;
    double aos_az = pred.predicted_ascension_azimuth;
    double los_az = pred.predicted_descent_azimuth;
    double aos_jul_pred = pred.predicted_ascension_jul_utc;
    double los_jul_pred = pred.predicted_descent_jul_utc;
    double pass_jul = los_jul_pred - aos_jul_pred;
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION > 90
        && pred.predicted_max_elevation
               >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD) {
        flip_mode = 1;
    }

    char dat_path[512];
    int n = snprintf(dat_path, sizeof dat_path, "%s/pass_plot.dat",
                     pass_folder);
    if (n <= 0 || (size_t)n >= sizeof dat_path) return;
    FILE *f = fopen(dat_path, "w");
    if (!f) {
        fprintf(stderr, "simple_sat_ops: fopen %s: %s\n",
                dat_path, strerror(errno));
        return;
    }
    fprintf(f, "# t_min\tsat_az\tsat_el\tbeam_az\tbeam_el\n");

    // 10 s cadence over the visible portion. predicted_pass_duration_minutes
    // includes the -5..0 deg pre-AOS / post-LOS buffer, so step a little
    // wider and let the el-filter below drop the wings.
    const double step_min = 10.0 / 60.0;
    const double duration_min = pred.predicted_pass_duration_minutes > 0
        ? pred.predicted_pass_duration_minutes
        : 15.0;
    const int n_steps = (int)(duration_min / step_min) + 1;

    int wrote = 0;
    for (int i = 0; i <= n_steps; ++i) {
        double t_min = i * step_min;
        double sample_jul = aos_jul + t_min / 1440.0;
        update_satellite_position(&pred, sample_jul);
        double sat_az = pred.satellite_ephem.azimuth;
        double sat_el = pred.satellite_ephem.elevation;
        if (sat_el < 0.0) continue;

        double mech_az = sat_az;
        double mech_el = sat_el;
        int half;
        double progress = 0.0;
        if (pass_jul > 0.0) {
            progress = (sample_jul - aos_jul_pred) / pass_jul;
        }
        antenna_rotator_to_mech_coords(flip_mode, aos_az, los_az,
                                       progress,
                                       sat_az, sat_el,
                                       &mech_az, &mech_el, &half);

        // Convert mech back to where the boom's beam actually points
        // on the sky. mech_el > 90 deg means back-pointing through the
        // rotator: equivalent sky direction is (mech_az + 180, 180 -
        // mech_el).
        double beam_az = mech_az;
        double beam_el = mech_el;
        if (beam_el > 90.0) {
            beam_az = fmod(beam_az + 180.0, 360.0);
            if (beam_az < 0.0) beam_az += 360.0;
            beam_el = 180.0 - beam_el;
        }

        fprintf(f, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
                t_min, sat_az, sat_el, beam_az, beam_el);
        ++wrote;
    }
    fclose(f);

    if (wrote == 0) {
        fprintf(stderr,
                "simple_sat_ops: no visible samples in predicted pass — "
                "skipping plot\n");
        return;
    }

    char gp_path[512];
    n = snprintf(gp_path, sizeof gp_path, "%s/pass_plot.gp", pass_folder);
    if (n <= 0 || (size_t)n >= sizeof gp_path) return;
    FILE *gp = fopen(gp_path, "w");
    if (!gp) {
        fprintf(stderr, "simple_sat_ops: fopen %s: %s\n",
                gp_path, strerror(errno));
        return;
    }

    const char *sat_name =
        (state->prediction.satellite_ephem.name
         && state->prediction.satellite_ephem.name[0])
            ? state->prediction.satellite_ephem.name : "satellite";

    // Mirror scripts/plot_sky_pass.sh's polar style so both plots read
    // the same way (N up, E clockwise, zenith at centre). Solid darker
    // grid + elevation-valued rtics so the reticules are readable.
    fprintf(gp,
        "set terminal pngcairo size 900,900 enhanced font 'Helvetica,11'\n"
        "set output '%s/az_el_plot.png'\n"
        "set polar\n"
        "set angles degrees\n"
        "set theta top clockwise\n"
        "set size square\n"
        // Explicit margins so the W cardinal label isn't clipped on the
        // left and the legend has room on the right.
        "set lmargin 8\n"
        "set rmargin 14\n"
        "set tmargin 4\n"
        "set bmargin 4\n"
        "set grid polar 30 lt 1 lw 0.6 lc rgb '#666666'\n"
        "unset border\n"
        "unset xtics\n"
        "unset ytics\n"
        "set rrange [0:90]\n"
        // Label the rings with the elevation they correspond to (r = 90
        // - el): inner ring = 60 deg el, outer ring = 0 deg el (horizon);
        // centre is zenith (90 deg el) and the title says so.
        "set rtics ('60' 30, '30' 60, '0' 90) "
              "font 'Helvetica,9' textcolor rgb '#333333'\n"
        "set ttics ('N' 0, 'E' 90, 'S' 180, 'W' 270) "
              "font 'Helvetica,11,Bold'\n"
        "set key outside right top\n"
        "set title \"%s  max_el=%.1f deg  flip=%s\\n"
                  "(zenith = centre; ring labels = elevation deg)\" noenhanced\n"
        "plot \\\n"
        "  '%s/pass_plot.dat' using 2:(90-$3) "
            "with lines lw 2 lc rgb '#1f77b4' title 'satellite', \\\n"
        "  '' using 4:(90-$5) "
            "with lines lw 2 lc rgb '#d62728' title 'antenna beam'\n",
        pass_folder,
        sat_name,
        pred.predicted_max_elevation,
        flip_mode ? "yes" : "no",
        pass_folder);
    fclose(gp);

    char cmd[1100];
    n = snprintf(cmd, sizeof cmd, "gnuplot '%s' 2>&1", gp_path);
    if (n <= 0 || (size_t)n >= sizeof cmd) return;
    int rc = system(cmd);
    if (rc != 0) {
        fprintf(stderr,
                "simple_sat_ops: gnuplot failed (rc=%d). Install gnuplot "
                "or run manually:  gnuplot '%s'\n",
                rc, gp_path);
    } else {
        fprintf(stderr,
                "simple_sat_ops: pass plot %s/az_el_plot.png "
                "(%d samples, %s)\n",
                pass_folder, wrote,
                flip_mode ? "flip" : "no flip");
    }
}

// --- Usage ---------------------------------------------------------

void usage(FILE *dest, const char *name, int full)
{
    fprintf(dest,
        "usage: %s [--control] [<satellite_id>] [options]\n"
        "\n"
        "Live satellite tracker for the FrontierSat ground station. Predicts\n"
        "passes, drives the SPID rotator, computes Doppler-shifted simplex\n"
        "frequencies, and (with --control) owns the USRP B210 directly:\n"
        "RX + AX100 decode + in-process TX bursts via the 't' compose modal.\n"
        "One process for everything live. Viewers connect over sso_ipc.\n"
        "\n"
        "Modes:\n"
        "  %s --control                 Operator: opens the sso_ipc server,\n"
        "                               drives the rotator. With no\n"
        "                               <satellite_id>, the newest *.tle under\n"
        "                               /FrontierSat/TLEs/ is loaded directly\n"
        "                               and pinned into the pass folder.\n"
        "  %s <satellite_id>            Standalone tracker (legacy / dev).\n"
        "  %s                           No args: connects to the running\n"
        "                               operator and shows its state\n"
        "                               read-only. Errors out if no\n"
        "                               operator is running.\n"
        "\n"
        "Positional arguments:\n"
        "  <satellite_id>               Name prefix to match in the TLE, or `next`\n"
        "                               to auto-select the next visible pass.\n"
        "                               Optional when --control is given.\n"
        "\n"
        "TLE source:\n"
        "  --tle=<path>                 Path to a TLE file (2 or 3-line format).\n"
        "                               Default: $HOME/.local/state/simple_sat_ops/active.tle\n"
        "                               (auto-populated by --control without a\n"
        "                               positional satellite_id).\n"
        "\n"
        "Hardware (rotator + B210):\n"
        "  --without-rotator            Skip the SPID Rot2Prog. Default is on:\n"
        "                               the tracker initialises and commands\n"
        "                               the rotator unless this flag is given.\n"
        "  --without-hardware           Synonym for --without-rotator\n"
        "  --without-b210               Skip the USRP B210 (dev hosts that\n"
        "                               have UHD but no device, or any time\n"
        "                               you just want the UI + rotator).\n"
        "  --no-tx                      Open the B210 for RX, but block the\n"
        "                               TX compose modal from actually keying\n"
        "                               the PA. Typing + preview broadcast to\n"
        "                               viewers still work, so the operator\n"
        "                               can rehearse a telecommand and get\n"
        "                               eyes on it without going on air.\n"
        "  --tc-file=<path>             Load a file of ASCII telecommands\n"
        "                               (CTS1+...; one per line; '#' lines\n"
        "                               and blank lines ignored). Press 'A'\n"
        "                               (or `:auto`) in the operator UI to\n"
        "                               open the auto-tcmd modal, set TX\n"
        "                               power / repeats-per-command / inter-\n"
        "                               send delay / allow-tx, and Enter to\n"
        "                               start. Sends each command N times\n"
        "                               with the chosen delay between every\n"
        "                               shot, then advances to the next line.\n"
        "                               Stops automatically at LOS so an\n"
        "                               unattended run can't keep TXing\n"
        "                               after the pass. All TX events land\n"
        "                               in <pass_folder>/tx.log.\n"
        "\n"
        "Operator coordination:\n"
        "  --control                    Open the sso_ipc server (operator mode).\n"
        "                               Without it, simple_sat_ops runs standalone\n"
        "                               and no other operator-aware tool can verify\n"
        "                               against it.\n"
        "\n"
        "Carrier display (informational; the B210 tools tune themselves):\n"
        "  --uplink-freq-mhz=<mhz>      Uplink nominal (default %.6f)\n"
        "  --downlink-freq-mhz=<mhz>    Downlink / simplex carrier nominal\n"
        "                               (default %.6f)\n"
        "  --no-doppler-correction      Display nominal freqs without Doppler\n"
        "\n"
        "Rotator overrides:\n"
        "  --rotator-device=<path>           SPID Rot2Prog tty. Default\n"
        "                                    /dev/ttyUSB0 (Linux).\n"
        "  --rotator-target-azimuth=<deg>    Park on a fixed azimuth\n"
        "  --rotator-target-elevation=<deg>  Park on a fixed elevation\n"
        "\n"
        "Observer location (default RAO Priddis):\n"
        "  --lat=<deg>                  Geodetic latitude\n"
        "  --lon=<deg>                  Geodetic longitude (east positive)\n"
        "  --alt=<m>                    Altitude above ellipsoid, metres\n"
        "\n"
        "Pass filter (used when <satellite_id> = `next`):\n"
        "  --include-constellations     Include Starlink/OneWeb-style swarms\n"
        "  --min-altitude-km=<km>       Minimum orbital altitude (default 0)\n"
        "  --max-altitude-km=<km>       Maximum orbital altitude (default 1000)\n"
        "  --min-elevation=<deg>        Minimum peak elevation (default 0)\n"
        "  --min-minutes=<n>            Minimum minutes until AOS (default 1)\n"
        "  --max-minutes=<n>            Maximum minutes until AOS (default 90)\n"
        "\n"
        "Other:\n"
        "  --verbose=<level>            Verbosity integer\n"
        "  --help                       Short help (this message)\n"
        "  --help-full                  Detailed help with keyboard layout\n",
        name, name, name, name,
        UPLINK_FREQ_MHZ, DOWNLINK_FREQ_MHZ);

    if (!full) return;

    fprintf(dest,
        "\n"
        "KEYBOARD (unlocked by default, press K to toggle lock state)\n"
        "\n"
        "  K         Toggle keyboard lock\n"
        "  T         Start tracking the current satellite\n"
        "  s         Stop tracking\n"
        "  r         Reset rotator to az=0, el=0\n"
        "  [         Nudge antenna azimuth -5 deg\n"
        "  ]         Nudge antenna azimuth +5 deg\n"
        "  q         Quit\n"
        "\n"
        "EXAMPLES\n"
        "\n"
        "  # Auto-pick next visible pass above 10 deg (rotator is on by default)\n"
        "  %s next --min-elevation=10 --min-minutes=10 --max-minutes=45\n"
        "\n"
        "  # Dry-run prediction on a dev host (no rotator hardware)\n"
        "  %s 'ISS (ZARYA)' --without-rotator\n"
        "\n"
        "  # Operator coordination (broadcasts state to b210_rx_tx + tx_frame_sdr)\n"
        "  %s next --control\n",
        name, name, name);
}

// --- ncurses ------------------------------------------------------

void init_window(void)
{
    // setlocale BEFORE initscr so ncurses knows the terminal can render
    // its alternate-character-set line glyphs (and UTF-8 elsewhere).
    // Without this, box() and friends emit the ACS fallback letters
    // (q for horizontal, x for vertical, lkjm for corners) instead of
    // line-drawing characters.
    setlocale(LC_ALL, "");

    initscr(); cbreak(); noecho();
    nonl();
    timeout(0);
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);
    // ncurses defaults ESCDELAY to 1000 ms — fine for distinguishing
    // bare Esc from the leading byte of a function-key sequence, but
    // makes Esc-to-cancel and arrow-key composition feel sluggish.
    // 25 ms is the conventional snappy value; any real escape sequence
    // arrives in a few ms so this isn't tight.
    set_escdelay(25);
    start_color();
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    curs_set(0);
}

// --- Reports -------------------------------------------------------

// Pure-render predictions panel — operator runs the SGP4 search
// upstream, viewer fills state.prediction from broadcast. No SGP4
// calls inside, no current-time reads, so both sides paint the same
// thing for the same input state.
static void render_predictions_panel(state_t *state, double jul_utc,
                                     int *print_row, int print_col)
{
    if (print_row == NULL) return;
    (void) jul_utc;
    int row = *print_row;
    int col = print_col;

    struct tm utc;
    UTC_Calendar_Now(&utc, NULL);
    char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    mvprintw(row++, col, "%15s   %d %s %04d %02d:%02d:%02d UTC", "date",
             utc.tm_mday, months[utc.tm_mon - 1], utc.tm_year,
             utc.tm_hour, utc.tm_min, utc.tm_sec);
    clrtoeol();

    row++;
    mvprintw(row++, col, "%15s   %s (%s)", "satellite",
             state->prediction.satellite_ephem.tle.sat_name,
             state->prediction.satellite_ephem.tle.idesg);
    clrtoeol();
    if (state->prediction.minutes_since_epoch / 1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attron(COLOR_PAIR(2));
    }
    mvprintw(row++, col, "%15s   %0.1f days", "epoch age",
             state->prediction.minutes_since_epoch / 1440.0);
    if (state->prediction.minutes_since_epoch / 1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attroff(COLOR_PAIR(2));
    }
    clrtoeol();

    if (state->in_pass) {
        mvprintw(row++, col, "%15s   %s", "status", "** IN PASS **");
        if (state->antenna_rotator.tracking) {
            printw(" (TRACKING)");
        } else {
            attron(COLOR_PAIR(1));
            printw(" (NOT tracking)");
            attroff(COLOR_PAIR(1));
        }
    } else {
        mvprintw(row++, col, "%15s   %s", "status", "** NOT in pass **");
    }
    clrtoeol();

    if (state->prediction.predicted_minutes_until_visible > 0) {
        if (state->prediction.predicted_minutes_until_visible < 1) {
            mvprintw(row++, col, "%15s   ", "next pass in");
            attron(COLOR_PAIR(2));
            printw("%.0f seconds",
                   floor(state->prediction.predicted_minutes_until_visible * 60.0));
            attroff(COLOR_PAIR(2));
        } else if (state->prediction.predicted_minutes_until_visible < 10) {
            mvprintw(row++, col, "%15s   %.1f minutes", "next pass in",
                     state->prediction.predicted_minutes_until_visible);
        } else {
            mvprintw(row++, col, "%15s   %.0f minutes", "next pass in",
                     state->prediction.predicted_minutes_until_visible);
        }
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   ", "elapsed time");
        attron(COLOR_PAIR(3));
        if (fabs(state->prediction.predicted_minutes_until_visible) < 1) {
            printw("%.0f seconds",
                   floor(-state->prediction.predicted_minutes_until_visible * 60.0));
        } else {
            printw("%.1f minutes",
                   -state->prediction.predicted_minutes_until_visible);
        }
        attroff(COLOR_PAIR(3));
        clrtoeol();
    }
    mvprintw(row++, col, "%15s   %.1f minutes", "duration",
             state->prediction.predicted_minutes_above_0_degrees);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f minutes", "el>30",
             state->prediction.predicted_minutes_above_30_degrees);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg", "max elevation",
             state->prediction.predicted_max_elevation);
    clrtoeol();

    *print_row = row;
}

// SGP4 work that report_predictions used to do inline. Operator calls
// this each tick so its state->prediction.predicted_* fields are
// fresh before render + broadcast.
static void compute_predictions(state_t *state, double jul_utc)
{
    minutes_until_visible(&state->prediction, jul_utc,
                          jul_utc + MAX_MINUTES_TO_PREDICT / 1440.0, 1.0);
    if (fabs(state->prediction.predicted_minutes_until_visible) < 1) {
        minutes_until_visible(&state->prediction, jul_utc,
                              jul_utc + 2.0 / 1440.0, 1. / 120.0);
    } else if (fabs(state->prediction.predicted_minutes_until_visible) < 10) {
        minutes_until_visible(&state->prediction, jul_utc,
                              jul_utc + 20.0 / 1440.0, 0.1);
    }
    if (state->prediction.predicted_minutes_until_visible > 0) {
        update_pass_predictions(&state->prediction,
            jul_utc + state->prediction.predicted_minutes_until_visible / 1440.0,
            0.1);
    } else if (state->prediction.predicted_max_elevation == -180.0) {
        // Started mid-pass: walk back to AOS so update_pass_predictions
        // captures the true max elevation rather than just the remainder.
        update_pass_predictions(&state->prediction,
            jul_utc + state->prediction.predicted_minutes_until_visible / 1440.0,
            0.1);
    }
}

void report_predictions(state_t *state, double jul_utc, int *print_row, int print_col)
{
    compute_predictions(state, jul_utc);
    render_predictions_panel(state, jul_utc, print_row, print_col);
}

// Render the operator/carrier/rotator status block. Caller supplies
// the values so this function works for both the operator (who reads
// the rotator from hardware) and the viewer (who pulls them from the
// IPC broadcast).
typedef struct {
    int    control_mode;     // 1 = operator process; 0 = viewer process
    const  char *operator_user;
    const  char *viewers;    // comma-separated viewer names, or "(none)"
    double carrier_hz;
    int    have_rotator;     // 1 -> render az/el block; 0 -> "not initialized"
    double current_az;
    double current_el;
    double target_az;
    double target_el;
    int    flip;
} status_panel_t;

static void render_status_panel(const status_panel_t *p,
                                int *print_row, int print_col)
{
    if (print_row == NULL) return;
    int row = *print_row;
    int col = print_col;

    mvprintw(0, 0, "%-15s %s   viewers: %s",
             "OPERATOR",
             p->operator_user ? p->operator_user : "?",
             p->viewers && p->viewers[0] ? p->viewers : "(none)");
    clrtoeol();

    mvprintw(row++, col, "%15s   %.6f MHz", "CARRIER", p->carrier_hz / 1e6);
    clrtoeol();
    row++;

    if (p->have_rotator) {
        double az_display = p->current_az;
        if (az_display < 0) az_display += 360.0;
        double target_az_display = p->target_az;
        if (target_az_display < 0) target_az_display += 360.0;
        const char *flip_tag = p->flip ? " (flip)" : "";
        mvprintw(row++, col, "%15s   %.1f deg%s", "target azimuth",
                 target_az_display, flip_tag);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "azimuth", az_display);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg%s", "target elevation",
                 p->target_el, flip_tag);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "elevation", p->current_el);
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   %s",
                 "antenna rotator", "* not initialized *");
        clrtoeol();
    }

    *print_row = row;
}

// Build a comma-separated list of currently-connected viewer/external
// clients. Skips anonymous (no-name) slots and is bounded by the
// caller's buffer.
static void operator_viewers_list(char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    if (!g_ipc) return;
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char user[64], role[16], since[40];
    size_t written = 0;
    while (sso_ipc_server_next_client(g_ipc, &it, &cid,
                                       user, sizeof user,
                                       role, sizeof role,
                                       since, sizeof since) == 0) {
        if (!user[0]) continue;
        size_t nlen = strlen(user);
        size_t need = nlen + (written > 0 ? 1 : 0);
        if (written + need + 1 >= out_size) break;
        if (written > 0) out[written++] = ',';
        memcpy(out + written, user, nlen);
        written += nlen;
        out[written] = '\0';
    }
}

void report_status(state_t *state, int *print_row, int print_col)
{
    if (print_row == NULL) return;

    static char viewers[256];
    operator_viewers_list(viewers, sizeof viewers);

    status_panel_t p;
    memset(&p, 0, sizeof p);
    p.control_mode  = g_control_mode;
    p.operator_user = g_operator_user;
    p.viewers       = viewers[0] ? viewers : "(none)";

    double display_dl_hz = state->doppler_downlink_frequency_hz;
    if (display_dl_hz == 0.0) display_dl_hz = state->nominal_downlink_frequency_hz;
    p.carrier_hz = display_dl_hz;

    p.have_rotator = state->have_antenna_rotator;
    if (state->have_antenna_rotator) {
        double azimuth = state->antenna_rotator.azimuth;
        double elevation = state->antenna_rotator.elevation;
        // The rotator STATUS command does a blocking read() on the
        // serial port (antenna_rotator.c:142) that can take hundreds of
        // milliseconds. Skip the live poll while the operator is typing
        // in the ":" prompt or the TX compose modal — the cached az/el
        // from the last poll is good enough for display, and the loop
        // needs to keep ticking at 50 Hz so each keystroke echoes
        // promptly. Rotator SET commands (the tracking output) are
        // unaffected; only the read-back blocks.
        if (!g_cmd_active && !g_tx_compose_active && !g_auto_tcmd_active) {
            if (antenna_rotator_command(&state->antenna_rotator,
                                        ANTENNA_ROTATOR_STATUS,
                                        &azimuth, &elevation) == ANTENNA_ROTATOR_OK) {
                state->antenna_rotator.azimuth   = azimuth;
                state->antenna_rotator.elevation = elevation;
            }
        }
        p.current_az = azimuth;
        p.current_el = elevation;
        p.target_az  = state->antenna_rotator.target_azimuth;
        p.target_el  = state->antenna_rotator.target_elevation;
        p.flip       = state->antenna_rotator.flip_mode_pass;
    }

    render_status_panel(&p, print_row, print_col);
}

void report_position(state_t *state, int *print_row, int print_col)
{
    if (print_row == NULL) {
        return;
    }
    int row = *print_row;
    int col = print_col;

    mvprintw(row++, col, "%15s   %.2f deg", "azimuth",
             state->prediction.satellite_ephem.azimuth);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f deg", "elevation",
             state->prediction.satellite_ephem.elevation);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km", "altitude",
             state->prediction.satellite_ephem.altitude_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg N", "latitude",
             state->prediction.satellite_ephem.latitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg E", "longitude",
             state->prediction.satellite_ephem.longitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "speed",
             state->prediction.satellite_ephem.speed_km_s);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f km", "range",
             state->prediction.satellite_ephem.range_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "range rate",
             state->prediction.satellite_ephem.range_rate_km_s);
    clrtoeol();

    *print_row = row;
}

// --- Viewer mode --------------------------------------------------
//
// Read-only mirror of the operator instance. The viewer does NOT run
// SGP4 and does NOT load a TLE — it just deposits every broadcast
// field into a state_t and calls the same render helpers the operator
// uses, so the two displays are byte-identical except for the help text.

static int    g_viewer_event_pending      = 0;
static int    g_viewer_has_state          = 0;
static char   g_viewer_operator[64]       = "";
static char   g_viewer_roster_json[1024]  = "";
static time_t g_viewer_last_event         = 0;
static int    g_viewer_running            = 1;
// Mirror of the operator's ":" prompt state. cmd_active = 1 between
// the first cmd-preview after :  and the cmd-executed that closes it.
// cmd_buf and cmd_status track g_cmd_buf / g_cmd_status verbatim so
// the viewer's bottom row matches the operator's exactly. Sized to the
// wire field (sso_event_t.cmd_text) so snprintf can't truncate.
static int    g_viewer_cmd_active         = 0;
static char   g_viewer_cmd_buf[160]       = "";
static char   g_viewer_cmd_status[160]    = "";
// Mirror of the operator's RX panel. Filled from STATE / WELCOME events;
// render_rx_panel reads it directly during viewer_render.
static rx_panel_data_t g_viewer_rx_panel;
// state_t whose fields the viewer mirrors from the broadcast each tick.
static state_t g_viewer_state;
static double  g_viewer_carrier_hz        = 0.0;
static double  g_viewer_jul_utc           = 0.0;
static int     g_viewer_has_rotator       = 0;
static char    g_viewer_tle_path[256]     = "";
static char    g_viewer_pass_folder[256]  = "";
// Take-control confirmation. Press 'c' once to arm, 'y' within
// CONFIRM_WINDOW_S seconds to commit. Anything else cancels.
#define VIEWER_CONFIRM_WINDOW_S 5
static time_t  g_viewer_confirm_until     = 0;

static void viewer_on_event(sso_ipc_client_t *cli, const sso_event_t *evt,
                            void *user)
{
    (void) cli;
    (void) user;
    if (evt->type == SSO_EVT_TX_COMMAND_PREVIEW
     || evt->type == SSO_EVT_TX_COMMAND_SENT
     || evt->type == SSO_EVT_TX_ACK) {
        tx_log_push(evt);
        g_viewer_last_event = time(NULL);
        g_viewer_event_pending = 1;
        return;
    }
    if (evt->type == SSO_EVT_CMD_PREVIEW) {
        g_viewer_cmd_active = 1;
        snprintf(g_viewer_cmd_buf, sizeof g_viewer_cmd_buf,
                 "%s", evt->cmd_text);
        g_viewer_last_event = time(NULL);
        g_viewer_event_pending = 1;
        return;
    }
    if (evt->type == SSO_EVT_CMD_EXECUTED) {
        // Empty cmd_text + empty cmd_status = Esc/cancel; clear the row.
        // Otherwise show the executed-command result string just like the
        // operator does after cmd_dispatch returns.
        g_viewer_cmd_active = 0;
        g_viewer_cmd_buf[0] = '\0';
        snprintf(g_viewer_cmd_status, sizeof g_viewer_cmd_status,
                 "%s", evt->cmd_status);
        g_viewer_last_event = time(NULL);
        g_viewer_event_pending = 1;
        return;
    }
    if (evt->type != SSO_EVT_STATE && evt->type != SSO_EVT_WELCOME) {
        return;
    }
    g_viewer_last_event = time(NULL);
    if (evt->operator_user[0]) {
        snprintf(g_viewer_operator, sizeof g_viewer_operator, "%s",
                 evt->operator_user);
    }
    if (evt->roster_json[0]) {
        snprintf(g_viewer_roster_json, sizeof g_viewer_roster_json, "%s",
                 evt->roster_json);
    }
    if (!evt->has_state) return;

    state_t *s = &g_viewer_state;
    snprintf(s->prediction.satellite_ephem.tle.sat_name,
             sizeof s->prediction.satellite_ephem.tle.sat_name, "%s",
             evt->satellite);
    snprintf(s->prediction.satellite_ephem.tle.idesg,
             sizeof s->prediction.satellite_ephem.tle.idesg, "%s",
             evt->idesg);
    s->prediction.minutes_since_epoch              = evt->epoch_min;
    s->prediction.predicted_minutes_until_visible  = evt->min_visible;
    s->prediction.predicted_minutes_above_0_degrees  = evt->min_above_0;
    s->prediction.predicted_minutes_above_30_degrees = evt->min_above_30;
    s->prediction.predicted_max_elevation          = evt->max_el;
    s->prediction.satellite_ephem.azimuth          = evt->pred_az;
    s->prediction.satellite_ephem.elevation        = evt->pred_el;
    s->prediction.satellite_ephem.altitude_km      = evt->alt_km;
    s->prediction.satellite_ephem.latitude         = evt->lat_deg;
    s->prediction.satellite_ephem.longitude        = evt->lon_deg;
    s->prediction.satellite_ephem.speed_km_s       = evt->speed_kms;
    s->prediction.satellite_ephem.range_km         = evt->range_km;
    s->prediction.satellite_ephem.range_rate_km_s  = evt->range_rate_kms;
    s->in_pass                                     = evt->in_pass;
    s->antenna_rotator.tracking                    = evt->tracking;
    s->antenna_rotator.azimuth                     = evt->az;
    s->antenna_rotator.elevation                   = evt->el;
    s->antenna_rotator.target_azimuth              = evt->target_az;
    s->antenna_rotator.target_elevation            = evt->target_el;
    s->antenna_rotator.flip_mode_pass              = evt->flip;

    g_viewer_has_rotator = evt->has_rotator;
    g_viewer_jul_utc     = evt->jul_utc;
    g_viewer_carrier_hz  = (evt->doppler_hz != 0.0)
        ? (double)evt->freq_hz + evt->doppler_hz
        : (double)evt->freq_hz;
    if (evt->tle_path[0]) {
        snprintf(g_viewer_tle_path, sizeof g_viewer_tle_path,
                 "%s", evt->tle_path);
    }
    if (evt->pass_folder[0]) {
        snprintf(g_viewer_pass_folder, sizeof g_viewer_pass_folder,
                 "%s", evt->pass_folder);
    }

    // Mirror the operator's RX panel from the broadcast. Wipe to zero
    // first so a slot that the operator hasn't decoded in this run
    // doesn't carry stale state from a previous event.
    memset(&g_viewer_rx_panel, 0, sizeof g_viewer_rx_panel);
    g_viewer_rx_panel.have_session  = evt->rx_have_session;
    snprintf(g_viewer_rx_panel.warning, sizeof g_viewer_rx_panel.warning,
             "%s", evt->rx_warning);
    if (evt->rx_have_session) {
        g_viewer_rx_panel.rec_active     = evt->rx_rec_active;
        g_viewer_rx_panel.rx_freq_hz     = evt->rx_freq_hz;
        g_viewer_rx_panel.peak_dbfs      = evt->rx_peak_dbfs;
        g_viewer_rx_panel.rms_dbfs       = evt->rx_rms_dbfs;
        g_viewer_rx_panel.frames_total   = (uint64_t) evt->rx_frames_total;
        g_viewer_rx_panel.frames_iq      = (uint64_t) evt->rx_frames_iq;
        g_viewer_rx_panel.frames_vit     = (uint64_t) evt->rx_frames_vit;
        snprintf(g_viewer_rx_panel.last_frame_summary,
                 sizeof g_viewer_rx_panel.last_frame_summary,
                 "%s", evt->rx_last_frame_summary);
        g_viewer_rx_panel.age_s = evt->rx_age_s;
        int slots = RX_PANEL_PT_COUNT < SSO_RX_PT_SLOTS
                  ? RX_PANEL_PT_COUNT : SSO_RX_PT_SLOTS;
        for (int s = 0; s < slots; ++s) {
            g_viewer_rx_panel.pt_count[s] = (uint64_t) evt->rx_pt_count[s];
            int pl = evt->rx_pt_payload_len[s];
            if (pl < 0) pl = 0;
            int copy = pl;
            if (copy > RX_PANEL_PAYLOAD_MAX) copy = RX_PANEL_PAYLOAD_MAX;
            g_viewer_rx_panel.pt_payload_len[s] = pl;
            memcpy(g_viewer_rx_panel.pt_payload[s],
                   evt->rx_pt_payload[s], (size_t) copy);
        }
        int rn = evt->rx_ribbon_n;
        if (rn > RIBBON_LEN) rn = RIBBON_LEN;
        g_viewer_rx_panel.ribbon_n = rn;
        memcpy(g_viewer_rx_panel.ribbon, evt->rx_ribbon, (size_t) rn);
        g_viewer_rx_panel.ribbon[rn] = '\0';
        memcpy(g_viewer_rx_panel.ribbon_peak, evt->rx_ribbon_peak,
               (size_t) rn * sizeof g_viewer_rx_panel.ribbon_peak[0]);
    }

    g_viewer_has_state   = 1;
    g_viewer_event_pending = 1;
}

// Format the roster array into "alice,bob,carol" for the header bar,
// skipping the operator (already shown separately) and any entry whose
// user is empty. The roster JSON is built by sso_event_set_roster with
// the schema [{"user":"...","role":"...","since":"..."},...], so we
// can scan for "user":"..." and "role":"..." pairs.
static void viewer_roster_users(char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    const char *p = g_viewer_roster_json;
    size_t written = 0;
    while ((p = strstr(p, "\"user\":\"")) != NULL) {
        p += 8;
        const char *uend = strchr(p, '"');
        if (!uend) break;
        char name[64];
        size_t nlen = (size_t)(uend - p);
        if (nlen >= sizeof name) nlen = sizeof name - 1;
        memcpy(name, p, nlen);
        name[nlen] = '\0';
        const char *q = strstr(uend, "\"role\":\"");
        const char *next = strstr(uend, "\"user\":\"");
        // role must belong to this object (before the next "user")
        int is_op = 0;
        if (q && (!next || q < next)) {
            q += 8;
            const char *rend = strchr(q, '"');
            if (rend && (size_t)(rend - q) == 8
                && memcmp(q, "operator", 8) == 0) {
                is_op = 1;
            }
        }
        p = uend;
        if (is_op || nlen == 0) continue;
        size_t need = nlen + (written > 0 ? 1 : 0);
        if (written + need + 1 >= out_size) break;
        if (written > 0) out[written++] = ',';
        memcpy(out + written, name, nlen);
        written += nlen;
        out[written] = '\0';
    }
}

// Read the operator's PID from /run/sso/simple_sat_ops.pid. Returns 0
// and sets *out_pid on success; -1 on missing/unreadable file.
static int read_operator_pid(pid_t *out_pid)
{
    char path[256];
    if (sso_ipc_pid_path(path, sizeof path, "simple_sat_ops") != 0)
        return -1;
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int pid = 0;
    int got = fscanf(f, "%d", &pid);
    fclose(f);
    if (got != 1 || pid <= 0) return -1;
    *out_pid = (pid_t) pid;
    return 0;
}

// Hand-off: send SIGUSR1 to the running operator, wait until its
// socket file goes away (= the process has finished cleanup), then
// re-exec this process as `simple_sat_ops --control` with the
// inherited TLE + pass folder so the new operator picks up where the
// old one left off. Does not return on success.
static void viewer_take_control(sso_ipc_client_t *cli, const char *argv0)
{
    if (!g_viewer_tle_path[0]) {
        fprintf(stderr,
            "simple_sat_ops viewer: no TLE path from operator yet — "
            "wait for a state broadcast and try again.\n");
        return;
    }
    pid_t op_pid = 0;
    if (read_operator_pid(&op_pid) != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: couldn't read operator pid file.\n");
        return;
    }
    if (kill(op_pid, SIGUSR1) != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: kill(%d, SIGUSR1) failed: %s\n",
            (int) op_pid, strerror(errno));
        return;
    }

    // Wait for the old operator to clean up and remove its socket.
    char sock[256];
    if (sso_ipc_socket_path(sock, sizeof sock, "simple_sat_ops") != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: socket path lookup failed.\n");
        return;
    }
    int waited_ms = 0;
    for (;;) {
        struct stat st;
        if (stat(sock, &st) != 0 && errno == ENOENT) break;
        if (waited_ms >= 5000) {
            fprintf(stderr,
                "simple_sat_ops viewer: timed out waiting for operator "
                "to release the socket (%s)\n", sock);
            return;
        }
        usleep(100000);
        waited_ms += 100;
    }

    // Close our viewer IPC + curses cleanly before exec'ing.
    sso_ipc_client_close(cli);
    endwin();

    // Re-exec self as --control with inherited TLE and pass folder.
    char tle_arg[280];
    char pass_arg[280];
    snprintf(tle_arg,  sizeof tle_arg,  "--tle=%s",         g_viewer_tle_path);
    snprintf(pass_arg, sizeof pass_arg, "--pass-folder=%s", g_viewer_pass_folder);
    char self_path[1024];
    ssize_t n = readlink("/proc/self/exe", self_path, sizeof self_path - 1);
    const char *exe = (n > 0) ? (self_path[n] = '\0', self_path) : argv0;
    char *new_argv[8];
    int ai = 0;
    new_argv[ai++] = (char *) exe;
    new_argv[ai++] = "--control";
    new_argv[ai++] = tle_arg;
    if (g_viewer_pass_folder[0]) new_argv[ai++] = pass_arg;
    new_argv[ai] = NULL;
    fprintf(stderr,
        "simple_sat_ops: taking control with %s%s%s\n",
        tle_arg,
        g_viewer_pass_folder[0] ? "  " : "",
        g_viewer_pass_folder[0] ? pass_arg : "");
    execv(exe, new_argv);
    // If we got here exec failed — best to bail loudly.
    fprintf(stderr,
        "simple_sat_ops viewer: execv %s failed: %s\n",
        exe, strerror(errno));
    exit(EXIT_FAILURE);
}

static void viewer_render(int connected)
{
    int cols = COLS;
    erase();

    if (!g_viewer_has_state) {
        mvprintw(2, 2, "(waiting for state from the operator...)");
    } else {
        int row = 1, col = 1;
        render_predictions_panel(&g_viewer_state, g_viewer_jul_utc,
                                 &row, col);

        char viewers[160];
        viewer_roster_users(viewers, sizeof viewers);
        int srow = row + 1;
        status_panel_t sp;
        memset(&sp, 0, sizeof sp);
        sp.control_mode  = 0;
        sp.operator_user = g_viewer_operator;
        sp.viewers       = viewers[0] ? viewers : "(none)";
        sp.carrier_hz    = g_viewer_carrier_hz;
        sp.have_rotator  = g_viewer_has_rotator;
        sp.current_az    = g_viewer_state.antenna_rotator.azimuth;
        sp.current_el    = g_viewer_state.antenna_rotator.elevation;
        sp.target_az     = g_viewer_state.antenna_rotator.target_azimuth;
        sp.target_el     = g_viewer_state.antenna_rotator.target_elevation;
        sp.flip          = g_viewer_state.antenna_rotator.flip_mode_pass;
        render_status_panel(&sp, &srow, col);

        int prow = 5;
        report_position(&g_viewer_state, &prow, 50);
        // RX panel directly below position (matches the operator's layout).
        prow++;
        render_rx_panel(&g_viewer_rx_panel, &prow, 50);

        // Vertical ribbon on the right edge, same placement as the
        // operator. The wire delivers the same '.'/'-' chars the
        // operator's collector built so both screens crawl in sync.
        int ribbon_col = COLS - 2;
        int ribbon_top = 1;
        int ribbon_bot = LINES - 2;
        if (ribbon_col >= 64 && ribbon_bot > ribbon_top) {
            render_ribbon_vertical(&g_viewer_rx_panel,
                                   ribbon_top, ribbon_bot, ribbon_col);
        }

        int tx_log_row = LINES - TX_LOG_SIZE - 2;
        if (tx_log_row >= 17) {
            render_tx_log_panel(tx_log_row, 1);
        }
    }

    time_t now = time(NULL);
    long stale_s = g_viewer_last_event > 0
        ? (long)(now - g_viewer_last_event)
        : -1;
    const char *status = !connected ? "DISCONNECTED"
                                    : (stale_s < 0 ? "WAITING"
                                                   : (stale_s > 5 ? "STALE"
                                                                  : "LIVE"));

    // Bottom row priority: take-control confirm > cmd mirror > footer.
    // The cmd mirror reproduces exactly what the operator sees on its
    // own LINES-1; the footer (connection status + viewer-only shortcuts)
    // is the fallback when neither is active. The mirror does not invert
    // the row — matches the operator's plain ":" prompt rendering.
    int show_confirm = (g_viewer_confirm_until > 0
                        && now < g_viewer_confirm_until);
    int show_mirror  = !show_confirm
        && (g_viewer_cmd_active || g_viewer_cmd_status[0]);

    move(LINES - 1, 0);
    clrtoeol();
    if (show_mirror) {
        if (g_viewer_cmd_active) {
            mvprintw(LINES - 1, 0, ":%s", g_viewer_cmd_buf);
            addch(' ' | A_REVERSE);
        } else {
            mvprintw(LINES - 1, 0, "%s", g_viewer_cmd_status);
        }
    } else {
        attron(A_REVERSE);
        char foot[200];
        if (show_confirm) {
            snprintf(foot, sizeof foot,
                " %s     Take control from %s? y/N ",
                status,
                g_viewer_operator[0] ? g_viewer_operator : "?");
        } else {
            snprintf(foot, sizeof foot,
                " %s     c : take control   q : quit ", status);
        }
        int flen = (int)strlen(foot);
        if (flen > cols) flen = cols;
        mvaddnstr(LINES - 1, 0, foot, flen);
        for (int i = flen; i < cols; i++) mvaddch(LINES - 1, i, ' ');
        attroff(A_REVERSE);
    }

    refresh();
}

static int run_viewer(const char *argv0)
{
    sso_ipc_client_t *cli = sso_ipc_client_connect("simple_sat_ops");
    if (cli == NULL) {
        fprintf(stderr,
                "simple_sat_ops viewer: connect failed: %s\n",
                strerror(errno));
        return EXIT_FAILURE;
    }
    sso_ipc_client_on_event(cli, viewer_on_event, NULL);

    // Viewer doesn't run SGP4 or load a TLE — it deposits every
    // displayed value into g_viewer_state from the broadcast and uses
    // the same render helpers the operator does. zero-init is enough.
    memset(&g_viewer_state, 0, sizeof g_viewer_state);

    sso_event_t hello;
    sso_event_init(&hello, SSO_EVT_HELLO);
    snprintf(hello.role, sizeof hello.role, "viewer");
    const char *me = getenv("USER");
    if (!me || !me[0]) me = sso_unix_user();
    snprintf(hello.user, sizeof hello.user, "%s", me ? me : "?");
    char buf[1024];
    if (sso_event_encode(&hello, buf, sizeof buf) == 0) {
        sso_ipc_client_send(cli, buf);
    }

    init_window();
    int last_connected = -1;
    time_t last_render = 0;
    viewer_render(sso_ipc_client_is_connected(cli));
    last_render = time(NULL);
    int confirm_was_armed = 0;
    while (g_viewer_running) {
        int rc = sso_ipc_client_step(cli, 200);
        if (rc < 0) break;
        int connected = sso_ipc_client_is_connected(cli);
        time_t now = time(NULL);
        int confirm_armed = (g_viewer_confirm_until > 0
                             && now < g_viewer_confirm_until);
        if (!confirm_armed && confirm_was_armed) {
            // Window just expired — re-render to drop the confirm footer.
            g_viewer_event_pending = 1;
        }
        confirm_was_armed = confirm_armed;
        if (g_viewer_event_pending
            || connected != last_connected
            || (now - last_render) >= 5) {
            viewer_render(connected);
            g_viewer_event_pending = 0;
            last_connected = connected;
            last_render = now;
        }
        int key = getch();
        if (key == ERR) continue;
        if (confirm_armed) {
            if (key == 'y' || key == 'Y') {
                // Commits: viewer_take_control re-execs on success and
                // doesn't return; if it returns, the operator wasn't
                // reachable and we just stay as a viewer.
                viewer_take_control(cli, argv0);
                g_viewer_confirm_until = 0;
                g_viewer_event_pending = 1;
            } else {
                // Anything else cancels the confirm window.
                g_viewer_confirm_until = 0;
                g_viewer_event_pending = 1;
            }
            continue;
        }
        if (key == 'q' || key == 'Q' || key == 27 /* Esc */) {
            g_viewer_running = 0;
        } else if (key == 'c' || key == 'C') {
            g_viewer_confirm_until = now + VIEWER_CONFIRM_WINDOW_S;
            g_viewer_event_pending = 1;
        }
    }

    endwin();
    sso_ipc_client_close(cli);
    return 0;
}

// --- main ---------------------------------------------------------

int main(int argc, char **argv)
{
    state_t state = {0};
    state.prediction.predicted_max_elevation = -180.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);
    int status = apply_args(&state, argc, argv, jul_utc);
    if (status != 0) {
        return status;
    }

    // Bare invocation found a running operator — run as a read-only
    // viewer and skip the rest of the operator/standalone bring-up.
    if (g_viewer_mode) {
        return run_viewer(argv[0]);
    }

    // Audit + operator IPC bring-up.
    g_operator_user = sso_unix_user();
    sso_audit_start("simple_sat_ops",
                    g_control_mode ? "operator" : "standalone");
    if (g_control_mode) {
        g_ipc = sso_ipc_server_open("simple_sat_ops");
        if (g_ipc == NULL) {
            fprintf(stderr, "simple_sat_ops: --control requested but socket "
                            "bind failed (already running? check "
                            "/run/sso/simple_sat_ops.{sock,pid}). "
                            "Continuing without IPC.\n");
            sso_audit_event("ipc-bind-failed", "");
        } else {
            sso_ipc_server_on_event(g_ipc, ipc_on_event, NULL);
            struct sigaction sa;
            memset(&sa, 0, sizeof(sa));
            sa.sa_handler = on_sigusr1;
            sigemptyset(&sa.sa_mask);
            sigaction(SIGUSR1, &sa, NULL);
            fprintf(stderr, "simple_sat_ops: operator=%s ipc=on\n",
                    g_operator_user);
        }
    }

    /* Parse TLE data */
    int tle_status = load_tle(&state.prediction);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.prediction.satellite_ephem.tle);

    // With a fresh TLE loaded, find the upcoming pass and stand up
    // /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/ for it before the
    // tracking loop opens ncurses. Only on --control — the
    // standalone-tracker / dev path leaves Operations/ alone.
    if (g_control_mode) {
        UTC_Calendar_Now(&utc, &tv);
        double jul_now = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_now);
        setup_pass_folder(&state, jul_now);
        if (g_pass_folder[0]) {
            generate_pass_plot(&state, g_pass_folder, jul_now);
        }
    }

    int antenna_rotator_result = 0;
    if (state.run_with_antenna_rotator) {
        state.antenna_rotator.is_required = 1;
        antenna_rotator_result = antenna_rotator_init(&state.antenna_rotator);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error initializing antenna rotator\n");
            return EXIT_FAILURE;
        }
        state.have_antenna_rotator = 1;
        // Adopt whatever extended position the SPID is already at so the
        // unwrapped accumulator starts grounded in reality.
        if (antenna_rotator_seed_from_status(&state.antenna_rotator) != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Warning: could not read SPID position; "
                            "check that the Rot2ProG is in 'A' mode\n");
        }
    }

#ifdef WITH_USRP_B210
    // Open the B210 once, here, before ncurses init — soft-fail on any
    // UHD error so a dev host without a device can still run the UI.
    // rx_session takes ownership of the core; we drop our local handle
    // afterwards so main never touches UHD off-thread.
    if (g_control_mode && !g_without_b210) {
        b210_rx_tx_core_params_t cp = {
            .freq_hz         = state.nominal_downlink_frequency_hz,
            .rate_hz         = 240000.0,
            .gain_db         = 50.0,
            .bw_hz           = -1.0,
            .fm_fullscale_hz = 25000.0,
            .device_args     = "type=b200",
            .rx_antenna      = "RX2",
            .decim_factor    = 5u,
            .decim_cutoff_hz = 18000.0,
            .decim_taps      = 96u,
        };
        b210_rx_tx_core_t *core = NULL;
        if (b210_rx_tx_core_open(&cp, &core) != 0) {
            fprintf(stderr,
                "simple_sat_ops: B210 open failed — continuing without RF "
                "(rotator + UI only). Pass --without-b210 to silence.\n");
        } else {
            fprintf(stderr,
                "simple_sat_ops: B210 open at %.6f MHz (post-decim rate %.0f)\n",
                b210_rx_tx_core_actual_freq(core) / 1e6,
                b210_rx_tx_core_actual_rate(core));
            rx_session_params_t rxp = {
                .bit_rate          = 9600,
                .window_s          = 1.5,
                .slide_s           = 0.5,
                .sync_max_ham      = 4,
                .use_hmac          = 0,
                .use_rs            = 1,
                .force_beacon      = 0,
                .show_packet_headers = 0,
                .csp_crc32         = 0,
                .pass_folder       = g_pass_folder[0] ? g_pass_folder : NULL,
                .want_wav          = 1,
                .tle_path          = state.prediction.tles_filename,
                .sat_name          = state.prediction.satellite_ephem.tle.sat_name[0]
                                     ? state.prediction.satellite_ephem.tle.sat_name
                                     : NULL,
                .session_dir       = g_pass_folder[0] ? g_pass_folder : NULL,
            };
            if (rx_session_open(&g_rx_session, &rxp, core) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: rx_session_open failed — closing B210\n");
                b210_rx_tx_core_close(core);
            }
            // rx_session_open succeeded → it owns `core` now.
            core = NULL;
        }
    }
#endif

    /* Tracking loop */
    double jul_idle_start = 0;  // last-tracked timestamp

    init_window();

    char key = '\0';
    int row = 0;
    int col = 2;
    state.running = 1;

    double current_uplink_frequency = state.nominal_uplink_frequency_hz;
    double current_downlink_frequency = state.nominal_downlink_frequency_hz;
    double doppler_delta_uplink = 0.0;
    double doppler_delta_downlink = 0.0;
    double doppler_max_delta = DOPPLER_SHIFT_RESOLUTION_KHZ * 1000.0;
    (void) doppler_delta_uplink;  // tracked for display symmetry / future IPC
    (void) doppler_max_delta;     // threshold for any future on-display retune

    double delta_az = 0.0;
    double delta_el = 0.0;

    state.antenna_rotator.antenna_should_be_controlled =
        state.run_with_antenna_rotator && state.have_antenna_rotator;
    state.antenna_rotator.antenna_is_under_control =
        state.antenna_rotator.antenna_should_be_controlled;

    int keyboard_unlocked = 1;
    int keyboard_info_row = 20;

    mvprintw(keyboard_info_row++, 3, "%s", "T  - Track satellite");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "s  - Stop antenna immediately");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "r  - Reset to az=0 el=0");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "[  - Jog azimuth -5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "]  - Jog azimuth +5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "t  - Compose TX command");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "A  - Auto-TCMD (needs --tc-file=)");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "K  - Lock/unlock keyboard");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "q  - Quit");
    clrtoeol();

    double current_az = 0;
    double current_el = 0;
    double last_az = 0;
    double last_el = 0;

    // Slow-cadence work is timestamp-gated so the fast UHD-pump loop
    // doesn't spam viewers or burn CPU on ncurses redraws.
    //
    // Two redraw gates: the "slow" one drives the predictions / status
    // / RX panel / TX log rows and is the one that costs CPU — most
    // notably report_status, which does a blocking read() against the
    // rotator serial port (antenna_rotator.c:142). Keep it at 2 Hz so
    // the rotator isn't hammered. The "fast" path is cmd_render only
    // — runs every loop tick while the operator is typing in the ":"
    // prompt so each keystroke echoes immediately.
    double t_last_ipc_broadcast = 0.0;
    double t_last_redraw        = 0.0;
    const double IPC_BROADCAST_PERIOD_S = 0.5;   // 2 Hz
    const double REDRAW_PERIOD_S        = 0.5;   // 2 Hz

    // Per-pass WAV recording: arm 1 min before AOS, hold open through
    // the pass, close 1 min after LOS. Multiple passes during one
    // simple_sat_ops run each get their own file under the pass folder.
    const double RECORDING_PREROLL_S  = 60.0;
    const double RECORDING_POSTROLL_S = 60.0;
    double t_recording_close_at = 0.0;  // monotonic deadline; 0 = unset

    while (state.running) {
        double t_now = monotonic_seconds();
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_utc);

        /* Calculate Doppler shift for the display + IPC publishing */
        if (state.doppler_correction_enabled) {
            update_doppler_shifted_frequencies(&state,
                                                state.nominal_uplink_frequency_hz,
                                                state.nominal_downlink_frequency_hz);
            doppler_delta_uplink   = fabs(state.doppler_uplink_frequency_hz
                                           - current_uplink_frequency);
            doppler_delta_downlink = fabs(state.doppler_downlink_frequency_hz
                                           - current_downlink_frequency);
            current_uplink_frequency   = state.doppler_uplink_frequency_hz;
            current_downlink_frequency = state.doppler_downlink_frequency_hz;
        }

#ifdef WITH_USRP_B210
        // Auto-record per pass: open the WAV 1 min before AOS (or as
        // soon as we're above the horizon, in case simple_sat_ops
        // started mid-pass), keep it open while the satellite is up,
        // close 1 min after LOS. Each pass gets its own auto-named
        // file in the pass folder. Note: this deliberately keys off the
        // satellite geometry (elevation + time-until-AOS) rather than
        // state.in_pass — the latter flips several minutes before AOS
        // (tracking_prep_time_minutes) so the rotator can pre-position,
        // which is far too early to start the WAV.
        if (g_rx_session) {
            double sec_to_aos =
                state.prediction.predicted_minutes_until_visible * 60.0;
            int visible   = (state.prediction.satellite_ephem.elevation > 0.0);
            int in_preroll = (sec_to_aos > 0.0
                              && sec_to_aos <= RECORDING_PREROLL_S);
            int active = rx_session_wav_active(g_rx_session);
            if (!active && (visible || in_preroll)) {
                rx_session_request_wav_start(g_rx_session);
                t_recording_close_at = 0.0;
            } else if (active) {
                if (visible) {
                    t_recording_close_at = 0.0;  // cancel any pending close
                } else if (t_recording_close_at == 0.0) {
                    t_recording_close_at = t_now + RECORDING_POSTROLL_S;
                } else if (t_now >= t_recording_close_at) {
                    rx_session_request_wav_stop(g_rx_session);
                    t_recording_close_at = 0.0;
                }
            }
        }
#endif

        current_az = state.antenna_rotator.azimuth;
        current_el = state.antenna_rotator.elevation;
        if (state.antenna_rotator.antenna_is_moving) {
            if (fabs(current_az - last_az) == 0
                && fabs(current_el - last_el) == 0) {
                state.antenna_rotator.antenna_is_moving = 0;
            }
            last_az = current_az;
            last_el = current_el;
        }

        // Drive the second leg of a two-step home once the first leg has stopped.
        if (state.antenna_rotator.homing_in_progress
            && !state.antenna_rotator.antenna_is_moving
            && state.have_antenna_rotator) {
            double final_az = state.antenna_rotator.home_pending_final_az;
            int rc = antenna_rotator_set_unwrapped(&state.antenna_rotator,
                                                    final_az, 0.0);
            if (rc == ANTENNA_ROTATOR_OK) {
                state.antenna_rotator.antenna_is_moving = 1;
            }
            state.antenna_rotator.homing_in_progress = 0;
            state.antenna_rotator.home_pending_final_az = 0.0;
        }
        if (state.satellite_tracking
            && state.prediction.predicted_minutes_until_visible
                   < state.antenna_rotator.tracking_prep_time_minutes) {
            if (!state.in_pass) {
                state.in_pass = 1;
            }
            if (state.antenna_rotator.antenna_should_be_controlled
                && !state.antenna_rotator.tracking) {
                if (!state.antenna_rotator.fixed_target
                    && !state.antenna_rotator.flip_decision_made) {
                    state.antenna_rotator.flip_mode_pass = 0;
                    state.antenna_rotator.flip_half = 0;
                    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION > 90
                        && state.prediction.predicted_max_elevation
                               >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD) {
                        state.antenna_rotator.flip_mode_pass = 1;
                        // Prefer the prediction-derived AOS azimuth (the
                        // satellite_ephem.azimuth here may be a few deg
                        // off as we are still pre-AOS); fall back to the
                        // live position if the pass walk didn't capture
                        // an ascension sample.
                        double aos_az_pred =
                            state.prediction.predicted_ascension_azimuth;
                        state.antenna_rotator.flip_aos_az =
                            (aos_az_pred != 0.0)
                                ? aos_az_pred
                                : state.prediction.satellite_ephem.azimuth;
                        state.antenna_rotator.flip_los_az =
                            state.prediction.predicted_descent_azimuth;
                        state.antenna_rotator.flip_aos_jul =
                            state.prediction.predicted_ascension_jul_utc;
                        state.antenna_rotator.flip_los_jul =
                            state.prediction.predicted_descent_jul_utc;
                    }
                    state.antenna_rotator.flip_decision_made = 1;
                    state.antenna_rotator.tracking = 1;
                }
            }

            if (state.antenna_rotator.tracking
                && state.antenna_rotator.antenna_is_under_control) {
                if (!state.antenna_rotator.unwrapped_target_valid) {
                    if (antenna_rotator_seed_from_status(&state.antenna_rotator)
                        != ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.tracking = 0;
                    }
                } else if (!state.antenna_rotator.antenna_is_moving) {
                    double pred_az = state.prediction.satellite_ephem.azimuth;
                    double pred_el = state.prediction.satellite_ephem.elevation;
                    double mech_az = pred_az;
                    double mech_el = pred_el;
                    int half = 0;
                    // AOS->LOS progress: drives the boom-meridian lerp
                    // in flip mode. Clamped to [0, 1] inside the
                    // function. For non-flip passes (and any tick
                    // where the AOS/LOS times weren't captured) the
                    // value is ignored.
                    double progress = 0.0;
                    double pass_jul =
                        state.antenna_rotator.flip_los_jul
                        - state.antenna_rotator.flip_aos_jul;
                    if (pass_jul > 0.0) {
                        progress = (jul_utc
                                    - state.antenna_rotator.flip_aos_jul)
                                   / pass_jul;
                    }
                    antenna_rotator_to_mech_coords(
                        state.antenna_rotator.flip_mode_pass,
                        state.antenna_rotator.flip_aos_az,
                        state.antenna_rotator.flip_los_az,
                        progress,
                        pred_az, pred_el,
                        &mech_az, &mech_el, &half);
                    if (state.antenna_rotator.flip_mode_pass
                        && half != state.antenna_rotator.flip_half) {
                        state.antenna_rotator.target_azimuth_unwrapped = mech_az;
                        state.antenna_rotator.flip_half = half;
                    }
                    double prev_unwrapped =
                        state.antenna_rotator.target_azimuth_unwrapped;
                    double next_az = antenna_rotator_accumulate_unwrapped(
                        prev_unwrapped, mech_az);
                    double next_el = mech_el;
                    if (next_el < ANTENNA_ROTATOR_MINIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MINIMUM_ELEVATION;
                    } else if (next_el > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                    }
                    delta_az = next_az - prev_unwrapped;
                    if (state.antenna_rotator.flip_mode_pass
                        || state.prediction.satellite_ephem.elevation >= 0) {
                        delta_el = next_el
                                   - state.antenna_rotator.target_elevation;
                    } else {
                        delta_el = 0.0;
                    }

                    if (fabs(delta_az) >= MAX_DELTA_AZIMUTH_DEGREES
                        || fabs(delta_el) >= MAX_DELTA_ELEVATION_DEGREES) {
                        if (next_az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH
                            || next_az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                            state.antenna_rotator.tracking = 0;
                        } else {
                            int rc = antenna_rotator_set_unwrapped(
                                &state.antenna_rotator, next_az, next_el);
                            if (rc != ANTENNA_ROTATOR_OK) {
                                fprintf(stderr,
                                        "Error setting antenna rotator position\n");
                            } else {
                                state.antenna_rotator.antenna_is_moving = 1;
                            }
                        }
                    }
                }
            }

            jul_idle_start = 0;
        } else {
            if (state.in_pass) {
                state.in_pass = 0;
                jul_idle_start = jul_utc;
            }
            if (state.antenna_rotator.tracking) {
                state.antenna_rotator.tracking = 0;
                state.antenna_rotator.flip_mode_pass = 0;
                state.antenna_rotator.flip_decision_made = 0;
                state.antenna_rotator.flip_half = 0;
            }
        }
        (void) jul_idle_start;  // reserved for any future idle-window behavior

        int redraw_due = (t_now - t_last_redraw) >= REDRAW_PERIOD_S;
        if (redraw_due) {
            row = 1;
            col = 1;
            report_predictions(&state, jul_utc, &row, col);

            row++;
            report_status(&state, &row, col);
            row = 5;
            col = 50;
            report_position(&state, &row, col);
            row++;
            // Refresh the low-disk warning lazily — statvfs every 30 s
            // is plenty given how slowly disk fills.
            low_disk_refresh(t_now);
            rx_panel_data_t rxd;
            rx_panel_collect_local(&rxd);
            render_rx_panel(&rxd, &row, 50);

            clrtoeol();

            // Vertical ribbon on the right edge — bottom = newest, with
            // a bold '-' tick crawling up one row per second so the
            // timeline is visibly alive even when the signal is flat.
            int ribbon_col = COLS - 2;
            int ribbon_top = 1;
            int ribbon_bot = LINES - 2;
            if (ribbon_col >= 64 && ribbon_bot > ribbon_top) {
                render_ribbon_vertical(&rxd, ribbon_top, ribbon_bot, ribbon_col);
            }

            // TX log lives below the keyboard info / antenna status if
            // the terminal is tall enough to host it without colliding.
            int tx_log_row = LINES - TX_LOG_SIZE - 2;
            if (tx_log_row >= keyboard_info_row + 4) {
                render_tx_log_panel(tx_log_row, 1);
            }
        }

        key = getch();
        if (g_tx_compose_active) {
            if (!tx_compose_handle_key(key)) {
                tx_compose_close();
            }
        } else if (g_auto_tcmd_active) {
            if (!auto_tcmd_handle_key(key)) {
                auto_tcmd_close();
            }
        } else if (g_cmd_active) {
            cmd_handle_key(key, &state);
        } else if (key == 'K') {
            keyboard_unlocked = !keyboard_unlocked;
        } else if (keyboard_unlocked) {
            switch (key) {
                case ':':
                    cmd_enter();
                    break;
                case 'q':
                    state.running = 0;
                    break;
                case 'T':
                    start_tracking(&state);
                    break;
                case 's':
                    stop_tracking(&state);
                    break;
                case 'r':
                    stop_tracking(&state);
                    point_to_stationary_target(&state, 0.0, 0.0);
                    break;
                case '[':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = antenna_rotator_increase_azimuth(
                        &state.antenna_rotator, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ']':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = antenna_rotator_increase_azimuth(
                        &state.antenna_rotator, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case 't':
                    tx_compose_open();
                    break;
                case 'A':
                    auto_tcmd_open();
                    break;
                default:
                    break;
            }
        }

        if (redraw_due) {
            mvprintw(keyboard_info_row, 3, "%s : %s", "Keyboard",
                     keyboard_unlocked ? "unlocked" : "LOCKED");
            clrtoeol();
            if (state.antenna_rotator.antenna_is_moving) {
                mvprintw(keyboard_info_row + 2, 0, "%s", "Antenna moving");
                clrtoeol();
            } else {
                mvprintw(keyboard_info_row + 2, 0, "%s", "Antenna stationary");
                clrtoeol();
            }
            t_last_redraw = t_now;
        }

        // Pump the modal's debounced preview broadcast before the
        // screen flush so the mirror line is current when we paint.
        tx_compose_pump();
        // Drive the auto-tcmd burst loop. Queues g_tx_request when
        // it's time for the next send; the existing main-loop burst
        // handler below transmits and emits the SENT/ACK events.
        auto_tcmd_tick(&state);

        // Bottom-row prompt + screen flush. When the operator is typing
        // in the ":" prompt we want this every tick (~50 Hz) so each
        // keystroke echoes immediately. Otherwise piggyback on the slow
        // redraw so the row picks up any post-command status string.
        // When a modal is open we force-redraw it on top of stdscr by
        // touchwin'ing every modal cell as dirty and wrefresh'ing the
        // window after stdscr's own refresh. doupdate's incremental
        // diff is otherwise free to skip "unchanged" modal cells, which
        // is what was letting panel updates (e.g. the antenna status
        // row) bleed through and overwrite the modal.
        if (redraw_due || g_cmd_active || g_tx_compose_active
            || g_auto_tcmd_active) {
            cmd_render();
            refresh();
            int show_hw_cursor = 0;
            if (g_tx_compose_active && g_tx_compose_win) {
                touchwin(g_tx_compose_win);
                wrefresh(g_tx_compose_win);
                tx_field_t f = g_tx_compose_state.focus;
                show_hw_cursor = (f == TXF_PAYLOAD || f == TXF_POWER);
            } else if (g_auto_tcmd_active && g_auto_tcmd_win) {
                touchwin(g_auto_tcmd_win);
                wrefresh(g_auto_tcmd_win);
                show_hw_cursor = (g_auto_tcmd.state != AUTO_STATE_RUNNING)
                              && auto_field_is_text(g_auto_tcmd.focus);
            } else if (g_cmd_active) {
                show_hw_cursor = 1;
            }
            curs_set(show_hw_cursor ? 1 : 0);
        }

#ifdef WITH_USRP_B210
        // Doppler retune — queued for the B210 worker. We read the
        // current actual_freq from the snapshot instead of caching it
        // here, since the worker thread owns the core's tune state.
        double snap_freq_hz = 0.0;
        if (g_rx_session) {
            rx_session_snapshot(g_rx_session, NULL, NULL, NULL,
                                &snap_freq_hz, NULL, 0);
        }
        // Signal ribbon sampler: push one peak-dBFS reading per second
        // so the ribbon in the RX panel rolls left in real time.
        if (g_rx_session && (t_now - g_ribbon_last_t) >= 1.0) {
            double peak = -90.0;
            rx_session_snapshot(g_rx_session, NULL, &peak, NULL,
                                NULL, NULL, 0);
            ribbon_push(peak);
            g_ribbon_last_t = t_now;
        }
        if (g_rx_session
            && state.doppler_correction_enabled
            && fabs(state.doppler_downlink_frequency_hz - snap_freq_hz)
                   >= DOPPLER_SHIFT_RESOLUTION_KHZ * 1000.0) {
            rx_session_request_freq(g_rx_session,
                                     state.doppler_downlink_frequency_hz);
        }
        if (g_rx_session) {
            double doppler_offset =
                state.doppler_downlink_frequency_hz
                - state.nominal_downlink_frequency_hz;
            rx_session_update_observer(g_rx_session,
                state.antenna_rotator.target_azimuth,
                state.antenna_rotator.target_elevation,
                state.prediction.satellite_ephem.range_km,
                state.prediction.satellite_ephem.range_rate_km_s,
                doppler_offset);
        }

        // Synchronously service any pending TX request. The B210
        // worker pauses RX, transmits, resumes RX, then unblocks us.
        // We block here for the burst duration (~1 s) — that's
        // intentional; the operator just hit Enter and is staring at
        // the screen waiting for confirmation.
        if (g_tx_request.pending && g_rx_session) {
            char summary[160];
            rx_burst_result_t br = rx_session_request_burst_sync(
                g_rx_session, &g_tx_request, NULL, 0,
                summary, sizeof summary);
            const char *ack = "ok";
            switch (br) {
                case RX_BURST_OK: break;
                case RX_BURST_NO_CORE:            ack = "rejected: no B210"; break;
                case RX_BURST_FRAME_BUILD_FAILED: ack = "rejected: frame build"; break;
                case RX_BURST_UHD_ERROR:          ack = "uhd-err"; break;
            }
            emit_tx_event_local(SSO_EVT_TX_ACK, summary, ack);
            if (br == RX_BURST_OK) {
                emit_tx_event_local(SSO_EVT_TX_COMMAND_SENT, summary, NULL);
            }
            g_tx_request.pending = 0;
        }
#endif

        // --- IPC: serve clients, fan out state, honour SIGUSR1 yield ---
        // Always service the socket (cheap; accepts new viewers) but
        // throttle STATE broadcasts to 2 Hz so viewers don't get
        // hammered when the loop is running at UHD-chunk cadence.
        if (g_ipc) {
            sso_ipc_server_step(g_ipc, 0);
            if ((t_now - t_last_ipc_broadcast) >= IPC_BROADCAST_PERIOD_S) {
                ipc_broadcast_state(&state, current_az, current_el,
                                     current_downlink_frequency,
                                     doppler_delta_downlink,
                                     jul_utc);
                t_last_ipc_broadcast = t_now;
            }
            // Debounced cmd-preview broadcast: viewers see the operator's
            // ":" prompt as it's typed, lagging by g_cmd_debounce_ns so we
            // don't fire a packet per keystroke.
            if (g_cmd_active && g_cmd_dirty
                && (cmd_now_ns() - g_cmd_last_edit_ns) >= g_cmd_debounce_ns) {
                cmd_broadcast_preview();
                g_cmd_dirty = 0;
            }
        }
        if (g_yield_requested) {
            sso_audit_event("yield-requested",
                            "SIGUSR1 (--force takeover) — exiting");
            state.running = 0;
        }

        // Surface a finished spectrum render so the operator sees the
        // outcome (PNG path or ffmpeg error) in the command-line status.
        // The reap only joins the worker thread; status_msg is left
        // alone, so reading it after reap is safe.
        if (g_spec_job.active && g_spec_job.done) {
            if (g_spec_job.status_msg[0]) {
                cmd_set_status("%s", g_spec_job.status_msg);
            }
            spectrum_job_reap();
        }

        if (state.running) {
            // The B210 worker thread pumps UHD on its own pthread now,
            // so the main loop doesn't pace itself off the radio. Sleep
            // at the historical 2 Hz so rotator-STATUS polls don't ramp
            // up unexpectedly; redraw/IPC gates do their own throttling.
            // Exception: while the operator is typing in the ":" prompt,
            // drop to 20 ms so getch() echoes each keystroke promptly
            // (the 500 ms tick was capping input at ~2 chars/sec).
            usleep((g_cmd_active || g_tx_compose_active || g_auto_tcmd_active)
                   ? 20000 : UPDATE_INTERVAL_MICROSEC);
        }
    }

    endwin();
    if (g_ipc) {
        sso_ipc_server_close(g_ipc);
        g_ipc = NULL;
    }
#ifdef WITH_USRP_B210
    char final_wav_path[512] = "";
    char final_iq_path[512]  = "";
    int  final_iq_rate       = 0;
    if (g_rx_session) {
        // Snapshot both sidecar paths before close so the full-pass
        // renderers can find the closed files on disk. Both paths
        // persist across wav_stop in rx_session.
        rx_session_wav_snapshot(g_rx_session,
                                final_wav_path, sizeof final_wav_path,
                                NULL, NULL, NULL);
        rx_session_iq_snapshot(g_rx_session,
                               final_iq_path, sizeof final_iq_path,
                               NULL, &final_iq_rate);
        // The worker owns the WAV writer and the B210 core. Closing
        // the session signals the worker to stop, joins it, then
        // tears down both. Any open WAV / .iq gets its header patched
        // (WAV) or its trailer flushed (IQ).
        rx_session_request_wav_stop(g_rx_session);
        rx_session_close(g_rx_session);
        g_rx_session = NULL;
    }

    // Any in-flight `:spectrum N` worker is touching the same WAV / IQ
    // — let it finish before we hand the file to the full-pass render.
    if (g_spec_job.active) {
        pthread_join(g_spec_job.thr, NULL);
        g_spec_job.active = 0;
        if (g_spec_job.status_msg[0]) {
            fprintf(stderr, "simple_sat_ops: %s\n", g_spec_job.status_msg);
        }
    }

    // Full-pass renderer. Prefer the IQ → gen_waterfall path because it
    // gives SatNOGS-style waterfalls (real complex FFT, median-subtracted
    // floor, viridis). Fall back to the FM-demod WAV via ffmpeg when the
    // IQ sidecar isn't on disk (e.g., disk full, mid-pass shutdown).
    int did_iq = 0;
    if (final_iq_path[0] && final_iq_rate > 0) {
        struct stat st;
        if (stat(final_iq_path, &st) == 0 && st.st_size > 0) {
            char png[640];
            if (generate_full_iq_waterfall(final_iq_path, final_iq_rate,
                                            png, sizeof png) == 0) {
                fprintf(stderr, "simple_sat_ops: waterfall -> %s\n", png);
                did_iq = 1;
            } else {
                fprintf(stderr,
                    "simple_sat_ops: gen_waterfall failed for %s "
                    "(gen_waterfall on PATH?)\n", final_iq_path);
            }
        }
    }
    if (!did_iq && final_wav_path[0]) {
        struct stat st;
        if (stat(final_wav_path, &st) == 0 && st.st_size > 44) {
            char png[640];
            if (generate_full_spectrogram(final_wav_path, png, sizeof png) == 0) {
                fprintf(stderr, "simple_sat_ops: pass spectrogram -> %s\n", png);
            } else {
                fprintf(stderr,
                    "simple_sat_ops: ffmpeg spectrogram failed for %s "
                    "(ffmpeg on PATH?)\n", final_wav_path);
            }
        }
    }
#endif

    if (state.prediction.auto_sat) {
        free_passes();
    }

    return 0;
}

// --- apply_args ---------------------------------------------------

int apply_args(state_t *state, int argc, char **argv, double jul_utc)
{
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;
    double min_altitude_km = 0.0;
    double max_altitude_km = 1000.0;
    double min_minutes_away = 1.0;
    double max_minutes_away = 90.0;
    double min_elevation = 0.0;
    double max_elevation = 90.0;
    int with_constellations = 0;
    state->antenna_rotator.tracking_prep_time_minutes = TRACKING_PREP_TIME_MINUTES;
    state->satellite_tracking = 0;

    state->nominal_uplink_frequency_hz = UPLINK_FREQ_MHZ * 1e6;
    state->nominal_downlink_frequency_hz = DOWNLINK_FREQ_MHZ * 1e6;
    state->doppler_uplink_frequency_hz = state->nominal_uplink_frequency_hz;
    state->doppler_downlink_frequency_hz = state->nominal_downlink_frequency_hz;
    state->doppler_correction_enabled = 1;

    state->run_with_antenna_rotator = 1;
    state->antenna_rotator.device_filename = "/dev/ttyUSB0";
    state->antenna_rotator.serial_speed = B600;
    state->antenna_rotator.fixed_target = 0;

    for (int i = 0; i < argc; i++) {
        if (strncmp("--verbose=", argv[i], 10) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 11) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->verbose_level = atoi(argv[i] + 10);
        } else if (strcmp("--with-rotator", argv[i]) == 0
                || strcmp("--with-hardware", argv[i]) == 0) {
            // Rotator is on by default now. These flags survive as
            // silent no-ops so existing scripts and muscle memory
            // keep working.
            state->n_options++;
            state->run_with_antenna_rotator = 1;
        } else if (strcmp("--without-rotator", argv[i]) == 0
                || strcmp("--without-hardware", argv[i]) == 0) {
            state->n_options++;
            state->run_with_antenna_rotator = 0;
        } else if (strcmp("--without-b210", argv[i]) == 0) {
            state->n_options++;
            g_without_b210 = 1;
        } else if (strcmp("--no-tx", argv[i]) == 0) {
            state->n_options++;
            g_no_tx = 1;
        } else if (strncmp("--tc-file=", argv[i], 10) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 11) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            snprintf(g_auto_tcmd_file_path, sizeof g_auto_tcmd_file_path,
                     "%s", argv[i] + 10);
        } else if (strncmp("--tle=", argv[i], 6) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->prediction.tles_filename = tle_path_resolve(argv[i] + 6);
        } else if (strncmp("--pass-folder=", argv[i], 14) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            // Pre-seed g_pass_folder; setup_pass_folder() then skips
            // its AOS-driven auto-discovery and uses the inherited
            // path (handoff case: new operator picks up the previous
            // operator's pass folder).
            snprintf(g_pass_folder, sizeof g_pass_folder, "%s", argv[i] + 14);
        } else if (strncmp("--rotator-device=", argv[i], 17) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 18) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->antenna_rotator.device_filename = argv[i] + 17;
        } else if (strncmp("--uplink-freq-mhz=", argv[i], 18) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->nominal_uplink_frequency_hz = atof(argv[i] + 18) * 1e6;
        } else if (strncmp("--downlink-freq-mhz=", argv[i], 20) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 21) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->nominal_downlink_frequency_hz = atof(argv[i] + 20) * 1e6;
        } else if (strcmp("--no-doppler-correction", argv[i]) == 0) {
            state->n_options++;
            state->doppler_correction_enabled = 0;
        } else if (strncmp("--rotator-target-elevation=", argv[i], 27) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 28) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->antenna_rotator.target_elevation = atof(argv[i] + 27);
            if (state->antenna_rotator.target_elevation < 0.0) {
                state->antenna_rotator.target_elevation = 0.0;
            } else if (state->antenna_rotator.target_elevation
                       > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                state->antenna_rotator.target_elevation =
                    ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
            }
            state->antenna_rotator.fixed_target = 1;
        } else if (strncmp("--rotator-target-azimuth=", argv[i], 25) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 26) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            double az = atof(argv[i] + 25);
            if (az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH) {
                az = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
            } else if (az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                az = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
            }
            state->antenna_rotator.target_azimuth = az;
            state->antenna_rotator.target_azimuth_unwrapped = az;
            state->antenna_rotator.unwrapped_target_valid = 1;
            state->antenna_rotator.fixed_target = 1;
        } else if (strncmp("--lat=", argv[i], 6) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_latitude = atof(argv[i] + 6);
        } else if (strncmp("--lon=", argv[i], 6) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_longitude = atof(argv[i] + 6);
        } else if (strncmp("--alt=", argv[i], 6) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_altitude = atof(argv[i] + 6);
        } else if (strcmp("--include-constellations", argv[i]) == 0) {
            state->n_options++;
            with_constellations = 1;
        } else if (strncmp("--min-altitude-km=", argv[i], 18) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_altitude_km = atof(argv[i] + 18);
        } else if (strncmp("--max-altitude-km=", argv[i], 18) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_altitude_km = atof(argv[i] + 18);
        } else if (strncmp("--min-elevation=", argv[i], 16) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_elevation = atof(argv[i] + 16);
        } else if (strncmp("--min-minutes=", argv[i], 14) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_minutes_away = atof(argv[i] + 14);
        } else if (strncmp("--max-minutes=", argv[i], 14) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_minutes_away = atof(argv[i] + 14);
        } else if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0], 0);
            return 2;
        } else if (strcmp("--help-full", argv[i]) == 0) {
            usage(stdout, argv[0], 1);
            return 2;
        } else if (strcmp("--control", argv[i]) == 0) {
            state->n_options++;
            g_control_mode = 1;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 3;
        }
    }
    int n_positional = argc - state->n_options - 1;  // -1 for argv[0]
    if (n_positional > 1) {
        usage(stderr, argv[0], 0);
        return 1;
    }

    // Find the (single) positional, if any. Existing convention is
    // "positional at argv[1]" but loop is robust to options-before /
    // options-after orderings.
    char *positional = NULL;
    for (int i = 1; i < argc; i++) {
        if (strncmp("--", argv[i], 2) == 0) continue;
        positional = argv[i];
        break;
    }

    // Bare invocation (no satellite_id, no --control): the standalone
    // tracker is being phased out in favour of the operator+viewer
    // split. Probe for the running operator and bail with a hint
    // either way.
    if (n_positional == 0 && !g_control_mode) {
        sso_ipc_client_t *probe = sso_ipc_client_connect("simple_sat_ops");
        if (probe == NULL) {
            fprintf(stderr,
                "operator not found: try `simple_sat_ops --control` "
                "to operate FrontierSat\n");
            return 1;
        }
        sso_ipc_client_close(probe);
        // Operator is up — main() will dispatch into run_viewer()
        // instead of the standalone-tracker path.
        g_viewer_mode = 1;
        return 0;
    }

    state->prediction.observer_ephem.position_geodetic.lat =
        site_latitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.lon =
        site_longitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.alt =
        site_altitude / 1000.0;

    // --control with no positional: auto-discover the newest TLE
    // under /FrontierSat/TLEs/ and load it directly. setup_pass_folder
    // pins this source file (under its original tle-YYYYMMDD.tle name)
    // into the pass folder once AOS is known.
    if (n_positional == 0 && g_control_mode) {
        if (state->prediction.tles_filename == NULL) {
            const char *tles_root = sso_tles_dir();
            static char src_tle[1024];
            time_t src_mtime = 0;
            if (find_newest_tle_recursive(tles_root, src_tle, sizeof src_tle,
                                          &src_mtime) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: --control wants a TLE under %s, "
                    "but no *.tle was found there. Drop one in "
                    "(or pass --tle=<path>).\n", tles_root);
                return EXIT_FAILURE;
            }
            fprintf(stderr, "simple_sat_ops: using TLE %s\n", src_tle);
            state->prediction.tles_filename = tle_path_resolve(src_tle);
        }
        static char sat_name[64];
        if (read_tle_name(state->prediction.tles_filename,
                          sat_name, sizeof sat_name) != 0) {
            fprintf(stderr,
                "simple_sat_ops: %s has no name line (2-line TLE?); "
                "pass the satellite name explicitly\n",
                state->prediction.tles_filename);
            return EXIT_FAILURE;
        }
        state->prediction.satellite_ephem.name = sat_name;
        fprintf(stderr, "simple_sat_ops: tracking '%s'\n", sat_name);
    } else {
        if (state->prediction.tles_filename == NULL) {
            static char default_tle[1024];
            if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
                fprintf(stderr,
                    "HOME unset or path too long; pass --tle=<path>\n");
                return EXIT_FAILURE;
            }
            state->prediction.tles_filename = tle_path_resolve(default_tle);
        }
        state->prediction.satellite_ephem.name = positional;
    }

    if (strcmp(state->prediction.satellite_ephem.name, "next") == 0) {
        state->prediction.auto_sat = 1;
        criteria_t criteria = {
            .min_altitude_km = min_altitude_km,
            .max_altitude_km = max_altitude_km,
            .min_minutes = min_minutes_away,
            .max_minutes = max_minutes_away,
            .min_elevation = min_elevation,
            .max_elevation = max_elevation,
            .regex = NULL,
            .regex_ignore_case = 0,
            .with_constellations = with_constellations,
        };
        prediction_t prediction_tmp = {0};
        prediction_tmp.tles_filename = state->prediction.tles_filename;
        prediction_tmp.observer_ephem.position_geodetic.lat =
            state->prediction.observer_ephem.position_geodetic.lat;
        prediction_tmp.observer_ephem.position_geodetic.lon =
            state->prediction.observer_ephem.position_geodetic.lon;
        prediction_tmp.observer_ephem.position_geodetic.alt =
            state->prediction.observer_ephem.position_geodetic.alt;
        find_passes(&prediction_tmp, jul_utc, 0.5, &criteria, NULL, NULL, 0, 0);
        const size_t n = number_of_passes();
        if (n == 0) {
            fprintf(stderr, "Unable to automatically find next in queue.\n");
            return 1;
        }

        const pass_t *p = get_pass(0);
        state->prediction.satellite_ephem.name = strdup(p->name);
        printf("Satellite: %s\n", state->prediction.satellite_ephem.name);
    }

    return 0;
}

// --- Tracking helpers ---------------------------------------------

void start_tracking(state_t *state)
{
    int antenna_rotator_result = 0;

    state->satellite_tracking = 1;
    state->doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control =
        state->antenna_rotator.antenna_should_be_controlled;
    // Clear the flip latch so the next tracking-enable re-decides for
    // the upcoming pass.
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;
    if (state->antenna_rotator.fixed_target) {
        antenna_rotator_result = antenna_rotator_set_unwrapped(
            &state->antenna_rotator,
            state->antenna_rotator.target_azimuth_unwrapped,
            state->antenna_rotator.target_elevation);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error setting antenna rotator position\n");
        } else {
            state->antenna_rotator.antenna_is_moving = 1;
        }
    }
}

void stop_tracking(state_t *state)
{
    int antenna_rotator_result = 0;
    double azimuth = 0.0;
    double elevation = 0.0;

    state->satellite_tracking = 0;
    state->doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control = 0;
    if (state->run_with_antenna_rotator) {
        antenna_rotator_result = antenna_rotator_command(
            &state->antenna_rotator, ANTENNA_ROTATOR_STOP, &azimuth, &elevation);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error stopping the antenna rotator\n");
        }
        if (antenna_rotator_seed_from_status(&state->antenna_rotator)
            != ANTENNA_ROTATOR_OK) {
            // leave unwrapped_target_valid as it was
        }
    }
    state->antenna_rotator.antenna_is_moving = 0;
    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;
}

int point_to_stationary_target(state_t *state, double azimuth, double elevation)
{
    state->satellite_tracking = 0;
    state->antenna_rotator.antenna_is_under_control = 0;
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;

    if (!state->antenna_rotator.unwrapped_target_valid) {
        if (antenna_rotator_seed_from_status(&state->antenna_rotator)
            != ANTENNA_ROTATOR_OK) {
            return ANTENNA_ROTATOR_BAD_RESPONSE;
        }
    }

    double prev = state->antenna_rotator.target_azimuth_unwrapped;
    double final_az = antenna_rotator_home_unwrapped_target(prev, azimuth);
    double delta = final_az - prev;

    if (fabs(delta) > 180.0) {
        // Two-step home: halfway waypoint first to disambiguate the
        // direction of rotation; the main loop drives the second leg
        // once the antenna has stopped at the intermediate.
        double mid = prev + delta / 2.0;
        if (mid < ANTENNA_ROTATOR_MINIMUM_AZIMUTH)
            mid = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
        if (mid > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH)
            mid = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
        state->antenna_rotator.home_pending_final_az = final_az;
        state->antenna_rotator.homing_in_progress = 1;
        int rc = antenna_rotator_set_unwrapped(&state->antenna_rotator,
                                                mid, elevation);
        if (rc == ANTENNA_ROTATOR_OK) {
            state->antenna_rotator.antenna_is_moving = 1;
        }
        return rc;
    }

    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    int rc = antenna_rotator_set_unwrapped(&state->antenna_rotator,
                                            final_az, elevation);
    if (rc == ANTENNA_ROTATOR_OK) {
        state->antenna_rotator.antenna_is_moving = 1;
    }
    return rc;
}

void update_doppler_shifted_frequencies(state_t *state,
                                          double uplink_freq,
                                          double downlink_freq)
{
    double doppler_factor = 1.0
        - state->prediction.satellite_ephem.range_rate_km_s / 299792.458;
    state->doppler_uplink_frequency_hz   = uplink_freq   * doppler_factor;
    state->doppler_downlink_frequency_hz = downlink_freq * doppler_factor;
}
