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
#define RIBBON_LEN 60
static double g_ribbon_peak[RIBBON_LEN];
static int    g_ribbon_count   = 0;   // number of valid samples (caps at RIBBON_LEN)
static int    g_ribbon_head    = 0;   // next write index (circular)
static double g_ribbon_last_t  = 0.0; // monotonic timestamp of last push

static void ribbon_push(double peak_dbfs)
{
    g_ribbon_peak[g_ribbon_head] = peak_dbfs;
    g_ribbon_head = (g_ribbon_head + 1) % RIBBON_LEN;
    if (g_ribbon_count < RIBBON_LEN) g_ribbon_count++;
}

// Map a dBFS sample to one of 8 UTF-8 block glyphs (U+2581..U+2588).
// -90 dBFS or lower renders as a space so silent gaps are visually
// distinct from "low" signal. 0 dBFS rails to the tallest block.
static const char *ribbon_glyph(double dbfs)
{
    if (!isfinite(dbfs) || dbfs <= -90.0) return " ";
    double level = (dbfs - (-90.0)) / 11.25;  // 8 bins of ~11.25 dB each
    int idx = (int)level;
    if (idx < 0) idx = 0;
    if (idx > 7) idx = 7;
    static const char *blocks[8] = {
        "\xE2\x96\x81", "\xE2\x96\x82", "\xE2\x96\x83", "\xE2\x96\x84",
        "\xE2\x96\x85", "\xE2\x96\x86", "\xE2\x96\x87", "\xE2\x96\x88",
    };
    return blocks[idx];
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
    char            wav_in[512];
    int             sample_rate;
    long            start_sample;
    long            n_samples;
    char            png_out[640];
    char            status_msg[1024];
} spectrum_job_t;

static spectrum_job_t g_spec_job;

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
            "showspectrumpic=s=1920x1080:mode=combined:color=intensity:legend=1",
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

static void *spectrum_worker(void *arg)
{
    spectrum_job_t *j = (spectrum_job_t *) arg;
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
static char g_cmd_status[160] = "";

static void cmd_enter(void)
{
    g_cmd_active = 1;
    g_cmd_buf[0] = '\0';
    g_cmd_len = 0;
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
static void run_tx_compose(void);
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
        run_tx_compose();
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

                            if (pthread_create(&g_spec_job.thr, NULL,
                                               spectrum_worker, &g_spec_job) != 0) {
                                cmd_set_status("spectrum: pthread_create failed: %s",
                                               strerror(errno));
                            } else {
                                g_spec_job.active = 1;
                                cmd_set_status("spectrum: rendering %.1fs -> %s",
                                               (double) want / (double) sample_rate,
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

// Returns 1 if key was consumed by the command line, 0 to fall through.
static int cmd_handle_key(int key, state_t *state)
{
    if (!g_cmd_active) return 0;
    if (key == ERR) return 1;
    if (key == 27 /* Esc */) {
        g_cmd_active = 0;
        g_cmd_status[0] = '\0';  // clear, don't leave a stale message
        return 1;
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        g_cmd_active = 0;
        cmd_dispatch(state);
        return 1;
    }
    if (key == KEY_BACKSPACE || key == 127 || key == 8) {
        if (g_cmd_len > 0) {
            g_cmd_buf[--g_cmd_len] = '\0';
        }
        return 1;
    }
    if (key >= 32 && key < 127 && g_cmd_len < (int)sizeof g_cmd_buf - 1) {
        g_cmd_buf[g_cmd_len++] = (char)key;
        g_cmd_buf[g_cmd_len] = '\0';
    }
    return 1;
}

// Render the command prompt (or last-result status) on the bottom row.
static void cmd_render(void)
{
    int row = LINES - 1;
    if (g_cmd_active) {
        mvprintw(row, 0, ":%s", g_cmd_buf);
        // Visible cursor block at the typing position.
        addch(' ' | A_REVERSE);
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
} tx_compose_t;

// Survives Esc / commit so the operator can reopen and pick up the
// previous typed string. First open seeds it with "CTS1+" — the OBC's
// CTS1 telecommand prefix.
static char g_tx_last_payload[160] = "CTS1+";
static char g_tx_last_power[12]    = "30.0";

static void tx_compose_init(tx_compose_t *c) {
    memset(c, 0, sizeof *c);
    snprintf(c->payload, sizeof c->payload, "%s", g_tx_last_payload);
    snprintf(c->power,   sizeof c->power,   "%s", g_tx_last_power);
    snprintf(c->status_msg, sizeof c->status_msg,
             "edit; viewers see drafts ~200 ms after you stop typing");
}

static void tx_compose_remember(const tx_compose_t *c) {
    snprintf(g_tx_last_payload, sizeof g_tx_last_payload, "%s", c->payload);
    snprintf(g_tx_last_power,   sizeof g_tx_last_power,   "%s", c->power);
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

static void tx_field_append(tx_compose_t *c, int ch) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    size_t n = strlen(buf);
    if (n + 1 >= cap) return;
    int accept = 0;
    if (tx_field_is_text(c->focus)) {
        accept = (ch >= 32 && ch < 127);
    } else if (tx_field_is_decimal(c->focus)) {
        accept = (ch >= '0' && ch <= '9') || ch == '.' || ch == '-';
    }
    if (!accept) return;
    buf[n++] = (char) ch;
    buf[n]   = '\0';
    c->preview_dirty = 1;
}

static void tx_field_backspace(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    size_t n = strlen(buf);
    if (n == 0) return;
    buf[n - 1] = '\0';
    c->preview_dirty = 1;
}

static void tx_field_toggle(tx_compose_t *c) {
    if (c->focus == TXF_ALLOW_TX) {
        c->allow_tx = !c->allow_tx;
        c->preview_dirty = 1;
    }
}

// Single-line redraw helper: prints a label + value, applies A_REVERSE
// on the value when this field is focused.
static void tx_draw_field(WINDOW *w, int row, int col, int focused,
                          const char *label, const char *value) {
    mvwprintw(w, row, col, "%s", label);
    if (focused) wattron(w, A_REVERSE);
    wprintw(w, "%s", value);
    if (focused) wattroff(w, A_REVERSE);
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
    snprintf(buf, sizeof buf, "%-*.*s", payload_w, payload_w,
             c->payload[0] ? c->payload : " ");
    tx_draw_field(w, 3, 2, c->focus == TXF_PAYLOAD,
                  "Payload  ", buf);

    snprintf(buf, sizeof buf, "%-8s", c->power);
    tx_draw_field(w, 5, 2, c->focus == TXF_POWER,
                  "TX power ", buf);
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
              "Tab/Shift-Tab move   Space toggles allow-tx   Enter commits   Esc cancels");
    wclrtoeol(w);
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

static void run_tx_compose(void) {
    if (!g_ipc) return;  // no compose without operator IPC
    tx_compose_t c;
    tx_compose_init(&c);

    // Wide modal so a long telecommand fits without scrolling.
    int h = 14, ww = 120;
    if (h > LINES) h = LINES;
    if (ww > COLS) ww = COLS;
    if (ww < 60)  ww = (COLS < 60) ? COLS : 60;
    WINDOW *w = newwin(h, ww, (LINES - h) / 2, (COLS - ww) / 2);
    if (!w) return;
    keypad(w, TRUE);
    nodelay(w, TRUE);

    tx_compose_draw(w, &c);
    long debounce_ns = 200000000L;
    long last_edit_ns = ts_now_ns();
    int active = 1;
    while (active) {
        int ch = wgetch(w);
        if (ch != ERR) {
            int changed = 1;
            if (ch == 27) {  // Esc — remember the typed string but don't send.
                tx_compose_remember(&c);
                active = 0; break;
            } else if (ch == '\n' || ch == '\r' || ch == KEY_ENTER) {
                char err[120];
                if (tx_compose_validate(&c, err, sizeof err) != 0) {
                    snprintf(c.status_msg, sizeof c.status_msg,
                             "rejected: %.*s",
                             (int)(sizeof c.status_msg - 16), err);
                } else if (tx_compose_commit(&c, err, sizeof err) != 0) {
                    snprintf(c.status_msg, sizeof c.status_msg,
                             "commit failed: %.*s",
                             (int)(sizeof c.status_msg - 20), err);
                } else {
                    tx_compose_remember(&c);
                    active = 0; break;
                }
            } else if (ch == '\t') {
                c.focus = (tx_field_t) ((c.focus + 1) % TXF_COUNT);
            } else if (ch == KEY_BTAB) {
                c.focus = (tx_field_t) ((c.focus + TXF_COUNT - 1) % TXF_COUNT);
            } else if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {
                tx_field_backspace(&c);
            } else if (ch == ' ' && tx_field_is_toggle(c.focus)) {
                tx_field_toggle(&c);
            } else if (ch >= 32 && ch < 127) {
                tx_field_append(&c, ch);
            } else {
                changed = 0;
            }
            if (changed) {
                last_edit_ns = ts_now_ns();
                tx_compose_draw(w, &c);
            }
        }
        // Pump operator IPC so viewers stay connected while the
        // operator is sitting in the modal. Non-blocking.
        if (g_ipc) sso_ipc_server_step(g_ipc, 0);
        // Debounced preview broadcast.
        if (c.preview_dirty
            && (ts_now_ns() - last_edit_ns) >= debounce_ns) {
            tx_compose_broadcast_preview(&c);
            c.preview_dirty = 0;
            tx_compose_draw(w, &c);  // re-render so the mirror is visible
        }
        usleep(20000);  // 20 ms
    }

    delwin(w);
    touchwin(stdscr);
    refresh();
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
        double azimuth = 0.0, elevation = 0.0;
        if (antenna_rotator_command(&state->antenna_rotator,
                                    ANTENNA_ROTATOR_STATUS,
                                    &azimuth, &elevation) == ANTENNA_ROTATOR_OK) {
            state->antenna_rotator.azimuth   = azimuth;
            state->antenna_rotator.elevation = elevation;
        } else {
            azimuth   = state->antenna_rotator.azimuth;
            elevation = state->antenna_rotator.elevation;
        }
        p.current_az = azimuth;
        p.current_el = elevation;
        p.target_az  = state->antenna_rotator.target_azimuth;
        p.target_el  = state->antenna_rotator.target_elevation;
        p.flip       = state->antenna_rotator.flip_mode_pass;
    }

    render_status_panel(&p, print_row, print_col);
}

// Render the B210 RX panel: live freq, IQ level meter, decoded-frame
// count, last decoded frame summary. No-op when there's no B210 in
// this build / run.
static void render_rx_panel(int *print_row, int print_col)
{
    if (print_row == NULL) return;
#ifdef WITH_USRP_B210
    int row = *print_row;
    int col = print_col;
    int rec_active = rx_session_wav_active(g_rx_session);
    mvprintw(row++, col, "%15s   %s%s", "B210",
             g_rx_session ? "active" : "(offline)",
             rec_active ? "  [REC]" : "");
    clrtoeol();
    if (g_rx_session) {
        uint64_t frames = 0;
        double   peak_dbfs = -90.0, rms_dbfs = -90.0, freq_hz = 0.0;
        char     last[80] = "";
        rx_session_snapshot(g_rx_session, &frames, &peak_dbfs, &rms_dbfs,
                            &freq_hz, last, sizeof last);
        mvprintw(row++, col, "%15s   %.6f MHz", "RX freq", freq_hz / 1e6);
        clrtoeol();
        mvprintw(row++, col, "%15s   peak %+5.1f  rms %+5.1f dBFS",
                 "level", peak_dbfs, rms_dbfs);
        clrtoeol();
        mvprintw(row++, col, "%15s   %llu",
                 "frames", (unsigned long long) frames);
        clrtoeol();
        if (last[0]) {
            mvprintw(row++, col, "%15s   %s", "last frame", last);
            clrtoeol();
        }

        // Per-type counters + time since the most recent decoded
        // frame. Goes after the live numbers so it doesn't bury them.
        rx_packet_type_stats_t per_type[RX_PT_COUNT];
        double age_s = -1.0;
        rx_session_stats_snapshot(g_rx_session, per_type, &age_s);
        if (age_s >= 0.0) {
            mvprintw(row++, col, "%15s   %.1f s ago",
                     "last frame T+", age_s);
            clrtoeol();
        }
        // One row of "name=N" pairs across all six slots.
        char by_type[160] = {0};
        size_t bt_len = 0;
        for (int s = 0; s < RX_PT_COUNT; ++s) {
            int n = snprintf(by_type + bt_len, sizeof by_type - bt_len,
                             "%s%s=%llu",
                             (bt_len ? "  " : ""),
                             rx_packet_type_label((rx_packet_type_slot_t)s),
                             (unsigned long long) per_type[s].count);
            if (n < 0 || (size_t)n >= sizeof by_type - bt_len) break;
            bt_len += (size_t) n;
        }
        mvprintw(row++, col, "%15s   %s", "by type", by_type);
        clrtoeol();
        // For each slot that's seen at least one frame, hex-preview
        // the latest payload (up to 24 bytes -- enough to read the
        // CSP header + first few payload bytes at a glance).
        for (int s = 0; s < RX_PT_COUNT; ++s) {
            if (per_type[s].count == 0 || per_type[s].last_payload_len <= 0) {
                continue;
            }
            char hexbuf[3 * 24 + 16];
            size_t hex_len = 0;
            int n_show = per_type[s].last_payload_len;
            if (n_show > 24) n_show = 24;
            for (int b = 0; b < n_show; ++b) {
                int w = snprintf(hexbuf + hex_len, sizeof hexbuf - hex_len,
                                 "%s%02X",
                                 (b ? " " : ""),
                                 per_type[s].last_payload[b]);
                if (w < 0 || (size_t)w >= sizeof hexbuf - hex_len) break;
                hex_len += (size_t) w;
            }
            const char *label = rx_packet_type_label((rx_packet_type_slot_t)s);
            mvprintw(row++, col, "%15s   %s%s", label, hexbuf,
                     per_type[s].last_payload_len > n_show ? " ..." : "");
            clrtoeol();
        }

        // Signal ribbon: oldest sample on the left, newest on the
        // right. Each glyph = 1 s of peak dBFS. Empty until the
        // sampler has accumulated history.
        if (g_ribbon_count > 0) {
            mvprintw(row, col, "%15s   ", "ribbon");
            int start = (g_ribbon_head - g_ribbon_count + RIBBON_LEN)
                        % RIBBON_LEN;
            for (int i = 0; i < g_ribbon_count; ++i) {
                int idx = (start + i) % RIBBON_LEN;
                addstr(ribbon_glyph(g_ribbon_peak[idx]));
            }
            clrtoeol();
            ++row;
            mvprintw(row++, col, "%15s   -90 dBFS .. 0 dBFS  (60 s)",
                     "scale");
            clrtoeol();
        }
    }
    *print_row = row;
#else
    (void) print_row; (void) print_col;
#endif
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

        int tx_log_row = LINES - TX_LOG_SIZE - 2;
        if (tx_log_row >= 17) {
            render_tx_log_panel(tx_log_row, 1);
        }
    }

    attron(A_REVERSE);
    char foot[200];
    time_t now = time(NULL);
    long stale_s = g_viewer_last_event > 0
        ? (long)(now - g_viewer_last_event)
        : -1;
    const char *status = !connected ? "DISCONNECTED"
                                    : (stale_s < 0 ? "WAITING"
                                                   : (stale_s > 5 ? "STALE"
                                                                  : "LIVE"));
    if (g_viewer_confirm_until > 0 && now < g_viewer_confirm_until) {
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
    double t_last_ipc_broadcast = 0.0;
    double t_last_redraw        = 0.0;
    const double IPC_BROADCAST_PERIOD_S = 0.5;   // 2 Hz
    const double REDRAW_PERIOD_S        = 0.1;   // 10 Hz

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
            render_rx_panel(&row, 50);

            clrtoeol();

            // TX log lives below the keyboard info / antenna status if
            // the terminal is tall enough to host it without colliding.
            int tx_log_row = LINES - TX_LOG_SIZE - 2;
            if (tx_log_row >= keyboard_info_row + 4) {
                render_tx_log_panel(tx_log_row, 1);
            }
        }

        key = getch();
        if (g_cmd_active) {
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
                    run_tx_compose();
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

            cmd_render();

            refresh();
            t_last_redraw = t_now;
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
            usleep(g_cmd_active ? 20000 : UPDATE_INTERVAL_MICROSEC);
        }
    }

    endwin();
    if (g_ipc) {
        sso_ipc_server_close(g_ipc);
        g_ipc = NULL;
    }
#ifdef WITH_USRP_B210
    char final_wav_path[512] = "";
    if (g_rx_session) {
        // Snapshot the WAV path before close so the full-pass spectrogram
        // can find the closed file on disk. snap_wav_path persists across
        // wav_stop, so this works even if recording was already stopped.
        rx_session_wav_snapshot(g_rx_session,
                                final_wav_path, sizeof final_wav_path,
                                NULL, NULL, NULL);
        // The worker owns the WAV writer and the B210 core. Closing
        // the session signals the worker to stop, joins it, then
        // tears down both. Any open WAV gets its header patched.
        rx_session_request_wav_stop(g_rx_session);
        rx_session_close(g_rx_session);
        g_rx_session = NULL;
    }

    // Any in-flight `:spectrum N` worker is touching the same WAV — let
    // it finish before we hand the file to the full-pass render.
    if (g_spec_job.active) {
        pthread_join(g_spec_job.thr, NULL);
        g_spec_job.active = 0;
        if (g_spec_job.status_msg[0]) {
            fprintf(stderr, "simple_sat_ops: %s\n", g_spec_job.status_msg);
        }
    }

    // Full-pass spectrogram. Best-effort: prints status to stderr (the
    // ncurses screen is already torn down). Skipped silently if no WAV
    // was ever opened or ffmpeg isn't on PATH (exec returns 127, which
    // generate_full_spectrogram reports as a -1 return).
    if (final_wav_path[0]) {
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
