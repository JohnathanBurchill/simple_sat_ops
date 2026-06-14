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
#include "antenna_rotator_async.h"
#include "rotator_calibrate.h"
#include "pursuit.h"
#include "tr_switch.h"
#include "state.h"
#include "prediction.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_operator.h"
#include "sso_ipc_paths.h"
#include "sso_paths.h"
#include "tle_csv.h"
#include "frontiersat.h"
#include "hmac_keyfile.h"
#include "agenda_line.h"
#include "tcmd_lint.h"
#include "sso_pseudo.h"
#include "sso_time.h"
#include "sso_version.h"
#include "panels.h"
#include "pass_session.h"
#include "scan_sky.h"
#include "tracking.h"
#include "spectrogram.h"
#include "tui.h"
#include "auto_tcmd.h"
#include "tx_compose.h"
#include "tx_log.h"
#include "argparse.h"

#ifdef SSO_WITH_SDR
#include "b210_rx_tx_core.h"
#include "carrier_trim.h"
#include "rx_session.h"
#include "tx_burst.h"
#endif

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h>
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
#include <fcntl.h>
#include <termios.h>

// Carrier defaults. FrontierSat is simplex on 436.150 MHz — uplink
// and downlink share the same frequency — so both default to the
// same value. Overridable with --uplink-freq-mhz= / --downlink-freq-mhz=
// for any future split-band bird.
#define UPLINK_FREQ_MHZ   436.150000
#define DOWNLINK_FREQ_MHZ 436.150000

// Doppler retune threshold (Hz). Frequencies update on the display
// when the residual drifts past this. 200 Hz keeps the residual offset
// well inside the 9600 GFSK clean-eye band (~±3 kHz) even at the peak
// ~100 Hz/s slew near TCA.
#define DOPPLER_SHIFT_RESOLUTION_KHZ 0.2

// Antenna rotator max angle from target before a new SET is issued.

// How close a STATUS azimuth must be to the just-commanded home waypoint to
// be treated as the controller's post-SET target echo -- which it reports
// for a couple of seconds before its feedback shows real motion -- rather
// than a real position reading. Used to gate the two-step home's final leg.
#define HOME_ECHO_TOLERANCE_DEG 2.0

// WARN_DAYS_SINCE_EPOCH lives in ui/panels.h's renderer; MAX_MINUTES_TO_PREDICT
// in control/pass_session.h (shared with the week-ahead pass search).

#define UPDATE_INTERVAL_MICROSEC 500000

#define SSO_IPC_MAX_CLIENTS_FOR_ROSTER 16

// --- Operator-mode IPC bookkeeping ---------------------------------
// Set by apply_args when --control is passed. When set, main() opens
// the sso_ipc server on /run/sso/simple_sat_ops.sock and fans out a
// state event on every UI tick. Other operator-aware tools
// (b210_rx_tx --control, tx_frame_sdr) verify the operator's Unix
// user matches their own via this socket.
// The IPC fan-out server and the operator's Unix user now live on
// state_t (state.ipc / state.operator_user).

// --self-test: after CLI parse + HMAC keyfile load, print the resolved
// configuration to stdout and exit 0 — BEFORE opening the IPC socket,
// the rotator, the B210, or loading the TLE. Useful for confirming
// "did my command line do what I think?" without keying any hardware
// or claiming any shared resource. Skips the no-arg viewer-probe in
// apply_args too (which is itself a side effect).

// Refuse to fully start when the --tc-file agenda has telecommand lint
// errors. --ignore-at-your-peril-all-tc-errors clears this and lets a
// known-bad agenda through anyway.

// TX dry-run: record the command as not-sent (reason "dry-run")
// instead of pushing the burst through rx_session. Lets the operator
// exercise the auto-tcmd
// state machine + the TX compose modal on a dev host with no B210 (or
// with --without-b210 to skip the device). The allow-tx safety
// checkbox still has to be ticked to enter RUNNING — dry-run is about
// hardware presence, not about the operator's intent to transmit.

// Live raylib waterfall viewer. Off by default; --live-waterfall on
// the command line opts in. When recording starts, fork+exec the
// live_waterfall binary with the active .iq path. Track the child
// PID so we can SIGTERM it on shutdown. The iq-path scratch is only
// referenced inside the WITH_USRP_B210 launch block, so tag it
// unused — the cleanup path at the bottom of main() does use the
// pid, so that one stays unannotated.

// --always-record: start the WAV / IQ / sidecar recording as soon
// as rx_session opens and keep it open until shutdown, ignoring the
// usual per-pass elevation gate. Intended for bench characterisation
// runs (noise floor vs. antenna orientation, no-antenna baseline,
// gain stability over hours) where the operator wants continuous
// capture without a satellite pass to drive AOS / LOS.

// --testing: bench / characterisation runs that aren't tied to a
// pass. Pass folder lands under <root>/Testing/yyyymmdd/hhmmLT/ using
// the CURRENT local time, not a predicted AOS — keeps test captures
// out of the operational Operations/ tree and skips the "no AOS in
// next N minutes" abort in setup_pass_folder.

static pid_t g_live_waterfall_pid = -1;
__attribute__((unused))
static char  g_live_waterfall_iq[512] = "";
// Write end of the pipe whose read end is dup2'd to the viewer's
// stdin. Colon commands like :wf_zoom_khz write line-based commands
// here so the running viewer can adjust without a relaunch. -1 when
// no viewer is alive.
static int   g_live_waterfall_stdin_fd = -1;



// Command line: vi-style ":" prompt at the bottom of the screen for
// runtime actions. State now lives in state_t.cmd (cmdline_t); the
// preview-debounce interval is the one remaining tunable here.
static long   g_cmd_debounce_ns  = 150000000L;  // 150 ms

static long cmd_now_ns(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (long) ts.tv_sec * 1000000000L + (long) ts.tv_nsec;
}

static void cmd_enter(state_t *state)
{
    state->cmd.active = 1;
    state->cmd.buf[0] = '\0';
    state->cmd.len = 0;
    state->cmd.cursor = 0;
    // Start a fresh history walk at the (empty) editing line.
    state->cmd.hist_pos = state->cmd.history_count;
    // Force an immediate preview broadcast so viewers see the ":" prompt
    // appear the moment the operator opens it.
    state->cmd.dirty = 1;
    state->cmd.last_edit_ns = 0;
}

void cmd_set_status(state_t *state, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(state->cmd.status, sizeof state->cmd.status, fmt, ap);
    va_end(ap);
}

// --- Command history -----------------------------------------------
// Append a just-executed line. Skips blanks and immediate duplicates so
// holding Enter or repeating a command doesn't bloat the ring.
static void cmd_history_push(state_t *state, const char *line)
{
    if (line == NULL || line[0] == '\0') return;
    if (state->cmd.history_count > 0
        && strcmp(state->cmd.history[state->cmd.history_count - 1], line) == 0) {
        state->cmd.hist_pos = state->cmd.history_count;
        return;
    }
    if (state->cmd.history_count < CMD_HISTORY_SIZE) {
        snprintf(state->cmd.history[state->cmd.history_count], CMD_BUF_SIZE, "%s", line);
        state->cmd.history_count++;
    } else {
        memmove(&state->cmd.history[0], &state->cmd.history[1],
                sizeof state->cmd.history[0] * (CMD_HISTORY_SIZE - 1));
        snprintf(state->cmd.history[CMD_HISTORY_SIZE - 1], CMD_BUF_SIZE, "%s", line);
    }
    state->cmd.hist_pos = state->cmd.history_count;
}

// Replace the live buffer and park the cursor at the end. Marks the
// buffer dirty so the next tick re-broadcasts the preview to viewers.
static void cmd_buf_set(state_t *state, const char *s)
{
    snprintf(state->cmd.buf, sizeof state->cmd.buf, "%s", s);
    state->cmd.len = (int) strlen(state->cmd.buf);
    state->cmd.cursor = state->cmd.len;
    state->cmd.dirty = 1;
    state->cmd.last_edit_ns = cmd_now_ns();
}

static void cmd_history_prev(state_t *state)   // Up
{
    if (state->cmd.history_count == 0) return;
    if (state->cmd.hist_pos == state->cmd.history_count) {
        snprintf(state->cmd.hist_saved, sizeof state->cmd.hist_saved, "%s", state->cmd.buf);
    }
    if (state->cmd.hist_pos > 0) state->cmd.hist_pos--;
    cmd_buf_set(state, state->cmd.history[state->cmd.hist_pos]);
}

static void cmd_history_next(state_t *state)   // Down
{
    if (state->cmd.history_count == 0) return;
    if (state->cmd.hist_pos >= state->cmd.history_count) return;
    state->cmd.hist_pos++;
    cmd_buf_set(state, state->cmd.hist_pos == state->cmd.history_count
                    ? state->cmd.hist_saved
                    : state->cmd.history[state->cmd.hist_pos]);
}

// --- Path expansion + tab completion -------------------------------

// Expand a leading ~ and any $VAR / ${VAR} references in `in` into
// `out`. ~ maps to $HOME at the very start only; unknown variables
// expand to empty. Returns 0 on success, -1 on overflow (out emptied).
// Lets a typed `$TLES/.../tle.tle` reach the filesystem at dispatch and
// during completion.
static int cmd_expand(const char *in, char *out, size_t cap)
{
    size_t o = 0;
#define CMD_EXPAND_PUT(c) \
    do { if (o + 1 >= cap) { out[0] = '\0'; return -1; } out[o++] = (c); } while (0)
    for (size_t i = 0; in[i] != '\0'; ) {
        if (i == 0 && in[0] == '~' && (in[1] == '/' || in[1] == '\0')) {
            const char *home = getenv("HOME");
            if (home) for (const char *h = home; *h; ++h) CMD_EXPAND_PUT(*h);
            i++;
            continue;
        }
        if (in[i] == '$') {
            int    braced = (in[i + 1] == '{');
            size_t s = i + 1 + (braced ? 1 : 0);
            size_t e = s;
            while (in[e] && (isalnum((unsigned char) in[e]) || in[e] == '_')) e++;
            if (e > s) {
                char   name[64];
                size_t nl = e - s;
                if (nl >= sizeof name) nl = sizeof name - 1;
                memcpy(name, in + s, nl);
                name[nl] = '\0';
                const char *val = getenv(name);
                if (val) for (const char *v = val; *v; ++v) CMD_EXPAND_PUT(*v);
                i = e;
                if (braced && in[i] == '}') i++;
                continue;
            }
            // Lone $ / ${ with no name -> emit literally.
        }
        CMD_EXPAND_PUT(in[i]);
        i++;
    }
    out[o] = '\0';
#undef CMD_EXPAND_PUT
    return 0;
}

// Insert a string at the cursor, shifting the tail right. Stops at the
// buffer cap. Marks the line dirty for the preview broadcast.
static void cmd_insert_str(state_t *state, const char *s)
{
    for (const char *p = s; *p; ++p) {
        if (state->cmd.len >= (int) sizeof state->cmd.buf - 1) break;
        memmove(&state->cmd.buf[state->cmd.cursor + 1], &state->cmd.buf[state->cmd.cursor],
                (size_t)(state->cmd.len - state->cmd.cursor + 1));  // include nul
        state->cmd.buf[state->cmd.cursor] = *p;
        state->cmd.len++;
        state->cmd.cursor++;
    }
    state->cmd.dirty = 1;
    state->cmd.last_edit_ns = cmd_now_ns();
}

// Command names, for first-token completion.
static const char *const g_cmd_names[] = {
    "help", "tx", "auto", "track", "stop", "home", "retarget",
    "freq", "rs", "spectrum", "lo_offset", "lo_bandwidth", "gain", "quit",
};

// Tab completion at the cursor. The first token completes against
// command names; any later token completes a filesystem path. Only the
// trailing path component is completed -- a $VAR / ~ prefix stays
// literal in the buffer and is expanded just to pick the directory to
// scan, so `:retarget $TLES/20260529/<TAB>` keeps `$TLES` and fills in
// the file. Single match -> full completion (with a trailing `/` for a
// directory); multiple -> extend to the longest common prefix.
static void cmd_tab_complete(state_t *state)
{
    if (!state->cmd.active) return;

    int start = state->cmd.cursor;
    while (start > 0 && state->cmd.buf[start - 1] != ' '
                     && state->cmd.buf[start - 1] != '\t') {
        start--;
    }
    int tok_len = state->cmd.cursor - start;
    char token[CMD_BUF_SIZE];
    if (tok_len < 0 || tok_len >= (int) sizeof token) return;
    memcpy(token, state->cmd.buf + start, (size_t) tok_len);
    token[tok_len] = '\0';

    // First token -> command-name completion.
    if (start == 0) {
        int    n = 0;
        size_t common = 0;
        char   lcp[32] = "";
        const char *only = NULL;
        for (size_t i = 0; i < sizeof g_cmd_names / sizeof g_cmd_names[0]; ++i) {
            if (strncmp(g_cmd_names[i], token, (size_t) tok_len) != 0) continue;
            n++;
            if (n == 1) {
                only = g_cmd_names[i];
                snprintf(lcp, sizeof lcp, "%s", g_cmd_names[i]);
                common = strlen(lcp);
            } else {
                size_t k = 0;
                while (k < common && g_cmd_names[i][k] == lcp[k]) k++;
                common = k;
                lcp[common] = '\0';
            }
        }
        if (n == 1) {
            cmd_insert_str(state, only + tok_len);
            cmd_insert_str(state, " ");
        } else if (n > 1 && common > (size_t) tok_len) {
            // lcp is already truncated to the common prefix.
            cmd_insert_str(state, lcp + tok_len);
        }
        return;
    }

    // Argument token -> path completion. Split into the literal dir
    // prefix (through the last '/') and the partial basename.
    const char *slash = strrchr(token, '/');
    char        dir_lit[CMD_BUF_SIZE];
    const char *base;
    if (slash) {
        size_t dl = (size_t)(slash - token) + 1;  // keep the '/'
        if (dl >= sizeof dir_lit) return;
        memcpy(dir_lit, token, dl);
        dir_lit[dl] = '\0';
        base = slash + 1;
    } else {
        dir_lit[0] = '\0';
        base = token;
    }
    size_t base_len = strlen(base);

    char scan_dir[1024];
    if (dir_lit[0] == '\0') {
        snprintf(scan_dir, sizeof scan_dir, ".");
    } else if (cmd_expand(dir_lit, scan_dir, sizeof scan_dir) != 0
               || scan_dir[0] == '\0') {
        return;
    }

    DIR *d = opendir(scan_dir);
    if (d == NULL) return;
    int    n = 0;
    size_t common = 0;
    // Sized to hold any directory entry name (gcc-15 treats d_name as up
    // to 1023 bytes for its truncation analysis).
    char   lcp[1024]  = "";
    char   only[1024] = "";
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        const char *nm = de->d_name;
        // Hide dotfiles (incl. . and ..) unless the user typed a dot.
        if (nm[0] == '.' && base[0] != '.') continue;
        if (strncmp(nm, base, base_len) != 0) continue;
        n++;
        if (n == 1) {
            snprintf(only, sizeof only, "%s", nm);
            snprintf(lcp,  sizeof lcp,  "%s", nm);
            common = strlen(lcp);
        } else {
            size_t k = 0;
            while (k < common && nm[k] == lcp[k]) k++;
            common = k;
            lcp[common] = '\0';
        }
    }
    closedir(d);
    if (n == 0) return;

    if (n == 1) {
        cmd_insert_str(state, only + base_len);
        // Append '/' when the single match is a directory.
        char full[2048];
        size_t sl = strlen(scan_dir);
        if (sl > 0 && scan_dir[sl - 1] == '/') {
            snprintf(full, sizeof full, "%s%s", scan_dir, only);
        } else {
            snprintf(full, sizeof full, "%s/%s", scan_dir, only);
        }
        struct stat st;
        if (stat(full, &st) == 0 && S_ISDIR(st.st_mode)) {
            cmd_insert_str(state, "/");
        }
    } else if (common > base_len) {
        // lcp is already truncated to the common prefix of all matches.
        cmd_insert_str(state, lcp + base_len);
    }
}


// Dispatch the typed command. state may be touched by tracking-related
// commands; nothing else needs it. Returns nothing -- result lands in
// state->cmd.status for the next redraw.
static void cmd_dispatch(state_t *state)
{
    char buf[CMD_BUF_SIZE];
    snprintf(buf, sizeof buf, "%s", state->cmd.buf);
    // Trim leading whitespace; an empty command is a no-op.
    char *p = buf;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '\0') { cmd_set_status(state, ""); return; }

    char *save = NULL;
    char *cmd  = strtok_r(p, " \t", &save);
    char *arg1 = strtok_r(NULL, " \t", &save);

    if (cmd == NULL) {
        cmd_set_status(state, "");
        return;
    }
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        cmd_set_status(state, "commands: help tx track stop home quit "
                       "retarget <tle-file> "
                       "freq <MHz> lo_offset <±kHz> lo_bandwidth <kHz> "
                       "gain <dB> rs on|off spectrum <sec>");
    } else if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "q") == 0
               || strcmp(cmd, "exit") == 0) {
        state->running = 0;
        cmd_set_status(state, "quitting");
    } else if (strcmp(cmd, "tx") == 0) {
        // Defer the modal until after we leave command-mode so the
        // bottom prompt doesn't bleed under the modal box.
        cmd_set_status(state, "opening TX compose...");
        state->cmd.active = 0;
        tx_compose_open(state);
    } else if (strcmp(cmd, "auto") == 0) {
        if (state->auto_tcmd_file_path[0] == '\0') {
            cmd_set_status(state, "auto: no --tc-file=<path> given on the cmdline");
        } else {
            cmd_set_status(state, "opening auto-tcmd...");
            state->cmd.active = 0;
            auto_tcmd_open(state);
        }
    } else if (strcmp(cmd, "track") == 0) {
        start_tracking(state);
        cmd_set_status(state, "tracking on");
        sso_audit_event("track-on",
            state->prediction.satellite_ephem.tle.sat_name[0]
                ? state->prediction.satellite_ephem.tle.sat_name : "");
    } else if (strcmp(cmd, "stop") == 0) {
        stop_tracking(state);
        cmd_set_status(state, "tracking stopped");
        sso_audit_event("track-off", "");
    } else if (strcmp(cmd, "home") == 0) {
        stop_tracking(state);
        point_to_stationary_target(state, 0.0, 0.0);
        cmd_set_status(state, "home: az=0 el=0");
        sso_audit_event("rotator-home", "az=0 el=0");
    } else if (strcmp(cmd, "retarget") == 0) {
        char expanded[1024];
        if (arg1 == NULL) {
            cmd_set_status(state, "retarget: usage `retarget <tle-file>` "
                           "(first satellite in the file is used)");
        } else if (cmd_expand(arg1, expanded, sizeof expanded) != 0) {
            cmd_set_status(state, "retarget: path too long after expansion");
        } else {
            int rc = retarget_to_tle(state, expanded);
            const char *name =
                state->prediction.satellite_ephem.tle.sat_name;
            double mins =
                state->prediction.predicted_minutes_until_visible;
            switch (rc) {
            case RETARGET_OK:
                if (mins > 0.0) {
                    cmd_set_status(state, "retarget -> %s (AOS in %.1f min)",
                                   name, mins);
                } else {
                    cmd_set_status(state, "retarget -> %s (in pass, %.0fs elapsed)",
                                   name, -mins * 60.0);
                }
                sso_audit_event("retarget", name);
                break;
            case RETARGET_SAME:
                cmd_set_status(state, "retarget: already on %s (same file)", arg1);
                break;
            case RETARGET_READ_ERR:
                cmd_set_status(state, "retarget: cannot read a TLE from '%s'", arg1);
                break;
            case RETARGET_BAD_TLE:
                cmd_set_status(state, "retarget: '%s' has invalid TLE elements",
                               arg1);
                break;
            default:
                cmd_set_status(state, "retarget: bad argument");
                break;
            }
        }
    } else if (strcmp(cmd, "freq") == 0) {
        if (arg1 == NULL) {
            cmd_set_status(state, "freq: missing argument (MHz)");
        } else {
#ifdef SSO_WITH_SDR
            double v = atof(arg1);
            double hz = (v < 1e6) ? v * 1e6 : v;   // accept MHz or Hz
            if (hz < 1e6 || hz > 6e9) {
                cmd_set_status(state, "freq: %g out of [1 MHz, 6 GHz]", hz);
            } else if (state->rx_session == NULL) {
                cmd_set_status(state, "freq: no RX session");
            } else {
                rx_session_request_freq(state->rx_session, hz);
                cmd_set_status(state, "freq -> %.6f MHz", hz / 1e6);
            }
#else
            cmd_set_status(state, "freq: this build has no USRP support");
#endif
        }
    } else if (strcmp(cmd, "rs") == 0) {
        // Reed-Solomon toggle isn't a runtime knob yet -- rx_session
        // sets reed_solomon at open() time. Flag this clearly instead
        // of silently no-op'ing.
        if (arg1 == NULL) {
            cmd_set_status(state, "rs: usage: rs on|off (not yet runtime-toggleable)");
        } else {
            cmd_set_status(state, "rs %s: NOT YET WIRED -- rx_session_open params only",
                           arg1);
        }
    } else if (strcmp(cmd, "spectrum") == 0 || strcmp(cmd, "spec") == 0) {
#ifdef SSO_WITH_SDR
        if (arg1 == NULL) {
            cmd_set_status(state, "spectrum: usage `spectrum <seconds>` (1..600)");
        } else if (state->rx_session == NULL) {
            cmd_set_status(state, "spectrum: no RX session");
        } else {
            double duration_s = atof(arg1);
            if (duration_s <= 0.0) {
                cmd_set_status(state, "spectrum: invalid duration '%s'", arg1);
            } else {
                if (duration_s > 600.0) duration_s = 600.0;
                if (duration_s < 1.0)   duration_s = 1.0;

                spectrum_job_reap(state);
                if (state->spec_job.active) {
                    cmd_set_status(state, "spectrum: a render is already in progress");
                } else {
                    char wav_path[512];
                    long n_samples = 0;
                    int  sample_rate = 0;
                    int  wav_active = 0;
                    rx_session_wav_snapshot(state->rx_session,
                                            wav_path, sizeof wav_path,
                                            &n_samples, &sample_rate, &wav_active);
                    if (wav_path[0] == '\0' || sample_rate <= 0) {
                        cmd_set_status(state, "spectrum: no WAV (recording not started yet)");
                    } else {
                        long want  = (long)(duration_s * (double) sample_rate);
                        long start = n_samples - want;
                        if (start < 0) { start = 0; want = n_samples; }
                        if (want <= 0) {
                            cmd_set_status(state, "spectrum: no samples captured yet");
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

                            memset(&state->spec_job, 0, sizeof state->spec_job);
                            snprintf(state->spec_job.wav_in, sizeof state->spec_job.wav_in,
                                     "%s", wav_path);
                            snprintf(state->spec_job.png_out, sizeof state->spec_job.png_out,
                                     "%.480s_LOCAL_%s_to_%s.png",
                                     base, ts_start, ts_end);
                            state->spec_job.sample_rate  = sample_rate;
                            state->spec_job.start_sample = start;
                            state->spec_job.n_samples    = want;

                            // Pair the WAV slice with an IQ slice if the
                            // sidecar exists — worker prefers IQ and only
                            // falls back to the WAV+ffmpeg path when iq_in
                            // is empty.
                            char iq_path[512] = "";
                            long iq_pairs = 0;
                            int  iq_rate  = 0;
                            rx_session_iq_snapshot(state->rx_session,
                                                   iq_path, sizeof iq_path,
                                                   &iq_pairs, &iq_rate);
                            if (iq_path[0] && iq_pairs > 0 && iq_rate > 0) {
                                long want_p  = (long)(duration_s * (double) iq_rate);
                                long start_p = iq_pairs - want_p;
                                if (start_p < 0) { start_p = 0; want_p = iq_pairs; }
                                if (want_p > 0) {
                                    snprintf(state->spec_job.iq_in,
                                             sizeof state->spec_job.iq_in, "%s", iq_path);
                                    state->spec_job.iq_sample_rate = iq_rate;
                                    state->spec_job.iq_start_pair  = start_p;
                                    state->spec_job.iq_pairs       = want_p;
                                }
                            }

                            if (pthread_create(&state->spec_job.thr, NULL,
                                               spectrum_worker, &state->spec_job) != 0) {
                                cmd_set_status(state, "spectrum: pthread_create failed: %s",
                                               strerror(errno));
                            } else {
                                state->spec_job.active = 1;
                                cmd_set_status(state, "spectrum: rendering %.1fs (%s) -> %s",
                                               (double) want / (double) sample_rate,
                                               state->spec_job.iq_in[0] ? "iq" : "wav",
                                               state->spec_job.png_out);
                            }
                        }
                    }
                }
            }
        }
#else
        cmd_set_status(state, "spectrum: this build has no USRP support");
#endif
    } else if (strcmp(cmd, "lo_offset") == 0) {
        // Move the hardware LO mid-pass to dodge a baseband artifact.
        // SIGNED kHz: positive → LO above nominal (signal at NEGATIVE
        // baseband); negative → LO below (signal at POSITIVE baseband).
        // Brief PLL-settle glitch — decode skips a few frames. Comfort
        // range ~±5..±40 kHz; clipped here to ±45 kHz so the worst
        // case still keeps a 3 kHz margin to the post-decim band edge.
        if (arg1 == NULL) {
            cmd_set_status(state, "lo_offset: usage `lo_offset <signed_kHz>` "
                           "(comfort range ±5..±40)");
        } else {
#ifdef SSO_WITH_SDR
            double khz = atof(arg1);
            if (khz < -45.0 || khz > 45.0) {
                cmd_set_status(state, "lo_offset: %g kHz out of [-45, +45]", khz);
            } else if (state->rx_session == NULL) {
                cmd_set_status(state, "lo_offset: no RX session");
            } else {
                double new_offset_hz = khz * 1000.0;
                state->rx_lo_offset_hz = new_offset_hz;
                rx_session_set_lo_offset(state->rx_session,
                                         state->nominal_downlink_frequency_hz,
                                         new_offset_hz);
                cmd_set_status(state, "lo_offset -> %+.1f kHz (PLL glitching, "
                               "decode resumes shortly)", khz);
            }
#else
            cmd_set_status(state, "lo_offset: this build has no USRP support");
#endif
        }
    } else if (strcmp(cmd, "lo_bandwidth") == 0) {
        // Adjust the live raylib waterfall's visible bandwidth at
        // runtime. The viewer reads line-based commands from its
        // stdin (we wired a pipe at fork time); send
        // "bandwidth N\n". Name mirrors :lo_offset so both LO-
        // relative knobs live in the same command family.
        if (arg1 == NULL) {
            cmd_set_status(state, "lo_bandwidth: usage `lo_bandwidth <N>` (kHz)");
        } else if (g_live_waterfall_stdin_fd < 0) {
            cmd_set_status(state, "lo_bandwidth: no live viewer running "
                           "(launch with --live-waterfall)");
        } else {
            double n = atof(arg1);
            if (n <= 0.0 || n > 1000.0) {
                cmd_set_status(state, "lo_bandwidth: %g out of (0, 1000] kHz", n);
            } else {
                char line[64];
                int  ln = snprintf(line, sizeof line, "bandwidth %g\n", n);
                ssize_t w = (ln > 0) ? write(g_live_waterfall_stdin_fd,
                                             line, (size_t) ln) : -1;
                if (w == ln) {
                    cmd_set_status(state, "lo_bandwidth: -> %g kHz", n);
                } else {
                    cmd_set_status(state, "lo_bandwidth: write failed: %s",
                                   strerror(errno));
                }
            }
        }
    } else if (strcmp(cmd, "gain") == 0) {
        // Change the AD9361 RX gain mid-pass. The current operating
        // point lives in state->rx_gain_db; routed through the
        // worker thread so we don't touch the UHD streamer from this
        // thread (same handoff as :lo_offset).
        if (arg1 == NULL) {
            cmd_set_status(state, "gain: usage `gain <dB>` (range 0-76; current %.1f)",
                           state->rx_gain_db);
        } else {
#ifdef SSO_WITH_SDR
            double g = atof(arg1);
            if (g < 0.0 || g > 76.0) {
                cmd_set_status(state, "gain: %g dB out of [0, 76]", g);
            } else if (state->rx_session == NULL) {
                cmd_set_status(state, "gain: no RX session");
            } else {
                state->rx_gain_db = g;
                rx_session_set_gain(state->rx_session, g);
                cmd_set_status(state, "gain -> %.1f dB", g);
            }
#else
            cmd_set_status(state, "gain: this build has no USRP support");
#endif
        }
    } else {
        cmd_set_status(state, "unknown command '%s' (try :help)", cmd);
    }
}

// Mirror the operator's ":" prompt to viewers. cmd-preview carries the
// live buffer (debounced in the main loop); cmd-executed carries the
// dispatched command + the resulting status string. Both helpers no-op
// when state->ipc isn't open (e.g., --no-control).
static void cmd_broadcast_preview(state_t *state)
{
    if (!state->ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_CMD_PREVIEW);
    snprintf(evt.from, sizeof evt.from, "%s",
             state->operator_user ? state->operator_user : "?");
    snprintf(evt.cmd_text, sizeof evt.cmd_text, "%s", state->cmd.buf);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(state->ipc, buf);
    }
}

static void cmd_broadcast_executed(state_t *state, const char *executed_cmd)
{
    if (!state->ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_CMD_EXECUTED);
    snprintf(evt.from, sizeof evt.from, "%s",
             state->operator_user ? state->operator_user : "?");
    snprintf(evt.cmd_text,   sizeof evt.cmd_text,   "%s",
             executed_cmd ? executed_cmd : "");
    snprintf(evt.cmd_status, sizeof evt.cmd_status, "%s", state->cmd.status);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(state->ipc, buf);
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
    CMD_ACTION_HIST_PREV,
    CMD_ACTION_HIST_NEXT,
} cmd_action_t;

static int cmd_apply_action(state_t *state, cmd_action_t a)
{
    switch (a) {
        case CMD_ACTION_LEFT:
            if (state->cmd.cursor > 0) state->cmd.cursor--;
            return 1;
        case CMD_ACTION_RIGHT:
            if (state->cmd.cursor < state->cmd.len) state->cmd.cursor++;
            return 1;
        case CMD_ACTION_HOME:
            state->cmd.cursor = 0;
            return 1;
        case CMD_ACTION_END:
            state->cmd.cursor = state->cmd.len;
            return 1;
        case CMD_ACTION_BACKSPACE:
            if (state->cmd.cursor > 0) {
                memmove(&state->cmd.buf[state->cmd.cursor - 1],
                        &state->cmd.buf[state->cmd.cursor],
                        (size_t)(state->cmd.len - state->cmd.cursor + 1));
                state->cmd.len--;
                state->cmd.cursor--;
                state->cmd.dirty = 1;
                state->cmd.last_edit_ns = cmd_now_ns();
            }
            return 1;
        case CMD_ACTION_DEL:
            if (state->cmd.cursor < state->cmd.len) {
                memmove(&state->cmd.buf[state->cmd.cursor],
                        &state->cmd.buf[state->cmd.cursor + 1],
                        (size_t)(state->cmd.len - state->cmd.cursor));
                state->cmd.len--;
                state->cmd.dirty = 1;
                state->cmd.last_edit_ns = cmd_now_ns();
            }
            return 1;
        case CMD_ACTION_HIST_PREV:
            cmd_history_prev(state);
            return 1;
        case CMD_ACTION_HIST_NEXT:
            cmd_history_next(state);
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
    if (b2 == 'A') return CMD_ACTION_HIST_PREV;
    if (b2 == 'B') return CMD_ACTION_HIST_NEXT;
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
    if (!state->cmd.active) return 0;
    if (key == ERR) return 1;
    if (key == 27 /* Esc OR start of a CSI sequence */) {
        cmd_action_t a = cmd_drain_csi();
        if (a != CMD_ACTION_NONE) {
            cmd_apply_action(state, a);
            return 1;
        }
        // Truly bare Esc — cancel.
        state->cmd.active = 0;
        state->cmd.status[0] = '\0';
        state->cmd.buf[0] = '\0';
        state->cmd.len = 0;
        state->cmd.cursor = 0;
        cmd_broadcast_executed(state, "");
        state->cmd.dirty = 0;
        return 1;
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        char executed[CMD_BUF_SIZE];
        snprintf(executed, sizeof executed, "%s", state->cmd.buf);
        state->cmd.active = 0;
        cmd_history_push(state, executed);
        cmd_dispatch(state);
        // Audit: one line per `:` command the operator pressed Enter on,
        // with the post-dispatch status so a reviewer sees both the
        // request and the immediate result (e.g. "freq 437.5" /
        // "freq -> 437.500000 MHz").
        if (executed[0]) {
            char det[480];
            snprintf(det, sizeof det,
                     "input=\"%.100s\" result=\"%.150s\"",
                     executed, state->cmd.status);
            sso_audit_event("cmd", det);
        }
        // After dispatch, state->cmd.status holds the result string. Mirror
        // both to viewers so they see exactly what the operator sees.
        cmd_broadcast_executed(state, executed);
        state->cmd.dirty = 0;
        return 1;
    }
    if (key == '\t')      { cmd_tab_complete(state); return 1; }
    if (key == KEY_UP)    { cmd_history_prev(state); return 1; }
    if (key == KEY_DOWN)  { cmd_history_next(state); return 1; }
    if (key == KEY_LEFT)  return cmd_apply_action(state, CMD_ACTION_LEFT);
    if (key == KEY_RIGHT) return cmd_apply_action(state, CMD_ACTION_RIGHT);
    if (key == KEY_HOME || key == 1 /* Ctrl-A */)
        return cmd_apply_action(state, CMD_ACTION_HOME);
    if (key == KEY_END  || key == 5 /* Ctrl-E */)
        return cmd_apply_action(state, CMD_ACTION_END);
    if (key == KEY_BACKSPACE || key == 127 || key == 8)
        return cmd_apply_action(state, CMD_ACTION_BACKSPACE);
    if (key == KEY_DC || key == 4 /* Ctrl-D */)
        return cmd_apply_action(state, CMD_ACTION_DEL);
    if (key >= 32 && key < 127 && state->cmd.len < (int) sizeof state->cmd.buf - 1) {
        // Insert at cursor: shift the tail right by one, drop char in.
        memmove(&state->cmd.buf[state->cmd.cursor + 1],
                &state->cmd.buf[state->cmd.cursor],
                (size_t)(state->cmd.len - state->cmd.cursor + 1));  // include nul
        state->cmd.buf[state->cmd.cursor] = (char) key;
        state->cmd.len++;
        state->cmd.cursor++;
        state->cmd.dirty = 1;
        state->cmd.last_edit_ns = cmd_now_ns();
    }
    return 1;
}

// Render the command prompt (or last-result status) on the bottom row.
// Cursor is drawn as a reverse-video block on the char at state->cmd.cursor
// — or on a trailing space when the cursor is at end-of-line. The
// surrounding text is plain so cursor position is unambiguous.
static void cmd_render(state_t *state)
{
    int row = LINES - 1;
    if (state->cmd.active) {
        move(row, 0);
        addch(':');
        for (int i = 0; i < state->cmd.len; ++i) {
            if (i == state->cmd.cursor) {
                addch(((unsigned char) state->cmd.buf[i]) | A_REVERSE);
            } else {
                addch((unsigned char) state->cmd.buf[i]);
            }
        }
        if (state->cmd.cursor == state->cmd.len) {
            addch(' ' | A_REVERSE);
        }
        clrtoeol();
        // Park the hardware cursor on the same cell the A_REVERSE
        // block highlights. The layered refresh below will curs_set(1)
        // when an editable context is active so the operator sees a
        // visible blinking cursor on top of the inverse block.
        move(row, 1 + state->cmd.cursor);
        return;
    } else if (state->cmd.status[0]) {
        mvprintw(row, 0, "%s", state->cmd.status);
    } else {
        move(row, 0);
    }
    clrtoeol();
}


// Snapshot the operator's RX panel into the wire-side fields of an
// event. Called for both STATE broadcasts and WELCOME replies so a
// newly-connecting viewer sees the same panel state everyone else does.
static void ipc_fill_rx_panel(state_t *state, sso_event_t *evt)
{
    rx_panel_data_t d;
    rx_panel_collect_local(state, &d);
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
    evt->rx_frames_pcm   = (long) d.frames_pcm;
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
        snprintf(evt->rx_pt_summary[s], sizeof evt->rx_pt_summary[s],
                 "%.*s", (int)(sizeof evt->rx_pt_summary[s] - 1),
                 d.pt_summary[s]);
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
    if (!s->ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_STATE);
    snprintf(evt.from, sizeof(evt.from), "%s",
             s->operator_user ? s->operator_user : "?");
    snprintf(evt.operator_user, sizeof(evt.operator_user), "%s",
             s->operator_user ? s->operator_user : "?");
    evt.has_state = 1;
    if (s->prediction.satellite_ephem.name) {
        snprintf(evt.satellite, sizeof(evt.satellite), "%s",
                 s->prediction.satellite_ephem.name);
    }
    evt.az = az;
    evt.el = el;
    evt.freq_hz = (long) downlink_freq;
    evt.doppler_hz = doppler_delta_dl;
    if (s->pass_folder[0]) {
        snprintf(evt.pass_folder, sizeof(evt.pass_folder), "%s",
                 s->pass_folder);
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

    // Auto-TCMD progress so viewers can follow the run without the modal.
    {
        int at_sent = 0, at_total = 0;
        const char *at_label = NULL;
        if (auto_tcmd_progress(s, &at_sent, &at_total, &at_label)) {
            evt.auto_tcmd_on    = 1;
            evt.auto_tcmd_sent  = at_sent;
            evt.auto_tcmd_total = at_total;
            snprintf(evt.auto_tcmd_state, sizeof evt.auto_tcmd_state,
                     "%s", at_label);
        }
    }
    sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
    size_t n = 0;
    if (n < sizeof(entries) / sizeof(entries[0])) {
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 s->operator_user ? s->operator_user : "?");
        snprintf(entries[n].role, sizeof(entries[n].role), "operator");
        entries[n].since[0] = '\0';
        n++;
    }
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char user[64], role[16], since[40];
    while (n < sizeof(entries) / sizeof(entries[0])
           && sso_ipc_server_next_client(s->ipc, &it, &cid,
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
    ipc_fill_rx_panel(s, &evt);
    char buf[4096];
    if (sso_event_encode(&evt, buf, sizeof(buf)) == 0) {
        sso_ipc_server_broadcast(s->ipc, buf);
    }

    // Cache for WELCOME replies so a viewer doesn't have to wait for
    // the next periodic broadcast to see anything.
    snprintf(s->last_state.sat, sizeof s->last_state.sat, "%s", evt.satellite);
    snprintf(s->last_state.tle, sizeof s->last_state.tle, "%s", evt.tle_path);
    s->last_state.az      = evt.az;
    s->last_state.el      = evt.el;
    s->last_state.freq_hz = evt.freq_hz;
    s->last_state.doppler = evt.doppler_hz;
    s->last_state.tgt_az  = evt.target_az;
    s->last_state.tgt_el  = evt.target_el;
    s->last_state.flip    = evt.flip;
    s->last_state.in_pass = evt.in_pass;
    s->last_state.tracking= evt.tracking;
    s->last_state.has_rot = evt.has_rotator;
    s->last_state.jul     = evt.jul_utc;
    snprintf(s->last_state.idesg, sizeof s->last_state.idesg, "%s", evt.idesg);
    s->last_state.epoch_min    = evt.epoch_min;
    s->last_state.min_visible  = evt.min_visible;
    s->last_state.min_above_0  = evt.min_above_0;
    s->last_state.min_above_30 = evt.min_above_30;
    s->last_state.max_el       = evt.max_el;
    s->last_state.pred_az      = evt.pred_az;
    s->last_state.pred_el      = evt.pred_el;
    s->last_state.alt_km       = evt.alt_km;
    s->last_state.lat_deg      = evt.lat_deg;
    s->last_state.lon_deg      = evt.lon_deg;
    s->last_state.speed_kms    = evt.speed_kms;
    s->last_state.range_km     = evt.range_km;
    s->last_state.rrate_kms    = evt.range_rate_kms;
    s->last_state.valid   = 1;
}

static void ipc_on_event(sso_ipc_server_t *srv, sso_client_id_t id,
                         const sso_event_t *evt, void *user) {
    state_t *state = (state_t *) user;
    if (evt->type != SSO_EVT_HELLO) return;
    sso_event_t welcome;
    sso_event_init(&welcome, SSO_EVT_WELCOME);
    snprintf(welcome.from, sizeof(welcome.from), "%s",
             state->operator_user ? state->operator_user : "?");
    snprintf(welcome.operator_user, sizeof(welcome.operator_user), "%s",
             state->operator_user ? state->operator_user : "?");
    if (state->pass_folder[0]) {
        snprintf(welcome.pass_folder, sizeof(welcome.pass_folder), "%s",
                 state->pass_folder);
    }
    if (state->last_state.valid) {
        welcome.has_state   = 1;
        snprintf(welcome.satellite, sizeof welcome.satellite,
                 "%s", state->last_state.sat);
        snprintf(welcome.tle_path, sizeof welcome.tle_path,
                 "%s", state->last_state.tle);
        welcome.az          = state->last_state.az;
        welcome.el          = state->last_state.el;
        welcome.freq_hz     = state->last_state.freq_hz;
        welcome.doppler_hz  = state->last_state.doppler;
        welcome.target_az   = state->last_state.tgt_az;
        welcome.target_el   = state->last_state.tgt_el;
        welcome.flip        = state->last_state.flip;
        welcome.in_pass     = state->last_state.in_pass;
        welcome.tracking    = state->last_state.tracking;
        welcome.has_rotator = state->last_state.has_rot;
        welcome.jul_utc     = state->last_state.jul;
        snprintf(welcome.idesg, sizeof welcome.idesg, "%s", state->last_state.idesg);
        welcome.epoch_min      = state->last_state.epoch_min;
        welcome.min_visible    = state->last_state.min_visible;
        welcome.min_above_0    = state->last_state.min_above_0;
        welcome.min_above_30   = state->last_state.min_above_30;
        welcome.max_el         = state->last_state.max_el;
        welcome.pred_az        = state->last_state.pred_az;
        welcome.pred_el        = state->last_state.pred_el;
        welcome.alt_km         = state->last_state.alt_km;
        welcome.lat_deg        = state->last_state.lat_deg;
        welcome.lon_deg        = state->last_state.lon_deg;
        welcome.speed_kms      = state->last_state.speed_kms;
        welcome.range_km       = state->last_state.range_km;
        welcome.range_rate_kms = state->last_state.rrate_kms;
        // Auto-TCMD progress reads the live modal state (like
        // ipc_fill_rx_panel below) — no state->last_state.* cache needed.
        {
            int at_sent = 0, at_total = 0;
            const char *at_label = NULL;
            if (auto_tcmd_progress(state, &at_sent, &at_total, &at_label)) {
                welcome.auto_tcmd_on    = 1;
                welcome.auto_tcmd_sent  = at_sent;
                welcome.auto_tcmd_total = at_total;
                snprintf(welcome.auto_tcmd_state,
                         sizeof welcome.auto_tcmd_state, "%s", at_label);
            }
        }
        // Roster — operator first, then the existing clients we know
        // of. The newly-connecting client is already in the slot table
        // (slot_dispatch_line ran first) but its role isn't populated
        // until HELLO is processed; that's why we iterate via
        // sso_ipc_server_next_client, which surfaces what HELLO set.
        sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
        size_t n = 0;
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 state->operator_user ? state->operator_user : "?");
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
        ipc_fill_rx_panel(state, &welcome);
    }
    char buf[4096];
    if (sso_event_encode(&welcome, buf, sizeof(buf)) == 0) {
        sso_ipc_server_send(srv, id, buf);
    }
}


// --- Forward decls -------------------------------------------------

static int apply_args(state_t *state, int argc, char **argv, double jul_utc, int help);






// --- Viewer mode --------------------------------------------------
//
// Read-only mirror of the operator instance. The viewer does NOT run
// SGP4 and does NOT load a TLE — it just deposits every broadcast
// field into a state_t and calls the same render helpers the operator
// uses, so the two displays are byte-identical except for the help text.

// All viewer-mode state in one struct, owned as a local in run_viewer and
// threaded by pointer through the viewer helpers (viewer_on_event receives
// it via the IPC callback's user channel). Kept separate from state_t: the
// viewer mirrors an operator's broadcast into `state` and never runs the
// tracker itself.
typedef struct viewer {
    int    event_pending;
    int    has_state;
    char   operator[64];
    char   roster_json[1024];
    time_t last_event;
    int    running;
    // Mirror of the operator's ":" prompt state. cmd_active = 1 between
    // the first cmd-preview after ':' and the cmd-executed that closes it.
    // cmd_buf and cmd_status track the operator's verbatim so the viewer's
    // bottom row matches exactly. Sized to the wire field (cmd_text).
    int    cmd_active;
    char   cmd_buf[160];
    char   cmd_status[160];
    // Mirror of the operator's auto-tcmd run. auto_on = 1 while the
    // operator has a run to show; the render line is "<sent>/<total>
    // sent (<state>)" and disappears when the operator closes the modal.
    int    auto_on;
    int    auto_sent;
    int    auto_total;
    char   auto_state[12];
    // Mirror of the operator's RX panel. Filled from STATE / WELCOME events;
    // render_rx_panel reads it directly during viewer_render.
    rx_panel_data_t rx_panel;
    // state_t whose fields the viewer mirrors from the broadcast each tick.
    state_t state;
    double carrier_hz;
    double jul_utc;
    int    has_rotator;
    char   tle_path[256];
    char   pass_folder[256];
    // Take-control confirmation. Press 'c' once to arm, 'y' within
    // VIEWER_CONFIRM_WINDOW_S seconds to commit. Anything else cancels.
    time_t confirm_until;
} viewer_t;

#define VIEWER_CONFIRM_WINDOW_S 5

static void viewer_on_event(sso_ipc_client_t *cli, const sso_event_t *evt,
                            void *user)
{
    (void) cli;
    viewer_t *v = (viewer_t *) user;
    if (evt->type == SSO_EVT_TX_COMMAND_PREVIEW
     || evt->type == SSO_EVT_TX_COMMAND_SENT
     || evt->type == SSO_EVT_TX_NOT_SENT) {
        tx_log_push(&v->state, evt);
        v->last_event = time(NULL);
        v->event_pending = 1;
        return;
    }
    if (evt->type == SSO_EVT_CMD_PREVIEW) {
        v->cmd_active = 1;
        snprintf(v->cmd_buf, sizeof v->cmd_buf,
                 "%s", evt->cmd_text);
        v->last_event = time(NULL);
        v->event_pending = 1;
        return;
    }
    if (evt->type == SSO_EVT_CMD_EXECUTED) {
        // Empty cmd_text + empty cmd_status = Esc/cancel; clear the row.
        // Otherwise show the executed-command result string just like the
        // operator does after cmd_dispatch returns.
        v->cmd_active = 0;
        v->cmd_buf[0] = '\0';
        snprintf(v->cmd_status, sizeof v->cmd_status,
                 "%s", evt->cmd_status);
        v->last_event = time(NULL);
        v->event_pending = 1;
        return;
    }
    if (evt->type != SSO_EVT_STATE && evt->type != SSO_EVT_WELCOME) {
        return;
    }
    v->last_event = time(NULL);
    if (evt->operator_user[0]) {
        snprintf(v->operator, sizeof v->operator, "%s",
                 evt->operator_user);
    }
    if (evt->roster_json[0]) {
        snprintf(v->roster_json, sizeof v->roster_json, "%s",
                 evt->roster_json);
    }
    if (!evt->has_state) return;

    state_t *s = &v->state;
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

    v->has_rotator = evt->has_rotator;
    v->jul_utc     = evt->jul_utc;
    v->carrier_hz  = (evt->doppler_hz != 0.0)
        ? (double)evt->freq_hz + evt->doppler_hz
        : (double)evt->freq_hz;
    if (evt->tle_path[0]) {
        snprintf(v->tle_path, sizeof v->tle_path,
                 "%s", evt->tle_path);
    }
    if (evt->pass_folder[0]) {
        snprintf(v->pass_folder, sizeof v->pass_folder,
                 "%s", evt->pass_folder);
    }

    // Auto-tcmd progress — stashed unconditionally so a broadcast
    // without the fields (run over, modal closed) clears the line.
    v->auto_on    = evt->auto_tcmd_on;
    v->auto_sent  = evt->auto_tcmd_sent;
    v->auto_total = evt->auto_tcmd_total;
    snprintf(v->auto_state, sizeof v->auto_state,
             "%s", evt->auto_tcmd_state);

    // Mirror the operator's RX panel from the broadcast. Wipe to zero
    // first so a slot that the operator hasn't decoded in this run
    // doesn't carry stale state from a previous event.
    memset(&v->rx_panel, 0, sizeof v->rx_panel);
    v->rx_panel.have_session  = evt->rx_have_session;
    snprintf(v->rx_panel.warning, sizeof v->rx_panel.warning,
             "%s", evt->rx_warning);
    if (evt->rx_have_session) {
        v->rx_panel.rec_active     = evt->rx_rec_active;
        v->rx_panel.rx_freq_hz     = evt->rx_freq_hz;
        v->rx_panel.peak_dbfs      = evt->rx_peak_dbfs;
        v->rx_panel.rms_dbfs       = evt->rx_rms_dbfs;
        v->rx_panel.frames_total   = (uint64_t) evt->rx_frames_total;
        v->rx_panel.frames_pcm     = (uint64_t) evt->rx_frames_pcm;
        v->rx_panel.frames_vit     = (uint64_t) evt->rx_frames_vit;
        snprintf(v->rx_panel.last_frame_summary,
                 sizeof v->rx_panel.last_frame_summary,
                 "%s", evt->rx_last_frame_summary);
        v->rx_panel.age_s = evt->rx_age_s;
        int slots = RX_PANEL_PT_COUNT < SSO_RX_PT_SLOTS
                  ? RX_PANEL_PT_COUNT : SSO_RX_PT_SLOTS;
        for (int s = 0; s < slots; ++s) {
            v->rx_panel.pt_count[s] = (uint64_t) evt->rx_pt_count[s];
            int pl = evt->rx_pt_payload_len[s];
            if (pl < 0) pl = 0;
            int copy = pl;
            if (copy > RX_PANEL_PAYLOAD_MAX) copy = RX_PANEL_PAYLOAD_MAX;
            v->rx_panel.pt_payload_len[s] = pl;
            memcpy(v->rx_panel.pt_payload[s],
                   evt->rx_pt_payload[s], (size_t) copy);
            snprintf(v->rx_panel.pt_summary[s],
                     sizeof v->rx_panel.pt_summary[s],
                     "%.*s",
                     (int)(sizeof v->rx_panel.pt_summary[s] - 1),
                     evt->rx_pt_summary[s]);
        }
        int rn = evt->rx_ribbon_n;
        if (rn > RIBBON_LEN) rn = RIBBON_LEN;
        v->rx_panel.ribbon_n = rn;
        memcpy(v->rx_panel.ribbon, evt->rx_ribbon, (size_t) rn);
        v->rx_panel.ribbon[rn] = '\0';
        memcpy(v->rx_panel.ribbon_peak, evt->rx_ribbon_peak,
               (size_t) rn * sizeof v->rx_panel.ribbon_peak[0]);
    }

    v->has_state   = 1;
    v->event_pending = 1;
}

// Format the roster array into "alice,bob,carol" for the header bar,
// skipping the operator (already shown separately) and any entry whose
// user is empty. The roster JSON is built by sso_event_set_roster with
// the schema [{"user":"...","role":"...","since":"..."},...], so we
// can scan for "user":"..." and "role":"..." pairs.
static void viewer_roster_users(viewer_t *v, char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    const char *p = v->roster_json;
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
static void viewer_take_control(viewer_t *v, sso_ipc_client_t *cli, const char *argv0)
{
    if (!v->tle_path[0]) {
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
    tui_release_stderr();

    // Re-exec self as --control with inherited TLE and pass folder.
    // Filename args use the space form so the spawned child sees the
    // same TAB-completable spelling the user types.
    char self_path[1024];
    ssize_t n = readlink("/proc/self/exe", self_path, sizeof self_path - 1);
    const char *exe = (n > 0) ? (self_path[n] = '\0', self_path) : argv0;
    char *new_argv[8];
    int ai = 0;
    new_argv[ai++] = (char *) exe;
    new_argv[ai++] = "--control";
    new_argv[ai++] = "--tle";
    new_argv[ai++] = v->tle_path;
    if (v->pass_folder[0]) {
        new_argv[ai++] = "--pass-folder";
        new_argv[ai++] = v->pass_folder;
    }
    new_argv[ai] = NULL;
    fprintf(stderr,
        "simple_sat_ops: taking control with --tle %s%s%s\n",
        v->tle_path,
        v->pass_folder[0] ? "  --pass-folder " : "",
        v->pass_folder[0] ? v->pass_folder : "");
    execv(exe, new_argv);
    // If we got here exec failed — best to bail loudly.
    fprintf(stderr,
        "simple_sat_ops viewer: execv %s failed: %s\n",
        exe, strerror(errno));
    exit(EXIT_FAILURE);
}

static void viewer_render(viewer_t *v, int connected)
{
    int cols = COLS;
    erase();

    if (!v->has_state) {
        mvprintw(2, 2, "(waiting for state from the operator...)");
    } else {
        int row = 1, col = 1;
        render_predictions_panel(&v->state, v->jul_utc,
                                 &row, col);

        char viewers[160];
        viewer_roster_users(v, viewers, sizeof viewers);
        int srow = row + 1;
        status_panel_t sp;
        memset(&sp, 0, sizeof sp);
        sp.control_mode  = 0;
        sp.operator_user = v->operator;
        sp.viewers       = viewers[0] ? viewers : "(none)";
        sp.carrier_hz    = v->carrier_hz;
        sp.have_rotator  = v->has_rotator;
        sp.current_az    = v->state.antenna_rotator.azimuth;
        sp.current_el    = v->state.antenna_rotator.elevation;
        sp.target_az     = v->state.antenna_rotator.target_azimuth;
        sp.target_el     = v->state.antenna_rotator.target_elevation;
        sp.flip          = v->state.antenna_rotator.flip_mode_pass;
        render_status_panel(&sp, &srow, col);

        // Auto-tcmd run progress, mirrored from the operator's modal.
        // Red while the run is live (the PA is being keyed on a timer),
        // matching the T/R panel's red-while-transmitting convention.
        if (v->auto_on) {
            srow++;
            int at_running = strcmp(v->auto_state, "running") == 0;
            if (at_running) attron(COLOR_PAIR(1));
            mvprintw(srow++, col, "%15s   %d/%d sent (%s)",
                     "auto-tcmd",
                     v->auto_sent, v->auto_total,
                     v->auto_state[0] ? v->auto_state : "?");
            if (at_running) attroff(COLOR_PAIR(1));
            clrtoeol();
        }

        int prow = 5;
        report_position(&v->state, &prow, 50);
        // RX panel directly below position (matches the operator's layout).
        prow++;
        render_rx_panel(&v->rx_panel, &prow, 50);

        // Vertical ribbon on the right edge, same placement as the
        // operator. The wire delivers the same '.'/'-' chars the
        // operator's collector built so both screens crawl in sync.
        int ribbon_col = COLS - 2;
        int ribbon_top = 1;
        int ribbon_bot = LINES - 2;
        if (ribbon_col >= 64 && ribbon_bot > ribbon_top) {
            render_ribbon_vertical(&v->rx_panel,
                                   ribbon_top, ribbon_bot, ribbon_col);
        }

        int tx_log_row = LINES - TX_LOG_SIZE - 2;
        if (tx_log_row >= 17) {
            render_tx_log_panel(&v->state, tx_log_row, 1);
        }
    }

    time_t now = time(NULL);
    long stale_s = v->last_event > 0
        ? (long)(now - v->last_event)
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
    int show_confirm = (v->confirm_until > 0
                        && now < v->confirm_until);
    int show_mirror  = !show_confirm
        && (v->cmd_active || v->cmd_status[0]);

    move(LINES - 1, 0);
    clrtoeol();
    if (show_mirror) {
        if (v->cmd_active) {
            mvprintw(LINES - 1, 0, ":%s", v->cmd_buf);
            addch(' ' | A_REVERSE);
        } else {
            mvprintw(LINES - 1, 0, "%s", v->cmd_status);
        }
    } else {
        attron(A_REVERSE);
        char foot[200];
        if (show_confirm) {
            snprintf(foot, sizeof foot,
                " %s     Take control from %s? y/N ",
                status,
                v->operator[0] ? v->operator : "?");
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
    viewer_t v = {0};
    v.running = 1;
    sso_ipc_client_t *cli = sso_ipc_client_connect("simple_sat_ops");
    if (cli == NULL) {
        fprintf(stderr,
                "simple_sat_ops viewer: connect failed: %s\n",
                strerror(errno));
        return EXIT_FAILURE;
    }
    sso_ipc_client_on_event(cli, viewer_on_event, &v);

    // Viewer doesn't run SGP4 or load a TLE — it deposits every
    // displayed value into v.state from the broadcast and uses
    // the same render helpers the operator does. zero-init is enough.
    memset(&v.state, 0, sizeof v.state);

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

    init_window(&v.state);
    int last_connected = -1;
    time_t last_render = 0;
    viewer_render(&v, sso_ipc_client_is_connected(cli));
    last_render = time(NULL);
    int confirm_was_armed = 0;
    while (v.running) {
        int rc = sso_ipc_client_step(cli, 200);
        if (rc < 0) break;
        int connected = sso_ipc_client_is_connected(cli);
        time_t now = time(NULL);
        int confirm_armed = (v.confirm_until > 0
                             && now < v.confirm_until);
        if (!confirm_armed && confirm_was_armed) {
            // Window just expired — re-render to drop the confirm footer.
            v.event_pending = 1;
        }
        confirm_was_armed = confirm_armed;
        if (v.event_pending
            || connected != last_connected
            || (now - last_render) >= 5) {
            viewer_render(&v, connected);
            v.event_pending = 0;
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
                viewer_take_control(&v, cli, argv0);
                v.confirm_until = 0;
                v.event_pending = 1;
            } else {
                // Anything else cancels the confirm window.
                v.confirm_until = 0;
                v.event_pending = 1;
            }
            continue;
        }
        if (key == 'q' || key == 'Q' || key == 27 /* Esc */) {
            v.running = 0;
        } else if (key == 'c' || key == 'C') {
            v.confirm_until = now + VIEWER_CONFIRM_WINDOW_S;
            v.event_pending = 1;
        }
    }

    endwin();
    tui_release_stderr();
    sso_ipc_client_close(cli);
    return 0;
}

// --- --self-test report -------------------------------------------
//
// Prints the resolved configuration after CLI parse + HMAC keyfile
// load, in a stable key: value layout so test harnesses can grep it.
// Every line is "key: value" with no surrounding quoting; values are
// short enough to fit on one line. The "self-test:" header line is
// the contract — downstream scripts can use it as a sentinel.

static const char *hmac_status_str(hmac_display_status_t s)
{
    switch (s) {
        case HMAC_DISPLAY_OK:      return "ok";
        case HMAC_DISPLAY_MISSING: return "missing";
        case HMAC_DISPLAY_BAD:     return "bad";
        case HMAC_DISPLAY_UNSET:   /* fall through */
        default:                   return "unset";
    }
}

static const char *baud_str(int speed_const)
{
    // antenna_rotator stores the serial speed as the POSIX termios
    // constant (B600 etc), not the integer baud rate. Map the ones
    // the rotator actually uses; "?" everything else so a change to
    // antenna_rotator.c shows up in the report instead of crashing
    // it.
    switch (speed_const) {
        case B600:    return "600";
        case B1200:   return "1200";
        case B2400:   return "2400";
        case B4800:   return "4800";
        case B9600:   return "9600";
        case B19200:  return "19200";
        case B38400:  return "38400";
        case B57600:  return "57600";
        case B115200: return "115200";
        default:      return "?";
    }
}

static void self_test_report(const state_t *state, FILE *out, int argc, char **argv)
{
    fprintf(out, "self-test: simple_sat_ops configuration snapshot\n");
    fprintf(out, "version: %s\n", sso_version_string());

    // Echo the command line so the report is self-describing — the
    // reader can see at a glance which flags produced this snapshot.
    fprintf(out, "argv:");
    for (int i = 1; i < argc; ++i) {
        fprintf(out, " %s", argv[i]);
    }
    fprintf(out, "\n");

    // Mode. apply_args has already set state->control_mode / state->viewer_mode
    // (the latter only via the auto-probe path, which --self-test
    // skips). Standalone is the default.
    const char *mode = state->control_mode ? "operator (--control)"
                     : state->viewer_mode  ? "viewer (auto-detected)"
                                       : "standalone";
    fprintf(out, "mode: %s\n", mode);

#ifdef SSO_WITH_SDR
    int sdr_compiled = 1;
#else
    int sdr_compiled = 0;
#endif
#ifdef WITH_USRP_B210
    int uhd_compiled = 1;
#else
    int uhd_compiled = 0;
#endif
#ifdef WITH_RTL_SDR
    int rtl_compiled = 1;
#else
    int rtl_compiled = 0;
#endif
    fprintf(out, "build: sdr=%s (uhd=%s, rtl-sdr=%s)\n",
            sdr_compiled ? "on" : "off",
            uhd_compiled ? "on" : "off",
            rtl_compiled ? "on" : "off");

    fprintf(out, "tle: %s\n",
            state->prediction.tles_filename
                ? state->prediction.tles_filename
                : "(auto-discover at startup)");

    // HMAC --- the operator's banner-and-sign state. CTS1 firmware
    // expects every uplink to be HMAC-signed; the dispatcher refuses
    // to key the PA if state->hmac_key_len == 0, so this line is the
    // single most-important pre-flight check.
    fprintf(out,
            "hmac: %s (path=%s, status=%s, bytes=%zu)\n",
            state->hmac_key_len > 0 ? "enabled (default)" : "DISABLED",
            state->hmac_keyfile_path[0] ? state->hmac_keyfile_path : "(unresolved)",
            hmac_status_str(state->hmac_display_status),
            state->hmac_key_len);

    // Doppler --- both the display correction and the TX-side burst
    // staging key off state->doppler_correction_enabled. On by
    // default; --no-doppler-correction clears it. Report RX and TX
    // separately so the operator can see where the correction is
    // applied: RX is software (sw_nco on post-decim IQ, no hardware
    // LO retune mid-pass — the threshold-driven retune was removed
    // because it caused phase resets in the coherent demod), TX is
    // hardware (b210_rx_tx_core_burst tunes the B210 LO to the
    // Doppler-corrected frequency for every burst).
    fprintf(out, "doppler-correction: %s\n",
            state->doppler_correction_enabled ? "enabled (default)"
                                              : "DISABLED (--no-doppler-correction)");
    fprintf(out, "doppler-rx: %s (software sw_nco on post-decim IQ; hardware LO fixed)\n",
            state->doppler_correction_enabled ? "enabled" : "disabled");
    fprintf(out, "doppler-tx: %s (hardware SDR LO retune per burst, f=carrier/(1-rr/c))\n",
            (!sdr_compiled || state->without_b210)
                ? "n/a (no SDR)"
                : (state->doppler_correction_enabled ? "enabled" : "disabled"));
    fprintf(out, "uplink-nominal-mhz: %.6f\n",
            state->nominal_uplink_frequency_hz / 1e6);
    fprintf(out, "downlink-nominal-mhz: %.6f\n",
            state->nominal_downlink_frequency_hz / 1e6);
    fprintf(out, "rx-lo-offset-khz: %+.3f\n", state->rx_lo_offset_hz / 1000.0);

    // TX safety / staging gates the operator might have set.
    fprintf(out, "tx-no-tx: %s\n", state->no_tx ? "on (--no-tx)" : "off");
    fprintf(out, "tx-dry-run: %s\n", state->tx_dry_run ? "on (--tx-dry-run)" : "off");
    fprintf(out, "tx-auto-tcmd-file: %s\n",
            state->auto_tcmd_file_path[0] ? state->auto_tcmd_file_path : "(none)");

    // Hardware. The flags don't reflect "is it physically present" —
    // they reflect "does this run intend to talk to it". The actual
    // open happens after the self-test exit.
    fprintf(out, "rotator: %s (device=%s, baud=%s)\n",
            state->run_with_antenna_rotator ? "enabled"
                                            : "disabled (--without-rotator)",
            state->antenna_rotator.device_filename,
            baud_str(state->antenna_rotator.serial_speed));
    fprintf(out, "sdr: %s\n",
            (!sdr_compiled || state->without_b210)
                ? "disabled (--without-b210 or build-time)"
                : "enabled");

    fprintf(out, "live-waterfall: %s\n",
            state->run_live_waterfall ? "on (--live-waterfall)" : "off");

    fprintf(out, "pass-folder-seed: %s\n",
            state->pass_folder[0] ? state->pass_folder : "(auto)");

    // Observer location. apply_args stored these in radians on the
    // ephem struct — convert back to degrees for the report.
    fprintf(out, "observer-lat-deg: %.6f\n",
            state->prediction.observer_ephem.position_geodetic.lat * 180.0 / M_PI);
    fprintf(out, "observer-lon-deg: %.6f\n",
            state->prediction.observer_ephem.position_geodetic.lon * 180.0 / M_PI);
    fprintf(out, "observer-alt-m: %.1f\n",
            state->prediction.observer_ephem.position_geodetic.alt * 1000.0);

    fprintf(out, "self-test: ok\n");
    fflush(out);
}

// --- main ---------------------------------------------------------

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "simple_sat_ops")) return 0;
    state_t state = {0};
    state.prediction.predicted_max_elevation = -180.0;
    // Seed the TX-compose "remembered" draft. state_t is zero-initialised,
    // so set the first-open defaults explicitly: the CTS1 prefix and 80 dB.
    snprintf(state.tx_last_payload, sizeof state.tx_last_payload, "CTS1+");
    snprintf(state.tx_last_power,   sizeof state.tx_last_power,   "80.0");
    // Non-zero TX-core defaults (state_t is zero-initialised). The Doppler
    // carrier falls back to the bare nominal until SGP4 has a range rate;
    // preroll matches the tx_burst_run fallback. Both may be overridden in
    // apply_args (--tx-preroll-ms) / the per-tick Doppler refresh.
    state.tx_freq_hz_doppler = (long) FRONTIERSAT_CARRIER_HZ;
    state.tx_preroll_ms      = 200;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);
    // --help / --help-full historically exited 2; preserve that. Every
    // parse-or-runtime failure inside apply_args collapses to exit 1
    // (the old code returned 1 for too-many-positionals / startup
    // errors and 3 for an unknown --option; both are now PARSE_ERROR ->
    // 1, each still printing its own distinct stderr message).
    int status = apply_args(&state, argc, argv, jul_utc, HELP_OFF);
    if (status != 0) {
        return status;
    }

    // Bare invocation found a running operator — run as a read-only
    // viewer and skip the rest of the operator/standalone bring-up.
    if (state.viewer_mode) {
        return run_viewer(argv[0]);
    }

#ifdef SSO_WITH_SDR
    // Pin the SSO+ @tssent dedup key for this session: the startup UTC,
    // truncated to the minute. Constant for the life of the process so the
    // satellite runs an SSO+ time-sync once per pass. See sso_pseudo.h.
    state.sso_pass_tssent_ms = (sso_now_utc_ms() / 60000LL) * 60000LL;
#endif

    // Resolve + load the HMAC keyfile. The bytes feed every TX burst's
    // AX100 frame (CTS1 firmware expects HMAC on every uplink), AND
    // light the operator banner — "(N bytes ok)" means TX is armed,
    // "(MISSING)" / "(BAD)" means the next TX request will be refused
    // before keying the PA. If --hmac-keyfile= wasn't given, fall back
    // to hmac_keyfile_default_path (shared first, per-user second).
    if (state.hmac_keyfile_path[0] == '\0') {
        if (hmac_keyfile_default_path(state.hmac_keyfile_path,
                                      sizeof state.hmac_keyfile_path) != 0) {
            state.hmac_keyfile_path[0] = '\0';
            state.hmac_display_status  = HMAC_DISPLAY_MISSING;
        }
    }
    if (state.hmac_keyfile_path[0] != '\0') {
        struct stat st;
        if (stat(state.hmac_keyfile_path, &st) != 0) {
            state.hmac_display_status = HMAC_DISPLAY_MISSING;
        } else {
            ssize_t got = hmac_keyfile_load(state.hmac_keyfile_path,
                                            state.hmac_key,
                                            sizeof state.hmac_key);
            if (got > 0) {
                state.hmac_display_status = HMAC_DISPLAY_OK;
                state.hmac_key_len        = (size_t) got;
            } else {
                state.hmac_display_status = HMAC_DISPLAY_BAD;
                state.hmac_key_len        = 0;
                memset(state.hmac_key, 0, sizeof state.hmac_key);
            }
        }
    }

    // --self-test: configuration snapshot, then exit. Runs after CLI
    // parse + HMAC keyfile load so every TX-relevant policy is
    // resolved; runs BEFORE the IPC socket bind, the rotator open,
    // the B210 open, and load_tle, so the process makes no observable
    // changes to the rest of the system.
    if (state.self_test) {
        self_test_report(&state, stdout, argc, argv);
        return 0;
    }

    // Telecommand-agenda lint gate. When a --tc-file was given, lint it
    // against the firmware's telecommand set (names, argument counts,
    // CTS1+...! framing, length limits) BEFORE bringing up anything that
    // can key the PA. Lint errors mean a command would be rejected (or
    // worse, mis-parsed) by the satellite, so refuse to start unless the
    // operator explicitly accepts the risk. Warnings (e.g. a command not
    // meant for routine flight operation) are printed but do not block.
    if (state.auto_tcmd_file_path[0] != '\0') {
        int tc_warns = 0;
        int tc_errs = tcmd_lint_file(state.auto_tcmd_file_path, stderr, &tc_warns);
        if (tc_errs > 0 && !state.ignore_tc_errors) {
            fprintf(stderr,
                "simple_sat_ops: %d error%s detected in the --tc-file content (%s).\n"
                "Refusing to start. Fix the agenda, or re-run with\n"
                "--ignore-at-your-peril-all-tc-errors to bypass this check.\n",
                tc_errs, tc_errs == 1 ? "" : "s", state.auto_tcmd_file_path);
            return EXIT_FAILURE;
        }
        if (tc_errs > 0) {
            fprintf(stderr,
                "simple_sat_ops: %d telecommand error%s in %s -- proceeding anyway "
                "(--ignore-at-your-peril-all-tc-errors).\n",
                tc_errs, tc_errs == 1 ? "" : "s", state.auto_tcmd_file_path);
        } else if (tc_warns > 0) {
            fprintf(stderr,
                "simple_sat_ops: %d telecommand warning%s in %s (see above); proceeding.\n",
                tc_warns, tc_warns == 1 ? "" : "s", state.auto_tcmd_file_path);
        }
    }

    // Audit + operator IPC bring-up.
    state.operator_user = sso_unix_user();
    sso_audit_start("simple_sat_ops",
                    state.control_mode ? "operator" : "standalone");
    // Record the exact command line so post-incident review can tie
    // every operator action back to the flags the session was started
    // with (recording mode, --tx settings, TLE, etc.). One line, tab-
    // safe (sso_audit's sanitiser replaces tabs/newlines with spaces).
    {
        char argv_buf[1024];
        size_t off = 0;
        argv_buf[0] = '\0';
        for (int i = 0; i < argc && off + 2 < sizeof argv_buf; ++i) {
            int n = snprintf(argv_buf + off, sizeof argv_buf - off,
                             "%s%s", (i == 0) ? "" : " ", argv[i]);
            if (n <= 0) break;
            off += (size_t) n;
            if (off >= sizeof argv_buf) { off = sizeof argv_buf - 1; break; }
        }
        sso_audit_event("argv", argv_buf);
    }
    if (state.control_mode) {
        // Refuse if another simple_sat_ops --control is already
        // bound — two operators driving the same SDR / rotator is
        // exactly the failure mode the IPC server existed to avoid.
        // The probe connects as a transient viewer, reads the
        // operator's identity off the welcome reply, and disconnects.
        char existing_user[64]    = {0};
        char existing_folder[256] = {0};
        int op_status = sso_operator_verify("viewer",
                                             existing_folder,
                                             sizeof existing_folder,
                                             existing_user,
                                             sizeof existing_user);
        if (op_status == SSO_OP_OK || op_status == SSO_OP_MISMATCH) {
            pid_t op_pid = 0;
            const char *who = existing_user[0] ? existing_user : "?";
            if (read_operator_pid(&op_pid) == 0) {
                fprintf(stderr,
                    "simple_sat_ops: --control refused: operator already "
                    "running as user=%s pid=%d.\n"
                    "  To take over, run a viewer (no --control) and press\n"
                    "  'c' then 'y' to force-claim; the running operator\n"
                    "  will yield and your viewer will re-exec into --control.\n",
                    who, (int) op_pid);
            } else {
                fprintf(stderr,
                    "simple_sat_ops: --control refused: operator already "
                    "running as user=%s.\n", who);
            }
            char det[96];
            snprintf(det, sizeof det,
                     "existing_user=%s existing_pid=%d",
                     who, (int) op_pid);
            sso_audit_event("control-refused", det);
            return EXIT_FAILURE;
        }

        state.ipc = sso_ipc_server_open("simple_sat_ops");
        if (state.ipc == NULL) {
            // Probe said "no operator" yet bind still failed — most
            // likely a stale socket / pid file from a crashed
            // previous operator (or a vanishingly-rare race with
            // another --control starting at the same instant).
            // Either way, refuse so we don't quietly drive hardware
            // alongside something else.
            fprintf(stderr,
                "simple_sat_ops: --control: socket bind failed. If this is "
                "from a crashed previous operator, remove "
                "/run/sso/simple_sat_ops.{sock,pid} and retry.\n");
            sso_audit_event("ipc-bind-failed", "");
            return EXIT_FAILURE;
        }
        sso_ipc_server_on_event(state.ipc, ipc_on_event, &state);
        tui_install_yield_handler();
        fprintf(stderr, "simple_sat_ops: operator=%s ipc=on\n",
                state.operator_user);
    }

    /* Parse TLE data */
    int tle_status = load_tle(&state.prediction);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.prediction.satellite_ephem.tle);

    // Seed the retarget guard with the startup TLE so a `:retarget` on
    // the same file is correctly a no-op.
    snprintf(state.target_tle_path, sizeof state.target_tle_path, "%s",
             state.prediction.tles_filename
                 ? state.prediction.tles_filename : "");

    // With a fresh TLE loaded, find the upcoming pass and stand up
    // /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/ for it before the
    // tracking loop opens ncurses. Only on --control — the
    // standalone-tracker / dev path leaves Operations/ alone.
    if (state.control_mode) {
        UTC_Calendar_Now(&utc, &tv);
        double jul_now = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_now);
        setup_pass_folder(&state, jul_now);
        if (state.pass_folder[0]) {
            generate_pass_plot(&state, state.pass_folder, jul_now);
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
        // Spawn the async worker. From here on, every serial roundtrip
        // happens on the worker thread; the main loop only reads the
        // snapshot via main_rotator_refresh_targets_from_snapshot() and
        // posts SETs via main_rotator_submit_set().
        if (antenna_rotator_async_open(&state.rot_async,
                                        &state.antenna_rotator, 0.5) != 0) {
            fprintf(stderr, "Error spawning antenna rotator worker\n");
            return EXIT_FAILURE;
        }
        // Adopt whatever extended position the SPID is already at so the
        // unwrapped accumulator starts grounded in reality. We wait
        // briefly for the worker's first STATUS read; the timeout is
        // bounded so a missing controller doesn't hang startup.
        //
        // The seed snapshot also overwrites target_* with the current
        // physical position — fine when nobody asked for a specific park
        // position, but a problem when the operator passed
        // --rotator-target-azimuth / --rotator-target-elevation: those
        // user-specified targets would be silently clobbered before T
        // ever fired. Snapshot them and restore after seeding.
        double sav_az    = state.antenna_rotator.target_azimuth;
        double sav_el    = state.antenna_rotator.target_elevation;
        double sav_az_uw = state.antenna_rotator.target_azimuth_unwrapped;
        int    sav_uw_ok = state.antenna_rotator.unwrapped_target_valid;
        if (antenna_rotator_async_wait_first_status(state.rot_async, 1500) != 0
            || main_rotator_refresh_targets_from_snapshot(&state) != 0) {
            fprintf(stderr, "Warning: could not read SPID position; "
                            "check that the Rot2ProG is in 'A' mode\n");
        }
        if (state.antenna_rotator.fixed_target) {
            state.antenna_rotator.target_azimuth            = sav_az;
            state.antenna_rotator.target_elevation          = sav_el;
            state.antenna_rotator.target_azimuth_unwrapped  = sav_az_uw;
            state.antenna_rotator.unwrapped_target_valid    = sav_uw_ok;
        }
    }

    // --calibrate-rotator: drive the antenna across known arcs to
    // measure deg/s on each axis, save the result to disk, and exit
    // without entering the operator UI. Requires the safety interlock
    // so a stray flag in a script can't move hardware.
    if (state.calibrate_rotator) {
        if (!state.have_antenna_rotator) {
            fprintf(stderr, "--calibrate-rotator: no rotator open "
                            "(was --without-rotator passed?)\n");
            return EXIT_FAILURE;
        }
        if (!state.confirm_rotator_calibrate) {
            fprintf(stderr,
                    "--calibrate-rotator will physically move the antenna.\n"
                    "Confirm the mast area is clear, then re-run with\n"
                    "  --calibrate-rotator --confirm-rotator-calibrate\n");
            return EXIT_FAILURE;
        }
        double az_dps = 0.0, el_dps = 0.0;
        rotator_calibrate_result_t cres = rotator_calibrate_run(
            state.rot_async, &az_dps, &el_dps, stderr);
        fprintf(stderr, "calibrate: result = %s\n",
                rotator_calibrate_result_name(cres));
        if (cres == ROTATOR_CALIBRATE_OK) {
            fprintf(stderr,
                    "calibrate: saved rates az=%.3f deg/s el=%.3f deg/s\n",
                    az_dps, el_dps);
        }
        // Shutdown cleanly — the operator UI never started, but the
        // rotator FD and worker are open.
        if (state.rot_async != NULL) {
            antenna_rotator_async_close(state.rot_async);
            state.rot_async = NULL;
        }
        if (state.have_antenna_rotator) {
            antenna_rotator_disconnect(&state.antenna_rotator);
            state.have_antenna_rotator = 0;
        }
        return (cres == ROTATOR_CALIBRATE_OK) ? 0 : 1;
    }

    // Normal startup: load saved rotator rates from the calibration
    // file. Missing or malformed file -> pursuit planner stays
    // disabled (Phase 2 hooks this in front of the track loop; Phase
    // 1 just loads + warns so the bench can see the values).
    if (state.have_antenna_rotator) {
        if (pursuit_load_rotator_rates(&state.pursuit_az_dps,
                                        &state.pursuit_el_dps) == 0) {
            fprintf(stderr,
                    "pursuit: loaded slew rates az=%.3f deg/s el=%.3f deg/s\n",
                    state.pursuit_az_dps, state.pursuit_el_dps);
        } else {
            fprintf(stderr,
                    "pursuit: no calibration on disk; run "
                    "`simple_sat_ops --calibrate-rotator "
                    "--confirm-rotator-calibrate` to enable lead-aim\n");
        }
    }

    // T/R antenna switch — auto-probe before ncurses takes the screen,
    // so a "not connected" warning lands on the terminal. Absent or
    // inaccessible hardware is non-fatal; the UI panel reads "not
    // connected" and the program runs normally.
    if (state.run_with_tr_switch) {
        if (tr_switch_init(&state.tr_switch) == 0) {
            state.have_tr_switch = 1;
        } else {
            fprintf(stderr,
                    "T/R switch: could not open %s (skipping; "
                    "pass --without-tr-switch to silence)\n",
                    state.tr_switch.device_filename
                        ? state.tr_switch.device_filename : "?");
        }
    }

#ifdef SSO_WITH_SDR
    // Open the B210 once, here, before ncurses init — soft-fail on any
    // UHD error so a dev host without a device can still run the UI.
    // rx_session takes ownership of the core; we drop our local handle
    // afterwards so main never touches UHD off-thread.
    if (state.control_mode && !state.without_b210) {
        // B210 RX rate doubled from the original 240 kHz / sps=5 to
        // 480 kHz / sps=10 (after the integer-5 decimation FIR). That
        // gives the modem_fsk clock-recovery loop the same oversampling
        // headroom the gr-satellites / AIT chain has (sps=6 with PFB-
        // Gardner) and then some, which is worth ~1-2 dB at marginal
        // SNR on real captures. The post-decim signal still only
        // carries the FrontierSat ±10 kHz FSK, so the decim FIR
        // cutoff stays at 18 kHz — narrower than the new 48 kHz
        // Nyquist, so the filter rejects more noise than it did at
        // the old 24 kHz Nyquist. IQ files double in size; with the
        // sustained-write rate at 96 kHz·2·2 = 384 kB/s, a 10-minute
        // pass produces ~230 MB which the laptop SSD has no trouble
        // with.
        b210_rx_tx_core_params_t cp = {
            // Tune the SDR LO off the nominal carrier so the corrected
            // signal lands well off DC. rx_lo_offset_hz is SIGNED:
            // positive → LO above nominal (signal at negative baseband),
            // negative → LO below (signal at positive baseband). Default
            // -25 kHz keeps existing pipelines unchanged; operator can
            // shift to dodge fixed-pattern noise.
            .freq_hz         = state.nominal_downlink_frequency_hz
                             + state.rx_lo_offset_hz,
            .rate_hz         = 480000.0,
            .gain_db         = state.rx_gain_db,
            .bw_hz           = -1.0,
            .fm_fullscale_hz = 25000.0,
            .rx_antenna      = "RX2",
            // fir_decim budget:
            //   - operator LO offset clamped ±45 kHz (apps/main.c
            //     KEY_LO_OFFSET clamp);
            //   - Doppler swing ±10 kHz for a typical LEO pass;
            //   - FM envelope ±5 kHz around the carrier;
            //   - Nyquist after decim by 5 = ±48 kHz.
            // Cutoff 42 kHz with 256 taps gives ~6 kHz transition
            // before Nyquist and lets the carrier sit anywhere
            // inside the clamp without the LPF rolling off half the
            // beacon. The carrier-at-DC convention moved to the
            // decode-only buffer (see b210_rx_tx_core.c); the IQ
            // tap now carries the carrier at +lo_offset baseband.
            .decim_factor    = 5u,
            .decim_cutoff_hz = 42000.0,
            .decim_taps      = 256u,
            // FM-path LO compensation: the core's second NCO cancels
            // the operator's lo_offset, the UHD-reported tune residual
            // (target − actual, from the AD9361 PLL step), AND the
            // persistent per-host carrier trim (TCXO calibration). The
            // carrier lands at exactly DC for every downstream consumer
            // (.iq sidecar, live waterfall, shadow IQ decoder, FM
            // discriminator).
            .rx_dc_offset_track  = state.rx_dc_offset_track,
            .rx_iq_balance_track = state.rx_iq_balance_track,
            .fm_lo_compensation_hz = state.rx_lo_offset_hz,
            .carrier_trim_hz       = carrier_trim_load_hz(),
            // SDR backend selection: type (default auto), and the UHD
            // clone overrides. --sdr-device routes to the UHD device
            // args when given (e.g. "serial=..."); --uhd-args takes
            // precedence and is passed verbatim.
            .backend_type        = state.sdr_type,
            .device_args         = state.sdr_device[0] ? state.sdr_device : "type=b200",
            .uhd_args_override   = state.uhd_args[0] ? state.uhd_args : NULL,
            .fpga_image_path     = state.sdr_fpga[0] ? state.sdr_fpga : NULL,
            // RTL-SDR dongle index (UHD ignores it; for UHD use --uhd-args).
            .device_index        = state.sdr_device[0] ? atoi(state.sdr_device) : 0,
        };
        b210_rx_tx_core_t *core = NULL;
        if (b210_rx_tx_core_open(&cp, &core) != 0) {
            fprintf(stderr,
                "simple_sat_ops: B210 open failed — continuing without RF "
                "(rotator + UI only). Pass --without-b210 to silence.\n");
            sso_audit_event("b210-open-failed", "");
        } else {
            fprintf(stderr,
                "simple_sat_ops: SDR open at %.6f MHz (post-decim rate %.0f, "
                "tx=%s)\n",
                b210_rx_tx_core_actual_freq(core) / 1e6,
                b210_rx_tx_core_actual_rate(core),
                b210_rx_tx_core_can_tx(core) ? "yes" : "no (RX-only)");
            {
                char det[256];
                snprintf(det, sizeof det,
                    "freq_hz=%.0f rate_hz=%.0f lo_offset_hz=%.0f",
                    b210_rx_tx_core_actual_freq(core),
                    b210_rx_tx_core_actual_rate(core),
                    state.rx_lo_offset_hz);
                sso_audit_event("b210-open", det);
            }
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
                .pass_folder       = state.pass_folder[0] ? state.pass_folder : NULL,
                .want_wav          = 1,
                .tle_path          = state.prediction.tles_filename,
                .sat_name          = state.prediction.satellite_ephem.tle.sat_name[0]
                                     ? state.prediction.satellite_ephem.tle.sat_name
                                     : NULL,
                .session_dir       = state.pass_folder[0] ? state.pass_folder : NULL,
                .lo_offset_hz      = state.rx_lo_offset_hz,
            };
            if (rx_session_open(&state.rx_session, &rxp, core) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: rx_session_open failed — closing B210\n");
                b210_rx_tx_core_close(core);
            }
            // rx_session_open succeeded → it owns `core` now.
            core = NULL;
            // --always-record: start WAV + .iq + sidecars right now,
            // before any pass logic gets a chance to gate them. The
            // per-pass start/stop block in the tracking loop checks
            // state.always_record and skips itself when this is on.
            if (state.always_record && state.rx_session) {
                rx_session_request_wav_start(state.rx_session);
                fprintf(stderr,
                    "simple_sat_ops: --always-record on — WAV/IQ "
                    "capture started, pass gating disabled\n");
                sso_audit_event("rec-start", "trigger=always-record");
            }
        }
    }
#endif

    /* Tracking loop */
    double jul_idle_start = 0;  // last-tracked timestamp

    // Capture the cooked terminal modes BEFORE ncurses switches the tty
    // to raw, so the crash handler can put it back deterministically.
    tui_save_termios();

    init_window(&state);

    // Catch a fatal device fault (or Ctrl-C) now that the screen is up,
    // so it restores the terminal instead of dumping a raw abort on it.
    install_signal_handlers();
    // Let the crash handler reach the live-waterfall child (owned here) so
    // it doesn't orphan when a device-loss abort skips normal teardown.
    tui_register_waterfall_pid(&g_live_waterfall_pid);

    // int (not char) — getch returns KEY_* codes well above 127 for
    // arrow keys / function keys / KEY_BACKSPACE etc. A signed char
    // would silently truncate those into bogus low-byte values, which
    // is what made KEY_LEFT (260) look like Ctrl-D (4) in the modal
    // handlers and broke field editing inside the auto-tcmd modal.
    int key = ERR;
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
    mvprintw(keyboard_info_row++, 3, "%s", "[/]- Jog azimuth -5/+5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "{/}- Jog azimuth -1/+1 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", ",/.- Jog elevation -5/+5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "</>- Jog elevation -1/+1 deg");
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
    // All three are consumed only inside #ifdef WITH_USRP_B210; the
    // attribute keeps gcc-15 quiet on the non-B210 dev build.
    __attribute__((unused)) const double RECORDING_PREROLL_S  = 60.0;
    __attribute__((unused)) const double RECORDING_POSTROLL_S = 60.0;
    __attribute__((unused)) double t_recording_close_at = 0.0;

    while (state.running) {
        // Ctrl-C / SIGTERM: leave the loop and run the normal teardown
        // (endwin, rotator home, device close) instead of dying raw.
        if (tui_should_quit()) { state.running = 0; break; }
        double t_now = monotonic_seconds();
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_utc);

        // Drain whatever the T/R switch emitted since the last tick.
        // Non-blocking; the firmware beats every ~2.5 s so most ticks
        // read zero bytes.
        if (state.have_tr_switch) {
            tr_switch_pump(&state.tr_switch, t_now);
        }

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

#ifdef SSO_WITH_SDR
        // TX-side Doppler: transmit at the frequency that places the
        // nominal carrier at the satellite. Sign: range_rate_km_s > 0
        // when the satellite is receding (LOS end of a pass), so the
        // ground must transmit higher to compensate for redshift at
        // the moving receiver. Mirror of the RX-side correction,
        // applied to FRONTIERSAT_CARRIER_HZ (the actual TX carrier) —
        // state.doppler_uplink_frequency_hz is computed from the 2 m
        // amateur nominal and would give the wrong absolute frequency
        // here. Off when doppler_correction_enabled is false (e.g.
        // bench loopback) so RX and TX share one constant carrier.
        state.tx_freq_hz_doppler = tx_burst_doppler_freq_hz(
            FRONTIERSAT_CARRIER_HZ,
            state.prediction.satellite_ephem.range_rate_km_s,
            state.doppler_correction_enabled);
#endif

#ifdef SSO_WITH_SDR
        // Auto-record per pass: open the WAV 1 min before AOS (or as
        // soon as we're above the horizon, in case simple_sat_ops
        // started mid-pass), keep it open while the satellite is up,
        // close 1 min after LOS. Each pass gets its own auto-named
        // file in the pass folder. Note: this deliberately keys off the
        // satellite geometry (elevation + time-until-AOS) rather than
        // state.in_pass — the latter flips several minutes before AOS
        // (tracking_prep_time_minutes) so the rotator can pre-position,
        // which is far too early to start the WAV.
        //
        // --always-record disables this gate entirely: recording was
        // started once at rx_session_open and stays open until shutdown.
        if (state.rx_session && !state.always_record) {
            double sec_to_aos =
                state.prediction.predicted_minutes_until_visible * 60.0;
            int visible   = (state.prediction.satellite_ephem.elevation > 0.0);
            int in_preroll = (sec_to_aos > 0.0
                              && sec_to_aos <= RECORDING_PREROLL_S);
            int active = rx_session_wav_active(state.rx_session);
            if (!active && (visible || in_preroll)) {
                rx_session_request_wav_start(state.rx_session);
                t_recording_close_at = 0.0;
                char det[64];
                snprintf(det, sizeof det,
                    "trigger=%s sec_to_aos=%.1f el=%.1f",
                    visible ? "elevation" : "preroll",
                    sec_to_aos,
                    state.prediction.satellite_ephem.elevation);
                sso_audit_event("rec-start", det);
            } else if (active) {
                if (visible) {
                    t_recording_close_at = 0.0;  // cancel any pending close
                } else if (t_recording_close_at == 0.0) {
                    t_recording_close_at = t_now + RECORDING_POSTROLL_S;
                } else if (t_now >= t_recording_close_at) {
                    rx_session_request_wav_stop(state.rx_session);
                    t_recording_close_at = 0.0;
                    sso_audit_event("rec-stop", "trigger=postroll-expired");
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

        // Drive the second leg of a two-step home. The first leg drops a mid
        // waypoint to start the antenna unwinding; the final 'go to target'
        // must wait until the antenna has unwound far enough that the
        // controller's SHORT path to the target runs the SAME way as the
        // unwind -- i.e. it has reached the 0..180 zone on the unwinding side.
        // Until then the short path is the opposite (winding) way, so issuing
        // the target now sends it back around and it winds up (330 -> 360).
        //
        // Complication: after a SET the controller's STATUS reports the
        // just-commanded target (the mid waypoint) for a couple of seconds
        // before its feedback shows real motion. So a reading that still
        // equals the commanded mid is that echo, not the real position --
        // ignore it. The first reading that DIFFERS is the antenna's true
        // position; act on that. Mid-slew the real position is far from the
        // mid waypoint, so there's no echo-vs-arrival ambiguity. (Unwinds
        // past a full turn, prev > 360, would need more than one waypoint;
        // a single pass winds < 360, so one mid waypoint suffices.)
        if (state.antenna_rotator.homing_in_progress
            && state.have_antenna_rotator) {
            double final_az  = state.antenna_rotator.home_pending_final_az;
            double mid_az    = state.antenna_rotator.target_azimuth_unwrapped;
            double from_mid  = fabs(antenna_rotator_wrap_to_pm180(current_az - mid_az));
            double unwind    = final_az - mid_az;   // sign = unwind direction
            double remaining = antenna_rotator_wrap_to_pm180(final_az - current_az);
            int in_zone = (remaining == 0.0)
                       || ((remaining > 0.0) == (unwind > 0.0));
            // from_mid > tol => the reading is real feedback, not the post-SET
            // target echo. The two-step always starts out of the unwind zone
            // (|prev| > 180), so the stale pre-SET reading can't fire early.
            if (from_mid > HOME_ECHO_TOLERANCE_DEG && in_zone) {
                int rc = main_rotator_submit_set(&state, final_az, 0.0);
                if (rc == ANTENNA_ROTATOR_OK) {
                    state.antenna_rotator.antenna_is_moving = 1;
                }
                state.antenna_rotator.homing_in_progress = 0;
                state.antenna_rotator.home_pending_final_az = 0.0;
                char det[96];
                snprintf(det, sizeof det, "leg2 fired at az=%.1f -> %.1f",
                         current_az, final_az);
                sso_audit_event("home", det);
            }
        }
        // --scan-sky: drives a sky grid one target at a time, dwelling
        // SCAN_DWELL_S at each. Bypasses the satellite_tracking +
        // pass-timing gate below entirely, so the operator can scan
        // regardless of TLE / pass state. 's' stops mid-scan.
        if (state.scan.active) {
            scan_sky_tick(&state, t_now);
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
                    // Pre-sample the trajectory and ask the planner
                    // for a rate-feasible whole-pass aim sequence. On
                    // any failure (no calibration, planner unhappy,
                    // --without-rotator-pursuit) the helper leaves
                    // state.pursuit_plan zero and the track loop below
                    // falls back to today's aim-where-sat-is-now path.
                    main_pursuit_build_plan(&state, jul_utc);
                }
            }

            if (state.antenna_rotator.tracking
                && state.antenna_rotator.antenna_is_under_control) {
                if (!state.antenna_rotator.unwrapped_target_valid) {
                    if (main_rotator_refresh_targets_from_snapshot(&state)
                        != 0) {
                        state.antenna_rotator.tracking = 0;
                        main_pursuit_clear_plan(&state);
                    }
                } else if (!state.antenna_rotator.antenna_is_moving) {
                    double next_az = 0.0, next_el = 0.0;
                    double prev_unwrapped =
                        state.antenna_rotator.target_azimuth_unwrapped;
                    int    used_pursuit = 0;
                    if (state.pursuit_plan.waypoints != NULL
                        && pursuit_aim_at(&state.pursuit_plan, jul_utc,
                                          &next_az, &next_el) == 0) {
                        used_pursuit = 1;
                    }
                    if (!used_pursuit) {
                        double pred_az =
                            state.prediction.satellite_ephem.azimuth;
                        double pred_el =
                            state.prediction.satellite_ephem.elevation;
                        double mech_az = pred_az;
                        double mech_el = pred_el;
                        int half = 0;
                        // AOS->LOS progress: drives the boom-meridian
                        // lerp in flip mode. Clamped to [0, 1] inside
                        // the function. Ignored for non-flip passes.
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
                            state.antenna_rotator.target_azimuth_unwrapped =
                                mech_az;
                            state.antenna_rotator.flip_half = half;
                            prev_unwrapped =
                                state.antenna_rotator.target_azimuth_unwrapped;
                        }
                        next_az = antenna_rotator_accumulate_unwrapped(
                            prev_unwrapped, mech_az);
                        next_el = mech_el;
                    }
                    if (next_el < ANTENNA_ROTATOR_MINIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MINIMUM_ELEVATION;
                    } else if (next_el > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                    }
                    delta_az = next_az - prev_unwrapped;
                    // With a plan in play the elevation is part of the
                    // trajectory; respect it even below the horizon.
                    // The pre-pursuit fallback keeps the existing
                    // "only chase el while the sat is visible" rule.
                    if (used_pursuit
                        || state.antenna_rotator.flip_mode_pass
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
                            main_pursuit_clear_plan(&state);
                        } else {
                            int rc = main_rotator_submit_set(
                                &state, next_az, next_el);
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
                // Released the pass; tear down the planner so the
                // memory comes back and so the next pass / mid-pass
                // 'T' rebuilds against fresh state.
                main_pursuit_clear_plan(&state);
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
            low_disk_refresh(&state, t_now);
            rx_panel_data_t rxd;
            rx_panel_collect_local(&state, &rxd);
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
                render_tx_log_panel(&state, tx_log_row, 1);
            }
        }

        key = getch();
        if (state.tx_compose_active) {
            if (!tx_compose_handle_key(&state, key)) {
                tx_compose_close(&state);
            }
        } else if (state.auto_tcmd_active) {
            if (!auto_tcmd_handle_key(&state, key)) {
                auto_tcmd_close(&state);
            }
        } else if (state.cmd.active) {
            cmd_handle_key(key, &state);
        } else if (key == 'K') {
            keyboard_unlocked = !keyboard_unlocked;
        } else if (keyboard_unlocked) {
            switch (key) {
                case ':':
                    cmd_enter(&state);
                    break;
                case 'q':
                    state.running = 0;
                    break;
                case 'T':
                    if (state.scan.mode) {
                        scan_sky_start(&state);
                    } else {
                        start_tracking(&state);
                        if (state.antenna_rotator.fixed_target) {
                            char det[128];
                            snprintf(det, sizeof det,
                                "mode=fixed-target az=%.1f el=%.1f",
                                state.antenna_rotator.target_azimuth,
                                state.antenna_rotator.target_elevation);
                            sso_audit_event("track-on", det);
                        } else {
                            sso_audit_event("track-on",
                                state.prediction.satellite_ephem.tle.sat_name[0]
                                    ? state.prediction.satellite_ephem.tle.sat_name : "");
                        }
                    }
                    break;
                case 's':
                    if (state.scan.active) {
                        scan_sky_stop(&state, "user");
                    }
                    stop_tracking(&state);
                    break;
                case 'r':
                    stop_tracking(&state);
                    point_to_stationary_target(&state, 0.0, 0.0);
                    break;
                case '[':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ']':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '{':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, -1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '}':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, 1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ',':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '.':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '<':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, -1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '>':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, 1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case 't':
                    tx_compose_open(&state);
                    break;
                case 'A':
                    auto_tcmd_open(&state);
                    break;
                default:
                    break;
            }
        }

        if (redraw_due) {
            // Width-padded prints (not clrtoeol) so we don't wipe the
            // signal ribbon that paints over the right edge of these rows.
            mvprintw(keyboard_info_row, 3, "%s : %-8s", "Keyboard",
                     keyboard_unlocked ? "unlocked" : "LOCKED");
            mvprintw(keyboard_info_row + 2, 0, "%-18s",
                     state.antenna_rotator.antenna_is_moving
                         ? "Antenna moving"
                         : "Antenna stationary");
            t_last_redraw = t_now;
        }

        // Pump the modal's debounced preview broadcast before the
        // screen flush so the mirror line is current when we paint.
        tx_compose_pump(&state);
        // Drive the auto-tcmd burst loop. Queues state.tx_request when
        // it's time for the next send; the existing main-loop burst
        // handler below transmits and emits the SENT/NOT_SENT events.
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
        if (redraw_due || state.cmd.active || state.tx_compose_active
            || state.auto_tcmd_active) {
            cmd_render(&state);
            refresh();
            int show_hw_cursor = 0;
            if (state.tx_compose_active && state.tx_compose_win) {
                touchwin(state.tx_compose_win);
                wrefresh(state.tx_compose_win);
                tx_field_t f = state.tx_compose.focus;
                show_hw_cursor = (f == TXF_PAYLOAD || f == TXF_POWER);
            } else if (state.auto_tcmd_active && state.auto_tcmd_win) {
                touchwin(state.auto_tcmd_win);
                wrefresh(state.auto_tcmd_win);
                show_hw_cursor = (state.auto_tcmd.state != AUTO_STATE_RUNNING)
                              && auto_field_is_text(state.auto_tcmd.focus);
            } else if (state.cmd.active) {
                show_hw_cursor = 1;
            }
            curs_set(show_hw_cursor ? 1 : 0);
        }

#ifdef SSO_WITH_SDR
        // Signal ribbon sampler: push one peak-dBFS reading per second
        // so the ribbon in the RX panel rolls left in real time. Also
        // grab the iq_burst bright-bin count so the renderer can pick
        // a character that distinguishes broadband packets from a CW
        // carrier at the same peak level.
        if (state.rx_session && (t_now - state.ribbon_last_t) >= 1.0) {
            double peak = -90.0;
            rx_session_snapshot(state.rx_session, NULL, &peak, NULL,
                                NULL, NULL, 0);
            int burst_bins = 0;
            rx_session_burst_snapshot(state.rx_session, &burst_bins, NULL);
            ribbon_push(&state, peak, burst_bins);
            state.ribbon_last_t = t_now;

            // Live waterfall: launch the raylib viewer the first time
            // a recording's .iq path appears, OR if the pass switched
            // to a new path. We poll once per second on the same
            // cadence as the ribbon — cheap, and a single second of
            // lag at viewer-launch is invisible to the operator.
            if (state.run_live_waterfall) {
                char iq_path[512] = "";
                int  iq_rate      = 0;
                rx_session_iq_snapshot(state.rx_session,
                                       iq_path, sizeof iq_path,
                                       NULL, &iq_rate);
                if (iq_path[0]
                    && strcmp(iq_path, g_live_waterfall_iq) != 0) {
                    // Tear down a viewer pointed at a stale path.
                    if (g_live_waterfall_pid > 0) {
                        kill(g_live_waterfall_pid, SIGTERM);
                        waitpid(g_live_waterfall_pid, NULL, 0);
                        g_live_waterfall_pid = -1;
                    }
                    if (g_live_waterfall_stdin_fd >= 0) {
                        close(g_live_waterfall_stdin_fd);
                        g_live_waterfall_stdin_fd = -1;
                    }
                    snprintf(g_live_waterfall_iq,
                             sizeof g_live_waterfall_iq, "%s", iq_path);
                    char rate_arg[32];
                    snprintf(rate_arg, sizeof rate_arg,
                             "--rate=%d",
                             iq_rate > 0 ? iq_rate : 96000);
                    // pipe()+dup2 so the parent can shove
                    // line-based commands (e.g. "zoom 60\n") at the
                    // viewer's stdin.
                    int pfd[2] = {-1, -1};
                    if (pipe(pfd) != 0) { pfd[0] = pfd[1] = -1; }
                    pid_t pid = fork();
                    if (pid == 0) {
                        if (pfd[0] >= 0) {
                            close(pfd[1]);
                            dup2(pfd[0], STDIN_FILENO);
                            close(pfd[0]);
                        }
                        char *args[] = {
                            (char *) "live_waterfall",
                            (char *) g_live_waterfall_iq,
                            rate_arg,
                            NULL
                        };
                        execvp("live_waterfall", args);
                        _exit(127);
                    } else if (pid > 0) {
                        g_live_waterfall_pid = pid;
                        if (pfd[0] >= 0) close(pfd[0]);
                        g_live_waterfall_stdin_fd = pfd[1];
                    } else {
                        if (pfd[0] >= 0) close(pfd[0]);
                        if (pfd[1] >= 0) close(pfd[1]);
                    }
                }
                // Reap a viewer that the operator closed via its
                // window — non-blocking so the main loop never stalls.
                if (g_live_waterfall_pid > 0) {
                    int status;
                    pid_t r = waitpid(g_live_waterfall_pid,
                                      &status, WNOHANG);
                    if (r == g_live_waterfall_pid) {
                        g_live_waterfall_pid = -1;
                        g_live_waterfall_iq[0] = '\0';
                        if (g_live_waterfall_stdin_fd >= 0) {
                            close(g_live_waterfall_stdin_fd);
                            g_live_waterfall_stdin_fd = -1;
                        }
                    }
                }
            }
        }
        // Software Doppler tracking: the SDR LO stays fixed at the
        // nominal carrier (set once at session open) and we apply the
        // Doppler correction inside the IQ pump as a complex multiply.
        // No PLL glitches, sub-Hz resolution, and the displayed RX freq
        // updates smoothly. The threshold-driven hardware retune that
        // lived here previously fired every 1–10 seconds during a
        // pass and caused brief phase resets in the coherent demod
        // chain; this loop runs every tick at full precision.
        if (state.rx_session && state.doppler_correction_enabled) {
            double offset = state.doppler_downlink_frequency_hz
                          - state.nominal_downlink_frequency_hz;
            rx_session_set_doppler_offset(state.rx_session, offset);
        }
        if (state.rx_session) {
            double doppler_offset =
                state.doppler_downlink_frequency_hz
                - state.nominal_downlink_frequency_hz;
            rx_session_update_observer(state.rx_session,
                state.antenna_rotator.target_azimuth,
                state.antenna_rotator.target_elevation,
                state.prediction.satellite_ephem.range_km,
                state.prediction.satellite_ephem.range_rate_km_s,
                doppler_offset);
        }

        // Service a pending TX request. Three paths:
        //
        //   1. --tx-dry-run:    synthesize "ok" without touching the
        //                       SDR. Auto-tcmd + compose still exercise
        //                       all their UI state on dev hosts.
        //   2. state.rx_session up: real burst — submitted async to the
        //                       worker, which pauses RX, transmits and
        //                       resumes RX (~1 s). The main loop keeps
        //                       running between submit and poll so the
        //                       rotator, redraw, IPC and the next auto-
        //                       tcmd tick aren't frozen by the burst.
        //                       state.tx_request.pending stays set across
        //                       the in-flight window, so auto-tcmd will
        //                       not queue a second burst on top.
        //   3. neither:         reject so auto-tcmd can move on. The
        //                       operator must have started simple_sat_ops
        //                       --without-b210 without also passing
        //                       --tx-dry-run; just clear the pending
        //                       slot rather than deadlocking.
        if (state.tx_request.pending) {
            char summary[SSO_TX_TEXT_MAX];
            const char *outcome = NULL;
            int  on_air = 0;
            int  finished = 0;        // emit the result + clear pending this tick
            if (state.tx_dry_run) {
                snprintf(summary, sizeof summary, "%s",
                         state.tx_request.summary);
                outcome = "dry-run";   // composed but deliberately not keyed
                finished = 1;
            } else if (state.hmac_key_len == 0) {
                // CTS1 expects HMAC on every uplink. Without a valid
                // key the burst would go out unsigned and the satellite
                // would silently drop it. Refuse here so the operator
                // sees a clear error instead of letting it go out unsigned.
                snprintf(summary, sizeof summary, "%s",
                         state.tx_request.summary);
                outcome = "rejected: no HMAC key (see banner)";
                finished = 1;
            } else if (state.rx_session != NULL && !rx_session_can_tx(state.rx_session)) {
                // RX-only backend (e.g. RTL-SDR): never reaches the air.
                // Backstop for a stale queued burst that slipped past the
                // compose / auto-tcmd gates.
                snprintf(summary, sizeof summary, "%s", state.tx_request.summary);
                outcome = "rejected: RX-only SDR";
                finished = 1;
            } else if (state.rx_session != NULL) {
                if (!state.tx_inflight) {
                    if (rx_session_submit_burst(state.rx_session, &state.tx_request,
                                                 state.hmac_key, state.hmac_key_len) == 0) {
                        state.tx_inflight = 1;
                        // Stay pending; we'll poll on subsequent ticks.
                    } else {
                        // Worker refused (slot already busy or rxs error).
                        snprintf(summary, sizeof summary, "%s",
                                 state.tx_request.summary);
                        outcome = "rejected: rx_session busy";
                        finished = 1;
                    }
                } else {
                    rx_burst_result_t br;
                    int done = rx_session_poll_burst(state.rx_session, &br,
                                                      summary, sizeof summary);
                    if (done == 1) {
                        switch (br) {
                            case RX_BURST_OK:                 outcome = "ok"; on_air = 1; break;
                            case RX_BURST_NO_CORE:            outcome = "rejected: no B210"; break;
                            case RX_BURST_FRAME_BUILD_FAILED: outcome = "rejected: frame build"; break;
                            case RX_BURST_UHD_ERROR:          outcome = "uhd-err"; break;
                        }
                        state.tx_inflight = 0;
                        finished = 1;
                    }
                    // else: still in flight; fall through and let the
                    // rest of the main loop run.
                }
            } else {
                snprintf(summary, sizeof summary, "%s",
                         state.tx_request.summary);
                outcome = "rejected: no B210";
                finished = 1;
            }
            if (finished) {
                // A command that made it on the air gets a plain TX
                // record, nothing more: the ground station can confirm
                // it transmitted, but only the satellite can acknowledge,
                // and that arrives on the downlink, not here. Anything
                // that did NOT reach the air (rejected, dry-run, uhd-err)
                // gets a not-sent note carrying the reason.
                if (on_air) {
                    emit_tx_event_local(&state, SSO_EVT_TX_COMMAND_SENT, summary, NULL);
                } else {
                    emit_tx_event_local(&state, SSO_EVT_TX_NOT_SENT, summary, outcome);
                }
                // Audit: the result of every queued TX burst, so post-
                // incident review can see each tx-commit and whether it
                // reached the air (on_air=1 means the burst left the radio).
                {
                    char det[512];
                    snprintf(det, sizeof det,
                             "outcome=\"%.80s\" on_air=%d summary=\"%.300s\"",
                             outcome ? outcome : "?", on_air, summary);
                    sso_audit_event("tx-result", det);
                }
                state.tx_request.pending = 0;
            }
        }
#endif

        // --- IPC: serve clients, fan out state, honour SIGUSR1 yield ---
        // Always service the socket (cheap; accepts new viewers) but
        // throttle STATE broadcasts to 2 Hz so viewers don't get
        // hammered when the loop is running at UHD-chunk cadence.
        if (state.ipc) {
            sso_ipc_server_step(state.ipc, 0);
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
            if (state.cmd.active && state.cmd.dirty
                && (cmd_now_ns() - state.cmd.last_edit_ns) >= g_cmd_debounce_ns) {
                cmd_broadcast_preview(&state);
                state.cmd.dirty = 0;
            }
        }
        if (tui_yield_requested()) {
            sso_audit_event("yield-requested",
                            "SIGUSR1 (--force takeover) — exiting");
            state.running = 0;
        }

        // Surface a finished spectrum render so the operator sees the
        // outcome (PNG path or ffmpeg error) in the command-line status.
        // The reap only joins the worker thread; status_msg is left
        // alone, so reading it after reap is safe.
        if (state.spec_job.active && state.spec_job.done) {
            if (state.spec_job.status_msg[0]) {
                cmd_set_status(&state, "%s", state.spec_job.status_msg);
            }
            spectrum_job_reap(&state);
        }

        if (state.running) {
            // The B210 worker thread pumps UHD on its own pthread now,
            // so the main loop doesn't pace itself off the radio. Sleep
            // at the historical 2 Hz so rotator-STATUS polls don't ramp
            // up unexpectedly; redraw/IPC gates do their own throttling.
            // Exception: while the operator is typing in the ":" prompt,
            // drop to 20 ms so getch() echoes each keystroke promptly
            // (the 500 ms tick was capping input at ~2 chars/sec).
            usleep((state.cmd.active || state.tx_compose_active || state.auto_tcmd_active)
                   ? 20000 : UPDATE_INTERVAL_MICROSEC);
        }
    }

    endwin();
    tui_release_stderr();
    if (state.have_tr_switch) {
        tr_switch_disconnect(&state.tr_switch);
        state.have_tr_switch = 0;
    }
    // Join the rotator worker before closing the serial FD — otherwise a
    // mid-read in the worker would see EBADF and corrupt the snapshot.
    if (state.rot_async != NULL) {
        antenna_rotator_async_close(state.rot_async);
        state.rot_async = NULL;
    }
    if (state.have_antenna_rotator) {
        antenna_rotator_disconnect(&state.antenna_rotator);
        state.have_antenna_rotator = 0;
    }
    // Free any plan that survived (mid-pass exit / crash on a key
    // before the LOS branch had a chance to clear it).
    main_pursuit_clear_plan(&state);
    if (state.ipc) {
        sso_ipc_server_close(state.ipc);
        state.ipc = NULL;
    }
    // Politely terminate the live raylib waterfall if we spawned one.
    // 5 s timeout via WNOHANG polling so the operator doesn't wait on
    // a hung viewer at shutdown.
    if (g_live_waterfall_pid > 0) {
        kill(g_live_waterfall_pid, SIGTERM);
        for (int t = 0; t < 50; ++t) {
            int status;
            pid_t r = waitpid(g_live_waterfall_pid, &status, WNOHANG);
            if (r == g_live_waterfall_pid) {
                g_live_waterfall_pid = -1;
                break;
            }
            usleep(100000);
        }
        if (g_live_waterfall_pid > 0) {
            kill(g_live_waterfall_pid, SIGKILL);
            waitpid(g_live_waterfall_pid, NULL, 0);
            g_live_waterfall_pid = -1;
        }
        if (g_live_waterfall_stdin_fd >= 0) {
            close(g_live_waterfall_stdin_fd);
            g_live_waterfall_stdin_fd = -1;
        }
    }
#ifdef SSO_WITH_SDR
    char final_wav_path[512] = "";
    char final_iq_path[512]  = "";
    int  final_iq_rate       = 0;
    if (state.rx_session) {
        // Snapshot both sidecar paths before close so the full-pass
        // renderers can find the closed files on disk. Both paths
        // persist across wav_stop in rx_session.
        rx_session_wav_snapshot(state.rx_session,
                                final_wav_path, sizeof final_wav_path,
                                NULL, NULL, NULL);
        rx_session_iq_snapshot(state.rx_session,
                               final_iq_path, sizeof final_iq_path,
                               NULL, &final_iq_rate);
        // The worker owns the WAV writer and the B210 core. Closing
        // the session signals the worker to stop, joins it, then
        // tears down both. Any open WAV / .iq gets its header patched
        // (WAV) or its trailer flushed (IQ).
        rx_session_request_wav_stop(state.rx_session);
        rx_session_close(state.rx_session);
        state.rx_session = NULL;
    }

    // Any in-flight `:spectrum N` worker is touching the same WAV / IQ
    // — let it finish before we hand the file to the full-pass render.
    if (state.spec_job.active) {
        pthread_join(state.spec_job.thr, NULL);
        state.spec_job.active = 0;
        if (state.spec_job.status_msg[0]) {
            fprintf(stderr, "simple_sat_ops: %s\n", state.spec_job.status_msg);
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

    // Final line: tell the operator whether anything landed in the
    // redirected stderr log during the pass.
    tui_report_errors();
    return 0;
}

// --- apply_args ---------------------------------------------------

// One self-contained block per option, each testing "... || help" so
// that in help mode (help != HELP_OFF) every block prints its one-line
// help and falls through to the next. In parse mode only the matching
// block runs its body and writes its result straight into *state (there
// is no separate config struct; apply_args has always filled state).
// See src/cli/argparse.h for the convention.
//
// Option column width: widest label below is
// "--ignore-at-your-peril-all-tc-errors" (36) + a small margin.
#define OPTW 38

static int apply_args(state_t *state, int argc, char **argv, double jul_utc, int help)
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
    char *positional = NULL;

    if (!help) {
        state->antenna_rotator.tracking_prep_time_minutes = TRACKING_PREP_TIME_MINUTES;
        state->satellite_tracking = 0;

        state->nominal_uplink_frequency_hz = UPLINK_FREQ_MHZ * 1e6;
        state->nominal_downlink_frequency_hz = DOWNLINK_FREQ_MHZ * 1e6;
        state->doppler_uplink_frequency_hz = state->nominal_uplink_frequency_hz;
        state->doppler_downlink_frequency_hz = state->nominal_downlink_frequency_hz;
        state->doppler_correction_enabled = 1;
        // SIGNED LO offset from the nominal carrier. Positive → LO ABOVE
        // nominal (signal lands at negative baseband). Negative → LO
        // BELOW nominal (signal at positive baseband). Default -25 kHz
        // puts the corrected signal at +25 kHz baseband, away from the
        // B210's DC null. Operator can shift it to dodge fixed-pattern
        // spurs — comfortable range is roughly ±5..±35 kHz: at least
        // 5 kHz to clear DC, and at most ~35 kHz so the ±10 kHz Doppler
        // swing stays inside the 48 kHz post-decim half-band.
        state->rx_lo_offset_hz = -25000.0;
        state->rx_gain_db      = 30.0;
        // AD9361 background tracking. The visible ~51 Hz comb of impulsive
        // spikes at mid-range gain is from the IQ-balance loop (discrete
        // phase-rotation steps applied to the captured IQ); the DC-offset
        // loop is a slow continuous IIR notch that DOESN'T produce
        // spikes but DOES suppress the AD9361's static ADC DC bias.
        // Turn IQ tracking off by default (kills the spikes), leave DC
        // tracking on (otherwise the static DC bias rotates into a strong
        // +lo_offset_hz sinusoid via fm_lo_nco on the decode path, which
        // dominates the IQ time series).
        state->rx_dc_offset_track  = 1;
        state->rx_iq_balance_track = 0;

        state->run_with_antenna_rotator = 1;
        state->antenna_rotator.device_filename = "/dev/ttyUSB0";
        state->antenna_rotator.serial_speed = B600;
        state->antenna_rotator.fixed_target = 0;

        // T/R antenna switch: auto-probe /dev/ttyACM0. Failure is a
        // one-line warning, not an error.
        state->run_with_tr_switch = 1;
        state->have_tr_switch     = 0;
        state->tr_switch.device_filename = "/dev/ttyACM0";
        state->tr_switch.serial_speed    = B115200;
    }

    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // Positional first so <satellite_id> lists above the options.
        // A token that is not "--"-prefixed counts as the positional.
        // The actual pointer is resolved by the discovery scan AFTER the
        // loop (which re-walks argv exactly as the pre-conversion code
        // did, including its quirk that a space-form option value can be
        // grabbed as the positional); here we only print the help line
        // and mark the token matched so a bare extra positional falls
        // through to the post-loop n_positional > 1 check rather than the
        // unknown-token branch.
        if (strncmp("--", arg, 2) != 0 || help) {
            if (help) parse_help_line(OPTW, "<satellite_id>",
                "name prefix in the TLE, or `next` to auto-pick the next pass");
            matched = 1;
        }
        if (strcmp("--help", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "short help (this message)");
            else { apply_args(state, argc, argv, jul_utc, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp("--help-full", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--help-full", "detailed help with keyboard layout");
            else { apply_args(state, argc, argv, jul_utc, HELP_FULL); return PARSE_HELP; }
            matched = 1;
        }
        if (strncmp("--verbose=", arg, 10) == 0 || help) {
            if (help) parse_help_line(OPTW, "--verbose=<level>", "verbosity integer");
            else {
                state->n_options++;
                if (strlen(arg) < 11) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->verbose_level = atoi(arg + 10);
            }
            matched = 1;
        }
        if (strcmp("--with-rotator", arg) == 0
                || strcmp("--with-hardware", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--with-rotator",
                "drive the rotator (default; --with-hardware synonym, no-op)");
            else {
                // Rotator is on by default now. These flags survive as
                // silent no-ops so existing scripts and muscle memory
                // keep working.
                state->n_options++;
                state->run_with_antenna_rotator = 1;
            }
            matched = 1;
        }
        if (strcmp("--without-rotator", arg) == 0
                || strcmp("--without-hardware", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-rotator",
                "skip the SPID Rot2Prog (--without-hardware synonym)");
            else {
                state->n_options++;
                state->run_with_antenna_rotator = 0;
            }
            matched = 1;
        }
        if (strcmp("--calibrate-rotator", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--calibrate-rotator",
                "measure rotator slew rates then exit (needs --confirm-rotator-calibrate)");
            else { state->n_options++; state->calibrate_rotator = 1; }
            matched = 1;
        }
        if (strcmp("--confirm-rotator-calibrate", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--confirm-rotator-calibrate",
                "safety interlock for --calibrate-rotator (antenna moves)");
            else { state->n_options++; state->confirm_rotator_calibrate = 1; }
            matched = 1;
        }
        if (strcmp("--without-rotator-pursuit", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-rotator-pursuit",
                "disable the pursuit / lead-aim planner even if calibrated");
            else { state->n_options++; state->without_rotator_pursuit = 1; }
            matched = 1;
        }
        if (strcmp("--without-tr-switch", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-tr-switch",
                "skip the T/R switch probe entirely");
            else { state->n_options++; state->run_with_tr_switch = 0; }
            matched = 1;
        }
        if (strncmp("--tr-switch-device=", arg, 19) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tr-switch-device=<path>",
                "UHF T/R antenna switch tty (default /dev/ttyACM0)");
            else {
                state->n_options++;
                if (strlen(arg) < 20) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->tr_switch.device_filename = arg + 19;
            }
            matched = 1;
        }
        if (strcmp("--without-b210", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-b210",
                "skip the USRP B210 (UI + rotator only)");
            else { state->n_options++; state->without_b210 = 1; }
            matched = 1;
        }
#ifdef SSO_WITH_SDR
        if (strncmp("--sdr-type=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-type=uhd|rtlsdr|auto",
                "SDR backend (default auto; RTL-SDR is RX-only)");
            else {
                state->n_options++;
                if (sdr_backend_type_from_string(arg + 11, &state->sdr_type) != 0) {
                    fprintf(stderr, "--sdr-type: unknown '%s' "
                            "(want uhd | rtlsdr | auto)\n", arg + 11);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--sdr-device=", arg, 13) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-device=<sel>",
                "backend device selector (RTL-SDR index; UHD use --uhd-args)");
            else {
                state->n_options++;
                snprintf(state->sdr_device, sizeof state->sdr_device, "%s", arg + 13);
            }
            matched = 1;
        }
        if (strncmp("--uhd-args=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--uhd-args=<args>",
                "UHD device-args verbatim; overrides detection");
            else {
                state->n_options++;
                snprintf(state->uhd_args, sizeof state->uhd_args, "%s", arg + 11);
            }
            matched = 1;
        }
        if (strncmp("--sdr-fpga=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-fpga=<path>",
                "force a UHD FPGA image (B2xx clone with non-stock bitstream)");
            else {
                state->n_options++;
                snprintf(state->sdr_fpga, sizeof state->sdr_fpga, "%s", arg + 11);
            }
            matched = 1;
        }
#endif
        if (strcmp("--no-tx", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-tx",
                "open the B210 for RX but block the TX compose modal from keying the PA");
            else { state->n_options++; state->no_tx = 1; }
            matched = 1;
        }
        if (strcmp("--live-waterfall", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--live-waterfall",
                "auto-launch the raylib live_waterfall viewer when recording starts");
            else { state->n_options++; state->run_live_waterfall = 1; }
            matched = 1;
        }
        if (strcmp("--always-record", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--always-record",
                "record from B210 open until shutdown (skip per-pass start/stop)");
            else { state->n_options++; state->always_record = 1; }
            matched = 1;
        }
        if (strcmp("--testing", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--testing",
                "bench mode: pass folder under Testing/ at current local time, no TLE");
            else { state->n_options++; state->testing_mode = 1; }
            matched = 1;
        }
        if (strcmp("--scan-sky", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--scan-sky",
                "rebind T to walk the rotator through a sky grid, dwelling 5 s each");
            else { state->n_options++; state->scan.mode = 1; }
            matched = 1;
        }
        if (strncmp("--scan-step=", arg, 12) == 0 || help) {
            if (help) parse_help_line(OPTW, "--scan-step=<deg>",
                "elevation ring spacing for --scan-sky (default 15, clamped [1,45])");
            else {
                state->n_options++;
                state->scan.step_deg = atof(arg + 12);
                if (state->scan.step_deg < 1.0)  state->scan.step_deg = 1.0;
                if (state->scan.step_deg > 45.0) state->scan.step_deg = 45.0;
            }
            matched = 1;
        }
        if (strcmp("--tx-dry-run", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tx-dry-run",
                "record every TX burst as not-sent instead of routing it through the SDR");
            else { state->n_options++; state->tx_dry_run = 1; }
            matched = 1;
        }
        if (strncmp("--tx-preroll-ms=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tx-preroll-ms=<n>",
                "modulated 0xAA carrier before each TX burst (default 200, [0,5000])");
            else {
                state->n_options++;
                int v = atoi(arg + 16);
                if (v < 0)    v = 0;
                if (v > 5000) v = 5000;
                state->tx_preroll_ms = v;
            }
            matched = 1;
        }
        // Filename args use the space form (--foo <path>) so bash
        // tab-completion works. The old --foo=<path> form is rejected
        // with a one-line hint pointing at the new spelling.
        if (strcmp("--tc-file", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tc-file <path>",
                "load ASCII telecommands (one per line; 'A' / `:auto` in the UI to send)");
            else {
                // arg is argv[t + 1]; its value is the next token,
                // argv[t + 2]. Consume it and step t past it.
                if (t + 2 >= argc) {
                    fprintf(stderr, "--tc-file: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                snprintf(state->auto_tcmd_file_path, sizeof state->auto_tcmd_file_path,
                         "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--tc-file=", arg, 10) == 0) {
            fprintf(stderr,
                "--tc-file=<path> is no longer accepted; "
                "use `--tc-file <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--ignore-at-your-peril-all-tc-errors", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ignore-at-your-peril-all-tc-errors",
                "start even if the --tc-file agenda has telecommand lint errors");
            else { state->n_options++; state->ignore_tc_errors = 1; }
            matched = 1;
        }
        if (strcmp("--hmac-keyfile", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--hmac-keyfile <path>",
                "HMAC key file shown on the operator banner (default shared, then user)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--hmac-keyfile: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                snprintf(state->hmac_keyfile_path, sizeof state->hmac_keyfile_path,
                         "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--hmac-keyfile=", arg, 15) == 0) {
            fprintf(stderr,
                "--hmac-keyfile=<path> is no longer accepted; "
                "use `--hmac-keyfile <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--tle", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tle <path>",
                "path to a TLE file (default $HOME/.local/state/simple_sat_ops/active.tle)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--tle: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                state->prediction.tles_filename = tle_path_resolve(argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--tle=", arg, 6) == 0) {
            fprintf(stderr,
                "--tle=<path> is no longer accepted; "
                "use `--tle <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--pass-folder", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--pass-folder <path>",
                "pre-seed the pass folder (handoff: inherit a previous operator's folder)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--pass-folder: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                // Pre-seed state->pass_folder; setup_pass_folder() then skips
                // its AOS-driven auto-discovery and uses the inherited
                // path (handoff case: new operator picks up the previous
                // operator's pass folder).
                snprintf(state->pass_folder, sizeof state->pass_folder, "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--pass-folder=", arg, 14) == 0) {
            fprintf(stderr,
                "--pass-folder=<path> is no longer accepted; "
                "use `--pass-folder <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--rotator-device", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-device <path>",
                "SPID Rot2Prog tty (default /dev/ttyUSB0)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--rotator-device: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                state->antenna_rotator.device_filename = argv[t + 2];
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--rotator-device=", arg, 17) == 0) {
            fprintf(stderr,
                "--rotator-device=<path> is no longer accepted; "
                "use `--rotator-device <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strncmp("--uplink-freq-mhz=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--uplink-freq-mhz=<mhz>",
                "uplink nominal carrier, MHz (informational)");
            else {
                state->n_options++;
                if (strlen(arg) < 19) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->nominal_uplink_frequency_hz = atof(arg + 18) * 1e6;
            }
            matched = 1;
        }
        if (strncmp("--downlink-freq-mhz=", arg, 20) == 0 || help) {
            if (help) parse_help_line(OPTW, "--downlink-freq-mhz=<mhz>",
                "downlink / simplex carrier nominal, MHz (informational)");
            else {
                state->n_options++;
                if (strlen(arg) < 21) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->nominal_downlink_frequency_hz = atof(arg + 20) * 1e6;
            }
            matched = 1;
        }
        if (strcmp("--no-doppler-correction", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-doppler-correction",
                "display nominal freqs without Doppler");
            else { state->n_options++; state->doppler_correction_enabled = 0; }
            matched = 1;
        }
        if (strncmp("--lo-offset=", arg, 12) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lo-offset=<kHz>",
                "park the SDR LO this far off the nominal carrier (signed, default -25)");
            else {
                state->n_options++;
                // Argument is kHz so an integer is easy to type; we store Hz.
                state->rx_lo_offset_hz = atof(arg + 12) * 1000.0;
            }
            matched = 1;
        }
        if (strncmp("--rx-gain=", arg, 10) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rx-gain=<dB>",
                "AD9361 RX gain at session open, dB (default 30, range [0,76])");
            else {
                state->n_options++;
                double g = atof(arg + 10);
                // AD9361 RX gain range is 0-76 dB; UHD coerces values outside
                // this and prints a warning, but we clip here so the value in
                // state matches what the hardware will use.
                if (g < 0.0)       g = 0.0;
                else if (g > 76.0) g = 76.0;
                state->rx_gain_db = g;
            }
            matched = 1;
        }
        if (strncmp("--ad9361-dc-track=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ad9361-dc-track=on|off",
                "AD9361 background DC-offset tracking (default on)");
            else {
                // on|off|true|false|1|0
                const char *v = arg + 18;
                state->n_options++;
                state->rx_dc_offset_track =
                    (strcmp(v, "on")   == 0
                     || strcmp(v, "true") == 0
                     || strcmp(v, "1")  == 0) ? 1 : 0;
            }
            matched = 1;
        }
        if (strncmp("--ad9361-iq-track=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ad9361-iq-track=on|off",
                "AD9361 background IQ-balance tracking (default off; ~51 Hz spike comb)");
            else {
                const char *v = arg + 18;
                state->n_options++;
                state->rx_iq_balance_track =
                    (strcmp(v, "on")   == 0
                     || strcmp(v, "true") == 0
                     || strcmp(v, "1")  == 0) ? 1 : 0;
            }
            matched = 1;
        }
        if (strncmp("--rotator-target-elevation=", arg, 27) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-target-elevation=<deg>",
                "park on a fixed elevation");
            else {
                state->n_options++;
                if (strlen(arg) < 28) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->antenna_rotator.target_elevation = atof(arg + 27);
                if (state->antenna_rotator.target_elevation < 0.0) {
                    state->antenna_rotator.target_elevation = 0.0;
                } else if (state->antenna_rotator.target_elevation
                           > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                    state->antenna_rotator.target_elevation =
                        ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                }
                state->antenna_rotator.fixed_target = 1;
            }
            matched = 1;
        }
        if (strncmp("--rotator-target-azimuth=", arg, 25) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-target-azimuth=<deg>",
                "park on a fixed azimuth");
            else {
                state->n_options++;
                if (strlen(arg) < 26) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                double az = atof(arg + 25);
                if (az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH) {
                    az = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
                } else if (az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                    az = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
                }
                state->antenna_rotator.target_azimuth = az;
                state->antenna_rotator.target_azimuth_unwrapped = az;
                state->antenna_rotator.unwrapped_target_valid = 1;
                state->antenna_rotator.fixed_target = 1;
            }
            matched = 1;
        }
        if (strncmp("--lat=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lat=<deg>", "geodetic latitude (default RAO Priddis)");
            else {
                state->n_options++;
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                site_latitude = atof(arg + 6);
            }
            matched = 1;
        }
        if (strncmp("--lon=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lon=<deg>", "geodetic longitude, east positive");
            else {
                state->n_options++;
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                site_longitude = atof(arg + 6);
            }
            matched = 1;
        }
        if (strncmp("--alt=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--alt=<m>", "altitude above ellipsoid, metres");
            else {
                state->n_options++;
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                site_altitude = atof(arg + 6);
            }
            matched = 1;
        }
        if (strcmp("--include-constellations", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--include-constellations",
                "include Starlink/OneWeb-style swarms in the `next` pass filter");
            else { state->n_options++; with_constellations = 1; }
            matched = 1;
        }
        if (strncmp("--min-altitude-km=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-altitude-km=<km>",
                "minimum orbital altitude (default 0)");
            else {
                state->n_options++;
                if (strlen(arg) < 19) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                min_altitude_km = atof(arg + 18);
            }
            matched = 1;
        }
        if (strncmp("--max-altitude-km=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-altitude-km=<km>",
                "maximum orbital altitude (default 1000)");
            else {
                state->n_options++;
                if (strlen(arg) < 19) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                max_altitude_km = atof(arg + 18);
            }
            matched = 1;
        }
        if (strncmp("--min-elevation=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-elevation=<deg>",
                "minimum peak elevation (default 0)");
            else {
                state->n_options++;
                if (strlen(arg) < 17) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                min_elevation = atof(arg + 16);
            }
            matched = 1;
        }
        if (strncmp("--min-minutes=", arg, 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-minutes=<n>",
                "minimum minutes until AOS (default 1)");
            else {
                state->n_options++;
                if (strlen(arg) < 15) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                min_minutes_away = atof(arg + 14);
            }
            matched = 1;
        }
        if (strncmp("--max-minutes=", arg, 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-minutes=<n>",
                "maximum minutes until AOS (default 90)");
            else {
                state->n_options++;
                if (strlen(arg) < 15) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                max_minutes_away = atof(arg + 14);
            }
            matched = 1;
        }
        if (strcmp("--control", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--control",
                "open the sso_ipc server (operator mode)");
            else { state->n_options++; state->control_mode = 1; }
            matched = 1;
        }
        if (strcmp("--self-test", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--self-test",
                "print the settings simple_sat_ops would run with, then exit 0");
            else { state->n_options++; state->self_test = 1; }
            matched = 1;
        }

        // Unknown token. Only "--"-prefixed tokens are an error here
        // (the old code's final `strncmp("--", ...) == 0` branch);
        // a bare extra positional falls through and is caught by the
        // n_positional > 1 check after the loop.
        if (!matched && !help) {
            if (strncmp("--", arg, 2) == 0) {
                fprintf(stderr, "Unable to parse option '%s'\n", arg);
                return PARSE_ERROR;
            }
        }
    }

    // Full-help epilog: the keyboard layout + examples, printed once
    // after all the option lines (only for --help-full).
    if (help >= HELP_FULL) {
        printf(
            "\n"
            "KEYBOARD (unlocked by default, press K to toggle lock state)\n"
            "\n"
            "  K         Toggle keyboard lock\n"
            "  T         Start tracking the current satellite\n"
            "  s         Stop tracking\n"
            "  r         Reset rotator to az=0, el=0\n"
            "  [ / ]     Nudge antenna azimuth -5 / +5 deg\n"
            "  { / }     Nudge antenna azimuth -1 / +1 deg (fine)\n"
            "  , / .     Nudge antenna elevation -5 / +5 deg\n"
            "  < / >     Nudge antenna elevation -1 / +1 deg (fine)\n"
            "  q         Quit\n"
            "\n"
            "EXAMPLES\n"
            "\n"
            "  # Auto-pick next visible pass above 10 deg (rotator on by default)\n"
            "  simple_sat_ops next --min-elevation=10 --min-minutes=10 --max-minutes=45\n"
            "\n"
            "  # Dry-run prediction on a dev host (no rotator hardware)\n"
            "  simple_sat_ops 'ISS (ZARYA)' --without-rotator\n"
            "\n"
            "  # Operator coordination (broadcasts state to viewers over sso_ipc)\n"
            "  simple_sat_ops next --control\n");
    }
    if (help) return PARSE_OK;

    int n_positional = argc - state->n_options - 1;  // -1 for argv[0]
    if (n_positional > 1) {
        fprintf(stderr,
            "simple_sat_ops: too many positional arguments "
            "(expected at most one <satellite_id>)\n");
        return PARSE_ERROR;
    }

    // Find the (single) positional, if any. Existing convention is
    // "positional at argv[1]" but loop is robust to options-before /
    // options-after orderings.
    for (int i = 1; i < argc; i++) {
        if (strncmp("--", argv[i], 2) == 0) continue;
        positional = argv[i];
        break;
    }

    // Any invocation without --control: the standalone tracker is being
    // phased out in favour of the operator+viewer split, so there is no
    // longer a "track this on my own" path. Probe for the running
    // operator and either attach as a viewer or bail with a hint.
    // This holds whether or not a satellite name was given on the
    // command line - a viewer mirrors whatever the operator is tracking,
    // so any positional is ignored here. --self-test skips the probe (a
    // side effect) so the config dump runs cleanly with no live operator.
    if (!state->control_mode && !state->self_test) {
        sso_ipc_client_t *probe = sso_ipc_client_connect("simple_sat_ops");
        if (probe == NULL) {
            fprintf(stderr,
                "operator not found: try `simple_sat_ops --control` "
                "to operate FrontierSat\n");
            return PARSE_ERROR;
        }
        sso_ipc_client_close(probe);
        // Operator is up — main() will dispatch into run_viewer()
        // instead of the standalone-tracker path.
        state->viewer_mode = 1;
        return PARSE_OK;
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
    if (n_positional == 0 && state->control_mode) {
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
                return PARSE_ERROR;
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
            return PARSE_ERROR;
        }
        state->prediction.satellite_ephem.name = sat_name;
        fprintf(stderr, "simple_sat_ops: tracking '%s'\n", sat_name);
    } else {
        if (state->prediction.tles_filename == NULL) {
            static char default_tle[1024];
            if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
                fprintf(stderr,
                    "HOME unset or path too long; pass --tle=<path>\n");
                return PARSE_ERROR;
            }
            state->prediction.tles_filename = tle_path_resolve(default_tle);
        }
        state->prediction.satellite_ephem.name = positional;
    }

    // --self-test exits before TLE/pass-search anyway, and the bare
    // form (no positional, no --control) leaves .name == NULL; skip
    // the auto-pass search rather than feeding NULL to strcmp.
    if (state->prediction.satellite_ephem.name != NULL
        && strcmp(state->prediction.satellite_ephem.name, "next") == 0) {
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
            return PARSE_ERROR;
        }

        const pass_t *p = get_pass(0);
        state->prediction.satellite_ephem.name = strdup(p->name);
        printf("Satellite: %s\n", state->prediction.satellite_ephem.name);
    }

    return PARSE_OK;
}

// --- Tracking helpers ---------------------------------------------

