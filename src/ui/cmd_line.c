/*

   Simple Satellite Operations  ui/cmd_line.c

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

#include "cmd_line.h"
#include "state.h"

#include "auto_tcmd.h"
#include "live_waterfall.h"
#include "spectrogram.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "tracking.h"
#include "tx_compose.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <ncurses.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef SSO_WITH_SDR
#include "rx_session.h"
#endif

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

void cmd_enter(state_t *state)
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
                       "freq <MHz> lo_offset <signed_kHz> lo_bandwidth <kHz> "
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
                           "(comfort range +/-5..+/-40)");
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
        } else if (live_waterfall_stdin_fd() < 0) {
            cmd_set_status(state, "lo_bandwidth: no live viewer running "
                           "(launch with --live-waterfall)");
        } else {
            double n = atof(arg1);
            if (n <= 0.0 || n > 1000.0) {
                cmd_set_status(state, "lo_bandwidth: %g out of (0, 1000] kHz", n);
            } else {
                char line[64];
                int  ln = snprintf(line, sizeof line, "bandwidth %g\n", n);
                ssize_t w = (ln > 0) ? write(live_waterfall_stdin_fd(),
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
void cmd_broadcast_preview(state_t *state)
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
int cmd_handle_key(int key, state_t *state)
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
void cmd_render(state_t *state)
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

// Per-tick: broadcast the command-line preview to viewers once the operator
// has paused typing for g_cmd_debounce_ns (so it's not a packet per key).
void cmd_pump(state_t *state)
{
    if (state->cmd.active && state->cmd.dirty
        && (cmd_now_ns() - state->cmd.last_edit_ns) >= g_cmd_debounce_ns) {
        cmd_broadcast_preview(state);
        state->cmd.dirty = 0;
    }
}
