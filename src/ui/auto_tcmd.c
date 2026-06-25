/*

   Simple Satellite Operations  ui/auto_tcmd.c

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

#include "auto_tcmd.h"
#include "state.h"

#include "panels.h"          // draw_box, tx_drain_csi
#include "agenda_line.h"
#include "frontiersat.h"
#include "sso_audit.h"
#include "sso_pseudo.h"
#include "sso_time.h"
#include "tcmd_lint.h"      // TCMD_RF_MAX_LEN, tcmd_lint_file
#include "ui_textfield.h"

#include <ctype.h>
#include <math.h>
#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- Auto-TCMD modal ----------------------------------------------
//
// Drives a file of ASCII telecommands through the TX path automatically.
// Loaded via --tc-file=<path>; opened with 'A' (or `:auto`) from the
// operator UI. Each non-blank, non-comment line in the file is one
// CTS1+ telecommand. The operator picks TX power, how many times each
// command should be sent, and the inter-send interval. Once started the
// modal's tick runs alongside the main loop — non-blocking, like the
// TX compose modal — and queues one state->tx.tx_request per shot, advancing
// through the file. Stops automatically when the satellite drops
// below the horizon (LOS) so an unattended run can't keep TXing after
// the pass. Every send goes through emit_tx_event_local, so the
// existing tx.log + viewer fanout capture all of them.
// auto_tcmd_field_t, auto_tcmd_state_t and auto_tcmd_t now live in
// state.h. The live modal state (active flag, window, the auto_tcmd_t
// run state, and the --tc-file path) are fields on state_t.

// Minimum spacing between the START of consecutive sends, in seconds. A
// half-duplex burst already occupies ~1.3+ s end to end (RX pause + UHD
// start lead + on-air frame + RX resume; see tx_burst.c), so a typed
// interval below this floor would never be honoured anyway. Clamping at
// run start, and saying so in the status line, keeps the readout honest
// rather than silently ignoring a too-small number. The operator can read
// the real measured per-burst time off the modal (last_burst_wall_s).
#define AUTO_TCMD_MIN_INTERVAL_S 1.0

// Wall-clock seconds one auto-tcmd send occupies, end to end. Mirrors
// the framing and the fixed timing in tx_burst.c's build_iq / tx_burst_run:
//
//   frame_bytes = prefill(32) + ASM(4) + Golay(3)
//                 + csp_hdr(4) + payload + hmac(4) + rs_parity(32)
//                 + tailfill(1)
//               = 80 + payload
//
//   burst_s     = start_delay(0.5)            // UHD timed-start lead
//                 + state->tx.tx_preroll_ms/1000       // modulated 0xAA carrier
//                 + frame_bytes * 8 / bit_rate // the frame itself
//                 + postroll(0.050)
//
// The start lead matters: tx_burst_run schedules the burst 0.5 s ahead
// and blocks until it completes, so each send is inhibited for the whole
// span -- leaving it out is most of the per-burst underestimate. auto-tcmd
// sends one burst per shot (repeat=1); the repeat count and inter-send
// interval are folded in by the caller, so this stays a per-send quantum.
static double auto_tcmd_burst_seconds(state_t *state, size_t payload_len) {
    const double start_delay_s = 0.500;   // tx_burst.c start_delay_s
    const double preroll_s     = (double) state->tx.tx_preroll_ms * 1e-3;
    const double postroll_s    = 0.050;   // tx_burst.c postroll_ms
    const double bit_rate      = 9600.0;
    size_t frame_bytes = 80 + payload_len;
    return start_delay_s + preroll_s
         + ((double)(frame_bytes * 8) / bit_rate) + postroll_s;
}

// "Xm Ys" formatter for the Progress line. Caller's buffer needs ~16
// chars to be safe across reasonable durations.
static void fmt_minsec(double seconds, char *out, size_t cap) {
    if (seconds < 0.0) seconds = 0.0;
    long total = (long)(seconds + 0.5);
    long m = total / 60;
    long s = total % 60;
    snprintf(out, cap, "%ldm %lds", m, s);
}

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

// Truncate an inline trailing comment from a command line in place. The
// rule (a '#' is a comment only when preceded by whitespace) is shared
// with agenda_check via agenda_find_inline_comment(); a '#' that is part
// of the command text is left intact -- a wrong telecommand is worse than
// an unstripped one. Whole-line comments (a leading '#') are handled by
// the caller before this is reached.
static void strip_inline_comment(char *s) {
    size_t cmd_len;
    agenda_find_inline_comment(s, &cmd_len);
    s[cmd_len] = '\0';
}

// Read commands from path; one per line. Whole-line comments (#...) and
// blank lines after trim are dropped, and an inline trailing comment
// (whitespace + #...) is stripped from each command. Returns 0 on
// success; allocates and stores in *out_commands / *out_n on success.
// Caller owns the allocation.
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
        strip_inline_comment(t);
        if (t[0] == '\0') continue;
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

// Record an auto-tcmd lifecycle event (start / pause / resume / abort /
// restart) in two places, because interrupting a run breaks the pass
// operations plan and the operator needs an after-the-fact trail:
//   1. /var/log/sso/runs.log via the shared audit log (sso_audit_event), and
//   2. <pass_folder>/session.log, a plain timestamped line kept alongside
//      tx.log in the pass's operation folder.
// The session-log write is a no-op until the pass folder exists.
static void auto_tcmd_log(state_t *state, const char *event, const char *detail) {
    sso_audit_event(event, detail);
    if (state->op.pass_folder[0] == '\0') return;
    char path[512];
    snprintf(path, sizeof path, "%.500s/session.log", state->op.pass_folder);
    FILE *fp = fopen(path, "a");
    if (!fp) return;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    char iso[40];
    sso_iso_utc_from_ts(&ts, iso, sizeof iso);
    fprintf(fp, "%s %s %s\n", iso, event, detail ? detail : "");
    fclose(fp);
}


// --- Auto-TCMD modal helpers --------------------------------------

static const char *auto_tcmd_state_label(auto_tcmd_state_t s) {
    switch (s) {
        case AUTO_STATE_SETUP:        return "idle";
        case AUTO_STATE_RUNNING:      return "running";
        case AUTO_STATE_STOPPED:      return "stopped";
        case AUTO_STATE_DONE:         return "done";
        case AUTO_STATE_PASS_OVER:    return "pass-over";
        case AUTO_STATE_PAUSE_PROMPT: return "interrupt?";
        case AUTO_STATE_PAUSED:       return "paused";
        case AUTO_STATE_RESUME_PROMPT:return "resume?";
    }
    return "?";
}

// Snapshot of the auto-tcmd run for the viewer broadcast: sends queued
// so far vs. the run's planned total (commands × repeats), plus the
// run-state label. Returns 1 when there is a run to report — the modal
// is open and Enter has started it (running, or finished and still on
// screen so viewers see the final tally). Returns 0 in setup or when
// the modal is closed, which drops the fields from the wire entirely.
int auto_tcmd_progress(tx_t *tx, int *sent, int *total, const char **label) {
    const auto_tcmd_t *a = &tx->auto_tcmd;
    if (!tx->auto_tcmd_active || a->state == AUTO_STATE_SETUP) return 0;
    *sent  = a->sends_total;
    *total = a->n_commands * a->repeats_total;
    *label = auto_tcmd_state_label(a->state);
    return 1;
}

int auto_field_is_text(auto_tcmd_field_t f) {
    return f == AUTO_F_POWER || f == AUTO_F_REPEATS || f == AUTO_F_INTERVAL;
}
static int auto_field_is_toggle(auto_tcmd_field_t f) {
    return f == AUTO_F_ALLOW_TX;
}

static char *auto_field_buf(auto_tcmd_t *a, auto_tcmd_field_t f, size_t *cap) {
    switch (f) {
        case AUTO_F_POWER:    *cap = sizeof a->power;      return a->power;
        case AUTO_F_REPEATS:  *cap = sizeof a->repeats;    return a->repeats;
        case AUTO_F_INTERVAL: *cap = sizeof a->interval_s; return a->interval_s;
        default:              *cap = 0; return NULL;
    }
}

static int auto_field_char_ok(auto_tcmd_field_t f, int ch) {
    if (f == AUTO_F_POWER || f == AUTO_F_INTERVAL) {
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
    ui_tf_clamp_cursor(buf, &a->cursors[f]);
}

static void auto_field_insert(auto_tcmd_t *a, int ch) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    if (!auto_field_char_ok(a->focus, ch)) return;
    ui_tf_insert(buf, cap, &a->cursors[a->focus], ch);
}
static void auto_field_backspace(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    ui_tf_backspace(buf, &a->cursors[a->focus]);
}
static void auto_field_delete(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    ui_tf_delete(buf, &a->cursors[a->focus]);
}
static void auto_field_kill_to_end(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    ui_tf_kill_to_end(buf, &a->cursors[a->focus]);
}
static void auto_field_left(auto_tcmd_t *a) {
    size_t cap = 0;
    if (!auto_field_buf(a, a->focus, &cap)) return;
    ui_tf_left(&a->cursors[a->focus]);
}
static void auto_field_right(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    ui_tf_right(buf, &a->cursors[a->focus]);
}
static void auto_field_home(auto_tcmd_t *a) {
    size_t cap = 0;
    if (!auto_field_buf(a, a->focus, &cap)) return;
    ui_tf_home(&a->cursors[a->focus]);
}
static void auto_field_end(auto_tcmd_t *a) {
    size_t cap = 0;
    char *buf = auto_field_buf(a, a->focus, &cap);
    if (!buf) return;
    ui_tf_end(buf, &a->cursors[a->focus]);
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

static void auto_tcmd_draw(state_t *state) {
    if (!state->tx.auto_tcmd_active || !state->tx.auto_tcmd_win) return;
    WINDOW *w = state->tx.auto_tcmd_win;
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    werase(w);
    draw_box(w);
    int width = getmaxx(w);

    mvwprintw(w, 0, 2, " Auto-TCMD (operator: %s)%s ",
              state->op.operator_user ? state->op.operator_user : "?",
              state->tx.no_tx ? "  [--no-tx]" : "");

    mvwprintw(w, 1, 2, "File:    %.*s  (%d commands)",
              width - 28, a->file_path[0] ? a->file_path : "(none)",
              a->n_commands);
    wclrtoeol(w);

    // Fields are read-only while the run is active or while a prompt /
    // paused state owns the modal -- only the SETUP form is editable.
    int running_ro = (a->state == AUTO_STATE_RUNNING
                      || a->state == AUTO_STATE_PAUSE_PROMPT
                      || a->state == AUTO_STATE_PAUSED
                      || a->state == AUTO_STATE_RESUME_PROMPT);

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
    mvwprintw(w, 4, 24, "per command (TCM1 Nx, then TCM2 Nx, ...)");
    wclrtoeol(w);

    auto_draw_text_field(w, 5, 2, "Interval ",
                         a->interval_s, 8,
                         !running_ro && a->focus == AUTO_F_INTERVAL,
                         a->cursors[AUTO_F_INTERVAL]);
    // Show the real measured per-burst wall-time next to the interval the
    // operator is setting, so the floor is grounded in hardware rather than
    // guessed. Reads "--" until the first burst of the session completes.
    if (state->tx.last_burst_wall_s >= 0.0) {
        mvwprintw(w, 5, 24, "s min between send starts  (last burst %.2f s)",
                  state->tx.last_burst_wall_s);
    } else {
        mvwprintw(w, 5, 24, "s min between send starts  (last burst --)");
    }
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
        char tx_spent[16], tx_total[16];
        fmt_minsec(a->tx_seconds_spent, tx_spent, sizeof tx_spent);
        fmt_minsec(a->tx_seconds_total, tx_total, sizeof tx_total);
        mvwprintw(w, 10, 2,
                  "Progress: cmd %d/%d   send %d/%d   total sent: %d   "
                  "(elapsed %s / ~%s)",
                  a->cmd_idx + (a->state == AUTO_STATE_RUNNING ? 1 : 0),
                  a->n_commands,
                  a->repeat_idx, rt,
                  a->sends_total,
                  tx_spent, tx_total);
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

    // Outcome of the most recent serviced burst. The modal covers the bottom
    // TX-log panel, so this is the only place the operator sees whether a
    // command reached the air or was rejected (no B210 / dry-run / ...). A
    // NOT-SENT line is drawn bold so a silently-rejecting run can't pass for a
    // transmitting one. See tx_burst_service_request.
    mvwprintw(w, 13, 2, "Last burst:");
    if (state->tx.last_burst_outcome[0] == '\0') {
        mvwprintw(w, 13, 13, "(none yet)");
    } else if (state->tx.last_burst_on_air) {
        // air    = submit -> done (the half-duplex burst itself)
        // held   = staging -> done (what actually gates the next auto-tcmd send)
        // period = this send's staging minus the previous send's ("--" on the
        //          first send of the run). The full hardware split lands in the
        //          tx-timing / tx-result audit lines. See tx_burst.c.
        char pstr[16];
        if (state->tx.last_send_period_s >= 0.0)
            snprintf(pstr, sizeof pstr, "%.2f", state->tx.last_send_period_s);
        else
            snprintf(pstr, sizeof pstr, "--");
        char line[120];
        snprintf(line, sizeof line,
                 "on air - %.30s  air=%.2f held=%.2f period=%s s",
                 state->tx.last_burst_outcome,
                 state->tx.last_burst_wall_s,
                 state->tx.last_burst_slot_s, pstr);
        mvwprintw(w, 13, 13, "%.*s", width - 15, line);
    } else {
        wattron(w, A_BOLD);
        mvwprintw(w, 13, 13, "NOT SENT - %.80s",
                  state->tx.last_burst_outcome);
        wattroff(w, A_BOLD);
    }
    wclrtoeol(w);

    if (a->state == AUTO_STATE_RUNNING) {
        mvwprintw(w, 14, 2,
                  "Running - s stops   Esc interrupts (pause/abort)");
    } else if (a->state == AUTO_STATE_PAUSE_PROMPT) {
        wattron(w, A_BOLD);
        mvwprintw(w, 14, 2,
                  "INTERRUPT:  P pause (resume later)   A abort   "
                  "Esc keep running");
        wattroff(w, A_BOLD);
    } else if (a->state == AUTO_STATE_RESUME_PROMPT) {
        wattron(w, A_BOLD);
        mvwprintw(w, 14, 2,
                  "PAUSED:  R resume from here   S start over   "
                  "Esc keep paused");
        wattroff(w, A_BOLD);
    } else {
        mvwprintw(w, 14, 2,
                  "Tab focus  Space toggle  Enter start  Esc cancel");
    }
    wclrtoeol(w);

    // Park the hardware cursor on the focused text field's cell. The
    // toggle field has no cursor; read-only states (running / prompts /
    // paused) skip cursor placement too.
    if (!running_ro) {
        if (a->focus == AUTO_F_POWER) {
            int cur = a->cursors[AUTO_F_POWER];
            int vis = (cur > 7) ? 7 : cur;
            wmove(w, 3, 2 + 9 + vis);   // "TX power " is 9 chars
        } else if (a->focus == AUTO_F_REPEATS) {
            int cur = a->cursors[AUTO_F_REPEATS];
            int vis = (cur > 5) ? 5 : cur;
            wmove(w, 4, 2 + 9 + vis);   // "Repeats  " is 9 chars
        } else if (a->focus == AUTO_F_INTERVAL) {
            int cur = a->cursors[AUTO_F_INTERVAL];
            int vis = (cur > 7) ? 7 : cur;
            wmove(w, 5, 2 + 9 + vis);   // "Interval " is 9 chars
        }
    }
    wrefresh(w);
}

// Repaint the open modal from the main-loop redraw tick. auto_tcmd_draw is
// otherwise only called on a keystroke or when a send is staged, so a burst
// outcome that resolves in tx_burst_service_request (after the last draw)
// would sit stale on screen. Calling this each redraw keeps the "Last burst"
// line — and the progress numbers — live. No-op unless the modal is open.
void auto_tcmd_refresh(state_t *state) {
    if (state->tx.auto_tcmd_active && state->tx.auto_tcmd_win)
        auto_tcmd_draw(state);
}

// Create the modal window. Returns 1 on success, 0 if newwin failed.
static int auto_tcmd_make_window(state_t *state) {
    int h = 17, ww = 110;
    if (h > LINES) h = LINES;
    if (ww > COLS) ww = COLS;
    if (ww < 60)  ww = (COLS < 60) ? COLS : 60;
    state->tx.auto_tcmd_win = newwin(h, ww, (LINES - h) / 2, (COLS - ww) / 2);
    if (!state->tx.auto_tcmd_win) return 0;
    keypad(state->tx.auto_tcmd_win, TRUE);
    nodelay(state->tx.auto_tcmd_win, TRUE);
    return 1;
}

// (Re)load the --tc-file from scratch and reset the modal fields to a fresh
// SETUP run. Frees any previously loaded commands. Returns 0 on success, -1
// if there is no file path or the load fails.
static int auto_tcmd_reload(state_t *state) {
    if (state->tx.auto_tcmd_file_path[0] == '\0') return -1;
    if (state->tx.auto_tcmd.commands) {
        auto_tcmd_free_commands(state->tx.auto_tcmd.commands, state->tx.auto_tcmd.n_commands);
        state->tx.auto_tcmd.commands = NULL;
        state->tx.auto_tcmd.n_commands = 0;
    }
    char **cmds = NULL;
    int    nc   = 0;
    if (auto_tcmd_load_file(state->tx.auto_tcmd_file_path, &cmds, &nc) != 0) {
        return -1;
    }

    memset(&state->tx.auto_tcmd, 0, sizeof state->tx.auto_tcmd);
    state->tx.auto_tcmd.commands   = cmds;
    state->tx.auto_tcmd.n_commands = nc;

    // Re-lint the freshly (re)loaded file. The startup gate ran once; if the
    // operator edited --tc-file since launch, this is the only check before
    // those commands can be keyed. Lint detail goes to /dev/null (printing to
    // stderr would corrupt the ncurses screen) -- we keep the error count to
    // gate the run in auto_tcmd_start and warn here, honouring the same
    // --ignore-at-your-peril-all-tc-errors opt-out as startup.
    {
        FILE *devnull = fopen("/dev/null", "w");
        if (devnull != NULL) {
            int w = 0;
            int e = tcmd_lint_file(state->tx.auto_tcmd_file_path, devnull, &w);
            state->tx.auto_tcmd.lint_errors = (e > 0) ? e : 0;
            fclose(devnull);
        }
    }
    snprintf(state->tx.auto_tcmd.file_path, sizeof state->tx.auto_tcmd.file_path,
             "%.*s", (int)(sizeof state->tx.auto_tcmd.file_path - 1),
             state->tx.auto_tcmd_file_path);
    snprintf(state->tx.auto_tcmd.power,   sizeof state->tx.auto_tcmd.power,   "80.0");
    snprintf(state->tx.auto_tcmd.repeats, sizeof state->tx.auto_tcmd.repeats, "3");
    snprintf(state->tx.auto_tcmd.interval_s, sizeof state->tx.auto_tcmd.interval_s, "1.0");
    state->tx.auto_tcmd.allow_tx = 0;
    state->tx.auto_tcmd.focus    = AUTO_F_POWER;
    state->tx.auto_tcmd.cursors[AUTO_F_POWER]    = (int) strlen(state->tx.auto_tcmd.power);
    state->tx.auto_tcmd.cursors[AUTO_F_REPEATS]  = (int) strlen(state->tx.auto_tcmd.repeats);
    state->tx.auto_tcmd.cursors[AUTO_F_INTERVAL] = (int) strlen(state->tx.auto_tcmd.interval_s);
    state->tx.auto_tcmd.state    = AUTO_STATE_SETUP;
    if (state->tx.auto_tcmd.lint_errors > 0 && !state->tx.ignore_tc_errors) {
        snprintf(state->tx.auto_tcmd.status_msg, sizeof state->tx.auto_tcmd.status_msg,
                 "loaded %d command(s) but %d lint error(s) -- fix the file; "
                 "run blocked", nc, state->tx.auto_tcmd.lint_errors);
    } else if (state->tx.auto_tcmd.lint_errors > 0) {
        snprintf(state->tx.auto_tcmd.status_msg, sizeof state->tx.auto_tcmd.status_msg,
                 "loaded %d command(s); %d lint error(s) ignored. Enter to start.",
                 nc, state->tx.auto_tcmd.lint_errors);
    } else {
        snprintf(state->tx.auto_tcmd.status_msg, sizeof state->tx.auto_tcmd.status_msg,
                 "loaded %d command(s). Set fields, then Enter to start.",
                 nc);
    }
    return 0;
}

// Open the modal. Refuses if the TX compose modal is already up — at
// most one modal owns the screen at a time. A run parked by a previous
// pause reopens straight into the resume/restart prompt; otherwise the
// --tc-file is (re)loaded fresh.
void auto_tcmd_open(state_t *state) {
    if (!state->op.ipc) return;
    if (state->tx.tx_compose_active) return;
    if (state->tx.auto_tcmd_active) return;

    // A run parked by a previous pause: reopen into the resume/restart
    // prompt rather than reloading. The command list, position, interval
    // and power were all preserved across the modal closing.
    if (state->tx.auto_tcmd.state == AUTO_STATE_PAUSED
        && state->tx.auto_tcmd.commands != NULL) {
        state->tx.auto_tcmd.state = AUTO_STATE_RESUME_PROMPT;
        snprintf(state->tx.auto_tcmd.status_msg, sizeof state->tx.auto_tcmd.status_msg,
                 "paused at cmd %d/%d, send %d/%d -- R resume, S start over, "
                 "Esc keep paused",
                 state->tx.auto_tcmd.cmd_idx + 1, state->tx.auto_tcmd.n_commands,
                 state->tx.auto_tcmd.repeat_idx, state->tx.auto_tcmd.repeats_total);
        if (!auto_tcmd_make_window(state)) return;
        state->tx.auto_tcmd_active = 1;
        auto_tcmd_draw(state);
        return;
    }

    if (state->tx.auto_tcmd_file_path[0] == '\0') return;
    if (auto_tcmd_reload(state) != 0) {
        return;  // silent — operator will notice via the absent modal
    }
    if (!auto_tcmd_make_window(state)) {
        auto_tcmd_free_commands(state->tx.auto_tcmd.commands, state->tx.auto_tcmd.n_commands);
        state->tx.auto_tcmd.commands = NULL;
        state->tx.auto_tcmd.n_commands = 0;
        return;
    }
    state->tx.auto_tcmd_active = 1;
    auto_tcmd_draw(state);
}

void auto_tcmd_close(tx_t *tx) {
    if (tx->auto_tcmd_win) {
        delwin(tx->auto_tcmd_win);
        tx->auto_tcmd_win = NULL;
    }
    tx->auto_tcmd_active = 0;
    // Keep a paused run parked across the close so the operator can compose
    // a one-off TX (or anything else) and then press 'A' to resume it. The
    // command list stays allocated until the run is resumed or restarted.
    if (tx->auto_tcmd.state != AUTO_STATE_PAUSED
        && tx->auto_tcmd.commands) {
        auto_tcmd_free_commands(tx->auto_tcmd.commands, tx->auto_tcmd.n_commands);
        tx->auto_tcmd.commands = NULL;
        tx->auto_tcmd.n_commands = 0;
    }
    touchwin(stdscr);
    refresh();
}

// Validate the setup fields and move to RUNNING. Returns 0 on success,
// fills status_msg + returns -1 on failure.
static int auto_tcmd_start(state_t *state) {
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    if (a->n_commands == 0) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: file has no commands");
        return -1;
    }
    // A file edited after launch can introduce commands the satellite would
    // reject or mis-parse. Re-lint on (re)load flagged them; refuse to run
    // unless the operator started with --ignore-at-your-peril-all-tc-errors.
    if (a->lint_errors > 0 && !state->tx.ignore_tc_errors) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: %d lint error(s) in the file -- fix it and reopen",
                 a->lint_errors);
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
    double interval = atof(a->interval_s);
    if (interval < 0.0) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: interval must be >= 0");
        return -1;
    }
    // A half-duplex burst can't be spaced tighter than its own wall-time;
    // clamp to the floor and tell the operator rather than silently honour a
    // smaller number. Note it for the status line set further down.
    int interval_floored = 0;
    if (interval < AUTO_TCMD_MIN_INTERVAL_S) {
        interval = AUTO_TCMD_MIN_INTERVAL_S;
        interval_floored = 1;
    }
    a->repeats_total  = repeats;
    a->interval_s_val = interval;
    a->cmd_idx       = 0;
    a->repeat_idx    = 0;
    a->sends_total   = 0;
    a->tx_seconds_spent = 0.0;
    // Wall-clock estimate for the whole run. auto_tcmd_tick spaces sends
    // by max(interval, burst): it waits `interval` measured from the start of
    // each send AND for that send's burst to clear, so the interval and the
    // burst overlap rather than add. Every command is sent `repeats`
    // times; only the final send has no trailing interval.
    a->tx_seconds_total = 0.0;
    double last_burst = 0.0;
    for (int i = 0; i < a->n_commands; ++i) {
        double burst = auto_tcmd_burst_seconds(state, strlen(a->commands[i]));
        double slot  = (burst > interval) ? burst : interval;
        a->tx_seconds_total += slot * (double) repeats;
        last_burst = burst;
    }
    if (a->n_commands > 0 && interval > last_burst)
        a->tx_seconds_total -= (interval - last_burst);
    a->start_ns      = ts_now_ns();
    a->next_send_ns  = a->start_ns;  // first send fires immediately
    a->state         = AUTO_STATE_RUNNING;
    if (interval_floored) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "running: %d cmds x %d repeats, interval raised to %.1f s floor",
                 a->n_commands, repeats, interval);
    } else {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "running: %d cmds x %d repeats, %.2f s interval",
                 a->n_commands, repeats, interval);
    }
    {
        char det[256];
        snprintf(det, sizeof det,
                 "n_commands=%d repeats=%d interval_s=%.2f "
                 "allow_tx=%d power=%.100s file=\"%.100s\"",
                 a->n_commands, repeats, interval, a->allow_tx,
                 a->power, a->file_path);
        sso_audit_event("auto-tcmd-start", det);
    }
    return 0;
}

// Pause / cancel without closing the modal so the operator can see the
// final progress numbers.
static void auto_tcmd_stop(state_t *state, const char *reason) {
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    if (a->state != AUTO_STATE_RUNNING) return;
    a->state = AUTO_STATE_STOPPED;
    snprintf(a->status_msg, sizeof a->status_msg, "stopped: %s",
             reason ? reason : "user");
    {
        char det[128];
        snprintf(det, sizeof det,
                 "reason=\"%.100s\" sends_total=%d",
                 reason ? reason : "user",
                 a->sends_total);
        sso_audit_event("auto-tcmd-stop", det);
    }
}

// 'P' from the interrupt prompt: park the run. Position, interval and power
// all stay on state->tx.auto_tcmd (and survive the modal closing), so the
// operator can compose a one-off TX and later resume with 'A'. Logged
// because an interrupted run departs from the pass operations plan.
static void auto_tcmd_pause(state_t *state) {
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    a->pause_ns = ts_now_ns();
    a->state    = AUTO_STATE_PAUSED;
    snprintf(a->status_msg, sizeof a->status_msg,
             "paused at cmd %d/%d -- press A to resume",
             a->cmd_idx + 1, a->n_commands);
    char det[256];
    snprintf(det, sizeof det,
             "cmd_idx=%d/%d repeat_idx=%d/%d sends_total=%d "
             "power=%.20s interval_s=%.2f file=\"%.100s\"",
             a->cmd_idx + 1, a->n_commands, a->repeat_idx, a->repeats_total,
             a->sends_total, a->power, a->interval_s_val, a->file_path);
    auto_tcmd_log(state, "auto-tcmd-pause", det);
}

// 'A' from the interrupt prompt: abort outright (no resume). The modal
// stays open in STOPPED so the operator sees the final numbers.
static void auto_tcmd_abort(state_t *state) {
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    a->state = AUTO_STATE_STOPPED;
    snprintf(a->status_msg, sizeof a->status_msg,
             "aborted at cmd %d/%d", a->cmd_idx + 1, a->n_commands);
    char det[160];
    snprintf(det, sizeof det, "cmd_idx=%d/%d sends_total=%d",
             a->cmd_idx + 1, a->n_commands, a->sends_total);
    auto_tcmd_log(state, "auto-tcmd-abort", det);
}

// 'R' from the resume prompt: continue from where it left off. Shift the
// elapsed-time origin forward by the pause duration so the progress readout
// stays honest, and fire the next send immediately.
static void auto_tcmd_resume(state_t *state) {
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    long now = ts_now_ns();
    if (a->pause_ns > 0 && now > a->pause_ns) {
        a->start_ns += (now - a->pause_ns);
    }
    a->pause_ns     = 0;
    a->next_send_ns = now;
    a->state        = AUTO_STATE_RUNNING;
    snprintf(a->status_msg, sizeof a->status_msg,
             "resumed at cmd %d/%d", a->cmd_idx + 1, a->n_commands);
    char det[160];
    snprintf(det, sizeof det, "cmd_idx=%d/%d repeat_idx=%d/%d sends_total=%d",
             a->cmd_idx + 1, a->n_commands, a->repeat_idx, a->repeats_total,
             a->sends_total);
    auto_tcmd_log(state, "auto-tcmd-resume", det);
}

// 'S' from the resume prompt: start the whole list over from the top,
// keeping the interval / power / allow-tx the operator already set.
static void auto_tcmd_restart(state_t *state) {
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    long now = ts_now_ns();
    a->cmd_idx          = 0;
    a->repeat_idx       = 0;
    a->sends_total      = 0;
    a->tx_seconds_spent = 0.0;
    a->start_ns         = now;
    a->next_send_ns     = now;
    a->pause_ns         = 0;
    a->state            = AUTO_STATE_RUNNING;
    snprintf(a->status_msg, sizeof a->status_msg,
             "restarted from the top (%d commands)", a->n_commands);
    char det[160];
    snprintf(det, sizeof det, "n_commands=%d repeats=%d",
             a->n_commands, a->repeats_total);
    auto_tcmd_log(state, "auto-tcmd-restart", det);
}

int auto_tcmd_handle_key(state_t *state, int key) {
    if (!state->tx.auto_tcmd_active) return 0;
    if (key == ERR) return 1;
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    int changed = 1;
    // Esc-as-CSI same fallback the TX modal uses.
    if (key == 27) {
        int translated = tx_drain_csi(state->tx.auto_tcmd_win);
        if (translated >= 0) {
            key = translated;
        } else {
            // Bare Esc -- meaning depends on the run state.
            if (a->state == AUTO_STATE_RUNNING) {
                // Don't let a stray Esc kill an active run; ask first.
                a->state = AUTO_STATE_PAUSE_PROMPT;
                snprintf(a->status_msg, sizeof a->status_msg,
                         "interrupt the run?  P pause (resume later)   "
                         "A abort   Esc keep running");
                auto_tcmd_draw(state);
                return 1;
            }
            if (a->state == AUTO_STATE_PAUSE_PROMPT) {
                a->state = AUTO_STATE_RUNNING;   // cancel: keep running
                snprintf(a->status_msg, sizeof a->status_msg, "running");
                auto_tcmd_draw(state);
                return 1;
            }
            if (a->state == AUTO_STATE_RESUME_PROMPT) {
                a->state = AUTO_STATE_PAUSED;    // leave it parked
                return 0;                        // close (stays parked)
            }
            return 0;  // SETUP / STOPPED / DONE / PASS_OVER -> close
        }
    }

    // Interrupt prompt (Esc during a run): pause or abort.
    if (a->state == AUTO_STATE_PAUSE_PROMPT) {
        if (key == 'p' || key == 'P') {
            auto_tcmd_pause(state);
            return 0;   // close so the operator can compose / do other work
        }
        if (key == 'a' || key == 'A') {
            auto_tcmd_abort(state);
            auto_tcmd_draw(state);
            return 1;
        }
        return 1;  // swallow everything else while prompting
    }

    // Resume prompt ('A' on a parked run): resume from here or start over.
    if (a->state == AUTO_STATE_RESUME_PROMPT) {
        if (key == 'r' || key == 'R') {
            auto_tcmd_resume(state);
            auto_tcmd_draw(state);
            return 1;
        }
        if (key == 's' || key == 'S') {
            auto_tcmd_restart(state);
            auto_tcmd_draw(state);
            return 1;
        }
        return 1;  // swallow everything else while prompting
    }

    if (a->state == AUTO_STATE_RUNNING) {
        // Run mode: only stop / close commands are honoured. Field
        // edits are blocked so an operator can't change power mid-run.
        if (key == 's' || key == 'S') {
            auto_tcmd_stop(state, "user");
            auto_tcmd_draw(state);
            return 1;
        }
        return 1;
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        // Redraw either way: auto_tcmd_start sets the status message on both
        // success and rejection, and the modal stays open in both cases.
        (void) auto_tcmd_start(state);
        auto_tcmd_draw(state);
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
    if (changed) auto_tcmd_draw(state);
    return 1;
}

// Per-tick burst driver. When running, queues one state->tx.tx_request when
// (a) the previous burst has cleared, and (b) the inter-send interval
// has elapsed. Stops automatically on LOS so an unattended run won't
// keep TXing after the pass. emit_tx_event_local fires from the main
// loop's burst-handler the same way it does for the manual TX
// compose path, so tx.log + viewer fanout capture every shot.
void auto_tcmd_tick(state_t *state) {
    if (!state->tx.auto_tcmd_active) return;
    auto_tcmd_t *a = &state->tx.auto_tcmd;
    if (a->state != AUTO_STATE_RUNNING) return;

    // Elapsed wall-clock since the run started, capped at the estimate,
    // so the Progress line reads elapsed/total (inter-send intervals and the
    // burst start lead included) rather than on-air seconds only. Frozen
    // automatically once the state leaves RUNNING -- this returns early then.
    {
        double elapsed = (double) (ts_now_ns() - a->start_ns) * 1e-9;
        if (elapsed < 0.0) elapsed = 0.0;
        if (elapsed > a->tx_seconds_total) elapsed = a->tx_seconds_total;
        a->tx_seconds_spent = elapsed;
    }

    // LOS guard. We consider the pass over once the elevation has
    // gone negative AND the predictor has rolled the next pass into
    // the future. Sitting on a freshly-loaded prediction during AOS
    // ambiguity (elevation < 0 but next pass not yet predicted) is
    // not enough to abort; the running flag stays so a momentary
    // numerical wobble can't kill an active session.
    //
    // --testing runs are bench / characterisation work that isn't tied to
    // a pass: the satellite is normally below the horizon the whole time,
    // so this guard would otherwise abort the run before its first send
    // (the manual compose path has no such guard, which is why it worked
    // out of a pass and auto-tcmd didn't). Skip the guard entirely then.
    double el = state->track.prediction.satellite_ephem.elevation;
    if (!state->app.testing_mode
        && el < 0.0
        && state->track.prediction.predicted_minutes_until_visible > 0.5) {
        a->state = AUTO_STATE_PASS_OVER;
        snprintf(a->status_msg, sizeof a->status_msg,
                 "stopped: pass over (elevation %.1f deg)", el);
        auto_tcmd_draw(state);
        return;
    }

    if (a->cmd_idx >= a->n_commands) {
        a->state = AUTO_STATE_DONE;
        snprintf(a->status_msg, sizeof a->status_msg,
                 "done: sent all %d command(s)", a->n_commands);
        auto_tcmd_draw(state);
        return;
    }

#ifdef SSO_WITH_SDR
    long now = ts_now_ns();
    if (now < a->next_send_ns) return;
    if (state->tx.tx_request.pending)  return;  // prior burst still inflight

    const char *raw = a->commands[a->cmd_idx];
    // Expand a simple_sat_ops-directed "SSO+..." line into the concrete
    // telecommand, clock captured now so each send (and each repeat) carries a
    // fresh time. A normal line passes through verbatim. Startup lint already
    // vetted SSO+ lines, so a failure here is defensive: skip the whole command
    // rather than key a half-built payload.
    sso_pseudo_ctx_t pc = { .now_ms    = sso_now_utc_ms(),
                            .tssent_ms = state->tx.sso_pass_tssent_ms };
    char wire[512];
    char sso_err[160];
    sso_pseudo_status_t pst =
        sso_pseudo_expand(raw, &pc, wire, sizeof wire, sso_err, sizeof sso_err);
    if (pst != SSO_PSEUDO_OK && pst != SSO_PSEUDO_NOT_PSEUDO) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "skipped SSO+ line %d: %.120s", a->cmd_idx + 1, sso_err);
        a->cmd_idx++;
        a->repeat_idx   = 0;
        // A skipped command keys nothing, so don't make the run idle the
        // inter-send interval over it -- advance to the next command on the
        // following tick. The interval paces actual transmissions, not no-ops.
        a->next_send_ns = now;
        auto_tcmd_draw(state);
        return;
    }
    size_t n = strlen(wire);
    // Never key a command longer than the RF link can carry. The Reed-Solomon
    // block caps the on-air telecommand at TCMD_RF_MAX_LEN; the tx_request
    // buffer is much larger, so the clamp below would silently truncate an
    // over-long command into a corrupt frame rather than reject it. Reload
    // lint catches over-long raw lines, but an SSO+ line only reaches its
    // final length after expansion here, so this is the authoritative gate.
    if (n > (size_t) TCMD_RF_MAX_LEN) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "skipped line %d: %zu chars over the %d-char RF limit",
                 a->cmd_idx + 1, n, TCMD_RF_MAX_LEN);
        a->cmd_idx++;
        a->repeat_idx   = 0;
        // A skipped command keys nothing, so don't make the run idle the
        // inter-send interval over it -- advance to the next command on the
        // following tick. The interval paces actual transmissions, not no-ops.
        a->next_send_ns = now;
        auto_tcmd_draw(state);
        return;
    }
    if (n > sizeof state->tx.tx_request.payload) n = sizeof state->tx.tx_request.payload;
    memcpy(state->tx.tx_request.payload, wire, n);
    state->tx.tx_request.payload_len  = n;
    state->tx.tx_request.is_hex       = 0;
    state->tx.tx_request.csp_hdr      = (csp_v1_header_t){
        .prio  = 2, .src = 10, .dst = 1, .dport = 7, .sport = 16, .flags = 0,
    };
    state->tx.tx_request.tx_freq_hz       = state->tx.tx_freq_hz_doppler;
    state->tx.tx_request.tx_gain_db       = atof(a->power);
    state->tx.tx_request.repeat           = 1;
    state->tx.tx_request.gap_ms           = 200;
    state->tx.tx_request.preroll_ms       = state->tx.tx_preroll_ms;
    // No state->tx.tx_request.allow_tx field — the TX-inhibit gate is enforced
    // at auto_tcmd_start time (refuses to enter RUNNING unless allow_tx
    // is ticked), same way tx_compose_validate handles it before commit.
    state->tx.tx_request.allow_high_power = 0;
    state->tx.tx_request.allow_hf_tx      = 0;
    snprintf(state->tx.tx_request.tx_source, sizeof state->tx.tx_request.tx_source,
             "auto-cmd (file)");
    if (pst == SSO_PSEUDO_OK)
        snprintf(state->tx.tx_request.sso_origin, sizeof state->tx.tx_request.sso_origin, "%s", raw);
    else
        state->tx.tx_request.sso_origin[0] = '\0';
    {
        int m = snprintf(state->tx.tx_request.summary, sizeof state->tx.tx_request.summary,
                         "auto[%d/%d %d/%d]: %.190s",
                         a->cmd_idx + 1, a->n_commands,
                         a->repeat_idx + 1, a->repeats_total,
                         wire);
        if (pst == SSO_PSEUDO_OK && m > 0 && (size_t) m < sizeof state->tx.tx_request.summary)
            snprintf(state->tx.tx_request.summary + m, sizeof state->tx.tx_request.summary - m,
                     " (replaced '%s')", raw);
    }
    state->tx.tx_request.pending = 1;
    // Stamp the staging moment so tx_burst_service_request can measure the
    // real send-to-send period and slot-hold of this burst.
    state->tx.tx_request_staged_ns = now;
    snprintf(a->last_sent, sizeof a->last_sent, "%.255s", wire);
    a->sends_total++;

    a->repeat_idx++;
    if (a->repeat_idx >= a->repeats_total) {
        a->cmd_idx++;
        a->repeat_idx = 0;
    }

    a->next_send_ns = now + (long)(a->interval_s_val * 1e9);
    auto_tcmd_draw(state);
#else
    (void) state;
    a->state = AUTO_STATE_STOPPED;
    snprintf(a->status_msg, sizeof a->status_msg,
             "stopped: this build has no SDR support");
    auto_tcmd_draw(state);
#endif
}
