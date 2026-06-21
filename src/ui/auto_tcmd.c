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
// command should be sent, and the inter-send delay. Once started the
// modal's tick runs alongside the main loop — non-blocking, like the
// TX compose modal — and queues one state->tx_request per shot, advancing
// through the file. Stops automatically when the satellite drops
// below the horizon (LOS) so an unattended run can't keep TXing after
// the pass. Every send goes through emit_tx_event_local, so the
// existing tx.log + viewer fanout capture all of them.
// auto_tcmd_field_t, auto_tcmd_state_t and auto_tcmd_t now live in
// state.h. The live modal state (active flag, window, the auto_tcmd_t
// run state, and the --tc-file path) are fields on state_t.

// Wall-clock seconds one auto-tcmd send occupies, end to end. Mirrors
// the framing and the fixed timing in tx_burst.c's build_iq / tx_burst_run:
//
//   frame_bytes = prefill(32) + ASM(4) + Golay(3)
//                 + csp_hdr(4) + payload + hmac(4) + rs_parity(32)
//                 + tailfill(1)
//               = 80 + payload
//
//   burst_s     = start_delay(0.5)            // UHD timed-start lead
//                 + state->tx_preroll_ms/1000       // modulated 0xAA carrier
//                 + frame_bytes * 8 / bit_rate // the frame itself
//                 + postroll(0.050)
//
// The start lead matters: tx_burst_run schedules the burst 0.5 s ahead
// and blocks until it completes, so each send is inhibited for the whole
// span -- leaving it out is most of the per-burst underestimate. auto-tcmd
// sends one burst per shot (repeat=1); the repeat count and inter-send
// delay are folded in by the caller, so this stays a per-send quantum.
static double auto_tcmd_burst_seconds(state_t *state, size_t payload_len) {
    const double start_delay_s = 0.500;   // tx_burst.c start_delay_s
    const double preroll_s     = (double) state->tx_preroll_ms * 1e-3;
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

// Snapshot of the auto-tcmd run for the viewer broadcast: sends queued
// so far vs. the run's planned total (commands × repeats), plus the
// run-state label. Returns 1 when there is a run to report — the modal
// is open and Enter has started it (running, or finished and still on
// screen so viewers see the final tally). Returns 0 in setup or when
// the modal is closed, which drops the fields from the wire entirely.
int auto_tcmd_progress(state_t *state, int *sent, int *total, const char **label) {
    const auto_tcmd_t *a = &state->auto_tcmd;
    if (!state->auto_tcmd_active || a->state == AUTO_STATE_SETUP) return 0;
    *sent  = a->sends_total;
    *total = a->n_commands * a->repeats_total;
    *label = auto_tcmd_state_label(a->state);
    return 1;
}

int auto_field_is_text(auto_tcmd_field_t f) {
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
    if (!state->auto_tcmd_active || !state->auto_tcmd_win) return;
    WINDOW *w = state->auto_tcmd_win;
    auto_tcmd_t *a = &state->auto_tcmd;
    werase(w);
    draw_box(w);
    int width = getmaxx(w);

    mvwprintw(w, 0, 2, " Auto-TCMD (operator: %s)%s ",
              state->operator_user ? state->operator_user : "?",
              state->no_tx ? "  [--no-tx]" : "");

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
    mvwprintw(w, 4, 24, "per command (TCM1 Nx, then TCM2 Nx, ...)");
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

    if (a->state == AUTO_STATE_RUNNING) {
        mvwprintw(w, 14, 2,
                  "Running - s stops   Esc closes (and stops)");
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
void auto_tcmd_open(state_t *state) {
    if (!state->ipc) return;
    if (state->tx_compose_active) return;
    if (state->auto_tcmd_active) return;
    if (state->auto_tcmd_file_path[0] == '\0') return;

    // (Re)load on open — file may have been edited since last open.
    if (state->auto_tcmd.commands) {
        auto_tcmd_free_commands(state->auto_tcmd.commands, state->auto_tcmd.n_commands);
        state->auto_tcmd.commands = NULL;
        state->auto_tcmd.n_commands = 0;
    }
    char **cmds = NULL;
    int    nc   = 0;
    if (auto_tcmd_load_file(state->auto_tcmd_file_path, &cmds, &nc) != 0) {
        return;  // silent — operator will notice via the absent modal
    }

    memset(&state->auto_tcmd, 0, sizeof state->auto_tcmd);
    state->auto_tcmd.commands   = cmds;
    state->auto_tcmd.n_commands = nc;

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
            int e = tcmd_lint_file(state->auto_tcmd_file_path, devnull, &w);
            state->auto_tcmd.lint_errors = (e > 0) ? e : 0;
            fclose(devnull);
        }
    }
    snprintf(state->auto_tcmd.file_path, sizeof state->auto_tcmd.file_path,
             "%.*s", (int)(sizeof state->auto_tcmd.file_path - 1),
             state->auto_tcmd_file_path);
    snprintf(state->auto_tcmd.power,   sizeof state->auto_tcmd.power,   "80.0");
    snprintf(state->auto_tcmd.repeats, sizeof state->auto_tcmd.repeats, "3");
    snprintf(state->auto_tcmd.delay_s, sizeof state->auto_tcmd.delay_s, "2.0");
    state->auto_tcmd.allow_tx = 0;
    state->auto_tcmd.focus    = AUTO_F_POWER;
    state->auto_tcmd.cursors[AUTO_F_POWER]   = (int) strlen(state->auto_tcmd.power);
    state->auto_tcmd.cursors[AUTO_F_REPEATS] = (int) strlen(state->auto_tcmd.repeats);
    state->auto_tcmd.cursors[AUTO_F_DELAY]   = (int) strlen(state->auto_tcmd.delay_s);
    state->auto_tcmd.state    = AUTO_STATE_SETUP;
    if (state->auto_tcmd.lint_errors > 0 && !state->ignore_tc_errors) {
        snprintf(state->auto_tcmd.status_msg, sizeof state->auto_tcmd.status_msg,
                 "loaded %d command(s) but %d lint error(s) -- fix the file; "
                 "run blocked", nc, state->auto_tcmd.lint_errors);
    } else if (state->auto_tcmd.lint_errors > 0) {
        snprintf(state->auto_tcmd.status_msg, sizeof state->auto_tcmd.status_msg,
                 "loaded %d command(s); %d lint error(s) ignored. Enter to start.",
                 nc, state->auto_tcmd.lint_errors);
    } else {
        snprintf(state->auto_tcmd.status_msg, sizeof state->auto_tcmd.status_msg,
                 "loaded %d command(s). Set fields, then Enter to start.",
                 nc);
    }

    int h = 17, ww = 110;
    if (h > LINES) h = LINES;
    if (ww > COLS) ww = COLS;
    if (ww < 60)  ww = (COLS < 60) ? COLS : 60;
    state->auto_tcmd_win = newwin(h, ww, (LINES - h) / 2, (COLS - ww) / 2);
    if (!state->auto_tcmd_win) {
        auto_tcmd_free_commands(state->auto_tcmd.commands, state->auto_tcmd.n_commands);
        state->auto_tcmd.commands = NULL;
        state->auto_tcmd.n_commands = 0;
        return;
    }
    keypad(state->auto_tcmd_win, TRUE);
    nodelay(state->auto_tcmd_win, TRUE);
    state->auto_tcmd_active = 1;
    auto_tcmd_draw(state);
}

void auto_tcmd_close(state_t *state) {
    if (state->auto_tcmd_win) {
        delwin(state->auto_tcmd_win);
        state->auto_tcmd_win = NULL;
    }
    state->auto_tcmd_active = 0;
    if (state->auto_tcmd.commands) {
        auto_tcmd_free_commands(state->auto_tcmd.commands, state->auto_tcmd.n_commands);
        state->auto_tcmd.commands = NULL;
        state->auto_tcmd.n_commands = 0;
    }
    touchwin(stdscr);
    refresh();
}

// Validate the setup fields and move to RUNNING. Returns 0 on success,
// fills status_msg + returns -1 on failure.
static int auto_tcmd_start(state_t *state) {
    auto_tcmd_t *a = &state->auto_tcmd;
    if (a->n_commands == 0) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "rejected: file has no commands");
        return -1;
    }
    // A file edited after launch can introduce commands the satellite would
    // reject or mis-parse. Re-lint on (re)load flagged them; refuse to run
    // unless the operator started with --ignore-at-your-peril-all-tc-errors.
    if (a->lint_errors > 0 && !state->ignore_tc_errors) {
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
    a->tx_seconds_spent = 0.0;
    // Wall-clock estimate for the whole run. auto_tcmd_tick spaces sends
    // by max(delay, burst): it waits `delay` measured from the start of
    // each send AND for that send's burst to clear, so the delay and the
    // burst overlap rather than add. Every command is sent `repeats`
    // times; only the final send has no trailing delay.
    a->tx_seconds_total = 0.0;
    double last_burst = 0.0;
    for (int i = 0; i < a->n_commands; ++i) {
        double burst = auto_tcmd_burst_seconds(state, strlen(a->commands[i]));
        double slot  = (burst > delay) ? burst : delay;
        a->tx_seconds_total += slot * (double) repeats;
        last_burst = burst;
    }
    if (a->n_commands > 0 && delay > last_burst)
        a->tx_seconds_total -= (delay - last_burst);
    a->start_ns      = ts_now_ns();
    a->next_send_ns  = a->start_ns;  // first send fires immediately
    a->state         = AUTO_STATE_RUNNING;
    snprintf(a->status_msg, sizeof a->status_msg,
             "running: %d cmds x %d repeats, %.2f s delay",
             a->n_commands, repeats, delay);
    {
        char det[256];
        snprintf(det, sizeof det,
                 "n_commands=%d repeats=%d delay_s=%.2f "
                 "allow_tx=%d power=%.100s file=\"%.100s\"",
                 a->n_commands, repeats, delay, a->allow_tx,
                 a->power, a->file_path);
        sso_audit_event("auto-tcmd-start", det);
    }
    return 0;
}

// Pause / cancel without closing the modal so the operator can see the
// final progress numbers.
static void auto_tcmd_stop(state_t *state, const char *reason) {
    auto_tcmd_t *a = &state->auto_tcmd;
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

int auto_tcmd_handle_key(state_t *state, int key) {
    if (!state->auto_tcmd_active) return 0;
    if (key == ERR) return 1;
    auto_tcmd_t *a = &state->auto_tcmd;
    int changed = 1;
    // Esc-as-CSI same fallback the TX modal uses.
    if (key == 27) {
        int translated = tx_drain_csi(state->auto_tcmd_win);
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
            auto_tcmd_stop(state, "user");
            auto_tcmd_draw(state);
            return 1;
        }
        return 1;
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        if (auto_tcmd_start(state) == 0) {
            auto_tcmd_draw(state);
        } else {
            auto_tcmd_draw(state);
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
    if (changed) auto_tcmd_draw(state);
    return 1;
}

// Per-tick burst driver. When running, queues one state->tx_request when
// (a) the previous burst has cleared, and (b) the inter-send delay
// has elapsed. Stops automatically on LOS so an unattended run won't
// keep TXing after the pass. emit_tx_event_local fires from the main
// loop's burst-handler the same way it does for the manual TX
// compose path, so tx.log + viewer fanout capture every shot.
void auto_tcmd_tick(state_t *state) {
    if (!state->auto_tcmd_active) return;
    auto_tcmd_t *a = &state->auto_tcmd;
    if (a->state != AUTO_STATE_RUNNING) return;

    // Elapsed wall-clock since the run started, capped at the estimate,
    // so the Progress line reads elapsed/total (inter-send delays and the
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
    double el = state->prediction.satellite_ephem.elevation;
    if (el < 0.0
        && state->prediction.predicted_minutes_until_visible > 0.5) {
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
    if (state->tx_request.pending)  return;  // prior burst still inflight

    const char *raw = a->commands[a->cmd_idx];
    // Expand a simple_sat_ops-directed "SSO+..." line into the concrete
    // telecommand, clock captured now so each send (and each repeat) carries a
    // fresh time. A normal line passes through verbatim. Startup lint already
    // vetted SSO+ lines, so a failure here is defensive: skip the whole command
    // rather than key a half-built payload.
    sso_pseudo_ctx_t pc = { .now_ms    = sso_now_utc_ms(),
                            .tssent_ms = state->sso_pass_tssent_ms };
    char wire[512];
    char sso_err[160];
    sso_pseudo_status_t pst =
        sso_pseudo_expand(raw, &pc, wire, sizeof wire, sso_err, sizeof sso_err);
    if (pst != SSO_PSEUDO_OK && pst != SSO_PSEUDO_NOT_PSEUDO) {
        snprintf(a->status_msg, sizeof a->status_msg,
                 "skipped SSO+ line %d: %.120s", a->cmd_idx + 1, sso_err);
        a->cmd_idx++;
        a->repeat_idx   = 0;
        a->next_send_ns = now + (long)(a->delay_s_val * 1e9);
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
        a->next_send_ns = now + (long)(a->delay_s_val * 1e9);
        auto_tcmd_draw(state);
        return;
    }
    if (n > sizeof state->tx_request.payload) n = sizeof state->tx_request.payload;
    memcpy(state->tx_request.payload, wire, n);
    state->tx_request.payload_len  = n;
    state->tx_request.is_hex       = 0;
    state->tx_request.csp_hdr      = (csp_v1_header_t){
        .prio  = 2, .src = 10, .dst = 1, .dport = 7, .sport = 16, .flags = 0,
    };
    state->tx_request.tx_freq_hz       = state->tx_freq_hz_doppler;
    state->tx_request.tx_gain_db       = atof(a->power);
    state->tx_request.repeat           = 1;
    state->tx_request.gap_ms           = 200;
    state->tx_request.preroll_ms       = state->tx_preroll_ms;
    // No state->tx_request.allow_tx field — the TX-inhibit gate is enforced
    // at auto_tcmd_start time (refuses to enter RUNNING unless allow_tx
    // is ticked), same way tx_compose_validate handles it before commit.
    state->tx_request.allow_high_power = 0;
    state->tx_request.allow_hf_tx      = 0;
    if (pst == SSO_PSEUDO_OK)
        snprintf(state->tx_request.sso_origin, sizeof state->tx_request.sso_origin, "%s", raw);
    else
        state->tx_request.sso_origin[0] = '\0';
    {
        int m = snprintf(state->tx_request.summary, sizeof state->tx_request.summary,
                         "auto[%d/%d %d/%d]: %.190s",
                         a->cmd_idx + 1, a->n_commands,
                         a->repeat_idx + 1, a->repeats_total,
                         wire);
        if (pst == SSO_PSEUDO_OK && m > 0 && (size_t) m < sizeof state->tx_request.summary)
            snprintf(state->tx_request.summary + m, sizeof state->tx_request.summary - m,
                     " (replaced '%s')", raw);
    }
    state->tx_request.pending = 1;
    snprintf(a->last_sent, sizeof a->last_sent, "%.255s", wire);
    a->sends_total++;

    a->repeat_idx++;
    if (a->repeat_idx >= a->repeats_total) {
        a->cmd_idx++;
        a->repeat_idx = 0;
    }

    a->next_send_ns = now + (long)(a->delay_s_val * 1e9);
    auto_tcmd_draw(state);
#else
    (void) state;
    a->state = AUTO_STATE_STOPPED;
    snprintf(a->status_msg, sizeof a->status_msg,
             "stopped: this build has no SDR support");
    auto_tcmd_draw(state);
#endif
}
