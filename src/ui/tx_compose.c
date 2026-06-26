/*

   Simple Satellite Operations  ui/tx_compose.c

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

#include "tx_compose.h"
#include "state.h"

#include "panels.h"          // draw_box, tx_drain_csi
#include "frontiersat.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_pseudo.h"
#include "sso_time.h"
#include "tcmd_lint.h"
#include "tx_log.h"
#include "ui_textfield.h"

#include <math.h>
#include <ncurses.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef SSO_WITH_SDR
#include "rx_session.h"
#endif

// TX-compose preview-debounce interval: hold off broadcasting a draft until
// the operator has been idle this long (ns).
static const long g_tx_compose_debounce_ns = 200000000L;

static void tx_history_push(state_t *state, const char *payload) {
    if (payload == NULL || payload[0] == '\0') return;
    if (state->tx.tx_history_count > 0
        && strcmp(state->tx.tx_history[0], payload) == 0) {
        return;  // suppress trivial duplicates of the most-recent entry
    }
    int keep = state->tx.tx_history_count < TX_HISTORY_MAX - 1
             ? state->tx.tx_history_count : TX_HISTORY_MAX - 1;
    for (int i = keep; i > 0; --i) {
        memcpy(state->tx.tx_history[i], state->tx.tx_history[i - 1], sizeof state->tx.tx_history[0]);
    }
    snprintf(state->tx.tx_history[0], sizeof state->tx.tx_history[0], "%s", payload);
    if (state->tx.tx_history_count < TX_HISTORY_MAX) state->tx.tx_history_count++;
}

static void tx_compose_init(state_t *state, tx_compose_t *c) {
    memset(c, 0, sizeof *c);
    snprintf(c->payload, sizeof c->payload, "%s", state->tx.tx_last_payload);
    snprintf(c->power,   sizeof c->power,   "%s", state->tx.tx_last_power);
    c->allow_tx                = state->tx.tx_last_allow_tx;
    c->cursors[TXF_PAYLOAD]    = (int) strlen(c->payload);
    c->cursors[TXF_POWER]      = (int) strlen(c->power);
    c->history_idx             = -1;
    snprintf(c->status_msg, sizeof c->status_msg,
             "edit; viewers see drafts ~200 ms after you stop typing");
}

static void tx_compose_remember(state_t *state, const tx_compose_t *c) {
    snprintf(state->tx.tx_last_payload, sizeof state->tx.tx_last_payload, "%s", c->payload);
    snprintf(state->tx.tx_last_power,   sizeof state->tx.tx_last_power,   "%s", c->power);
    state->tx.tx_last_allow_tx = c->allow_tx;
}

static void tx_compose_summary(const tx_compose_t *c, char *out, size_t out_size) {
    if (out_size == 0) return;
    snprintf(out, out_size, "%s", c->payload[0] ? c->payload : "(empty)");
}

static void tx_compose_fill_event(state_t *state, const tx_compose_t *c, sso_event_t *evt) {
    snprintf(evt->tx_payload_kind, sizeof evt->tx_payload_kind, "ascii");
    snprintf(evt->tx_payload, sizeof evt->tx_payload, "%s", c->payload);
    // CSP defaults match cts_send -> FrontierSat OBC (CTS1 cmd handler).
    evt->tx_csp_src   = 10;
    evt->tx_csp_dst   = 1;
    evt->tx_csp_dport = 7;
    evt->tx_csp_sport = 16;
    evt->tx_csp_prio  = 2;
#ifdef SSO_WITH_SDR
    evt->tx_freq_hz   = state->tx.tx_freq_hz_doppler;
#else
    evt->tx_freq_hz   = (long) FRONTIERSAT_CARRIER_HZ;
#endif
    evt->tx_gain_db   = atof(c->power);
    evt->tx_repeat    = 1;
    evt->tx_gap_ms    = 200;
    evt->tx_allow_tx         = c->allow_tx;
    evt->tx_allow_high_power = 0;
    evt->tx_allow_hf_tx      = 0;
    char summary[SSO_TX_TEXT_MAX];
    tx_compose_summary(c, summary, sizeof summary);
    snprintf(evt->ascii, sizeof evt->ascii, "%s", summary);
    snprintf(evt->from, sizeof evt->from, "%s",
             state->op.operator_user ? state->op.operator_user : "?");
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
    ui_tf_clamp_cursor(buf, &c->cursors[f]);
}

// Mark an edit: refresh the preview and, on the payload field, cancel
// history navigation so the next UP walks history fresh.
static void tx_field_after_edit(tx_compose_t *c) {
    c->preview_dirty = 1;
    if (c->focus == TXF_PAYLOAD) c->history_idx = -1;
}

static void tx_field_insert(tx_compose_t *c, int ch) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    // The payload is transmitted verbatim, so cap typing at the over-the-
    // air limit (TCMD_RF_MAX_LEN chars) even though the buffer is larger:
    // a longer telecommand can't be framed for the radio.
    if (c->focus == TXF_PAYLOAD && cap > (size_t) TCMD_RF_MAX_LEN + 1)
        cap = (size_t) TCMD_RF_MAX_LEN + 1;
    int n = (int) strlen(buf);
    if (n + 1 >= (int) cap) {
        if (c->focus == TXF_PAYLOAD)
            snprintf(c->status_msg, sizeof c->status_msg,
                     "at the %d-char RF uplink limit", TCMD_RF_MAX_LEN);
        return;
    }
    int accept = 0;
    if (tx_field_is_text(c->focus)) {
        accept = (ch >= 32 && ch < 127);
    } else if (tx_field_is_decimal(c->focus)) {
        accept = (ch >= '0' && ch <= '9') || ch == '.' || ch == '-';
    }
    if (!accept) return;
    if (ui_tf_insert(buf, cap, &c->cursors[c->focus], ch))
        tx_field_after_edit(c);
}

static void tx_field_backspace(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    if (ui_tf_backspace(buf, &c->cursors[c->focus]))
        tx_field_after_edit(c);
}

static void tx_field_delete(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    if (ui_tf_delete(buf, &c->cursors[c->focus]))
        tx_field_after_edit(c);
}

static void tx_field_kill_to_end(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    if (ui_tf_kill_to_end(buf, &c->cursors[c->focus]))
        tx_field_after_edit(c);
}

static void tx_field_kill_word_back(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    if (ui_tf_kill_word_back(buf, &c->cursors[c->focus]))
        tx_field_after_edit(c);
}

static void tx_field_left(tx_compose_t *c) {
    size_t cap = 0;
    if (!tx_field_buf(c, c->focus, &cap)) return;
    ui_tf_left(&c->cursors[c->focus]);
}

static void tx_field_right(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    ui_tf_right(buf, &c->cursors[c->focus]);
}

static void tx_field_home(tx_compose_t *c) {
    size_t cap = 0;
    if (!tx_field_buf(c, c->focus, &cap)) return;
    ui_tf_home(&c->cursors[c->focus]);
}

static void tx_field_end(tx_compose_t *c) {
    size_t cap = 0;
    char *buf = tx_field_buf(c, c->focus, &cap);
    if (!buf) return;
    ui_tf_end(buf, &c->cursors[c->focus]);
}

// direction = -1 (UP, older) or +1 (DOWN, newer). No-op when focus
// is not on the payload field, when history is empty, or at the edge.
static void tx_history_recall(state_t *state, tx_compose_t *c, int direction) {
    if (c->focus != TXF_PAYLOAD) return;
    if (state->tx.tx_history_count == 0) return;
    int step = (direction < 0) ? +1 : -1;
    int new_idx = c->history_idx + step;
    if (new_idx < -1) return;
    if (new_idx >= state->tx.tx_history_count) return;
    if (c->history_idx == -1 && new_idx >= 0) {
        snprintf(c->history_saved_edit, sizeof c->history_saved_edit,
                 "%s", c->payload);
    }
    if (new_idx == -1) {
        snprintf(c->payload, sizeof c->payload, "%s",
                 c->history_saved_edit);
    } else {
        snprintf(c->payload, sizeof c->payload, "%s",
                 state->tx.tx_history[new_idx]);
    }
    // Bulk-populate bypasses the per-keystroke clamp, so re-clamp to the
    // on-air limit here — the field should never show more than what could
    // actually be committed, even momentarily.
    if (strlen(c->payload) > (size_t) TCMD_RF_MAX_LEN)
        c->payload[TCMD_RF_MAX_LEN] = '\0';
    c->cursors[TXF_PAYLOAD] = (int) strlen(c->payload);
    c->history_idx          = new_idx;
    c->preview_dirty        = 1;
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


static void tx_compose_draw(state_t *state, WINDOW *w, const tx_compose_t *c) {
    werase(w);
    draw_box(w);
    int width = getmaxx(w);
    int payload_w = width - 14;
    if (payload_w < 32) payload_w = 32;
    if (payload_w > (int) sizeof c->payload - 1)
        payload_w = (int) sizeof c->payload - 1;

    mvwprintw(w, 0, 2, " TX compose (operator: %s)%s ",
              state->op.operator_user ? state->op.operator_user : "?",
              state->tx.no_tx ? "  [--no-tx]" : "");
#ifdef SSO_WITH_SDR
    mvwprintw(w, 1, 2,
              "B210: %s",
              state->sdr.rx_session ? "in-process (this binary)" : "(offline)");
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

    char summary[SSO_TX_TEXT_MAX];
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
    if (strlen(c->payload) > (size_t) TCMD_RF_MAX_LEN) {
        snprintf(err, err_size,
                 "payload %zu chars exceeds the %d-char RF uplink limit",
                 strlen(c->payload), TCMD_RF_MAX_LEN);
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

static void tx_compose_broadcast_preview(state_t *state, const tx_compose_t *c) {
    if (!state->op.ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_TX_COMMAND_PREVIEW);
    tx_compose_fill_event(state, c, &evt);
    char buf[2048];
    if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
        sso_ipc_server_broadcast(state->op.ipc, buf);
    }
    // Mirror into our own ring buffer so the operator's TX log shows
    // the same draft line viewers are seeing.
    tx_log_push(state, &evt);
}

static int tx_compose_commit(state_t *state, const tx_compose_t *c, char *err, size_t err_size) {
#ifdef SSO_WITH_SDR
    if (state->tx.no_tx) {
        snprintf(err, err_size,
                 "TX disabled by --no-tx (preview still goes to viewers)");
        return -1;
    }
    if (state->sdr.rx_session != NULL && !rx_session_can_tx(state->sdr.rx_session)) {
        snprintf(err, err_size,
                 "TX not supported by this SDR (RX-only backend)");
        return -1;
    }
    if (state->tx.tx_request.pending) {
        snprintf(err, err_size, "previous burst still in flight");
        return -1;
    }
    // Expand a simple_sat_ops-directed "SSO+..." pseudo-command into the
    // concrete telecommand to transmit, with the clock captured now so the
    // embedded timestamp is current. A normal command passes through verbatim.
    sso_pseudo_ctx_t pc = { .now_ms    = sso_now_utc_ms(),
                            .tssent_ms = state->tx.sso_pass_tssent_ms };
    char wire[512];
    char sso_err[160];
    sso_pseudo_status_t pst =
        sso_pseudo_expand(c->payload, &pc, wire, sizeof wire, sso_err, sizeof sso_err);
    if (pst != SSO_PSEUDO_OK && pst != SSO_PSEUDO_NOT_PSEUDO) {
        snprintf(err, err_size, "SSO+ expansion failed: %s", sso_err);
        return -1;
    }
    size_t n = strlen(wire);
    if (n == 0) {
        snprintf(err, err_size, "empty payload");
        return -1;
    }
    if (n > sizeof state->tx.tx_request.payload) n = sizeof state->tx.tx_request.payload;
    memcpy(state->tx.tx_request.payload, wire, n);
    state->tx.tx_request.payload_len  = n;
    state->tx.tx_request.is_hex       = 0;  // always ascii in the simplified modal
    state->tx.tx_request.csp_hdr      = (csp_v1_header_t){
        .prio  = 2, .src = 10, .dst = 1, .dport = 7, .sport = 16, .flags = 0,
    };
    state->tx.tx_request.tx_freq_hz       = state->tx.tx_freq_hz_doppler;
    state->tx.tx_request.tx_gain_db       = atof(c->power);
    state->tx.tx_request.repeat           = 1;
    state->tx.tx_request.gap_ms           = 200;
    state->tx.tx_request.preroll_ms       = state->tx.tx_preroll_ms;
    state->tx.tx_request.allow_high_power = 0;
    state->tx.tx_request.allow_hf_tx      = 0;
    snprintf(state->tx.tx_request.tx_source, sizeof state->tx.tx_request.tx_source,
             "manual send");
    if (pst == SSO_PSEUDO_OK) {
        // Heritage: stash the SSO+ origin so the on-air summary notes it, and
        // bake the same note into the queue-time summary the dry-run /
        // rejected paths show.
        snprintf(state->tx.tx_request.sso_origin, sizeof state->tx.tx_request.sso_origin,
                 "%s", c->payload);
        snprintf(state->tx.tx_request.summary, sizeof state->tx.tx_request.summary,
                 "ascii:%.150s (replaced '%.80s')", wire, c->payload);
    } else {
        state->tx.tx_request.sso_origin[0] = '\0';
        tx_compose_summary(c, state->tx.tx_request.summary, sizeof state->tx.tx_request.summary);
    }
    state->tx.tx_request.pending = 1;
    // Stamp the staging moment so the burst handler can measure the slot-hold
    // (and, across sends, the period) the same way the auto-tcmd path does.
    state->tx.tx_request_staged_ns = ts_now_ns();
    {
        // Audit: TX commit — the moment the operator pressed Enter in
        // the compose modal and the burst was queued for the main loop
        // to actually transmit. The matching tx-command-sent (or
        // tx-not-sent) event lands when the burst returns (see
        // emit_tx_event_local site below).
        char det[512];
        snprintf(det, sizeof det,
                 "len=%zu freq_hz=%ld gain_db=%.1f payload=\"%.255s\"",
                 state->tx.tx_request.payload_len,
                 (long) state->tx.tx_request.tx_freq_hz,
                 state->tx.tx_request.tx_gain_db,
                 c->payload);
        sso_audit_event("tx-commit", det);
    }
    return 0;
#else
    (void) c;
    snprintf(err, err_size, "this build has no B210 support");
    return -1;
#endif
}

#ifdef SSO_WITH_SDR
// Emit a TX event locally: push into the operator's own TX log and
// broadcast to viewers via the IPC server.
void emit_tx_event_local(state_t *state, sso_event_type_t type,
                                 const char *summary,
                                 const char *ack_status)
{
    sso_event_t evt;
    sso_event_init(&evt, type);
    snprintf(evt.from, sizeof evt.from, "%s",
             state->op.operator_user ? state->op.operator_user : "?");
    // Carry the originating command's source ("auto-cmd (file)" / "manual
    // send") into the SENT / NOT_SENT event. The request slot still holds it
    // here -- tx_burst_service_request clears pending only after this emit.
    snprintf(evt.tx_origin, sizeof evt.tx_origin, "%s", state->tx.tx_request.tx_source);
    if (summary && summary[0]) {
        snprintf(evt.ascii, sizeof evt.ascii, "%s", summary);
    }
    if (ack_status && ack_status[0]) {
        snprintf(evt.tx_not_sent_reason, sizeof evt.tx_not_sent_reason, "%s", ack_status);
    }
    tx_log_push(state, &evt);
    if (state->op.ipc) {
        char buf[2048];
        if (sso_event_encode(&evt, buf, sizeof buf) == 0) {
            sso_ipc_server_broadcast(state->op.ipc, buf);
        }
    }
}

#endif

// Open the modal — allocate the window, seed the compose state, draw
// once, and flip state->tx.tx_compose_active so the main loop starts ticking
// it. Idempotent: re-opening while already active is a no-op.
void tx_compose_open(state_t *state) {
    if (!state->op.ipc) return;
    if (state->tx.tx_compose_active) return;
    if (state->tx.auto_tcmd_active) return;  // one modal at a time
    int h = 14, ww = 120;
    if (h > LINES) h = LINES;
    if (ww > COLS) ww = COLS;
    if (ww < 60)  ww = (COLS < 60) ? COLS : 60;
    state->tx.tx_compose_win = newwin(h, ww, (LINES - h) / 2, (COLS - ww) / 2);
    if (!state->tx.tx_compose_win) return;
    keypad(state->tx.tx_compose_win, TRUE);
    nodelay(state->tx.tx_compose_win, TRUE);
    tx_compose_init(state, &state->tx.tx_compose);
#ifdef SSO_WITH_SDR
    // RX-only SDR (e.g. RTL-SDR): the burst can never reach the air, so
    // keep the allow-tx gate forced off. Compose + preview still work
    // (commit refuses with a clear message).
    if (state->sdr.rx_session != NULL && !rx_session_can_tx(state->sdr.rx_session)) {
        state->tx.tx_compose.allow_tx = 0;
    }
#endif
    tx_compose_draw(state, state->tx.tx_compose_win, &state->tx.tx_compose);
    state->tx.tx_compose_last_edit_ns = ts_now_ns();
    state->tx.tx_compose_active = 1;
}

// Tear the modal down. Touchwin + refresh paints stdscr's cells back
// into the area the modal occupied so the operator's normal panels
// become visible again.
void tx_compose_close(tx_t *tx) {
    if (tx->tx_compose_win) {
        delwin(tx->tx_compose_win);
        tx->tx_compose_win = NULL;
    }
    tx->tx_compose_active = 0;
    touchwin(stdscr);
    refresh();
}

// Consume one key (from stdscr's getch, which the main loop is doing).
// Returns 1 to keep the modal open, 0 when the operator's Enter or
// Esc closed it — the caller invokes tx_compose_close() in that case.
int tx_compose_handle_key(state_t *state, int key) {
    if (!state->tx.tx_compose_active) return 0;
    if (key == ERR) return 1;
    tx_compose_t *c = &state->tx.tx_compose;
    WINDOW *w = state->tx.tx_compose_win;
    int changed = 1;
    // Esc may be a bare cancel OR the start of a CSI sequence (arrow
    // keys, Home/End, Delete) when keypad mode can't translate them.
    if (key == 27) {
        int translated = tx_drain_csi(w);
        if (translated >= 0) {
            key = translated;
        } else {
            tx_compose_remember(state, c);
            return 0;
        }
    }
    if (key == '\n' || key == '\r' || key == KEY_ENTER) {
        char err[120];
        if (tx_compose_validate(c, err, sizeof err) != 0) {
            snprintf(c->status_msg, sizeof c->status_msg,
                     "rejected: %.*s",
                     (int)(sizeof c->status_msg - 16), err);
        } else if (tx_compose_commit(state, c, err, sizeof err) != 0) {
            snprintf(c->status_msg, sizeof c->status_msg,
                     "commit failed: %.*s",
                     (int)(sizeof c->status_msg - 20), err);
        } else {
            tx_history_push(state, c->payload);
            tx_compose_remember(state, c);
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
    } else if (key == 23 /* Ctrl-W */) {
        tx_field_kill_word_back(c);
    } else if (key == KEY_LEFT) {
        tx_field_left(c);
    } else if (key == KEY_RIGHT) {
        tx_field_right(c);
    } else if (key == KEY_HOME || key == 1 /* Ctrl-A */) {
        tx_field_home(c);
    } else if (key == KEY_END  || key == 5 /* Ctrl-E */) {
        tx_field_end(c);
    } else if (key == KEY_UP) {
        tx_history_recall(state, c, -1);
    } else if (key == KEY_DOWN) {
        tx_history_recall(state, c, +1);
    } else if (key == ' ' && tx_field_is_toggle(c->focus)) {
        tx_field_toggle(c);
    } else if (key >= 32 && key < 127) {
        tx_field_insert(c, key);
    } else {
        changed = 0;
    }
    if (changed) {
        state->tx.tx_compose_last_edit_ns = ts_now_ns();
        tx_compose_draw(state, w, c);
    }
    return 1;
}

// Per-tick housekeeping. Pumps the debounced preview broadcast and
// re-renders if the broadcast fired (so the mirror line refreshes).
// Called every main-loop iteration when active.
void tx_compose_pump(state_t *state) {
    if (!state->tx.tx_compose_active) return;
    tx_compose_t *c = &state->tx.tx_compose;
    if (c->preview_dirty
        && (ts_now_ns() - state->tx.tx_compose_last_edit_ns) >= g_tx_compose_debounce_ns) {
        tx_compose_broadcast_preview(state, c);
        c->preview_dirty = 0;
        tx_compose_draw(state, state->tx.tx_compose_win, c);
    }
}
