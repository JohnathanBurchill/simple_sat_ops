/*

   Simple Satellite Operations  ui/panels.h

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

// ncurses panel rendering for the operator + viewer UIs. The render_*
// helpers are pure (caller supplies the data) so the operator (reading live
// hardware) and the viewer (filling from an IPC broadcast) paint identical
// panels. The rx_panel_data_t / status_panel_t snapshots are the contract
// between the two sides.

#ifndef UI_PANELS_H
#define UI_PANELS_H

#include "state.h"

#include <ncurses.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef SSO_WITH_SDR
#include "rx_session.h"     // RX_PT_COUNT, RX_LAST_PAYLOAD_MAX, RX_LAST_SUMMARY_MAX
#endif

#ifdef __cplusplus
extern "C" {
#endif

// RX_PT_COUNT is gated by WITH_USRP_B210 (it lives in rx_session.h). On
// builds without B210 we still want the viewer to draw the panel from
// broadcast data, so define a fallback so the struct compiles everywhere.
#ifdef SSO_WITH_SDR
#define RX_PANEL_PT_COUNT RX_PT_COUNT
#define RX_PANEL_PAYLOAD_MAX RX_LAST_PAYLOAD_MAX
#define RX_PANEL_SUMMARY_MAX RX_LAST_SUMMARY_MAX
#else
#define RX_PANEL_PT_COUNT 6
#define RX_PANEL_PAYLOAD_MAX 64
#define RX_PANEL_SUMMARY_MAX 160
#endif

// Snapshot of the operator's live RX panel data, self-contained so the same
// renderer can be driven by either the operator (reading rx_session +
// state->ribbon_*) or the viewer (filling it from a STATE event). ribbon is a
// nul-terminated string of glyph-index chars (' ' or '0'..'7'); empty when no
// samples have arrived yet.
typedef struct {
    int        have_session;
    int        rec_active;
    char       sdr_name[32];       // active SDR (operator-side; "" for viewers)
    int        can_tx;             // 0 => RX-only backend (operator-side)
    double     rx_freq_hz;         // effective Doppler-shifted carrier
    double     rx_lo_hz;           // hardware SDR LO (without the
                                   // intentional LO offset added back)
    double     rx_bandwidth_hz;    // post-decim sample rate (BW = ±half)
    double     peak_dbfs;
    double     rms_dbfs;
    uint64_t   frames_total;
    char       last_frame_summary[80]; // "<ts>  N bytes" or empty
    double     age_s;                  // <0 = no frame yet
    uint64_t   pt_count[RX_PANEL_PT_COUNT];
    int        pt_payload_len[RX_PANEL_PT_COUNT];
    uint8_t    pt_payload[RX_PANEL_PT_COUNT][RX_PANEL_PAYLOAD_MAX];
    // One-line decoded summary built by rx_session when the payload
    // sniffs as a known FrontierSat packet type. Empty = no decode
    // available; render falls back to the hex preview above.
    char       pt_summary[RX_PANEL_PT_COUNT][RX_PANEL_SUMMARY_MAX];
    int        ribbon_n;
    char       ribbon[RIBBON_LEN + 1];
    // Parallel array: peak dBFS for the i-th second back. Clamped into
    // int8 (dBFS is naturally -90..0, well inside int8's range).
    int8_t     ribbon_peak[RIBBON_LEN];
    // frames_total above is the live IQ-domain chain (the one that
    // writes the DB + drives the per-type panel). The two fields
    // below are the shadow counts from the PCM/FM-audio and Viterbi
    // MLSE chains running in parallel on the same window — pure A/B
    // diagnostics so the operator can spot a chain regression.
    uint64_t   frames_pcm;
    uint64_t   frames_vit;
    // Optional warning row (e.g., low-disk). Empty when no warning.
    char       warning[80];
} rx_panel_data_t;

// Operator/carrier/rotator status block. Caller supplies the values so this
// works for both the operator (reading the rotator from hardware) and the
// viewer (pulling them from the IPC broadcast).
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
    // HMAC keyfile display. Only the operator process fills these; the
    // viewer leaves status == HMAC_DISPLAY_UNSET so the row is skipped.
    const char           *hmac_path;
    hmac_display_status_t hmac_status;
    ssize_t               hmac_bytes;
    // T/R antenna switch. Operator-only: the viewer leaves tr_show = 0
    // so the block is skipped (the switch status isn't broadcast).
    int         tr_show;
    int         tr_connected;
    int         tr_stale;
    const char *tr_device;
    const char *tr_state;        // "RX" / "TX" / "?"
    const char *tr_mode;         // "AUTO" / "FORCE_TX" / ...
    double      tr_last_tx_ago_s; // NAN or +inf -> placeholder
} status_panel_t;

// Per-tick data updaters (operator-side).
void ribbon_push(ui_t *ui, double peak_dbfs, int bright_bins);
void low_disk_refresh(op_t *op, double t_now);

// Snapshot the operator's live RX state into d for rendering / broadcast.
void rx_panel_collect_local(state_t *state, rx_panel_data_t *d);

// Pure renderers (operator + viewer).
void render_rx_panel(const rx_panel_data_t *d, int *print_row, int print_col);
void render_ribbon_vertical(const rx_panel_data_t *d, int top_row, int bot_row, int col);
void render_tx_log_panel(const tx_t *tx, int start_row, int col);
void render_predictions_panel(state_t *state, double jul_utc, int *print_row, int print_col);
void render_status_panel(const status_panel_t *p, int *print_row, int print_col);

// Operator-side: run the week-ahead pass search into track->prediction.
void compute_predictions(track_t *track, double jul_utc);

// Operator report wrappers (compute + render).
void report_predictions(state_t *state, double jul_utc, int *print_row, int print_col);
void report_status(state_t *state, int *print_row, int print_col);
void report_position(track_t *track, int *print_row, int print_col);

// Paint the operator's whole-screen layout for one redraw tick (predictions /
// status / position, low-disk refresh, RX panel, right-edge ribbon, the
// keyboard legend, and the TX log). The legend hangs off the actual bottom of
// the left status column, so it sizes itself each tick.
void render_operator_screen(state_t *state, double jul_utc, double t_now);

// Plain-ASCII modal box drawer (locale-bulletproof, unlike ncurses box()).
// Shared by the TX-compose and auto-tcmd modals.
void draw_box(WINDOW *w);

// CSI fallback parser for terminals where ncurses' keypad mode doesn't
// translate arrow/nav keys into KEY_* (some tmux configs). Returns a KEY_*
// code, or -1 if the lookahead isn't a recognised CSI. Shared by the modals.
int tx_drain_csi(WINDOW *w);

#ifdef __cplusplus
}
#endif

#endif // UI_PANELS_H
