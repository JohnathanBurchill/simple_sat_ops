/*

   Simple Satellite Operations  src/tx_state.h

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

// Uplink slice of state_t: HMAC key, TX-compose modal, auto-telecommand
// modal, the burst-request slot, and the TX log. See state.h.

#ifndef TX_STATE_H
#define TX_STATE_H

#include "sso_ipc.h"     // sso_event_type_t, SSO_TX_TEXT_MAX
#include "tx_burst.h"    // tx_request_slot_t

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

// HMAC keyfile status, resolved once at startup so the operator banner can
// show "(N bytes ok)" / "(missing)" / "(bad)". The CTS1 flight firmware
// expects HMAC on every uplink — if the keyfile is missing or won't parse,
// TX is REFUSED rather than sending unsigned frames the satellite drops.
typedef enum {
    HMAC_DISPLAY_UNSET   = 0,
    HMAC_DISPLAY_OK      = 1,
    HMAC_DISPLAY_MISSING = 2,
    HMAC_DISPLAY_BAD     = 3,
} hmac_display_status_t;

#define TX_HISTORY_MAX 32

// TX compose modal. payload (ASCII telecommand), a TX-power-in-dB field,
// and the --allow-tx checkbox. CSP fields, freq, repeat/gap and the
// secondary allow-flags are hard-coded to FrontierSat defaults at fill.
typedef enum {
    TXF_PAYLOAD = 0,
    TXF_POWER,
    TXF_ALLOW_TX,
    TXF_COUNT,
} tx_field_t;

typedef struct tx_compose {
    // Big enough to type a full RF telecommand. tx_field_insert caps the
    // payload at TCMD_RF_MAX_LEN (215) chars — the over-the-air limit —
    // so the extra room here is headroom, not a typeable length.
    char payload[256];
    char power[12];           // TX power in dB
    int  allow_tx;
    tx_field_t focus;
    int  preview_dirty;
    struct timespec last_edit;
    char status_msg[160];
    // Per-field text cursor (only meaningful for the text fields —
    // payload, power). 0..strlen(buf).
    int  cursors[TXF_COUNT];
    // Payload-history navigation. history_idx == -1 means "editing the
    // current draft"; 0..N-1 points at tx.tx_history[i] (newest at 0).
    int  history_idx;
    char history_saved_edit[256];
} tx_compose_t;

// Auto-TCMD modal: drives a file of ASCII telecommands through the TX
// path automatically (--tc-file). Ticked alongside the main loop.
typedef enum {
    AUTO_F_POWER = 0,
    AUTO_F_REPEATS,
    AUTO_F_INTERVAL,
    AUTO_F_ALLOW_TX,
    AUTO_F_COUNT,
} auto_tcmd_field_t;

typedef enum {
    AUTO_STATE_SETUP = 0,
    AUTO_STATE_RUNNING,
    AUTO_STATE_STOPPED,    // user stopped, file not exhausted
    AUTO_STATE_DONE,       // file exhausted
    AUTO_STATE_PASS_OVER,  // LOS hit while running
    AUTO_STATE_PAUSE_PROMPT,   // Esc during a run: pause or abort?
    AUTO_STATE_PAUSED,         // parked across the modal closing; 'A' resumes
    AUTO_STATE_RESUME_PROMPT,  // 'A' on a parked run: resume or restart?
} auto_tcmd_state_t;

typedef struct auto_tcmd {
    // Commands loaded from --tc-file. commands[i] is one CTS1+ line,
    // trimmed; comment lines (#...) and blank lines are dropped at load.
    char **commands;
    int    n_commands;
    char   file_path[256];
    // Error count from re-linting the file on the last (re)load. The startup
    // lint gate runs once; this catches a --tc-file edited after launch.
    // Non-zero blocks the run (auto_tcmd_start) unless --ignore-...-tc-errors.
    int    lint_errors;

    // Editable fields (text-edit semantics shared with TX compose).
    char power[12];
    char repeats[8];
    // Minimum spacing between the START of consecutive sends, in seconds.
    // The next send fires at max(send_start + interval, burst_done) — the
    // interval overlaps the burst rather than adding to it. Clamped to a
    // floor at run start (AUTO_TCMD_MIN_INTERVAL_S).
    char interval_s[12];
    int  allow_tx;
    auto_tcmd_field_t focus;
    int               cursors[AUTO_F_COUNT];

    // Run state.
    auto_tcmd_state_t state;
    int    cmd_idx;        // index into commands[]
    int    repeat_idx;     // how many sends of commands[cmd_idx] so far
    int    repeats_total;  // parsed from repeats at start
    double interval_s_val; // parsed from interval_s at start (after floor clamp)
    long   next_send_ns;
    long   start_ns;       // wall-clock at run start, for elapsed TX time
    long   pause_ns;       // ts_now_ns() when paused; shifts start_ns on resume
    int    sends_total;    // running tally — every queued burst
    // On-air seconds accumulated and total (AX100/9600/preroll math).
    double tx_seconds_spent;
    double tx_seconds_total;
    char   last_sent[SSO_TX_TEXT_MAX];   // full command text, not clipped
    char   status_msg[160];
} auto_tcmd_t;

// TX log ring buffer entry — one PREVIEW / TX_COMMAND_SENT / TX_NOT_SENT
// event for the operator + viewer TX panels. ascii matches the upstream
// sso_event_t.ascii field (SSO_TX_TEXT_MAX) so the panel renders the full
// command text instead of a truncated preview.
typedef struct {
    sso_event_type_t kind;     // PREVIEW | TX_COMMAND_SENT | TX_NOT_SENT
    char             ts[16];   // HH:MM:SS
    char             ascii[SSO_TX_TEXT_MAX];
    char             tx_not_sent_reason[24];
    char             source[16];  // "auto-cmd (file)" | "manual send"; empty on previews
} tx_log_entry_t;
#define TX_LOG_SIZE 8

typedef struct tx {
    // HMAC keyfile resolved once at startup: path, parse status, and the
    // key bytes used to sign every TX burst's AX100 frame.
    char                  hmac_keyfile_path[512];
    hmac_display_status_t hmac_display_status;
    uint8_t               hmac_key[64];
    size_t                hmac_key_len;

    // --ignore-at-your-peril-all-tc-errors: let a known-bad --tc-file agenda
    // start anyway. --tx-dry-run records each command as not-sent
    // (reason "dry-run") instead of keying the PA.
    int ignore_tc_errors;
    int tx_dry_run;

    // TX compose modal + payload history. tx_last_* survive Esc/commit so
    // a reopened modal picks up the previous draft (seeded "CTS1+" once).
    char         tx_last_payload[256];
    char         tx_last_power[12];
    int          tx_last_allow_tx;
    char         tx_history[TX_HISTORY_MAX][256];  // newest at index 0
    int          tx_history_count;
    int          tx_compose_active;
    void        *tx_compose_win;          // WINDOW* (ncurses kept out of this header)
    tx_compose_t tx_compose;
    long         tx_compose_last_edit_ns;

    // Auto-TCMD modal (--tc-file). file_path is captured from the CLI and
    // read lazily when the modal opens.
    int          auto_tcmd_active;
    void        *auto_tcmd_win;           // WINDOW*
    auto_tcmd_t  auto_tcmd;
    char         auto_tcmd_file_path[512];

    // TX burst path (SDR side). tx_request is the slot tx_compose_commit /
    // auto_tcmd_tick stage a burst into; the main loop submits it to the
    // rx_session worker and clears pending. tx_inflight gates submit-vs-poll
    // so the loop stays responsive while the worker pauses RX, transmits,
    // and resumes RX.
    tx_request_slot_t tx_request;
    int               tx_inflight;
    // End-to-end burst timing. tx_burst_submit_ns is stamped (ts_now_ns) the
    // moment a burst is handed to the worker; last_burst_wall_s is the measured
    // submit->done wall-clock of the most recent burst (RX pause + UHD start
    // lead + on-air frame + RX resume), or -1 before any burst has completed.
    // Surfaced in the auto-tcmd modal + the tx-result audit so the operator can
    // read the real per-burst floor on hardware.
    long              tx_burst_submit_ns;
    double            last_burst_wall_s;
    // Doppler-corrected uplink carrier (Hz), refreshed each tick from the
    // range rate; snapshotted by compose-preview / commit / auto-tcmd so a
    // burst is keyed where the satellite hears the nominal carrier. Seeded
    // to the bare nominal in main() before SGP4 has a valid range rate.
    long              tx_freq_hz_doppler;
    // @tssent dedup key for SSO+ expansions: startup UTC truncated to the
    // minute, pinned once in main() so the flight firmware runs an SSO+
    // time-sync once per pass. See sso_pseudo.h.
    long long         sso_pass_tssent_ms;
    // --no-tx blocks the compose modal from keying the PA; typing/preview
    // still work so the operator can rehearse.
    int               no_tx;
    // Modulated 0xAA carrier prepended to every burst (ms). Stamped into
    // tx_request.preroll_ms and read for the on-air estimate. Default 200,
    // overridable via --tx-preroll-ms.
    int               tx_preroll_ms;
    // TX log ring buffer — last few PREVIEW/SENT/NOT_SENT events for the
    // operator + viewer panels. Plus a persistent JSONL on-disk log opened
    // lazily after pass_folder is set and fflushed per line.
    tx_log_entry_t    tx_log[TX_LOG_SIZE];
    size_t            tx_log_count;
    FILE             *tx_log_fp;
    char              tx_log_path[512];
} tx_t;

#endif // TX_STATE_H
