/*

    Simple Satellite Operations  state.h

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

#ifndef STATE_H
#define STATE_H

#include "antenna_rotator.h"
#include "prediction.h"
#include "sso_ipc.h"
#include "telemetry.h"
#include "tr_switch.h"

#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>

#define SCAN_MAX_TARGETS 512

// --scan-sky: drive the rotator through a grid of (az, el) targets spaced
// for roughly equal solid angle, dwelling at each while writing per-target
// arrival timestamps to a CSV. Owned by the operator loop.
typedef struct { double az_deg; double el_deg; } scan_target_t;
typedef struct scan_sky
{
    int           mode;            // CLI: --scan-sky rebinds T to a sky scan
    double        step_deg;        // elevation ring spacing (deg)
    scan_target_t targets[SCAN_MAX_TARGETS];
    int           n_targets;
    int           active;
    int           idx;
    // Set to t_now when the rotator's motion-flag clears at a target; the
    // dwell expires SCAN_DWELL_S later. 0 means "haven't arrived yet".
    double        dwell_start_s;
    FILE         *csv_fp;
    char          csv_path[640];
} scan_sky_t;

#define CMD_BUF_SIZE 128
#define CMD_HISTORY_SIZE 64

// Bottom-of-screen ":" command line (vi-style). While active, every key
// is routed through the command handler instead of the main key switch.
typedef struct cmdline
{
    int  active;
    char buf[CMD_BUF_SIZE];
    int  len;
    int  cursor;          // 0..len; insert position
    char status[160];
    // Preview debounce: dirty is set on every edit; the main loop
    // broadcasts a preview event once the buffer has been idle long enough.
    int  dirty;
    long last_edit_ns;
    // History: Up/Down cycle previously executed commands. The line being
    // edited is stashed on the first Up so Down can return to it.
    char history[CMD_HISTORY_SIZE][CMD_BUF_SIZE];
    int  history_count;   // entries in use (capped at SIZE)
    int  hist_pos;        // 0..count; ==count -> editing line
    char hist_saved[CMD_BUF_SIZE];  // editing line stash
} cmdline_t;

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
    // current draft"; 0..N-1 points at state.tx_history[i] (newest at 0).
    int  history_idx;
    char history_saved_edit[256];
} tx_compose_t;

// Auto-TCMD modal: drives a file of ASCII telecommands through the TX
// path automatically (--tc-file). Ticked alongside the main loop.
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

typedef struct auto_tcmd {
    // Commands loaded from --tc-file. commands[i] is one CTS1+ line,
    // trimmed; comment lines (#...) and blank lines are dropped at load.
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
    long   start_ns;       // wall-clock at run start, for elapsed TX time
    int    sends_total;    // running tally — every queued burst
    // On-air seconds accumulated and total (AX100/9600/preroll math).
    double tx_seconds_spent;
    double tx_seconds_total;
    char   last_sent[SSO_TX_TEXT_MAX];   // full command text, not clipped
    char   status_msg[160];
} auto_tcmd_t;

#define MAX_TLE_LINE_LENGTH 128
#define TRACKING_PREP_TIME_MINUTES 5.0

// SDR-only state. The CAT-radio + ALSA-audio fields are gone; the B210
// is now driven externally by b210_rx_tx / tx_frame_sdr, which talk
// to simple_sat_ops over the sso_ipc socket for operator coordination.
// What remains here is the satellite tracker, the rotator, and the
// nominal/Doppler-shifted frequency outputs computed for display + IPC
// broadcast.
typedef struct state
{
    // General
    int n_options;
    int running;
    int verbose_level;
    int in_pass;

    // Run mode + one-shot CLI flags (set once in apply_args / main).
    int control_mode;        // --control: this process is the operator
    int viewer_mode;         // bare invocation found a running operator
    int self_test;           // --self-test: print a report and exit
    int ignore_tc_errors;    // --ignore-...-all-tc-errors
    int tx_dry_run;          // --tx-dry-run
    int run_live_waterfall;  // --live-waterfall
    int always_record;       // --always-record
    int testing_mode;        // --testing

    // Tracking
    int satellite_tracking;
    prediction_t prediction;

    // Nominal + Doppler-corrected frequencies (Hz). Computed each tick
    // by main.c from the prediction's range-rate. simple_sat_ops no
    // longer drives a radio directly — these are display-only and get
    // published into the IPC state events for any subscriber
    // (tx_frame_sdr can pick up the current uplink freq this way).
    double nominal_uplink_frequency_hz;
    double nominal_downlink_frequency_hz;
    double doppler_uplink_frequency_hz;
    double doppler_downlink_frequency_hz;
    int doppler_correction_enabled;

    // SDR LO offset (Hz) below the nominal downlink carrier. Without this,
    // the corrected signal sits at DC after software Doppler tracking and
    // gets eaten by the B210's DC blocker whenever Doppler crosses zero
    // (i.e. exactly at TCA, the worst possible time). Offsetting the LO
    // by ~25 kHz parks the signal in a clean part of the captured 96 kHz
    // baseband for the whole pass. Configurable via --lo-offset=<kHz>.
    double rx_lo_offset_hz;

    // AD9361 RX gain (dB) — fed to b210_rx_tx_core at session open.
    // Configurable via --rx-gain=<dB>; runtime adjustment is via the
    // :gain colon command in the operator UI.
    double rx_gain_db;

    // AD9361 background tracking loops. Default off — these add a
    // ~51 Hz comb of impulsive spikes to the captured IQ at mid gain
    // settings (see b210_rx_tx_core.h). Configurable via
    // --ad9361-dc-track=on|off and --ad9361-iq-track=on|off so the
    // operator can A/B against the default-on UHD baseline.
    int rx_dc_offset_track;
    int rx_iq_balance_track;

    // Antenna rotator
    antenna_rotator_t antenna_rotator;
    int run_with_antenna_rotator;
    int have_antenna_rotator;

    // T/R antenna switch (USB-CDC, default /dev/ttyACM0).
    // run_with_tr_switch defaults to 1: auto-probe the device on start.
    // Absent hardware is a one-line warning, not an error.
    tr_switch_t tr_switch;
    int run_with_tr_switch;
    int have_tr_switch;

    // Telemetry overlay (still rendered alongside the prediction).
    telemetry_t telemetry;

    // Sky-scan grid + per-target CSV logging (--scan-sky).
    scan_sky_t scan;

    // Bottom-of-screen ":" command line.
    cmdline_t cmd;

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
} state_t;


#endif // STATE_H
