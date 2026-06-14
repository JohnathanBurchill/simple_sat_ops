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
#include "telemetry.h"
#include "tr_switch.h"

#include <stdint.h>
#include <stdio.h>
#include <termios.h>

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
} state_t;


#endif // STATE_H
