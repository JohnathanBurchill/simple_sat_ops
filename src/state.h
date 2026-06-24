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

#include "rot_state.h"
#include "sdr_state.h"
#include "track_state.h"
#include "sso_ipc.h"
#include "telemetry.h"
#include "trsw_state.h"
#include "tx_state.h"
#include "ui_state.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>

// Latest broadcast snapshot, kept so a newly-connecting viewer gets state
// in its WELCOME response without waiting up to 500 ms for the next
// periodic STATE broadcast. ipc_broadcast_state fills it after each send;
// ipc_on_event reads it to seed a WELCOME.
typedef struct last_state {
    int    valid;
    char   sat[64];
    double az;
    double el;
    long   freq_hz;
    double doppler;
    char   tle[256];
    double tgt_az;
    double tgt_el;
    int    flip;
    int    in_pass;
    int    tracking;
    int    has_rot;
    double jul;
    char   idesg[9];
    double epoch_min;
    double min_visible;
    double min_above_0;
    double min_above_30;
    double max_el;
    double pred_az;
    double pred_el;
    double alt_km;
    double lat_deg;
    double lon_deg;
    double speed_kms;
    double range_km;
    double rrate_kms;
} last_state_t;

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

    // Run mode + one-shot CLI flags (set once in apply_args / main).
    int control_mode;        // --control: this process is the operator
    int viewer_mode;         // bare invocation found a running operator
    int viewer_stream;       // --viewer-stream: headless JSON stream to stdout
    int self_test;           // --self-test: print a report and exit
    int testing_mode;        // --testing

    const char       *operator_user;   // Unix user running the operator
    sso_ipc_server_t *ipc;             // IPC fan-out server (operator side)
    last_state_t      last_state;      // last broadcast snapshot (WELCOME seed)
    char   pass_folder[256];           // this pass's output folder; "" until set
    char   low_disk_msg[80];           // non-empty -> low-disk warning to show
    double low_disk_last_t;            // last low-disk probe (monotonic s)

    // Orbit prediction + Doppler frequencies + tracked-satellite identity.
    // See track_state.h.
    track_t track;

    // SDR backend selection + the live RX session + RX config. See sdr_state.h.
    sdr_t sdr;

    // Operator-UI extras: signal ribbon, spectrogram job, live-waterfall
    // flag. See ui_state.h.
    ui_t ui;

    // Antenna rotator + pursuit planner. See rot_state.h.
    rot_t rot;

    // T/R antenna switch (USB-CDC). See trsw_state.h.
    trsw_t trsw;

    // Telemetry overlay (still rendered alongside the prediction).
    telemetry_t telemetry;

    // Sky-scan grid + per-target CSV logging (--scan-sky).
    scan_sky_t scan;

    // Bottom-of-screen ":" command line.
    cmdline_t cmd;

    // Uplink: HMAC key, TX-compose + auto-telecommand modals, the burst
    // request slot, and the TX log. See tx_state.h.
    tx_t tx;
} state_t;


#endif // STATE_H
