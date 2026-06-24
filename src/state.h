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

#include "op_state.h"
#include "rot_state.h"
#include "sdr_state.h"
#include "track_state.h"
#include "telemetry.h"
#include "trsw_state.h"
#include "tx_state.h"
#include "ui_state.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>

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

    // Operator: IPC fan-out server + user, last broadcast snapshot, pass
    // output folder, low-disk warning. See op_state.h.
    op_t op;

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
