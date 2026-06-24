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

#include "app_state.h"
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

// The single shared context, now a composition of per-subsystem sub-structs
// (each defined in its own <name>_state.h). A function takes a pointer to
// just the part it needs — &state.rot, &state.tx, ... — instead of the whole
// state_t, so its reach is bounded to that subsystem; only the orchestrators
// (the main loop, apply_args, the colon-command dispatcher, the renderers)
// still take state_t* because they genuinely coordinate across subsystems.
typedef struct state
{
    // Process run-mode flags (running, verbosity, --control / --self-test /
    // ...). See app_state.h.
    app_t app;

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
