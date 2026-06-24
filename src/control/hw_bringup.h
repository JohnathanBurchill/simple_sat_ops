/*

   Simple Satellite Operations  control/hw_bringup.h

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

// Hardware bring-up steps run during simple_sat_ops startup, before the
// ncurses screen opens: the antenna rotator (open + async worker + position
// seed), the one-shot rotator-rate calibration mode, the saved slew-rate
// load, the T/R antenna switch, and the SDR (B2xx / RTL-SDR) RX/TX chain.
// Each writes its result into state_t; main() calls them in sequence.

#ifndef CONTROL_HW_BRINGUP_H
#define CONTROL_HW_BRINGUP_H

#include "trsw_state.h"

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Open the antenna rotator and start its async worker, then seed the
// unwrapped-position accumulator from the controller's current STATUS.
// Returns 0 to continue startup; EXIT_FAILURE if the rotator is required but
// can't be opened — except under --self-test, where a missing rotator is
// reported and startup continues (returns 0) so the dry-run report can still
// print. A no-op (returns 0) when --without-rotator was passed.
int hw_rotator_open(state_t *state);

// --calibrate-rotator: drive the antenna across known arcs to measure deg/s
// on each axis, save the rates, close the rotator, and return the process
// exit code (0 on success, 1 otherwise). Call only when
// state->calibrate_rotator is set; main() returns this value directly.
int hw_rotator_calibrate(state_t *state);

// Load the saved rotator slew rates into state->pursuit_az_dps/el_dps for the
// lead-aim planner. Missing/invalid calibration leaves the planner disabled
// and prints a hint. No-op when the rotator isn't open.
void hw_pursuit_rates_load(state_t *state);

// Auto-probe the T/R antenna switch. Absent or inaccessible hardware is a
// one-line warning, not an error. No-op when --without-tr-switch was passed.
void hw_tr_switch_open(trsw_t *trsw);

// Open the SDR (B2xx via UHD, or RTL-SDR) and hand the core to rx_session for
// the RX + decode chain. Operator (--control) mode only, and soft-fail: any
// device error leaves the session running rotator + UI only. A no-op in a
// build without SDR support. Recording is not started under --self-test.
void hw_sdr_open(state_t *state);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_HW_BRINGUP_H
