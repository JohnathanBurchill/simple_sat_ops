/*

   Simple Satellite Operations  src/rot_state.h

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

// Antenna-rotator + pursuit-planner slice of state_t — see state.h.

#ifndef ROT_STATE_H
#define ROT_STATE_H

#include "antenna_rotator.h"
#include "pursuit.h"

#include <stddef.h>

// The async rotator wrapper: rot_t keeps an opaque handle, so this header
// stays free of antenna_rotator_async.h.
typedef struct antenna_rotator_async antenna_rotator_async_t;

// Pre-sampled mech-frame satellite trajectory backing the pursuit
// planner's sat_sample_fn_t callback. Sampled once at plan-build time so
// the planner's iterations see a consistent, order-independent function
// without ever mutating prediction.satellite_ephem on the side. Owned
// alongside pursuit_plan; both live for the current tracking session and
// are freed together at LOS / 's' / shutdown.
typedef struct {
    double *t_jul;
    double *az_unwrapped;
    double *el;
    size_t  n;
} pursuit_track_t;

typedef struct rot {
    antenna_rotator_t antenna_rotator;
    int run_with_antenna_rotator;
    int have_antenna_rotator;

    // Async wrapper around antenna_rotator's serial I/O. NULL if no rotator
    // (--without-rotator, or antenna_rotator_init failed). Spawned right
    // after antenna_rotator_init succeeds; joined on shutdown. While set, no
    // other thread touches antenna_rotator's serial FD.
    antenna_rotator_async_t *rot_async;
    // --calibrate-rotator runs the calibration routine after opening the
    // rotator and exits without entering the UI; --confirm-rotator-calibrate
    // is the safety interlock so the physical motion is always deliberate.
    int calibrate_rotator;
    int confirm_rotator_calibrate;
    // Rotator slew rates loaded from disk at startup (deg/s). Either both
    // > 0 (calibration present, pursuit planner can run) or both 0
    // (calibration absent, the track loop falls back to "aim where the
    // satellite is now"). without_rotator_pursuit (--without-rotator-pursuit)
    // disables the planner without removing the calibration files.
    double pursuit_az_dps;
    double pursuit_el_dps;
    int    without_rotator_pursuit;
    // Pursuit planner outputs for the current tracking session, freed
    // together at LOS / 's' / shutdown (see pursuit_track_t above).
    pursuit_plan_t  pursuit_plan;
    pursuit_track_t pursuit_track;
} rot_t;

#endif // ROT_STATE_H
