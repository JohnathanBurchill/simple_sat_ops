/*

   Simple Satellite Operations  rotator_calibrate.h

   Calibration routine for the SPID Rot2Prog (or any constant-rate
   rotator the async worker drives). Sweeps the antenna across known
   arcs and times each leg via the async snapshot to measure deg/s on
   each axis. On success, persists the rates with
   pursuit_save_rotator_rates() so the next normal startup picks them
   up for the pursuit planner.

   Operator-only: this function physically moves the antenna. The
   caller is responsible for confirming with the operator before
   invoking it.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
*/

#ifndef ROTATOR_CALIBRATE_H
#define ROTATOR_CALIBRATE_H

#include "antenna_rotator_async.h"

#include <stdio.h>

typedef enum {
    ROTATOR_CALIBRATE_OK = 0,
    ROTATOR_CALIBRATE_NO_INITIAL_STATUS, // never got an OK STATUS reply
    ROTATOR_CALIBRATE_PARK_TIMEOUT,      // initial park leg stalled
    ROTATOR_CALIBRATE_AZ_TIMEOUT,        // az sweep stalled
    ROTATOR_CALIBRATE_EL_TIMEOUT,        // el sweep stalled
    ROTATOR_CALIBRATE_BAD_RATE,          // measured rate was <=0 or nan
    ROTATOR_CALIBRATE_SAVE_FAILED,       // I/O error writing rate files
    ROTATOR_CALIBRATE_NO_WORKER,         // async handle was NULL
} rotator_calibrate_result_t;

const char *rotator_calibrate_result_name(rotator_calibrate_result_t r);

// Run the calibration sequence on the async worker:
//   1. Park to (az=AR_CAL_PARK_AZ, el=AR_CAL_PARK_EL) and wait for
//      arrival (or PARK_TIMEOUT).
//   2. Sweep az by AR_CAL_AZ_ARC degrees; time it via snapshots.
//   3. Sweep el by AR_CAL_EL_ARC degrees; time it via snapshots.
//   4. Return to the park position.
//   5. Persist deg/s rates to disk via pursuit_save_rotator_rates().
//
// On success, returns ROTATOR_CALIBRATE_OK and writes the measured
// rates to *out_az_dps / *out_el_dps (caller can ignore if NULL).
// On any leg timing out (no convergence inside the per-leg budget),
// returns the matching _TIMEOUT code without writing rates to disk.
// `log`, if non-NULL, receives one progress line per leg.
rotator_calibrate_result_t rotator_calibrate_run(
    antenna_rotator_async_t *async,
    double *out_az_dps,
    double *out_el_dps,
    FILE *log);

#endif // ROTATOR_CALIBRATE_H
