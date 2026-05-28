/*

   Simple Satellite Operations  rotator_calibrate.c

   See rotator_calibrate.h. Drives the antenna across known arcs and
   times each leg by polling the async worker's snapshot.

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

#include "rotator_calibrate.h"
#include "pursuit.h"

#include <math.h>
#include <time.h>
#include <unistd.h>

// Park position for the sweep, and the arc lengths. The arcs are
// chosen short enough that even a slow rotator finishes inside the
// per-leg timeout, but long enough that fixed startup latency
// (~5-10 ms) doesn't dominate the timing.
#define AR_CAL_PARK_AZ           0.0
#define AR_CAL_PARK_EL          10.0
#define AR_CAL_AZ_ARC           90.0   // sweep az from PARK_AZ to PARK_AZ + 90
#define AR_CAL_EL_ARC           45.0   // sweep el from PARK_EL to PARK_EL + 45
#define AR_CAL_PARK_TOL_DEG      1.0
#define AR_CAL_ARRIVAL_TOL_DEG   0.5
#define AR_CAL_PARK_TIMEOUT_S    30.0
#define AR_CAL_SWEEP_TIMEOUT_S   60.0
#define AR_CAL_POLL_INTERVAL_S    0.05

const char *rotator_calibrate_result_name(rotator_calibrate_result_t r)
{
    switch (r) {
    case ROTATOR_CALIBRATE_OK:                  return "ok";
    case ROTATOR_CALIBRATE_NO_INITIAL_STATUS:   return "no_initial_status";
    case ROTATOR_CALIBRATE_PARK_TIMEOUT:        return "park_timeout";
    case ROTATOR_CALIBRATE_AZ_TIMEOUT:          return "az_timeout";
    case ROTATOR_CALIBRATE_EL_TIMEOUT:          return "el_timeout";
    case ROTATOR_CALIBRATE_BAD_RATE:            return "bad_rate";
    case ROTATOR_CALIBRATE_SAVE_FAILED:         return "save_failed";
    case ROTATOR_CALIBRATE_NO_WORKER:           return "no_worker";
    }
    return "?";
}

static double now_mono_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double) ts.tv_sec + (double) ts.tv_nsec / 1e9;
}

// Block until both (snapshot az, snapshot el) are within tol of the
// target, or timeout_s elapses. Returns 0 on arrival, -1 on timeout.
static int wait_for_arrival(antenna_rotator_async_t *async,
                            double target_az, double target_el,
                            double tol_deg, double timeout_s,
                            double *out_arrival_az,
                            double *out_arrival_el)
{
    double t0 = now_mono_s();
    while (now_mono_s() - t0 < timeout_s) {
        double az = 0.0, el = 0.0;
        int ok = 0, stale_ms = 0;
        antenna_rotator_async_snapshot(async, &az, &el, &ok, &stale_ms, NULL);
        if (ok && fabs(az - target_az) <= tol_deg
               && fabs(el - target_el) <= tol_deg) {
            if (out_arrival_az) *out_arrival_az = az;
            if (out_arrival_el) *out_arrival_el = el;
            return 0;
        }
        usleep((useconds_t)(AR_CAL_POLL_INTERVAL_S * 1e6));
    }
    return -1;
}

rotator_calibrate_result_t rotator_calibrate_run(
    antenna_rotator_async_t *async,
    double *out_az_dps,
    double *out_el_dps,
    FILE *log)
{
    if (async == NULL) return ROTATOR_CALIBRATE_NO_WORKER;

    // Step 0: confirm we have a valid snapshot before commanding any
    // motion. If the rotator has never replied, there's no point
    // sending SETs.
    if (antenna_rotator_async_wait_first_status(async, 2000) != 0) {
        return ROTATOR_CALIBRATE_NO_INITIAL_STATUS;
    }

    // Step 1: park to known reference. The arc lengths in steps 2/3
    // are measured from this position; the park leg itself isn't
    // timed (it may be a long pre-positioning move from wherever the
    // operator left the antenna).
    if (log) fprintf(log,
                     "calibrate: parking to (%.1f, %.1f) deg...\n",
                     AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    antenna_rotator_async_submit_set(async,
                                     AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    double park_az = 0.0, park_el = 0.0;
    if (wait_for_arrival(async, AR_CAL_PARK_AZ, AR_CAL_PARK_EL,
                         AR_CAL_PARK_TOL_DEG, AR_CAL_PARK_TIMEOUT_S,
                         &park_az, &park_el) != 0) {
        return ROTATOR_CALIBRATE_PARK_TIMEOUT;
    }
    if (log) fprintf(log, "calibrate: parked at (%.2f, %.2f) deg\n",
                     park_az, park_el);

    // Step 2: time the az sweep.
    double target_az = park_az + AR_CAL_AZ_ARC;
    if (log) fprintf(log, "calibrate: sweeping az %.1f -> %.1f deg\n",
                     park_az, target_az);
    double t_start = now_mono_s();
    antenna_rotator_async_submit_set(async, target_az, park_el);
    double az_arrival = 0.0, el_arrival = 0.0;
    if (wait_for_arrival(async, target_az, park_el,
                         AR_CAL_ARRIVAL_TOL_DEG, AR_CAL_SWEEP_TIMEOUT_S,
                         &az_arrival, &el_arrival) != 0) {
        return ROTATOR_CALIBRATE_AZ_TIMEOUT;
    }
    double az_seconds = now_mono_s() - t_start;
    double az_actual_arc = fabs(az_arrival - park_az);
    double az_dps = (az_seconds > 0.0) ? az_actual_arc / az_seconds : 0.0;
    if (log) fprintf(log,
                     "calibrate: az sweep %.2f deg in %.3f s -> %.3f deg/s\n",
                     az_actual_arc, az_seconds, az_dps);
    if (!(az_dps > 0.0) || !isfinite(az_dps)) return ROTATOR_CALIBRATE_BAD_RATE;

    // Step 3: time the el sweep. We stay at the post-az position so
    // the el move doesn't have to redo the az leg.
    double target_el = park_el + AR_CAL_EL_ARC;
    if (log) fprintf(log, "calibrate: sweeping el %.1f -> %.1f deg\n",
                     park_el, target_el);
    t_start = now_mono_s();
    antenna_rotator_async_submit_set(async, az_arrival, target_el);
    double el_after_az = 0.0, el_after_el = 0.0;
    if (wait_for_arrival(async, az_arrival, target_el,
                         AR_CAL_ARRIVAL_TOL_DEG, AR_CAL_SWEEP_TIMEOUT_S,
                         &el_after_az, &el_after_el) != 0) {
        return ROTATOR_CALIBRATE_EL_TIMEOUT;
    }
    double el_seconds = now_mono_s() - t_start;
    double el_actual_arc = fabs(el_after_el - park_el);
    double el_dps = (el_seconds > 0.0) ? el_actual_arc / el_seconds : 0.0;
    if (log) fprintf(log,
                     "calibrate: el sweep %.2f deg in %.3f s -> %.3f deg/s\n",
                     el_actual_arc, el_seconds, el_dps);
    if (!(el_dps > 0.0) || !isfinite(el_dps)) return ROTATOR_CALIBRATE_BAD_RATE;

    // Step 4: return to park (cosmetic, but leaves the antenna in a
    // known state). Don't error on timeout here — the rates are
    // already measured, so we'd rather save them than discard.
    if (log) fprintf(log, "calibrate: returning to (%.1f, %.1f) deg\n",
                     AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    antenna_rotator_async_submit_set(async,
                                     AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    (void) wait_for_arrival(async, AR_CAL_PARK_AZ, AR_CAL_PARK_EL,
                            AR_CAL_PARK_TOL_DEG, AR_CAL_PARK_TIMEOUT_S,
                            NULL, NULL);

    // Step 5: persist.
    if (pursuit_save_rotator_rates(az_dps, el_dps) != 0) {
        return ROTATOR_CALIBRATE_SAVE_FAILED;
    }
    if (out_az_dps) *out_az_dps = az_dps;
    if (out_el_dps) *out_el_dps = el_dps;
    return ROTATOR_CALIBRATE_OK;
}
