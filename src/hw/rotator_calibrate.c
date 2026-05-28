/*

   Simple Satellite Operations  rotator_calibrate.c

   See rotator_calibrate.h. Drives the antenna across known arcs and
   times each leg using OPERATOR confirmation (press ENTER when the
   antenna stops moving), because the SPID Rot2Prog firmware updates
   its internal position counter atomically on receiving a SET (so a
   subsequent STATUS query reports the target position immediately,
   regardless of where the motor actually is). Auto-detecting motion
   completion via STATUS would yield a ~serial-roundtrip-time
   measurement, not a real slew rate — bench-confirmed 2026-05-28
   when an early auto-timed version produced ~81 deg/s for an SPID
   that physically moves at ~3-6 deg/s.

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
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define AR_CAL_PARK_AZ          0.0
#define AR_CAL_PARK_EL         10.0
#define AR_CAL_AZ_ARC          90.0
#define AR_CAL_EL_ARC          45.0

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

// Read a full line from stdin, discarding everything up to and
// including the newline. Returns 0 on success, -1 on EOF / read error
// (so the calibrator can bail cleanly when the operator hits Ctrl-D).
static int wait_for_enter(FILE *log)
{
    if (log) {
        fputs(">>> press ENTER when the antenna has stopped moving "
              "(Ctrl-D to abort): ", log);
        fflush(log);
    }
    int ch;
    int saw_any = 0;
    while ((ch = getchar()) != EOF) {
        saw_any = 1;
        if (ch == '\n') return 0;
    }
    return saw_any ? 0 : -1;
}

rotator_calibrate_result_t rotator_calibrate_run(
    antenna_rotator_async_t *async,
    double *out_az_dps,
    double *out_el_dps,
    FILE *log)
{
    if (async == NULL) return ROTATOR_CALIBRATE_NO_WORKER;

    // Initial status — confirm the rotator is online before commanding
    // any motion.
    if (antenna_rotator_async_wait_first_status(async, 2000) != 0) {
        return ROTATOR_CALIBRATE_NO_INITIAL_STATUS;
    }

    if (log) {
        fputs("\n=== rotator calibration ===\n", log);
        fputs("This will physically move the antenna across three\n", log);
        fputs("known arcs. After each SET, the SPID Rot2Prog reports\n", log);
        fputs("the TARGET position via STATUS as soon as the command\n", log);
        fputs("is received (its position counter updates atomically),\n", log);
        fputs("so we can't time the motion automatically. Watch the\n", log);
        fputs("antenna and press ENTER when each leg actually stops.\n\n", log);
    }

    // --- Leg 0: park to a known reference ---
    if (log) fprintf(log,
        "[1/3] parking to (az=%.0f, el=%.0f)... SET issued.\n",
        AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    antenna_rotator_async_submit_set(async, AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    if (wait_for_enter(log) != 0) return ROTATOR_CALIBRATE_PARK_TIMEOUT;

    // --- Leg 1: az sweep ---
    double az_target = AR_CAL_PARK_AZ + AR_CAL_AZ_ARC;
    if (log) fprintf(log,
        "\n[2/3] sweeping azimuth %.0f -> %.0f deg (%.0f deg arc)... "
        "SET issued, timing starts now.\n",
        AR_CAL_PARK_AZ, az_target, AR_CAL_AZ_ARC);
    double t_start = now_mono_s();
    antenna_rotator_async_submit_set(async, az_target, AR_CAL_PARK_EL);
    if (wait_for_enter(log) != 0) return ROTATOR_CALIBRATE_AZ_TIMEOUT;
    double az_seconds = now_mono_s() - t_start;
    double az_dps = (az_seconds > 0.0) ? (AR_CAL_AZ_ARC / az_seconds) : 0.0;
    if (!(az_dps > 0.0) || !isfinite(az_dps))
        return ROTATOR_CALIBRATE_BAD_RATE;
    if (log) fprintf(log,
        "    az %.1f deg in %.2f s -> %.3f deg/s\n",
        AR_CAL_AZ_ARC, az_seconds, az_dps);

    // --- Leg 2: el sweep (from the post-az position) ---
    double el_target = AR_CAL_PARK_EL + AR_CAL_EL_ARC;
    if (log) fprintf(log,
        "\n[3/3] sweeping elevation %.0f -> %.0f deg (%.0f deg arc)... "
        "SET issued, timing starts now.\n",
        AR_CAL_PARK_EL, el_target, AR_CAL_EL_ARC);
    t_start = now_mono_s();
    antenna_rotator_async_submit_set(async, az_target, el_target);
    if (wait_for_enter(log) != 0) return ROTATOR_CALIBRATE_EL_TIMEOUT;
    double el_seconds = now_mono_s() - t_start;
    double el_dps = (el_seconds > 0.0) ? (AR_CAL_EL_ARC / el_seconds) : 0.0;
    if (!(el_dps > 0.0) || !isfinite(el_dps))
        return ROTATOR_CALIBRATE_BAD_RATE;
    if (log) fprintf(log,
        "    el %.1f deg in %.2f s -> %.3f deg/s\n",
        AR_CAL_EL_ARC, el_seconds, el_dps);

    // --- Return to park (no timing needed) ---
    if (log) fprintf(log,
        "\nreturning to (az=%.0f, el=%.0f); no need to confirm.\n",
        AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    antenna_rotator_async_submit_set(async, AR_CAL_PARK_AZ, AR_CAL_PARK_EL);

    if (pursuit_save_rotator_rates(az_dps, el_dps) != 0) {
        return ROTATOR_CALIBRATE_SAVE_FAILED;
    }
    if (out_az_dps) *out_az_dps = az_dps;
    if (out_el_dps) *out_el_dps = el_dps;
    return ROTATOR_CALIBRATE_OK;
}
