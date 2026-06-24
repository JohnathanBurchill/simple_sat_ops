/*

   Simple Satellite Operations  rotator_calibrate.c

   See rotator_calibrate.h. Drives the antenna across known arcs and
   times each leg using STATUS encoder samples streamed from the async
   worker — no operator timing, no camera-delay artifacts.

   Bench-discovered 2026-05-28: the SPID Rot2Prog's first STATUS reply
   after a SET reflects the new TARGET (its position counter updates
   atomically on receipt). The encoder catches up over the next several
   STATUS samples as the motor actually moves. We therefore:

     - drop the first 2 samples after each SET (target ack + worker
       race);
     - watch consecutive samples for genuine motion (delta > epsilon);
     - declare arrival once N consecutive samples are within tolerance
       (motor at rest);
     - drop the last 2 samples (deceleration / settle);
     - fit the rate over the "middle" samples that show steady motion.

   If the firmware truly target-latches and the encoder NEVER reports
   intermediate positions, this still detects the problem (no motion
   sample beyond the first ack) and bails with ROTATOR_CALIBRATE_BAD_RATE
   rather than producing a misleading "rate".

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

// Park position + arc lengths. Larger arcs improve rate accuracy
// (fixed startup latency is a smaller fraction of the motion).
#define AR_CAL_PARK_AZ              0.0
#define AR_CAL_PARK_EL             10.0
#define AR_CAL_AZ_ARC              90.0
#define AR_CAL_EL_ARC              45.0
#define AR_CAL_PARK_TIMEOUT_S      60.0
#define AR_CAL_SWEEP_TIMEOUT_S    120.0

// Sample analysis. Tuned for the SPID Rot2Prog's ~3-6 deg/s slew at a
// nominal 0.5 s worker poll period (so ~2 samples / s in motion).
#define AR_CAL_MAX_SAMPLES         512
// Discard the first / last N motion samples to skip the SET-ack +
// startup transient and the deceleration / settle.
#define AR_CAL_DISCARD_HEAD          2
#define AR_CAL_DISCARD_TAIL          2
// Stop sampling once we see this many consecutive samples whose axis
// value barely changes (within AR_CAL_STILL_EPS_DEG of the previous).
#define AR_CAL_STILL_RUN             4
// Wider than the SPID's 0.1 deg encoder resolution so a single-tenth
// jitter at rest doesn't reset the still counter.
#define AR_CAL_STILL_EPS_DEG       0.15
// Per-sample axis motion threshold for "this sample shows real motion".
// One encoder tick (0.1 deg) is borderline; we count anything strictly
// above as motion so the slowest in-motion samples still register.
#define AR_CAL_MOTION_EPS_DEG      0.10

typedef struct {
    double t_mono;
    double az;
    double el;
} cal_sample_t;

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

// Drain STATUS samples from the async worker until either (a) the
// motor has been visibly still for AR_CAL_STILL_RUN samples in a row,
// or (b) timeout_s elapses. Returns the number of samples collected
// (0 on no-fresh-STATUS).
static int collect_samples(antenna_rotator_async_t *async,
                            double timeout_s,
                            cal_sample_t *samples, int cap)
{
    double t0 = now_mono_s();
    int n = 0;
    int still = 0;
    double last_az = 0.0, last_el = 0.0;
    int have_last = 0;
    while (now_mono_s() - t0 < timeout_s && n < cap) {
        // wait for the worker's NEXT good STATUS — this is the natural
        // sample stream we monitor.
        if (antenna_rotator_async_wait_next_good_status(async, 2000) != 0) {
            break;
        }
        double az = 0.0, el = 0.0;
        int ok = 0, stale_ms = 0;
        antenna_rotator_async_snapshot(async, &az, &el, &ok, &stale_ms, NULL);
        if (!ok) continue;
        samples[n].t_mono = now_mono_s();
        samples[n].az     = az;
        samples[n].el     = el;
        ++n;
        if (have_last
            && fabs(az - last_az) < AR_CAL_STILL_EPS_DEG
            && fabs(el - last_el) < AR_CAL_STILL_EPS_DEG) {
            ++still;
            // Need a minimum count of samples before we trust the
            // "still" detector — early on we have only the SET-ack
            // which looks still by definition.
            if (still >= AR_CAL_STILL_RUN
                && n >= AR_CAL_DISCARD_HEAD + AR_CAL_STILL_RUN) {
                break;
            }
        } else {
            still = 0;
        }
        last_az = az;
        last_el = el;
        have_last = 1;
    }
    return n;
}

// Given a sequence of samples, find the time-stamped first/last
// "actually moving" samples on the specified axis (in the trimmed
// interior between AR_CAL_DISCARD_HEAD and n - AR_CAL_DISCARD_TAIL),
// then compute the rate as |Δv| / Δt over that range. Returns the
// rate (deg/s), or -1 on insufficient or no-motion data.
static double rate_from_samples(const cal_sample_t *s, int n,
                                 int axis_is_az,
                                 FILE *log,
                                 const char *axis_label)
{
    if (n < AR_CAL_DISCARD_HEAD + AR_CAL_DISCARD_TAIL + 2) {
        if (log) fprintf(log,
            "    %s: only %d samples; need %d. Rotator may not be reporting "
            "encoder progression via STATUS.\n",
            axis_label, n,
            AR_CAL_DISCARD_HEAD + AR_CAL_DISCARD_TAIL + 2);
        return -1.0;
    }
    int first_motion = -1;
    int last_motion  = -1;
    int lo = AR_CAL_DISCARD_HEAD;
    int hi = n - AR_CAL_DISCARD_TAIL;
    for (int i = lo + 1; i < hi; ++i) {
        double prev_v = axis_is_az ? s[i - 1].az : s[i - 1].el;
        double cur_v  = axis_is_az ? s[i].az     : s[i].el;
        if (fabs(cur_v - prev_v) > AR_CAL_MOTION_EPS_DEG) {
            if (first_motion < 0) first_motion = i - 1;
            last_motion = i;
        }
    }
    if (first_motion < 0 || last_motion <= first_motion) {
        if (log) fprintf(log,
            "    %s: no motion detected across %d interior samples "
            "(SPID may be target-latching encoder reports).\n",
            axis_label, hi - lo);
        return -1.0;
    }
    double dv = (axis_is_az ? s[last_motion].az - s[first_motion].az
                            : s[last_motion].el - s[first_motion].el);
    double dt = s[last_motion].t_mono - s[first_motion].t_mono;
    if (dt <= 0.0 || !isfinite(dt) || !isfinite(dv)) return -1.0;
    if (log) fprintf(log,
        "    %s: %d samples, motion across idx %d..%d, "
        "%.2f deg in %.2f s -> %.3f deg/s\n",
        axis_label, n, first_motion, last_motion,
        fabs(dv), dt, fabs(dv) / dt);
    return fabs(dv) / dt;
}

rotator_calibrate_result_t rotator_calibrate_run(
    antenna_rotator_async_t *async,
    double *out_az_dps,
    double *out_el_dps,
    FILE *log)
{
    if (async == NULL) return ROTATOR_CALIBRATE_NO_WORKER;

    if (antenna_rotator_async_wait_first_status(async, 2000) != 0) {
        return ROTATOR_CALIBRATE_NO_INITIAL_STATUS;
    }

    if (log) {
        fputs("\n=== rotator calibration ===\n", log);
        fputs("Streaming STATUS encoder samples during each leg. The\n", log);
        fputs("first sample after a SET is the target-ack; the next few\n", log);
        fputs("should show the encoder progressing. We auto-detect\n", log);
        fputs("stop and fit the rate over the steady-motion samples.\n", log);
        fputs("No need to time anything by eye / camera.\n\n", log);
    }

    cal_sample_t samples[AR_CAL_MAX_SAMPLES];
    int n_samples = 0;

    // --- Leg 0: park (timing not used; we just need the antenna at a
    //                  known starting position before each measured leg) ---
    if (log) fprintf(log,
        "[1/3] parking to (az=%.1f, el=%.1f)... waiting for stop.\n",
        AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    antenna_rotator_async_submit_set(async, AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    n_samples = collect_samples(async, AR_CAL_PARK_TIMEOUT_S,
                                 samples, AR_CAL_MAX_SAMPLES);
    // 0 samples here means we never got a single status reply during the park
    // window — reported as PARK_TIMEOUT. The label is slightly loose (it's
    // really "no status from the rotator while parking", which a genuine
    // timeout subsumes), but it's the closest existing code and park timing
    // isn't otherwise used.
    if (n_samples == 0) return ROTATOR_CALIBRATE_PARK_TIMEOUT;

    // --- Leg 1: az sweep ---
    double az_target = AR_CAL_PARK_AZ + AR_CAL_AZ_ARC;
    if (log) fprintf(log,
        "\n[2/3] sweeping azimuth %.1f -> %.1f deg... streaming STATUS.\n",
        AR_CAL_PARK_AZ, az_target);
    antenna_rotator_async_submit_set(async, az_target, AR_CAL_PARK_EL);
    n_samples = collect_samples(async, AR_CAL_SWEEP_TIMEOUT_S,
                                 samples, AR_CAL_MAX_SAMPLES);
    double az_dps = rate_from_samples(samples, n_samples,
                                       /*axis_is_az=*/1, log, "az");
    if (!(az_dps > 0.0)) return ROTATOR_CALIBRATE_BAD_RATE;

    // --- Leg 2: el sweep (from the post-az position) ---
    double el_target = AR_CAL_PARK_EL + AR_CAL_EL_ARC;
    if (log) fprintf(log,
        "\n[3/3] sweeping elevation %.1f -> %.1f deg... streaming STATUS.\n",
        AR_CAL_PARK_EL, el_target);
    antenna_rotator_async_submit_set(async, az_target, el_target);
    n_samples = collect_samples(async, AR_CAL_SWEEP_TIMEOUT_S,
                                 samples, AR_CAL_MAX_SAMPLES);
    double el_dps = rate_from_samples(samples, n_samples,
                                       /*axis_is_az=*/0, log, "el");
    if (!(el_dps > 0.0)) return ROTATOR_CALIBRATE_BAD_RATE;

    // --- Return to park (no timing needed) ---
    if (log) fprintf(log,
        "\nreturning to (az=%.1f, el=%.1f); the rates are already known.\n",
        AR_CAL_PARK_AZ, AR_CAL_PARK_EL);
    antenna_rotator_async_submit_set(async, AR_CAL_PARK_AZ, AR_CAL_PARK_EL);

    if (pursuit_save_rotator_rates(az_dps, el_dps) != 0) {
        return ROTATOR_CALIBRATE_SAVE_FAILED;
    }
    if (out_az_dps) *out_az_dps = az_dps;
    if (out_el_dps) *out_el_dps = el_dps;
    return ROTATOR_CALIBRATE_OK;
}
