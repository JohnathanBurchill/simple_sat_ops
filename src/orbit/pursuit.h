/*

   Simple Satellite Operations  pursuit.h

   Whole-pass antenna trajectory planner. At AOS the operator passes in
   the pass window, the calibrated rotator slew rates, and a callback
   that samples the satellite's mech-frame (az, el) at an arbitrary
   Julian date. The planner returns a sequence of (jul_utc, az, el)
   waypoints whose linear interpolation is a rate-feasible antenna
   trajectory that minimizes integrated pointing error.

   The track loop plays the plan back by aiming at the next waypoint
   each tick; the existing async rotator worker turns that into a
   constant-rate slew. The planner is pure-function math (no SGP4,
   ncurses, libuhd, or rotator hardware in this TU) so the selftest
   exercises every case against closed-form mock sat tracks.

   See pursuit_selftest.c for examples and the project's
   ~/.claude/plans/let-s-implement-a-decode-dynamic-newt.md for the
   surrounding design.

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

#ifndef PURSUIT_H
#define PURSUIT_H

#include <stddef.h>

// Sampler the planner calls to learn where the satellite is at
// jul_utc. In production this wraps update_satellite_position()
// followed by antenna_rotator_to_mech_coords(); in unit tests it's a
// closed-form function. Must populate *out_az_unwrapped and *out_el
// (already unwrapped — the planner does not call accumulate_unwrapped
// internally). Returns 0 on success, non-zero on error.
typedef int (*pursuit_sat_sample_fn_t)(double jul_utc,
                                       double *out_az_unwrapped,
                                       double *out_el,
                                       void *ctx);

typedef struct {
    double jul_utc;
    double az_unwrapped;
    double el;
} pursuit_waypoint_t;

typedef struct {
    pursuit_waypoint_t *waypoints;
    size_t n_waypoints;
    int    iterations_used;
    double max_error_deg;
    double mean_error_deg;
    double final_cost;          // last-iter sum of squared errors
} pursuit_plan_t;

typedef struct {
    double jul_aos;
    double jul_los;
    // Time between consecutive waypoints, seconds. The default 5.0 is
    // a balance: small enough to follow apex curvature, large enough
    // that consecutive segments are meaningfully larger than a single
    // SET-and-read roundtrip.
    double waypoint_dt_s;
    // Time between dense error-evaluation samples, seconds. Cheaper
    // than waypoint_dt_s; cost is one extra sat_fn call per sample
    // per iteration.
    double dense_dt_s;
    // Max forward/backward projection iterations. 6 is plenty for
    // typical passes; the pathological-input test caps at 16.
    int    max_iter;
    // Stop iterating once one full sweep improves the cost by less
    // than this fraction (0..1). 0.005 = "less than 0.5%".
    double cost_improvement_eps;
    // Rotator axis slew rates, deg / second.
    double r_az_dps;
    double r_el_dps;
    // Mechanical bounds (passed through to a saturating clamp). Use
    // ANTENNA_ROTATOR_MINIMUM/MAXIMUM_* from antenna_rotator.h.
    double az_min;
    double az_max;
    double el_min;
    double el_max;
    // Antenna's position at jul_aos (or the planner-call moment, if
    // the planner is invoked mid-pass — the planner doesn't care).
    // The first waypoint is pinned here.
    double a0_unwrapped;
    double e0;
} pursuit_config_t;

// Fill cfg with planner defaults. Caller still has to set jul_aos /
// jul_los / rates / bounds / start.
void pursuit_config_defaults(pursuit_config_t *cfg);

// Build a rate-feasible trajectory across the pass window. Returns 0
// on success and populates `out` (caller frees with pursuit_plan_free).
// Returns -1 on bad input (NULL ptrs, jul_los <= jul_aos, non-positive
// rate, allocation failure).
int  pursuit_plan_build(const pursuit_config_t *cfg,
                        pursuit_sat_sample_fn_t sat_fn,
                        void *sat_ctx,
                        pursuit_plan_t *out);

// Free a plan's waypoints. Idempotent; safe with NULL or zero-init.
void pursuit_plan_free(pursuit_plan_t *plan);

// Look up the aim target at jul_utc. Returns 0 and populates
// out_az_unwrapped + out_el (the NEXT waypoint after jul_utc; that's
// where we tell the rotator to go and let its constant-rate slew
// interpolate). Returns -1 if jul_utc is outside the plan window.
int  pursuit_aim_at(const pursuit_plan_t *plan,
                    double jul_utc,
                    double *out_az_unwrapped,
                    double *out_el);

// Load saved rotator slew rates from
// ~/.local/share/simple_sat_ops/rotator_{az,el}_rate_dps. Returns 0 on
// success and writes both rates. Returns -1 on missing file, malformed
// content, or non-positive value (in which case the outputs are left
// untouched and the caller falls back to today's "aim where sat is
// now" track loop).
int  pursuit_load_rotator_rates(double *out_az_dps, double *out_el_dps);

// Save rotator slew rates, called from --calibrate-rotator after a
// successful sweep. Returns 0 on success; -1 on I/O error or invalid
// inputs (non-positive rate). The parent directory is created as
// needed.
int  pursuit_save_rotator_rates(double az_dps, double el_dps);

#endif // PURSUIT_H
