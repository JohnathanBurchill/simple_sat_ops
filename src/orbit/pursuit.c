/*

   Simple Satellite Operations  pursuit.c

   See pursuit.h for the public API. The planner here is a constraint-
   projection scheme: start with waypoints at the satellite's predicted
   sky position, then iteratively project each interior waypoint into
   the intersection of the two rate-feasible boxes implied by its
   neighbors. The unconstrained optimum (sat_pos) is L2-projected onto
   that intersection, so the trajectory both stays within rate budget
   and stays as close as possible to the sat. Forward and backward
   sweeps share the same projection step so the implementation is one
   short loop.

   Cost is sum of squared (Δaz, Δel) in degrees on the dense sampling
   grid — close enough to true angular separation for the optimizer's
   purposes; the planner exposes max_error_deg in the result so the
   caller can sanity-check.

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

#include "pursuit.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define PURSUIT_PATH_MAX 512

void pursuit_config_defaults(pursuit_config_t *cfg)
{
    if (cfg == NULL) return;
    cfg->jul_aos              = 0.0;
    cfg->jul_los              = 0.0;
    cfg->waypoint_dt_s        = 5.0;
    cfg->dense_dt_s           = 1.0;
    cfg->max_iter             = 6;
    cfg->cost_improvement_eps = 0.005;
    cfg->r_az_dps             = 0.0;
    cfg->r_el_dps             = 0.0;
    cfg->az_min               = -179.0;
    cfg->az_max               = 539.0;
    cfg->el_min               = -5.0;
    cfg->el_max               = 180.0;
    cfg->a0_unwrapped         = 0.0;
    cfg->e0                   = 0.0;
}

static double clamp_d(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Interpolate the antenna's position at an arbitrary time inside the
// plan. plan must have >= 1 waypoint; outside the [first, last] window
// the closest waypoint's value is returned.
static void interp_waypoint(const pursuit_waypoint_t *W, size_t n,
                            double t, double *az, double *el)
{
    if (n == 0) {
        if (az) *az = 0.0;
        if (el) *el = 0.0;
        return;
    }
    if (t <= W[0].jul_utc) {
        if (az) *az = W[0].az_unwrapped;
        if (el) *el = W[0].el;
        return;
    }
    if (t >= W[n - 1].jul_utc) {
        if (az) *az = W[n - 1].az_unwrapped;
        if (el) *el = W[n - 1].el;
        return;
    }
    // Linear scan from the front. Plans are small (<= a few hundred
    // waypoints) so this is faster than a branch-predicted bsearch and
    // it makes pursuit_aim_at trivially correct.
    size_t k = 0;
    while (k + 1 < n && W[k + 1].jul_utc <= t) ++k;
    double t0 = W[k].jul_utc;
    double t1 = W[k + 1].jul_utc;
    double frac = (t - t0) / (t1 - t0);
    if (az) *az = W[k].az_unwrapped
                  + (W[k + 1].az_unwrapped - W[k].az_unwrapped) * frac;
    if (el) *el = W[k].el + (W[k + 1].el - W[k].el) * frac;
}

// Evaluate the trajectory cost (sum of squared errors over the dense
// grid). Also writes back the per-iteration max / mean stats.
static double evaluate_cost(const pursuit_waypoint_t *W, size_t n,
                            pursuit_sat_sample_fn_t sat_fn, void *sat_ctx,
                            double jul_aos, double jul_los,
                            double dense_dt_s,
                            double *out_max_deg, double *out_mean_deg)
{
    double cost = 0.0;
    double max_err = 0.0;
    double sum_err = 0.0;
    int    n_samples = 0;
    double dense_dt_days = dense_dt_s / 86400.0;
    if (dense_dt_days <= 0.0) dense_dt_days = 1.0 / 86400.0;
    for (double t = jul_aos; t <= jul_los; t += dense_dt_days) {
        double sat_az = 0.0, sat_el = 0.0;
        if (sat_fn(t, &sat_az, &sat_el, sat_ctx) != 0) continue;
        double ant_az = 0.0, ant_el = 0.0;
        interp_waypoint(W, n, t, &ant_az, &ant_el);
        double daz = ant_az - sat_az;
        double del = ant_el - sat_el;
        double err_sq = daz * daz + del * del;
        cost += err_sq;
        double err = sqrt(err_sq);
        sum_err += err;
        if (err > max_err) max_err = err;
        ++n_samples;
    }
    if (out_max_deg) *out_max_deg = max_err;
    if (out_mean_deg) *out_mean_deg = (n_samples > 0)
                                         ? (sum_err / (double) n_samples)
                                         : 0.0;
    return cost;
}

// Project a single interior waypoint k. Returns 1 if it moved (any
// component changed by more than epsilon), 0 otherwise.
static int project_waypoint(pursuit_waypoint_t *W, size_t n, size_t k,
                            const pursuit_config_t *cfg,
                            pursuit_sat_sample_fn_t sat_fn,
                            void *sat_ctx)
{
    if (k == 0 || k >= n) return 0;
    double dt_before_s = (W[k].jul_utc - W[k - 1].jul_utc) * 86400.0;
    if (dt_before_s < 0.0) dt_before_s = 0.0;

    double az_lo = W[k - 1].az_unwrapped - cfg->r_az_dps * dt_before_s;
    double az_hi = W[k - 1].az_unwrapped + cfg->r_az_dps * dt_before_s;
    double el_lo = W[k - 1].el - cfg->r_el_dps * dt_before_s;
    double el_hi = W[k - 1].el + cfg->r_el_dps * dt_before_s;

    if (k + 1 < n) {
        double dt_after_s = (W[k + 1].jul_utc - W[k].jul_utc) * 86400.0;
        if (dt_after_s < 0.0) dt_after_s = 0.0;
        double az_lo2 = W[k + 1].az_unwrapped - cfg->r_az_dps * dt_after_s;
        double az_hi2 = W[k + 1].az_unwrapped + cfg->r_az_dps * dt_after_s;
        double el_lo2 = W[k + 1].el - cfg->r_el_dps * dt_after_s;
        double el_hi2 = W[k + 1].el + cfg->r_el_dps * dt_after_s;
        if (az_lo2 > az_lo) az_lo = az_lo2;
        if (az_hi2 < az_hi) az_hi = az_hi2;
        if (el_lo2 > el_lo) el_lo = el_lo2;
        if (el_hi2 < el_hi) el_hi = el_hi2;
    }

    // Apply absolute bounds.
    if (cfg->az_min > az_lo) az_lo = cfg->az_min;
    if (cfg->az_max < az_hi) az_hi = cfg->az_max;
    if (cfg->el_min > el_lo) el_lo = cfg->el_min;
    if (cfg->el_max < el_hi) el_hi = cfg->el_max;

    // Unconstrained optimum is the sat position at this time.
    double desired_az = W[k].az_unwrapped;
    double desired_el = W[k].el;
    double sat_az = 0.0, sat_el = 0.0;
    if (sat_fn(W[k].jul_utc, &sat_az, &sat_el, sat_ctx) == 0) {
        desired_az = sat_az;
        desired_el = sat_el;
    }

    // When the rate budget can carry W[k] from W[k-1] AND back from
    // W[k+1] inside a non-empty intersection, project the
    // unconstrained optimum (the sat position) into it. When the
    // intersection is EMPTY — happens whenever the sat is moving
    // faster than the rotator and consecutive waypoints aren't yet
    // self-consistent — fall back to the forward-feasible box only
    // (the one anchored on W[k-1]) and aim its boundary closest to
    // W[k+1]. The next iteration's pass over W[k+1] will pull it in
    // turn; alternating projections converges to a strictly
    // rate-feasible trajectory.
    double new_az;
    {
        double fwd_lo = W[k - 1].az_unwrapped - cfg->r_az_dps * dt_before_s;
        double fwd_hi = W[k - 1].az_unwrapped + cfg->r_az_dps * dt_before_s;
        if (az_lo > az_hi) {
            double target = (k + 1 < n) ? W[k + 1].az_unwrapped : desired_az;
            new_az = clamp_d(target, fwd_lo, fwd_hi);
        } else {
            new_az = clamp_d(desired_az, az_lo, az_hi);
        }
    }
    double new_el;
    {
        double fwd_lo = W[k - 1].el - cfg->r_el_dps * dt_before_s;
        double fwd_hi = W[k - 1].el + cfg->r_el_dps * dt_before_s;
        if (el_lo > el_hi) {
            double target = (k + 1 < n) ? W[k + 1].el : desired_el;
            new_el = clamp_d(target, fwd_lo, fwd_hi);
        } else {
            new_el = clamp_d(desired_el, el_lo, el_hi);
        }
    }
    new_az = clamp_d(new_az, cfg->az_min, cfg->az_max);
    new_el = clamp_d(new_el, cfg->el_min, cfg->el_max);

    int moved = (fabs(new_az - W[k].az_unwrapped) > 1e-6)
             || (fabs(new_el - W[k].el) > 1e-6);
    W[k].az_unwrapped = new_az;
    W[k].el = new_el;
    return moved;
}

int pursuit_plan_build(const pursuit_config_t *cfg,
                       pursuit_sat_sample_fn_t sat_fn,
                       void *sat_ctx,
                       pursuit_plan_t *out)
{
    if (cfg == NULL || sat_fn == NULL || out == NULL) return -1;
    if (cfg->jul_los <= cfg->jul_aos) return -1;
    if (cfg->r_az_dps <= 0.0 || cfg->r_el_dps <= 0.0) return -1;
    if (cfg->waypoint_dt_s <= 0.0) return -1;

    memset(out, 0, sizeof *out);

    // Time grid.
    double wp_dt_days = cfg->waypoint_dt_s / 86400.0;
    double span_days = cfg->jul_los - cfg->jul_aos;
    size_t N = (size_t) floor(span_days / wp_dt_days) + 1;
    if (N < 2) N = 2;
    if (N > 8192) N = 8192;   // sanity cap; a multi-hour pass at 5 s -> 720

    pursuit_waypoint_t *W = calloc(N, sizeof *W);
    if (W == NULL) return -1;

    // Initial seed: waypoint at sat position. Last waypoint pinned to
    // jul_los so the playback's end is well-defined.
    for (size_t k = 0; k < N; ++k) {
        double frac = (N == 1) ? 0.0 : (double) k / (double) (N - 1);
        W[k].jul_utc = cfg->jul_aos + frac * span_days;
        double az = 0.0, el = 0.0;
        if (sat_fn(W[k].jul_utc, &az, &el, sat_ctx) != 0) {
            az = cfg->a0_unwrapped;
            el = cfg->e0;
        }
        W[k].az_unwrapped = az;
        W[k].el = el;
    }
    // Pin the first waypoint to the antenna's current position so
    // the plan begins where the rotator is, not where the sat is.
    W[0].az_unwrapped = cfg->a0_unwrapped;
    W[0].el           = cfg->e0;

    int max_iter = cfg->max_iter > 0 ? cfg->max_iter : 6;
    if (max_iter > 16) max_iter = 16;

    double prev_cost = -1.0;
    int    iter_used = 0;
    double max_err   = 0.0;
    double mean_err  = 0.0;
    for (int iter = 0; iter < max_iter; ++iter) {
        int moved = 0;
        // Forward sweep.
        for (size_t k = 1; k < N; ++k) {
            moved |= project_waypoint(W, N, k, cfg, sat_fn, sat_ctx);
        }
        // Backward sweep, smoothing the corners the forward pass left.
        for (size_t k = N; k-- > 1; ) {
            moved |= project_waypoint(W, N, k, cfg, sat_fn, sat_ctx);
        }
        ++iter_used;
        double cost = evaluate_cost(W, N, sat_fn, sat_ctx,
                                    cfg->jul_aos, cfg->jul_los,
                                    cfg->dense_dt_s,
                                    &max_err, &mean_err);
        if (prev_cost > 0.0) {
            double improvement = (prev_cost - cost) / prev_cost;
            if (improvement >= 0.0 && improvement < cfg->cost_improvement_eps) {
                prev_cost = cost;
                break;
            }
        }
        if (!moved) {
            prev_cost = cost;
            break;
        }
        prev_cost = cost;
    }

    out->waypoints       = W;
    out->n_waypoints     = N;
    out->iterations_used = iter_used;
    out->max_error_deg   = max_err;
    out->mean_error_deg  = mean_err;
    out->final_cost      = (prev_cost >= 0.0) ? prev_cost : 0.0;
    return 0;
}

void pursuit_plan_free(pursuit_plan_t *plan)
{
    if (plan == NULL) return;
    free(plan->waypoints);
    plan->waypoints   = NULL;
    plan->n_waypoints = 0;
}

int pursuit_aim_at(const pursuit_plan_t *plan, double jul_utc,
                   double *out_az_unwrapped, double *out_el)
{
    if (plan == NULL || plan->waypoints == NULL || plan->n_waypoints == 0)
        return -1;
    if (jul_utc < plan->waypoints[0].jul_utc) return -1;
    if (jul_utc > plan->waypoints[plan->n_waypoints - 1].jul_utc) return -1;
    // Aim at the NEXT waypoint after jul_utc — that's where we
    // command the rotator; its constant slew interpolates the segment.
    size_t k = 0;
    while (k + 1 < plan->n_waypoints
           && plan->waypoints[k + 1].jul_utc <= jul_utc) {
        ++k;
    }
    size_t target = (k + 1 < plan->n_waypoints) ? (k + 1) : k;
    if (out_az_unwrapped) *out_az_unwrapped = plan->waypoints[target].az_unwrapped;
    if (out_el)           *out_el           = plan->waypoints[target].el;
    return 0;
}

// --- Calibration file I/O ------------------------------------------

static void resolve_rate_path(const char *name, char *out, size_t out_cap)
{
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') home = "/tmp";
    int n = snprintf(out, out_cap,
                     "%s/.local/share/simple_sat_ops/%s", home, name);
    if (n < 0 || (size_t) n >= out_cap) {
        out[out_cap - 1] = '\0';
    }
}

static void ensure_parent_dir(const char *path)
{
    char dir[PURSUIT_PATH_MAX];
    snprintf(dir, sizeof dir, "%s", path);
    char *slash = strrchr(dir, '/');
    if (slash == NULL || slash == dir) return;
    *slash = '\0';
    // Walk the path creating each directory if missing. Mirrors the
    // pattern used in carrier_trim.c and sso_paths.c (sso_mkdir_p)
    // without needing the linker dependency on sso_paths from the
    // selftest target.
    struct stat st;
    if (stat(dir, &st) == 0 && S_ISDIR(st.st_mode)) return;
    char tmp[PURSUIT_PATH_MAX];
    snprintf(tmp, sizeof tmp, "%s", dir);
    for (char *p = tmp + 1; *p; ++p) {
        if (*p != '/') continue;
        *p = '\0';
        if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
            *p = '/';
            return;
        }
        *p = '/';
    }
    (void) mkdir(tmp, 0755);
}

static int load_one_rate(const char *name, double *out)
{
    char path[PURSUIT_PATH_MAX];
    resolve_rate_path(name, path, sizeof path);
    FILE *fp = fopen(path, "r");
    if (fp == NULL) return -1;
    double v = 0.0;
    int rc = fscanf(fp, "%lf", &v);
    fclose(fp);
    if (rc != 1) return -1;
    if (!(v > 0.0) || !isfinite(v)) return -1;
    *out = v;
    return 0;
}

int pursuit_load_rotator_rates(double *out_az_dps, double *out_el_dps)
{
    double az = 0.0, el = 0.0;
    if (load_one_rate("rotator_az_rate_dps", &az) != 0) return -1;
    if (load_one_rate("rotator_el_rate_dps", &el) != 0) return -1;
    if (out_az_dps) *out_az_dps = az;
    if (out_el_dps) *out_el_dps = el;
    return 0;
}

static int save_one_rate(const char *name, double value)
{
    if (!(value > 0.0) || !isfinite(value)) return -1;
    char path[PURSUIT_PATH_MAX];
    resolve_rate_path(name, path, sizeof path);
    ensure_parent_dir(path);
    FILE *fp = fopen(path, "w");
    if (fp == NULL) return -1;
    int rc = fprintf(fp, "%.6f\n", value);
    fclose(fp);
    return (rc > 0) ? 0 : -1;
}

int pursuit_save_rotator_rates(double az_dps, double el_dps)
{
    if (save_one_rate("rotator_az_rate_dps", az_dps) != 0) return -1;
    if (save_one_rate("rotator_el_rate_dps", el_dps) != 0) return -1;
    return 0;
}
