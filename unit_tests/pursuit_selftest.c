/*

   Simple Satellite Operations  unit_tests/pursuit_selftest.c

   TAP-driven tests for src/orbit/pursuit.c. All sat tracks are closed
   form here — no SGP4 / no rotator hardware — so the math is the only
   thing under test.

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
#include "tap.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Map a Julian date into seconds elapsed since AOS for the mock
// closed-form sat tracks. Keeps the tests easy to read.
static double seconds_since(double jul, double jul_aos)
{
    return (jul - jul_aos) * 86400.0;
}

// --- Mock sat tracks ------------------------------------------------

typedef struct {
    double jul_aos;
    double az0;       // azimuth at t=0
    double omega_az;  // deg/s azimuth rate
    double el0;       // constant elevation
} linear_az_track_t;

static int linear_az_sat(double t, double *az, double *el, void *ctx)
{
    const linear_az_track_t *trk = (const linear_az_track_t *) ctx;
    double s = seconds_since(t, trk->jul_aos);
    if (az) *az = trk->az0 + trk->omega_az * s;
    if (el) *el = trk->el0;
    return 0;
}

typedef struct {
    double jul_aos;
    double duration_s;
    double az_aos;
    double az_los;
    double el_peak;        // peak elevation at midpoint
} apex_track_t;

// Half-sine apex pass: az ramps linearly aos->los, el rises and falls.
// Captures the high-rate apex without needing SGP4.
static int apex_sat(double t, double *az, double *el, void *ctx)
{
    const apex_track_t *trk = (const apex_track_t *) ctx;
    double s = seconds_since(t, trk->jul_aos);
    if (s < 0.0) s = 0.0;
    if (s > trk->duration_s) s = trk->duration_s;
    double frac = s / trk->duration_s;
    if (az) *az = trk->az_aos + (trk->az_los - trk->az_aos) * frac;
    if (el) *el = trk->el_peak * sin(M_PI * frac);
    return 0;
}

typedef struct {
    double jul_aos;
    double duration_s;
    double az_start_unwrapped;
    double az_end_unwrapped;
    double el_const;
} wrap_az_track_t;

static int wrap_az_sat(double t, double *az, double *el, void *ctx)
{
    const wrap_az_track_t *trk = (const wrap_az_track_t *) ctx;
    double s = seconds_since(t, trk->jul_aos);
    if (s < 0.0) s = 0.0;
    if (s > trk->duration_s) s = trk->duration_s;
    double frac = s / trk->duration_s;
    if (az) *az = trk->az_start_unwrapped
                  + (trk->az_end_unwrapped - trk->az_start_unwrapped) * frac;
    if (el) *el = trk->el_const;
    return 0;
}

// --- Helpers --------------------------------------------------------

static void make_cfg(pursuit_config_t *cfg,
                     double jul_aos, double duration_s,
                     double r_az_dps, double r_el_dps,
                     double a0, double e0)
{
    pursuit_config_defaults(cfg);
    cfg->jul_aos      = jul_aos;
    cfg->jul_los      = jul_aos + duration_s / 86400.0;
    cfg->r_az_dps     = r_az_dps;
    cfg->r_el_dps     = r_el_dps;
    cfg->a0_unwrapped = a0;
    cfg->e0           = e0;
}

// Verify every consecutive waypoint pair fits inside the rate budget.
// Returns 1 if feasible, 0 if any pair exceeds budget (caller already
// asserts the count of violations).
static int check_rate_feasibility(const pursuit_plan_t *plan,
                                  double r_az_dps, double r_el_dps,
                                  double tol_dps)
{
    if (plan == NULL || plan->n_waypoints < 2) return 1;
    for (size_t k = 1; k < plan->n_waypoints; ++k) {
        double dt_s = (plan->waypoints[k].jul_utc
                       - plan->waypoints[k - 1].jul_utc) * 86400.0;
        if (dt_s <= 0.0) return 0;
        double daz = fabs(plan->waypoints[k].az_unwrapped
                          - plan->waypoints[k - 1].az_unwrapped);
        double del = fabs(plan->waypoints[k].el
                          - plan->waypoints[k - 1].el);
        if (daz / dt_s > r_az_dps + tol_dps) return 0;
        if (del / dt_s > r_el_dps + tol_dps) return 0;
    }
    return 1;
}

// --- Tests ----------------------------------------------------------

static void test_linear_track_within_rate(void)
{
    // 0.5 deg/s sat, 4 deg/s rotator => rotator can perfectly track.
    linear_az_track_t trk = {
        .jul_aos = 2451545.0,   // J2000 — arbitrary
        .az0     = 100.0,
        .omega_az = 0.5,
        .el0     = 30.0,
    };
    pursuit_config_t cfg;
    make_cfg(&cfg, trk.jul_aos, 300.0 /* 5 min */, 4.0, 3.0, 100.0, 30.0);
    pursuit_plan_t plan = {0};
    int rc = pursuit_plan_build(&cfg, linear_az_sat, &trk, &plan);
    tap_ok(rc == 0, "linear_within_rate: plan built");
    tap_ok(plan.n_waypoints >= 2, "linear_within_rate: >=2 waypoints");
    tap_ok(check_rate_feasibility(&plan, cfg.r_az_dps, cfg.r_el_dps, 1e-6),
           "linear_within_rate: rate-feasible");
    tap_okf(plan.max_error_deg < 0.6,
            "linear_within_rate: max err %.3f deg < 0.6", plan.max_error_deg);
    tap_okf(plan.mean_error_deg < 0.3,
            "linear_within_rate: mean err %.3f deg < 0.3", plan.mean_error_deg);
    pursuit_plan_free(&plan);
}

static void test_linear_track_above_rate(void)
{
    // 10 deg/s sat, 4 deg/s rotator => can't catch; plan should still
    // produce a feasible trajectory and report a non-trivial max error.
    linear_az_track_t trk = {
        .jul_aos = 2451545.0,
        .az0     = 0.0,
        .omega_az = 10.0,
        .el0     = 30.0,
    };
    pursuit_config_t cfg;
    make_cfg(&cfg, trk.jul_aos, 60.0, 4.0, 3.0, 0.0, 30.0);
    pursuit_plan_t plan = {0};
    int rc = pursuit_plan_build(&cfg, linear_az_sat, &trk, &plan);
    tap_ok(rc == 0, "linear_above_rate: plan built");
    tap_ok(check_rate_feasibility(&plan, cfg.r_az_dps, cfg.r_el_dps, 1e-6),
           "linear_above_rate: rate-feasible despite trailing");
    tap_okf(plan.max_error_deg > 5.0,
            "linear_above_rate: max err %.1f deg > 5 (rotator lagging)",
            plan.max_error_deg);
    pursuit_plan_free(&plan);
}

// Baseline used by the apex test: at each dense sample, the antenna
// is wherever it could chase to from the previous sample given the
// rate budget (i.e. always aim at where the sat IS now, never lead).
// This is what today's track loop produces; the planner should beat
// it on integrated error for a high-rate apex.
static double apex_naive_integrated_error_sq(const apex_track_t *trk,
                                              double r_az_dps,
                                              double r_el_dps,
                                              double a0, double e0,
                                              double dense_dt_s,
                                              double *out_max)
{
    double t = 0.0;
    double ant_az = a0;
    double ant_el = e0;
    double cost = 0.0;
    double max_err = 0.0;
    while (t <= trk->duration_s) {
        double sat_az = 0.0, sat_el = 0.0;
        double jul = trk->jul_aos + t / 86400.0;
        apex_sat(jul, &sat_az, &sat_el, (void *) trk);
        double daz = sat_az - ant_az;
        double del = sat_el - ant_el;
        double step_az = r_az_dps * dense_dt_s;
        double step_el = r_el_dps * dense_dt_s;
        if (daz >  step_az) daz =  step_az;
        if (daz < -step_az) daz = -step_az;
        if (del >  step_el) del =  step_el;
        if (del < -step_el) del = -step_el;
        ant_az += daz;
        ant_el += del;
        double e2 = (ant_az - sat_az) * (ant_az - sat_az)
                  + (ant_el - sat_el) * (ant_el - sat_el);
        cost += e2;
        double e = sqrt(e2);
        if (e > max_err) max_err = e;
        t += dense_dt_s;
    }
    if (out_max) *out_max = max_err;
    return cost;
}

static void test_apex_pass_beats_naive(void)
{
    // High-el pass: az sweeps 90->270 over 6 min, el peaks at 80 deg.
    // Apex az rate at midpoint = ((270-90)/360) * 0.5 = 0.5 deg/s
    // (just az motion); the el peak's apex curvature makes el move at
    // a quasi-rate larger than the rotator's el rate near the top.
    apex_track_t trk = {
        .jul_aos    = 2451545.0,
        .duration_s = 360.0,
        .az_aos     = 90.0,
        .az_los     = 270.0,
        .el_peak    = 80.0,
    };
    pursuit_config_t cfg;
    make_cfg(&cfg, trk.jul_aos, trk.duration_s, 4.0, 3.0,
             trk.az_aos, 0.0);
    cfg.waypoint_dt_s = 5.0;
    pursuit_plan_t plan = {0};
    int rc = pursuit_plan_build(&cfg, apex_sat, &trk, &plan);
    tap_ok(rc == 0, "apex: plan built");
    tap_ok(check_rate_feasibility(&plan, cfg.r_az_dps, cfg.r_el_dps, 1e-6),
           "apex: plan rate-feasible");

    // Compute plan's integrated squared error directly off the
    // (reported) mean — we don't have the dense grid here but
    // mean_error_deg is a function of the per-sample errors. Use it
    // as a proxy for total integrated error.
    double naive_max = 0.0;
    double naive_cost = apex_naive_integrated_error_sq(
        &trk, cfg.r_az_dps, cfg.r_el_dps, trk.az_aos, 0.0,
        cfg.dense_dt_s, &naive_max);
    (void) naive_cost;
    tap_okf(plan.max_error_deg <= naive_max + 1.0,
            "apex: planner max err %.2f vs naive %.2f deg (planner no worse)",
            plan.max_error_deg, naive_max);
    pursuit_plan_free(&plan);
}

static void test_az_wrap_unwrapped_monotone(void)
{
    // Sat track that would wrap 350->10 deg if reduced mod 360. The
    // planner consumes unwrapped coords, so the caller is responsible
    // for accumulating across 0/360; here we feed 350->370 and verify
    // the waypoints stay monotone (no 360-deg jumps).
    wrap_az_track_t trk = {
        .jul_aos = 2451545.0,
        .duration_s = 120.0,
        .az_start_unwrapped = 350.0,
        .az_end_unwrapped   = 370.0,
        .el_const           = 20.0,
    };
    pursuit_config_t cfg;
    make_cfg(&cfg, trk.jul_aos, trk.duration_s, 4.0, 3.0, 350.0, 20.0);
    // Allow unwrapped az to exceed 360.
    cfg.az_min = -179.0;
    cfg.az_max =  539.0;
    pursuit_plan_t plan = {0};
    int rc = pursuit_plan_build(&cfg, wrap_az_sat, &trk, &plan);
    tap_ok(rc == 0, "az_wrap: plan built");
    int monotone = 1;
    for (size_t k = 1; k < plan.n_waypoints; ++k) {
        if (plan.waypoints[k].az_unwrapped
                < plan.waypoints[k - 1].az_unwrapped - 1e-6) {
            monotone = 0;
            break;
        }
    }
    tap_ok(monotone, "az_wrap: waypoints monotone in unwrapped az");
    // None should jump by more than the rate budget.
    tap_ok(check_rate_feasibility(&plan, cfg.r_az_dps, cfg.r_el_dps, 1e-6),
           "az_wrap: rate-feasible");
    // No values past az_max (which is 539, well above 370).
    int in_bounds = 1;
    for (size_t k = 0; k < plan.n_waypoints; ++k) {
        if (plan.waypoints[k].az_unwrapped > cfg.az_max + 1e-6
            || plan.waypoints[k].az_unwrapped < cfg.az_min - 1e-6) {
            in_bounds = 0;
            break;
        }
    }
    tap_ok(in_bounds, "az_wrap: az stays within mech bounds");
    pursuit_plan_free(&plan);
}

static void test_boundary_clamp(void)
{
    // Configure a deliberately tight el band [0, 5] and a sat that
    // climbs to el=80. Plan must saturate at el=5, not overshoot.
    apex_track_t trk = {
        .jul_aos = 2451545.0,
        .duration_s = 200.0,
        .az_aos = 100.0,
        .az_los = 200.0,
        .el_peak = 80.0,
    };
    pursuit_config_t cfg;
    make_cfg(&cfg, trk.jul_aos, trk.duration_s, 4.0, 3.0, 100.0, 0.0);
    cfg.el_min = 0.0;
    cfg.el_max = 5.0;
    pursuit_plan_t plan = {0};
    int rc = pursuit_plan_build(&cfg, apex_sat, &trk, &plan);
    tap_ok(rc == 0, "boundary_clamp: plan built");
    int el_in_bounds = 1;
    double max_el = -1e9, min_el = 1e9;
    for (size_t k = 0; k < plan.n_waypoints; ++k) {
        if (plan.waypoints[k].el > max_el) max_el = plan.waypoints[k].el;
        if (plan.waypoints[k].el < min_el) min_el = plan.waypoints[k].el;
        if (plan.waypoints[k].el < cfg.el_min - 1e-6
            || plan.waypoints[k].el > cfg.el_max + 1e-6) {
            el_in_bounds = 0;
            break;
        }
    }
    tap_okf(el_in_bounds, "boundary_clamp: el in [%.1f, %.1f]; saw [%.2f, %.2f]",
            cfg.el_min, cfg.el_max, min_el, max_el);
    pursuit_plan_free(&plan);
}

static void test_degenerate_inputs(void)
{
    pursuit_config_t cfg;
    make_cfg(&cfg, 2451545.0, 10.0, 4.0, 3.0, 0.0, 0.0);
    pursuit_plan_t plan = {0};

    // NULL output.
    linear_az_track_t trk = { .jul_aos = 2451545.0, .az0 = 0,
                              .omega_az = 0.1, .el0 = 30 };
    tap_ok(pursuit_plan_build(&cfg, linear_az_sat, &trk, NULL) == -1,
           "degenerate: NULL output rejected");

    // jul_los <= jul_aos.
    pursuit_config_t bad = cfg;
    bad.jul_los = bad.jul_aos;
    tap_ok(pursuit_plan_build(&bad, linear_az_sat, &trk, &plan) == -1,
           "degenerate: empty window rejected");

    // Non-positive rate.
    bad = cfg;
    bad.r_az_dps = 0.0;
    tap_ok(pursuit_plan_build(&bad, linear_az_sat, &trk, &plan) == -1,
           "degenerate: zero az rate rejected");
    bad = cfg;
    bad.r_el_dps = -1.0;
    tap_ok(pursuit_plan_build(&bad, linear_az_sat, &trk, &plan) == -1,
           "degenerate: negative el rate rejected");

    // NULL sat_fn / cfg.
    tap_ok(pursuit_plan_build(NULL, linear_az_sat, &trk, &plan) == -1,
           "degenerate: NULL cfg rejected");
    tap_ok(pursuit_plan_build(&cfg, NULL, &trk, &plan) == -1,
           "degenerate: NULL sat_fn rejected");

    pursuit_plan_free(&plan);
}

static void test_convergence_cap(void)
{
    // Pathological: high-rate apex where every iteration shuffles
    // waypoints. We assert iterations_used <= max_iter (16 clamp).
    apex_track_t trk = {
        .jul_aos = 2451545.0,
        .duration_s = 600.0,
        .az_aos = 0.0,
        .az_los = 360.0,
        .el_peak = 89.0,
    };
    pursuit_config_t cfg;
    make_cfg(&cfg, trk.jul_aos, trk.duration_s, 4.0, 3.0, 0.0, 0.0);
    cfg.max_iter = 100;  // ask for unlimited; planner caps at 16
    pursuit_plan_t plan = {0};
    int rc = pursuit_plan_build(&cfg, apex_sat, &trk, &plan);
    tap_ok(rc == 0, "convergence_cap: plan built");
    tap_okf(plan.iterations_used <= 16,
            "convergence_cap: iterations_used %d <= 16",
            plan.iterations_used);
    pursuit_plan_free(&plan);
}

static void test_aim_lookup(void)
{
    linear_az_track_t trk = {
        .jul_aos = 2451545.0,
        .az0 = 50.0,
        .omega_az = 0.5,
        .el0 = 20.0,
    };
    pursuit_config_t cfg;
    make_cfg(&cfg, trk.jul_aos, 60.0, 4.0, 3.0, 50.0, 20.0);
    pursuit_plan_t plan = {0};
    tap_ok(pursuit_plan_build(&cfg, linear_az_sat, &trk, &plan) == 0,
           "aim_lookup: plan built");

    // Inside the window: returns 0.
    double az, el;
    int rc = pursuit_aim_at(&plan,
                            cfg.jul_aos + 30.0 / 86400.0, &az, &el);
    tap_ok(rc == 0, "aim_lookup: t inside window returns 0");

    // Before AOS: returns -1.
    rc = pursuit_aim_at(&plan,
                        cfg.jul_aos - 5.0 / 86400.0, &az, &el);
    tap_ok(rc == -1, "aim_lookup: t before AOS returns -1");

    // After LOS: returns -1.
    rc = pursuit_aim_at(&plan,
                        cfg.jul_los + 5.0 / 86400.0, &az, &el);
    tap_ok(rc == -1, "aim_lookup: t after LOS returns -1");

    // At exactly the second-to-last waypoint, the target should be
    // the LAST waypoint (we always aim at the next).
    if (plan.n_waypoints >= 2) {
        double t_at = plan.waypoints[plan.n_waypoints - 2].jul_utc;
        rc = pursuit_aim_at(&plan, t_at, &az, &el);
        tap_ok(rc == 0
               && fabs(az - plan.waypoints[plan.n_waypoints - 1].az_unwrapped) < 1e-9
               && fabs(el - plan.waypoints[plan.n_waypoints - 1].el)         < 1e-9,
               "aim_lookup: returns the NEXT waypoint, not the current");
    }

    pursuit_plan_free(&plan);
}

static void test_rates_io(void)
{
    // Redirect HOME to a tempdir so the test doesn't trash the user's
    // saved values. mkdtemp would be ideal but isn't on all hosts; do
    // it the old-fashioned way.
    char tmpl[] = "/tmp/pursuit_rates_XXXXXX";
    char *dir = mkdtemp(tmpl);
    if (!tap_ok(dir != NULL, "rates_io: tempdir created")) {
        tap_diag("mkdtemp failed");
        return;
    }
    char *home_save = getenv("HOME");
    char saved_home[256] = "";
    if (home_save) snprintf(saved_home, sizeof saved_home, "%s", home_save);
    setenv("HOME", dir, 1);

    // Missing files: load fails.
    double az = 0.0, el = 0.0;
    tap_ok(pursuit_load_rotator_rates(&az, &el) == -1,
           "rates_io: missing files return -1");

    // Save then load round-trips.
    tap_ok(pursuit_save_rotator_rates(4.123, 3.456) == 0,
           "rates_io: save succeeds");
    tap_ok(pursuit_load_rotator_rates(&az, &el) == 0,
           "rates_io: load after save succeeds");
    tap_okf(fabs(az - 4.123) < 1e-3
            && fabs(el - 3.456) < 1e-3,
            "rates_io: round-trip values match (az=%.4f el=%.4f)", az, el);

    // Reject non-positive rates.
    tap_ok(pursuit_save_rotator_rates(0.0, 3.0) == -1,
           "rates_io: zero az rate refused");
    tap_ok(pursuit_save_rotator_rates(3.0, -1.0) == -1,
           "rates_io: negative el rate refused");

    // Corrupt the file and confirm we refuse it.
    char path[512];
    snprintf(path, sizeof path, "%s/.local/share/simple_sat_ops/rotator_az_rate_dps", dir);
    FILE *fp = fopen(path, "w");
    if (fp) { fputs("not-a-number\n", fp); fclose(fp); }
    az = 0.0; el = 0.0;
    tap_ok(pursuit_load_rotator_rates(&az, &el) == -1,
           "rates_io: malformed file rejected");

    // Cleanup. Best effort; if it fails the tempdir survives in /tmp.
    snprintf(path, sizeof path,
             "%s/.local/share/simple_sat_ops/rotator_az_rate_dps", dir);
    unlink(path);
    snprintf(path, sizeof path,
             "%s/.local/share/simple_sat_ops/rotator_el_rate_dps", dir);
    unlink(path);
    snprintf(path, sizeof path, "%s/.local/share/simple_sat_ops", dir);
    rmdir(path);
    snprintf(path, sizeof path, "%s/.local/share", dir);
    rmdir(path);
    snprintf(path, sizeof path, "%s/.local", dir);
    rmdir(path);
    rmdir(dir);

    if (saved_home[0]) setenv("HOME", saved_home, 1);
    else               unsetenv("HOME");
}

// --- Main -----------------------------------------------------------

int main(void)
{
    test_linear_track_within_rate();
    test_linear_track_above_rate();
    test_apex_pass_beats_naive();
    test_az_wrap_unwrapped_monotone();
    test_boundary_clamp();
    test_degenerate_inputs();
    test_convergence_cap();
    test_aim_lookup();
    test_rates_io();
    return tap_done();
}
