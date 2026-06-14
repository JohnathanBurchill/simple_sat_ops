/*

   Simple Satellite Operations  control/scan_sky.c

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

#include "scan_sky.h"
#include "state.h"

#include "sso_audit.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

// Seconds the rotator dwells at each scan target before logging + stepping.
#define SCAN_DWELL_S 5.0

// Defined in control/tracking (extracted later); declared here until then.
int point_to_stationary_target(state_t *state, double azimuth, double elevation);

// -------------------------------------------------------------------
// --scan-sky helpers
// -------------------------------------------------------------------
//
// scan_build_targets fills state->scan.targets with a roughly equal-solid-
// angle grid covering the sky above the horizon. Elevation rings are
// spaced del_deg apart; at each ring, the azimuth count is round(
// 360/del_deg * cos(el)) so high-elevation rings (which subtend less
// solid angle on the sphere) carry proportionally fewer az samples.
// Ring direction alternates so consecutive targets across rings sit
// close in azimuth (snake pattern, minimises rotator transit).
//
// Azimuths are emitted in the range (-180, 180), with the exact
// boundary skipped so the rotator never tries to drive past its
// mechanical wrap limits (range is [-179, 539]; we stay well inside).
// The first target is forced to (0, 0) so every run starts from a
// known reference.

static int scan_build_targets(state_t *state, double del_deg)
{
    if (del_deg < 1.0) del_deg = 1.0;
    if (del_deg > 45.0) del_deg = 45.0;
    int n = 0;
    // Force the starting target.
    state->scan.targets[n].az_deg = 0.0;
    state->scan.targets[n].el_deg = 0.0;
    ++n;
    int direction = 1;
    int el_steps  = (int) round(90.0 / del_deg);
    for (int eli = 0; eli <= el_steps && n < SCAN_MAX_TARGETS; ++eli) {
        double el = (double) eli * del_deg;
        if (el > 90.0) el = 90.0;
        int n_az;
        if (el >= 90.0 - 0.001) {
            n_az = 1;
        } else {
            double cos_el = cos(el * M_PI / 180.0);
            n_az = (int) round(360.0 / del_deg * cos_el);
            if (n_az < 1) n_az = 1;
        }
        for (int i = 0; i < n_az && n < SCAN_MAX_TARGETS; ++i) {
            int idx = (direction > 0) ? i : (n_az - 1 - i);
            double az;
            if (n_az == 1) {
                az = 0.0;
            } else {
                az = (double) idx * 360.0 / (double) n_az;
                if (az >= 180.0) az -= 360.0;
            }
            // Stay strictly inside the rotator's [-179, +179] safe window.
            if (az <= -179.0) continue;
            if (az >= 179.5)  az = 179.0;
            // First sample on ring 0 is (0,0) — we already emitted it
            // as the forced starting target; skip the dup.
            if (eli == 0 && fabs(az) < 0.001 && n > 0
                && state->scan.targets[0].az_deg == 0.0
                && state->scan.targets[0].el_deg == 0.0
                && n == 1) {
                continue;
            }
            state->scan.targets[n].az_deg = az;
            state->scan.targets[n].el_deg = el;
            ++n;
        }
        direction = -direction;
    }
    state->scan.n_targets = n;
    return n;
}

static void scan_csv_open(state_t *state)
{
    if (state->scan.csv_fp != NULL) return;
    struct timeval tv;
    struct tm utc;
    if (gettimeofday(&tv, NULL) != 0 || gmtime_r(&tv.tv_sec, &utc) == NULL) {
        return;
    }
    const char *dir = state->pass_folder[0] ? state->pass_folder : ".";
    int n = snprintf(state->scan.csv_path, sizeof state->scan.csv_path,
                     "%s/scan_sky_UT=%04d%02d%02dT%02d%02d%02dZ.csv",
                     dir,
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec);
    if (n <= 0 || (size_t) n >= sizeof state->scan.csv_path) {
        state->scan.csv_path[0] = '\0';
        return;
    }
    state->scan.csv_fp = fopen(state->scan.csv_path, "w");
    if (state->scan.csv_fp == NULL) {
        state->scan.csv_path[0] = '\0';
        return;
    }
    fputs("# scan-sky log\n"
          "# unix_time_ms,target_az_deg,target_el_deg,"
          "actual_az_deg,actual_el_deg,event\n",
          state->scan.csv_fp);
    fflush(state->scan.csv_fp);
}

static void scan_csv_log(state_t *state, double tgt_az, double tgt_el,
                          double act_az, double act_el,
                          const char *event)
{
    if (state->scan.csv_fp == NULL) return;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long u_ms = (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000;
    fprintf(state->scan.csv_fp, "%lld,%.3f,%.3f,%.3f,%.3f,%s\n",
            u_ms, tgt_az, tgt_el, act_az, act_el,
            (event && event[0]) ? event : "");
    fflush(state->scan.csv_fp);
}

static void scan_csv_close(state_t *state)
{
    if (state->scan.csv_fp != NULL) {
        fclose(state->scan.csv_fp);
        state->scan.csv_fp = NULL;
    }
}

void scan_sky_start(state_t *state)
{
    if (state->scan.active) return;
    if (state->scan.n_targets == 0) scan_build_targets(state, state->scan.step_deg);
    if (state->scan.n_targets == 0) return;
    scan_csv_open(state);
    state->scan.active        = 1;
    state->scan.idx           = 0;
    state->scan.dwell_start_s = 0.0;
    // Make sure no concurrent satellite-tracking logic competes for
    // the rotator.
    state->satellite_tracking         = 0;
    state->antenna_rotator.tracking   = 0;
    state->antenna_rotator.flip_mode_pass     = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half  = 0;
    // Command the first target via point_to_stationary_target so the
    // two-step homing handles wraparound shortest-path correctly.
    point_to_stationary_target(state,
                                state->scan.targets[0].az_deg,
                                state->scan.targets[0].el_deg);
    {
        char det[160];
        snprintf(det, sizeof det,
            "n_targets=%d step_deg=%.1f dwell_s=%.1f csv=\"%.100s\"",
            state->scan.n_targets, state->scan.step_deg, SCAN_DWELL_S,
            state->scan.csv_path[0] ? state->scan.csv_path : "(none)");
        sso_audit_event("scan-sky-start", det);
    }
}

void scan_sky_stop(state_t *state, const char *reason)
{
    if (!state->scan.active) return;
    scan_csv_log(state, NAN, NAN,
                 state->antenna_rotator.azimuth,
                 state->antenna_rotator.elevation,
                 reason ? reason : "stop");
    scan_csv_close(state);
    int done_idx = state->scan.idx;
    int total    = state->scan.n_targets;
    state->scan.active        = 0;
    state->scan.idx           = 0;
    state->scan.dwell_start_s = 0.0;
    {
        char det[160];
        snprintf(det, sizeof det,
            "reason=\"%.60s\" completed=%d/%d",
            reason ? reason : "?", done_idx, total);
        sso_audit_event("scan-sky-stop", det);
    }
}

// Drive the scan state machine. Called once per tick of the main loop
// while state->scan.active is 1.
void scan_sky_tick(state_t *state, double t_now)
{
    if (!state->scan.active) return;
    if (state->scan.idx >= state->scan.n_targets) {
        scan_sky_stop(state, "complete");
        return;
    }
    // Wait for the rotator to settle before dwelling.
    if (state->antenna_rotator.antenna_is_moving) return;
    if (state->scan.dwell_start_s <= 0.0) {
        state->scan.dwell_start_s = t_now;
        const scan_target_t *t = &state->scan.targets[state->scan.idx];
        scan_csv_log(state, t->az_deg, t->el_deg,
                     state->antenna_rotator.azimuth,
                     state->antenna_rotator.elevation,
                     "arrived");
        return;
    }
    if (t_now - state->scan.dwell_start_s < SCAN_DWELL_S) return;
    // Dwell expired — advance to the next target.
    ++state->scan.idx;
    state->scan.dwell_start_s = 0.0;
    if (state->scan.idx >= state->scan.n_targets) {
        scan_sky_stop(state, "complete");
        return;
    }
    const scan_target_t *t = &state->scan.targets[state->scan.idx];
    point_to_stationary_target(state, t->az_deg, t->el_deg);
}
