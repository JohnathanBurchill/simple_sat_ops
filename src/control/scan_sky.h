/*

   Simple Satellite Operations  control/scan_sky.h

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

// --scan-sky mode: drive the rotator through a roughly equal-solid-angle
// grid of (az, el) targets, dwelling at each while writing per-target
// arrival timestamps to a CSV — for noise-floor / antenna-pattern runs.
// State lives on state_t.scan (scan_sky_t).

#ifndef CONTROL_SCAN_SKY_H
#define CONTROL_SCAN_SKY_H

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Begin a sky scan: build the target grid (if needed), open the CSV, and
// point at the first target.
void scan_sky_start(state_t *state);

// Stop a scan in progress, recording `reason` ("user" / "complete" / ...).
void scan_sky_stop(state_t *state, const char *reason);

// Advance the scan: when the rotator has settled and the dwell has elapsed,
// log the target and step to the next. `t_now` is monotonic seconds.
void scan_sky_tick(state_t *state, double t_now);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_SCAN_SKY_H
