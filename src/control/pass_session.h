/*

   Simple Satellite Operations  control/pass_session.h

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

// Per-pass operational session: locating a TLE to track, creating the dated
// pass output folder (and the Operations/current symlink), and rendering the
// whole-pass az/el plot. Owns the file/directory side of a tracking session;
// the rotator/Doppler side lives in control/tracking.

#ifndef CONTROL_PASS_SESSION_H
#define CONTROL_PASS_SESSION_H

#include <stddef.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

struct state;
typedef struct state state_t;

// Pass-search horizon (minutes): one week ahead. Shared by the pass-folder
// setup and the operator's prediction panel.
#define MAX_MINUTES_TO_PREDICT ((7 * 1440))

// Recursively scan `dir` for *.tle files; return the newest-mtime path via
// out_path. Returns 0 on success, -1 if dir is unreadable or empty of TLEs.
int find_newest_tle_recursive(const char *dir, char *out_path, size_t out_cap,
                              time_t *best_mtime);

// Read the satellite name (line 0) from a 3-line TLE file into `out`.
// Returns 0 on success, -1 on read failure.
int read_tle_name(const char *tle_path, char *out, size_t out_cap);

// Create (or adopt) this pass's dated output folder, point
// Operations/current at it, and stash the path in state->pass_folder.
void setup_pass_folder(state_t *state, double jul_utc_now);

// Render the whole-pass az/el trajectory plot for the current prediction
// into `pass_folder` via gnuplot.
void generate_pass_plot(state_t *state, const char *pass_folder, double jul_utc_now);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_PASS_SESSION_H
