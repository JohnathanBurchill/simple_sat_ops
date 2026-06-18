/*

    Simple Satellite Operations  pass_schedule.h

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

// The upcoming-passes wire event for --viewer-stream.
//
// A second NDJSON event, separate from sso_ipc's sso_event_t so it doesn't
// bloat that struct. The ground station already runs SGP4 and knows the
// observer location, so it computes the next several days of passes and ships
// them as one {"t":"passes",...} line. Read-only viewers cache the list and
// render / alert from it with no on-device propagation, so the pass list is
// available even when the SSH link is down.
//
// This file (and pass_schedule.c) is the single source of truth for the schema;
// the standalone viewer app vendors a copy with identical encode/decode logic.
// Keep the two in lockstep if the format changes.

#ifndef PASS_SCHEDULE_H
#define PASS_SCHEDULE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Upper bound on passes carried in one event. ~7 days of LEO passes is well
// under this; the encoder/decoder clamp to it.
#define PASS_SCHED_MAX 128

// One predicted pass. Times are Unix seconds (UTC); angles are degrees.
typedef struct {
    double aos_unix;     // acquisition of signal (rises above the horizon mask)
    double los_unix;     // loss of signal (sets below it)
    double peak_unix;    // time of maximum elevation
    double peak_el_deg;  // maximum elevation this pass
    double peak_az_deg;  // azimuth at maximum elevation
} pass_t_wire;

typedef struct {
    char   satellite[64];   // common name, e.g. "ISS (ZARYA)"
    char   idesg[9];        // international designation (matches sso_event_t)
    double generated_unix;  // when the producer computed this schedule (monotonic)
    double tle_epoch_min;   // minutes_since_epoch of the TLE used (staleness hint)
    int    count;           // number of valid entries in passes[]
    pass_t_wire passes[PASS_SCHED_MAX];
} pass_schedule_t;

// Encode to one newline-terminated JSON line. Returns 0 on success, -1 on
// overflow. Mirrors sso_event_encode's contract.
int pass_schedule_encode(const pass_schedule_t *s, char *out, size_t out_size);

// Decode one {"t":"passes",...} line (with or without trailing newline) into
// out. Returns 0 on success, -1 on parse error / not a passes line.
int pass_schedule_decode(const char *line, pass_schedule_t *out);

#ifdef __cplusplus
}
#endif

#endif // PASS_SCHEDULE_H
