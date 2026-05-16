/*

    Simple Satellite Operations  oem.h

    CCSDS OEM (Orbit Ephemeris Message) parser and sampler. Populated
    from `ssm trajectory <id>` stdout. State is in ITRF (Earth-fixed)
    Cartesian km, km/s; time in UTC (represented as Julian Date).

    Used by next_in_queue to plan passes against a propagated trajectory
    stored in the SpaceX Space Safety API, without going through TLEs.

    Copyright (C) 2025  Johnathan K Burchill

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

#ifndef OEM_H
#define OEM_H

#include <stddef.h>

typedef struct oem_sample {
    double jul_utc;      // Julian Date, UTC
    double r_ecef[3];    // km, ITRF
    double v_ecef[3];    // km/s, ITRF
} oem_sample_t;

typedef struct oem_table {
    char object_name[64];
    char object_id[64];
    char ref_frame[32];
    char time_system[16];
    double start_jul_utc;
    double stop_jul_utc;
    oem_sample_t *samples;
    size_t n_samples;
    size_t capacity;
} oem_table_t;

// Parse an OEM document from a null-terminated string buffer.
// Returns 0 on success, -1 on parse error (stderr diagnostic).
// On success, `out` owns a malloc'd samples array; free via oem_free().
int oem_parse(const char *text, oem_table_t *out);

// Runs `ssm trajectory <trajectory_id>` via popen(), slurps stdout,
// and parses the result. Returns 0 on success, -1 on error.
int oem_load_from_ssm(const char *trajectory_id, oem_table_t *out);

// Interpolate state at the given Julian Date via cubic Hermite using
// bracketing samples (position + velocity as derivative). Returns 0 on
// success, -1 if jul_utc is outside [start_jul_utc, stop_jul_utc].
int oem_sample_at(const oem_table_t *t, double jul_utc,
                  double r_ecef[3], double v_ecef[3]);

void oem_free(oem_table_t *t);

#endif // OEM_H
