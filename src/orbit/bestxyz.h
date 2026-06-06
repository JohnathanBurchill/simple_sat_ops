/*

    Simple Satellite Operations  bestxyz.h

    Parser for a NovAtel OEM7 BESTXYZA log -- the ASCII Earth-fixed
    position/velocity solution FrontierSat's GNSS receiver returns in a
    telecommand response. Pure C, no external library: the equivalent of
    the team's novatel_edie Python snippet, just enough of it to recover
    the state vector and its time so tle_from_state can build a TLE.

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

#ifndef SSO_BESTXYZ_H
#define SSO_BESTXYZ_H

#include <stddef.h>

// The GPS-UTC offset in whole seconds (GPS time runs ahead of UTC).
// 18 since 2017-01-01; bump this if a new leap second is introduced.
#define BESTXYZ_DEFAULT_LEAP_SECONDS 18

// One parsed BESTXYZA log. Distances are metres, velocities metres per
// second, both in the Earth-fixed (ECEF) frame the receiver reports.
typedef struct {
    int    gps_week;            // GPS week number (header field 6)
    double gps_sow;             // GPS seconds into the week (header field 7)
    char   time_status[24];     // e.g. FINESTEERING, FREEWHEELING

    char   pos_sol_status[24];  // e.g. SOL_COMPUTED, INSUFFICIENT_OBS
    char   pos_type[24];        // e.g. SINGLE, NARROW_INT
    double pos[3];              // ECEF X,Y,Z (m)
    double pos_sigma[3];        // 1-sigma X,Y,Z (m)

    char   vel_sol_status[24];
    char   vel_type[24];        // e.g. DOPPLER_VELOCITY
    double vel[3];              // ECEF VX,VY,VZ (m/s)
    double vel_sigma[3];        // 1-sigma VX,VY,VZ (m/s)
    double vel_latency;         // velocity time-tag offset (s)
    double sol_age;             // solution age (s)
    int    num_sv;              // satellites tracked
    int    num_sol_sv;          // satellites used in the solution

    unsigned crc_read;          // CRC printed after '*' (0 if absent)
    unsigned crc_calc;          // CRC we computed over the message
    int      crc_present;       // 1 if the log carried a *CRC
    int      crc_ok;            // 1 if crc_read == crc_calc
} bestxyz_t;

// Parse the first BESTXYZA log found anywhere in text (a leading command
// wrapper like "<OK [COM1]" or other surrounding bytes are skipped). On
// success returns 0 and fills *out; on failure returns -1 and writes a
// short reason into err (when errsz > 0).
int bestxyz_parse(const char *text, bestxyz_t *out, char *err, size_t errsz);

// Convert a GPS week + seconds-of-week to UTC, applying leap_seconds.
// Fills broken-down UTC using this codebase's convention (full year,
// month 1-12) plus the fractional second. Pass leap_seconds as
// BESTXYZ_DEFAULT_LEAP_SECONDS unless overridden.
void bestxyz_gps_to_utc(int gps_week, double gps_sow, int leap_seconds,
                        int *year, int *mon, int *day,
                        int *hh, int *mm, double *ss);

#endif
