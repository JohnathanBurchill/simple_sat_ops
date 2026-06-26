/*

    Simple Satellite Operations  src/proto/adcs_mag.h

    Pure helpers for the magnetometer reporter (mag_reports): pull a magnetic
    field vector out of an ADCS telecommand-response JSON, compute its
    magnitude, turn a TLE epoch into a Unix timestamp, and find the TLE whose
    epoch sits closest in time to a measurement. No I/O, no SGP4, no sqlite --
    everything here is exercised by unit_tests/adcs_mag_selftest.c.

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

#ifndef ADCS_MAG_H
#define ADCS_MAG_H

#include <stdint.h>

// Which JSON shape a magnetic field vector came out of. The firmware emits
// the SAME {"x_nT":..} object for both the measured magnetometer field
// (adcs_magnetic_field_vector) and the IGRF model field
// (adcs_igrf_magnetic_field_vector), so the JSON shape alone cannot tell
// them apart -- only the originating command can. ADCS_MAG_MEASUREMENTS is
// the larger adcs_measurements object, whose magnetic_field_*_nT fields are
// always the measured field.
typedef enum {
    ADCS_MAG_NONE = 0,        // no recognizable field vector in the text
    ADCS_MAG_NT,              // {"x_nT":..,"y_nT":..,"z_nT":..}  (calibrated nT)
    ADCS_MAG_MEASUREMENTS,    // adcs_measurements: magnetic_field_{x,y,z}_nT
    ADCS_MAG_RAW,             // {"x":..,"y":..,"z":..}  (raw counts, uncalibrated)
} adcs_mag_kind_t;

typedef struct {
    adcs_mag_kind_t kind;
    long x_nT, y_nT, z_nT;
} adcs_mag_vec_t;

// Scan a NUL-terminated, reassembled telecommand-response for a magnetic
// field vector in nanotesla. Prefers adcs_measurements' magnetic_field_*_nT
// (unambiguously measured) over the bare {"x_nT":..} object. Returns 1 and
// fills *out on success, 0 if no field vector is present (out->kind set to
// ADCS_MAG_NONE).
int adcs_mag_parse(const char *text, adcs_mag_vec_t *out);

// CubeADCS telemetry frames this tool decodes as a magnetic field vector, and
// the firmware's count->nanotesla scale (adcs_struct_packers.c).
#define ADCS_TELEM_FRAME_MAG_FIELD  151   // CubeACP measured magnetic field
#define ADCS_TELEM_FRAME_IGRF       159   // IGRF model field (same wire format)
#define ADCS_TELEM_NT_PER_LSB        10   // each int16 count is 10 nT

// Decode a CubeADCS generic-telemetry-frame response body. Unlike the named
// commands' JSON, a frame response is a whitespace-separated hex byte dump
// (e.g. "55 a 8 f4 6a 1"; bytes are not zero-padded), little-endian int16 per
// axis, which the firmware scales by nt_per_lsb to nanotesla (10 for the
// magnetic field frame 151 and the IGRF model frame 159). Reads the first
// three int16 (6 bytes); stops cleanly at the first non-hex token, so the
// trailing "[END_RESPONSE]" marker is ignored. Returns 1 and fills *out
// (kind = ADCS_MAG_NT) when at least 6 hex bytes are present, 0 otherwise.
int adcs_mag_parse_telem_hex(const char *body, int nt_per_lsb, adcs_mag_vec_t *out);

// The telemetry frame id of a generic-telemetry command string -- the first
// argument of adcs_generic_telemetry_request(<id>,<len>). Tolerates a space
// after the '(' or comma ("(151, 6)"). Returns the id, or -1 if the text is
// not a generic telemetry request (e.g. has no parenthesized argument list).
int adcs_generic_telem_frame(const char *command);

// Euclidean magnitude |B| in nT. Components reach a few tens of thousands of
// nT, so the squares are formed in double to avoid integer overflow.
double adcs_mag_magnitude(const adcs_mag_vec_t *v);

// Convert a TLE epoch -- 2- or 4-digit year plus a 1-based fractional
// day-of-year (so day 1.0 is Jan 1 00:00:00Z) -- to Unix milliseconds.
// Returns 0 on success, -1 on out-of-range input.
int adcs_tle_epoch_unix_ms(int epoch_year, double epoch_day, int64_t *out_ms);

// Index into epochs_ms[] of the entry closest in absolute time to target_ms.
// Returns -1 if n <= 0. On ties the earlier index wins.
int adcs_closest_index(const int64_t *epochs_ms, int n, int64_t target_ms);

#endif // ADCS_MAG_H
