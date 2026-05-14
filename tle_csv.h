/*

    Simple Satellite Operations  tle_csv.h

    Celestrak switched their bulk satellite element lists from the
    classic 3-line TLE format (name + Line 1 + Line 2) to an OMM-style
    CSV with the header:

      OBJECT_NAME,OBJECT_ID,EPOCH,MEAN_MOTION,ECCENTRICITY,INCLINATION,
      RA_OF_ASC_NODE,ARG_OF_PERICENTER,MEAN_ANOMALY,EPHEMERIS_TYPE,
      CLASSIFICATION_TYPE,NORAD_CAT_ID,ELEMENT_SET_NO,REV_AT_EPOCH,
      BSTAR,MEAN_MOTION_DOT,MEAN_MOTION_DDOT

    The rest of the codebase still wants classic 3-line TLEs (the
    sgp4sdp4 ports, find_passes, load_tle, read_tle_lines, ...). This
    module is the single funnel that converts CSV input to a 3-line
    tempfile so callers don't have to care which format they got.

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

#ifndef TLE_CSV_H
#define TLE_CSV_H

#include <stddef.h>

// Resolves `raw` to a path readable as a classic 3-line TLE.
//
// If `raw` is NULL/empty or already looks like a classic TLE, the same
// pointer is returned unchanged. If `raw` is a Celestrak OMM CSV (first
// non-blank line begins with "OBJECT_NAME"), each row is converted to
// classic name+L1+L2 form and written to a per-process tempfile. The
// returned string then points at the tempfile. Calls with the same raw
// path return the same tempfile (cached). Tempfiles are unlinked at
// process exit via atexit().
//
// On any conversion error the function logs to stderr and returns `raw`
// unchanged, so upstream load_tle / find_passes will fail with their
// usual diagnostics.
//
// Returned pointer is owned by tle_csv (string interned in a static
// table) or borrowed from `raw`; do not free or mutate.
char *tle_path_resolve(const char *raw);

#endif  // TLE_CSV_H
