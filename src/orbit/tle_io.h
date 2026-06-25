/*

   Simple Satellite Operations  tle_io.h

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

// The two line-level primitives for reading a 3-line TLE: pull the next
// non-blank line (trimming trailing CR/LF and spaces/tabs), and test whether
// a line is a "1 " or "2 " element card. These were written out identically
// in pass_session.c, tracking.c, and tle_keps.c.
//
// Deliberately just the line primitives, not a whole-record reader: the
// callers' record loops differ on purpose (first-record-only, by-name match,
// read-every-object with a "0 " 3LE prefix and bare two-line fallback), and
// the file-discovery strategies above them are three distinct policies
// (newest by mtime, newest dated filename, the FrontierSat day-directory
// layout), so folding those together would relocate working code without
// removing any real duplication. prediction.c and rx_replay.c keep their own
// line readers because they trim only CR/LF (not trailing spaces), which the
// by-name prefix match and the offline replay path depend on.
//
// Header-only (static inline) so each translation unit pulls it in with no
// link dependency.

#ifndef SSO_ORBIT_TLE_IO_H
#define SSO_ORBIT_TLE_IO_H

#include <stdio.h>
#include <string.h>

// Read the next non-blank line from f into buf, trimming trailing CR, LF,
// spaces, and tabs. Blank lines are skipped. Returns 1 on a line, 0 at end
// of file. buf must hold the longest expected line plus a terminator.
static inline int tle_io_read_line(FILE *f, char *buf, size_t cap)
{
    while (fgets(buf, (int) cap, f) != NULL) {
        size_t n = strlen(buf);
        while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r'
                      || buf[n - 1] == ' '  || buf[n - 1] == '\t')) {
            buf[--n] = '\0';
        }
        if (n == 0) continue;
        return 1;
    }
    return 0;
}

// True if the line is the TLE element card given by `card` ('1' or '2'),
// i.e. it begins with that digit followed by a space.
static inline int tle_io_is_element_line(const char *s, char card)
{
    return s[0] == card && s[1] == ' ';
}

#endif // SSO_ORBIT_TLE_IO_H
