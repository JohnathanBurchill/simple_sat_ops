/*

    Simple Satellite Operations  ui/duration_fmt.h

    Compact human-readable duration / age formatters shared by the
    predictions panel (and unit-tested in isolation). Pure C, no ncurses.

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

#ifndef SSO_UI_DURATION_FMT_H
#define SSO_UI_DURATION_FMT_H

#include <stddef.h>

// Format a duration (seconds) as a compact "Dd Hh Mm Ss" string, emitting
// only the parts that are needed: "2s", "1h 12s", "3d 4h", etc. Leading
// (and interior) zero units are dropped, so "1h 0m 12s" -> "1h 12s". A
// duration that rounds to zero renders as "0s". Seconds are rounded to the
// nearest whole second; a negative input is clamped to zero. The output is
// always NUL-terminated when n > 0.
void format_duration_compact(double seconds, char *out, size_t n);

// Format an age (seconds) as a compact "Mmo Dd Hh Mm" string: months,
// days, hours, minutes, dropping leading (and interior) zero units, so a
// few-hours-old TLE reads "3h 12m" rather than "0mo 0d 3h 12m". A month is
// a flat 30 days -- this is an elapsed span (e.g. since a TLE epoch), not a
// gap between two calendar dates, so there is no calendar month to anchor
// to. Seconds are not shown; an age under a minute renders as "0m". A
// negative input is clamped to zero. The output is always NUL-terminated
// when n > 0.
void format_age_compact(double seconds, char *out, size_t n);

#endif
