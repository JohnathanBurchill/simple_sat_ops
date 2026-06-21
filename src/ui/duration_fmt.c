/*

    Simple Satellite Operations  ui/duration_fmt.c

    Compact human-readable duration / age formatters. See duration_fmt.h
    for the contract. Pulled out of panels.c so they can be unit-tested
    without dragging in ncurses and the whole state struct.

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

#include "duration_fmt.h"

#include <stdio.h>

void format_duration_compact(double seconds, char *out, size_t n)
{
    if (n == 0) return;
    if (seconds < 0) seconds = 0;
    long total = (long) (seconds + 0.5);
    long days  =  total / 86400;
    long hours = (total % 86400) / 3600;
    long mins  = (total % 3600) / 60;
    long secs  =  total % 60;

    // snprintf returns the length it WOULD have written, so off can run past
    // n on truncation (or wrap huge from a negative return). Clamp after each
    // append so the next "n - off" can't underflow and out + off stays in
    // bounds, independent of the off < n guards.
    size_t off = 0;
    if (days > 0) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldd",
                                 off ? " " : "", days);
        if (off >= n) off = n;
    }
    if (hours > 0 && off < n) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldh",
                                 off ? " " : "", hours);
        if (off >= n) off = n;
    }
    if (mins > 0 && off < n) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldm",
                                 off ? " " : "", mins);
        if (off >= n) off = n;
    }
    // Show seconds when nonzero, or when nothing else was emitted (so a
    // sub-second / zero duration still prints "0s").
    if ((secs > 0 || off == 0) && off < n) {
        snprintf(out + off, n - off, "%s%lds", off ? " " : "", secs);
    }
}

void format_age_compact(double seconds, char *out, size_t n)
{
    if (n == 0) return;
    if (seconds < 0) seconds = 0;
    long total_min = (long) (seconds / 60.0 + 0.5);
    long months =  total_min / (30L * 1440L);
    long days   = (total_min % (30L * 1440L)) / 1440L;
    long hours  = (total_min % 1440L) / 60L;
    long mins   =  total_min % 60L;

    // Same clamp-after-append discipline as format_duration_compact: off can
    // run past n on truncation, so pin it before the next "n - off".
    size_t off = 0;
    if (months > 0) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldmo",
                                 off ? " " : "", months);
        if (off >= n) off = n;
    }
    if (days > 0 && off < n) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldd",
                                 off ? " " : "", days);
        if (off >= n) off = n;
    }
    if (hours > 0 && off < n) {
        off += (size_t) snprintf(out + off, n - off, "%s%ldh",
                                 off ? " " : "", hours);
        if (off >= n) off = n;
    }
    // Minutes when nonzero, or when nothing else was emitted (so a
    // sub-minute / zero age still prints "0m").
    if ((mins > 0 || off == 0) && off < n) {
        snprintf(out + off, n - off, "%s%ldm", off ? " " : "", mins);
    }
}
