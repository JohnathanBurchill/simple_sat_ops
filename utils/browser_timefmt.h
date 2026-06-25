/*

   Simple Satellite Operations  browser_timefmt.h

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

// Timestamp rendering shared by the ncurses packet/command browsers
// (packet_browser, tcmd_browser). Both show stored ISO-8601 UTC strings
// from packet_db and humanize unix-millisecond ts_sent values, with an
// 'l' key that flips the column between UTC and the host's local zone.
//
// Header-only (static inline) so each browser pulls it in without a link
// dependency. Both browsers live in utils/, so the same-directory rule
// for #include "..." finds this header with no extra include path.

#ifndef BROWSER_TIMEFMT_H
#define BROWSER_TIMEFMT_H

#include <stdint.h>
#include <stdio.h>
#include <time.h>

// Humanize a unix-millisecond instant as "YYYY-MM-DDTHH:MM:SSZ" (UTC,
// second resolution). Used for group headers and time windows.
static inline void fmt_epoch_ms(uint64_t ms, char *out, size_t outn)
{
    time_t t = (time_t)(ms / 1000);
    struct tm tm;
    if (gmtime_r(&t, &tm) == NULL) { snprintf(out, outn, "?"); return; }
    strftime(out, outn, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

// Render a stored ISO-8601 UTC timestamp into the chosen display mode.
// local==0 is a passthrough (the string is already UTC). local!=0 parses
// the "YYYY-MM-DDTHH:MM:SS[.fff]Z" form back to a time_t (via timegm) and
// re-formats in the host's local zone as "YYYY-MM-DD HH:MM:SS.mmm TZ".
// Anything that doesn't match the pattern falls through unchanged so the
// operator can still see what the column actually contains.
static inline void format_ts(const char *iso, int local, char *out, size_t outn)
{
    if (iso == NULL || iso[0] == '\0') {
        if (outn > 0) out[0] = '\0';
        return;
    }
    if (!local) {
        snprintf(out, outn, "%s", iso);
        return;
    }
    int yr, mo, dd, hh, mm, ss, ms = 0;
    int got = sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2d.%3d",
                     &yr, &mo, &dd, &hh, &mm, &ss, &ms);
    if (got < 6) {
        snprintf(out, outn, "%s", iso);
        return;
    }
    struct tm utc = {0};
    utc.tm_year = yr - 1900;
    utc.tm_mon  = mo - 1;
    utc.tm_mday = dd;
    utc.tm_hour = hh;
    utc.tm_min  = mm;
    utc.tm_sec  = ss;
    time_t epoch = timegm(&utc);
    if (epoch == (time_t)-1) {
        snprintf(out, outn, "%s", iso);
        return;
    }
    struct tm local_tm;
    localtime_r(&epoch, &local_tm);
    char base[40];
    strftime(base, sizeof base, "%Y-%m-%d %H:%M:%S", &local_tm);
    const char *tz = tzname[local_tm.tm_isdst > 0 ? 1 : 0];
    if (tz == NULL) tz = "";
    snprintf(out, outn, "%s.%03d %s", base, ms, tz);
}

#endif // BROWSER_TIMEFMT_H
