/*

   Simple Satellite Operations  sso_time.h

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

// Small timing helpers shared by the operator loop and the TX modals.
// Header-only (static inline) so the split-out translation units can each
// pull them in without a link dependency.

#ifndef SSO_TIME_H
#define SSO_TIME_H

#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// Monotonic clock in nanoseconds. Used for the modal edit-debounce timers.
static inline long ts_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long) ts.tv_sec * 1000000000L + (long) ts.tv_nsec;
}

// Monotonic clock in seconds. The main loop runs at the SDR-chunk cadence
// (~120 Hz with a B210 attached); slow-cadence work (IPC broadcast, ncurses
// redraw, rotator) is timestamp-gated against this so it stays at its
// historical 2 Hz / 10 Hz rates.
static inline double monotonic_seconds(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

// Current UTC in milliseconds -- the queue-time clock for expanding an
// "SSO+..." pseudo-command (see sso_pseudo.h). Captured fresh per send so
// each transmission carries a current time.
static inline long long sso_now_utc_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long) tv.tv_sec * 1000LL + tv.tv_usec / 1000LL;
}

// Format a timestamp as "YYYY-MM-DDThh:mm:ss.mmmZ" (UTC, millisecond
// precision) into out. Shared by the IPC codec's per-event timestamps and
// the audit log so the two formats never drift.
static inline void sso_iso_utc_from_ts(const struct timespec *ts,
                                        char *out, size_t out_size)
{
    struct tm tm;
    gmtime_r(&ts->tv_sec, &tm);
    snprintf(out, out_size,
             "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             (long) (ts->tv_nsec / 1000000));
}

#ifdef __cplusplus
}
#endif

#endif // SSO_TIME_H
