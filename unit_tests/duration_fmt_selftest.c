/*

    Simple Satellite Operations  unit_tests/duration_fmt_selftest.c

    Tests for the compact duration / age formatters shown in the
    predictions panel:
      - format_duration_compact() -- the "next pass (in ...)" countdown,
        "Dd Hh Mm Ss" with leading/interior zero units dropped.
      - format_age_compact()      -- the TLE "epoch age", "Mmo Dd Hh Mm"
        (a month is a flat 30 days; no seconds).

    The contract under test:
      - Only nonzero units print; leading AND interior zero units are
        dropped (3601 s -> "1h 1s", NOT "1h 0m 1s").
      - A value that rounds below the smallest shown unit still prints
        that unit as zero ("0s" / "0m"), never the empty string.
      - Seconds (duration) / minutes (age) round to nearest; negatives
        clamp to zero.
      - Output is always NUL-terminated and never overruns the buffer,
        even when the result is truncated.

    Two independent oracles guard against a formatter that is merely
    self-consistent:
      1. Hand-computed golden strings (exact spacing and labels), written
         out by a human, not by re-running the breakdown arithmetic.
      2. A parse-back check: re-parse the emitted text token-by-token back
         into a total and compare to the rounded input. This re-derives
         the magnitude through a different code path (text scan + a
         separate unit->multiplier table), so a wrong divisor, a swapped
         field, or a mislabelled unit makes the totals disagree even when
         the golden set happens to miss the case.

    Exit status: 0 = all tests passed, non-zero = failure.

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
#include "tap.h"

#include <ctype.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Independent parse-back oracle.
//
// Re-read a formatted string "<num><unit> <num><unit> ..." into a single
// total, using a caller-supplied unit table. This deliberately does NOT
// reuse the formatter's day/hour/min breakdown -- it walks the text and
// looks each unit up in a flat table -- so it catches the formatter using
// the wrong divisor or label. Returns -1 on an unknown unit or malformed
// token.
struct unit_mult { const char *name; long mult; };

static long parse_back(const char *s, const struct unit_mult *units)
{
    long total = 0;
    const char *p = s;
    while (*p) {
        while (*p == ' ') p++;
        if (!*p) break;
        if (!isdigit((unsigned char) *p)) return -1;
        long num = 0;
        while (isdigit((unsigned char) *p)) {
            num = num * 10 + (*p - '0');
            p++;
        }
        char unit[4];
        size_t u = 0;
        while (isalpha((unsigned char) *p)) {
            if (u + 1 >= sizeof unit) return -1;
            unit[u++] = *p++;
        }
        unit[u] = '\0';
        if (u == 0) return -1;
        long mult = -1;
        for (const struct unit_mult *um = units; um->name; um++) {
            if (strcmp(um->name, unit) == 0) { mult = um->mult; break; }
        }
        if (mult < 0) return -1;
        total += num * mult;
    }
    return total;
}

// Duration units are seconds; age units are minutes (months = 30 days).
static const struct unit_mult DUR_UNITS[] = {
    {"d", 86400}, {"h", 3600}, {"m", 60}, {"s", 1}, {NULL, 0}
};
static const struct unit_mult AGE_UNITS[] = {
    {"mo", 30L * 1440L}, {"d", 1440}, {"h", 60}, {"m", 1}, {NULL, 0}
};

// A tiny seeded LCG -- a constant seed keeps the run reproducible (no
// wall-clock entropy), per the project's test conventions.
static uint32_t rng_state = 0x9e3779b9u;
static uint32_t rng_next(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

// ---------------------------------------------------------------------------

static void check_dur(double secs, const char *want)
{
    char got[40];
    format_duration_compact(secs, got, sizeof got);
    tap_okf(strcmp(got, want) == 0,
            "duration %.1fs -> \"%s\" (got \"%s\")", secs, want, got);
}

static void check_age(double secs, const char *want)
{
    char got[40];
    format_age_compact(secs, got, sizeof got);
    tap_okf(strcmp(got, want) == 0,
            "age %.0fs -> \"%s\" (got \"%s\")", secs, want, got);
}

int main(void)
{
    // ---- format_duration_compact: hand-computed golden strings ----------
    check_dur(0.0,      "0s");        // zero -> the smallest unit, not ""
    check_dur(0.4,      "0s");        // rounds down to 0
    check_dur(0.6,      "1s");        // rounds up
    check_dur(1.0,      "1s");
    check_dur(59.0,     "59s");
    check_dur(59.6,     "1m");        // rounds to 60 -> carries to a minute
    check_dur(60.0,     "1m");
    check_dur(61.0,     "1m 1s");
    check_dur(119.0,    "1m 59s");
    check_dur(120.0,    "2m");
    check_dur(600.0,    "10m");
    check_dur(3599.0,   "59m 59s");
    check_dur(3600.0,   "1h");
    check_dur(3601.0,   "1h 1s");     // interior zero MINUTE dropped
    check_dur(3660.0,   "1h 1m");
    check_dur(3661.0,   "1h 1m 1s");
    check_dur(7200.0,   "2h");
    check_dur(86399.0,  "23h 59m 59s");
    check_dur(86400.0,  "1d");
    check_dur(86461.0,  "1d 1m 1s");  // interior zero HOUR dropped
    check_dur(90000.0,  "1d 1h");
    check_dur(-5.0,     "0s");        // negative clamps to zero

    // ---- format_age_compact: hand-computed golden strings ---------------
    check_age(0.0,        "0m");      // zero -> "0m", never ""
    check_age(20.0,       "0m");      // sub-minute rounds to 0
    check_age(60.0,       "1m");
    check_age(3540.0,     "59m");     // 59 min
    check_age(3600.0,     "1h");
    check_age(3660.0,     "1h 1m");
    check_age(7200.0,     "2h");
    check_age(86400.0,    "1d");      // 1440 min
    check_age(86460.0,    "1d 1m");   // interior zero HOUR dropped
    check_age(90000.0,    "1d 1h");
    check_age(2592000.0,  "1mo");     // 30 days exactly
    check_age(2595600.0,  "1mo 1h");  // 30d + 1h: interior zero DAYS dropped
    check_age(2678400.0,  "1mo 1d");  // 31 days
    check_age(8215500.0,  "3mo 5d 2h 5m"); // 136925 min, all four units
    check_age(-5.0,       "0m");      // negative clamps to zero

    // ---- parse-back oracle over a wide seeded sweep ---------------------
    // Re-parsing the output and summing must reproduce the rounded input.
    // A wrong divisor / label / dropped unit makes these disagree.
    int dur_roundtrip_ok = 1, age_roundtrip_ok = 1;
    long dur_bad = -1, age_bad = -1;
    for (int i = 0; i < 5000; i++) {
        // Span sub-second up to ~46 days for durations.
        long secs = (long) (rng_next() % 4000000u);
        char buf[40];
        format_duration_compact((double) secs, buf, sizeof buf);
        if (parse_back(buf, DUR_UNITS) != secs) {
            dur_roundtrip_ok = 0; dur_bad = secs; break;
        }
    }
    for (int i = 0; i < 5000; i++) {
        // Span minutes up to ~7 years for ages (input is seconds, but the
        // formatter only resolves to the minute, so feed whole minutes to
        // compare exactly against parse-back-in-minutes).
        long mins = (long) (rng_next() % 4000000u);
        char buf[40];
        format_age_compact((double) mins * 60.0, buf, sizeof buf);
        if (parse_back(buf, AGE_UNITS) != mins) {
            age_roundtrip_ok = 0; age_bad = mins; break;
        }
    }
    tap_okf(dur_roundtrip_ok,
            "duration parse-back matches input over 5000 cases (first bad: %ld s)",
            dur_bad);
    tap_okf(age_roundtrip_ok,
            "age parse-back matches input over 5000 cases (first bad: %ld min)",
            age_bad);

    // ---- truncation safety: never overrun, always a NUL-terminated -----
    // ---- prefix of the untruncated result ------------------------------
    {
        char full[40];
        format_duration_compact(90061.0, full, sizeof full); // "1d 1h 1m 1s"
        tap_okf(strcmp(full, "1d 1h 1m 1s") == 0,
                "truncation reference is \"1d 1h 1m 1s\" (got \"%s\")", full);
        int trunc_ok = 1;
        for (size_t n = 1; n <= strlen(full) + 1; n++) {
            char small[40];
            memset(small, '#', sizeof small);
            format_duration_compact(90061.0, small, n);
            // Within the buffer: NUL-terminated and a prefix of the full
            // string. Past index n-1 must be untouched ('#').
            if (strnlen(small, n) >= n) { trunc_ok = 0; break; }
            if (strncmp(small, full, strlen(small)) != 0) { trunc_ok = 0; break; }
            if (small[n] != '#') { trunc_ok = 0; break; }
        }
        tap_ok(trunc_ok, "duration truncation stays in bounds and prefix-correct");
    }

    // n == 0 must not write a single byte.
    {
        char guard[2] = { '#', '#' };
        format_duration_compact(123.0, guard, 0);
        format_age_compact(123.0, guard, 0);
        tap_ok(guard[0] == '#' && guard[1] == '#',
               "n == 0 writes nothing");
    }

    return tap_done();
}
