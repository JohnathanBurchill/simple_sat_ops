/*

    Simple Satellite Operations  unit_tests/timea_selftest.c

    Tests for the NovAtel OEM7 TIMEA parser (src/orbit/bestxyz.c). TIMEA is
    the log that carries the receiver clock offset (absent from BESTXYZA) and
    the live GPS-UTC leap-second count, so gnss_opm pairs one with a fix to
    nail the epoch.

    The primary fixture is a genuine log: its NovAtel CRC validates (so the
    bytes are real, not hand-typed), AND its own UTC body fields are an
    independent, receiver-produced statement of what GPS week+sow converts to.
    That makes it a true external oracle for both the parser and the GPS->UTC
    direction -- subtract the leap seconds, never add them.

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
*/

#include "bestxyz.h"
#include "tap.h"

#include <math.h>
#include <string.h>

#define check(cond, what) tap_ok((cond), (what))

static int approx(double a, double b, double tol) { return fabs(a - b) <= tol; }

// Genuine TIMEA: CRC 2a066e78 validates over the message span, and the log's
// own UTC fields read 2017-01-05 22:58:50.000 -- which is exactly GPS week
// 1930, sow 428348 (Thu 22:59:08 GPS) minus the 18 s leap. From the flight
// firmware's unit_test_gnss.c.
static const char *LOG =
    "#TIMEA,COM1,0,86.5,FINESTEERING,1930,428348.000,02000020,9924,32768;"
    "VALID,1.667187222e-10,9.641617960e-10,-18.00000000000,"
    "2017,1,5,22,58,50000,VALID*2a066e78";

static void test_fields(void)
{
    timea_t t;
    char err[160];
    check(timea_parse(LOG, &t, err, sizeof err) == 0, "parse the genuine TIMEA log");

    check(t.gps_week == 1930, "GPS week");
    check(approx(t.gps_sow, 428348.0, 1e-6), "GPS seconds of week");
    check(strcmp(t.time_status, "FINESTEERING") == 0, "time status");

    check(strcmp(t.clock_status, "VALID") == 0, "clock status VALID");
    check(approx(t.clock_offset, 1.667187222e-10, 1e-18), "clock offset");
    check(approx(t.clock_offset_std, 9.641617960e-10, 1e-18), "clock offset sigma");
    check(approx(t.utc_offset, -18.0, 1e-9), "utc offset (signed UTC-GPS)");
    check(strcmp(t.utc_status, "VALID") == 0, "utc status VALID");

    check(t.crc_present == 1, "CRC present");
    check(t.crc_read == 0x2a066e78u, "CRC read from message");
    check(t.crc_ok == 1, "genuine CRC validates");
}

// The leap seconds the correction uses come from the TIMEA utc offset, which
// is signed UTC-GPS: leap = -utc_offset. A sign slip here would yield -18.
static void test_leap_from_utc_offset(void)
{
    timea_t t;
    char err[160];
    check(timea_parse(LOG, &t, err, sizeof err) == 0, "parse for leap derivation");
    int leap = (int)lround(-t.utc_offset);
    check(leap == 18, "leap seconds = -utc_offset = 18");
}

// Cross-check against the log's own UTC fields. This is the load-bearing
// direction test: GPS week 1930 sow 428348 with leap 18 must land on
// 2017-01-05 22:58:50, exactly what the receiver wrote in the same log.
static void test_utc_cross_check(void)
{
    timea_t t;
    char err[160];
    check(timea_parse(LOG, &t, err, sizeof err) == 0, "parse for UTC cross-check");

    int leap = (int)lround(-t.utc_offset);
    int y, mo, d, h, mi; double s;
    // Apply the clock offset exactly as gnss_opm does (sow - offset), then leap.
    bestxyz_gps_to_utc(t.gps_week, t.gps_sow - t.clock_offset, leap,
                       &y, &mo, &d, &h, &mi, &s);
    check(y == 2017, "cross-check year");
    check(mo == 1, "cross-check month");
    check(d == 5, "cross-check day");
    check(h == 22, "cross-check hour");
    check(mi == 58, "cross-check minute");
    check(approx(s, 50.0, 1e-3), "cross-check second matches the log's own UTC field");

    // Direction guard: with no leap applied we get GPS time 22:59:08, i.e.
    // exactly 18 s later. If the code ever added the leap instead, the result
    // would move the wrong way and this relationship would break.
    int y0, mo0, d0, h0, mi0; double s0;
    bestxyz_gps_to_utc(t.gps_week, t.gps_sow, 0, &y0, &mo0, &d0, &h0, &mi0, &s0);
    check(h0 == 22 && mi0 == 59 && approx(s0, 8.0, 1e-3),
          "leap=0 yields GPS time 22:59:08 (UTC is 18 s earlier, so leap subtracts)");
}

static void test_wrapper_and_errors(void)
{
    // A leading command-response wrapper must be skipped.
    char wrapped[512];
    snprintf(wrapped, sizeof wrapped, "<OK [COM1]\r\n%s", LOG);
    timea_t t;
    char err[160];
    check(timea_parse(wrapped, &t, err, sizeof err) == 0, "parse with leading wrapper");
    check(t.gps_week == 1930 && t.crc_ok == 1, "wrapped parse yields the same fields and CRC");

    // A flipped body byte must fail the CRC but still parse. This is also a
    // hand-edited copy whose original CRC no longer matches.
    char bad[512];
    snprintf(bad, sizeof bad, "%s", LOG);
    char *p = strstr(bad, "VALID,1.667");
    check(p != NULL, "locate the clock-offset field in fixture");
    p[6] = (p[6] == '1') ? '2' : '1';   // perturb the first offset digit
    check(timea_parse(bad, &t, err, sizeof err) == 0, "corrupted log still parses");
    check(t.crc_present == 1 && t.crc_ok == 0, "corrupted log fails CRC");

    // A negative clock offset parses with the right sign (separate fixture;
    // its CRC is a placeholder, so we only check the field, not crc_ok).
    const char *neg =
        "#TIMEA,COM1,0,80.0,FINESTEERING,2367,411646.000,02040000,9924,17402;"
        "VALID,-4.474795457e-08,9.384349250e-09,-18.00000000000,"
        "2025,5,22,21,0,28526,VALID*deadbeef";
    check(timea_parse(neg, &t, err, sizeof err) == 0, "parse negative-offset TIMEA");
    check(approx(t.clock_offset, -4.474795457e-08, 1e-16), "negative clock offset sign preserved");

    // Non-TIMEA input is rejected.
    check(timea_parse("hello, world\n", &t, err, sizeof err) == -1,
          "non-TIMEA input rejected");
    check(timea_parse("#TIMEA,no,semicolon,here", &t, err, sizeof err) == -1,
          "header without ';' rejected");
}

int main(void)
{
    test_fields();
    test_leap_from_utc_offset();
    test_utc_cross_check();
    test_wrapper_and_errors();
    return tap_done();
}
