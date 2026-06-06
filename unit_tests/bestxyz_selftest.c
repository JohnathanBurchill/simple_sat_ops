/*

    Simple Satellite Operations  unit_tests/bestxyz_selftest.c

    Tests for the NovAtel OEM7 BESTXYZA parser (src/orbit/bestxyz.c). The
    fixture is the exact log the team handed us for the novatel_edie
    conversion, so the parsed fields are checked against the values their
    Python to_dict() produced, plus the GPS-week/seconds epoch, the
    message CRC, and the leading-wrapper and corruption paths.

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

// The team's example BESTXYZA log (the bytes after the "<OK [COM1]" wrapper).
static const char *LOG =
    "#BESTXYZA,COM1,0,53.5,FREEWHEELING,2421,510103.000,02488000,44cf,17402;"
    "INSUFFICIENT_OBS,SINGLE,1618093.1208,1369036.9069,6530652.8463,"
    "38.2289,6.5930,54.6648,INSUFFICIENT_OBS,DOPPLER_VELOCITY,"
    "7184.8868,1708.6610,-2130.7723,0.6078,0.1605,0.6073,"
    "\"\",0.000,0.000,324.200,0,0,0,0,0,00,00,00*0bf1610e\r\n";

static void test_fields(void)
{
    bestxyz_t b;
    char err[160];
    check(bestxyz_parse(LOG, &b, err, sizeof err) == 0, "parse the example log");

    check(b.gps_week == 2421, "GPS week");
    check(approx(b.gps_sow, 510103.0, 1e-6), "GPS seconds of week");
    check(strcmp(b.time_status, "FREEWHEELING") == 0, "time status");

    check(strcmp(b.pos_sol_status, "INSUFFICIENT_OBS") == 0, "position solution status");
    check(strcmp(b.pos_type, "SINGLE") == 0, "position type");
    check(approx(b.pos[0], 1618093.1208, 1e-4), "ECEF X");
    check(approx(b.pos[1], 1369036.9069, 1e-4), "ECEF Y");
    check(approx(b.pos[2], 6530652.8463, 1e-4), "ECEF Z");
    check(approx(b.pos_sigma[0], 38.2289, 1e-4), "X sigma");
    check(approx(b.pos_sigma[1], 6.5930, 1e-4), "Y sigma");
    check(approx(b.pos_sigma[2], 54.6648, 1e-4), "Z sigma");

    check(strcmp(b.vel_sol_status, "INSUFFICIENT_OBS") == 0, "velocity solution status");
    check(strcmp(b.vel_type, "DOPPLER_VELOCITY") == 0, "velocity type");
    check(approx(b.vel[0], 7184.8868, 1e-4), "ECEF VX");
    check(approx(b.vel[1], 1708.6610, 1e-4), "ECEF VY");
    check(approx(b.vel[2], -2130.7723, 1e-4), "ECEF VZ");
    check(approx(b.vel_sigma[0], 0.6078, 1e-4), "VX sigma");
    check(approx(b.vel_sigma[1], 0.1605, 1e-4), "VY sigma");
    check(approx(b.vel_sigma[2], 0.6073, 1e-4), "VZ sigma");

    check(approx(b.vel_latency, 0.0, 1e-9), "velocity latency");
    check(approx(b.sol_age, 324.200, 1e-3), "solution age");

    check(b.crc_present == 1, "CRC present");
    check(b.crc_read == 0x0bf1610eu, "CRC read from message");
    check(b.crc_ok == 1, "computed CRC matches");
}

static void test_epoch(void)
{
    // GPS week 2421 + 510103 s, leap 18 -> 2026-06-05T21:41:25Z.
    int y, mo, d, h, mi;
    double s;
    bestxyz_gps_to_utc(2421, 510103.0, 18, &y, &mo, &d, &h, &mi, &s);
    check(y == 2026, "epoch year");
    check(mo == 6, "epoch month");
    check(d == 5, "epoch day");
    check(h == 21, "epoch hour");
    check(mi == 41, "epoch minute");
    check(approx(s, 25.0, 1e-3), "epoch second");

    // GPS epoch itself: week 0, sow 0, leap 0 -> 1980-01-06T00:00:00Z.
    bestxyz_gps_to_utc(0, 0.0, 0, &y, &mo, &d, &h, &mi, &s);
    check(y == 1980 && mo == 1 && d == 6 && h == 0 && mi == 0 && approx(s, 0.0, 1e-6),
          "GPS epoch maps to 1980-01-06T00:00:00Z");
}

static void test_wrapper_and_errors(void)
{
    // A leading command-response wrapper must be skipped.
    char wrapped[1024];
    snprintf(wrapped, sizeof wrapped, "<OK [COM1]\r\n%s", LOG);
    bestxyz_t b;
    char err[160];
    check(bestxyz_parse(wrapped, &b, err, sizeof err) == 0, "parse with leading wrapper");
    check(b.gps_week == 2421 && approx(b.pos[0], 1618093.1208, 1e-4),
          "wrapped parse yields the same fields");

    // A flipped body byte must fail the CRC but still parse.
    char bad[1024];
    snprintf(bad, sizeof bad, "%s", LOG);
    char *p = strstr(bad, "*0bf1610e");
    check(p != NULL, "locate CRC in fixture");
    p[-1] = (p[-1] == '0') ? '1' : '0';   // perturb the last body char
    check(bestxyz_parse(bad, &b, err, sizeof err) == 0, "corrupted log still parses");
    check(b.crc_present == 1 && b.crc_ok == 0, "corrupted log fails CRC");

    // Non-BESTXYZA input is rejected.
    check(bestxyz_parse("hello, world\n", &b, err, sizeof err) == -1,
          "non-BESTXYZA input rejected");
    check(bestxyz_parse("#BESTXYZA,no,semicolon,here", &b, err, sizeof err) == -1,
          "header without ';' rejected");
}

int main(void)
{
    test_fields();
    test_epoch();
    test_wrapper_and_errors();
    return tap_done();
}
