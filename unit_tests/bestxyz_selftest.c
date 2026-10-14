/*

    Simple Satellite Operations  unit_tests/bestxyz_selftest.c

    Tests for the NovAtel OEM7 BESTXYZA parser (src/orbit/bestxyz.c). The
    fixture is the exact log the team handed us for the novatel_edie
    conversion, so the parsed fields are checked against the values their
    Python to_dict() produced, plus the GPS-week/seconds epoch, the
    message CRC, and the leading-wrapper and corruption paths.

    The binary BESTXYZB parser is checked on two samples FrontierSat's GNSS
    ring blob sent down on 2026-10-13. The receiver's own CRC proves the
    framing, and the fix's size, speed and timing are checked against the
    orbit and the reception time, which a wrong field offset would break.

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
#include <stdio.h>
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

// Two GNSS ring-blob packets (type 0x30) as received, from the packet
// database: one type byte, the 16-bit downlink sequence number and ring
// position, then the 144-byte BESTXYZB log. RING_FIX holds the fix taken at
// 2026-10-13T21:03:08Z and arrived at 21:46:40.624Z; SGP4 from the TLE then
// in force put the satellite within 0.63 km of the fix. RING_WARMUP is from
// just after the receiver was switched on, before it had a solution.
static const char *RING_FIX =
    "3002150900AA44121CF10000207000000097B48809306DD10E00000002CF44FA"
    "430000000010000000D997697AC2644BC1C0DAEC89545A42C1697CE5EABB6854"
    "C1C623833F97588A3F02A7F53F00000000080000005DC71782DFDAB6C0BB3B6F"
    "DF8C0D9AC0141EF516DE4CB24025D7123D7C01033DAC536E3D00000000000000"
    "000000000000000000271F1F1E00063533FD56DDC6";
static const char *RING_WARMUP =
    "303E159601AA44121CF100002070000000A4788809D8B1C10E00004802CF44FA"
    "4301000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000001000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000003EB2CA11";

#define RING_HDR 5      // type byte, sequence number, ring position
#define RING_LEN 149

static size_t unhex(const char *s, unsigned char *out, size_t cap)
{
    size_t n = 0;
    for (; s[0] && s[1] && n < cap; s += 2) {
        unsigned v = 0;
        sscanf(s, "%2x", &v);
        out[n++] = (unsigned char) v;
    }
    return n;
}

static void test_binary_fields(void)
{
    unsigned char pkt[RING_LEN];
    check(unhex(RING_FIX, pkt, sizeof pkt) == RING_LEN, "fixture is one 149-byte packet");

    bestxyz_t b;
    char err[160];
    check(bestxyzb_parse(pkt + RING_HDR, RING_LEN - RING_HDR, &b, err, sizeof err) == 0,
          "parse the downlinked BESTXYZB");
    // The receiver computed this CRC over the header and body; matching it
    // means the header and body lengths were read correctly.
    check(b.crc_present == 1 && b.crc_ok == 1, "receiver CRC matches");
    check(b.crc_read == 0xc6dd56fdu, "CRC read from the trailing four bytes");

    check(b.gps_week == 2440, "GPS week");
    check(approx(b.gps_sow, 248606.0, 1e-6), "GPS seconds of week");
    check(strcmp(b.time_status, "FINESTEERING") == 0, "time status 180");
    check(strcmp(b.pos_sol_status, "SOL_COMPUTED") == 0, "position solution status 0");
    check(strcmp(b.pos_type, "SINGLE") == 0, "position type 16");
    check(approx(b.pos[0], -3590532.9563, 1e-3), "ECEF X");
    check(approx(b.pos[1], -2405545.0775, 1e-3), "ECEF Y");
    check(approx(b.pos[2], -5350127.6703, 1e-3), "ECEF Z");
    check(approx(b.pos_sigma[0], 1.024529, 1e-5), "X sigma");
    check(approx(b.pos_sigma[2], 1.919159, 1e-5), "Z sigma");
    check(strcmp(b.vel_sol_status, "SOL_COMPUTED") == 0, "velocity solution status");
    check(strcmp(b.vel_type, "DOPPLER_VELOCITY") == 0, "velocity type 8");
    check(approx(b.vel[0], -5850.8731, 1e-3), "ECEF VX");
    check(approx(b.vel[2], 4684.8675, 1e-3), "ECEF VZ");
    check(approx(b.vel_sigma[1], 0.031984, 1e-5), "VY sigma");
    check(b.num_sv == 39 && b.num_sol_sv == 31, "39 satellites tracked, 31 used");

    // Checks that do not lean on the values above. FrontierSat flies a
    // near-circular orbit about 500 km up, so the fix sits 6700-7000 km
    // from the Earth's centre, moves at 7.0-7.8 km/s in the Earth-fixed
    // frame, and its velocity is almost square to its position (the Earth's
    // rotation adds only a component perpendicular to the position).
    double r = sqrt(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1] + b.pos[2] * b.pos[2]);
    double v = sqrt(b.vel[0] * b.vel[0] + b.vel[1] * b.vel[1] + b.vel[2] * b.vel[2]);
    double rv = (b.pos[0] * b.vel[0] + b.pos[1] * b.vel[1] + b.pos[2] * b.vel[2]) / (r * v);
    check(r > 6.7e6 && r < 7.0e6, "position is at orbit radius");
    check(v > 7.0e3 && v < 7.8e3, "velocity is orbital speed");
    check(fabs(rv) < 0.01, "velocity is square to the position");

    // The fix was taken before the packet came down. The blob sends a random
    // run from its ring of the latest 500 fixes, 10 s apart while the
    // receiver is on (sunlight only), so a sample is at most a few hours old.
    int y, mo, d, h, mi;
    double s;
    bestxyz_gps_to_utc(b.gps_week, b.gps_sow, BESTXYZ_DEFAULT_LEAP_SECONDS,
                       &y, &mo, &d, &h, &mi, &s);
    double fix_s = h * 3600.0 + mi * 60.0 + s;
    double rx_s = 21 * 3600.0 + 46 * 60.0 + 40.624;
    check(y == 2026 && mo == 10 && d == 13, "fix is on the reception day");
    check(fix_s < rx_s && rx_s - fix_s < 3 * 3600.0, "fix precedes reception by under 3 h");

    // The warm-up sample: no solution yet, so the enumerations differ.
    check(unhex(RING_WARMUP, pkt, sizeof pkt) == RING_LEN, "warm-up fixture is 149 bytes");
    check(bestxyzb_parse(pkt + RING_HDR, RING_LEN - RING_HDR, &b, err, sizeof err) == 0
          && b.crc_ok == 1, "parse the warm-up sample, CRC matches");
    check(strcmp(b.time_status, "COARSESTEERING") == 0, "time status 120");
    check(strcmp(b.pos_sol_status, "INSUFFICIENT_OBS") == 0, "position solution status 1");
    check(strcmp(b.pos_type, "NONE") == 0, "position type 0");
    check(b.gps_week == 2440 && approx(b.gps_sow, 247575.0, 1e-6), "warm-up GPS time");
}

static void test_binary_errors(void)
{
    unsigned char pkt[RING_LEN];
    unhex(RING_FIX, pkt, sizeof pkt);
    unsigned char *m = pkt + RING_HDR;
    size_t n = RING_LEN - RING_HDR;
    bestxyz_t b;
    char err[160];

    // A flipped body byte still parses but fails the CRC.
    m[40] ^= 0x01;
    check(bestxyzb_parse(m, n, &b, err, sizeof err) == 0 && b.crc_ok == 0,
          "flipped body byte fails the CRC");
    m[40] ^= 0x01;

    check(bestxyzb_parse(m, n - 1, &b, err, sizeof err) == -1, "one byte short is rejected");
    check(bestxyzb_parse(pkt, n, &b, err, sizeof err) == -1, "missing sync is rejected");
    // Message ID 42 is BESTPOS: the same framing, a different body.
    m[4] = 42;
    check(bestxyzb_parse(m, n, &b, err, sizeof err) == -1, "another log ID is rejected");
}

int main(void)
{
    test_fields();
    test_epoch();
    test_wrapper_and_errors();
    test_binary_fields();
    test_binary_errors();
    return tap_done();
}
