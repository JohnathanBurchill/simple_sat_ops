/*

    Simple Satellite Operations  unit_tests/adcs_mag_selftest.c

    Exercises src/proto/adcs_mag.c: pulling a magnetic field vector out of an
    ADCS telecommand-response JSON, the |B| magnitude, the TLE-epoch to
    unix-ms conversion, and the closest-epoch search that mag_reports uses to
    tie a reading to a TLE. The Unix-time constants below are independent
    golden values (worked out by hand), so a mistake in the conversion goes
    red rather than agreeing with itself.

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

#include "tap.h"
#include "adcs_mag.h"

#include <math.h>
#include <stdint.h>

// ---- adcs_mag_parse --------------------------------------------------------

static void test_parse(void)
{
    adcs_mag_vec_t v;

    // The bare measured/IGRF field object.
    tap_ok(adcs_mag_parse("{\"x_nT\":12345,\"y_nT\":-6789,\"z_nT\":23456}", &v) == 1,
           "nT object parses");
    tap_ok(v.kind == ADCS_MAG_NT, "nT object is ADCS_MAG_NT (shape can't tell measured from IGRF)");
    tap_ok(v.x_nT == 12345 && v.y_nT == -6789 && v.z_nT == 23456,
           "nT object values incl. negative y");

    // adcs_measurements: the measured field sits among many other sensors.
    // The magnetic_field_*_nT branch must win and pick the right three.
    const char *meas =
        "{\"magnetic_field_x_nT\":111,\"magnetic_field_y_nT\":222,"
        "\"magnetic_field_z_nT\":-333,\"coarse_sun_x_micro\":7,"
        "\"sun_x_micro\":8,\"x_angular_rate_mdeg_per_sec\":9,"
        "\"x_wheel_speed_rpm\":10}";
    tap_ok(adcs_mag_parse(meas, &v) == 1, "measurements blob parses");
    tap_ok(v.kind == ADCS_MAG_MEASUREMENTS, "measurements blob is ADCS_MAG_MEASUREMENTS");
    tap_ok(v.x_nT == 111 && v.y_nT == 222 && v.z_nT == -333,
           "measurements blob picks the magnetic_field_*_nT triple");

    // The raw magnetometer object: bare {"x":,"y":,"z":} counts.
    tap_ok(adcs_mag_parse("{\"x\":13461,\"y\":13544,\"z\":-7077}", &v) == 1,
           "raw counts object parses");
    tap_ok(v.kind == ADCS_MAG_RAW, "bare {x,y,z} is ADCS_MAG_RAW (counts, not nT)");
    tap_ok(v.x_nT == 13461 && v.y_nT == 13544 && v.z_nT == -7077,
           "raw counts values");

    // A calibrated nT object must classify as NT, not RAW: the "x" key match
    // must not fire inside "x_nT".
    tap_ok(adcs_mag_parse("{\"x_nT\":7,\"y_nT\":8,\"z_nT\":9}", &v) == 1 && v.kind == ADCS_MAG_NT,
           "nT object is NT, never misread as raw");

    // A sun/nadir vector ({"x_micro":..}) is NOT a magnetic field -- the
    // quoted-key match must not confuse "x_micro" for "x_nT" or bare "x".
    tap_ok(adcs_mag_parse("{\"x_micro\":1,\"y_micro\":2,\"z_micro\":3}", &v) == 0,
           "sun/nadir micro vector is not a field reading");
    tap_ok(v.kind == ADCS_MAG_NONE, "no-field parse leaves kind NONE");

    // An unrelated ACK-style response.
    tap_ok(adcs_mag_parse("ADCS telemetry request failed (err 3)", &v) == 0,
           "non-JSON error string yields no reading");

    // A partial object (missing z) must fail, not return garbage.
    tap_ok(adcs_mag_parse("{\"x_nT\":1,\"y_nT\":2}", &v) == 0,
           "incomplete nT object (no z) fails");
}

// ---- adcs_mag_parse_telem_hex (generic-telemetry frame response) -----------

static void test_telem_hex(void)
{
    adcs_mag_vec_t v;

    // The primary path: frame 151's hex byte dump. Bytes are little-endian
    // int16 per axis, x10 nT. Golden values worked out by hand from the wire
    // bytes 55 0a / 08 f4 / 6a 01: 0x0a55=2645, 0xf408=-3064, 0x016a=362.
    tap_ok(adcs_mag_parse_telem_hex("55 a 8 f4 6a 1", 10, &v) == 1,
           "frame hex dump parses (6 bytes)");
    tap_ok(v.kind == ADCS_MAG_NT, "frame hex dump is calibrated nT");
    tap_ok(v.x_nT == 26450 && v.y_nT == -30640 && v.z_nT == 3620,
           "frame 151 decode: little-endian int16 x10, signed y");

    // The [END_RESPONSE] marker (and any trailing junk) must not corrupt the
    // read: '[' is not a hex digit, so the scan stops cleanly after 6 bytes.
    tap_ok(adcs_mag_parse_telem_hex("55 a 8 f4 6a 1 \n[END_RESPONSE]", 10, &v) == 1
           && v.x_nT == 26450 && v.z_nT == 3620,
           "trailing [END_RESPONSE] and newline are ignored");

    // Leading whitespace/newline before the dump is skipped.
    tap_ok(adcs_mag_parse_telem_hex("\n   c 0 21 0 ef ff", 10, &v) == 1
           && v.x_nT == 120 && v.y_nT == 330 && v.z_nT == -170,
           "leading whitespace skipped; negative z sign-extends");

    // The scale argument is applied verbatim (1 nT/LSB leaves raw int16).
    tap_ok(adcs_mag_parse_telem_hex("c 0 21 0 ef ff", 1, &v) == 1
           && v.x_nT == 12 && v.y_nT == 33 && v.z_nT == -17,
           "nt_per_lsb=1 yields the raw int16 counts");

    // Fewer than 6 bytes is not a 3-axis vector.
    tap_ok(adcs_mag_parse_telem_hex("55 a 8 f4", 10, &v) == 0,
           "short dump (4 bytes) fails");
    tap_ok(v.kind == ADCS_MAG_NONE, "failed hex parse leaves kind NONE");

    // A 3-hex-digit token is not a byte: reject rather than misread.
    tap_ok(adcs_mag_parse_telem_hex("555 a 8 f4 6a 1", 10, &v) == 0,
           "over-long hex token (3 digits) rejected");

    // Empty / non-hex bodies.
    tap_ok(adcs_mag_parse_telem_hex("", 10, &v) == 0, "empty body fails");
    tap_ok(adcs_mag_parse_telem_hex("[END_RESPONSE]", 10, &v) == 0,
           "no leading bytes fails");
}

// ---- adcs_generic_telem_frame (frame id from the command) ------------------

static void test_telem_frame(void)
{
    // The real command form, with the @-suffixes the ground appends.
    tap_ok(adcs_generic_telem_frame(
               "CTS1+adcs_generic_telemetry_request(151,6)@tssent=1@tsexec=1!o") == 151,
           "frame id 151 parsed from a real command");
    tap_ok(adcs_generic_telem_frame("CTS1+adcs_generic_telemetry_request(159,6)!x") == 159,
           "frame id 159 (IGRF)");
    // A space after the '(' or comma is tolerated (strtol skips leading space).
    tap_ok(adcs_generic_telem_frame("adcs_generic_telemetry_request( 151, 6)") == 151,
           "space after '(' tolerated");

    // Not a generic telemetry request -> -1.
    tap_ok(adcs_generic_telem_frame("CTS1+adcs_magnetic_field_vector()!x") == -1,
           "named field command is not a generic telemetry request");
    // The bare name with no argument list (e.g. a bulk-log \"tcmd\" value).
    tap_ok(adcs_generic_telem_frame("adcs_generic_telemetry_request") == -1,
           "bare command name (no parens) yields -1");
    tap_ok(adcs_generic_telem_frame(NULL) == -1, "NULL yields -1");
}

// ---- adcs_mag_magnitude ----------------------------------------------------

static void test_magnitude(void)
{
    adcs_mag_vec_t a = { ADCS_MAG_NT, 3, 4, 0 };
    tap_ok(fabs(adcs_mag_magnitude(&a) - 5.0) < 1e-9, "|(3,4,0)| == 5");

    adcs_mag_vec_t b = { ADCS_MAG_NT, 0, 0, -100 };
    tap_ok(fabs(adcs_mag_magnitude(&b) - 100.0) < 1e-9, "|(0,0,-100)| == 100 (sign-agnostic)");

    adcs_mag_vec_t c = { ADCS_MAG_NT, 0, 0, 0 };
    tap_ok(adcs_mag_magnitude(&c) == 0.0, "|(0,0,0)| == 0");

    // Large components: 40000^2 overflows int32, so a magnitude formed in
    // int would be wrong here. Reference: sqrt(3)*40000 = 69282.0323...
    adcs_mag_vec_t big = { ADCS_MAG_NT, 40000, 40000, 40000 };
    tap_ok(fabs(adcs_mag_magnitude(&big) - 69282.0323) < 0.01,
           "|(40000,40000,40000)| ~ 69282 (no integer overflow)");
}

// ---- adcs_tle_epoch_unix_ms ------------------------------------------------

static void test_epoch(void)
{
    int64_t ms;
    // 2026-01-01T00:00:00Z is 1767225600 s since the Unix epoch (worked out
    // by hand: 2025-01-01 = 1735689600, + 365 d for the non-leap 2025).
    tap_ok(adcs_tle_epoch_unix_ms(2026, 1.0, &ms) == 0 && ms == 1767225600000LL,
           "epoch (2026, day 1.0) -> 2026-01-01T00:00Z");
    tap_ok(adcs_tle_epoch_unix_ms(26, 1.0, &ms) == 0 && ms == 1767225600000LL,
           "two-digit year 26 widens to 2026 (NORAD pivot <57 -> 2000s)");
    tap_ok(adcs_tle_epoch_unix_ms(99, 1.0, &ms) == 0 && ms == 915148800000LL,
           "two-digit year 99 widens to 1999 (>=57 -> 1900s)");

    // Day-of-year is 1-based: day 2.5 is 1.5 days past Jan 1 00:00.
    tap_ok(adcs_tle_epoch_unix_ms(2026, 2.5, &ms) == 0 && ms == 1767355200000LL,
           "epoch (2026, day 2.5) is 1.5 days after Jan 1");

    tap_ok(adcs_tle_epoch_unix_ms(2026, 0.5, &ms) == -1, "day < 1 rejected");
    tap_ok(adcs_tle_epoch_unix_ms(2026, 400.0, &ms) == -1, "day >= 367 rejected");
    tap_ok(adcs_tle_epoch_unix_ms(1900, 1.0, &ms) == -1, "pre-satellite year rejected");
}

// ---- adcs_closest_index ----------------------------------------------------

static void test_closest(void)
{
    int64_t e[] = { 1000, 5000, 9000 };
    tap_ok(adcs_closest_index(e, 3, 4000) == 1, "4000 closest to 5000 (gap 1000 < 3000)");
    tap_ok(adcs_closest_index(e, 3, 100) == 0, "before all -> first");
    tap_ok(adcs_closest_index(e, 3, 100000) == 2, "after all -> last");

    int64_t tie[] = { 0, 200 };
    tap_ok(adcs_closest_index(tie, 2, 100) == 0, "exact tie -> earlier index");

    tap_ok(adcs_closest_index(e, 0, 4000) == -1, "empty set -> -1");
}

int main(void)
{
    test_parse();
    test_telem_hex();
    test_telem_frame();
    test_magnitude();
    test_epoch();
    test_closest();
    return tap_done();
}
