/*

    Simple Satellite Operations  unit_tests/beacon_series_selftest.c

    Coverage for src/beacon/beacon_series.{c,h} -- the catalogue that
    says where each telemetry field sits in a beacon and how to turn its
    bytes into a number. telemetry_browser plots whatever this table
    says, so a wrong offset here draws a wrong curve with no warning:
    the plot looks like data either way.

    The values expected below come from the same real extended beacon
    the beacon_cts1 selftest uses (row 638383 of the operational packet
    database, 2026-09-18T01:28:34Z), read out of its bytes by hand with
    a shell script before this code existed. The catalogue's offsets
    come from the structs via offsetof, so what is really under test
    here is that each entry names the field it claims to.

    What's covered:
      - every entry's offset and width land inside the packet it belongs
        to, and no two entries share a key.
      - a spread of fields across the whole packet read the hand-decoded
        values: battery voltage, charge, both battery thermistors, the
        OBC temperature, uptime in hours, the solar channels, the field
        vector and its magnitude, the rates, and the attitude angles.
      - the unit scaling: millivolts to volts, hundredths of a degree to
        degrees, centiwatts to watts, milliseconds to hours.
      - a basic beacon yields the shared fields and refuses the
        extended-only ones.
      - every "no reading" marker -- the blob's -9999 and -99999, a dead
        thermistor's INT16_MAX, the MPI's -99 -- is refused rather than
        returned as a number.
      - a field that needs the ADCS is refused when the ADCS was silent,
        and one that needs an attitude is refused outside estimation
        modes 3 to 6.
      - the derived entries: the field magnitude against the components,
        the unpacked modes, and the fault count.
      - lookup by key, and a truncated payload.

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

#include "beacon_cts1.h"
#include "beacon_series.h"
#include "tap.h"

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// The same real v4 extended beacon the beacon_cts1 selftest carries.
static const char EXT_HEX[] =
    "204354533102009EB34D046E5F980166FA21B2A00100000501D7002900730D0000"
    "0102701701000000F13D60A902BA02C60E0000A30000000D000000340000001000"
    "00003600000046050000060201020002000248656C6C6F2066726F6D2043616C67"
    "617279546F53706163652046726F6E7469657253617400000000002058340019463"
    "D0D86221D001300EF28F0FF0300E42506000600E425000006000080BA0004014531"
    "8700000005047B0D050C080BDB0403F0BAFB3A00F7FFFCFF0000B11A7F0A460E";

static size_t ext_fixture(uint8_t *buf, size_t cap)
{
    const char *h = EXT_HEX;
    size_t n = 0;
    while (h[0] != '\0' && h[1] != '\0' && n < cap) {
        unsigned v = 0;
        if (sscanf(h, "%2x", &v) != 1) break;
        buf[n++] = (uint8_t) v;
        h += 2;
    }
    return n;
}

// A basic beacon, for the "this field is in both" and "this one is not"
// halves of the catalogue. Values are distinct from the extended
// fixture's so a test cannot pass by reading the wrong packet.
static size_t basic_fixture(uint8_t *buf)
{
    COMMS_beacon_basic_packet_t b;
    memset(&b, 0, sizeof b);
    b.packet_type = COMMS_PACKET_TYPE_BEACON_BASIC;
    memcpy(b.satellite_name, "CTS1", 4);
    b.uptime_ms = 7200000;                 // exactly 2 hours
    b.eps_battery_voltage_mV = 7654;
    b.eps_battery_percent = 42;
    b.eps_battery_temperature_0_cC = -1250;
    b.obc_temperature_cC = 2345;
    b.eps_total_pcu_power_input_cW = 777;
    memcpy(b.end_message, "END", 4);
    memcpy(buf, &b, sizeof b);
    return sizeof b;
}

static double must_read(const char *key, const uint8_t *p, size_t n, int *ok)
{
    const bs_field_t *f = beacon_series_find(key);
    double v = 0.0;
    *ok = (f != NULL) && beacon_series_value(f, p, n, &v);
    return v;
}

static void test_catalogue_shape(void)
{
    tap_okf(BEACON_SERIES_FIELD_N > 40,
            "catalogue: holds %zu fields", BEACON_SERIES_FIELD_N);

    // Every entry has to fit in the packet it belongs to, and the
    // derived entries need six bytes from their offset.
    int bad_bounds = 0, bad_text = 0;
    for (size_t i = 0; i < BEACON_SERIES_FIELD_N; i++) {
        const bs_field_t *f = &BEACON_SERIES_FIELDS[i];
        size_t w = 1;
        switch (f->type) {
            case BS_U16: case BS_I16: w = 2; break;
            case BS_U32: case BS_I32: w = 4; break;
            case BS_MAG_MAGNITUDE:
            case BS_ADCS_EST_MODE: case BS_ADCS_CTRL_MODE:
            case BS_ADCS_RUN_MODE: case BS_ADCS_SUN_UP:
            case BS_ADCS_FAULT_COUNT: w = 6; break;
            default: w = 1; break;
        }
        const size_t limit = f->ext_only
            ? sizeof(COMMS_beacon_extended_packet_t)
            : sizeof(COMMS_beacon_basic_packet_t);
        if (f->off + w > limit) {
            printf("# %s runs past the end of its packet (%u + %zu > %zu)\n",
                   f->key, f->off, w, limit);
            bad_bounds++;
        }
        if (f->key == NULL || f->key[0] == '\0'
            || f->label == NULL || f->label[0] == '\0'
            || f->unit == NULL || f->scale == 0.0) bad_text++;
    }
    tap_okf(bad_bounds == 0,
            "catalogue: every field fits inside its packet (%d that do not)",
            bad_bounds);
    tap_okf(bad_text == 0,
            "catalogue: every field has a key, a label, a unit and a scale");

    // Duplicate keys would make --fields and the CSV header ambiguous.
    int dups = 0;
    for (size_t i = 0; i < BEACON_SERIES_FIELD_N; i++)
        for (size_t k = i + 1; k < BEACON_SERIES_FIELD_N; k++)
            if (strcmp(BEACON_SERIES_FIELDS[i].key,
                       BEACON_SERIES_FIELDS[k].key) == 0) dups++;
    tap_okf(dups == 0, "catalogue: no two fields share a key (%d clashes)", dups);

    // Every group has a name, and no group's name is the fallback.
    int unnamed = 0;
    for (int g = 0; g < BS_GROUP_N; g++)
        if (strcmp(beacon_series_group_name((bs_group_t) g), "?") == 0) unnamed++;
    tap_okf(unnamed == 0, "catalogue: every group is named");

    tap_ok(beacon_series_find("batt_v") != NULL,
           "lookup: a key that exists is found");
    tap_ok(beacon_series_find("no_such_field") == NULL,
           "lookup: a key that does not exist is not");
    tap_ok(beacon_series_find(NULL) == NULL, "lookup: NULL is safe");
}

static void test_values_from_the_real_beacon(void)
{
    uint8_t p[256];
    const size_t n = ext_fixture(p, sizeof p);
    tap_okf(n == sizeof(COMMS_beacon_extended_packet_t),
            "fixture: %zu bytes", n);

    // Each expected value was read out of these bytes by hand. The
    // tolerance is the last digit of the packet's own resolution --
    // a battery voltage is a whole millivolt, so 1e-9 V of slack is
    // there for the binary representation of 15.857 and nothing else.
    const struct { const char *key; double want; double eps; } cases[] = {
        { "batt_v",       15.857,    1e-9  },
        { "batt_pct",     96.0,      0.0   },
        { "batt_t0",      6.81,      1e-9  },
        { "batt_t1",      6.98,      1e-9  },
        { "obc_t",        13.50,     1e-9  },
        { "pcu_in",       0.13,      1e-9  },
        { "pcu_out",      0.52,      1e-9  },
        { "pcu_avg_in",   0.16,      1e-9  },
        { "pcu_avg_out",  0.54,      1e-9  },
        { "eps_faults",   3782.0,    0.0   },
        { "eps_mode",     1.0,       0.0   },
        { "uptime",       72201118.0 / 3600000.0, 1e-9 },
        { "eps_uptime",   71536.0 / 3600.0,       1e-9 },
        { "beacon_count", 3443.0,    0.0   },
        { "tcmd_queued",  215.0,     0.0   },
        { "tcmd_pending", 41.0,      0.0   },
        { "op_state",     2.0,       0.0   },
        { "antenna",      2.0,       0.0   },
        { "osc",          25.0,      0.0   },
        { "obc_batt_v",   15.686,    1e-9  },
        { "mpi_t",        13.0,      0.0   },
        { "solar0_v",     8.838,     1e-9  },
        { "solar0_i",     29.0,      0.0   },
        { "solar1_v",     10.479,    1e-9  },
        { "solar1_i",    -16.0,      0.0   },   // negative, so the sign is tested
        { "solar2_v",     9.700,     1e-9  },
        { "solar3_i",     0.0,       0.0   },
        { "net_batt_w",   1.86,      1e-9  },
        { "distrib_w",    2.60,      1e-9  },
        { "pack_status",  32768.0,   0.0   },
        { "est_mode",     5.0,       0.0   },
        { "ctrl_mode",    4.0,       0.0   },
        { "run_mode",     1.0,       0.0   },
        { "sun_up",       1.0,       0.0   },
        { "adcs_faults",  0.0,       0.0   },
        { "mag_x",        12.43,     1e-9  },
        { "mag_y",       -40.93,     1e-9  },
        { "mag_z",       -10.94,     1e-9  },
        { "css1",         5.0,       0.0   },
        { "css3",         123.0,     0.0   },
        { "css9",         11.0,      0.0   },
        { "rate_norm",    0.58,      1e-9  },
        { "rate_x",      -0.09,      1e-9  },
        { "rate_y",      -0.04,      1e-9  },
        { "rate_z",       0.0,       0.0   },
        { "roll",         68.33,     1e-9  },
        { "pitch",        26.87,     1e-9  },
        { "yaw",          36.54,     1e-9  },
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        int ok = 0;
        const double got = must_read(cases[i].key, p, n, &ok);
        tap_okf(ok && fabs(got - cases[i].want) <= cases[i].eps + 1e-12,
                "value: %s is %.6g (want %.6g)%s",
                cases[i].key, got, cases[i].want, ok ? "" : " [not read]");
    }

    // The magnitude has to agree with the three components it is built
    // from -- an independent check of the derived entry against the
    // plain ones.
    int ok_x = 0, ok_y = 0, ok_z = 0, ok_m = 0;
    const double bx = must_read("mag_x", p, n, &ok_x);
    const double by = must_read("mag_y", p, n, &ok_y);
    const double bz = must_read("mag_z", p, n, &ok_z);
    const double bm = must_read("mag_norm", p, n, &ok_m);
    tap_okf(ok_x && ok_y && ok_z && ok_m
            && fabs(bm - sqrt(bx * bx + by * by + bz * bz)) < 1e-9,
            "value: mag_norm %.4f is the length of (%.2f, %.2f, %.2f)",
            bm, bx, by, bz);
    tap_okf(bm > 40.0 && bm < 50.0,
            "value: and %.2f uT is a low-Earth-orbit field strength", bm);
}

static void test_basic_beacon(void)
{
    uint8_t p[140];
    const size_t n = basic_fixture(p);

    const struct { const char *key; double want; } shared[] = {
        { "batt_v",   7.654  },
        { "batt_pct", 42.0   },
        { "batt_t0", -12.50  },
        { "obc_t",    23.45  },
        { "pcu_in",   7.77   },
        { "uptime",   2.0    },
    };
    for (size_t i = 0; i < sizeof shared / sizeof shared[0]; i++) {
        int ok = 0;
        const double got = must_read(shared[i].key, p, n, &ok);
        tap_okf(ok && fabs(got - shared[i].want) < 1e-9,
                "basic beacon: %s is %.6g (want %.6g)",
                shared[i].key, got, shared[i].want);
    }

    // The extended beacon's own fields are not in a basic one, and
    // asking for them must come back empty rather than reading whatever
    // byte happens to sit at that offset -- which for a 130-byte packet
    // would be off the end of it.
    const char *ext_only[] = { "osc", "solar0_v", "mag_x", "roll",
                               "est_mode", "mpi_t", "rate_norm" };
    int leaked = 0;
    for (size_t i = 0; i < sizeof ext_only / sizeof ext_only[0]; i++) {
        int ok = 0;
        must_read(ext_only[i], p, n, &ok);
        if (ok) { printf("# %s was read out of a basic beacon\n", ext_only[i]); leaked++; }
    }
    tap_okf(leaked == 0,
            "basic beacon: the extended beacon's own fields are refused (%d leaked)",
            leaked);

    // With the CSP CRC32 trailer left on, the shared fields still read.
    uint8_t withcrc[140];
    memcpy(withcrc, p, n);
    memcpy(withcrc + n, "\x19\x2e\x0d\x3d", 4);
    int ok = 0;
    const double v = must_read("batt_v", withcrc, n + 4, &ok);
    tap_okf(ok && fabs(v - 7.654) < 1e-9,
            "basic beacon: the trailer does not disturb the read");

    // ...and the extended beacon's first extra fields must still be
    // refused. This is the case that needs its own test: those fields
    // start at byte 130, which on a 134-byte basic beacon is exactly
    // where the CRC32 trailer sits. A reader that decided by length
    // alone would report the checksum's first byte as the oscillator
    // frequency and its next two as the OBC's battery voltage. The
    // trailer bytes above are chosen so those misreads would land on
    // plausible-looking numbers -- 25 MHz and 15.629 V, the values a
    // healthy satellite actually sends -- rather than on something a
    // reader would notice.
    const char *ext_at_the_seam[] = { "osc", "obc_batt_v", "mpi_t" };
    int seam_leaked = 0;
    for (size_t i = 0; i < sizeof ext_at_the_seam / sizeof ext_at_the_seam[0]; i++) {
        int got_ok = 0;
        const double got = must_read(ext_at_the_seam[i], withcrc, n + 4, &got_ok);
        if (got_ok) {
            printf("# %s read %.6g out of a basic beacon's CRC trailer\n",
                   ext_at_the_seam[i], got);
            seam_leaked++;
        }
    }
    tap_okf(seam_leaked == 0,
            "basic beacon: the extended fields at byte 130 are not read out "
            "of the CRC trailer (%d leaked)", seam_leaked);

    // Something that is not a beacon at all.
    uint8_t junk[64];
    memset(junk, 0xAB, sizeof junk);
    must_read("batt_v", junk, sizeof junk, &ok);
    tap_ok(!ok, "a payload that is not a beacon yields nothing");
    const bs_field_t *f = beacon_series_find("batt_v");
    double dummy = 0.0;
    tap_ok(beacon_series_value(f, NULL, 130, &dummy) == 0
           && beacon_series_value(NULL, p, n, &dummy) == 0
           && beacon_series_value(f, p, n, NULL) == 0,
           "NULL arguments are safe");
}

static void test_sentinels(void)
{
    uint8_t p[256];
    const size_t n = ext_fixture(p, sizeof p);
    uint8_t t[256];

    // The blob's -9999 placeholder for a failed EPS query, on a 16-bit
    // field: not -9.999 volts.
    memcpy(t, p, n);
    const size_t s0 = offsetof(COMMS_beacon_extended_packet_t,
                               eps_pcu_ch0_volt_in_mppt_mV);
    t[s0] = 0xF1; t[s0 + 1] = 0xD8;
    int ok = 0;
    must_read("solar0_v", t, n, &ok);
    tap_ok(!ok, "sentinel: -9999 on a solar channel is refused");
    // ...while its neighbour still reads, so the refusal is the value's
    // and not the whole packet's.
    const double v1 = must_read("solar1_v", t, n, &ok);
    tap_okf(ok && fabs(v1 - 10.479) < 1e-9,
            "sentinel: the next channel along still reads (%.3f V)", v1);

    // -99999 on a 32-bit power field.
    memcpy(t, p, n);
    const size_t pi = offsetof(COMMS_beacon_extended_packet_t,
                               eps_total_pcu_power_input_cW);
    t[pi] = 0x61; t[pi + 1] = 0x79; t[pi + 2] = 0xFE; t[pi + 3] = 0xFF;  // -99999
    must_read("pcu_in", t, n, &ok);
    tap_ok(!ok, "sentinel: -99999 on a PCU power is refused");

    // A dead thermistor pegs at INT16_MAX, which would plot as 327 C.
    memcpy(t, p, n);
    const size_t bt = offsetof(COMMS_beacon_extended_packet_t,
                               eps_battery_temperature_0_cC);
    t[bt] = 0xFF; t[bt + 1] = 0x7F;
    must_read("batt_t0", t, n, &ok);
    tap_ok(!ok, "sentinel: INT16_MAX on a thermistor is refused");
    t[bt] = 0x00; t[bt + 1] = 0x80;   // INT16_MIN
    must_read("batt_t0", t, n, &ok);
    tap_ok(!ok, "sentinel: INT16_MIN on a thermistor is refused too");

    // The MPI's three markers, and a real reading either side of them.
    memcpy(t, p, n);
    const size_t mt = offsetof(COMMS_beacon_extended_packet_t,
                               mpi_last_temperature_C);
    const struct { uint8_t raw; int want_ok; double want; } mpi[] = {
        { 0x9D, 0, 0.0    },   // -99, inactive
        { 0x9E, 0, 0.0    },   // -98, an error code
        { 0x9F, 0, 0.0    },   // -97, an error code
        { 0x9C, 1, -100.0 },   // -100: a real, if unlikely, reading
        { 0xE7, 1, -25.0  },   // -25
    };
    for (size_t i = 0; i < sizeof mpi / sizeof mpi[0]; i++) {
        t[mt] = mpi[i].raw;
        const double got = must_read("mpi_t", t, n, &ok);
        tap_okf(ok == mpi[i].want_ok
                && (!ok || fabs(got - mpi[i].want) < 1e-9),
                "sentinel: MPI temperature 0x%02x %s", mpi[i].raw,
                mpi[i].want_ok ? "reads as a temperature" : "is refused");
    }

    // The battery-pack status is 0xFFFF when the EPS did not answer.
    memcpy(t, p, n);
    const size_t bp = offsetof(COMMS_beacon_extended_packet_t,
                               eps_battery_pack_status_bitfield);
    t[bp] = 0xFF; t[bp + 1] = 0xFF;
    must_read("pack_status", t, n, &ok);
    tap_ok(!ok, "sentinel: 0xFFFF battery-pack status is refused");

    // The blob's own -9999 for a battery temperature, which sits
    // alongside the flight firmware's INT16_MAX for the same thing.
    memcpy(t, p, n);
    t[bt] = 0xF1; t[bt + 1] = 0xD8;
    must_read("batt_t0", t, n, &ok);
    tap_ok(!ok, "sentinel: -9999 on a battery temperature is refused too");

    // Zero battery voltage. The EPS leaves it there when it does not
    // answer, which is two beacons in three across the record -- and a
    // satellite whose pack really was at zero volts would not be
    // beaconing. Same for the charge percentage.
    memcpy(t, p, n);
    const size_t bv = offsetof(COMMS_beacon_extended_packet_t,
                               eps_battery_voltage_mV);
    t[bv] = 0x00; t[bv + 1] = 0x00;
    must_read("batt_v", t, n, &ok);
    tap_ok(!ok, "sentinel: a zero battery voltage is refused");
    memcpy(t, p, n);
    t[offsetof(COMMS_beacon_extended_packet_t, eps_battery_percent)] = 0;
    must_read("batt_pct", t, n, &ok);
    tap_ok(!ok, "sentinel: a zero battery percentage is refused");
    // One millivolt is not zero, and has to come through -- the test is
    // for the exact value, not for "small".
    memcpy(t, p, n);
    t[bv] = 0x01; t[bv + 1] = 0x00;
    const double mv = must_read("batt_v", t, n, &ok);
    tap_okf(ok && fabs(mv - 0.001) < 1e-12,
            "sentinel: one millivolt is a reading, not a marker (%.6f V)", mv);

    // A fault count of -1 is the blob's placeholder; zero faults is a
    // real and common answer and must survive.
    memcpy(t, p, n);
    const size_t fc = offsetof(COMMS_beacon_extended_packet_t,
                               eps_total_fault_count);
    t[fc] = 0xFF; t[fc + 1] = 0xFF; t[fc + 2] = 0xFF; t[fc + 3] = 0xFF;
    must_read("eps_faults", t, n, &ok);
    tap_ok(!ok, "sentinel: a fault count of -1 is refused");
    memset(t + fc, 0, 4);
    const double zf = must_read("eps_faults", t, n, &ok);
    tap_okf(ok && zf == 0.0, "sentinel: a fault count of zero is a reading");

    // EPS mode 255 is the placeholder; mode 0 (startup) is real.
    memcpy(t, p, n);
    const size_t em = offsetof(COMMS_beacon_extended_packet_t, eps_mode_enum);
    t[em] = 255;
    must_read("eps_mode", t, n, &ok);
    tap_ok(!ok, "sentinel: EPS mode 255 is refused");
    t[em] = 0;
    const double m0 = must_read("eps_mode", t, n, &ok);
    tap_okf(ok && m0 == 0.0, "sentinel: EPS mode 0 is a reading");
}

static void test_preconditions(void)
{
    uint8_t p[256];
    const size_t n = ext_fixture(p, sizeof p);
    uint8_t t[256];
    const size_t st = offsetof(COMMS_beacon_extended_packet_t,
                               adcs_current_state_1);

    // A silent ADCS leaves its whole block zeroed, and every field that
    // comes out of that block has to be refused -- otherwise the plots
    // show a satellite in a perfect zero attitude, in a zero magnetic
    // field, with every sun sensor dark.
    memcpy(t, p, n);
    memset(t + st, 0, 6);
    const char *adcs_fields[] = { "mag_x", "mag_norm", "css1", "rate_norm",
                                  "rate_x", "roll", "pitch", "yaw",
                                  "est_mode", "ctrl_mode", "sun_up",
                                  "adcs_faults" };
    int leaked = 0;
    for (size_t i = 0; i < sizeof adcs_fields / sizeof adcs_fields[0]; i++) {
        int ok = 0;
        must_read(adcs_fields[i], t, n, &ok);
        if (ok) { printf("# %s read from a silent ADCS\n", adcs_fields[i]); leaked++; }
    }
    tap_okf(leaked == 0,
            "precondition: a silent ADCS refuses all %zu of its fields "
            "(%d leaked)", sizeof adcs_fields / sizeof adcs_fields[0], leaked);
    // The EPS fields in the same packet are untouched by that.
    int ok = 0;
    const double bv = must_read("batt_v", t, n, &ok);
    tap_okf(ok && fabs(bv - 15.857) < 1e-9,
            "precondition: and the EPS fields in that packet still read");

    // The attitude angles only exist in estimation modes 3 to 6; the
    // rates and the field vector exist in all of them.
    for (unsigned mode = 0; mode <= 7; mode++) {
        memcpy(t, p, n);
        t[st] = (uint8_t) ((t[st] & 0xF0) | mode);
        int ok_roll = 0, ok_mag = 0;
        must_read("roll", t, n, &ok_roll);
        must_read("mag_x", t, n, &ok_mag);
        const int want = (mode >= 3 && mode <= 6);
        tap_okf(ok_roll == want && ok_mag == 1,
                "precondition: estimation mode %u %s an attitude, and always "
                "a field vector", mode, want ? "has" : "has no");
    }
}

static void test_derived_modes_and_faults(void)
{
    uint8_t p[256];
    const size_t n = ext_fixture(p, sizeof p);
    uint8_t t[256];
    const size_t st = offsetof(COMMS_beacon_extended_packet_t,
                               adcs_current_state_1);
    int ok = 0;

    // Walk the two nibbles of the first state byte independently, so a
    // swap between estimation and control mode cannot pass.
    memcpy(t, p, n);
    t[st] = 0x72;   // control 7, estimation 2
    tap_okf(must_read("est_mode", t, n, &ok) == 2.0 && ok,
            "derived: estimation mode is the low nibble");
    tap_okf(must_read("ctrl_mode", t, n, &ok) == 7.0 && ok,
            "derived: control mode is the high nibble");

    // The run mode is the low two bits of the second byte, and the
    // Sun flag the top bit of the third.
    memcpy(t, p, n);
    t[st + 1] = (uint8_t) ((t[st + 1] & 0xFC) | 0x02);
    tap_okf(must_read("run_mode", t, n, &ok) == 2.0 && ok,
            "derived: run mode is the low two bits of the second byte");
    memcpy(t, p, n);
    t[st + 2] &= 0x7F;
    tap_okf(must_read("sun_up", t, n, &ok) == 0.0 && ok,
            "derived: the Sun flag follows its own bit");

    // The fault count counts flags, one at a time and then together.
    memcpy(t, p, n);
    t[st + 4] |= 0x01;                         // magnetometer_range_error
    tap_okf(must_read("adcs_faults", t, n, &ok) == 1.0 && ok,
            "derived: one fault flag counts as one");
    t[st + 5] |= 0x20;                         // coarse_sun_sensor_error
    tap_okf(must_read("adcs_faults", t, n, &ok) == 2.0 && ok,
            "derived: two flags in different bytes count as two");
    memcpy(t, p, n);
    t[st + 3] = 0xFF; t[st + 4] = 0xFF; t[st + 5] = 0xFF;
    const double all_faults = must_read("adcs_faults", t, n, &ok);
    tap_okf(ok && all_faults == 24.0,
            "derived: every fault flag set counts all 24 of them (got %.0f)",
            all_faults);
}

static void test_lengths_that_are_not_beacons(void)
{
    // Nothing reads a field out of a payload whose length is not one a
    // beacon has. That is what makes a bounds check inside the reader
    // unnecessary -- see the note in beacon_series_value -- so it is
    // worth pinning: the last field, the yaw angle, ends on the
    // packet's final byte, and every length short of that is refused
    // before an offset is used.
    uint8_t p[256];
    const size_t n = ext_fixture(p, sizeof p);

    const size_t bad[] = { 0, 1, 129, 131, 133, 135, 197, 199, 201, 203 };
    int leaked = 0;
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        int ok = 0;
        must_read("yaw",    p, bad[i], &ok); if (ok) leaked++;
        must_read("batt_v", p, bad[i], &ok); if (ok) leaked++;
    }
    tap_okf(leaked == 0,
            "lengths: ten payload lengths no beacon has are all refused "
            "(%d read anyway)", leaked);

    int ok = 0;
    must_read("yaw", p, n, &ok);
    tap_ok(ok, "lengths: the real one is not");
}

int main(void)
{
    test_catalogue_shape();
    test_values_from_the_real_beacon();
    test_basic_beacon();
    test_sentinels();
    test_preconditions();
    test_derived_modes_and_faults();
    test_lengths_that_are_not_beacons();
    return tap_done();
}
