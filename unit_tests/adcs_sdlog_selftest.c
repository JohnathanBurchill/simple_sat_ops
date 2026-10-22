/*

    Simple Satellite Operations  unit_tests/adcs_sdlog_selftest.c

    Exercises src/proto/adcs_sdlog.c: reading an ADCS SD-card log's header,
    finding a frame inside a record, and telling a chunk of one log from a
    chunk of any other file by the log's record schedule.

    The chunks below are verbatim bulk_file payloads from packet_db.sqlite,
    every one with a CSP CRC32 that checked out: the first chunk of
    ADCS/log_9a51.TLM (a 10 s log, 2026-10-05 08:01-10:30 UTC, which covers
    the 10:23 MPI run), its chunk at offset 76635, the first chunk of
    ADCS/log_9c08.TLM (a 60 s log, 2026-10-05 21:40 to 10-06 17:45), its
    chunk at offset 1950, and two other chunks that also sit at offset 76635:
    one of the 2026-09-30 log, which has log_9a51's mask, and one of an MPI
    file. The expected times and angles are not worked out from these bytes
    here: they are the values an earlier, separate decoder wrote for those two
    logs into ~/FrontierSat/Commissioning/ADCS/20261009/logs/log_9a51.tsv and
    log_9c08.tsv, which the ADCS commissioning work checked against the
    magnetic field.

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
#include "adcs_sdlog.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// log_9a51: mask 80 7B 80 10 E8 00 00 08 00 00, 14 frames, 90-byte records.
static const char *HDR_9A51 =
    "807b8010e800000800004e59c36ad80315001800d1ff5efe92fe21fac2093507"
    "9d080000000000000000000000000500c6fff7ff5ffebafe21fa90012c015802"
    "f5f299bf91c0cc0349fc1af20400f9ff010084ffd900d7ff93db230792062300"
    "91fdc7ff5859c36ad80310001400d3ff5efed7fd21fa5a0a4607aa0700000000"
    "00000000000000000700c7fff7ff5ffef7fd21fa900190015802f1f15cc1b0bf"
    "d4034cfc39f20d00e9ff050065ff0b01d4ffffdbef06de072700aafdc8ff6259"
    "c36ad8";

// log_9a51 at offset 76635: records 852 and 853 start inside it.
static const char *CHUNK_9A51_76635 =
    "000200eeff03000dfe18feea0390019001f401dfff1400f5ff2bfc6a00251106"
    "0091ff1e0090ff0c01e2ffe3ff1100f7ff0c00f6ff0500977ac36a1300fcff01"
    "00ffff0bfe12fef503a1fb1300d9100000000000000000000000000100edff03"
    "000cfe1ffef30390019001f401deff1100f3ff0afc6a000f11060091ff1e0081"
    "ffe400deffe2ff0f00f4fffffff1ff0200a17ac36a1300fbff0100ffff02fe09"
    "fee80382fb0900b910000000000000000000000000fffff0ff030006fef4fdfc"
    "039001";

// log_9c08: mask 00 62 00 10 0A 00 08 08 00 00, 8 frames, 54-byte records.
static const char *HDR_9C08 =
    "006200100a00080800006919c46a6703f9fed107b8f9f0fff2fff3ff36fd3b08"
    "64076e0ba10cc82422035306cac4fdf53b010dff2b289107f3dca3ffdbffacff"
    "a519c46a67032afecc0621f9f0ffeffff5ff36fd7e066407e00af10ae921a901"
    "0806f7c480f6460173fd2b23a10b07daabffc6ffbfffe119c46a670388fdc505"
    "b0f8f2ffeefff3ff36fd950464077e0a7309291f3000be0532c52cf7500106fc"
    "181ecc0ed7d7c6ffc6ffacff1d1ac46a670327fde30446f8f4ffeefff2ff36fd"
    "ab0265";

// log_9c08 at offset 1950: records 36 to 39 start inside it.
static const char *CHUNK_9C08_1950 =
    "15001300d921c46a72032403ac04c2ef0c00faff0400ab00f903c703dfffd4ff"
    "eaff22ecabbf1fc94604180403f057193bf342ab0c00eaff26001522c46a7203"
    "b603a1045df00c00feff03007f0021043a03e4ffddffd5ff97ed20bf62c8f404"
    "09049ff018195df057ae08000e0019005122c46a72033704a704eff00b00fbff"
    "070079002e049002efffd0ffc0ff0defa4bea2c79a05f9034ff12e19d5ed35b1"
    "fcfff0ff38008d22c46a7203c404b00497f10900fbff08008f003904fa01fcff"
    "d2ff9f";

// The 2026-09-30 07:05 log at the same offset. It has log_9a51's mask, so
// its records start at the same bytes and carry well-formed stamps -- just
// five days earlier than log_9a51's schedule puts them.
static const char *CHUNK_0930_76635 =
    "00b5ff0a00d7ff5c00a2fecffb90019001f401000036390000cbf860fe51f2ff"
    "ff2f00d900c3fff2ff540000006725000031fce6fffcfc19d6bc6aea020000fb"
    "ffefff5d00a1fecffb860afcf5b404000000000000000000000000b2ff1300db"
    "ff5c00a1fed0fb90019001f401000056370000e2f85bfe26f2ffff2f00d900d5"
    "ffd9ff09000000e5240000f8fb3f0014fd23d6bc6aea020000f4ffedff5d00a1"
    "fecffbee0a10f64d04000000000000000000000000b2ff1c00dcff5c00a2fecf"
    "fb9001";

// An MPI science file at the same offset as CHUNK_9A51_76635 (its frame
// sync word 0C FF FF 0C is in the second line).
static const char *CHUNK_MPI_76635 =
    "47082008470791082207ae080807b807d807f307ff077007b407ac07a8078d07"
    "de0791081f07b3080d07b50828075c07bf07c107f107f816150cffff0c01f505"
    "501103ff1fff0d0f523070010407be07a0077907df07e6078207d4082107a408"
    "2e078c07bd07cb0775079807d9080107f20819081e07f907f007af07c507df07"
    "a207dd07dc07a607c0081f07b507c6079a07bc07ca078a07fd07c9080f07f508"
    "0207a2080c081207d507c007f4077507e407cd07b807d607c0077b087c079207"
    "e107db";

#define CHUNK_LEN 195

static void unhex(const char *s, uint8_t *out)
{
    for (int i = 0; s[2 * i] != '\0'; i++) {
        unsigned v = 0;
        for (int k = 0; k < 2; k++) {
            char c = s[2 * i + k];
            v = v * 16 + (unsigned) (c <= '9' ? c - '0' : c - 'a' + 10);
        }
        out[i] = (uint8_t) v;
    }
}

static int le16s(const uint8_t *p)
{
    return (int16_t) ((uint16_t) p[0] | ((uint16_t) p[1] << 8));
}

// The estimated angles of record i, read from a chunk at file offset off.
static void angles(const adcs_sdlog_t *log, const uint8_t *chunk, long off,
                   long i, int *r, int *p, int *y)
{
    long a = adcs_sdlog_record_offset(log, i)
           + adcs_sdlog_frame_offset(log, ADCS_SDLOG_FRAME_EST_ANGLES) - off;
    *r = le16s(chunk + a);
    *p = le16s(chunk + a + 2);
    *y = le16s(chunk + a + 4);
}

static uint8_t hdr_a[CHUNK_LEN], chunk_a[CHUNK_LEN];
static uint8_t hdr_b[CHUNK_LEN], chunk_b[CHUNK_LEN], chunk_mpi[CHUNK_LEN];
static uint8_t chunk_0930[CHUNK_LEN];
static adcs_sdlog_t log_a, log_b;

static void test_header(void)
{
    tap_ok(adcs_sdlog_parse_header(hdr_a, CHUNK_LEN, &log_a) == 1,
           "the first chunk of log_9a51 reads as a log");
    tap_okf(log_a.nframes == 14, "it selects 14 frames (got %d)", log_a.nframes);
    tap_okf(log_a.rec_len == 90, "its records are 90 bytes (got %ld)", log_a.rec_len);
    tap_okf(fabs(log_a.t0 - 1791187278.984) < 1e-6,
            "its first record is at 1791187278.984 (got %.3f)", log_a.t0);

    tap_ok(adcs_sdlog_parse_header(hdr_b, CHUNK_LEN, &log_b) == 1,
           "the first chunk of log_9c08 reads as a log");
    tap_okf(log_b.rec_len == 54, "its records are 54 bytes (got %ld)", log_b.rec_len);
    tap_okf(fabs(log_b.t0 - 1791236457.871) < 1e-6,
            "its first record is at 1791236457.871 (got %.3f)", log_b.t0);
}

static void test_header_rejects(void)
{
    adcs_sdlog_t h;
    uint8_t b[CHUNK_LEN];

    tap_ok(adcs_sdlog_parse_header(chunk_mpi, CHUNK_LEN, &h) == 0,
           "a chunk of an MPI file does not read as a log");

    memcpy(b, hdr_a, CHUNK_LEN);
    memset(b, 0, ADCS_SDLOG_MASK_LEN);
    tap_ok(adcs_sdlog_parse_header(b, CHUNK_LEN, &h) == 0,
           "a mask selecting nothing is not a log");

    // Milliseconds 1000 (E8 03) is a value no real stamp holds.
    memcpy(b, hdr_a, CHUNK_LEN);
    b[14] = 0xE8; b[15] = 0x03;
    tap_ok(adcs_sdlog_parse_header(b, CHUNK_LEN, &h) == 0,
           "a first stamp reading 1000 ms is rejected");
    b[14] = 0xE7;
    tap_ok(adcs_sdlog_parse_header(b, CHUNK_LEN, &h) == 1,
           "one reading 999 ms is accepted");

    // 2025-01-01, when the ADCS clock was still a year behind.
    memcpy(b, hdr_a, CHUNK_LEN);
    b[10] = 0x80; b[11] = 0x85; b[12] = 0x74; b[13] = 0x67;
    tap_ok(adcs_sdlog_parse_header(b, CHUNK_LEN, &h) == 0,
           "a first record dated before 2026 is rejected");

    tap_ok(adcs_sdlog_parse_header(hdr_a, 15, &h) == 0,
           "15 bytes are too few to hold the mask and a stamp");
    tap_ok(adcs_sdlog_parse_header(hdr_a, 16, &h) == 1,
           "16 bytes are enough");
}

static void test_frame_offset(void)
{
    // Bits in mask order, lowest first: 7 | 8 9 11 12 13 14 | 23 | 28 | ...
    tap_okf(adcs_sdlog_frame_offset(&log_a, 7) == 6,
            "the lowest selected frame comes straight after the stamp (got %d)",
            adcs_sdlog_frame_offset(&log_a, 7));
    tap_okf(adcs_sdlog_frame_offset(&log_a, ADCS_SDLOG_FRAME_EST_ANGLES) == 54,
            "the angles are the ninth frame, at byte 54 (got %d)",
            adcs_sdlog_frame_offset(&log_a, ADCS_SDLOG_FRAME_EST_ANGLES));
    tap_okf(adcs_sdlog_frame_offset(&log_a, 59) == 84,
            "the highest selected frame ends the record (got %d)",
            adcs_sdlog_frame_offset(&log_a, 59));
    tap_ok(adcs_sdlog_frame_offset(&log_a, ADCS_SDLOG_FRAME_STATE) == -1,
           "a frame the mask does not select has no offset");
    tap_ok(adcs_sdlog_frame_offset(&log_a, 80) == -1,
           "a frame past the mask has no offset");
    tap_okf(adcs_sdlog_frame_offset(&log_b, ADCS_SDLOG_FRAME_EST_ANGLES) == 24,
            "log_9c08's angles are at byte 24 (got %d)",
            adcs_sdlog_frame_offset(&log_b, ADCS_SDLOG_FRAME_EST_ANGLES));
}

static void test_angles(void)
{
    int r, p, y;
    angles(&log_a, hdr_a, 0, 0, &r, &p, &y);
    tap_okf(r == -3339 && p == -16487 && y == -16239,
            "log_9a51 record 0 reads -3339 -16487 -16239 (got %d %d %d)", r, p, y);
    // 10:23:19 UTC, during the MPI run.
    angles(&log_a, chunk_a, 76635, 852, &r, &p, &y);
    tap_okf(r == -34 && p == 17 && y == -13,
            "log_9a51 record 852 reads -34 17 -13 (got %d %d %d)", r, p, y);

    static const int want[3][3] = { { -33, -44, -22 }, { -28, -35, -43 }, { -17, -48, -64 } };
    for (int k = 0; k < 3; k++) {
        angles(&log_b, chunk_b, 1950, 36 + k, &r, &p, &y);
        tap_okf(r == want[k][0] && p == want[k][1] && y == want[k][2],
                "log_9c08 record %d reads %d %d %d (got %d %d %d)", 36 + k,
                want[k][0], want[k][1], want[k][2], r, p, y);
    }
}

static void test_stamps(void)
{
    long a = -1, b = -1;
    tap_okf(adcs_sdlog_stamps_in(&log_a, 76635, CHUNK_LEN, &a, &b) == 2 && a == 852 && b == 853,
            "the 76635 chunk holds the stamps of records 852 and 853 (got %ld..%ld)", a, b);
    // Record 852's stamp is bytes 76690..76695.
    tap_ok(adcs_sdlog_stamps_in(&log_a, 76690, 6, &a, &b) == 1 && a == 852 && b == 852,
           "six bytes starting on a record hold its stamp");
    tap_ok(adcs_sdlog_stamps_in(&log_a, 76690, 5, &a, &b) == 0,
           "five do not");
    tap_ok(adcs_sdlog_stamps_in(&log_a, 76691, CHUNK_LEN, &a, &b) == 2 && a == 853,
           "a chunk starting one byte into a stamp does not count it");
    tap_ok(adcs_sdlog_stamps_in(&log_a, 0, 15, &a, &b) == 0,
           "the first 15 bytes do not hold record 0's whole stamp");
    tap_ok(adcs_sdlog_stamps_in(&log_a, 0, 16, &a, &b) == 1 && a == 0,
           "the first 16 do");

    double t = adcs_sdlog_stamp(chunk_a + (76690 - 76635));
    tap_okf(fabs(t - 1791195799.019) < 1e-6,
            "record 852's stamp reads 1791195799.019 (got %.3f)", t);
    static const uint8_t ff[ADCS_SDLOG_STAMP_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    tap_ok(adcs_sdlog_stamp(ff) < 0.0, "a stamp of FF bytes is not a time");

    tap_okf(adcs_sdlog_period_at(&log_a, 852, t) == 10,
            "record 852 at that time says the period is 10 s (got %ld)",
            adcs_sdlog_period_at(&log_a, 852, t));
    tap_ok(adcs_sdlog_period_at(&log_a, 851, t) == 0,
           "taken as record 851 it fits no whole-second period");
    tap_ok(adcs_sdlog_period_at(&log_a, 0, t) == 0, "record 0 says nothing about the period");
    tap_ok(adcs_sdlog_period_at(&log_a, 852, log_a.t0 - 10.0) == 0,
           "a time before the log began says nothing either");
}

static void test_chunk_fits(void)
{
    tap_okf(adcs_sdlog_chunk_fits(&log_a, 10, 76635, chunk_a, CHUNK_LEN) == 2,
            "log_9a51's own chunk fits its 10 s schedule (got %ld)",
            adcs_sdlog_chunk_fits(&log_a, 10, 76635, chunk_a, CHUNK_LEN));
    tap_ok(adcs_sdlog_chunk_fits(&log_a, 10, 0, hdr_a, CHUNK_LEN) == 2,
           "so does its first chunk, records 0 and 1");
    tap_ok(adcs_sdlog_chunk_fits(&log_a, 11, 76635, chunk_a, CHUNK_LEN) == -1,
           "the same chunk is off an 11 s schedule");
    tap_ok(adcs_sdlog_chunk_fits(&log_a, 10, 76635, chunk_mpi, CHUNK_LEN) == -1,
           "an MPI chunk at the same offset does not fit log_9a51");
    tap_ok(adcs_sdlog_chunk_fits(&log_a, 10, 76635, chunk_0930, CHUNK_LEN) == -1,
           "nor does a chunk of another log with the same mask, at the same offset");
    tap_ok(adcs_sdlog_chunk_fits(&log_a, 10, 1950, chunk_b, CHUNK_LEN) == -1,
           "log_9c08's chunk does not fit log_9a51");
    tap_okf(adcs_sdlog_chunk_fits(&log_b, 60, 1950, chunk_b, CHUNK_LEN) == 4,
            "log_9c08's chunk fits its own 60 s schedule (got %ld)",
            adcs_sdlog_chunk_fits(&log_b, 60, 1950, chunk_b, CHUNK_LEN));
    tap_ok(adcs_sdlog_chunk_fits(&log_b, 60, 76635, chunk_a, CHUNK_LEN) == -1,
           "log_9a51's chunk does not fit log_9c08");
    // Bytes 76696..76700 of log_9a51: past record 852's stamp, before 853's.
    tap_ok(adcs_sdlog_chunk_fits(&log_a, 10, 76696, chunk_a + 61, 5) == 0,
           "a piece holding no whole stamp cannot be judged");
}

int main(void)
{
    unhex(HDR_9A51, hdr_a);
    unhex(CHUNK_9A51_76635, chunk_a);
    unhex(HDR_9C08, hdr_b);
    unhex(CHUNK_9C08_1950, chunk_b);
    unhex(CHUNK_MPI_76635, chunk_mpi);
    unhex(CHUNK_0930_76635, chunk_0930);

    test_header();
    test_header_rejects();
    test_frame_offset();
    test_angles();
    test_stamps();
    test_chunk_fits();
    return tap_done();
}
