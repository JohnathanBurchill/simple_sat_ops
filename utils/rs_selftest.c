/*

    Simple Satellite Operations  utils/rs_selftest.c

    Round-trip and error-injection test for the rs.c module. Validates
    that the C port matches the reed_solomon_ccsds Python reference in
    (a) raw parity bytes on a known input, and (b) byte-error correction
    up to the claimed t=16 limit.

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

#include "rs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;

static void check(int cond, const char *what)
{
    fprintf(stderr, "  %s: %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) ++failures;
}

// Hex-dump helper for failed comparisons.
static void dump(const char *label, const uint8_t *buf, size_t len)
{
    fprintf(stderr, "    %s (%zu): ", label, len);
    for (size_t i = 0; i < len; ++i) fprintf(stderr, "%02x", buf[i]);
    fputc('\n', stderr);
}

// Deterministic pseudo-random fill (xorshift32) so tests are reproducible.
static uint32_t xs_state = 0xdeadbeefu;
static uint32_t xs_next(void)
{
    uint32_t x = xs_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xs_state = x;
    return x;
}
static void fill_pseudo(uint8_t *buf, size_t len, uint32_t seed)
{
    xs_state = seed;
    for (size_t i = 0; i < len; ++i) buf[i] = (uint8_t)xs_next();
}

static int bytes_equal(const uint8_t *a, const uint8_t *b, size_t len)
{
    return memcmp(a, b, len) == 0;
}

// Test 1: zero-data encode. First 223 bytes all zeros; parity must also
// be all zeros (since 0·x^32 mod g(x) = 0).
static void test_zero_encode(void)
{
    uint8_t cw[RS_N];
    uint8_t zeros[RS_K] = {0};
    rs_encode(zeros, cw);
    int ok = 1;
    for (int i = 0; i < RS_N; ++i) if (cw[i] != 0) { ok = 0; break; }
    check(ok, "zero-data encode yields all-zero codeword");
}

// Test 2: known-vector cross-check against Python reed_solomon_ccsds.
// Input: 223 bytes = 0x00, 0x01, 0x02, ... 0xde (i.e., i & 0xff, wrapping
// once at 256 → 0 but we only go to 222). Python reference parity is
// captured at test time by running: see generate_vector.py or
// equivalently `python3 -c "import reed_solomon_ccsds as rs; import numpy
// as np; d=bytes(range(223)); print(rs.encode(d, False, 1)[223:].hex())"`.
static void test_known_vector(void)
{
    uint8_t data[RS_K];
    for (int i = 0; i < RS_K; ++i) data[i] = (uint8_t)i;

    uint8_t cw[RS_N];
    rs_encode(data, cw);

    // Expected parity — filled in by reference script; leave zero to
    // skip this test if vector hasn't been populated yet.
    // Captured from reed_solomon_ccsds v1.0.3 Python:
    //   data = bytes(range(223))
    //   parity = rs.encode(data, False, 1)[223:]
    static const uint8_t expected_parity[RS_NROOTS] = {
        0x2f, 0xbd, 0x4f, 0xb4, 0x74, 0x84, 0x94, 0xb9,
        0xac, 0xd5, 0x54, 0x62, 0x72, 0x12, 0xee, 0xb3,
        0xeb, 0xed, 0x41, 0x19, 0x1d, 0xe1, 0xd3, 0x63,
        0x20, 0xea, 0x49, 0x29, 0x0b, 0x25, 0xab, 0xcf,
    };

    int ok = bytes_equal(cw + RS_K, expected_parity, RS_NROOTS);
    check(ok, "known-vector parity matches Python reference");
    if (!ok) {
        dump("expected", expected_parity, RS_NROOTS);
        dump("actual  ", cw + RS_K, RS_NROOTS);
    }
}

// Test 3: clean decode returns 0 errors and leaves data intact.
static void test_clean_decode(void)
{
    uint8_t data[RS_K];
    fill_pseudo(data, RS_K, 0x12345678u);

    uint8_t cw[RS_N];
    rs_encode(data, cw);

    int errs = rs_decode(cw, NULL);
    check(errs == 0, "clean decode returns 0 errors");
    check(bytes_equal(cw, data, RS_K), "clean decode preserves data");
}

// Test 4: flip exactly t bytes for t = 1..16; decode should correct all
// and report the right count. Use non-overlapping positions.
static void test_correct_up_to_t(void)
{
    uint8_t data[RS_K];
    uint8_t cw[RS_N];
    uint8_t corrupt[RS_N];

    for (int t = 1; t <= 16; ++t) {
        fill_pseudo(data, RS_K, 0xcafe0000u + (uint32_t)t);
        rs_encode(data, cw);
        memcpy(corrupt, cw, RS_N);

        // Flip `t` bytes at spread-out deterministic positions. Use a step
        // of 15 so the 16 indices 3, 18, ..., 228 are all distinct and
        // still inside the codeword (last < 255).
        for (int i = 0; i < t; ++i) {
            int pos = i * 15 + 3;
            corrupt[pos] ^= 0xA5;
        }

        int locs[RS_NROOTS];
        int errs = rs_decode(corrupt, locs);
        char name[64];
        snprintf(name, sizeof name, "correct %d byte error(s) (count)", t);
        check(errs == t, name);
        snprintf(name, sizeof name, "correct %d byte error(s) (data)", t);
        check(bytes_equal(corrupt, data, RS_K), name);
        // Verify the reported locations match the positions we flipped
        // (3, 18, 33, ...). Order isn't guaranteed by Forney, so sort first.
        int sorted[RS_NROOTS];
        for (int i = 0; i < t; ++i) sorted[i] = locs[i];
        for (int i = 1; i < t; ++i) {
            int v = sorted[i], j = i - 1;
            while (j >= 0 && sorted[j] > v) { sorted[j+1] = sorted[j]; --j; }
            sorted[j+1] = v;
        }
        int locs_match = 1;
        for (int i = 0; i < t; ++i) {
            if (sorted[i] != i * 15 + 3) { locs_match = 0; break; }
        }
        snprintf(name, sizeof name, "correct %d byte error(s) (locs)", t);
        check(locs_match, name);
    }
}

// Test 5: flipping > t bytes should return -1 (uncorrectable) OR misdecode.
// We only assert it does NOT silently "succeed" with wrong data —
// i.e., if it returns a non-negative count, the data must equal original;
// otherwise it must return -1. 17 byte errors is beyond capacity.
static void test_beyond_capacity(void)
{
    uint8_t data[RS_K];
    uint8_t cw[RS_N];
    uint8_t corrupt[RS_N];

    fill_pseudo(data, RS_K, 0xbadd0000u);
    rs_encode(data, cw);
    memcpy(corrupt, cw, RS_N);

    for (int i = 0; i < 17; ++i) {
        int pos = (i * 13 + 7) % RS_N;
        corrupt[pos] ^= 0x5A;
    }
    int errs = rs_decode(corrupt, NULL);
    int ok = (errs < 0) || bytes_equal(corrupt, data, RS_K);
    check(ok, "17-byte corruption either fails cleanly or somehow recovers");
}

// Test 6: pycsp wrapper round-trip with short payload.
static void test_pycsp_wrapper(void)
{
    const char *msg = "Hello, FrontierSat!";
    size_t msg_len = strlen(msg);

    uint8_t encoded[RS_N];
    ssize_t enc_len = rs_pycsp_encode((const uint8_t *)msg, msg_len,
                                      encoded, sizeof encoded);
    check(enc_len == (ssize_t)(msg_len + RS_NROOTS),
          "pycsp encode length = in_len + 32");

    // Flip 10 bytes (well under capacity).
    for (int i = 0; i < 10; ++i) encoded[i * 3 + 1] ^= 0x77;

    uint8_t decoded[256];
    int errs = -1;
    int locs[RS_NROOTS];
    ssize_t dec_len = rs_pycsp_decode(encoded, (size_t)enc_len,
                                      decoded, sizeof decoded, &errs, locs);
    check(dec_len == (ssize_t)msg_len, "pycsp decode length = out_len");
    check(errs == 10, "pycsp decode reports 10 errors corrected");
    check(memcmp(decoded, msg, msg_len) == 0, "pycsp decode recovers message");
    // Locations must match the on-wire byte indices we flipped: i*3+1 for
    // i in 0..9. pycsp decode returns positions relative to the start of
    // `encoded` (length enc_len); indices should be 1, 4, 7, ..., 28.
    int sorted[RS_NROOTS];
    for (int i = 0; i < errs; ++i) sorted[i] = locs[i];
    for (int i = 1; i < errs; ++i) {
        int v = sorted[i], j = i - 1;
        while (j >= 0 && sorted[j] > v) { sorted[j+1] = sorted[j]; --j; }
        sorted[j+1] = v;
    }
    int locs_ok = 1;
    for (int i = 0; i < errs; ++i) {
        if (sorted[i] != i * 3 + 1) { locs_ok = 0; break; }
    }
    check(locs_ok, "pycsp decode reports correct on-wire byte offsets");
}

int main(void)
{
    fprintf(stderr, "rs_selftest: running...\n");
    test_zero_encode();
    test_known_vector();
    test_clean_decode();
    test_correct_up_to_t();
    test_beyond_capacity();
    test_pycsp_wrapper();

    if (failures == 0) {
        fprintf(stderr, "rs_selftest: OK\n");
        return 0;
    }
    fprintf(stderr, "rs_selftest: %d failure(s)\n", failures);
    return 1;
}
