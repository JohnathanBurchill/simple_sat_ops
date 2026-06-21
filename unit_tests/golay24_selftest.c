/*

    Simple Satellite Operations  unit_tests/golay24_selftest.c

    Direct self-test for the extended Golay(24,12) coder in
    src/proto/golay24.c. ax100_selftest exercises Golay only on the
    correctable side and explicitly disclaims the 4-bit-error boundary,
    so this test pins the code's structure and its full correction /
    detection behaviour:

      - the systematic layout (low 12 bits carry the data verbatim),
      - the textbook weight enumerator of the extended binary Golay
        code (A0=1, A8=759, A12=2576, A16=759, A24=1) and d_min=8 — an
        EXTERNAL oracle that pins the exact generator matrix, not just
        a self-consistent round-trip,
      - correction of every 1-bit error and sampled 2- and 3-bit errors
        to the right data with the right reported error count,
      - the 4-bit boundary: with d_min=8 a 4-bit error sits distance 4
        from the sent word and >=4 from every other codeword, so the
        decoder must report it uncorrectable (returns -1), never
        silently mis-correct.

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
#include "golay24.h"

#include <stdint.h>
#include <stddef.h>

// Deterministic LCG so a failure is reproducible from the source alone
// (never seed from time()).
static uint32_t g_rng = 0x0ce1a24du;
static uint32_t rnd(void)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}

static int weight24(uint32_t x)
{
    return __builtin_popcount(x & 0x00FFFFFFu);
}

// A distinct random bit position in [0,24) not already in used[0..n).
static int fresh_bit(const int *used, int n)
{
    for (;;) {
        int b = (int)(rnd() % 24u);
        int dup = 0;
        for (int i = 0; i < n; ++i) {
            if (used[i] == b) { dup = 1; break; }
        }
        if (!dup) return b;
    }
}

static void test_systematic_and_roundtrip(void)
{
    int sys_ok = 1, rt_ok = 1, clean_ok = 1;
    for (uint32_t d = 0; d < 4096; ++d) {
        uint32_t cw = golay24_encode((uint16_t)d);
        if ((cw & 0x0FFFu) != d) sys_ok = 0;     // data sits in the low 12 bits
        if (cw > 0x00FFFFFFu)    sys_ok = 0;      // and the word fits in 24 bits
        uint16_t out = 0xFFFF;
        int ec = -9;
        int rc = golay24_decode(cw, &out, &ec);
        if (rc != 0 || out != (uint16_t)d) rt_ok = 0;
        if (ec != 0) clean_ok = 0;                // a clean codeword has 0 errors
    }
    tap_ok(sys_ok, "encode is systematic: data is the low 12 bits, word fits in 24 bits");
    tap_ok(rt_ok, "all 4096 codewords decode back to their data");
    tap_ok(clean_ok, "clean codewords report 0 corrected errors");
}

static void test_weight_enumerator(void)
{
    // For a linear code the weight enumerator is the histogram of
    // codeword weights; these counts are the published constants for the
    // extended binary Golay(24,12) code. A wrong generator row shifts
    // them, so this catches a transcription error the round-trip can't.
    int counts[25] = {0};
    for (uint32_t d = 0; d < 4096; ++d) {
        counts[weight24(golay24_encode((uint16_t)d))]++;
    }
    tap_okf(counts[0] == 1,     "weight-0 codewords: %d (want 1)", counts[0]);
    tap_okf(counts[8] == 759,   "weight-8 codewords: %d (want 759)", counts[8]);
    tap_okf(counts[12] == 2576, "weight-12 codewords: %d (want 2576)", counts[12]);
    tap_okf(counts[16] == 759,  "weight-16 codewords: %d (want 759)", counts[16]);
    tap_okf(counts[24] == 1,    "weight-24 codewords: %d (want 1)", counts[24]);

    int total = 0;
    for (int i = 0; i < 25; ++i) total += counts[i];
    tap_okf(total == 4096, "weight histogram sums to 4096 (got %d)", total);

    int dmin = 99;
    for (uint32_t d = 1; d < 4096; ++d) {
        int w = weight24(golay24_encode((uint16_t)d));
        if (w < dmin) dmin = w;
    }
    tap_okf(dmin == 8, "minimum distance d_min = %d (want 8, so it corrects up to 3 errors)", dmin);
}

static void test_single_bit_correction(void)
{
    // Every single-bit error in every position, for a sample of data.
    int ok = 1;
    for (int t = 0; t < 64; ++t) {
        uint16_t d = (uint16_t)(rnd() & 0x0FFFu);
        uint32_t cw = golay24_encode(d);
        for (int b = 0; b < 24; ++b) {
            uint16_t out = 0;
            int ec = -9;
            int rc = golay24_decode(cw ^ (1u << b), &out, &ec);
            if (rc != 0 || out != d || ec != 1) ok = 0;
        }
    }
    tap_ok(ok, "every 1-bit error is corrected to the right data (errors=1)");
}

static void test_two_and_three_bit_correction(void)
{
    int ok2 = 1, ok3 = 1;
    for (int t = 0; t < 1000; ++t) {
        uint16_t d = (uint16_t)(rnd() & 0x0FFFu);
        uint32_t cw = golay24_encode(d);

        int b[3];
        b[0] = fresh_bit(b, 0);
        b[1] = fresh_bit(b, 1);
        uint32_t r2 = cw ^ (1u << b[0]) ^ (1u << b[1]);
        uint16_t o2 = 0;
        int e2 = -9;
        if (golay24_decode(r2, &o2, &e2) != 0 || o2 != d || e2 != 2) ok2 = 0;

        b[2] = fresh_bit(b, 2);
        uint32_t r3 = r2 ^ (1u << b[2]);
        uint16_t o3 = 0;
        int e3 = -9;
        if (golay24_decode(r3, &o3, &e3) != 0 || o3 != d || e3 != 3) ok3 = 0;
    }
    tap_ok(ok2, "sampled 2-bit errors corrected to the right data (errors=2)");
    tap_ok(ok3, "sampled 3-bit errors corrected to the right data (errors=3)");
}

static void test_four_bit_detected_uncorrectable(void)
{
    // d_min=8 => a 4-bit error is distance 4 from the sent word and >=4
    // from any other codeword, so the nearest is never within the
    // radius-3 ball: the decoder must reject it rather than mis-correct.
    int ok = 1;
    for (int t = 0; t < 1000; ++t) {
        uint16_t d = (uint16_t)(rnd() & 0x0FFFu);
        uint32_t cw = golay24_encode(d);
        int b[4];
        b[0] = fresh_bit(b, 0);
        b[1] = fresh_bit(b, 1);
        b[2] = fresh_bit(b, 2);
        b[3] = fresh_bit(b, 3);
        uint32_t r = cw ^ (1u << b[0]) ^ (1u << b[1]) ^ (1u << b[2]) ^ (1u << b[3]);
        uint16_t out = 0;
        int ec = -9;
        if (golay24_decode(r, &out, &ec) != -1) ok = 0;
    }
    tap_ok(ok, "sampled 4-bit errors are flagged uncorrectable (decode returns -1)");
}

static void test_null_safety(void)
{
    uint16_t out = 0;
    int rc = golay24_decode(golay24_encode(0x5A5u), &out, NULL);
    tap_ok(rc == 0 && out == 0x5A5u, "decode accepts a NULL errors-out pointer");

    // NULL data-out must be rejected, not dereferenced.
    int ec = -9;
    rc = golay24_decode(golay24_encode(0x123u), NULL, &ec);
    tap_ok(rc == -1, "decode rejects a NULL data-out pointer (returns -1)");
}

int main(void)
{
    test_systematic_and_roundtrip();
    test_weight_enumerator();
    test_single_bit_correction();
    test_two_and_three_bit_correction();
    test_four_bit_detected_uncorrectable();
    test_null_safety();
    return tap_done();
}
