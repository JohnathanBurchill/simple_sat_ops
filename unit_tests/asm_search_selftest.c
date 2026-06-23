/*

    Simple Satellite Operations  unit_tests/asm_search_selftest.c

    Exercises src/dsp/asm_search.c: asm_find_best(), the sliding-window
    Hamming-distance search every demodulator (modem, modem_fsk, modem_iq,
    modem_viterbi) uses to lock onto the AX100 attached sync marker in a
    noisy bit stream.

    Two oracles, deliberately independent of the source's sliding-window
    trick:

      - Hand-built golden cases that pin the wire bit order (first bit is
        the window MSB), the LSB-only masking, and the documented
        "lowest-Hamming-distance, earliest on ties" selection rule.

      - A brute-force reference (ref_find_best) that rebuilds the window
        from scratch at every offset and iterates offsets directly. The
        randomized differential run plants needles with random bit flips
        and checks the source agrees with the reference on both the
        returned offset and the matched distance.

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
#include "asm_search.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define NOMATCH ((size_t) -1)

// ---- helpers -----------------------------------------------------------

// Lay the 32 bits of v into bits[off..off+31] MSB first: bits[off] carries
// bit 31 of v, bits[off+31] carries bit 0. This is the order asm_find_best
// reconstructs its window in (window = (window<<1) | bit), so a window read
// back from this region equals v exactly. The golden cases stand or fall on
// this convention, which is what pins it.
static void plant_u32(uint8_t *bits, size_t off, uint32_t v)
{
    for (int b = 0; b < 32; ++b)
        bits[off + b] = (uint8_t) ((v >> (31 - b)) & 1u);
}

// Independent population count (Kernighan), so the oracle does not share the
// source's __builtin_popcount path.
static int ham32(uint32_t x)
{
    int c = 0;
    while (x) { x &= x - 1; ++c; }
    return c;
}

// Brute-force reference: for every candidate offset >= min_offset, rebuild
// the 32-bit window and measure its distance to needle; keep the lowest
// distance <= max_ham, earliest position winning ties (strict <, matching
// the contract). Structurally unlike the source (no sliding window, no
// early break), so the differential run catches offset/loop-bound/tie bugs.
static size_t ref_find_best(const uint8_t *bits, size_t n_bits, uint32_t needle,
                            int max_ham, size_t min_offset, int *out_ham)
{
    int    best     = 33;
    size_t best_off = NOMATCH;
    if (n_bits >= 32) {
        for (size_t off = min_offset; off + 32u <= n_bits; ++off) {
            uint32_t w = 0;
            for (int b = 0; b < 32; ++b)
                w = (w << 1) | (bits[off + b] & 1u);
            int h = ham32(w ^ needle);
            if (h <= max_ham && h < best) {
                best     = h;
                best_off = off;
            }
        }
    }
    if (out_ham) *out_ham = (best_off != NOMATCH) ? best : 33;
    return best_off;
}

// Deterministic xorshift32 so the randomized run is reproducible across
// platforms (rand() differs between libcs). Seed is a fixed nonzero constant.
static uint32_t xnext(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

// Convenience: a fresh zeroed bit buffer for the golden cases. Zero windows
// never match a nonzero needle at max_ham 0, so plants are the only matches.
#define NEEDLE 0x930B51DEu

// ---- the wire constant -------------------------------------------------

static void test_constant_pinned(void)
{
    // The DSP layer hard-codes the transformed ASM so it need not include
    // ax100.h. If the AX100 sync marker ever changes, this line is the
    // tripwire that says "the demods are now hunting for the wrong word".
    tap_ok(ASM_BIG_ENDIAN_U32 == 0x930B51DEu,
           "ASM wire constant is 0x930B51DE");
}

// ---- exact matches and bit order --------------------------------------

static void test_exact_match(void)
{
    uint8_t bits[128] = {0};
    int ham = -1;

    plant_u32(bits, 0, NEEDLE);
    size_t off = asm_find_best(bits, 128, NEEDLE, 0, 0, &ham);
    tap_okf(off == 0 && ham == 0,
            "exact needle at offset 0 found at 0, distance 0 (off=%ld ham=%d)",
            (long) off, ham);

    memset(bits, 0, sizeof bits);
    ham = -1;
    plant_u32(bits, 40, NEEDLE);
    off = asm_find_best(bits, 128, NEEDLE, 0, 0, &ham);
    tap_okf(off == 40 && ham == 0,
            "exact needle at offset 40 found at 40, distance 0 (off=%ld ham=%d)",
            (long) off, ham);
}

static void test_n_bits_equals_32(void)
{
    // The smallest buffer that can hold the needle: exactly one window.
    uint8_t bits[32];
    plant_u32(bits, 0, NEEDLE);
    int ham = -1;
    size_t off = asm_find_best(bits, 32, NEEDLE, 0, 0, &ham);
    tap_okf(off == 0 && ham == 0,
            "32-bit buffer with the needle matches at 0 (off=%ld ham=%d)",
            (long) off, ham);
}

static void test_lsb_masking(void)
{
    // Plant the needle, then jam 0xFE into the high bits of every byte while
    // leaving the LSB intact (0x00->0xFE keeps LSB 0, 0x01->0xFF keeps LSB 1).
    // The search must mask with & 1u; if it ever stopped, the window would be
    // polluted and the exact match would vanish at max_ham 0.
    uint8_t bits[64] = {0};
    plant_u32(bits, 0, NEEDLE);
    for (int i = 0; i < 32; ++i)
        bits[i] |= 0xFEu;
    int ham = -1;
    size_t off = asm_find_best(bits, 64, NEEDLE, 0, 0, &ham);
    tap_okf(off == 0 && ham == 0,
            "only the byte LSB is significant; high junk bits are ignored (off=%ld ham=%d)",
            (long) off, ham);
}

// ---- the selection rule: lowest distance, earliest on ties ------------

static void test_best_not_first(void)
{
    // A Hamming-2 hit sits earlier (offset 5) than a perfect hit (offset 50).
    // With max_ham 4 the first one is a valid candidate, but the contract is
    // to return the *closest* match -- this is the whole reason the function
    // exists rather than a first-match scan.
    uint8_t bits[256] = {0};
    plant_u32(bits, 5, NEEDLE);
    bits[5] ^= 1u;          // two flips -> distance 2
    bits[10] ^= 1u;
    plant_u32(bits, 50, NEEDLE);    // distance 0
    int ham = -1;
    size_t off = asm_find_best(bits, 256, NEEDLE, 4, 0, &ham);
    tap_okf(off == 50 && ham == 0,
            "closer later match beats a weaker earlier one (off=%ld ham=%d)",
            (long) off, ham);
}

static void test_tie_breaks_earliest(void)
{
    // Two equal-distance (Hamming-1) hits, no perfect hit anywhere. The
    // earlier one wins. A >= / <= slip in the update test would return 60.
    // The two 32-bit regions must not overlap, or the second plant corrupts
    // the tail of the first and there is only one real distance-1 match.
    uint8_t bits[128] = {0};
    plant_u32(bits, 10, NEEDLE);
    bits[10] ^= 1u;          // distance 1
    plant_u32(bits, 60, NEEDLE);
    bits[60] ^= 1u;          // distance 1
    int ham = -1;
    size_t off = asm_find_best(bits, 128, NEEDLE, 1, 0, &ham);
    tap_okf(off == 10 && ham == 1,
            "equal-distance ties resolve to the earliest offset (off=%ld ham=%d)",
            (long) off, ham);
}

static void test_max_ham_boundary(void)
{
    // A distance-3 hit. max_ham is inclusive: 3 accepts it, 2 rejects it.
    uint8_t bits[128] = {0};
    plant_u32(bits, 10, NEEDLE);
    bits[10] ^= 1u;
    bits[11] ^= 1u;
    bits[12] ^= 1u;          // distance 3

    int ham = -1;
    size_t off = asm_find_best(bits, 128, NEEDLE, 3, 0, &ham);
    tap_okf(off == 10 && ham == 3,
            "distance equal to max_ham is accepted (off=%ld ham=%d)",
            (long) off, ham);

    ham = 99;
    off = asm_find_best(bits, 128, NEEDLE, 2, 0, &ham);
    tap_okf(off == NOMATCH && ham == 33,
            "distance above max_ham is rejected, distance reset to 33 (off=%ld ham=%d)",
            (long) off, ham);
}

// ---- min_offset --------------------------------------------------------

static void test_min_offset(void)
{
    // Exact match at offset 30, nothing else matches at max_ham 0.
    uint8_t bits[128] = {0};
    plant_u32(bits, 30, NEEDLE);
    int ham;

    ham = -1;
    size_t off = asm_find_best(bits, 128, NEEDLE, 0, 0, &ham);
    tap_okf(off == 30 && ham == 0, "min_offset 0 finds the match at 30 (off=%ld)", (long) off);

    ham = -1;
    off = asm_find_best(bits, 128, NEEDLE, 0, 30, &ham);
    tap_okf(off == 30 && ham == 0, "min_offset equal to the match offset still finds it (off=%ld)", (long) off);

    ham = 99;
    off = asm_find_best(bits, 128, NEEDLE, 0, 31, &ham);
    tap_okf(off == NOMATCH && ham == 33,
            "min_offset one past the match skips it (off=%ld ham=%d)", (long) off, ham);
}

static void test_last_window(void)
{
    // n_bits 64, match in the final possible window (offset 32 == n_bits-32).
    // start == n_bits-32 takes the path where the slide loop never runs and
    // only the initial window is tested.
    uint8_t bits[64] = {0};
    plant_u32(bits, 32, NEEDLE);
    int ham;

    ham = -1;
    size_t off = asm_find_best(bits, 64, NEEDLE, 0, 32, &ham);
    tap_okf(off == 32 && ham == 0, "match in the last window is found (off=%ld ham=%d)", (long) off, ham);

    ham = 99;
    off = asm_find_best(bits, 64, NEEDLE, 0, 33, &ham);
    tap_okf(off == NOMATCH && ham == 33,
            "min_offset past the last window returns no match (off=%ld ham=%d)", (long) off, ham);
}

// ---- degenerate inputs -------------------------------------------------

static void test_too_few_bits(void)
{
    // 31 bits cannot hold a 32-bit needle, even if they are the needle's bits.
    uint8_t bits[31];
    for (int i = 0; i < 31; ++i)
        bits[i] = (uint8_t) ((NEEDLE >> (31 - i)) & 1u);
    int ham = 99;
    size_t off = asm_find_best(bits, 31, NEEDLE, 0, 0, &ham);
    tap_okf(off == NOMATCH && ham == 33,
            "fewer than 32 bits cannot match (off=%ld ham=%d)", (long) off, ham);
}

static void test_no_match_resets_distance(void)
{
    // All-zero window, nonzero needle, generous max_ham: still no match, and
    // the caller's distance sentinel must come back as 33, not left dirty.
    uint8_t bits[128] = {0};
    int ham = 99;
    size_t off = asm_find_best(bits, 128, NEEDLE, 4, 0, &ham);
    tap_okf(off == NOMATCH && ham == 33,
            "no match within max_ham returns -1 and distance 33 (off=%ld ham=%d)",
            (long) off, ham);
}

static void test_out_ham_null(void)
{
    // out_ham == NULL must be tolerated (callers that don't care pass NULL).
    uint8_t bits[64] = {0};
    plant_u32(bits, 8, NEEDLE);
    size_t off = asm_find_best(bits, 64, NEEDLE, 0, 0, NULL);
    tap_okf(off == 8, "NULL out_ham is accepted and the offset is still returned (off=%ld)", (long) off);
}

// ---- randomized differential against the brute-force oracle ------------

static void test_random_differential(void)
{
    uint32_t s = 0x1234567u;
    int  mismatches = 0;
    int  n_found = 0, n_notfound = 0;
    char first[256] = {0};
    const int ITERS = 5000;

    for (int it = 0; it < ITERS; ++it) {
        uint8_t bits[401];
        size_t  n_bits = xnext(&s) % 401u;          // 0..400, incl. < 32
        for (size_t i = 0; i < n_bits; ++i)
            bits[i] = (uint8_t) (xnext(&s) & 1u);

        uint32_t needle  = xnext(&s);               // full random 32-bit word
        int      max_ham = (int) (xnext(&s) % 7u);  // 0..6

        // Plant 0..3 copies of the needle at random offsets, each with 0..7
        // random bit flips, so distances straddle max_ham and several
        // candidates can compete for "best".
        if (n_bits >= 32) {
            int copies = (int) (xnext(&s) % 4u);
            for (int c = 0; c < copies; ++c) {
                size_t off = xnext(&s) % (n_bits - 31u);
                plant_u32(bits, off, needle);
                int flips = (int) (xnext(&s) % 8u);
                for (int f = 0; f < flips; ++f)
                    bits[off + (xnext(&s) % 32u)] ^= 1u;
            }
        }

        // min_offset spans 0..n_bits-31, so it sometimes lands one past the
        // last legal window and exercises the immediate no-match branch.
        size_t min_off = (n_bits >= 32) ? (xnext(&s) % (n_bits - 30u)) : 0u;

        int    g_ham = -7, r_ham = -7;
        size_t g = asm_find_best(bits, n_bits, needle, max_ham, min_off, &g_ham);
        size_t r = ref_find_best(bits, n_bits, needle, max_ham, min_off, &r_ham);

        if (g != r || g_ham != r_ham) {
            if (mismatches == 0)
                snprintf(first, sizeof first,
                         "it=%d n=%zu mh=%d minoff=%zu got off=%ld ham=%d, want off=%ld ham=%d",
                         it, n_bits, max_ham, min_off,
                         (long) g, g_ham, (long) r, r_ham);
            ++mismatches;
        }
        if (g != NOMATCH) ++n_found; else ++n_notfound;
    }

    tap_okf(mismatches == 0,
            "%d randomized cases agree with the brute-force oracle (%d mismatches)",
            ITERS, mismatches);
    if (mismatches)
        tap_diag("first mismatch: %s", first);

    // Guard against a degenerate run that never plants in range: the
    // differential is only meaningful if both branches were actually hit.
    tap_okf(n_found > 0, "differential exercised the match-found path (%d cases)", n_found);
    tap_okf(n_notfound > 0, "differential exercised the no-match path (%d cases)", n_notfound);
}

int main(void)
{
    test_constant_pinned();
    test_exact_match();
    test_n_bits_equals_32();
    test_lsb_masking();
    test_best_not_first();
    test_tie_breaks_earliest();
    test_max_ham_boundary();
    test_min_offset();
    test_last_window();
    test_too_few_bits();
    test_no_match_resets_distance();
    test_out_ham_null();
    test_random_differential();
    return tap_done();
}
