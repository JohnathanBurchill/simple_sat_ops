/*

    Simple Satellite Operations  unit_tests/ax100_selftest.c

    Coverage:
      - ax100_opts_defaults populates the pycsplink-default profile.
      - ax100_hmac is deterministic, key/data-sensitive, AND matches
        byte-exact golden trailers computed independently with openssl
        (an external oracle that a frame->unframe round trip cannot give).
      - the CCSDS scrambler output is checked against the randomizer
        sequence generated independently from the LFSR polynomial (and the
        CCSDS-published opening bytes) -- not just round-tripped.
      - ax100_frame lays out preamble, ASM, Golay24 length, scrambled
        payload, postamble in the expected positions and lengths.
      - ax100_frame -> ax100_unframe round-trips every combination of
        the four togglable options (randomize, hmac, RS, syncword off
        + len_field off).
      - ax100_unframe recovers from 1..3 bit flips in the 3-byte Golay
        length header (Golay(24,12,8) correction radius).
      - ax100_unframe + RS correct up to t=16 byte errors in the
        on-wire payload; t+1 byte errors are reported as uncorrectable.
      - ax100_unframe rejects an HMAC-only frame whose payload has been
        tampered with.
      - ax100_unframe brute-forces the length when the Golay header is
        uncorrectable (>= 4 bit errors) AND RS + HMAC are both enabled,
        and reports out_used_golay_len = 0 in that case.
      - Missing ASM / truncated input / oversize packet all rejected.

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

#include "ax100.h"
#include "golay24.h"
#include "rs.h"
#include "tap.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define check(cond, what) tap_ok((cond), (what))

// xorshift32 — deterministic so failures are reproducible without
// having to also dump the seed.
static uint32_t xs_state = 0xc0ffee01u;
static uint32_t xs_next(void)
{
    uint32_t x = xs_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    xs_state = x;
    return x;
}
static void fill_pseudo(uint8_t *buf, size_t n, uint32_t seed)
{
    xs_state = seed;
    for (size_t i = 0; i < n; ++i) buf[i] = (uint8_t)xs_next();
}

static void flip_bit(uint8_t *buf, size_t bit_index)
{
    buf[bit_index / 8] ^= (uint8_t)(1u << (7 - (bit_index % 8)));
}

// --------------------------------------------------------- defaults

static void test_opts_defaults(void)
{
    fprintf(stderr, "ax100_opts_defaults:\n");
    ax100_opts_t o;
    memset(&o, 0xAB, sizeof o);  // pre-trash to prove the function writes every field
    ax100_opts_defaults(&o);
    check(o.hmac_key == NULL,    "hmac_key = NULL");
    check(o.hmac_key_len == 0,   "hmac_key_len = 0");
    check(o.randomize == 1,      "randomize = 1");
    check(o.reed_solomon == 0,   "reed_solomon = 0");
    check(o.len_field == 1,      "len_field = 1");
    check(o.syncword == 1,       "syncword = 1");
    check(o.prefill == 32,       "prefill = 32");
    check(o.tailfill == 1,       "tailfill = 1");

    // NULL arg is a no-op rather than a crash.
    ax100_opts_defaults(NULL);
    check(1, "ax100_opts_defaults(NULL) doesn't crash");
}

// ------------------------------------------------------------- hmac

// Properties (no known-vector here; pycsp reference is the source of
// truth but rather than hand-encoding a vector we lean on round-trip
// + determinism + key/data sensitivity).
static void test_hmac_properties(void)
{
    fprintf(stderr, "ax100_hmac:\n");
    const uint8_t key1[] = "frontiersat-test-key";
    const uint8_t key2[] = "frontiersat-test-key-alt";
    const uint8_t data1[] = "the rocket is fine";
    const uint8_t data2[] = "the rocket is not fine";

    uint8_t mac_a[4], mac_b[4], mac_c[4], mac_d[4];
    int rc;

    rc = ax100_hmac(key1, sizeof key1 - 1, data1, sizeof data1 - 1, mac_a);
    check(rc == 0, "key1+data1 -> rc 0");
    rc = ax100_hmac(key1, sizeof key1 - 1, data1, sizeof data1 - 1, mac_b);
    check(rc == 0 && memcmp(mac_a, mac_b, 4) == 0,
          "deterministic: same key + same data -> same MAC");

    rc = ax100_hmac(key2, sizeof key2 - 1, data1, sizeof data1 - 1, mac_c);
    check(rc == 0 && memcmp(mac_a, mac_c, 4) != 0,
          "key-sensitive: different key -> different MAC");

    rc = ax100_hmac(key1, sizeof key1 - 1, data2, sizeof data2 - 1, mac_d);
    check(rc == 0 && memcmp(mac_a, mac_d, 4) != 0,
          "data-sensitive: different data -> different MAC");

    // Absolute golden trailers (external oracle). Computed independently
    // with the openssl CLI, NOT by this code, so a symmetric bug in the
    // construction -- wrong truncation, raw key instead of SHA1(key)[:16],
    // wrong ipad/opad -- is caught; a frame->unframe round trip would not
    // see it. Reproduce a trailer with (key, data) from below:
    //   rkey=$(printf %s "$key" | openssl dgst -sha1 -binary | xxd -p -c256 | cut -c1-32)
    //   printf %s "$data" | openssl dgst -sha1 -mac HMAC \
    //       -macopt hexkey:$rkey -binary | xxd -p -c256 | cut -c1-8
    static const uint8_t GOLDEN_K1_D1[4] = { 0xEE, 0xC8, 0x29, 0x56 };
    static const uint8_t GOLDEN_K2_D1[4] = { 0x94, 0x90, 0xE8, 0xAE };
    static const uint8_t GOLDEN_K1_D2[4] = { 0x17, 0x39, 0x78, 0x9A };
    check(memcmp(mac_a, GOLDEN_K1_D1, 4) == 0,
          "HMAC(key1,data1) == openssl golden EE C8 29 56");
    check(memcmp(mac_c, GOLDEN_K2_D1, 4) == 0,
          "HMAC(key2,data1) == openssl golden 94 90 E8 AE");
    check(memcmp(mac_d, GOLDEN_K1_D2, 4) == 0,
          "HMAC(key1,data2) == openssl golden 17 39 78 9A");

    // Bad args: NULL trailer.
    rc = ax100_hmac(key1, sizeof key1 - 1, data1, sizeof data1 - 1, NULL);
    check(rc == -1, "NULL out_trailer -> -1");
}

// ------------------------------------------- scrambler external oracle

// Independently generate the CCSDS pseudo-randomizer sequence from the
// generator definition -- an 8-bit LFSR with feedback mask 0xA9 (the
// polynomial x^8 + x^7 + x^5 + x^3 + 1), seeded all-ones, output LSB-first
// into MSB-first bytes. This is derived from the polynomial, NOT copied
// from ax100.c's precomputed table, so it is a genuine external oracle: a
// symmetric typo in that table survives a frame->unframe round trip (both
// sides use the same wrong byte) but fails this comparison.
static void ccsds_sequence(uint8_t *out, size_t n)
{
    uint8_t reg = 0xFFu;
    for (size_t i = 0; i < n; ++i) {
        uint8_t b = 0;
        for (int k = 0; k < 8; ++k) {
            b = (uint8_t)((b << 1) | (reg & 1u));   // assemble MSB-first
            uint8_t fb = 0;
            for (int j = 0; j < 8; ++j)
                if ((0xA9u >> j) & 1u) fb ^= (uint8_t)((reg >> j) & 1u);
            reg = (uint8_t)((reg >> 1) | (fb << 7));
        }
        out[i] = b;
    }
}

static void test_scrambler_known_sequence(void)
{
    fprintf(stderr, "ax100 scrambler vs independent CCSDS sequence:\n");

    // CCSDS 131.0-B pseudo-randomizer opening bytes -- a third-party
    // reference (the same values gr-satellites and libfec document). Pinned
    // so a corrupted scrambler table is caught even if the LFSR oracle
    // below were itself wrong.
    static const uint8_t CCSDS_PUBLISHED_HEAD[16] = {
        0xFF, 0x48, 0x0E, 0xC0, 0x9A, 0x0D, 0x70, 0xBC,
        0x8E, 0x2C, 0x93, 0xAD, 0xA7, 0xB7, 0x46, 0xCE,
    };
    uint8_t seq[255];
    ccsds_sequence(seq, sizeof seq);
    check(memcmp(seq, CCSDS_PUBLISHED_HEAD, sizeof CCSDS_PUBLISHED_HEAD) == 0,
          "LFSR oracle reproduces the CCSDS-published opening 16 bytes");

    // Frame an all-zero payload with ONLY the scrambler on (no length, no
    // sync, no pre/postamble): the on-wire bytes are then 0 ^ table[i],
    // i.e. the scrambler sequence itself, exposed for direct comparison.
    ax100_opts_t o;
    ax100_opts_defaults(&o);
    o.randomize = 1;
    o.len_field = 0;
    o.syncword  = 0;
    o.prefill   = 0;
    o.tailfill  = 0;

    uint8_t zeros[255] = {0};
    uint8_t frame[255];
    ssize_t n = ax100_frame(zeros, sizeof zeros, &o, frame, sizeof frame);
    check(n == (ssize_t)sizeof zeros, "framed 255 zero bytes -> 255 on-wire bytes");
    check(n == (ssize_t)sizeof zeros && memcmp(frame, seq, sizeof zeros) == 0,
          "scrambler output over zeros == independent CCSDS sequence (255 bytes)");
    check(memcmp(frame, CCSDS_PUBLISHED_HEAD, sizeof CCSDS_PUBLISHED_HEAD) == 0,
          "scrambler output head == CCSDS-published 16 bytes");

    // Wrap-around: the table has 255 entries and indexing is i % 255, so
    // on-wire byte 255 must repeat table[0], 256 -> table[1], and so on.
    uint8_t zeros2[300] = {0};
    uint8_t frame2[300];
    ssize_t n2 = ax100_frame(zeros2, sizeof zeros2, &o, frame2, sizeof frame2);
    check(n2 == (ssize_t)sizeof zeros2, "framed 300 zero bytes -> 300 on-wire bytes");
    int wrap_ok = (n2 == (ssize_t)sizeof zeros2);
    for (size_t i = 255; wrap_ok && i < sizeof zeros2; ++i)
        if (frame2[i] != seq[i - 255]) wrap_ok = 0;
    check(wrap_ok, "scrambler table wraps at 255 (byte 255 repeats table[0])");
}

// -------------------------------------------------- frame structure

// Plain frame (no scramble, no hmac, no RS): every byte in the output is
// inspectable. Verifies position + length math + Golay24 length encoding.
static void test_frame_structure_plain(void)
{
    fprintf(stderr, "ax100_frame structure (plain):\n");
    ax100_opts_t o;
    ax100_opts_defaults(&o);
    o.randomize = 0;     // expose the inner bytes in the on-wire frame
    o.prefill = 8;       // shorter than default 32, easier to inspect
    o.tailfill = 2;

    uint8_t pkt[24];
    fill_pseudo(pkt, sizeof pkt, 0xa1b2c3d4u);

    uint8_t frame[256];
    ssize_t n = ax100_frame(pkt, sizeof pkt, &o, frame, sizeof frame);
    size_t expected = (size_t)o.prefill + 4 + 3 + sizeof pkt + (size_t)o.tailfill;
    check(n == (ssize_t)expected, "total length = prefill+4+3+pkt+tailfill");

    int prefill_ok = 1;
    for (int i = 0; i < o.prefill; ++i) if (frame[i] != 0xAA) prefill_ok = 0;
    check(prefill_ok, "prefill is N x 0xAA");

    check(frame[o.prefill + 0] == AX100_ASM_0 &&
          frame[o.prefill + 1] == AX100_ASM_1 &&
          frame[o.prefill + 2] == AX100_ASM_2 &&
          frame[o.prefill + 3] == AX100_ASM_3,
          "ASM sync word at offset prefill..prefill+3");

    // Golay-encoded inner length (= sizeof pkt with no HMAC / no RS).
    uint32_t g_expected = golay24_encode((uint16_t)sizeof pkt);
    uint32_t g_actual = ((uint32_t)frame[o.prefill + 4] << 16)
                      | ((uint32_t)frame[o.prefill + 5] <<  8)
                      |  (uint32_t)frame[o.prefill + 6];
    check(g_actual == g_expected,
          "Golay-encoded length field at offset prefill+4..+6");

    // With randomize=0 the inner payload sits on-wire verbatim.
    check(memcmp(&frame[o.prefill + 7], pkt, sizeof pkt) == 0,
          "plain payload (randomize=0) appears verbatim post-Golay");

    int tail_ok = 1;
    for (int i = 0; i < o.tailfill; ++i) {
        if (frame[(size_t)n - o.tailfill + i] != 0xAA) tail_ok = 0;
    }
    check(tail_ok, "tailfill is N x 0xAA");
}

// ------------------------------------------------ round-trip matrix

typedef struct {
    int randomize;
    int hmac;
    int rs;
    int syncword;
    int len_field;
    const char *label;
} rt_case_t;

static void run_round_trip(const rt_case_t *c)
{
    const uint8_t key[] = "rtkey-frontiersat";
    ax100_opts_t o;
    ax100_opts_defaults(&o);
    o.randomize = c->randomize;
    o.reed_solomon = c->rs;
    o.syncword = c->syncword;
    o.len_field = c->len_field;
    if (c->hmac) {
        o.hmac_key = key;
        o.hmac_key_len = sizeof key - 1;
    }

    // RS caps payload + HMAC at RS_K = 223.
    uint8_t pkt[200];
    size_t pkt_len = c->rs ? (RS_K - (c->hmac ? 4 : 0) - 16) : sizeof pkt;
    fill_pseudo(pkt, pkt_len, 0xd1ce0001u + (uint32_t)(uintptr_t)c);

    uint8_t frame[1024];
    ssize_t n = ax100_frame(pkt, pkt_len, &o, frame, sizeof frame);
    check(n > 0, c->label);
    if (n <= 0) return;

    // unframe input starts AT the ASM (or just after the prefill when
    // syncword is off — the next 3 bytes are then the Golay header).
    const uint8_t *in = frame + o.prefill;
    size_t in_len = (size_t)n - o.prefill - o.tailfill;

    uint8_t out[1024];
    int golay_errs = -2, hmac_ok = -2, rs_errs = -2, used_golay = -2;
    ssize_t r = ax100_unframe(in, in_len, &o, out, sizeof out,
                              &golay_errs, &hmac_ok, &rs_errs, &used_golay,
                              NULL);
    char msg[128];
    snprintf(msg, sizeof msg, "  %s -> length matches", c->label);
    check(r == (ssize_t)pkt_len, msg);
    snprintf(msg, sizeof msg, "  %s -> payload matches", c->label);
    check(r == (ssize_t)pkt_len && memcmp(out, pkt, pkt_len) == 0, msg);

    if (c->len_field) {
        snprintf(msg, sizeof msg, "  %s -> Golay errors = 0", c->label);
        check(golay_errs == 0, msg);
    }
    if (c->hmac) {
        snprintf(msg, sizeof msg, "  %s -> hmac_ok = 1", c->label);
        check(hmac_ok == 1, msg);
    }
    if (c->rs) {
        snprintf(msg, sizeof msg, "  %s -> rs_errors = 0", c->label);
        check(rs_errs == 0, msg);
    }
}

static void test_round_trip_permutations(void)
{
    fprintf(stderr, "ax100 frame -> unframe round trip (option matrix):\n");
    const rt_case_t cases[] = {
        { 0, 0, 0, 1, 1, "plain" },
        { 1, 0, 0, 1, 1, "scrambler only" },
        { 1, 1, 0, 1, 1, "scrambler + hmac" },
        { 1, 0, 1, 1, 1, "scrambler + RS" },
        { 1, 1, 1, 1, 1, "scrambler + hmac + RS (pycsplink uplink)" },
        { 0, 0, 0, 0, 1, "syncword off" },
        { 0, 0, 0, 1, 0, "len_field off (consume-rest)" },
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
        run_round_trip(&cases[i]);
    }
}

// ----------------------------------------- Golay header error recovery

// Flip up to k bits in the 3-byte Golay header. With k <= 3 the
// Golay(24,12,8) code corrects cleanly, so unframe succeeds without
// touching the brute-force fallback. Tests several distinct bit
// positions to exercise the lookup, not just one.
static void test_golay_header_recovery(void)
{
    fprintf(stderr, "ax100_unframe (Golay header bit-flip recovery):\n");
    ax100_opts_t o;
    ax100_opts_defaults(&o);
    o.prefill = 4;
    o.tailfill = 1;

    uint8_t pkt[40];
    fill_pseudo(pkt, sizeof pkt, 0xbeef0001u);
    uint8_t frame[256];
    ssize_t n = ax100_frame(pkt, sizeof pkt, &o, frame, sizeof frame);
    check(n > 0, "preflight frame built");
    if (n <= 0) return;

    // Bit positions to flip in the 3-byte header (24 bits total).
    // Pick a spread that exercises multiple Golay codeword neighbors.
    static const int positions[] = { 1, 5, 11, 17, 22, 9, 15, 19, 23 };
    for (int n_errs = 1; n_errs <= 3; ++n_errs) {
        // Copy fresh, flip the first n_errs distinct positions.
        uint8_t corrupt[256];
        memcpy(corrupt, frame, (size_t)n);
        size_t golay_off = o.prefill + 4;  // 4 ASM bytes between prefill and Golay
        for (int i = 0; i < n_errs; ++i) {
            flip_bit(corrupt + golay_off, (size_t)positions[i]);
        }

        uint8_t out[256];
        int golay_errs = -2, hmac_ok = -2, rs_errs = -2, used_golay = -2;
        ssize_t r = ax100_unframe(corrupt + o.prefill,
                                  (size_t)n - o.prefill - o.tailfill,
                                  &o, out, sizeof out,
                                  &golay_errs, &hmac_ok, &rs_errs,
                                  &used_golay, NULL);
        char msg[128];
        snprintf(msg, sizeof msg,
                 "%d-bit Golay error: unframe still recovers", n_errs);
        check(r == (ssize_t)sizeof pkt
              && memcmp(out, pkt, sizeof pkt) == 0, msg);
        snprintf(msg, sizeof msg,
                 "%d-bit Golay error: out_golay_errors = %d",
                 n_errs, n_errs);
        check(golay_errs == n_errs, msg);
    }
}

// ----------------------------------------------- RS payload recovery

// Inject byte errors into the on-wire scrambled payload (between the
// Golay header and the postamble). Up to RS_NROOTS/2 = 16 errors must
// be corrected; 17 errors must be reported as uncorrectable.
static void test_rs_payload_recovery(void)
{
    fprintf(stderr, "ax100_unframe (RS byte-error recovery):\n");
    const uint8_t key[] = "rs-test";
    ax100_opts_t o;
    ax100_opts_defaults(&o);
    o.reed_solomon = 1;
    o.hmac_key = key;
    o.hmac_key_len = sizeof key - 1;
    o.prefill = 4;
    o.tailfill = 1;

    // Pick a small packet so the payload is short and the corruption
    // pattern is easy to reason about. With RS the on-wire payload
    // size = pkt_len + 4 (HMAC) + 32 (RS parity).
    uint8_t pkt[40];
    fill_pseudo(pkt, sizeof pkt, 0xb0b0c0c0u);
    uint8_t frame[1024];
    ssize_t n = ax100_frame(pkt, sizeof pkt, &o, frame, sizeof frame);
    check(n > 0, "preflight frame built (RS + HMAC)");
    if (n <= 0) return;

    size_t payload_off = (size_t)o.prefill + 4 + 3;  // past prefill+ASM+Golay
    size_t payload_len = (size_t)n - payload_off - (size_t)o.tailfill;

    static const int err_counts[] = { 1, 8, 16 };
    for (size_t k = 0; k < sizeof err_counts / sizeof err_counts[0]; ++k) {
        int n_errs = err_counts[k];
        uint8_t corrupt[1024];
        memcpy(corrupt, frame, (size_t)n);
        for (int i = 0; i < n_errs; ++i) {
            // Spread the errors across the payload at unique positions.
            size_t pos = payload_off + (size_t)i * (payload_len / 17);
            corrupt[pos] ^= 0xA5;
        }
        uint8_t out[1024];
        int golay_errs = -2, hmac_ok = -2, rs_errs = -2, used_golay = -2;
        ssize_t r = ax100_unframe(corrupt + o.prefill,
                                  (size_t)n - o.prefill - o.tailfill,
                                  &o, out, sizeof out,
                                  &golay_errs, &hmac_ok, &rs_errs,
                                  &used_golay, NULL);
        char msg[128];
        snprintf(msg, sizeof msg, "%d byte errors: RS corrects and HMAC validates",
                 n_errs);
        check(r == (ssize_t)sizeof pkt
              && memcmp(out, pkt, sizeof pkt) == 0
              && hmac_ok == 1, msg);
        snprintf(msg, sizeof msg, "%d byte errors: rs_errors = %d",
                 n_errs, n_errs);
        check(rs_errs == n_errs, msg);
    }

    // 17 byte errors -> uncorrectable. The brute-force length search is
    // also unlikely to find a chance HMAC match, so unframe returns -1.
    {
        uint8_t corrupt[1024];
        memcpy(corrupt, frame, (size_t)n);
        for (int i = 0; i < 17; ++i) {
            size_t pos = payload_off + (size_t)i * (payload_len / 18);
            corrupt[pos] ^= 0xA5;
        }
        uint8_t out[1024];
        ssize_t r = ax100_unframe(corrupt + o.prefill,
                                  (size_t)n - o.prefill - o.tailfill,
                                  &o, out, sizeof out,
                                  NULL, NULL, NULL, NULL, NULL);
        check(r == -1, "17 byte errors: unframe rejects (uncorrectable)");
    }
}

// ----------------------------------------------- HMAC tamper detection

// Without RS, a single byte flip in the on-wire payload descrambles to
// corrupted data; the HMAC computed over that data won't match the
// trailer, so unframe must reject.
static void test_hmac_tamper(void)
{
    fprintf(stderr, "ax100_unframe (HMAC tamper without RS):\n");
    const uint8_t key[] = "hmac-only";
    ax100_opts_t o;
    ax100_opts_defaults(&o);
    o.reed_solomon = 0;
    o.hmac_key = key;
    o.hmac_key_len = sizeof key - 1;
    o.prefill = 4;
    o.tailfill = 1;

    uint8_t pkt[32];
    fill_pseudo(pkt, sizeof pkt, 0x11223344u);
    uint8_t frame[256];
    ssize_t n = ax100_frame(pkt, sizeof pkt, &o, frame, sizeof frame);
    check(n > 0, "preflight frame built");
    if (n <= 0) return;

    // Flip a byte in the middle of the on-wire payload.
    size_t payload_off = (size_t)o.prefill + 4 + 3;
    frame[payload_off + 5] ^= 0xFF;

    uint8_t out[256];
    int hmac_ok = -2;
    ssize_t r = ax100_unframe(frame + o.prefill,
                              (size_t)n - o.prefill - o.tailfill,
                              &o, out, sizeof out,
                              NULL, &hmac_ok, NULL, NULL, NULL);
    check(r == -1, "tampered payload (HMAC only): unframe rejects");
    check(hmac_ok == 0, "out_hmac_ok = 0 on mismatch");
}

// ---------------------------------------- brute-force length fallback

// Make the Golay header uncorrectable (>=4 bit errors) but leave the
// scrambled payload + RS parity untouched. With RS + HMAC on,
// ax100_unframe scans length candidates and picks the one that both
// RS-decodes and HMAC-validates. The original length wins.
static void test_brute_force_length_fallback(void)
{
    fprintf(stderr, "ax100_unframe (brute-force length fallback):\n");
    const uint8_t key[] = "bf-key";
    ax100_opts_t o;
    ax100_opts_defaults(&o);
    o.reed_solomon = 1;
    o.hmac_key = key;
    o.hmac_key_len = sizeof key - 1;
    o.prefill = 4;
    o.tailfill = 1;

    uint8_t pkt[50];
    fill_pseudo(pkt, sizeof pkt, 0xfeedbeefu);
    uint8_t frame[1024];
    ssize_t n = ax100_frame(pkt, sizeof pkt, &o, frame, sizeof frame);
    check(n > 0, "preflight frame built (RS + HMAC)");
    if (n <= 0) return;

    // 5 bit flips in the 3-byte Golay header — well past the 3-bit
    // correction radius.
    size_t golay_off = (size_t)o.prefill + 4;
    static const int bits[] = { 0, 4, 9, 14, 20 };
    for (size_t i = 0; i < sizeof bits / sizeof bits[0]; ++i) {
        flip_bit(frame + golay_off, (size_t)bits[i]);
    }

    uint8_t out[1024];
    int golay_errs = -2, hmac_ok = -2, rs_errs = -2, used_golay = -2;
    ssize_t r = ax100_unframe(frame + o.prefill,
                              (size_t)n - o.prefill - o.tailfill,
                              &o, out, sizeof out,
                              &golay_errs, &hmac_ok, &rs_errs, &used_golay,
                              NULL);
    check(r == (ssize_t)sizeof pkt
          && memcmp(out, pkt, sizeof pkt) == 0,
          "brute-force recovers payload despite 5-bit Golay error");
    check(used_golay == 0, "out_used_golay_len = 0 (brute force succeeded)");
    check(hmac_ok == 1, "HMAC validates on the recovered candidate");
    // We can't reliably assert golay_errs > 3 here: 5 bit flips can land
    // within Golay's correction radius of a *different* codeword, in
    // which case the decoder "succeeds" with a wrong length (and the
    // brute force picks up the slack). out_used_golay_len = 0 is the
    // real signal that the original Golay decode wasn't the one that
    // recovered the frame.
    (void)golay_errs;
}

// --------------------------------------------------- bad inputs

static void test_bad_inputs(void)
{
    fprintf(stderr, "ax100_unframe / ax100_frame (bad inputs):\n");
    ax100_opts_t o;
    ax100_opts_defaults(&o);

    uint8_t out[64];
    // Missing ASM: arbitrary 16 bytes that don't start with 0x93 0x0B 0x51 0xDE.
    uint8_t no_asm[16];
    memset(no_asm, 0xCC, sizeof no_asm);
    ssize_t r = ax100_unframe(no_asm, sizeof no_asm, &o, out, sizeof out,
                              NULL, NULL, NULL, NULL, NULL);
    check(r == -1, "unframe: missing ASM -> -1");

    // Truncated: under 4 bytes (can't even check ASM).
    uint8_t short_in[2] = { AX100_ASM_0, AX100_ASM_1 };
    r = ax100_unframe(short_in, sizeof short_in, &o, out, sizeof out,
                      NULL, NULL, NULL, NULL, NULL);
    check(r == -1, "unframe: truncated input -> -1");

    // ASM + truncated Golay header (only 2 of 3 length bytes).
    uint8_t short2[6] = {
        AX100_ASM_0, AX100_ASM_1, AX100_ASM_2, AX100_ASM_3, 0x00, 0x00
    };
    r = ax100_unframe(short2, sizeof short2, &o, out, sizeof out,
                      NULL, NULL, NULL, NULL, NULL);
    check(r == -1, "unframe: ASM but truncated Golay header -> -1");

    // Frame: oversize packet with RS = 1 (needed > RS_K).
    uint8_t big[300];
    fill_pseudo(big, sizeof big, 0xdeadbeefu);
    ax100_opts_t o2;
    ax100_opts_defaults(&o2);
    o2.reed_solomon = 1;
    uint8_t frame[1024];
    ssize_t n = ax100_frame(big, sizeof big, &o2, frame, sizeof frame);
    check(n == -1, "frame: packet > 223 with RS=1 -> -1");

    // Frame: NULL packet, packet_len > 0.
    n = ax100_frame(NULL, 10, &o, frame, sizeof frame);
    check(n == -1, "frame: NULL packet + non-zero len -> -1");

    // Frame: out_buf too small.
    uint8_t pkt[10];
    fill_pseudo(pkt, sizeof pkt, 0x12345678u);
    uint8_t tiny[8];
    n = ax100_frame(pkt, sizeof pkt, &o, tiny, sizeof tiny);
    check(n == -1, "frame: out_buf too small -> -1");
}

int main(void)
{
    test_opts_defaults();
    test_hmac_properties();
    test_scrambler_known_sequence();
    test_frame_structure_plain();
    test_round_trip_permutations();
    test_golay_header_recovery();
    test_rs_payload_recovery();
    test_hmac_tamper();
    test_brute_force_length_fallback();
    test_bad_inputs();
    return tap_done();
}
