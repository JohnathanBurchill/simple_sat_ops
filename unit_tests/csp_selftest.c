/*

    Simple Satellite Operations  unit_tests/csp_selftest.c

    Coverage for src/proto/csp.c. The CSP v1 header packs six fields of
    odd widths (2 + 5 + 5 + 6 + 6 + 8 bits) into a single 32-bit big-
    endian word; an off-by-one bit shift would silently misroute frames
    on the link. The selftest pins the wire bytes for a known fixture,
    exhaustively round-trips every {min,max} corner, and audits the
    error returns + CRC-32 against known IEEE 802.3 test vectors.

    What's covered:
      - csp_v1_encode produces the exact 4-byte big-endian wire layout
        for a known fixture (catches bit-shift / endian regressions).
      - Each field, isolated, only flips the bits it owns (catches
        cross-field aliasing).
      - Exhaustive {min,max} round-trip across all 6 fields (64 combos)
        plus a randomised round-trip across many iterations.
      - Payload bytes round-trip verbatim across multiple sizes.
      - Out-of-range field values rejected (-1 return).
      - Buffer-too-small rejected (-1 return).
      - NULL pointers rejected for both encode and decode.
      - csp_crc32_zlib matches IEEE 802.3 test vectors.

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

#include "csp.h"
#include "tap.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

// xorshift32 — deterministic so failures are reproducible without
// also dumping the seed.
static uint32_t xs_state = 0xc0ffee01u;
static uint32_t xs_next(void)
{
    uint32_t x = xs_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    xs_state = x;
    return x;
}

// ------------------------------------------------------------------
// 1. Byte-layout pin: the wire bytes for a known fixture must match.
// ------------------------------------------------------------------

static void test_encode_byte_layout(void)
{
    // prio=2, src=10, dst=5, dport=20, sport=3, flags=0x09.
    //   h = (2<<30)|(10<<25)|(5<<20)|(20<<14)|(3<<8)|0x09
    //     = 0x80000000 + 0x14000000 + 0x00500000 + 0x00050000
    //     + 0x00000300 + 0x00000009 = 0x94550309
    csp_v1_header_t h = { .prio = 2,  .src = 10, .dst = 5,
                          .dport = 20, .sport = 3, .flags = 0x09 };
    uint8_t buf[16] = {0};
    ssize_t n = csp_v1_encode(&h, NULL, 0, buf, sizeof buf);
    tap_okf(n == 4, "encode no-payload returns 4 (got %zd)", (ssize_t) n);
    tap_okf(buf[0] == 0x94 && buf[1] == 0x55
         && buf[2] == 0x03 && buf[3] == 0x09,
            "wire bytes %02x %02x %02x %02x match 94 55 03 09",
            buf[0], buf[1], buf[2], buf[3]);
}

// ------------------------------------------------------------------
// 2. Field isolation: each field at max, others at 0, only its bits flip.
// ------------------------------------------------------------------

static void test_encode_field_isolation(void)
{
    uint8_t buf[8];
    csp_v1_header_t h;

    // prio = 3 (bits 31:30 → byte 0 bits 7:6 = 11 → 0xC0).
    memset(&h, 0, sizeof h); h.prio = 3;
    csp_v1_encode(&h, NULL, 0, buf, sizeof buf);
    tap_okf(buf[0] == 0xC0 && buf[1] == 0x00 && buf[2] == 0x00 && buf[3] == 0x00,
            "prio=3 isolated -> C0 00 00 00 (got %02x %02x %02x %02x)",
            buf[0], buf[1], buf[2], buf[3]);

    // src = 31 (bits 29:25 → byte 0 bits 5:1 = 11111 → 0x3E).
    memset(&h, 0, sizeof h); h.src = 31;
    csp_v1_encode(&h, NULL, 0, buf, sizeof buf);
    tap_okf(buf[0] == 0x3E && buf[1] == 0x00 && buf[2] == 0x00 && buf[3] == 0x00,
            "src=31 isolated -> 3E 00 00 00 (got %02x %02x %02x %02x)",
            buf[0], buf[1], buf[2], buf[3]);

    // dst = 31 (bits 24:20 → byte 0 bit 0 + byte 1 bits 7:4 = 1 1111 → 01 F0).
    memset(&h, 0, sizeof h); h.dst = 31;
    csp_v1_encode(&h, NULL, 0, buf, sizeof buf);
    tap_okf(buf[0] == 0x01 && buf[1] == 0xF0 && buf[2] == 0x00 && buf[3] == 0x00,
            "dst=31 isolated -> 01 F0 00 00 (got %02x %02x %02x %02x)",
            buf[0], buf[1], buf[2], buf[3]);

    // dport = 63 (bits 19:14 → byte 1 bits 3:0 + byte 2 bits 7:6 = 1111 11 → 0F C0).
    memset(&h, 0, sizeof h); h.dport = 63;
    csp_v1_encode(&h, NULL, 0, buf, sizeof buf);
    tap_okf(buf[0] == 0x00 && buf[1] == 0x0F && buf[2] == 0xC0 && buf[3] == 0x00,
            "dport=63 isolated -> 00 0F C0 00 (got %02x %02x %02x %02x)",
            buf[0], buf[1], buf[2], buf[3]);

    // sport = 63 (bits 13:8 → byte 2 bits 5:0 = 111111 → 3F).
    memset(&h, 0, sizeof h); h.sport = 63;
    csp_v1_encode(&h, NULL, 0, buf, sizeof buf);
    tap_okf(buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0x3F && buf[3] == 0x00,
            "sport=63 isolated -> 00 00 3F 00 (got %02x %02x %02x %02x)",
            buf[0], buf[1], buf[2], buf[3]);

    // flags = 0xFF (bits 7:0 → byte 3 = FF).
    memset(&h, 0, sizeof h); h.flags = 0xFF;
    csp_v1_encode(&h, NULL, 0, buf, sizeof buf);
    tap_okf(buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0x00 && buf[3] == 0xFF,
            "flags=0xFF isolated -> 00 00 00 FF (got %02x %02x %02x %02x)",
            buf[0], buf[1], buf[2], buf[3]);
}

// ------------------------------------------------------------------
// 3. Exhaustive {min,max} round-trip across all 6 fields (64 combos).
// ------------------------------------------------------------------

static void test_roundtrip_corner_combinations(void)
{
    int failed = 0;
    for (int combo = 0; combo < 64; ++combo) {
        csp_v1_header_t h = {
            .prio  = (combo & (1 << 0)) ? 3    : 0,
            .src   = (combo & (1 << 1)) ? 31   : 0,
            .dst   = (combo & (1 << 2)) ? 31   : 0,
            .dport = (combo & (1 << 3)) ? 63   : 0,
            .sport = (combo & (1 << 4)) ? 63   : 0,
            .flags = (combo & (1 << 5)) ? 0xFF : 0,
        };
        uint8_t buf[4];
        if (csp_v1_encode(&h, NULL, 0, buf, sizeof buf) != 4) {
            ++failed; continue;
        }
        csp_v1_header_t back;
        if (csp_v1_decode(buf, &back) != 0) { ++failed; continue; }
        if (back.prio != h.prio || back.src != h.src || back.dst != h.dst
         || back.dport != h.dport || back.sport != h.sport
         || back.flags != h.flags) {
            ++failed;
        }
    }
    tap_okf(failed == 0,
            "exhaustive {min,max}^6 round-trip (64 combos, %d failed)",
            failed);
}

// ------------------------------------------------------------------
// 4. Randomised round-trip across the full valid range.
// ------------------------------------------------------------------

static void test_roundtrip_randomised(void)
{
    xs_state = 0xc0ffee01u;
    int iters = 4096;
    int failed = 0;
    for (int i = 0; i < iters; ++i) {
        uint32_t r = xs_next();
        csp_v1_header_t h = {
            .prio  = (uint8_t)( r        & 0x03),
            .src   = (uint8_t)((r >>  2) & 0x1F),
            .dst   = (uint8_t)((r >>  7) & 0x1F),
            .dport = (uint8_t)((r >> 12) & 0x3F),
            .sport = (uint8_t)((r >> 18) & 0x3F),
            .flags = (uint8_t)((r >> 24) & 0xFF),
        };
        uint8_t payload[32];
        uint32_t plen_r = xs_next();
        size_t plen = (size_t)(plen_r % (sizeof payload + 1));
        for (size_t j = 0; j < plen; ++j) {
            payload[j] = (uint8_t) xs_next();
        }
        uint8_t buf[64];
        ssize_t n = csp_v1_encode(&h, plen ? payload : NULL, plen,
                                  buf, sizeof buf);
        if (n != (ssize_t)(4 + plen)) { ++failed; continue; }
        csp_v1_header_t back;
        if (csp_v1_decode(buf, &back) != 0) { ++failed; continue; }
        if (back.prio != h.prio || back.src != h.src || back.dst != h.dst
         || back.dport != h.dport || back.sport != h.sport
         || back.flags != h.flags
         || (plen > 0 && memcmp(buf + 4, payload, plen) != 0)) {
            ++failed;
        }
    }
    tap_okf(failed == 0, "randomised round-trip (%d iters, %d failed)",
            iters, failed);
}

// ------------------------------------------------------------------
// 5. Out-of-range field rejection.
// ------------------------------------------------------------------

static void test_encode_rejects_oversize_fields(void)
{
    uint8_t buf[8];
    csp_v1_header_t h;

    memset(&h, 0, sizeof h); h.prio = 4;
    tap_ok(csp_v1_encode(&h, NULL, 0, buf, sizeof buf) == -1,
           "encode rejects prio=4 (>3)");
    memset(&h, 0, sizeof h); h.src = 32;
    tap_ok(csp_v1_encode(&h, NULL, 0, buf, sizeof buf) == -1,
           "encode rejects src=32 (>31)");
    memset(&h, 0, sizeof h); h.dst = 32;
    tap_ok(csp_v1_encode(&h, NULL, 0, buf, sizeof buf) == -1,
           "encode rejects dst=32 (>31)");
    memset(&h, 0, sizeof h); h.dport = 64;
    tap_ok(csp_v1_encode(&h, NULL, 0, buf, sizeof buf) == -1,
           "encode rejects dport=64 (>63)");
    memset(&h, 0, sizeof h); h.sport = 64;
    tap_ok(csp_v1_encode(&h, NULL, 0, buf, sizeof buf) == -1,
           "encode rejects sport=64 (>63)");
    // flags is 8 bits → no oversize case; verify 0xFF accepted as a
    // sanity counter-test.
    memset(&h, 0, sizeof h); h.flags = 0xFF;
    tap_ok(csp_v1_encode(&h, NULL, 0, buf, sizeof buf) == 4,
           "encode accepts flags=0xFF (full byte width)");
}

// ------------------------------------------------------------------
// 6. Buffer-size and NULL guards.
// ------------------------------------------------------------------

static void test_encode_buffer_guards(void)
{
    csp_v1_header_t h = { .prio = 1, .src = 2, .dst = 3,
                          .dport = 4, .sport = 5, .flags = 0 };
    uint8_t buf[16];   // sized to fit the legitimate 12-byte write below
    uint8_t payload[8] = {1,2,3,4,5,6,7,8};

    tap_ok(csp_v1_encode(&h, NULL, 0, buf, 3) == -1,
           "encode rejects out_buf_size=3 (need 4)");
    tap_ok(csp_v1_encode(&h, payload, 8, buf, 11) == -1,
           "encode rejects out_buf_size=11 (need 12)");
    tap_ok(csp_v1_encode(&h, payload, 8, buf, 12) == 12,
           "encode accepts out_buf_size=12 for 8-byte payload");
    tap_ok(csp_v1_encode(NULL,   NULL, 0, buf, sizeof buf) == -1,
           "encode rejects NULL header");
    tap_ok(csp_v1_encode(&h,     NULL, 0, NULL, 4) == -1,
           "encode rejects NULL out_buf");
    // payload == NULL with payload_len > 0 is invalid; with == 0 is OK.
    tap_ok(csp_v1_encode(&h, NULL, 4, buf, sizeof buf) == -1,
           "encode rejects payload=NULL with payload_len>0");
    tap_ok(csp_v1_encode(&h, NULL, 0, buf, sizeof buf) == 4,
           "encode accepts payload=NULL with payload_len=0");
}

static void test_decode_null_safety(void)
{
    uint8_t bytes[4] = {0x94, 0x55, 0x03, 0x09};
    csp_v1_header_t h;
    tap_ok(csp_v1_decode(NULL, &h) == -1,
           "decode rejects NULL bytes");
    tap_ok(csp_v1_decode(bytes, NULL) == -1,
           "decode rejects NULL out_hdr");
    tap_ok(csp_v1_decode(bytes, &h) == 0,
           "decode accepts valid args");
}

// ------------------------------------------------------------------
// 7. Payload pass-through across multiple sizes.
// ------------------------------------------------------------------

static void test_payload_passthrough(void)
{
    csp_v1_header_t h = { .prio = 2, .src = 1, .dst = 2,
                          .dport = 3, .sport = 4, .flags = 0 };
    uint8_t buf[256];
    uint8_t payload[200];
    for (size_t i = 0; i < sizeof payload; ++i) payload[i] = (uint8_t) i;

    const size_t sizes[] = {0, 1, 4, 32, 100, 199, 200};
    for (size_t s = 0; s < sizeof sizes / sizeof sizes[0]; ++s) {
        size_t n = sizes[s];
        ssize_t got = csp_v1_encode(&h, n ? payload : NULL, n,
                                    buf, sizeof buf);
        tap_okf(got == (ssize_t)(4 + n),
                "payload pass-through size=%zu return value", n);
        if (n > 0) {
            tap_okf(memcmp(buf + 4, payload, n) == 0,
                    "payload pass-through size=%zu bytes verbatim", n);
        }
    }
}

// ------------------------------------------------------------------
// 8. csp_crc32_zlib against IEEE 802.3 / zlib known vectors.
// ------------------------------------------------------------------

static void test_crc32_known_vectors(void)
{
    // All values are the canonical CRC-32 (poly 0xEDB88320 reflected,
    // init 0xFFFFFFFF, final XOR 0xFFFFFFFF). Reproducible with
    // `python3 -c 'import zlib;print(hex(zlib.crc32(b"...")))'`.
    struct {
        const char *data;
        uint32_t    expected;
    } v[] = {
        { "",                                              0x00000000u },
        { "a",                                             0xe8b7be43u },
        { "abc",                                           0x352441c2u },
        { "123456789",                                     0xcbf43926u },
        { "The quick brown fox jumps over the lazy dog",   0x414fa339u },
    };
    for (size_t i = 0; i < sizeof v / sizeof v[0]; ++i) {
        uint32_t got = csp_crc32_zlib((const uint8_t *) v[i].data,
                                      strlen(v[i].data));
        tap_okf(got == v[i].expected,
                "crc32(\"%s\") = 0x%08x (want 0x%08x)",
                v[i].data, got, v[i].expected);
    }

    // CRC of a 4-byte CSP header — useful regression check that the
    // CRC computation is byte-stable across builds (it's called once
    // per decoded frame on the link's CRC trailer path).
    csp_v1_header_t h = { .prio = 2, .src = 10, .dst = 5,
                          .dport = 20, .sport = 3, .flags = 0x09 };
    uint8_t hdr[4];
    csp_v1_encode(&h, NULL, 0, hdr, sizeof hdr);
    uint32_t crc_header = csp_crc32_zlib(hdr, 4);
    // Reference: python3 -c 'import zlib;print(hex(zlib.crc32(bytes([0x94,0x55,0x03,0x09]))))'
    tap_okf(crc_header == 0x2be0aed3u,
            "crc32 of 4-byte CSP header == 0x2be0aed3 (got 0x%08x)",
            crc_header);
}

// ------------------------------------------------------------------
// 9. Reserved-bits clearance: decode discards bits 7:6 of the flags
//    byte that the codec doesn't define as flags. Strictly the codec
//    treats the whole byte as flags, so this is more of a documentation
//    pin than a correctness check — but if someone ever splits the
//    upper nibble for a new feature, the round-trip must still hold.
// ------------------------------------------------------------------

static void test_decode_flags_full_byte(void)
{
    uint8_t bytes[4] = {0x00, 0x00, 0x00, 0xAB};
    csp_v1_header_t h;
    tap_ok(csp_v1_decode(bytes, &h) == 0,
           "decode flags=0xAB succeeds");
    tap_okf(h.flags == 0xAB,
            "decode preserves full flags byte (got 0x%02x)", h.flags);
    tap_ok(h.prio == 0 && h.src == 0 && h.dst == 0
        && h.dport == 0 && h.sport == 0,
           "decode flags-only frame leaves all other fields 0");
}

int main(void)
{
    test_encode_byte_layout();
    test_encode_field_isolation();
    test_roundtrip_corner_combinations();
    test_roundtrip_randomised();
    test_encode_rejects_oversize_fields();
    test_encode_buffer_guards();
    test_decode_null_safety();
    test_payload_passthrough();
    test_crc32_known_vectors();
    test_decode_flags_full_byte();
    return tap_done();
}
