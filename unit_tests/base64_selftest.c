/*

    Simple Satellite Operations  unit_tests/base64_selftest.c

    Coverage for src/ipc/sso_base64.c — the base64 codec that carries binary
    Ogg/Vorbis audio inside the newline-JSON viewer stream. The encoder side
    runs on the operator; the decoder side mirrors what the viewer (sso_viewers)
    must do, so a divergence here is a divergence from the wire contract.

    External oracle: the RFC 4648 §10 test vectors ("" "f" "fo" ... "foobar")
    are fixed published constants, NOT produced by this code, so a symmetric
    bug in the encoder + decoder (a wrong alphabet char, a transposed shift)
    that survives an encode->decode round trip still fails these. The high-byte
    vector {0xFF,0x00,0xFE} -> "/wD+" additionally pins the '+' and '/' alphabet
    and that the top bit isn't sign-extended.

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

#include "sso_base64.h"
#include "tap.h"

#include <stdint.h>
#include <string.h>

// RFC 4648 §10 test vectors — the external oracle.
static const struct {
    const char *in;
    const char *b64;
} RFC4648[] = {
    { "",       ""         },
    { "f",      "Zg=="     },
    { "fo",     "Zm8="     },
    { "foo",    "Zm9v"     },
    { "foob",   "Zm9vYg==" },
    { "fooba",  "Zm9vYmE=" },
    { "foobar", "Zm9vYmFy" },
};

int main(void)
{
    char  out[256];
    uint8_t dec[256];

    // --- RFC 4648 golden vectors, both directions ---------------------
    for (size_t i = 0; i < sizeof RFC4648 / sizeof RFC4648[0]; ++i) {
        const char *in  = RFC4648[i].in;
        const char *exp = RFC4648[i].b64;
        size_t      n   = strlen(in);

        long el = sso_base64_encode((const uint8_t *) in, n, out, sizeof out);
        tap_okf(el == (long) strlen(exp) && strcmp(out, exp) == 0,
                "encode(\"%s\") == \"%s\" (got \"%s\")", in, exp, out);
        tap_okf(sso_base64_encoded_len(n) == strlen(exp),
                "encoded_len(\"%s\") == %zu", in, strlen(exp));

        long dl = sso_base64_decode(exp, dec, sizeof dec);
        tap_okf(dl == (long) n && memcmp(dec, in, n) == 0,
                "decode(\"%s\") == \"%s\" (%ld bytes)", exp, in, dl);
    }

    // --- High-byte vector: pins '+' '/' and no sign extension ----------
    {
        const uint8_t bin[3] = { 0xFF, 0x00, 0xFE };
        long el = sso_base64_encode(bin, 3, out, sizeof out);
        tap_okf(el == 4 && strcmp(out, "/wD+") == 0,
                "encode({FF,00,FE}) == \"/wD+\" (got \"%s\")", out);
        long dl = sso_base64_decode("/wD+", dec, sizeof dec);
        tap_okf(dl == 3 && dec[0] == 0xFF && dec[1] == 0x00 && dec[2] == 0xFE,
                "decode(\"/wD+\") == {FF,00,FE}");
    }

    // --- Round-trip over every byte value, every residue length --------
    // Catches off-by-one in the 1/2/3-byte tail handling across all inputs.
    {
        uint8_t src[300];
        for (size_t i = 0; i < sizeof src; ++i) src[i] = (uint8_t) (i * 7 + 1);
        int all_ok = 1;
        for (size_t len = 0; len <= 258; ++len) {
            char  enc[400];
            uint8_t back[300];
            long el = sso_base64_encode(src, len, enc, sizeof enc);
            if (el != (long) sso_base64_encoded_len(len)) { all_ok = 0; break; }
            // No stray padding except at the very end.
            long dl = sso_base64_decode(enc, back, sizeof back);
            if (dl != (long) len || memcmp(back, src, len) != 0) { all_ok = 0; break; }
        }
        tap_ok(all_ok, "encode->decode round-trips for every length 0..258");
    }

    // --- Encoded length never injects a newline (JSON-string-safe) -----
    {
        uint8_t src[64];
        for (size_t i = 0; i < sizeof src; ++i) src[i] = (uint8_t) i;
        sso_base64_encode(src, sizeof src, out, sizeof out);
        tap_ok(strchr(out, '\n') == NULL && strchr(out, '"') == NULL
               && strchr(out, '\\') == NULL,
               "base64 output contains no newline/quote/backslash");
    }

    // --- Malformed decode is rejected ---------------------------------
    {
        tap_ok(sso_base64_decode("Zg=", dec, sizeof dec) == -1,
               "decode rejects non-multiple-of-4 length");
        tap_ok(sso_base64_decode("Zg.=", dec, sizeof dec) == -1,
               "decode rejects an illegal character");
        tap_ok(sso_base64_decode("Z=g=", dec, sizeof dec) == -1,
               "decode rejects padding in the middle of a quad");
        tap_ok(sso_base64_decode("====", dec, sizeof dec) == -1,
               "decode rejects a leading '=' (no data byte)");
        tap_ok(sso_base64_decode("Zm9vYmFy====", dec, sizeof dec) == -1,
               "decode rejects padding before the final quad");
    }

    // --- Buffer-size guards -------------------------------------------
    {
        const uint8_t bin[3] = { 1, 2, 3 };
        char small[4];   // need 5 (4 + nul)
        tap_ok(sso_base64_encode(bin, 3, small, sizeof small) == -1,
               "encode into a too-small buffer rejected");
        uint8_t dsmall[2];   // "Zm9v" decodes to 3 bytes
        tap_ok(sso_base64_decode("Zm9v", dsmall, sizeof dsmall) == -1,
               "decode into a too-small buffer rejected");
        // Exactly-fitting encode buffer is accepted.
        char exact[5];
        tap_ok(sso_base64_encode(bin, 3, exact, sizeof exact) == 4,
               "encode into an exactly-fitting buffer accepted");
    }

    return tap_done();
}
