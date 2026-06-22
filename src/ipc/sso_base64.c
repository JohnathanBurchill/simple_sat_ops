/*

    Simple Satellite Operations  ipc/sso_base64.c  — see sso_base64.h.

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

#include "sso_base64.h"

#include <string.h>

static const char ENC[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t sso_base64_encoded_len(size_t n)
{
    return ((n + 2) / 3) * 4;
}

long sso_base64_encode(const uint8_t *in, size_t n, char *out, size_t out_cap)
{
    if (!out) return -1;
    size_t need = sso_base64_encoded_len(n);
    if (out_cap < need + 1) return -1;
    if (n > 0 && !in) return -1;

    size_t oi = 0, i = 0;
    while (i + 3 <= n) {
        uint32_t v = ((uint32_t) in[i] << 16)
                   | ((uint32_t) in[i + 1] << 8)
                   |  (uint32_t) in[i + 2];
        out[oi++] = ENC[(v >> 18) & 0x3F];
        out[oi++] = ENC[(v >> 12) & 0x3F];
        out[oi++] = ENC[(v >> 6) & 0x3F];
        out[oi++] = ENC[v & 0x3F];
        i += 3;
    }
    size_t rem = n - i;
    if (rem == 1) {
        uint32_t v = (uint32_t) in[i] << 16;
        out[oi++] = ENC[(v >> 18) & 0x3F];
        out[oi++] = ENC[(v >> 12) & 0x3F];
        out[oi++] = '=';
        out[oi++] = '=';
    } else if (rem == 2) {
        uint32_t v = ((uint32_t) in[i] << 16) | ((uint32_t) in[i + 1] << 8);
        out[oi++] = ENC[(v >> 18) & 0x3F];
        out[oi++] = ENC[(v >> 12) & 0x3F];
        out[oi++] = ENC[(v >> 6) & 0x3F];
        out[oi++] = '=';
    }
    out[oi] = '\0';
    return (long) oi;
}

static int b64val(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

long sso_base64_decode(const char *in, uint8_t *out, size_t out_cap)
{
    if (!in || !out) return -1;
    size_t len = strlen(in);
    if (len % 4 != 0) return -1;   // standard base64 is always padded to /4

    size_t oi = 0;
    for (size_t i = 0; i < len; i += 4) {
        char c0 = in[i], c1 = in[i + 1], c2 = in[i + 2], c3 = in[i + 3];
        int v0 = b64val(c0), v1 = b64val(c1);
        if (v0 < 0 || v1 < 0) return -1;

        int npad = 0;
        if (c2 == '=') npad++;
        if (c3 == '=') npad++;
        // Padding only in the final quad, and never "X=X" (a gap).
        if (npad && i + 4 != len) return -1;
        if (c2 == '=' && c3 != '=') return -1;

        int v2 = 0, v3 = 0;
        if (c2 != '=') { v2 = b64val(c2); if (v2 < 0) return -1; }
        if (c3 != '=') { v3 = b64val(c3); if (v3 < 0) return -1; }

        uint32_t v = ((uint32_t) v0 << 18) | ((uint32_t) v1 << 12)
                   | ((uint32_t) v2 << 6)  |  (uint32_t) v3;
        int outb = 3 - npad;
        if (oi + (size_t) outb > out_cap) return -1;
        out[oi++] = (uint8_t) ((v >> 16) & 0xFF);
        if (outb >= 2) out[oi++] = (uint8_t) ((v >> 8) & 0xFF);
        if (outb >= 3) out[oi++] = (uint8_t) (v & 0xFF);
    }
    return (long) oi;
}
