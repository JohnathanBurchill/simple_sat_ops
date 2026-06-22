/*

    Simple Satellite Operations  ipc/sso_base64.h

    Standard base64 (RFC 4648, '+' '/' alphabet, '=' padding). Used to
    carry binary Ogg/Vorbis audio inside the newline-JSON wire — the
    operator base64-encodes encoded audio bytes into an SSO_EVT_AUDIO
    `data` field; the viewer decodes it. Decode is provided too so the
    codec selftest can round-trip and to keep the two halves in one place.

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

#ifndef SSO_BASE64_H
#define SSO_BASE64_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Encoded length (excluding the null terminator) for `n` input bytes.
size_t sso_base64_encoded_len(size_t n);

// Encode `n` bytes of `in` into `out` as a null-terminated base64 string.
// `out_cap` must be at least sso_base64_encoded_len(n) + 1. Returns the
// encoded length (excluding null) on success, or -1 if out_cap is too small.
long sso_base64_encode(const uint8_t *in, size_t n, char *out, size_t out_cap);

// Decode the null-terminated base64 string `in` into `out`. Returns the
// number of bytes written, or -1 on malformed input (bad length, illegal
// character, misplaced padding) or if out_cap is too small.
long sso_base64_decode(const char *in, uint8_t *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif // SSO_BASE64_H
