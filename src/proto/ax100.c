/*

    Simple Satellite Operations  ax100.c

    Copyright (C) 2025, 2026  Johnathan K Burchill

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

#include <string.h>

#include <openssl/evp.h>

// CCSDS RX scrambler 256-byte precomputed XOR table.
// Polynomial h(x) = x^8 + x^7 + x^5 + x^3 + 1, fbmask = 0xA9, initreg = 0xFF.
// Table is verbatim from pycsplink.CCSDSRxScrambler._TABLE (255 bytes;
// the 256th byte wraps around to byte 0 in the generator but the Python
// table only defines 255 entries — matched exactly here for bit-compat).
static const uint8_t CCSDS_SCRAMBLER_TABLE[255] = {
    0xFF, 0x48, 0x0E, 0xC0, 0x9A, 0x0D, 0x70, 0xBC,
    0x8E, 0x2C, 0x93, 0xAD, 0xA7, 0xB7, 0x46, 0xCE,
    0x5A, 0x97, 0x7D, 0xCC, 0x32, 0xA2, 0xBF, 0x3E,
    0x0A, 0x10, 0xF1, 0x88, 0x94, 0xCD, 0xEA, 0xB1,
    0xFE, 0x90, 0x1D, 0x81, 0x34, 0x1A, 0xE1, 0x79,
    0x1C, 0x59, 0x27, 0x5B, 0x4F, 0x6E, 0x8D, 0x9C,
    0xB5, 0x2E, 0xFB, 0x98, 0x65, 0x45, 0x7E, 0x7C,
    0x14, 0x21, 0xE3, 0x11, 0x29, 0x9B, 0xD5, 0x63,
    0xFD, 0x20, 0x3B, 0x02, 0x68, 0x35, 0xC2, 0xF2,
    0x38, 0xB2, 0x4E, 0xB6, 0x9E, 0xDD, 0x1B, 0x39,
    0x6A, 0x5D, 0xF7, 0x30, 0xCA, 0x8A, 0xFC, 0xF8,
    0x28, 0x43, 0xC6, 0x22, 0x53, 0x37, 0xAA, 0xC7,
    0xFA, 0x40, 0x76, 0x04, 0xD0, 0x6B, 0x85, 0xE4,
    0x71, 0x64, 0x9D, 0x6D, 0x3D, 0xBA, 0x36, 0x72,
    0xD4, 0xBB, 0xEE, 0x61, 0x95, 0x15, 0xF9, 0xF0,
    0x50, 0x87, 0x8C, 0x44, 0xA6, 0x6F, 0x55, 0x8F,
    0xF4, 0x80, 0xEC, 0x09, 0xA0, 0xD7, 0x0B, 0xC8,
    0xE2, 0xC9, 0x3A, 0xDA, 0x7B, 0x74, 0x6C, 0xE5,
    0xA9, 0x77, 0xDC, 0xC3, 0x2A, 0x2B, 0xF3, 0xE0,
    0xA1, 0x0F, 0x18, 0x89, 0x4C, 0xDE, 0xAB, 0x1F,
    0xE9, 0x01, 0xD8, 0x13, 0x41, 0xAE, 0x17, 0x91,
    0xC5, 0x92, 0x75, 0xB4, 0xF6, 0xE8, 0xD9, 0xCB,
    0x52, 0xEF, 0xB9, 0x86, 0x54, 0x57, 0xE7, 0xC1,
    0x42, 0x1E, 0x31, 0x12, 0x99, 0xBD, 0x56, 0x3F,
    0xD2, 0x03, 0xB0, 0x26, 0x83, 0x5C, 0x2F, 0x23,
    0x8B, 0x24, 0xEB, 0x69, 0xED, 0xD1, 0xB3, 0x96,
    0xA5, 0xDF, 0x73, 0x0C, 0xA8, 0xAF, 0xCF, 0x82,
    0x84, 0x3C, 0x62, 0x25, 0x33, 0x7A, 0xAC, 0x7F,
    0xA4, 0x07, 0x60, 0x4D, 0x06, 0xB8, 0x5E, 0x47,
    0x16, 0x49, 0xD6, 0xD3, 0xDB, 0xA3, 0x67, 0x2D,
    0x4B, 0xBE, 0xE6, 0x19, 0x51, 0x5F, 0x9F, 0x05,
    0x08, 0x78, 0xC4, 0x4A, 0x66, 0xF5, 0x58,
};

void ax100_opts_defaults(ax100_opts_t *opts)
{
    if (opts == NULL) return;
    opts->hmac_key = NULL;
    opts->hmac_key_len = 0;
    opts->randomize = 1;
    opts->reed_solomon = 0;
    opts->len_field = 1;
    opts->syncword = 1;
    opts->prefill = 32;
    opts->tailfill = 1;
}

// SHA-1 of `data` into out[0..19]. Returns 0 on success.
static int sha1_once(const uint8_t *data, size_t data_len, uint8_t out[20])
{
    unsigned int md_len = 0;
    int ok = EVP_Digest(data, data_len, out, &md_len, EVP_sha1(), NULL);
    return (ok == 1 && md_len == 20) ? 0 : -1;
}

// Concatenated SHA-1 of two byte ranges into out[0..19]. Returns 0 on success.
static int sha1_two(const uint8_t *a, size_t a_len,
                    const uint8_t *b, size_t b_len,
                    uint8_t out[20])
{
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    if (ctx == NULL) return -1;
    int rc = -1;
    unsigned int md_len = 0;
    if (EVP_DigestInit_ex(ctx, EVP_sha1(), NULL) == 1 &&
        EVP_DigestUpdate(ctx, a, a_len) == 1 &&
        EVP_DigestUpdate(ctx, b, b_len) == 1 &&
        EVP_DigestFinal_ex(ctx, out, &md_len) == 1 &&
        md_len == 20) {
        rc = 0;
    }
    EVP_MD_CTX_free(ctx);
    return rc;
}

int ax100_hmac(const uint8_t *key, size_t key_len,
               const uint8_t *data, size_t data_len,
               uint8_t out_trailer[4])
{
    if (out_trailer == NULL || (key_len > 0 && key == NULL) ||
        (data_len > 0 && data == NULL)) {
        return -1;
    }

    // Key derivation: rkey = SHA1(user_key)[:16], padded to 64 bytes with 0x00.
    // Matches pycsp.HMACEngine exactly.
    uint8_t rkey[64] = {0};
    uint8_t sha1_of_key[20] = {0};
    if (sha1_once(key, key_len, sha1_of_key) != 0) return -1;
    memcpy(rkey, sha1_of_key, 16);
    // Upper 48 bytes remain zero-filled.

    uint8_t ipad[64];
    uint8_t opad[64];
    for (int i = 0; i < 64; ++i) {
        ipad[i] = rkey[i] ^ 0x36;
        opad[i] = rkey[i] ^ 0x5C;
    }

    uint8_t inner[20];
    if (sha1_two(ipad, sizeof(ipad), data, data_len, inner) != 0) return -1;

    uint8_t outer[20];
    if (sha1_two(opad, sizeof(opad), inner, sizeof(inner), outer) != 0) return -1;

    memcpy(out_trailer, outer, 4);
    return 0;
}

ssize_t ax100_frame(const uint8_t *packet, size_t packet_len,
                    const ax100_opts_t *opts,
                    uint8_t *out_buf, size_t out_buf_size)
{
    if (opts == NULL || out_buf == NULL ||
        (packet_len > 0 && packet == NULL)) {
        return -1;
    }
    if (opts->prefill < 0 || opts->tailfill < 0) {
        return -1;
    }

    // Assemble the "x" buffer (matches pycsplink.AX100.encode's x variable)
    // = packet || optional HMAC trailer || optional 32-byte RS parity,
    // optionally scrambled afterward.
    // 12-bit Golay length field caps inner_len at 4095; RS-shortened code
    // caps inner_len at 255; we respect both.
    uint8_t inner[4100];
    size_t inner_len = 0;

    size_t needed = packet_len + (opts->hmac_key != NULL ? 4 : 0);
    if (opts->reed_solomon) {
        // pycsplink.AX100.encode truncates pre-RS input to 223 silently.
        // We refuse instead — C callers should see the error rather than
        // unknowingly lose bytes.
        if (needed > RS_K) return -1;
        // Final on-wire size will be `needed + 32`; RS-shortened codes
        // never exceed 255 and the 12-bit Golay field covers that.
        if (needed + RS_NROOTS > sizeof(inner)) return -1;
    } else {
        if (needed > sizeof(inner) || needed > 0x0FFFu) {
            return -1;
        }
    }
    // packet may legitimately be NULL when packet_len == 0 (an empty CSP
    // packet); memcpy(dst, NULL, 0) is undefined behaviour, so guard it.
    if (packet_len > 0) {
        memcpy(inner, packet, packet_len);
    }
    inner_len = packet_len;

    if (opts->hmac_key != NULL) {
        uint8_t mac[4] = {0};
        if (ax100_hmac(opts->hmac_key, opts->hmac_key_len,
                       inner, inner_len, mac) != 0) {
            return -1;
        }
        memcpy(inner + inner_len, mac, 4);
        inner_len += 4;
    }

    // Reed-Solomon encode (post-HMAC/CRC, pre-scrambler). pycsp semantics:
    // left-zero-pad to 223, rs_encode to 255, strip the leading padding.
    // Result is (inner_len + 32) bytes — the 32-byte parity is always
    // appended regardless of input length.
    if (opts->reed_solomon) {
        uint8_t rs_out[RS_N];
        ssize_t rs_len = rs_pycsp_encode(inner, inner_len, rs_out, sizeof rs_out);
        if (rs_len < 0) return -1;
        memcpy(inner, rs_out, (size_t)rs_len);
        inner_len = (size_t)rs_len;
    }

    // CCSDS scrambler: XOR each byte with the table, wrapping the table.
    // The table is 255 bytes, so this wraps with period 255, NOT the 256 of a
    // textbook CCSDS randomizer. That is correct only because the peer
    // (pycsplink) uses the identical 255-entry table; the two would diverge
    // once a frame exceeded 255 payload bytes — which can't happen here (RS
    // caps inner_len at 255, and the no-RS path is bounded by the Golay field).
    if (opts->randomize) {
        for (size_t i = 0; i < inner_len; ++i) {
            inner[i] ^= CCSDS_SCRAMBLER_TABLE[i % sizeof(CCSDS_SCRAMBLER_TABLE)];
        }
    }

    // Compute total wrapped size and validate against out_buf.
    size_t wrap_prefix = 0;
    if (opts->syncword) wrap_prefix += 4;
    if (opts->len_field) wrap_prefix += 3;
    size_t total = (size_t)opts->prefill + wrap_prefix + inner_len + (size_t)opts->tailfill;
    if (total > out_buf_size) {
        return -1;
    }

    uint8_t *p = out_buf;

    // Preamble (0xAA x prefill)
    memset(p, 0xAA, (size_t)opts->prefill);
    p += opts->prefill;

    // Sync word (ASM)
    if (opts->syncword) {
        p[0] = AX100_ASM_0;
        p[1] = AX100_ASM_1;
        p[2] = AX100_ASM_2;
        p[3] = AX100_ASM_3;
        p += 4;
    }

    // 3-byte Golay24-encoded length field (big-endian).
    if (opts->len_field) {
        if (inner_len > 0x0FFFu) {
            return -1;  // Golay24 data field is 12 bits wide
        }
        uint32_t g = golay24_encode((uint16_t)inner_len);
        p[0] = (uint8_t)((g >> 16) & 0xFF);
        p[1] = (uint8_t)((g >>  8) & 0xFF);
        p[2] = (uint8_t)( g        & 0xFF);
        p += 3;
    }

    // Scrambled payload.
    memcpy(p, inner, inner_len);
    p += inner_len;

    // Postamble (0xAA x tailfill)
    memset(p, 0xAA, (size_t)opts->tailfill);
    p += opts->tailfill;

    return (ssize_t)(p - out_buf);
}

// Try one length hypothesis. `scrambled` points at the first byte after
// the Golay header. `on_wire_len` is how many scrambled bytes we'll eat.
// Writes recovered CSP packet to out_packet on full success (RS + HMAC).
// Returns:
//   >= 0: packet length, hmac_ok accurately set (if HMAC requested).
//   -1  : reject (descramble ok, but RS uncorrectable, or HMAC mismatch,
//         or buffer overflow, or insufficient bytes).
static ssize_t ax100_try_length(const uint8_t *scrambled, size_t on_wire_len,
                                const ax100_opts_t *opts,
                                uint8_t *out_packet, size_t out_packet_cap,
                                int *out_hmac_ok,
                                int *out_rs_errors,
                                int *out_rs_locs)
{
    // Scratch large enough for either RS-off (up to 4095 post-scramble,
    // which is impossible on-wire but tolerate here) or RS-on (<=255).
    uint8_t inner[4100];
    if (on_wire_len > sizeof(inner)) return -1;

    memcpy(inner, scrambled, on_wire_len);
    if (opts->randomize) {
        for (size_t i = 0; i < on_wire_len; ++i) {
            inner[i] ^= CCSDS_SCRAMBLER_TABLE[i % sizeof(CCSDS_SCRAMBLER_TABLE)];
        }
    }

    size_t data_len = on_wire_len;
    if (opts->reed_solomon) {
        if (on_wire_len <= RS_NROOTS || on_wire_len > RS_N) return -1;
        uint8_t decoded[RS_K];
        int rs_errs = 0;
        ssize_t rc = rs_pycsp_decode(inner, on_wire_len,
                                     decoded, sizeof decoded, &rs_errs,
                                     out_rs_locs);
        if (rc < 0) {
            if (out_rs_errors) *out_rs_errors = -1;
            return -1;
        }
        if (out_rs_errors) *out_rs_errors = rs_errs;
        memcpy(inner, decoded, (size_t)rc);
        data_len = (size_t)rc;
    }

    size_t packet_len = data_len;
    if (opts->hmac_key != NULL) {
        if (data_len < 4) return -1;
        uint8_t expected[4] = {0};
        if (ax100_hmac(opts->hmac_key, opts->hmac_key_len,
                       inner, data_len - 4, expected) != 0) {
            return -1;
        }
        // Not a constant-time compare. Fine here: this is a receive-side
        // integrity check on already-public downlink, not a secret-dependent
        // auth gate. If this code is ever reused to authenticate uplink, swap
        // in a constant-time comparison to avoid a timing side channel.
        int match = (memcmp(expected, inner + data_len - 4, 4) == 0);
        if (out_hmac_ok) *out_hmac_ok = match ? 1 : 0;
        if (!match) return -1;
        packet_len = data_len - 4;
    }

    if (packet_len > out_packet_cap) return -1;
    memcpy(out_packet, inner, packet_len);
    return (ssize_t)packet_len;
}

ssize_t ax100_unframe(const uint8_t *bytes, size_t n_bytes,
                      const ax100_opts_t *opts,
                      uint8_t *out_packet, size_t out_packet_cap,
                      int *out_golay_errors,
                      int *out_hmac_ok,
                      int *out_rs_errors,
                      int *out_used_golay_len,
                      int *out_rs_locs)
{
    if (bytes == NULL || opts == NULL || out_packet == NULL) return -1;
    // Clear the caller's buffer before writing this frame, so a decode
    // that ends up shorter than a previous one can never leave stale tail
    // bytes from the prior frame for a downstream reader to pick up. The
    // caller reuses one buffer across every window/attempt.
    memset(out_packet, 0, out_packet_cap);
    if (out_hmac_ok) *out_hmac_ok = -1;
    if (out_golay_errors) *out_golay_errors = 0;
    if (out_rs_errors) *out_rs_errors = -1;
    if (out_used_golay_len) *out_used_golay_len = -1;

    size_t off = 0;

    if (opts->syncword) {
        if (n_bytes < 4) return -1;
        if (bytes[0] != AX100_ASM_0 || bytes[1] != AX100_ASM_1 ||
            bytes[2] != AX100_ASM_2 || bytes[3] != AX100_ASM_3) {
            return -1;
        }
        off += 4;
    }

    // Decode the Golay length header (if present). If Golay reports an
    // uncorrectable header, we don't abort outright when RS+HMAC are on —
    // the brute-force length search below can still recover.
    size_t golay_len = 0;
    int golay_ok = 0;
    if (opts->len_field) {
        if (n_bytes - off < 3) return -1;
        uint32_t g = ((uint32_t)bytes[off    ] << 16)
                   | ((uint32_t)bytes[off + 1] <<  8)
                   |  (uint32_t)bytes[off + 2];
        off += 3;
        uint16_t decoded = 0;
        int errs = 0;
        int rc = golay24_decode(g, &decoded, &errs);
        if (out_golay_errors) *out_golay_errors = errs;
        if (rc == 0) {
            golay_len = (size_t)decoded;
            golay_ok = 1;
        }
    } else {
        // No length field — caller is saying "all remaining bytes belong
        // to the payload". Fall through with golay_ok=0 so we try
        // n_bytes-off as the length.
        golay_len = n_bytes - off;
        golay_ok = 1;
    }

    size_t avail = n_bytes - off;
    const uint8_t *scrambled = bytes + off;

    int can_validate_hmac = (opts->hmac_key != NULL);

    // First attempt: the Golay-decoded length, if sensible.
    if (golay_ok && golay_len > 0 && golay_len <= avail) {
        int hmac_tmp = -1, rs_tmp = -1;
        ssize_t r = ax100_try_length(scrambled, golay_len, opts,
                                     out_packet, out_packet_cap,
                                     &hmac_tmp, &rs_tmp,
                                     out_rs_locs);
        if (r >= 0) {
            if (out_hmac_ok) *out_hmac_ok = hmac_tmp;
            if (out_rs_errors) *out_rs_errors = rs_tmp;
            if (out_used_golay_len) *out_used_golay_len = 1;
            return r;
        }
        // Keep the HMAC / RS diagnostics from the first try in case we
        // fall out below without a brute-force retry.
        if (out_hmac_ok && hmac_tmp != -1) *out_hmac_ok = hmac_tmp;
        if (out_rs_errors && rs_tmp != -1) *out_rs_errors = rs_tmp;
    }

    // Brute-force fallback: when RS+HMAC are BOTH on, scan every plausible
    // on-wire length [RS_NROOTS+1, min(RS_N, avail)] and accept the first
    // that RS-decodes AND HMAC-validates. This rescues frames where the
    // Golay length header took >3 bit errors and misdecoded. Without HMAC
    // we have no way to distinguish a false RS match, so we skip this path.
    if (opts->reed_solomon && can_validate_hmac) {
        size_t lo = RS_NROOTS + 1;
        size_t hi = avail < RS_N ? avail : RS_N;
        for (size_t L = lo; L <= hi; ++L) {
            if (golay_ok && L == golay_len) continue;  // already tried
            int hmac_tmp = -1, rs_tmp = -1;
            ssize_t r = ax100_try_length(scrambled, L, opts,
                                         out_packet, out_packet_cap,
                                         &hmac_tmp, &rs_tmp,
                                         out_rs_locs);
            if (r >= 0) {
                if (out_hmac_ok) *out_hmac_ok = hmac_tmp;
                if (out_rs_errors) *out_rs_errors = rs_tmp;
                if (out_used_golay_len) *out_used_golay_len = 0;
                return r;
            }
        }
    }

    return -1;
}
