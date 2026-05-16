/*

    Simple Satellite Operations  rs.c

    Direct C port of reed_solomon_ccsds v1.0.3 (Python) encode_block /
    decode_block, which are themselves a translation of Phil Karn's
    libfec encoder/decoder for the CCSDS conventional-basis
    RS(255, 223) code. The tables (ALPHA_TO, INDEX_OF, GEN_POLY) are
    copied verbatim from reed_solomon_ccsds/reed_solomon.py so byte
    equivalence with the Python reference is guaranteed.

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

#include <string.h>

#define NN    RS_N       // 255
#define KK    RS_K       // 223
#define NROOTS RS_NROOTS // 32
#define FCR   112
#define PRIM  11
#define IPRIM 116
#define A0    NN         // sentinel for log(0)

// α^i for i = 0..254 in GF(256) with p(x) = x^8+x^7+x^2+x+1.
// ALPHA_TO[255] = 0 (convention: A0 maps to zero).
static const uint8_t ALPHA_TO[256] = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x87, 0x89, 0x95, 0xad, 0xdd, 0x3d, 0x7a, 0xf4,
    0x6f, 0xde, 0x3b, 0x76, 0xec, 0x5f, 0xbe, 0xfb, 0x71, 0xe2, 0x43, 0x86, 0x8b, 0x91, 0xa5, 0xcd,
    0x1d, 0x3a, 0x74, 0xe8, 0x57, 0xae, 0xdb, 0x31, 0x62, 0xc4, 0x0f, 0x1e, 0x3c, 0x78, 0xf0, 0x67,
    0xce, 0x1b, 0x36, 0x6c, 0xd8, 0x37, 0x6e, 0xdc, 0x3f, 0x7e, 0xfc, 0x7f, 0xfe, 0x7b, 0xf6, 0x6b,
    0xd6, 0x2b, 0x56, 0xac, 0xdf, 0x39, 0x72, 0xe4, 0x4f, 0x9e, 0xbb, 0xf1, 0x65, 0xca, 0x13, 0x26,
    0x4c, 0x98, 0xb7, 0xe9, 0x55, 0xaa, 0xd3, 0x21, 0x42, 0x84, 0x8f, 0x99, 0xb5, 0xed, 0x5d, 0xba,
    0xf3, 0x61, 0xc2, 0x03, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x07, 0x0e, 0x1c, 0x38, 0x70, 0xe0,
    0x47, 0x8e, 0x9b, 0xb1, 0xe5, 0x4d, 0x9a, 0xb3, 0xe1, 0x45, 0x8a, 0x93, 0xa1, 0xc5, 0x0d, 0x1a,
    0x34, 0x68, 0xd0, 0x27, 0x4e, 0x9c, 0xbf, 0xf9, 0x75, 0xea, 0x53, 0xa6, 0xcb, 0x11, 0x22, 0x44,
    0x88, 0x97, 0xa9, 0xd5, 0x2d, 0x5a, 0xb4, 0xef, 0x59, 0xb2, 0xe3, 0x41, 0x82, 0x83, 0x81, 0x85,
    0x8d, 0x9d, 0xbd, 0xfd, 0x7d, 0xfa, 0x73, 0xe6, 0x4b, 0x96, 0xab, 0xd1, 0x25, 0x4a, 0x94, 0xaf,
    0xd9, 0x35, 0x6a, 0xd4, 0x2f, 0x5e, 0xbc, 0xff, 0x79, 0xf2, 0x63, 0xc6, 0x0b, 0x16, 0x2c, 0x58,
    0xb0, 0xe7, 0x49, 0x92, 0xa3, 0xc1, 0x05, 0x0a, 0x14, 0x28, 0x50, 0xa0, 0xc7, 0x09, 0x12, 0x24,
    0x48, 0x90, 0xa7, 0xc9, 0x15, 0x2a, 0x54, 0xa8, 0xd7, 0x29, 0x52, 0xa4, 0xcf, 0x19, 0x32, 0x64,
    0xc8, 0x17, 0x2e, 0x5c, 0xb8, 0xf7, 0x69, 0xd2, 0x23, 0x46, 0x8c, 0x9f, 0xb9, 0xf5, 0x6d, 0xda,
    0x33, 0x66, 0xcc, 0x1f, 0x3e, 0x7c, 0xf8, 0x77, 0xee, 0x5b, 0xb6, 0xeb, 0x51, 0xa2, 0xc3, 0x00,
};

// log_α(x) for x = 0..255. INDEX_OF[0] = A0 = 255 (log of zero sentinel).
static const uint8_t INDEX_OF[256] = {
    0xff, 0x00, 0x01, 0x63, 0x02, 0xc6, 0x64, 0x6a, 0x03, 0xcd, 0xc7, 0xbc, 0x65, 0x7e, 0x6b, 0x2a,
    0x04, 0x8d, 0xce, 0x4e, 0xc8, 0xd4, 0xbd, 0xe1, 0x66, 0xdd, 0x7f, 0x31, 0x6c, 0x20, 0x2b, 0xf3,
    0x05, 0x57, 0x8e, 0xe8, 0xcf, 0xac, 0x4f, 0x83, 0xc9, 0xd9, 0xd5, 0x41, 0xbe, 0x94, 0xe2, 0xb4,
    0x67, 0x27, 0xde, 0xf0, 0x80, 0xb1, 0x32, 0x35, 0x6d, 0x45, 0x21, 0x12, 0x2c, 0x0d, 0xf4, 0x38,
    0x06, 0x9b, 0x58, 0x1a, 0x8f, 0x79, 0xe9, 0x70, 0xd0, 0xc2, 0xad, 0xa8, 0x50, 0x75, 0x84, 0x48,
    0xca, 0xfc, 0xda, 0x8a, 0xd6, 0x54, 0x42, 0x24, 0xbf, 0x98, 0x95, 0xf9, 0xe3, 0x5e, 0xb5, 0x15,
    0x68, 0x61, 0x28, 0xba, 0xdf, 0x4c, 0xf1, 0x2f, 0x81, 0xe6, 0xb2, 0x3f, 0x33, 0xee, 0x36, 0x10,
    0x6e, 0x18, 0x46, 0xa6, 0x22, 0x88, 0x13, 0xf7, 0x2d, 0xb8, 0x0e, 0x3d, 0xf5, 0xa4, 0x39, 0x3b,
    0x07, 0x9e, 0x9c, 0x9d, 0x59, 0x9f, 0x1b, 0x08, 0x90, 0x09, 0x7a, 0x1c, 0xea, 0xa0, 0x71, 0x5a,
    0xd1, 0x1d, 0xc3, 0x7b, 0xae, 0x0a, 0xa9, 0x91, 0x51, 0x5b, 0x76, 0x72, 0x85, 0xa1, 0x49, 0xeb,
    0xcb, 0x7c, 0xfd, 0xc4, 0xdb, 0x1e, 0x8b, 0xd2, 0xd7, 0x92, 0x55, 0xaa, 0x43, 0x0b, 0x25, 0xaf,
    0xc0, 0x73, 0x99, 0x77, 0x96, 0x5c, 0xfa, 0x52, 0xe4, 0xec, 0x5f, 0x4a, 0xb6, 0xa2, 0x16, 0x86,
    0x69, 0xc5, 0x62, 0xfe, 0x29, 0x7d, 0xbb, 0xcc, 0xe0, 0xd3, 0x4d, 0x8c, 0xf2, 0x1f, 0x30, 0xdc,
    0x82, 0xab, 0xe7, 0x56, 0xb3, 0x93, 0x40, 0xd8, 0x34, 0xb0, 0xef, 0x26, 0x37, 0x0c, 0x11, 0x44,
    0x6f, 0x78, 0x19, 0x9a, 0x47, 0x74, 0xa7, 0xc1, 0x23, 0x53, 0x89, 0xfb, 0x14, 0x5d, 0xf8, 0x97,
    0x2e, 0x4b, 0xb9, 0x60, 0x0f, 0xed, 0x3e, 0xe5, 0xf6, 0x87, 0xa5, 0x17, 0x3a, 0xa3, 0x3c, 0xb7,
};

// Generator polynomial coefficients in α-index (log) form. 33 entries.
// Entries 0 and 32 are 0, meaning α^0 = 1 at those positions (monic).
static const uint8_t GEN_POLY[NROOTS + 1] = {
    0x00, 0xf9, 0x3b, 0x42, 0x04, 0x2b, 0x7e, 0xfb, 0x61, 0x1e, 0x03, 0xd5, 0x32, 0x42, 0xaa, 0x05,
    0x18, 0x05, 0xaa, 0x42, 0x32, 0xd5, 0x03, 0x1e, 0x61, 0xfb, 0x7e, 0x2b, 0x04, 0x42, 0x3b, 0xf9,
    0x00,
};

static inline int modnn(int x)
{
    x %= NN;
    if (x < 0) x += NN;
    return x;
}

void rs_encode(const uint8_t data_in[KK], uint8_t codeword_out[NN])
{
    if (data_in != codeword_out) {
        memcpy(codeword_out, data_in, KK);
    }
    uint8_t *parity = codeword_out + KK;
    memset(parity, 0, NROOTS);

    for (int i = 0; i < KK; ++i) {
        uint8_t feedback = INDEX_OF[codeword_out[i] ^ parity[0]];
        if (feedback != A0) {
            for (int j = 1; j < NROOTS; ++j) {
                parity[j] ^= ALPHA_TO[modnn((int)feedback + (int)GEN_POLY[NROOTS - j])];
            }
        }
        memmove(parity, parity + 1, NROOTS - 1);
        if (feedback != A0) {
            parity[NROOTS - 1] = ALPHA_TO[modnn((int)feedback + (int)GEN_POLY[0])];
        } else {
            parity[NROOTS - 1] = 0;
        }
    }
}

int rs_decode(uint8_t block[NN], int *out_locs)
{
    int syndromes[NROOTS];
    for (int i = 0; i < NROOTS; ++i) syndromes[i] = block[0];

    for (int j = 1; j < NN; ++j) {
        for (int i = 0; i < NROOTS; ++i) {
            if (syndromes[i] == 0) {
                syndromes[i] = block[j];
            } else {
                syndromes[i] = block[j]
                    ^ ALPHA_TO[modnn((int)INDEX_OF[syndromes[i]] + (FCR + i) * PRIM)];
            }
        }
    }

    int syn_error = 0;
    for (int i = 0; i < NROOTS; ++i) {
        syn_error |= syndromes[i];
        syndromes[i] = INDEX_OF[syndromes[i]];
    }

    if (syn_error == 0) {
        return 0;
    }

    // Berlekamp-Massey to find error locator polynomial Λ(x).
    int lambda[NROOTS + 1] = {0};
    lambda[0] = 1;
    int t[NROOTS + 1];
    int b[NROOTS + 1];
    for (int i = 0; i <= NROOTS; ++i) b[i] = INDEX_OF[lambda[i]];

    int r = 0;
    int el = 0;
    while (r + 1 <= NROOTS) {
        r++;
        int discrepancy = 0;
        for (int i = 0; i < r; ++i) {
            if (lambda[i] != 0 && syndromes[r - i - 1] != A0) {
                discrepancy ^= ALPHA_TO[modnn(
                    (int)INDEX_OF[lambda[i]] + syndromes[r - i - 1])];
            }
        }
        discrepancy = INDEX_OF[discrepancy];

        if (discrepancy == A0) {
            for (int i = NROOTS; i > 0; --i) b[i] = b[i - 1];
            b[0] = A0;
        } else {
            t[0] = lambda[0];
            for (int i = 0; i < NROOTS; ++i) {
                if (b[i] != A0) {
                    t[i + 1] = lambda[i + 1] ^ ALPHA_TO[modnn(discrepancy + b[i])];
                } else {
                    t[i + 1] = lambda[i + 1];
                }
            }
            if (2 * el <= r - 1) {
                el = r - el;
                for (int i = 0; i < NROOTS; ++i) {
                    b[i] = (lambda[i] == 0)
                        ? A0
                        : modnn((int)INDEX_OF[lambda[i]] - discrepancy + NN);
                }
            } else {
                for (int i = NROOTS; i > 0; --i) b[i] = b[i - 1];
                b[0] = A0;
            }
            memcpy(lambda, t, sizeof(lambda));
        }
    }

    // Convert Λ to index form; find its degree.
    int lambda_idx[NROOTS + 1];
    int deg_lambda = 0;
    for (int i = 0; i <= NROOTS; ++i) {
        lambda_idx[i] = INDEX_OF[lambda[i]];
        if (lambda_idx[i] != A0) deg_lambda = i;
    }

    // Chien search for roots of Λ(x).
    int reg[NROOTS + 1];
    for (int i = 1; i <= NROOTS; ++i) reg[i] = lambda_idx[i];
    int root[NROOTS];
    int loc[NROOTS];
    int count = 0;
    int ii = 1;
    int k = IPRIM - 1;
    while (ii <= NN) {
        int q = 1;
        for (int j = deg_lambda; j > 0; --j) {
            if (reg[j] != A0) {
                reg[j] = modnn(reg[j] + j);
                q ^= ALPHA_TO[reg[j]];
            }
        }
        if (q != 0) {
            ii++;
            k = modnn(k + IPRIM);
            continue;
        }
        root[count] = ii;
        loc[count] = k;
        count++;
        if (count == deg_lambda) break;
        ii++;
        k = modnn(k + IPRIM);
    }

    if (deg_lambda != count) {
        return -1;  // number of roots unequal to degree — uncorrectable
    }

    // Compute omega(x) = S(x)·Λ(x) mod x^NROOTS, in index form.
    int omega[NROOTS + 1];
    int deg_omega = 0;
    for (int i = 0; i < NROOTS; ++i) {
        int tmp = 0;
        int jmax = (deg_lambda < i) ? deg_lambda : i;
        for (int j = jmax; j >= 0; --j) {
            if (syndromes[i - j] != A0 && lambda_idx[j] != A0) {
                tmp ^= ALPHA_TO[modnn(syndromes[i - j] + lambda_idx[j])];
            }
        }
        if (tmp != 0) deg_omega = i;
        omega[i] = INDEX_OF[tmp];
    }
    omega[NROOTS] = A0;

    // Forney: compute error magnitudes and apply.
    for (int j = count - 1; j >= 0; --j) {
        int num1 = 0;
        for (int i = deg_omega; i >= 0; --i) {
            if (omega[i] != A0) {
                num1 ^= ALPHA_TO[modnn((int)omega[i] + i * root[j])];
            }
        }
        int num2 = ALPHA_TO[modnn(root[j] * (FCR - 1) + NN)];
        int den = 0;

        int start = (deg_lambda < NROOTS - 1) ? deg_lambda : (NROOTS - 1);
        start &= ~1;
        for (int i = start; i >= 0; i -= 2) {
            if (lambda_idx[i + 1] != A0) {
                den ^= ALPHA_TO[modnn((int)lambda_idx[i + 1] + i * root[j])];
            }
        }
        if (den == 0) return -1;
        if (num1 != 0) {
            block[loc[j]] ^= ALPHA_TO[modnn(
                (int)INDEX_OF[num1] + (int)INDEX_OF[num2] + NN - (int)INDEX_OF[den])];
        }
    }
    if (out_locs != NULL) {
        for (int j = 0; j < count; ++j) out_locs[j] = loc[j];
    }
    return count;
}

ssize_t rs_pycsp_encode(const uint8_t *in, size_t in_len,
                        uint8_t *out, size_t out_cap)
{
    if (in == NULL || out == NULL) return -1;
    if (in_len > KK) return -1;
    if (out_cap < in_len + NROOTS) return -1;

    uint8_t buf[NN];
    size_t padding = KK - in_len;
    memset(buf, 0, padding);
    memcpy(buf + padding, in, in_len);

    rs_encode(buf, buf);

    // Output = buf[padding..], which is in_len data bytes + 32 parity bytes.
    memcpy(out, buf + padding, in_len + NROOTS);
    return (ssize_t)(in_len + NROOTS);
}

ssize_t rs_pycsp_decode(const uint8_t *in, size_t in_len,
                        uint8_t *out, size_t out_cap,
                        int *out_errors,
                        int *out_locs)
{
    if (in == NULL || out == NULL) return -1;
    if (in_len <= NROOTS || in_len > NN) return -1;
    size_t data_len = in_len - NROOTS;
    if (out_cap < data_len) return -1;

    uint8_t buf[NN];
    size_t padding = NN - in_len;
    memset(buf, 0, padding);
    memcpy(buf + padding, in, in_len);

    int errs = rs_decode(buf, out_locs);
    if (errs < 0) return -1;
    if (out_errors) *out_errors = errs;

    // Translate codeword indices (0..NN-1) to on-wire byte offsets
    // (0..in_len-1). Anything left negative landed in the synthetic
    // zero-pad and signals an RS-solver false correction.
    if (out_locs != NULL) {
        for (int i = 0; i < errs; ++i) out_locs[i] -= (int)padding;
    }

    memcpy(out, buf + padding, data_len);
    return (ssize_t)data_len;
}
