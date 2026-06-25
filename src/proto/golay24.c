/*

    Simple Satellite Operations  golay24.c

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

#include "golay24.h"

#include <stddef.h>

// Parity-check / generator rows, matching pycsplink.Golay24.H verbatim.
// Each row is a 24-bit integer; the low 12 bits select which data bits
// participate in that parity bit.
static const uint32_t GOLAY24_GEN[12] = {
    0x8008EDu, 0x4001DBu, 0x2003B5u, 0x100769u,
    0x080ED1u, 0x040DA3u, 0x020B47u, 0x01068Fu,
    0x008D1Du, 0x004A3Bu, 0x002477u, 0x001FFEu,
};

static inline int golay24_parity(uint32_t x)
{
    return __builtin_popcount(x) & 1;
}

uint32_t golay24_encode(uint16_t data12)
{
    uint32_t r = (uint32_t)(data12 & 0x0FFFu);
    uint32_t s = 0;
    for (int i = 0; i < 12; ++i) {
        s <<= 1;
        s |= (uint32_t)golay24_parity(GOLAY24_GEN[i] & r);
    }
    return ((s & 0x0FFFu) << 12) | r;
}

int golay24_decode(uint32_t word24, uint16_t *out_data12,
                   int *out_errors_corrected)
{
    if (out_data12 == NULL) return -1;
    uint32_t rx = word24 & 0x00FFFFFFu;
    int best_dist = 25;
    uint16_t best_data = 0;
    for (uint32_t d = 0; d < 4096; ++d) {
        uint32_t code = golay24_encode((uint16_t)d);
        int dist = __builtin_popcount(code ^ rx);
        if (dist < best_dist) {
            best_dist = dist;
            best_data = (uint16_t)d;
            if (best_dist == 0) break;
        }
    }
    if (best_dist > 3) {
        if (out_errors_corrected) *out_errors_corrected = best_dist;
        return -1;
    }
    *out_data12 = best_data;
    if (out_errors_corrected) *out_errors_corrected = best_dist;
    return 0;
}
