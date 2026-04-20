/*

    Simple Satellite Operations  golay24.c

    Copyright (C) 2025  Johnathan K Burchill

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
