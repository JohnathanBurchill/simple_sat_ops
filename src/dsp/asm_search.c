/*

   Simple Satellite Operations  asm_search.c

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

#include "asm_search.h"

size_t asm_find_best(const uint8_t *bits, size_t n_bits,
                     uint32_t needle, int max_ham,
                     size_t min_offset, int *out_ham)
{
    if (out_ham) *out_ham = 33;
    if (n_bits < 32) return (size_t) -1;
    size_t start = min_offset;
    if (start > n_bits - 32) return (size_t) -1;
    uint32_t window = 0;
    for (size_t i = 0; i < 32; ++i) {
        window = (window << 1) | (bits[start + i] & 1u);
    }
    int best_ham = 33;
    size_t best_off = (size_t) -1;
    int h = (int) __builtin_popcount(window ^ needle);
    if (h <= max_ham) {
        best_ham = h;
        best_off = start;
    }
    for (size_t i = 32; i < n_bits - start; ++i) {
        window = (window << 1) | (bits[start + i] & 1u);
        h = (int) __builtin_popcount(window ^ needle);
        if (h <= max_ham && h < best_ham) {
            best_ham = h;
            best_off = start + i - 31;
            if (best_ham == 0) break;  // can't beat zero
        }
    }
    if (out_ham && best_off != (size_t) -1) *out_ham = best_ham;
    return best_off;
}
