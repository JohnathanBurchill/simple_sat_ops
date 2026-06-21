/*

   Simple Satellite Operations  asm_search.h

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

#ifndef ASM_SEARCH_H
#define ASM_SEARCH_H

#include <stddef.h>
#include <stdint.h>

// AX100 attached sync marker, as the 32 transformed bits appear on the wire.
// Must match AX100_ASM_* in ax100.h -- defined here instead of including
// ax100.h so the DSP layer stays a pure DSP module.
#define ASM_BIG_ENDIAN_U32  0x930B51DEu

// Scan `bits` (one bit per byte, LSB significant) for the 32-bit `needle`,
// returning the bit offset of the LOWEST-Hamming-distance match (ties broken
// by earliest position) with distance <= max_ham, searching from min_offset.
// Returns (size_t)-1 if nothing is within max_ham. *out_ham (if non-NULL)
// receives the matched Hamming distance, or 33 if there was no match.
//
// The "best" rather than "first" match matters at sync_max_ham >= 4: the bit
// stream is then sprinkled with random Hamming-4-or-5 noise hits, and taking
// the first one would bury the real Hamming-1 ASM behind a wall of noise.
size_t asm_find_best(const uint8_t *bits, size_t n_bits,
                     uint32_t needle, int max_ham,
                     size_t min_offset, int *out_ham);

#endif
