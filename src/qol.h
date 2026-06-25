/*

   Simple Satellite Operations  qol.h

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

#ifndef QOL_H
#define QOL_H

#ifdef QOL
// Evaluate `array` once into a typed local so callers passing a cast
// expression like `(uint8_t *)reply` parse correctly. Without this the
// macro body would expand to `(uint8_t *)reply[i]` and trigger
// -Wint-to-pointer-cast.
#define printcmd(msg, array, len) do { \
    const unsigned char *_qol_arr = (const unsigned char *)(array); \
    int _qol_len = (int)(len); \
    fprintf(stderr, "%s", (msg)); \
    for (int _qol_i = 0; _qol_i < _qol_len; ++_qol_i) { \
        fprintf(stderr, " %02X", _qol_arr[_qol_i]); \
    } \
    fprintf(stderr, "\n"); \
} while (0)
#else
#define printcmd(msg, array, len) do {} while (0)
#endif

#endif // QOL_H
