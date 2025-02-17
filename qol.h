/*

   Simple Satellite Operations  qol.h

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

#ifndef QOL_H
#define QOL_H

#ifdef QOL
#define printcmd(msg, array, len) do { \
    fprintf(stderr, msg); \
    for (int i = 0; i < len; ++i) { \
        fprintf(stderr, " %02X", array[i]); \
    } \
    fprintf(stderr, "\n"); \
} while (0);
#else
#define printcmd(msg, array, len) do {} while (0);
#endif

#endif // QOL_H
