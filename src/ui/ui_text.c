/*

    Simple Satellite Operations  ui/ui_text.c

    Small text helpers shared by the on-screen panels and modals. Pure C,
    no ncurses, so they can be unit-tested without a terminal.

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

#include "ui_text.h"

#include <string.h>

const char *clip_ellipsis(char *dst, size_t cap, const char *src, int max_w)
{
    if (cap == 0) return dst;
    if (max_w < 0) max_w = 0;
    // Never write past the buffer, whatever the caller asked for.
    if ((size_t) max_w > cap - 1) max_w = (int) (cap - 1);
    size_t len = strlen(src);
    if ((int) len <= max_w) {
        memcpy(dst, src, len);
        dst[len] = '\0';
        return dst;
    }
    // Too wide: keep max_w columns and mark the cut with a trailing "..."
    // (only if there is room for it -- a sub-3-column field just gets cut).
    memcpy(dst, src, (size_t) max_w);
    dst[max_w] = '\0';
    if (max_w >= 3) {
        dst[max_w - 1] = '.';
        dst[max_w - 2] = '.';
        dst[max_w - 3] = '.';
    }
    return dst;
}
