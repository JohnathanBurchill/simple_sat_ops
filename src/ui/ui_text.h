/*

    Simple Satellite Operations  ui/ui_text.h

    Small text helpers shared by the on-screen panels and modals (and
    unit-tested in isolation). Pure C, no ncurses.

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

#ifndef SSO_UI_TEXT_H
#define SSO_UI_TEXT_H

#include <stddef.h>

// Copy src into dst (cap bytes) limited to a display width of max_w
// columns. When src is wider than max_w the last three kept columns are
// overwritten with "..." so the clip is visible on screen instead of a
// command silently running off the panel edge. If max_w < 3 there is no
// room for the marker and the text is simply cut. max_w is clamped to the
// buffer (cap - 1) so dst never overruns; dst is always NUL-terminated
// when cap > 0. Returns dst so it can be used inline as a printf argument.
const char *clip_ellipsis(char *dst, size_t cap, const char *src, int max_w);

#endif
