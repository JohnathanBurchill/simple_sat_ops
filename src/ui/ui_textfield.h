/*

   Simple Satellite Operations  ui_textfield.h

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

#ifndef UI_TEXTFIELD_H
#define UI_TEXTFIELD_H

#include <stddef.h>

// Cursor-based editing primitives for a fixed-capacity, NUL-terminated
// text buffer. The caller owns the buffer and the cursor (an int index in
// [0, strlen]); these only do the buffer mechanics. The caller decides
// which field is focused, filters which characters are acceptable, and
// applies any side effects (marking a preview dirty, resetting history,
// ...) based on the returned "did the buffer change" flag.
//
// Shared by the tx-compose modal and the auto-telecommand editor, which
// both keep a per-field cursor array and were each carrying their own copy
// of this logic.

// Insert ch at the cursor, shifting the tail (and its NUL) right. The
// caller has already vetted ch. Returns 1 if inserted, 0 if the buffer is
// full (cap counts the NUL). The cursor is clamped into range first, then
// advanced past the inserted character.
int ui_tf_insert(char *buf, size_t cap, int *cursor, int ch);

// Delete the character before the cursor. Returns 1 if one was removed.
int ui_tf_backspace(char *buf, int *cursor);

// Delete the character at the cursor. Returns 1 if one was removed.
int ui_tf_delete(char *buf, int *cursor);

// Truncate the buffer at the cursor. Returns 1 if anything was removed.
int ui_tf_kill_to_end(char *buf, int *cursor);

// Cursor movement (clamped to [0, strlen(buf)]).
void ui_tf_left(int *cursor);
void ui_tf_right(const char *buf, int *cursor);
void ui_tf_home(int *cursor);
void ui_tf_end(const char *buf, int *cursor);

// Clamp the cursor into [0, strlen(buf)].
void ui_tf_clamp_cursor(const char *buf, int *cursor);

#endif
