/*

   Simple Satellite Operations  ui_textfield.c

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

#include "ui_textfield.h"

#include <string.h>

int ui_tf_insert(char *buf, size_t cap, int *cursor, int ch)
{
    int n = (int) strlen(buf);
    if (n + 1 >= (int) cap) return 0;
    int cur = *cursor;
    if (cur < 0) cur = 0;
    if (cur > n) cur = n;
    // Shift the tail (including the existing NUL) right by one.
    memmove(buf + cur + 1, buf + cur, (size_t)(n - cur + 1));
    buf[cur] = (char) ch;
    *cursor = cur + 1;
    return 1;
}

int ui_tf_backspace(char *buf, int *cursor)
{
    int n = (int) strlen(buf);
    int cur = *cursor;
    if (cur <= 0 || n == 0) return 0;
    memmove(buf + cur - 1, buf + cur, (size_t)(n - cur + 1));
    *cursor = cur - 1;
    return 1;
}

int ui_tf_delete(char *buf, int *cursor)
{
    int n = (int) strlen(buf);
    int cur = *cursor;
    if (cur >= n) return 0;
    memmove(buf + cur, buf + cur + 1, (size_t)(n - cur));
    return 1;
}

int ui_tf_kill_to_end(char *buf, int *cursor)
{
    int n = (int) strlen(buf);
    int cur = *cursor;
    if (cur >= n) return 0;
    buf[cur] = '\0';
    return 1;
}

void ui_tf_left(int *cursor)
{
    if (*cursor > 0) (*cursor)--;
}

void ui_tf_right(const char *buf, int *cursor)
{
    int n = (int) strlen(buf);
    if (*cursor < n) (*cursor)++;
}

void ui_tf_home(int *cursor)
{
    *cursor = 0;
}

void ui_tf_end(const char *buf, int *cursor)
{
    *cursor = (int) strlen(buf);
}

void ui_tf_clamp_cursor(const char *buf, int *cursor)
{
    int n = (int) strlen(buf);
    if (*cursor < 0) *cursor = 0;
    if (*cursor > n) *cursor = n;
}
