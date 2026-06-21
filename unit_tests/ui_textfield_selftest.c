/*

    Simple Satellite Operations  unit_tests/ui_textfield_selftest.c

    Coverage for src/ui/ui_textfield.c, the cursor-based text-editing
    primitives shared by the tx-compose modal and the auto-telecommand
    editor. Both modals keep a fixed-capacity buffer plus an integer
    cursor and used to carry their own copy of this logic; the shared
    primitive must behave identically so an operator typing into either
    modal sees the same insert / backspace / delete / kill / cursor-move
    behaviour.

    What's covered:
      - insert at end, in the middle, and into an empty buffer.
      - insert refuses to overflow (cap counts the NUL) and reports it.
      - insert clamps an out-of-range cursor before writing.
      - backspace / delete / kill-to-end at the edges (no-op) and mid-string.
      - the "did the buffer change" return value (drives the callers'
        preview-dirty / history-reset side effects).
      - left / right / home / end cursor moves, clamped to [0, len].
      - clamp_cursor pulls an out-of-range cursor back into range.

    Exit status: 0 = all tests passed, non-zero = failure.

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

#include "tap.h"
#include "ui_textfield.h"

#include <string.h>

int main(void)
{
    // --- insert ------------------------------------------------------
    {
        char buf[8] = "";
        int cur = 0;
        int r = ui_tf_insert(buf, sizeof buf, &cur, 'A');
        tap_ok(r == 1 && strcmp(buf, "A") == 0 && cur == 1,
               "insert into empty buffer appends and advances cursor");

        ui_tf_insert(buf, sizeof buf, &cur, 'C');     // "AC", cur=2
        cur = 1;
        r = ui_tf_insert(buf, sizeof buf, &cur, 'B');  // insert B at 1 -> "ABC"
        tap_ok(r == 1 && strcmp(buf, "ABC") == 0 && cur == 2,
               "insert in the middle shifts the tail right");
    }

    // insert refuses to overflow (cap counts the NUL).
    {
        char buf[4] = "abc";   // full: 3 chars + NUL == cap
        int cur = 3;
        int r = ui_tf_insert(buf, sizeof buf, &cur, 'd');
        tap_ok(r == 0 && strcmp(buf, "abc") == 0 && cur == 3,
               "insert into a full buffer is refused and reports 0");
    }

    // insert clamps an out-of-range cursor before writing.
    {
        char buf[8] = "hi";
        int cur = 99;                                  // past the end
        int r = ui_tf_insert(buf, sizeof buf, &cur, '!');
        tap_ok(r == 1 && strcmp(buf, "hi!") == 0 && cur == 3,
               "insert clamps a too-large cursor to the end");

        cur = -5;                                      // before the start
        r = ui_tf_insert(buf, sizeof buf, &cur, '>');
        tap_ok(r == 1 && strcmp(buf, ">hi!") == 0 && cur == 1,
               "insert clamps a negative cursor to the start");
    }

    // --- backspace ---------------------------------------------------
    {
        char buf[8] = "abc";
        int cur = 0;
        int r = ui_tf_backspace(buf, &cur);
        tap_ok(r == 0 && strcmp(buf, "abc") == 0 && cur == 0,
               "backspace at the start is a no-op (returns 0)");

        cur = 2;                                       // between b and c
        r = ui_tf_backspace(buf, &cur);                // removes 'b'
        tap_ok(r == 1 && strcmp(buf, "ac") == 0 && cur == 1,
               "backspace removes the char before the cursor");
    }

    // --- delete ------------------------------------------------------
    {
        char buf[8] = "abc";
        int cur = 3;
        int r = ui_tf_delete(buf, &cur);
        tap_ok(r == 0 && strcmp(buf, "abc") == 0 && cur == 3,
               "delete at the end is a no-op (returns 0)");

        cur = 1;
        r = ui_tf_delete(buf, &cur);                   // removes 'b'
        tap_ok(r == 1 && strcmp(buf, "ac") == 0 && cur == 1,
               "delete removes the char at the cursor, cursor stays");
    }

    // --- kill to end -------------------------------------------------
    {
        char buf[16] = "hello world";
        int cur = 11;
        int r = ui_tf_kill_to_end(buf, &cur);
        tap_ok(r == 0 && strcmp(buf, "hello world") == 0,
               "kill-to-end at the end is a no-op (returns 0)");

        cur = 5;
        r = ui_tf_kill_to_end(buf, &cur);
        tap_ok(r == 1 && strcmp(buf, "hello") == 0 && cur == 5,
               "kill-to-end truncates at the cursor");
    }

    // --- cursor moves ------------------------------------------------
    {
        char buf[8] = "abc";
        int cur = 2;
        ui_tf_left(&cur);
        tap_ok(cur == 1, "left moves the cursor back one");
        ui_tf_home(&cur);
        tap_ok(cur == 0, "home moves the cursor to the start");
        ui_tf_left(&cur);
        tap_ok(cur == 0, "left at the start is a no-op");

        ui_tf_right(buf, &cur);
        tap_ok(cur == 1, "right moves the cursor forward one");
        ui_tf_end(buf, &cur);
        tap_ok(cur == 3, "end moves the cursor to strlen");
        ui_tf_right(buf, &cur);
        tap_ok(cur == 3, "right at the end is a no-op");
    }

    // --- clamp_cursor ------------------------------------------------
    {
        char buf[8] = "abc";
        int cur = 99;
        ui_tf_clamp_cursor(buf, &cur);
        tap_ok(cur == 3, "clamp pulls a too-large cursor to strlen");
        cur = -3;
        ui_tf_clamp_cursor(buf, &cur);
        tap_ok(cur == 0, "clamp pulls a negative cursor to 0");
    }

    return tap_done();
}
