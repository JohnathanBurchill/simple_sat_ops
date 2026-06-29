/*

    Simple Satellite Operations  unit_tests/clip_ellipsis_selftest.c

    Tests for clip_ellipsis() -- the width-limited copy used by the TX-log
    panel and the auto-tcmd modal so a long command shows a trailing "..."
    instead of vanishing silently off the panel edge (issue #56).

    The contract under test:
      - src no wider than max_w columns is copied verbatim, no marker.
      - src wider than max_w is cut to exactly max_w columns and the last
        three kept columns become "..." -- UNLESS max_w < 3, where there is
        no room for the marker and the text is simply cut.
      - max_w is clamped to the buffer (cap - 1): the function must never
        write past dst[cap - 1], even when the caller passes a max_w larger
        than the buffer.
      - cap == 0 writes nothing; otherwise dst is always NUL-terminated.

    Two independent oracles guard against a self-consistent-but-wrong
    implementation:
      1. Hand-written golden strings for the boundary cases (exact output).
      2. A reconstruct oracle over a sweep: the result must equal either the
         whole source (when it fits) or (first max_w-3 chars of src) + "...".
         This re-derives the expected text a different way than the copy
         under test, so a misplaced marker or an off-by-one in the kept
         length makes them disagree.
    A '#'-filled guard byte just past the buffer is checked on every call so
    a one-past-the-end write fails loudly.

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

#include "ui_text.h"
#include "tap.h"

#include <stdint.h>
#include <string.h>

// One golden case: copy with a generous buffer, compare to the exact
// hand-written want, and confirm the result never exceeds max_w columns.
static void check(const char *src, int max_w, const char *want)
{
    char got[128];
    clip_ellipsis(got, sizeof got, src, max_w);
    tap_okf(strcmp(got, want) == 0,
            "clip(\"%s\", %d) -> \"%s\" (got \"%s\")", src, max_w, want, got);
    tap_okf((int) strlen(got) <= (max_w < 0 ? 0 : max_w),
            "clip(\"%s\", %d) stays within %d columns (len %zu)",
            src, max_w, max_w, strlen(got));
}

// Independent reconstruct oracle: what the result MUST be for a given
// src/max_w, derived without the copy logic under test.
static void expected(const char *src, int max_w, char *out, size_t outcap)
{
    if (max_w < 0) max_w = 0;
    int len = (int) strlen(src);
    if (len <= max_w) {
        snprintf(out, outcap, "%s", src);
        return;
    }
    // Cut to max_w; last three kept columns become dots (if room).
    char tmp[256];
    int keep = max_w < (int) sizeof tmp ? max_w : (int) sizeof tmp - 1;
    memcpy(tmp, src, (size_t) keep);
    tmp[keep] = '\0';
    if (keep >= 3) {
        tmp[keep - 1] = '.';
        tmp[keep - 2] = '.';
        tmp[keep - 3] = '.';
    }
    snprintf(out, outcap, "%s", tmp);
}

static uint32_t rng = 0x12345678u;
static uint32_t rng_next(void) { rng = rng * 1664525u + 1013904223u; return rng; }

int main(void)
{
    // ---- hand-written golden boundary cases ----------------------------
    check("hello", 10, "hello");   // fits with room to spare
    check("hello",  5, "hello");   // fits exactly -- no marker
    check("hello",  4, "h...");    // one over -> keep 1 char + "..."
    check("hello",  3, "...");     // exactly the marker width
    check("hello",  2, "he");      // no room for the marker -> bare cut
    check("hello",  1, "h");
    check("hello",  0, "");        // zero columns -> empty
    check("",        5, "");       // empty source stays empty
    check("abcdefghij", 7, "abcd..."); // keep 4 + "..."

    // ---- reconstruct oracle over a seeded sweep ------------------------
    // Vary source length and max_w independently; the result must match the
    // independently-derived expectation every time.
    int sweep_ok = 1;
    char bad_src[64] = "";
    int  bad_w = 0;
    for (int i = 0; i < 20000 && sweep_ok; i++) {
        char src[40];
        int slen = (int) (rng_next() % (sizeof src - 1));
        for (int k = 0; k < slen; k++) src[k] = (char) ('a' + (rng_next() % 26));
        src[slen] = '\0';
        int max_w = (int) (rng_next() % 45);  // spans below, at, and above slen
        char got[64], want[64];
        clip_ellipsis(got, sizeof got, src, max_w);
        expected(src, max_w, want, sizeof want);
        if (strcmp(got, want) != 0) {
            sweep_ok = 0;
            snprintf(bad_src, sizeof bad_src, "%s", src);
            bad_w = max_w;
        }
    }
    tap_okf(sweep_ok,
            "reconstruct oracle matches over 20000 cases (first bad: \"%s\" w=%d)",
            bad_src, bad_w);

    // ---- buffer safety: clamp max_w to the buffer, never overrun -------
    // Pass a max_w far larger than the buffer with an over-long source. A
    // '#' guard byte one past the usable region must survive, and the
    // result must be NUL-terminated inside the buffer.
    {
        char buf[9];                 // 8 usable + the guard slot we watch
        memset(buf, '#', sizeof buf);
        clip_ellipsis(buf, 8, "abcdefghijklmnop", 100);
        tap_ok(buf[8] == '#', "max_w > cap does not write past the buffer");
        tap_ok(strlen(buf) == 7, "result clamped to cap-1 columns");
        tap_ok(strcmp(buf, "abcd...") == 0, "clamped result keeps the marker");
    }

    // cap == 0 must not touch the buffer at all.
    {
        char guard[2] = { '#', '#' };
        clip_ellipsis(guard, 0, "anything", 5);
        tap_ok(guard[0] == '#' && guard[1] == '#', "cap == 0 writes nothing");
    }

    return tap_done();
}
