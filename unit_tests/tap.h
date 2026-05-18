/*

    Simple Satellite Operations  unit_tests/tap.h

    Tiny TAP (Test Anything Protocol) emitter shared by every *_selftest
    binary. Each selftest #include's this header; calls tap_ok(cond,
    "description") (or tap_okf(cond, fmt, ...)) for each assertion; then
    main() ends with `return tap_done();` to emit "1..N" and exit non-
    zero on failure.

    Output format (stdout, line-buffered):

        ok 1 - clean decode returns 0 errors
        ok 2 - clean decode preserves data
        not ok 3 - correct 1 byte error(s) (count) # got -1, want 1
        ...
        1..42

    Lines starting with "#" are diagnostic comments and are passed
    through by the runner unchanged. "Bail out! <reason>" terminates a
    group early with a clean abort.

    Single-TU usage only — the helpers are `static`, so every selftest
    binary gets its own counters with no link conflicts.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#ifndef SSO_UNIT_TESTS_TAP_H
#define SSO_UNIT_TESTS_TAP_H

#include <stdarg.h>
#include <stdio.h>

static int tap_seq  = 0;
static int tap_fail = 0;

static void tap_okf(int cond, const char *fmt, ...)
{
    ++tap_seq;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (cond) {
        printf("ok %d - %s\n", tap_seq, buf);
    } else {
        printf("not ok %d - %s\n", tap_seq, buf);
        ++tap_fail;
    }
    fflush(stdout);
}

__attribute__((unused))
static int tap_ok(int cond, const char *what)
{
    tap_okf(cond, "%s", what ? what : "");
    return cond;
}

__attribute__((unused, format(printf, 1, 2)))
static void tap_diag(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    printf("# %s\n", buf);
    fflush(stdout);
}

__attribute__((unused, format(printf, 1, 2)))
static void tap_bail(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    printf("Bail out! %s\n", buf);
    fflush(stdout);
}

static int tap_done(void)
{
    printf("1..%d\n", tap_seq);
    fflush(stdout);
    return tap_fail ? 1 : 0;
}

#endif
