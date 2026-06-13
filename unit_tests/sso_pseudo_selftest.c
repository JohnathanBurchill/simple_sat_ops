/*

    Simple Satellite Operations  unit_tests/sso_pseudo_selftest.c

    Tests for the simple_sat_ops-directed pseudo-command expander
    (sso_pseudo.c). An "SSO+<name>(args)!" agenda/compose line is expanded into
    a concrete "CTS1+...!" telecommand at TX time. These pin the first command
    (sync_sat_time_to_ground -> set_system_time), the framing/arg-count checks,
    the verbatim passthrough of a non-SSO+ line, and that a valid expansion
    lints clean through the same firmware-derived linter that gates startup.

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
*/

#include "sso_pseudo.h"
#include "tcmd_lint.h"
#include "tap.h"

#include <stdio.h>
#include <string.h>

// A fixed queue-time clock and pass-start dedup key, so expansions are exact.
//   now_ms    = 1749752645123  -> arg = now_ms + 500 = 1749752645623
//   tssent_ms = 1749752580000  (already minute-truncated by the caller)
#define TEST_NOW_MS    1749752645123LL
#define TEST_TSSENT_MS 1749752580000LL

static sso_pseudo_status_t expand(const char *cmd, long long now_ms,
                                  char *out, size_t out_cap)
{
    sso_pseudo_ctx_t ctx = { .now_ms = now_ms, .tssent_ms = TEST_TSSENT_MS };
    char err[160];
    return sso_pseudo_expand(cmd, &ctx, out, out_cap, err, sizeof err);
}

static void test_is_directed(void)
{
    fprintf(stderr, "sso_pseudo: prefix probe\n");
    tap_ok(sso_pseudo_is_directed("SSO+sync_sat_time_to_ground()!") == 1,
           "SSO+ line is directed");
    tap_ok(sso_pseudo_is_directed("CTS1+hello_world()!") == 0,
           "CTS1+ line is not directed");
    tap_ok(sso_pseudo_is_directed("") == 0, "empty line is not directed");
}

static void test_passthrough(void)
{
    fprintf(stderr, "sso_pseudo: non-SSO+ line passes through verbatim\n");
    char out[512];
    sso_pseudo_status_t st = expand("CTS1+hello_world()!", TEST_NOW_MS,
                                    out, sizeof out);
    tap_okf(st == SSO_PSEUDO_NOT_PSEUDO, "status NOT_PSEUDO (got %d)", (int) st);
    tap_okf(strcmp(out, "CTS1+hello_world()!") == 0,
            "out is a verbatim copy (got \"%s\")", out);
}

static void test_time_sync_expansion(void)
{
    fprintf(stderr, "sso_pseudo: sync_sat_time_to_ground -> set_system_time\n");
    char out[512];
    sso_pseudo_status_t st = expand("SSO+sync_sat_time_to_ground()!",
                                    TEST_NOW_MS, out, sizeof out);
    tap_okf(st == SSO_PSEUDO_OK, "status OK (got %d)", (int) st);
    const char *want =
        "CTS1+set_system_time(1749752645623)@tssent=1749752580000!";
    tap_okf(strcmp(out, want) == 0, "exact expansion (got \"%s\")", out);
}

static void test_argument_is_fresh_tssent_is_pinned(void)
{
    fprintf(stderr, "sso_pseudo: argument tracks now_ms; @tssent stays pinned\n");
    // A later send (now_ms advanced by 10000 ms) must move the set_system_time
    // argument (fresh per queue) but NOT the @tssent (pinned pass-start key).
    char a[512], b[512];
    expand("SSO+sync_sat_time_to_ground()!", TEST_NOW_MS, a, sizeof a);
    expand("SSO+sync_sat_time_to_ground()!", TEST_NOW_MS + 10000, b, sizeof b);
    tap_okf(strcmp(a, b) != 0, "different now_ms -> different command");
    tap_ok(strstr(a, "@tssent=1749752580000!") != NULL
        && strstr(b, "@tssent=1749752580000!") != NULL,
           "both carry the same pinned @tssent");
    tap_ok(strstr(b, "set_system_time(1749752655623)") != NULL,
           "the +10000 ms send sets the later time");
}

static void test_unknown_command(void)
{
    fprintf(stderr, "sso_pseudo: unknown SSO+ command is rejected\n");
    char out[512];
    char err[160];
    sso_pseudo_ctx_t ctx = { .now_ms = TEST_NOW_MS, .tssent_ms = TEST_TSSENT_MS };
    sso_pseudo_status_t st =
        sso_pseudo_expand("SSO+frobnicate()!", &ctx, out, sizeof out, err, sizeof err);
    tap_okf(st == SSO_PSEUDO_UNKNOWN_CMD, "status UNKNOWN_CMD (got %d)", (int) st);
    tap_okf(strstr(err, "unknown SSO+ command") != NULL,
            "err mentions unknown SSO+ command (got \"%s\")", err);
}

static void test_bad_syntax(void)
{
    fprintf(stderr, "sso_pseudo: malformed SSO+ lines are rejected\n");
    char out[512];
    tap_ok(expand("SSO+sync_sat_time_to_ground()", TEST_NOW_MS, out, sizeof out)
               == SSO_PSEUDO_BAD_SYNTAX, "missing '!' -> BAD_SYNTAX");
    tap_ok(expand("SSO+sync_sat_time_to_ground!", TEST_NOW_MS, out, sizeof out)
               == SSO_PSEUDO_BAD_SYNTAX, "missing '(' -> BAD_SYNTAX");
    tap_ok(expand("SSO+()!", TEST_NOW_MS, out, sizeof out)
               == SSO_PSEUDO_BAD_SYNTAX, "missing name -> BAD_SYNTAX");
    tap_ok(expand("SSO+sync_sat_time_to_ground(1)!", TEST_NOW_MS, out, sizeof out)
               == SSO_PSEUDO_BAD_SYNTAX, "wrong arg count -> BAD_SYNTAX");
    tap_ok(expand("SSO+sync_sat_time_to_ground()!!", TEST_NOW_MS, out, sizeof out)
               == SSO_PSEUDO_BAD_SYNTAX, "two '!' -> BAD_SYNTAX");
}

static void test_lint_round_trip(void)
{
    fprintf(stderr, "sso_pseudo: expansion lints clean; linter gates SSO+ lines\n");
    // The concrete expansion is a valid set_system_time(1 uint64 arg).
    char out[512];
    expand("SSO+sync_sat_time_to_ground()!", TEST_NOW_MS, out, sizeof out);
    char msg[512];
    tap_okf(tcmd_lint_command(out, msg, sizeof msg) == TCMD_LINT_OK,
            "expansion lints OK (msg: %s)", msg[0] ? msg : "ok");

    // The linter accepts a well-formed SSO+ line directly (it expands+lints).
    tap_okf(tcmd_lint_command("SSO+sync_sat_time_to_ground()!", msg, sizeof msg)
                == TCMD_LINT_OK, "linter accepts the SSO+ line (msg: %s)",
            msg[0] ? msg : "ok");

    // ...and still rejects an unknown or garbled SSO+ line.
    tap_ok(tcmd_lint_command("SSO+frobnicate()!", msg, sizeof msg)
               == TCMD_LINT_ERROR, "linter rejects unknown SSO+ command");
    tap_ok(tcmd_lint_command("SSO+sync_sat_time_to_ground()", msg, sizeof msg)
               == TCMD_LINT_ERROR, "linter rejects malformed SSO+ line");
}

int main(void)
{
    test_is_directed();
    test_passthrough();
    test_time_sync_expansion();
    test_argument_is_fresh_tssent_is_pinned();
    test_unknown_command();
    test_bad_syntax();
    test_lint_round_trip();
    return tap_done();
}
