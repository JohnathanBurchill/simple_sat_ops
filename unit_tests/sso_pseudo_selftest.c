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
    return sso_pseudo_expand(cmd, &ctx, out, out_cap, NULL, err, sizeof err);
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
        sso_pseudo_expand("SSO+frobnicate()!", &ctx, out, sizeof out, NULL, err, sizeof err);
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

static void test_raw_csp(void)
{
    fprintf(stderr, "sso_pseudo: ping_ax100 / raw_csp route raw CSP\n");
    sso_pseudo_ctx_t ctx = { .now_ms = TEST_NOW_MS, .tssent_ms = TEST_TSSENT_MS };
    char out[512], err[160];
    sso_pseudo_route_t r;

    sso_pseudo_expand("CTS1+hello_world()!", &ctx, out, sizeof out, &r, err, sizeof err);
    tap_ok(r.is_hex == 0 && r.hdr.dst == 1 && r.hdr.dport == 7 && r.hdr.src == 10,
           "plain telecommand gets the OBC default route");

    sso_pseudo_status_t st =
        sso_pseudo_expand("SSO+ping_ax100()!", &ctx, out, sizeof out, &r, err, sizeof err);
    tap_okf(st == SSO_PSEUDO_OK, "ping_ax100 OK (err: %s)", err);
    tap_okf(strcmp(out, "50494e47") == 0, "ping payload is ASCII PING (got %s)", out);
    tap_okf(r.is_hex == 1 && r.hdr.dst == 5 && r.hdr.dport == 1 && r.hdr.sport == 9,
            "ping goes to node 5 port 1 from port 9 as hex (dst=%u dport=%u sport=%u)",
            r.hdr.dst, r.hdr.dport, r.hdr.sport);

    st = sso_pseudo_expand("SSO+raw_csp(5,7,20,0A0b)!", &ctx, out, sizeof out, &r, err, sizeof err);
    tap_okf(st == SSO_PSEUDO_OK && strcmp(out, "0A0b") == 0
            && r.hdr.dst == 5 && r.hdr.dport == 7 && r.hdr.sport == 20 && r.is_hex,
            "raw_csp(5,7,20,0A0b) routes the hex payload (out=\"%s\")", out);

    const char *bad[] = { "SSO+raw_csp(32,7,16,00)!", "SSO+raw_csp(5,64,16,00)!",
                          "SSO+raw_csp(5,7,64,00)!", "SSO+raw_csp(5,7,x,00)!",
                          "SSO+raw_csp(5,7,16,0)!", "SSO+raw_csp(5,7,16,zz)!",
                          "SSO+raw_csp(x,7,16,00)!", "SSO+raw_csp(5,7,16,)!",
                          "SSO+raw_csp(5,7,00)!" };
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; ++i)
        tap_okf(sso_pseudo_expand(bad[i], &ctx, out, sizeof out, &r, err, sizeof err)
                    == SSO_PSEUDO_BAD_SYNTAX, "%s rejected", bad[i]);

    char msg[512];
    tap_ok(tcmd_lint_command("SSO+ping_ax100()!", msg, sizeof msg) == TCMD_LINT_OK,
           "linter accepts SSO+ping_ax100()!");
    tap_ok(tcmd_lint_command("SSO+raw_csp(5,7,16,zz)!", msg, sizeof msg) == TCMD_LINT_ERROR,
           "linter rejects bad SSO+raw_csp hex");
}

int main(void)
{
    test_raw_csp();
    test_is_directed();
    test_passthrough();
    test_time_sync_expansion();
    test_argument_is_fresh_tssent_is_pinned();
    test_unknown_command();
    test_bad_syntax();
    test_lint_round_trip();
    return tap_done();
}
