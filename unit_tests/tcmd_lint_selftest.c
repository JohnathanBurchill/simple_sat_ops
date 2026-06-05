/*

    Simple Satellite Operations  unit_tests/tcmd_lint_selftest.c

    Tests for the firmware-derived telecommand linter (tcmd_lint.c) over the
    generated command table (tcmd_spec.c). The linter is what agenda_check
    runs and what gates simple_sat_ops startup, so it must agree with the
    flight firmware's parser on what is and isn't a valid telecommand.

    The command names / arg counts used below are real entries in the
    generated table (tag sat-1-rc3): hello_world(0), set_system_time(1),
    camera_capture(2), the ground-only mpi_demo_tx_to_mpi(0), and the
    high-risk flash_force_corrupt_filesystem(1).

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

#include "tcmd_lint.h"
#include "tcmd_spec.h"
#include "tap.h"

#include <stdio.h>
#include <string.h>

// Assert the worst severity for a command, and on mismatch show the message.
static void expect_sev(const char *cmd, tcmd_lint_severity_t want)
{
    char msg[512];
    tcmd_lint_severity_t got = tcmd_lint_command(cmd, msg, sizeof msg);
    tap_okf(got == want, "[%s] severity=%d want=%d (msg: %s)",
            cmd, (int) got, (int) want, msg[0] ? msg : "ok");
}

// Assert ERROR and that the message mentions `needle`.
static void expect_err(const char *cmd, const char *needle)
{
    char msg[512];
    tcmd_lint_severity_t got = tcmd_lint_command(cmd, msg, sizeof msg);
    tap_okf(got == TCMD_LINT_ERROR && strstr(msg, needle) != NULL,
            "[%s] error mentioning \"%s\" (got sev=%d msg: %s)",
            cmd, needle, (int) got, msg);
}

static void test_valid_commands(void)
{
    fprintf(stderr, "tcmd_lint: valid commands pass\n");
    expect_sev("CTS1+hello_world()!", TCMD_LINT_OK);
    expect_sev("CTS1+available_telecommands()!", TCMD_LINT_OK);
    expect_sev("CTS1+set_system_time(1779961244000)!", TCMD_LINT_OK);
    expect_sev("CTS1+camera_capture(0,1)!", TCMD_LINT_OK);
    // Suffix tags after the args are fine when well-formed.
    expect_sev("CTS1+hello_world()@tssent=1779961244000@tsexec=1779961244000!",
               TCMD_LINT_OK);
    expect_sev("CTS1+set_system_time(123)@tssent=123@resp_fname=t.json!",
               TCMD_LINT_OK);
}

static void test_unknown_name(void)
{
    fprintf(stderr, "tcmd_lint: unknown command name is an error\n");
    expect_err("CTS1+definitely_not_a_real_command()!", "unknown telecommand");
    expect_err("CTS1+Hello_World()!", "unknown telecommand"); // case-sensitive
}

static void test_arg_count(void)
{
    fprintf(stderr, "tcmd_lint: argument count must match\n");
    expect_err("CTS1+hello_world(1)!", "expects 0 arg");      // 0 expected, 1 given
    expect_err("CTS1+set_system_time()!", "expects 1 arg");   // 1 expected, 0 given
    expect_err("CTS1+camera_capture(0)!", "expects 2 arg");   // 2 expected, 1 given
    expect_err("CTS1+camera_capture(0,1,2)!", "expects 2 arg"); // 2 expected, 3 given
}

static void test_framing(void)
{
    fprintf(stderr, "tcmd_lint: CTS1+...! framing\n");
    expect_err("hello_world()!", "prefix");
    expect_err("CTS1+hello_world()", "terminating");          // no '!'
    expect_err("CTS1+hello_world()!!", "more than one");      // two '!'
    expect_err("CTS1+hello_world!", "missing '('");           // no args parens
    expect_err("CTS1+hello_world(!", "missing ')'");          // no closing paren
}

static void test_directives(void)
{
    fprintf(stderr, "tcmd_lint: @tssent / @tsexec must be well-formed\n");
    expect_err("CTS1+hello_world()@tssent=abc!", "@tssent");
    expect_err("CTS1+hello_world()@tsexec=!", "@tsexec");
}

static void test_readiness_warns(void)
{
    fprintf(stderr, "tcmd_lint: non-operation readiness warns (does not block)\n");
    // Ground-only and high-risk commands, with correct arg counts, warn.
    expect_sev("CTS1+mpi_demo_tx_to_mpi()!", TCMD_LINT_WARN);
    expect_sev("CTS1+flash_force_corrupt_filesystem(1)!", TCMD_LINT_WARN);
    // But a real error on the same command outranks the warning.
    expect_sev("CTS1+flash_force_corrupt_filesystem()!", TCMD_LINT_ERROR);
}

static void test_length_limit(void)
{
    fprintf(stderr, "tcmd_lint: over-length command is an error\n");
    char big[400];
    int n = snprintf(big, sizeof big, "CTS1+set_system_time(");
    for (int i = 0; i < 300; ++i) big[n++] = '0';
    big[n++] = ')';
    big[n++] = '!';
    big[n] = '\0';
    char msg[512];
    tap_ok(tcmd_lint_command(big, msg, sizeof msg) == TCMD_LINT_ERROR,
           "300-char-arg command flagged as error");
}

static void test_blank(void)
{
    fprintf(stderr, "tcmd_lint: empty string is OK (caller skips blanks)\n");
    expect_sev("", TCMD_LINT_OK);
}

static void test_spec_table(void)
{
    fprintf(stderr, "tcmd_spec: generated table sanity\n");
    tap_okf(TCMD_SPEC_COUNT == 242, "table has 242 commands (got %zu)",
            TCMD_SPEC_COUNT);
    tap_ok(strcmp(TCMD_SPEC_FW_TAG, "sat-1-rc3") == 0,
           "table generated from sat-1-rc3");
    tap_ok(tcmd_spec_find("hello_world", 11) != NULL, "find known command");
    tap_ok(tcmd_spec_find("hello_world", 5) == NULL,
           "partial-length name does not match (full-length only)");
    tap_ok(tcmd_spec_find("nope_not_here", 13) == NULL, "find unknown -> NULL");
}

int main(void)
{
    test_valid_commands();
    test_unknown_name();
    test_arg_count();
    test_framing();
    test_directives();
    test_readiness_warns();
    test_length_limit();
    test_blank();
    test_spec_table();
    return tap_done();
}
