/*

    Simple Satellite Operations  unit_tests/tcmd_lint_selftest.c

    Tests for the firmware-derived telecommand linter (tcmd_lint.c) over the
    generated command table (tcmd_spec.c). The linter is what agenda_check
    runs and what gates simple_sat_ops startup, so it must agree with the
    flight firmware's parser on what is and isn't a valid telecommand.

    The command names / arg counts used below are real entries in the
    generated table (tag sat-1-rc3): hello_world(0), set_system_time(1),
    camera_capture(2), the ground-only mpi_demo_tx_to_mpi(0), and the
    high-risk flash_force_corrupt_filesystem(1). The brick-risk blacklist
    test uses fs_write_file_str(2)/fs_delete_file(1) naming the boot-time
    agenda file default_tcmd_agenda.txt (issue #43).

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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

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

// Assert WARN and that the message mentions `needle`. The command is not
// echoed (these are deliberately over-length lines).
static void expect_warn(const char *cmd, const char *needle)
{
    char msg[512];
    tcmd_lint_severity_t got = tcmd_lint_command(cmd, msg, sizeof msg);
    tap_okf(got == TCMD_LINT_WARN && strstr(msg, needle) != NULL,
            "warn mentioning \"%s\" (got sev=%d msg: %s)",
            needle, (int) got, msg);
}

// Assert DANGER and that the message mentions `needle`.
static void expect_danger(const char *cmd, const char *needle)
{
    char msg[512];
    tcmd_lint_severity_t got = tcmd_lint_command(cmd, msg, sizeof msg);
    tap_okf(got == TCMD_LINT_DANGER && strstr(msg, needle) != NULL,
            "[%s] danger mentioning \"%s\" (got sev=%d msg: %s)",
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

static void test_rf_length_warn(void)
{
    fprintf(stderr, "tcmd_lint: the 215-char RF uplink boundary warns (under firmware max)\n");
    // uart_send_str takes two free-form string args (no per-char format
    // limits), so we can build a line of an exact total length. A command
    // exactly TCMD_RF_MAX_LEN chars long still passes; one char longer
    // warns -- it can't be framed for the radio, though it stays well under
    // the 255-char firmware maximum (so it is not an error).
    char ok[300], over[300];
    int n = snprintf(ok, sizeof ok, "CTS1+uart_send_str(a,");
    while (n < TCMD_RF_MAX_LEN - 2) ok[n++] = 'x';   // leave room for ")!"
    ok[n++] = ')';
    ok[n++] = '!';
    ok[n] = '\0';
    tap_okf((int) strlen(ok) == TCMD_RF_MAX_LEN,
            "built a line of exactly %d chars (got %d)",
            TCMD_RF_MAX_LEN, (int) strlen(ok));
    expect_sev(ok, TCMD_LINT_OK);

    n = snprintf(over, sizeof over, "CTS1+uart_send_str(a,");
    while (n < TCMD_RF_MAX_LEN - 1) over[n++] = 'x'; // one more -> 216 total
    over[n++] = ')';
    over[n++] = '!';
    over[n] = '\0';
    tap_okf((int) strlen(over) == TCMD_RF_MAX_LEN + 1,
            "built a line of exactly %d chars (got %d)",
            TCMD_RF_MAX_LEN + 1, (int) strlen(over));
    expect_warn(over, "RF uplink");
}

static void test_blank(void)
{
    fprintf(stderr, "tcmd_lint: empty string is OK (caller skips blanks)\n");
    expect_sev("", TCMD_LINT_OK);
}

static void test_arg_types(void)
{
    fprintf(stderr, "tcmd_lint: per-argument type checks mirror the firmware parser\n");

    // Well-formed arguments of each type pass.
    expect_sev("CTS1+echo_back_uint32_args(1,2,3)!", TCMD_LINT_OK);          // uint64
    expect_sev("CTS1+adcs_set_wheel_speed(-1,2,-3)!", TCMD_LINT_OK);         // int64 (sign)
    expect_sev("CTS1+adcs_set_magnetorquer_output(1.5,-2.0,3)!", TCMD_LINT_OK); // double
    expect_sev("CTS1+bulkup16(DEADBEEF)!", TCMD_LINT_OK);                    // hex
    expect_sev("CTS1+bulkup16(DE AD BE EF)!", TCMD_LINT_OK);                 // hex, spaced bytes
    expect_sev("CTS1+bulkup64(SGVsbG8=)!", TCMD_LINT_OK);                    // base64
    expect_sev("CTS1+bulkup64(SGVs bG8=)!", TCMD_LINT_OK);                   // base64, spaced quartets

    // The headline case: a space after a comma is fatal for a numeric arg
    // (the firmware integer parser sees a leading space and rejects), so the
    // linter errors -- but a string arg tolerates it, so that one passes.
    expect_err("CTS1+echo_back_uint32_args(1, 2, 3)!", "arg 1");
    expect_sev("CTS1+uart_send_str(hello, world)!", TCMD_LINT_OK);

    // uint64 rejects sign, non-digit, and >19 digits.
    expect_err("CTS1+echo_back_uint32_args(-1,2,3)!", "unsigned integer");
    expect_err("CTS1+echo_back_uint32_args(1,2,x)!", "unsigned integer");
    expect_err("CTS1+set_system_time(12345678901234567890)!", "unsigned integer");

    // int64 takes a sign but not a decimal point; double rejects exponent and
    // a dot at the end.
    expect_err("CTS1+adcs_set_wheel_speed(1,2,3.5)!", "integer");
    expect_err("CTS1+adcs_set_magnetorquer_output(1e3,2,3)!", "decimal");
    expect_err("CTS1+adcs_set_magnetorquer_output(1.,2,3)!", "decimal");

    // hex rejects odd nibble count, a non-hex char, and a space inside a byte.
    expect_err("CTS1+bulkup16(DEADBEE)!", "hex");
    expect_err("CTS1+bulkup16(GG)!", "hex");
    expect_err("CTS1+bulkup16(D EAD)!", "hex");

    // base64 rejects a partial quartet and out-of-alphabet characters.
    expect_err("CTS1+bulkup64(SGV)!", "base64");
    expect_err("CTS1+bulkup64(@@@@)!", "base64");

    // A bad arg is still caught with well-formed suffix tags appended.
    expect_err("CTS1+echo_back_uint32_args(1, 2, 3)@tssent=123!", "arg 1");
}

static void test_arg_types_consistent(void)
{
    fprintf(stderr, "tcmd_spec: arg_types length matches num_args; codes are known\n");
    int len_ok = 1, code_ok = 1;
    for (size_t i = 0; i < TCMD_SPEC_COUNT; ++i) {
        const tcmd_spec_t *s = &TCMD_SPEC[i];
        const char *t = s->arg_types ? s->arg_types : NULL;
        if (t == NULL || (int) strlen(t) != s->num_args) {
            len_ok = 0;
            fprintf(stderr, "  %s: num_args=%d arg_types=\"%s\"\n",
                    s->name, s->num_args, t ? t : "(null)");
            continue;
        }
        for (const char *p = t; *p; ++p) {
            if (!strchr("uidhbs?", *p)) {
                code_ok = 0;
                fprintf(stderr, "  %s: unknown type code '%c'\n", s->name, *p);
            }
        }
    }
    tap_ok(len_ok, "every row: strlen(arg_types) == num_args");
    tap_ok(code_ok, "every row: arg_types uses only known codes");
}

static void test_file_level(void)
{
    // Locks in that the file path still allows blank lines, whole-line '#'
    // comments, and inline ' # ...' comments -- while the per-arg check still
    // fires on the (comment-stripped) command.
    fprintf(stderr, "tcmd_lint_file: blanks/comments allowed; arg checks still fire\n");
    char path[] = "/tmp/sso_tcmd_lint_selftest_XXXXXX";
    int fd = mkstemp(path);
    tap_ok(fd >= 0, "mkstemp temp agenda");
    if (fd < 0) return;
    FILE *f = fdopen(fd, "w");
    if (!f) { close(fd); unlink(path); tap_ok(0, "fdopen temp agenda"); return; }
    fputs("# whole-line comment, ignored\n", f);
    fputs("\n", f);                                                   // blank
    fputs("   \n", f);                                               // whitespace only
    fputs("CTS1+hello_world()!\n", f);                               // valid
    fputs("CTS1+echo_back_uint32_args(1,2,3)!   # inline ok\n", f);  // valid + inline comment
    fputs("CTS1+echo_back_uint32_args(1, 2, 3)! # bad: spaces\n", f);// 1 error (numeric spaces)
    fclose(f);

    int warns = -1;
    FILE *out = fopen("/dev/null", "w");
    // NULL danger_count exercises the optional-out-pointer path.
    int errors = tcmd_lint_file(path, out ? out : stderr, &warns, NULL);
    if (out) fclose(out);
    unlink(path);
    tap_okf(errors == 1,
            "exactly one error line (the spaced numeric args); blanks/comments skipped (got %d)",
            errors);
}

static void test_dangerous_blacklist(void)
{
    fprintf(stderr, "tcmd_lint: brick-risk blacklist (boot-time agenda file)\n");

    // The headline case: a routine, perfectly well-formed command that arms
    // the boot-time agenda. fs_write_file_str is operation-readiness with two
    // string args, so nothing else here would flag it -- yet bad contents in
    // default_tcmd_agenda.txt boot-loop the satellite. Must be DANGER.
    expect_danger("CTS1+fs_write_file_str(default_tcmd_agenda.txt,x)!",
                  "default_tcmd_agenda.txt");
    // Caught no matter which command names the file (substring match).
    expect_danger("CTS1+fs_delete_file(default_tcmd_agenda.txt)!",
                  "default_tcmd_agenda.txt");
    expect_danger("CTS1+agenda_enqueue_from_file(default_tcmd_agenda.txt,0,0)!",
                  "default_tcmd_agenda.txt");

    // Danger outranks a structural error on the same line (3 > 2): the arg
    // count is wrong AND the filename is blacklisted -> still DANGER, so the
    // brick risk is never masked by an ordinary parse error.
    expect_sev("CTS1+fs_write_file_str(default_tcmd_agenda.txt)!", TCMD_LINT_DANGER);

    // No false positives: a different filename is fine.
    expect_sev("CTS1+fs_write_file_str(todays_agenda.txt,x)!", TCMD_LINT_OK);
    expect_sev("CTS1+fs_delete_file(beacon_log.txt)!", TCMD_LINT_OK);

    // File level: a danger is counted in danger_count, NOT the error return,
    // because the two gate separately (different override flags).
    char path[] = "/tmp/sso_tcmd_danger_selftest_XXXXXX";
    int fd = mkstemp(path);
    tap_ok(fd >= 0, "mkstemp danger agenda");
    if (fd < 0) return;
    FILE *f = fdopen(fd, "w");
    if (!f) { close(fd); unlink(path); tap_ok(0, "fdopen danger agenda"); return; }
    fputs("CTS1+hello_world()!\n", f);                            // clean
    fputs("CTS1+fs_write_file_str(default_tcmd_agenda.txt,x)!\n", f); // 1 danger
    fclose(f);
    int warns = -1, dangers = -1;
    FILE *out = fopen("/dev/null", "w");
    int errors = tcmd_lint_file(path, out ? out : stderr, &warns, &dangers);
    if (out) fclose(out);
    unlink(path);
    tap_okf(errors == 0 && dangers == 1,
            "file: 0 errors, 1 danger for the boot-agenda write (got err=%d danger=%d)",
            errors, dangers);
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
    test_rf_length_warn();
    test_blank();
    test_arg_types();
    test_arg_types_consistent();
    test_dangerous_blacklist();
    test_file_level();
    test_spec_table();
    return tap_done();
}
