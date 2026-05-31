/*

    Simple Satellite Operations  unit_tests/agenda_line_selftest.c

    Tests for agenda_find_inline_comment() -- the shared rule that
    simple_sat_ops (transmit path) and agenda_check (audit path) use to
    decide where a telecommand ends and an inline '#' comment begins.

    The contract under test:
      - A '#' preceded by whitespace (or at index 0) begins a comment.
      - A '#' embedded in the command text (no preceding whitespace) is
        NOT a comment -- it stays part of the command, because uplinking a
        truncated command is worse than uplinking an unstripped one.
      - On a hit, *cmd_len is the command length with the whitespace run
        before the '#' trimmed off, and the return value points at the '#'.
      - On a miss, the function returns NULL and *cmd_len == strlen(s).

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

#include "agenda_line.h"
#include "tap.h"

#include <stddef.h>
#include <string.h>

#define check(cond, what) tap_ok((cond), (what))

// Helper: assert that `line` splits into command `want_cmd` and comment
// `want_cmt` (NULL want_cmt means "no inline comment expected").
static void expect_split(const char *line, const char *want_cmd,
                         const char *want_cmt)
{
    size_t cmd_len = (size_t) -1;
    const char *cmt = agenda_find_inline_comment(line, &cmd_len);

    if (want_cmt == NULL) {
        tap_okf(cmt == NULL, "[%s] no inline comment", line);
        tap_okf(cmd_len == strlen(line),
                "[%s] cmd_len == strlen on miss", line);
    } else {
        tap_okf(cmt != NULL, "[%s] inline comment found", line);
        tap_okf(cmt != NULL && strcmp(cmt, want_cmt) == 0,
                "[%s] comment == \"%s\"", line, want_cmt);
    }

    // The command is line[0 .. cmd_len); compare against want_cmd.
    size_t want_len = strlen(want_cmd);
    tap_okf(cmd_len == want_len,
            "[%s] cmd_len == %zu (got %zu)", line, want_len, cmd_len);
    tap_okf(cmd_len == want_len && strncmp(line, want_cmd, want_len) == 0,
            "[%s] command == \"%s\"", line, want_cmd);
}

// The bread-and-butter case: a real telecommand with a trailing comment.
static void test_basic_trailing_comment(void)
{
    fprintf(stderr, "inline comment: basic trailing comment\n");
    expect_split("CTS1+hello_world()! # a comment",
                 "CTS1+hello_world()!", "# a comment");
}

// All whitespace before the '#' is trimmed from the command, regardless of
// how much there is or whether it's spaces or tabs.
static void test_whitespace_run_trimmed(void)
{
    fprintf(stderr, "inline comment: whitespace run before '#'\n");
    expect_split("CTS1+ping()!    # padded",
                 "CTS1+ping()!", "# padded");
    expect_split("CTS1+ping()!\t# tab",
                 "CTS1+ping()!", "# tab");
    expect_split("CTS1+ping()! \t # mixed",
                 "CTS1+ping()!", "# mixed");
}

// A '#' with no preceding whitespace is part of the command, not a
// comment. This is the safety property: never silently truncate a command.
static void test_hash_in_command_kept(void)
{
    fprintf(stderr, "inline comment: '#' embedded in command is kept\n");
    expect_split("CTS1+set_label(foo#bar)!", "CTS1+set_label(foo#bar)!", NULL);
    expect_split("CTS1+ping()!#nospace", "CTS1+ping()!#nospace", NULL);
}

// No comment at all -> NULL, cmd_len == full length.
static void test_no_comment(void)
{
    fprintf(stderr, "inline comment: no comment present\n");
    expect_split("CTS1+adcs_identification()!",
                 "CTS1+adcs_identification()!", NULL);
    expect_split("CTS1+x()@tssent=1779961244000@tsexec=1779961244000!",
                 "CTS1+x()@tssent=1779961244000@tsexec=1779961244000!", NULL);
}

// A comment after a command that carries @tssent/@tsexec directives: the
// directives stay with the command, the comment is split off.
static void test_comment_after_directives(void)
{
    fprintf(stderr, "inline comment: comment after @tssent/@tsexec\n");
    expect_split(
        "CTS1+fs_list_directory_json(/,0,20)@tssent=1779961244000@tsexec=1779961244000! # nominal pass",
        "CTS1+fs_list_directory_json(/,0,20)@tssent=1779961244000@tsexec=1779961244000!",
        "# nominal pass");
}

// Index-0 '#' (a whole-line comment) is reported as a comment starting at
// 0 with an empty command. Callers skip these earlier, but the primitive
// must still behave predictably when handed one.
static void test_leading_hash(void)
{
    fprintf(stderr, "inline comment: leading '#'\n");
    size_t cmd_len = (size_t) -1;
    const char *cmt = agenda_find_inline_comment("# whole line", &cmd_len);
    check(cmt != NULL, "leading '#' reported as comment");
    check(cmt != NULL && strcmp(cmt, "# whole line") == 0,
          "leading '#' comment text is the whole line");
    check(cmd_len == 0, "leading '#' leaves an empty command (cmd_len 0)");
}

// The '#' nearest the front (first whitespace-preceded one) wins, so a '#'
// inside the comment body doesn't move the split point.
static void test_first_hash_wins(void)
{
    fprintf(stderr, "inline comment: first delimiter wins\n");
    expect_split("CTS1+ping()! # note with # inside",
                 "CTS1+ping()!", "# note with # inside");
}

// Empty string and a NULL cmd_len pointer must not crash.
static void test_edge_inputs(void)
{
    fprintf(stderr, "inline comment: edge inputs\n");
    size_t cmd_len = (size_t) -1;
    const char *cmt = agenda_find_inline_comment("", &cmd_len);
    check(cmt == NULL, "empty string -> no comment");
    check(cmd_len == 0, "empty string -> cmd_len 0");
    // NULL cmd_len is allowed.
    const char *cmt2 = agenda_find_inline_comment("CTS1+ping()! # x", NULL);
    check(cmt2 != NULL && strcmp(cmt2, "# x") == 0,
          "NULL cmd_len pointer is tolerated");
}

int main(void)
{
    test_basic_trailing_comment();
    test_whitespace_run_trimmed();
    test_hash_in_command_kept();
    test_no_comment();
    test_comment_after_directives();
    test_leading_hash();
    test_first_hash_wins();
    test_edge_inputs();
    return tap_done();
}
