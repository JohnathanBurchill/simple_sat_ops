/*

    Simple Satellite Operations  src/cli/argparse.h

    A convention (not a library) for keeping a tool's command-line parsing
    and its --help text on the SAME line of source, so the two can't drift.

    There is no separate option table and no preprocessor magic: each tool
    writes one ordinary function

        int parse_args(State *state, int argc, char **argv, int help);

    The `help` argument is a level: HELP_OFF parses argv and fills *state;
    HELP_BRIEF prints one help line per option (--help); HELP_FULL prints the
    same plus any extended detail a tool gates behind --help-full. Each option
    is a single self-contained block whose test carries "|| help":

        if (strcmp(arg, "--option1") == 0 || help) {
            if (help) parse_help_line(OPTW, " --option1", "enable option 1");
            else      state->option1 = 1;
            matched = 1;
        }

    The "|| help" forces every block's test true in help mode, so each one
    prints its line and falls through to the next (the help "fall-through").
    In parse mode only the block whose test matches runs its body, and the
    `matched` flag lets the driver flag an unrecognized token afterwards.

    Declare POSITIONAL blocks (<path>, <name>, ...) BEFORE the option blocks
    so the <...> arguments list above the --options in help. Block order does
    not affect parse-mode correctness (tests are mutually exclusive), only the
    order help prints them.

    A tool's parse_args therefore looks like:

        #define OPTW 25   // widest option name in THIS tool + a small margin

        static int parse_args(State *s, int argc, char **argv, int help)
        {
            // help mode: one pass with a throwaway token; parse mode: one
            // pass per token (so the matching block runs exactly once).
            int ntokens = help ? 1 : argc - 1;
            for (int t = 0; t < ntokens; ++t) {
                const char *arg = help ? "" : argv[t + 1];
                int matched = 0;

                // Positionals first so <...> lists above the --options.
                if ((s->path == NULL && (arg[0] != '-' || strcmp(arg, "-") == 0)) || help) {
                    if (help) parse_help_line(OPTW, "<path>", "input file");
                    else s->path = arg;
                    matched = 1;
                }
                if (strcmp(arg, "--help") == 0 || help) {
                    if (help) parse_help_line(OPTW, "--help", "show this help and exit");
                    else { parse_args(s, argc, argv, HELP_BRIEF); return PARSE_HELP; }
                    matched = 1;
                }
                // Only tools that want extended help add this block:
                if (strcmp(arg, "--help-full") == 0 || help) {
                    if (help) parse_help_line(OPTW, "--help-full", "help plus examples");
                    else { parse_args(s, argc, argv, HELP_FULL); return PARSE_HELP; }
                    matched = 1;
                }
                // ... one block per option ...

                if (!matched && !help) {
                    fprintf(stderr, "tool: unable to parse '%s'\n", arg);
                    return PARSE_ERROR;
                }
            }
            // Full-help-only epilog (examples, long notes), printed once.
            if (help >= HELP_FULL)
                printf("\nExamples:\n  tool foo.in --rate=48000\n");
            return PARSE_OK;
        }

    main() does:

        if (sso_version_handle(argc, argv, "tool")) return 0;  // owns -V/--version
        State s = { ...defaults... };
        switch (parse_args(&s, argc, argv, HELP_OFF)) {
            case PARSE_HELP:  return 0;   // help already printed to stdout
            case PARSE_ERROR: return 1;   // message already printed to stderr
        }

    Recipes (all stay ordinary C, which is the point):
      - Value option "--rate=9600": test strncmp(arg, "--rate=", 7) == 0,
        body atoi(arg + 7). Space form "--rate 9600": match "--rate" exactly,
        then read argv[t + 1] and do t++ (bounds-check it; if t+1 == argc set
        an error). Filename options that want bash tab-completion use the
        space form and reject "--rate=..." with a hint.
      - Boolean flag: test strcmp(arg, "--flag") == 0, body sets the field.
        A "--flag" / "--no-flag" pair is just two adjacent blocks.
      - Positional: a block whose test claims the first non-option token,
        e.g. (s->path == NULL && (arg[0] != '-' || strcmp(arg, "-") == 0)).
        Use strcmp(arg, "-") rather than indexing arg[1] so the help-mode
        sentinel "" is never read past its end. Declare positionals first.
      - --help-full: add the --help-full block above; print extra per-option
        detail inside a block with `if (help >= HELP_FULL) ...`, and/or a
        trailing examples epilog after the loop gated the same way.
      - Cross-argument logic (mutual exclusion, "explicit beats inferred"
        extension sniffing, range clamps) lives in plain C in the block body
        or just after parse_args returns -- not modeled here.

    Help is RIGHT-aligned via parse_help_line()'s "%*s" (matches main.c's
    status panels). printf width is a MINIMUM, so size OPTW to the tool's
    widest option name or longer entries overflow the column.

    Alternative kept on the table: an arg_*(ap, "--name", meta, help) helper
    set with a visible token loop in main() and no per-tool parse_args
    boilerplate. It shares this PARSE_* enum and parse_help_line(); a tool may
    use whichever style reads better for it.

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

#ifndef SSO_CLI_ARGPARSE_H
#define SSO_CLI_ARGPARSE_H

#include <stdio.h>

// parse_args() return codes. main() maps PARSE_HELP -> exit 0 (the help text
// was already printed to stdout) and PARSE_ERROR -> exit 1 (a diagnostic was
// already printed to stderr).
enum {
    PARSE_OK = 0,
    PARSE_HELP,
    PARSE_ERROR
};

// `help` levels passed to a tool's parse_args(). HELP_OFF parses argv;
// HELP_BRIEF prints the one-line-per-option block (--help); HELP_FULL also
// prints whatever extended detail a tool gates behind --help-full. The option
// blocks test "... || help", so any value > 0 selects help mode; a tool
// without --help-full simply never passes HELP_FULL.
enum {
    HELP_OFF = 0,
    HELP_BRIEF = 1,
    HELP_FULL = 2
};

// Print one right-aligned "<option> - <help>" help line. `width` is the
// option column width; size it per tool to the widest option name (+ a small
// margin). printf width is a MINIMUM, so an option longer than `width` simply
// overflows the column rather than being truncated.
static inline void parse_help_line(int width, const char *option,
                                   const char *help)
{
    printf("%*s - %s\n", width, option, help);
}

#endif // SSO_CLI_ARGPARSE_H
