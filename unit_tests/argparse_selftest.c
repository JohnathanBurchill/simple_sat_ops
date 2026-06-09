/*

    Simple Satellite Operations  unit_tests/argparse_selftest.c

    Regression test for the dual-mode parse_args() convention in
    src/cli/argparse.h. The convention is a pattern, not a shared parser, so
    this test defines a representative parse_args() (positional, flag, value
    option, --x/--no-x pair, range check, --help / --help-full) and pins down
    the behaviour every converted tool relies on:

      - parse mode fills the config; help mode prints one line per option;
      - positionals are claimed first and listed before the --options;
      - a lone "-" is a positional, not an option;
      - unknown option / extra positional / out-of-range value -> PARSE_ERROR;
      - --help -> PARSE_HELP (HELP_BRIEF), --help-full -> PARSE_HELP (HELP_FULL,
        which also prints the full-only epilog);
      - parse_help_line() right-aligns the option column.

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

#include "argparse.h"
#include "tap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// A representative tool config + parse_args following the convention.
typedef struct {
    const char *input;
    int raw;
    int rate;
    int hmac;
    int level;
} sample_cfg_t;

#define OPTW 16

static int parse_args(sample_cfg_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // Positional first, so <path> lists above the --options in help.
        if ((a->input == NULL && (arg[0] != '-' || strcmp(arg, "-") == 0)) || help) {
            if (help) parse_help_line(OPTW, "<path>", "input file");
            else a->input = arg;
            matched = 1;
        }
        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "--help-full") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help-full", "this help plus examples");
            else { parse_args(a, argc, argv, HELP_FULL); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "--raw") == 0 || help) {
            if (help) parse_help_line(OPTW, "--raw", "treat input as raw");
            else a->raw = 1;
            matched = 1;
        }
        if (strcmp(arg, "--hmac") == 0 || help) {
            if (help) parse_help_line(OPTW, "--hmac", "enable HMAC");
            else a->hmac = 1;
            matched = 1;
        }
        if (strcmp(arg, "--no-hmac") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-hmac", "disable HMAC");
            else a->hmac = 0;
            matched = 1;
        }
        if (strncmp(arg, "--rate=", 7) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rate=<hz>", "sample rate");
            else a->rate = atoi(arg + 7);
            matched = 1;
        }
        if (strncmp(arg, "--level=", 8) == 0 || help) {
            if (help) parse_help_line(OPTW, "--level=<0..9>", "level in [0,9]");
            else {
                a->level = atoi(arg + 8);
                if (a->level < 0 || a->level > 9) {
                    fprintf(stderr, "selftest: --level out of range\n");
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "selftest: unable to parse '%s'\n", arg);
            return PARSE_ERROR;
        }
    }
    // Full-help-only epilog, printed once.
    if (help >= HELP_FULL)
        printf("\nEPILOG-ONLY: example usage here\n");
    return PARSE_OK;
}

// --- stdout capture helpers (keep parse/help output out of the TAP stream) ---

static int g_saved_stdout = -1;
static FILE *g_cap = NULL;

static void capture_begin(void)
{
    fflush(stdout);
    g_saved_stdout = dup(fileno(stdout));
    g_cap = tmpfile();
    dup2(fileno(g_cap), fileno(stdout));
}

static void capture_end(char *buf, size_t cap)
{
    fflush(stdout);
    dup2(g_saved_stdout, fileno(stdout));
    close(g_saved_stdout);
    g_saved_stdout = -1;
    rewind(g_cap);
    size_t n = fread(buf, 1, cap - 1, g_cap);
    buf[n] = '\0';
    fclose(g_cap);
    g_cap = NULL;
}

// Run parse mode with stdout+stderr suppressed (some paths print). Returns the
// PARSE_* code; *c is filled.
static int run_parse(sample_cfg_t *c, int argc, char **argv)
{
    fflush(stdout);
    fflush(stderr);
    int so = dup(fileno(stdout));
    int se = dup(fileno(stderr));
    FILE *devnull = fopen("/dev/null", "w");
    dup2(fileno(devnull), fileno(stdout));
    dup2(fileno(devnull), fileno(stderr));

    int rc = parse_args(c, argc, argv, HELP_OFF);

    fflush(stdout);
    fflush(stderr);
    dup2(so, fileno(stdout));
    dup2(se, fileno(stderr));
    close(so);
    close(se);
    fclose(devnull);
    return rc;
}

static void capture_help(int level, char *buf, size_t cap)
{
    sample_cfg_t c = {0};
    char *argv[] = { "selftest" };  // ignored in help mode (ntokens == 1, arg == "")
    capture_begin();
    parse_args(&c, 1, argv, level);
    capture_end(buf, cap);
}

int main(void)
{
    // parse: positional + value option
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "file.wav", "--rate=48000" };
        int rc = run_parse(&c, 3, av);
        tap_okf(rc == PARSE_OK, "parse: PARSE_OK");
        tap_okf(c.input && strcmp(c.input, "file.wav") == 0, "parse: positional captured");
        tap_okf(c.rate == 48000, "parse: value option parsed");
    }
    // options may precede the positional (order independent)
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "--rate=96000", "f" };
        run_parse(&c, 3, av);
        tap_okf(c.input && strcmp(c.input, "f") == 0 && c.rate == 96000,
                "parse: option before positional");
    }
    // a lone "-" is a positional, not an option
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "-" };
        int rc = run_parse(&c, 2, av);
        tap_okf(rc == PARSE_OK && c.input && strcmp(c.input, "-") == 0,
                "parse: lone dash is a positional");
    }
    // boolean flag
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "f", "--raw" };
        run_parse(&c, 3, av);
        tap_okf(c.raw == 1, "parse: flag set");
    }
    // --x / --no-x pair: last on the command line wins
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "f", "--hmac", "--no-hmac" };
        run_parse(&c, 4, av);
        tap_okf(c.hmac == 0, "parse: --no-hmac last wins");
    }
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "f", "--no-hmac", "--hmac" };
        run_parse(&c, 4, av);
        tap_okf(c.hmac == 1, "parse: --hmac last wins");
    }
    // unknown option -> PARSE_ERROR
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "--bogus" };
        tap_okf(run_parse(&c, 2, av) == PARSE_ERROR, "parse: unknown option -> PARSE_ERROR");
    }
    // unexpected second positional -> PARSE_ERROR
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "a", "b" };
        tap_okf(run_parse(&c, 3, av) == PARSE_ERROR, "parse: extra positional -> PARSE_ERROR");
    }
    // range check rejects / accepts
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "f", "--level=99" };
        tap_okf(run_parse(&c, 3, av) == PARSE_ERROR, "parse: out-of-range value rejected");
    }
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "f", "--level=3" };
        int rc = run_parse(&c, 3, av);
        tap_okf(rc == PARSE_OK && c.level == 3, "parse: in-range value accepted");
    }
    // --help / --help-full -> PARSE_HELP
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "--help" };
        tap_okf(run_parse(&c, 2, av) == PARSE_HELP, "parse: --help -> PARSE_HELP");
    }
    {
        sample_cfg_t c = {0};
        char *av[] = { "x", "--help-full" };
        tap_okf(run_parse(&c, 2, av) == PARSE_HELP, "parse: --help-full -> PARSE_HELP");
    }
    // HELP_BRIEF: positional listed before options; lists --rate; no full epilog
    {
        char buf[4096];
        capture_help(HELP_BRIEF, buf, sizeof buf);
        char *ppos = strstr(buf, "<path>");
        char *popt = strstr(buf, "--help");
        tap_okf(ppos && popt && ppos < popt, "help: positional listed before options");
        tap_okf(strstr(buf, "--rate=<hz>") != NULL, "help: lists every option");
        tap_okf(strstr(buf, "EPILOG-ONLY") == NULL, "help: brief omits the full-only epilog");
    }
    // HELP_FULL: includes the full-only epilog
    {
        char buf[4096];
        capture_help(HELP_FULL, buf, sizeof buf);
        tap_okf(strstr(buf, "EPILOG-ONLY") != NULL, "help: full includes the epilog");
    }
    // parse_help_line right-aligns the option in the given width
    {
        char buf[64];
        capture_begin();
        parse_help_line(10, "--x", "hi");
        capture_end(buf, sizeof buf);
        tap_okf(strcmp(buf, "       --x - hi\n") == 0, "parse_help_line: right-aligned column");
    }
    return tap_done();
}
