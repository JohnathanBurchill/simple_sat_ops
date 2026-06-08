/*

    Simple Satellite Operations  tcmd_lint.c

    See tcmd_lint.h. The single-command checks deliberately follow the order
    and rules of the flight firmware's TCMD_parse_full_telecommand so the
    ground verdict matches what the satellite would do with the same bytes.

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
#include "agenda_line.h"

#include <ctype.h>
#include <errno.h>
#include <string.h>

// Mirror of the firmware constants (telecommand_parser.h / _types.h).
#define TCMD_PREFIX        "CTS1+"
#define TCMD_PREFIX_LEN    5
#define TCMD_MAX_FULL_LEN  255   // full length incl. null terminator
#define TCMD_MAX_ARGS_LEN  240   // args-without-parens incl. null terminator

// Append one issue to the message buffer, semicolon-separated, and raise the
// running worst severity.
static void flag(char *msg, size_t cap, size_t *len,
                 tcmd_lint_severity_t *worst, tcmd_lint_severity_t sev,
                 const char *text)
{
    if (sev > *worst) *worst = sev;
    if (!msg || cap == 0) return;
    if (*len > 0 && *len + 2 < cap) {
        msg[(*len)++] = ';';
        msg[(*len)++] = ' ';
    }
    size_t room = (*len < cap) ? cap - 1 - *len : 0;
    size_t t = strlen(text);
    if (t > room) t = room;
    memcpy(msg + *len, text, t);
    *len += t;
    msg[*len] = '\0';
}

static int is_name_char(char c)
{
    return isalnum((unsigned char) c) || c == '_';
}

// Per-argument-type validators. Each mirrors the corresponding firmware
// extractor in telecommand_args_helpers.c and answers one question: would the
// satellite's parser accept this token? They are deliberately conservative --
// they reject only what the firmware definitely rejects, so the linter never
// blocks a command the satellite would actually run. (A token is the raw text
// between two commas; no trimming here, exactly as the firmware splits.)

// 'u' uint64: 1..19 digits, nothing else (TCMD_ascii_to_uint64).
static int arg_ok_uint64(const char *s, size_t n)
{
    if (n < 1 || n > 19) return 0;
    for (size_t i = 0; i < n; ++i)
        if (s[i] < '0' || s[i] > '9') return 0;
    return 1;
}

// 'i' int64: optional leading '-', then digits (TCMD_ascii_to_int64).
static int arg_ok_int64(const char *s, size_t n)
{
    if (n == 0) return 0;
    size_t i = (s[0] == '-') ? 1 : 0;   // lone "-" the firmware accepts as 0
    for (; i < n; ++i)
        if (s[i] < '0' || s[i] > '9') return 0;
    return 1;
}

// 'd' double: optional leading '-', digits, at most one '.', not at either
// end, no exponent (TCMD_ascii_to_double).
static int arg_ok_double(const char *s, size_t n)
{
    if (n == 0) return 0;
    int seen_dot = 0;
    for (size_t i = 0; i < n; ++i) {
        char c = s[i];
        if (i == 0 && c == '-') continue;
        if (c == '.' && !seen_dot && i != 0 && i != n - 1) { seen_dot = 1; continue; }
        if (c < '0' || c > '9') return 0;
    }
    return 1;
}

// 'h' hex bytes: hex digits; ' '/'_' only between whole bytes; even nibble
// count (TCMD_extract_hex_array_arg). Empty token = 0 bytes, accepted.
static int arg_ok_hex(const char *s, size_t n)
{
    int nibbles = 0;
    for (size_t i = 0; i < n; ++i) {
        char c = s[i];
        if (c == ' ' || c == '_') {
            if (nibbles % 2 != 0) return 0;   // separator mid-byte
            continue;
        }
        if (!isxdigit((unsigned char) c)) return 0;
        nibbles++;
    }
    return (nibbles % 2) == 0;
}

// 'b' base64: standard + URL-safe alphabet + '='; ' ' only between whole
// quartets (TCMD_extract_base64_array_arg). Empty token accepted.
static int arg_ok_base64(const char *s, size_t n)
{
    int q = 0;   // chars in the current quartet
    for (size_t i = 0; i < n; ++i) {
        char c = s[i];
        if (c == ' ') {
            if (q != 0 && q != 4) return 0;   // space mid-quartet
            continue;
        }
        int is_b64 = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')
                  || (c >= '0' && c <= '9')
                  || c == '+' || c == '/' || c == '-' || c == '_' || c == '=';
        if (!is_b64) return 0;
        if (++q == 4) q = 0;
    }
    return q == 0;   // no dangling partial quartet
}

// Validate one argument token against its firmware type code. Returns 1 if the
// satellite's parser would accept it. 's' (string / free-form) and '?'
// (unknown) accept anything -- the firmware doesn't gate those on format.
static int arg_token_ok(char type_code, const char *tok, size_t n)
{
    switch (type_code) {
        case 'u': return arg_ok_uint64(tok, n);
        case 'i': return arg_ok_int64(tok, n);
        case 'd': return arg_ok_double(tok, n);
        case 'h': return arg_ok_hex(tok, n);
        case 'b': return arg_ok_base64(tok, n);
        default:  return 1;   // 's', '?', or anything unexpected: accept
    }
}

static const char *arg_type_expectation(char type_code)
{
    switch (type_code) {
        case 'u': return "an unsigned integer (digits only)";
        case 'i': return "an integer";
        case 'd': return "a decimal number";
        case 'h': return "hex bytes";
        case 'b': return "base64";
        default:  return "a value";
    }
}

tcmd_lint_severity_t tcmd_lint_command(const char *cmd, char *msg, size_t msg_cap)
{
    size_t len = 0;
    tcmd_lint_severity_t worst = TCMD_LINT_OK;
    if (msg && msg_cap) msg[0] = '\0';

    size_t n = strlen(cmd);
    if (n == 0) return TCMD_LINT_OK;   // caller skips blank lines

    if (n >= TCMD_MAX_FULL_LEN) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR,
             "exceeds firmware max telecommand length (255 chars)");
    } else if (n > TCMD_RF_MAX_LEN) {
        // The firmware would parse it over the umbilical, but it can't be
        // framed for the radio (RS block capacity). Warn rather than block
        // so an umbilical-only agenda still lints clean.
        char m[120];
        snprintf(m, sizeof m,
                 "%zu chars: over the %d-char RF uplink limit, "
                 "won't fit one radio frame (umbilical only)",
                 n, TCMD_RF_MAX_LEN);
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_WARN, m);
    }

    // Prefix.
    if (n <= TCMD_PREFIX_LEN || strncmp(cmd, TCMD_PREFIX, TCMD_PREFIX_LEN) != 0) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR, "missing 'CTS1+' prefix");
        return worst;   // nothing else is meaningful without the prefix
    }

    // Terminating '!': exactly one, anywhere after the prefix (the firmware
    // rejects zero, or more than one -- the latter guards against two
    // commands smashed onto one line).
    int bang_count = 0;
    for (size_t i = TCMD_PREFIX_LEN; i < n; ++i) {
        if (cmd[i] == '!') bang_count++;
    }
    if (bang_count == 0) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR, "missing terminating '!'");
    } else if (bang_count > 1) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR,
             "more than one '!' (two commands on one line?)");
    }

    // Command name: from after the prefix up to the first non-name char.
    size_t i = TCMD_PREFIX_LEN;
    while (i < n && is_name_char(cmd[i])) i++;
    size_t name_len = i - TCMD_PREFIX_LEN;
    if (name_len == 0) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR,
             "missing telecommand name after 'CTS1+'");
        return worst;
    }
    const tcmd_spec_t *spec = tcmd_spec_find(cmd + TCMD_PREFIX_LEN, name_len);
    if (!spec) {
        char m[96];
        snprintf(m, sizeof m, "unknown telecommand '%.*s'", (int) name_len,
                 cmd + TCMD_PREFIX_LEN);
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR, m);
    }

    // Opening paren must immediately follow the name; a closing paren must
    // exist (the firmware takes the first ')').
    if (i >= n || cmd[i] != '(') {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR,
             "missing '(' after telecommand name");
        return worst;
    }
    const char *close = strchr(cmd + i, ')');
    if (!close) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR,
             "missing ')' to close the argument list");
        return worst;
    }
    size_t open_idx  = i;
    size_t close_idx = (size_t) (close - cmd);
    size_t arg_len   = close_idx - open_idx - 1;   // chars between the parens

    if (arg_len >= TCMD_MAX_ARGS_LEN) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR,
             "argument string exceeds firmware limit (240 chars)");
    }

    // Argument count, exactly as the firmware computes it: 0 args when the
    // parens are empty, otherwise (commas + 1).
    if (spec) {
        int num_commas = 0;
        for (size_t j = open_idx + 1; j < close_idx; ++j) {
            if (cmd[j] == ',') num_commas++;
        }
        int provided = (arg_len == 0) ? 0 : (num_commas + 1);
        if (provided != spec->num_args) {
            char m[96];
            snprintf(m, sizeof m, "'%s' expects %d arg%s, got %d",
                     spec->name, spec->num_args,
                     spec->num_args == 1 ? "" : "s", provided);
            flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR, m);
        }

        // A command not meant for routine flight operation: warn, don't block.
        if (spec->readiness != TCMD_READY_OPERATION) {
            char m[128];
            snprintf(m, sizeof m,
                     "readiness '%s' -- not for routine flight operation",
                     tcmd_readiness_label(spec->readiness));
            flag(msg, msg_cap, &len, &worst, TCMD_LINT_WARN, m);
        }

        // Per-argument format check. Only meaningful when the arg COUNT is
        // right (otherwise token-to-type alignment is off, and the count error
        // above already covers it) and we have a verified arg_types of the
        // matching length (fail open if a future spec regen dropped it). Each
        // comma-delimited token is checked against the type the firmware will
        // parse it as, rejecting exactly what the satellite's parser would --
        // e.g. a space after a comma in a numeric arg -- so a doomed command
        // never costs an uplink. Tokens are split on ',' with no trimming,
        // matching the firmware; an arg value containing a literal ',' would
        // mis-split here just as it does on the satellite.
        if (provided == spec->num_args && spec->num_args > 0
            && spec->arg_types
            && (int) strlen(spec->arg_types) == spec->num_args) {
            size_t tok_start = open_idx + 1;
            int ai = 0;
            for (size_t j = open_idx + 1; j <= close_idx && ai < spec->num_args; ++j) {
                if (j == close_idx || cmd[j] == ',') {
                    char tc = spec->arg_types[ai];
                    const char *tok = cmd + tok_start;
                    size_t tn = j - tok_start;
                    if (!arg_token_ok(tc, tok, tn)) {
                        // Echo the offending token (whitespace stays visible
                        // inside the quotes), truncated to keep msg bounded.
                        char tb[40];
                        size_t cn = (tn < sizeof tb - 4) ? tn : sizeof tb - 4;
                        memcpy(tb, tok, cn);
                        if (tn > cn) {
                            tb[cn] = tb[cn + 1] = tb[cn + 2] = '.';
                            tb[cn + 3] = '\0';
                        } else {
                            tb[cn] = '\0';
                        }
                        char m[160];
                        snprintf(m, sizeof m,
                                 "arg %d expects %s; the satellite would reject \"%s\"",
                                 ai, arg_type_expectation(tc), tb);
                        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR, m);
                    }
                    tok_start = j + 1;
                    ai++;
                }
            }
        }
    }

    // Suffix timestamp tags: if present, they must be well-formed digits.
    long long v;
    if (strstr(cmd, "@tssent=") && !agenda_parse_directive_ms(cmd, "@tssent=", &v)) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR,
             "malformed @tssent= (expected unix-ms digits)");
    }
    if (strstr(cmd, "@tsexec=") && !agenda_parse_directive_ms(cmd, "@tsexec=", &v)) {
        flag(msg, msg_cap, &len, &worst, TCMD_LINT_ERROR,
             "malformed @tsexec= (expected unix-ms digits)");
    }

    return worst;
}

int tcmd_lint_file(const char *path, FILE *out, int *warn_count)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(out, "tcmd lint: cannot open %s: %s\n", path, strerror(errno));
        if (warn_count) *warn_count = 0;
        return -1;
    }

    char line[4096];
    int lineno = 0, errors = 0, warns = 0;
    while (fgets(line, sizeof line, f) != NULL) {
        ++lineno;

        // Skip blank lines and whole-line comments (first non-blank is '#').
        const char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#') continue;

        // Strip an inline trailing comment, sharing the rule with the
        // transmit path so we lint exactly what would go on the air.
        size_t cmd_len;
        agenda_find_inline_comment(line, &cmd_len);

        char cmd[4096];
        size_t L = (cmd_len < sizeof cmd) ? cmd_len : sizeof cmd - 1;
        memcpy(cmd, line, L);
        cmd[L] = '\0';

        // Trim surrounding whitespace.
        char *s = cmd;
        while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
        size_t sl = strlen(s);
        while (sl > 0 && (s[sl-1] == ' ' || s[sl-1] == '\t'
                          || s[sl-1] == '\r' || s[sl-1] == '\n')) {
            s[--sl] = '\0';
        }
        if (*s == '\0') continue;

        char msg[512];
        tcmd_lint_severity_t sev = tcmd_lint_command(s, msg, sizeof msg);
        if (sev == TCMD_LINT_ERROR) {
            ++errors;
            fprintf(out, "%s:%d: error: %s\n", path, lineno, msg);
        } else if (sev == TCMD_LINT_WARN) {
            ++warns;
            fprintf(out, "%s:%d: warning: %s\n", path, lineno, msg);
        }
    }
    fclose(f);

    if (warn_count) *warn_count = warns;
    return errors;
}
