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
