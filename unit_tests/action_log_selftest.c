/*

    Simple Satellite Operations  unit_tests/action_log_selftest.c

    Coverage for src/ipc/sso_action_log.c — the per-pass actions.log.
    Pins the line format, that lines logged before the pass folder exists
    are held and written out first (in order) once it does, that newlines
    in a message can't split a line, that pointing at the same folder twice
    doesn't re-open or duplicate, and that moving to a new folder sends
    later lines there only.

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

#include "sso_action_log.h"
#include "tap.h"

#include <regex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Read <dir>/actions.log into buf; returns the number of lines, -1 if absent.
static int read_log(const char *dir, char *buf, size_t cap)
{
    char path[600];
    snprintf(path, sizeof path, "%s/actions.log", dir);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    size_t n = fread(buf, 1, cap - 1, f);
    fclose(f);
    buf[n] = '\0';
    int lines = 0;
    for (size_t i = 0; i < n; ++i) if (buf[i] == '\n') ++lines;
    return lines;
}

int main(void)
{
    char tmpl_a[] = "/tmp/sso_action_log_a_XXXXXX";
    char tmpl_b[] = "/tmp/sso_action_log_b_XXXXXX";
    const char *dir_a = mkdtemp(tmpl_a);
    const char *dir_b = mkdtemp(tmpl_b);
    if (!dir_a || !dir_b) {
        tap_bail("mkdtemp failed");
        return 1;
    }
    char buf[8192];

    // --- Before any folder: held, nothing on disk ----------------------
    sso_action_log("app", "start user=%s", "tester");
    sso_action_log("rot", "first\nsecond");
    tap_ok(read_log(dir_a, buf, sizeof buf) == -1,
           "no file before sso_action_log_set_dir");

    // --- Folder set: held lines flushed in order -----------------------
    tap_ok(sso_action_log_set_dir(dir_a) == 0, "set_dir opens actions.log");
    sso_action_log("ui", "key 'T': Track satellite");
    int lines = read_log(dir_a, buf, sizeof buf);
    tap_okf(lines == 3, "held lines + new line written (got %d lines)", lines);
    const char *p_start = strstr(buf, "start user=tester");
    const char *p_rot   = strstr(buf, "first second");
    const char *p_key   = strstr(buf, "key 'T': Track satellite");
    tap_ok(p_start && p_rot && p_key && p_start < p_rot && p_rot < p_key,
           "held lines come first, in logged order");
    tap_ok(p_rot != NULL, "newline in message flattened to a space");

    // --- Line format ---------------------------------------------------
    regex_t re;
    regcomp(&re,
            "^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}\\.[0-9]{3}Z "
            "\\[ui\\] +key 'T': Track satellite$",
            REG_EXTENDED | REG_NEWLINE);
    tap_ok(regexec(&re, buf, 0, NULL, 0) == 0,
           "line is '<ISO UTC ms> [cat] message'");
    regfree(&re);

    // --- Same folder again: no reopen, no duplicate flush --------------
    tap_ok(sso_action_log_set_dir(dir_a) == 0, "set_dir same folder is ok");
    lines = read_log(dir_a, buf, sizeof buf);
    tap_okf(lines == 3, "same folder adds nothing (got %d lines)", lines);

    // --- New folder: later lines go there only -------------------------
    tap_ok(sso_action_log_set_dir(dir_b) == 0, "set_dir moves to a new folder");
    sso_action_log("pass", "moved");
    lines = read_log(dir_a, buf, sizeof buf);
    tap_okf(lines == 3, "old folder untouched after move (got %d lines)", lines);
    lines = read_log(dir_b, buf, sizeof buf);
    tap_okf(lines == 1 && strstr(buf, "[pass]  moved"),
            "new folder gets the later line (got %d lines)", lines);

    // --- Close: back to holding ----------------------------------------
    sso_action_log_close();
    sso_action_log("app", "after close");
    lines = read_log(dir_b, buf, sizeof buf);
    tap_okf(lines == 1, "nothing written after close (got %d lines)", lines);

    // --- Unwritable folder fails cleanly -------------------------------
    tap_ok(sso_action_log_set_dir("/nonexistent/sso_action_log") == -1,
           "set_dir on a missing folder returns -1");
    tap_ok(sso_action_log_set_dir("") == -1, "set_dir on \"\" returns -1");

    char path[600];
    snprintf(path, sizeof path, "%s/actions.log", dir_a); unlink(path);
    snprintf(path, sizeof path, "%s/actions.log", dir_b); unlink(path);
    rmdir(dir_a);
    rmdir(dir_b);
    return tap_done();
}
