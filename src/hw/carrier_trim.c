/*

    Simple Satellite Operations  src/hw/carrier_trim.c

    See carrier_trim.h for the data-file contract.

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

#include "carrier_trim.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define TRIM_PATH_MAX 512

static char        g_trim_path[TRIM_PATH_MAX];
static int         g_trim_path_resolved = 0;

static void resolve_trim_path(void)
{
    if (g_trim_path_resolved) return;
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') home = "/tmp";
    int n = snprintf(g_trim_path, sizeof g_trim_path,
                     "%s/.local/share/simple_sat_ops/carrier-trim-hz", home);
    if (n < 0 || (size_t)n >= sizeof g_trim_path) {
        // Truncated — leave the buffer as-is so the caller's stderr
        // log at least shows the prefix; load_hz will fall back to 0.
        g_trim_path[sizeof g_trim_path - 1] = '\0';
    }
    g_trim_path_resolved = 1;
}

const char *carrier_trim_path(void)
{
    resolve_trim_path();
    return g_trim_path;
}

// Create the parent directory if it doesn't already exist. Best
// effort; on failure we still try to open the file (and that open
// will surface the underlying errno).
static void ensure_parent_dir(const char *path)
{
    char dir[TRIM_PATH_MAX];
    snprintf(dir, sizeof dir, "%s", path);
    char *slash = strrchr(dir, '/');
    if (slash == NULL) return;
    *slash = '\0';
    if (dir[0] == '\0') return;
    struct stat st;
    if (stat(dir, &st) == 0 && S_ISDIR(st.st_mode)) return;
    (void)mkdir(dir, 0755);
}

double carrier_trim_load_hz(void)
{
    resolve_trim_path();
    const char *path = g_trim_path;

    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        if (errno != ENOENT) {
            fprintf(stderr,
                    "carrier_trim: open %s failed: %s — assuming 0 Hz\n",
                    path, strerror(errno));
            return 0.0;
        }
        // Missing — create it with 0 so the operator sees the file
        // and knows where the knob is.
        ensure_parent_dir(path);
        FILE *wfp = fopen(path, "w");
        if (wfp == NULL) {
            fprintf(stderr,
                    "carrier_trim: create %s failed: %s — assuming 0 Hz\n",
                    path, strerror(errno));
            return 0.0;
        }
        fputs("0\n", wfp);
        fclose(wfp);
        fprintf(stderr, "carrier_trim: created %s with 0 Hz\n", path);
        return 0.0;
    }

    char line[64] = {0};
    if (fgets(line, sizeof line, fp) == NULL) {
        fclose(fp);
        fprintf(stderr,
                "carrier_trim: %s is empty — assuming 0 Hz\n", path);
        return 0.0;
    }
    fclose(fp);

    char *end = NULL;
    double hz = strtod(line, &end);
    if (end == line) {
        fprintf(stderr,
                "carrier_trim: %s did not parse as a number "
                "(got %.*s...) — assuming 0 Hz\n",
                path, (int)(strnlen(line, 32)), line);
        return 0.0;
    }
    if (hz != 0.0) {
        fprintf(stderr,
                "carrier_trim: loaded %.3f Hz from %s\n", hz, path);
    }
    return hz;
}
