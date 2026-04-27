/*

   Simple Satellite Operations  radio_device_store.c

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "radio_device_store.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define STORE_RELPATH       "/.local/share/simple_sat_ops/radio_device"
#define STORE_SPEED_RELPATH "/.local/share/simple_sat_ops/radio_serial_speed"
#define MAX_PATH_LEN        1024

int radio_device_store_path(char *out, size_t cap)
{
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') return -1;
    int n = snprintf(out, cap, "%s%s", home, STORE_RELPATH);
    if (n < 0 || (size_t)n >= cap) return -1;
    return 0;
}

// Create every parent directory of `path` (the file's directory and its
// ancestors). 0755 throughout. Existing directories are tolerated.
static int mkdir_p_for_file(const char *path)
{
    char buf[MAX_PATH_LEN];
    size_t len = strlen(path);
    if (len >= sizeof buf) { errno = ENAMETOOLONG; return -1; }
    memcpy(buf, path, len + 1);

    char *last_slash = strrchr(buf, '/');
    if (last_slash == NULL || last_slash == buf) return 0;  // root or no parent
    *last_slash = '\0';

    // Walk from the front. mkdir each prefix; tolerate EEXIST.
    for (char *p = buf + 1; *p != '\0'; ++p) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(buf, 0755) != 0 && errno != EEXIST) return -1;
            *p = '/';
        }
    }
    if (mkdir(buf, 0755) != 0 && errno != EEXIST) return -1;
    return 0;
}

int radio_device_store_load(char *out, size_t cap)
{
    char path[MAX_PATH_LEN];
    if (radio_device_store_path(path, sizeof path) < 0) return -2;
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return (errno == ENOENT) ? -1 : -2;
    }
    char line[MAX_PATH_LEN];
    char *got = fgets(line, sizeof line, f);
    fclose(f);
    if (got == NULL) return -1;

    // Trim trailing newline / CR / whitespace
    size_t len = strlen(line);
    while (len > 0) {
        char c = line[len - 1];
        if (c == '\n' || c == '\r' || c == ' ' || c == '\t') {
            line[--len] = '\0';
        } else {
            break;
        }
    }
    if (len == 0) return -1;
    if (len >= cap) return -2;
    memcpy(out, line, len + 1);
    return 0;
}

int radio_device_store_save(const char *path)
{
    char store_path[MAX_PATH_LEN];
    if (radio_device_store_path(store_path, sizeof store_path) < 0) return -1;
    if (mkdir_p_for_file(store_path) < 0) return -1;

    FILE *f = fopen(store_path, "w");
    if (f == NULL) return -1;
    int n = fprintf(f, "%s\n", path);
    int closed = fclose(f);
    if (n < 0 || closed != 0) return -1;
    return 0;
}

static int speed_store_path(char *out, size_t cap)
{
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') return -1;
    int n = snprintf(out, cap, "%s%s", home, STORE_SPEED_RELPATH);
    if (n < 0 || (size_t)n >= cap) return -1;
    return 0;
}

int radio_device_store_load_speed(int *out_bps)
{
    if (out_bps == NULL) return -2;
    char path[MAX_PATH_LEN];
    if (speed_store_path(path, sizeof path) < 0) return -2;
    FILE *f = fopen(path, "r");
    if (f == NULL) return (errno == ENOENT) ? -1 : -2;
    int v = 0;
    int got = fscanf(f, "%d", &v);
    fclose(f);
    if (got != 1 || v <= 0) return -1;
    *out_bps = v;
    return 0;
}

int radio_device_store_save_speed(int bps)
{
    char path[MAX_PATH_LEN];
    if (speed_store_path(path, sizeof path) < 0) return -1;
    if (mkdir_p_for_file(path) < 0) return -1;
    FILE *f = fopen(path, "w");
    if (f == NULL) return -1;
    int n = fprintf(f, "%d\n", bps);
    int closed = fclose(f);
    if (n < 0 || closed != 0) return -1;
    return 0;
}
