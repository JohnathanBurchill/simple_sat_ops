/*

   Simple Satellite Operations  bulk_size.c

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

#include "bulk_size.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Read an unsigned decimal run at *s, advancing *s past it. Returns -1 if there
// are no digits or the value runs past what a file length could be.
static long scan_num(const char **s)
{
    const char *p = *s;
    if (*p < '0' || *p > '9') return -1;
    long v = 0;
    while (*p >= '0' && *p <= '9') {
        v = v * 10 + (*p - '0');
        if (v > BULK_SIZE_MAX_PLAUSIBLE) return -1;
        p++;
    }
    *s = p;
    return v;
}

// Copy a satellite file path ending at any of `stops` (or at whitespace, which
// ends the path in the unquoted "File: ..." form). Returns 1 on success.
static int scan_path(const char *s, const char *stops, char *out, size_t outn)
{
    size_t k = 0;
    while (s[k] != '\0' && (unsigned char) s[k] > ' ' && strchr(stops, s[k]) == NULL) {
        if (k + 1 >= outn) return 0;
        k++;
    }
    if (k == 0) return 0;
    memcpy(out, s, k);
    out[k] = '\0';
    return 1;
}

int bulk_parse_complete(const char *text, bulk_complete_t *out)
{
    if (text == NULL) return 0;
    const char *s = strstr(text, "Bulk downlink complete. ");
    if (s == NULL) return 0;
    s += strlen("Bulk downlink complete. ");

    bulk_complete_t rec = {0};
    rec.bytes = scan_num(&s);
    if (rec.bytes < 0) return 0;

    if (strncmp(s, " bytes (", 8) != 0) return 0;
    s += 8;
    rec.packets = scan_num(&s);
    if (rec.packets < 0) return 0;

    const char *tail = " packets) downlinked. File: ";
    if (strncmp(s, tail, strlen(tail)) != 0) return 0;
    s += strlen(tail);
    if (!scan_path(s, "", rec.path, sizeof rec.path)) return 0;

    *out = rec;
    return 1;
}

int bulk_parse_start(const char *text, bulk_start_t *out)
{
    if (text == NULL) return 0;

    // How the satellite logs a download it was asked to start:
    // args_str: 'path;start;count'.
    const char *s = strstr(text, "(bulk_downlink_start_blob) args_str: '");
    char sep = ';';
    if (s != NULL) {
        s += strlen("(bulk_downlink_start_blob) args_str: '");
    } else if ((s = strstr(text, "exec_blob_from_fs(")) != NULL) {
        // How the ground writes the same thing in the command it transmits:
        //   exec_blob_from_fs(blobs/bulk_downlink_start_v2.blob,0,path;start;count)
        // The blob's name carries a version, so match the stem and then step
        // over the two arguments that precede the semicolon-separated ones.
        s += strlen("exec_blob_from_fs(");
        const char *comma = strchr(s, ',');
        if (comma == NULL) return 0;
        if (strstr(s, "bulk_downlink_start") == NULL
            || strstr(s, "bulk_downlink_start") > comma) return 0;
        comma = strchr(comma + 1, ',');
        if (comma == NULL) return 0;
        s = comma + 1;
    } else {
        // The direct firmware command, should it ever be sent on its own.
        s = strstr(text, "comms_bulk_file_downlink_start(");
        if (s == NULL) return 0;
        s += strlen("comms_bulk_file_downlink_start(");
        sep = ',';
    }

    bulk_start_t rec = {0};
    char stops[2] = { sep, '\0' };
    if (!scan_path(s, stops, rec.path, sizeof rec.path)) return 0;
    s += strlen(rec.path);
    if (*s != sep) return 0;
    s++;

    rec.start = scan_num(&s);
    if (rec.start < 0 || *s != sep) return 0;
    s++;
    rec.count = scan_num(&s);
    if (rec.count < 0) return 0;

    *out = rec;
    return 1;
}

// The table's row for `path`, appending one if it is new. NULL if out of memory.
static bulk_size_entry_t *entry_for(bulk_size_table_t *t, const char *path)
{
    for (int i = 0; i < t->n; i++)
        if (strcmp(t->v[i].path, path) == 0) return &t->v[i];

    if (t->n == t->cap) {
        int ncap = t->cap ? t->cap * 2 : 8;
        bulk_size_entry_t *nv = (bulk_size_entry_t *) realloc(t->v, (size_t) ncap * sizeof *nv);
        if (nv == NULL) return NULL;
        t->v = nv;
        t->cap = ncap;
    }
    bulk_size_entry_t *e = &t->v[t->n++];
    memset(e, 0, sizeof *e);
    snprintf(e->path, sizeof e->path, "%s", path);
    e->pend_start = -1;
    return e;
}

void bulk_size_feed(bulk_size_table_t *t, const char *text, double ts_ms)
{
    bulk_start_t st = {0};
    if (bulk_parse_start(text, &st)) {
        bulk_size_entry_t *e = entry_for(t, st.path);
        if (e != NULL) {
            e->pend_start = st.start;
            e->pend_count = st.count;
            e->pend_ms = ts_ms;
        }
        return;
    }

    bulk_complete_t cp = {0};
    if (!bulk_parse_complete(text, &cp)) return;

    bulk_size_entry_t *e = entry_for(t, cp.path);
    if (e == NULL) return;

    // Pair with the start line for this file if one is recent enough to belong
    // to the same download; otherwise read the download as starting at 0, which
    // can only understate the length.
    long start = 0, count = 0;
    if (e->pend_start >= 0 && ts_ms - e->pend_ms <= BULK_SIZE_PAIR_WINDOW_MS
        && ts_ms >= e->pend_ms) {
        start = e->pend_start;
        count = e->pend_count;
    }

    long size = start + cp.bytes;
    if (size <= 0 || size > BULK_SIZE_MAX_PLAUSIBLE) return;

    // The download reached the end of the file only if it stopped before its
    // allowance ran out. Two caps apply: the one the ground asked for, and the
    // firmware's own, which binds even when the ground asked for the whole file.
    long cap = (count > 0 && count < BULK_SIZE_FIRMWARE_CAP) ? count : BULK_SIZE_FIRMWARE_CAP;
    int exact = (cp.bytes < cap);

    // An exact length settles the question; a lower bound only ever raises it.
    if (exact && !e->exact) {
        e->size = size;
        e->exact = 1;
    } else if (exact == e->exact && size > e->size) {
        e->size = size;
    }
}

long bulk_size_lookup(const bulk_size_table_t *t, const char *path, int *exact)
{
    if (exact != NULL) *exact = 0;
    if (path == NULL || path[0] == '\0') return 0;
    for (int i = 0; i < t->n; i++) {
        if (strcmp(t->v[i].path, path) != 0) continue;
        if (exact != NULL) *exact = t->v[i].exact;
        return t->v[i].size;
    }
    return 0;
}

void bulk_size_free(bulk_size_table_t *t)
{
    free(t->v);
    t->v = NULL;
    t->n = 0;
    t->cap = 0;
}

void bulk_grid_learn(const long *off, const unsigned char *verified, int n,
                     unsigned char grid[195])
{
    memset(grid, 0, 195);

    int nver = 0;
    for (int i = 0; i < n; i++) {
        if (!verified[i] || off[i] < 0) continue;
        grid[off[i] % 195] = 1;
        nver++;
    }
    // Too little verified data to know the download's grid: open it rather than
    // shut out offsets on the strength of a handful of samples.
    if (nver < BULK_GRID_MIN_VERIFIED) memset(grid, 1, 195);
}

int bulk_grid_ok(const unsigned char grid[195], long off)
{
    if (off < 0) return 0;
    return grid[off % 195] != 0;
}
