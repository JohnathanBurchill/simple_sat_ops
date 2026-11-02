/*

    Simple Satellite Operations  unit_tests/viewer_cache_selftest.c

    Exercises utils/viewer_cache.c, the cache file the raylib viewers keep.
    What matters most is what it refuses: a cache written for another
    database, one the database has outgrown the other way (it was replaced
    by an older one), and a damaged file must all read as no cache at all,
    because serving one of those would show pictures that are not in the
    database without a word. A write that never finishes must leave the old
    cache readable. The one refusal not covered is a cache from another
    build of the viewer, which would take a second binary.

    The ISO times are checked against the system's own `date -u -r`.

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

#include "tap.h"
#include "viewer_cache.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static char g_dir[512];

static void touch(const char *path)
{
    FILE *f = fopen(path, "w");
    if (f != NULL) fclose(f);
}

static int write_one(const char *name, const char *db, double cutoff, long long max_id,
                     const char *payload)
{
    viewer_cache_info_t info = { .cutoff_ms = cutoff, .max_id = max_id };
    viewer_cache_writer_t w;
    if (viewer_cache_create(&w, name, db, &info) != 0) return -1;
    viewer_cache_write(&w, payload, strlen(payload) + 1);
    return viewer_cache_commit(&w);
}

// The payload an open cache holds, or "" if it cannot be read.
static void read_payload(FILE *f, char *out, size_t n)
{
    out[0] = '\0';
    size_t k = 0;
    while (k + 1 < n && viewer_cache_read(f, out + k, 1) == 0 && out[k] != '\0') k++;
    out[k] = '\0';
}

int main(void)
{
    const char *tmp = getenv("TMPDIR");
    snprintf(g_dir, sizeof g_dir, "%s/viewer_cache_selftest.XXXXXX",
             tmp != NULL && tmp[0] != '\0' ? tmp : "/tmp");
    if (mkdtemp(g_dir) == NULL) { tap_bail("cannot make a scratch directory"); return 1; }
    setenv("XDG_CACHE_HOME", g_dir, 1);

    char db_a[600], db_a_alias[620], db_b[600], cache_file[700];
    snprintf(db_a, sizeof db_a, "%s/a.sqlite", g_dir);
    snprintf(db_a_alias, sizeof db_a_alias, "%s/./a.sqlite", g_dir);
    snprintf(db_b, sizeof db_b, "%s/b.sqlite", g_dir);
    snprintf(cache_file, sizeof cache_file, "%s/simple_sat_ops/t.cache", g_dir);
    touch(db_a);
    touch(db_b);

    viewer_cache_info_t info = {0};
    char got[64];

    tap_ok(write_one("t", db_a, 1760000000000.0, 500, "first") == 0,
           "a cache is written");
    FILE *f = viewer_cache_open("t", db_a, 500, &info);
    tap_ok(f != NULL, "it opens for the database it was written for");
    if (f != NULL) {
        tap_okf(info.cutoff_ms == 1760000000000.0 && info.max_id == 500,
                "the cutoff and newest row come back (%.0f, %lld)", info.cutoff_ms, info.max_id);
        read_payload(f, got, sizeof got);
        tap_okf(strcmp(got, "first") == 0, "the contents come back (\"%s\")", got);
        char extra;
        tap_ok(viewer_cache_read(f, &extra, 1) == -1, "reading past the end fails");
        fclose(f);
    }

    f = viewer_cache_open("t", db_a_alias, 500, &info);
    tap_ok(f != NULL, "another spelling of the same database's path still matches");
    if (f != NULL) fclose(f);

    f = viewer_cache_open("t", db_b, 500, &info);
    tap_ok(f == NULL, "another database's cache is refused");
    if (f != NULL) fclose(f);

    f = viewer_cache_open("t", db_a, 900, &info);
    tap_ok(f != NULL, "a database that has grown since keeps its cache");
    if (f != NULL) fclose(f);

    f = viewer_cache_open("t", db_a, 499, &info);
    tap_ok(f == NULL, "a database older than its cache (replaced) is refused");
    if (f != NULL) fclose(f);

    f = viewer_cache_open("other", db_a, 500, &info);
    tap_ok(f == NULL, "a name never written has no cache");
    if (f != NULL) fclose(f);

    // A writer that dies before its commit: the temporary file goes, and the
    // cache in place is the old one.
    viewer_cache_info_t later = { .cutoff_ms = 1784756948000.0, .max_id = 800 };
    viewer_cache_writer_t w;
    tap_ok(viewer_cache_create(&w, "t", db_a, &later) == 0, "a second write starts");
    viewer_cache_write(&w, "second", 7);
    fclose(w.f);
    remove(w.tmp);
    f = viewer_cache_open("t", db_a, 800, &info);
    tap_ok(f != NULL && info.max_id == 500, "an unfinished write leaves the old cache in place");
    if (f != NULL) {
        read_payload(f, got, sizeof got);
        tap_okf(strcmp(got, "first") == 0, "with its old contents (\"%s\")", got);
        fclose(f);
    }

    tap_ok(write_one("t", db_a, 1784756948000.0, 800, "second") == 0, "a finished write");
    f = viewer_cache_open("t", db_a, 800, &info);
    if (f != NULL) read_payload(f, got, sizeof got);
    tap_ok(f != NULL && info.max_id == 800 && strcmp(got, "second") == 0,
           "replaces the cache");
    if (f != NULL) fclose(f);

    // Damage.
    f = fopen(cache_file, "r+b");
    if (f != NULL) { fputc('X', f); fclose(f); }
    f = viewer_cache_open("t", db_a, 800, &info);
    tap_ok(f == NULL, "a cache whose first byte is wrong is refused");
    if (f != NULL) fclose(f);

    tap_ok(write_one("t", db_a, 1784756948000.0, 800, "third") == 0, "rewritten");
    tap_ok(truncate(cache_file, 40) == 0, "cut short inside its header");
    f = viewer_cache_open("t", db_a, 800, &info);
    tap_ok(f == NULL, "a cache cut short is refused");
    if (f != NULL) fclose(f);

    // The newest row of a real packet table, and of a database without one.
    sqlite3 *db = NULL;
    sqlite3_open(db_a, &db);
    tap_ok(viewer_cache_db_max_id(db) == -1, "no packet table reads as -1");
    sqlite3_exec(db, "CREATE TABLE packet (id INTEGER PRIMARY KEY, ts_received TEXT);"
                     "INSERT INTO packet (id, ts_received) VALUES (7, 'x'), (41, 'y'), (12, 'z');",
                 NULL, NULL, NULL);
    tap_okf(viewer_cache_db_max_id(db) == 41, "the newest row of a packet table (%lld)",
            viewer_cache_db_max_id(db));
    sqlite3_close(db);

    // As `date -u -r 1760000000` and `date -u -r 1784756948` print them, with
    // the milliseconds dropped.
    char iso[32];
    viewer_cache_iso(1760000000000.0, iso, sizeof iso);
    tap_okf(strcmp(iso, "2025-10-09T08:53:20.000Z") == 0, "ISO time %s", iso);
    viewer_cache_iso(1784756948999.0, iso, sizeof iso);
    tap_okf(strcmp(iso, "2026-07-22T21:49:08.000Z") == 0, "ISO time rounds down: %s", iso);

    remove(cache_file);
    remove(db_a);
    remove(db_b);
    char sub[600];
    snprintf(sub, sizeof sub, "%s/simple_sat_ops", g_dir);
    rmdir(sub);
    rmdir(g_dir);
    return tap_done();
}
