/*

    Simple Satellite Operations  utils/viewer_cache.h

    A cache file for the raylib viewers, so that each start need not put
    everything they show back together from the whole packet database.

    What goes into a cache is each viewer's own business; this module keeps
    the file honest. A cache belongs to one build of one viewer and one
    database: its header records the size and modification time of the
    viewer binary that wrote it and the database's full path, and a cache
    that does not match all three is not read. A new build therefore starts
    over with a full rebuild, which is what stops a cache from serving
    pictures put together by older code. Because the same binary reads what
    it wrote, the contents are the viewer's own structures as they sit in
    memory, followed by the arrays they point to.

    A cache is written to a temporary file beside it and renamed into place,
    so a viewer killed half way through writing leaves the old one whole.

    The policy the viewers share. Imagery whose moment (a picture's capture,
    a recording's start) is more than VIEWER_CACHE_SETTLED_MS old when the
    cache is written is kept in the cache, and is put together again from
    the database only on command (shift-F5, or --rebuild-cache). Everything
    newer is rebuilt at every start from the packets received since
    VIEWER_CACHE_MARGIN_MS before the cutoff -- the margin covers a
    satellite clock that is off -- together with every packet added to the
    database since the cache was written, whatever its time. When those
    packets turn out to belong to cached imagery (a re-download of an old
    recording, say), the viewer does not show them; it says that there are
    some, and a rebuild brings them in. Once the cutoff has fallen more than
    VIEWER_CACHE_ROLL_MS behind a week ago, a start moves it forward and
    writes the cache again, so that the part rebuilt at every start stays
    about a week long.

    Files live in $XDG_CACHE_HOME/simple_sat_ops/, else
    ~/.cache/simple_sat_ops/. Deleting one is always safe.

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

#ifndef VIEWER_CACHE_H
#define VIEWER_CACHE_H

#include <stddef.h>
#include <stdio.h>

#include <sqlite3.h>

#define VIEWER_CACHE_SETTLED_MS (7.0 * 86400000.0)
#define VIEWER_CACHE_MARGIN_MS  (1.0 * 86400000.0)
#define VIEWER_CACHE_ROLL_MS    (1.0 * 86400000.0)

typedef struct {
    // What was captured before this moment (unix ms) is in the cache. A
    // whole number of seconds.
    double    cutoff_ms;
    // The newest packet row in the database when the cache was written.
    long long max_id;
} viewer_cache_info_t;

// Open cache `name` for reading. Returns NULL -- and the caller does a full
// rebuild -- when there is none, when another build of the viewer or
// another database wrote it, when the database's newest row is older than
// the cache's (the database was replaced), or when the file is damaged.
// Otherwise fills *info and leaves the file positioned at the contents.
FILE *viewer_cache_open(const char *name, const char *db_path,
                        long long db_max_id, viewer_cache_info_t *info);

// Read n bytes. Returns 0, or -1 if the file ran out first.
int viewer_cache_read(FILE *f, void *p, size_t n);

// Read n bytes into a fresh malloc'd buffer (never NULL for n = 0). Returns
// NULL if the file ran out or memory did.
void *viewer_cache_read_alloc(FILE *f, size_t n);

typedef struct {
    FILE *f;
    int   failed;
    char  path[1024];
    char  tmp[1100];
} viewer_cache_writer_t;

// Start writing cache `name`: the header now, the contents with
// viewer_cache_write, then viewer_cache_commit. Returns 0, or -1 if the
// file could not be created.
int viewer_cache_create(viewer_cache_writer_t *w, const char *name,
                        const char *db_path, const viewer_cache_info_t *info);

// Append n bytes. A failure is remembered and reported by the commit.
void viewer_cache_write(viewer_cache_writer_t *w, const void *p, size_t n);

// Close the file and rename it into place. Returns 0, or -1 (the old
// cache, if any, is left as it was) if anything went wrong on the way.
int viewer_cache_commit(viewer_cache_writer_t *w);

// The database's newest packet row id, -1 if it cannot be read.
long long viewer_cache_db_max_id(sqlite3 *db);

// The time now, unix ms, to the whole second.
double viewer_cache_now_ms(void);

// unix ms as the packet table writes ts_received, "2026-10-19T17:46:49.000Z",
// to compare against it in SQL. Rounds down to the second.
void viewer_cache_iso(double ms, char *out, size_t n);

#endif // VIEWER_CACHE_H
