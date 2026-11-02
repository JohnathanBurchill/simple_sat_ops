/*

    Simple Satellite Operations  utils/viewer_cache.c

    The cache file the raylib viewers share. See viewer_cache.h.

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

#include "viewer_cache.h"

#include "sso_paths.h"

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#define CACHE_MAGIC  "SSO viewer cache"
#define CACHE_FORMAT 1

// The header, written as it sits in memory. It is zeroed before it is filled
// in, so the padding between its fields is the same every time and the whole
// thing can be compared byte for byte.
typedef struct {
    char      magic[sizeof CACHE_MAGIC];
    int       format;
    char      name[64];
    long long exe_size;
    long long exe_mtime_s;
    long long exe_mtime_ns;
    char      db_path[1024];
    viewer_cache_info_t info;
} header_t;

static int cache_path(const char *name, char *out, size_t n)
{
    const char *xdg = getenv("XDG_CACHE_HOME");
    const char *home = getenv("HOME");
    int k;
    if (xdg != NULL && xdg[0] != '\0')
        k = snprintf(out, n, "%s/simple_sat_ops/%s.cache", xdg, name);
    else if (home != NULL && home[0] != '\0')
        k = snprintf(out, n, "%s/.cache/simple_sat_ops/%s.cache", home, name);
    else
        return -1;
    return (k < 0 || (size_t) k >= n) ? -1 : 0;
}

// The running binary's size and modification time: what tells this build's
// cache from another's. A rebuild or a reinstall changes the time.
static int exe_identity(header_t *h)
{
    char p[PATH_MAX];
#ifdef __APPLE__
    uint32_t n = sizeof p;
    if (_NSGetExecutablePath(p, &n) != 0) return -1;
#else
    ssize_t n = readlink("/proc/self/exe", p, sizeof p - 1);
    if (n <= 0) return -1;
    p[n] = '\0';
#endif
    struct stat st;
    if (stat(p, &st) != 0) return -1;
    h->exe_size = (long long) st.st_size;
#ifdef __APPLE__
    h->exe_mtime_s = (long long) st.st_mtimespec.tv_sec;
    h->exe_mtime_ns = (long long) st.st_mtimespec.tv_nsec;
#else
    h->exe_mtime_s = (long long) st.st_mtim.tv_sec;
    h->exe_mtime_ns = (long long) st.st_mtim.tv_nsec;
#endif
    return 0;
}

// The header a cache written now, by this binary, for this database would
// carry. The database is named by its full path, so --db=./x and --db=x are
// the same store.
static int expected_header(header_t *h, const char *name, const char *db_path,
                           const viewer_cache_info_t *info)
{
    memset(h, 0, sizeof *h);
    memcpy(h->magic, CACHE_MAGIC, sizeof CACHE_MAGIC);
    h->format = CACHE_FORMAT;
    if (strlen(name) >= sizeof h->name) return -1;
    snprintf(h->name, sizeof h->name, "%s", name);
    if (exe_identity(h) != 0) return -1;
    char full[PATH_MAX];
    const char *p = realpath(db_path, full) != NULL ? full : db_path;
    if (strlen(p) >= sizeof h->db_path) return -1;
    snprintf(h->db_path, sizeof h->db_path, "%s", p);
    if (info != NULL) h->info = *info;
    return 0;
}

FILE *viewer_cache_open(const char *name, const char *db_path,
                        long long db_max_id, viewer_cache_info_t *info)
{
    char path[1024];
    header_t want, got;
    if (cache_path(name, path, sizeof path) != 0) return NULL;
    if (expected_header(&want, name, db_path, NULL) != 0) return NULL;

    FILE *f = fopen(path, "rb");
    if (f == NULL) return NULL;
    if (fread(&got, sizeof got, 1, f) != 1) { fclose(f); return NULL; }
    // Everything but the contents' own description has to match.
    want.info = got.info;
    if (memcmp(&want, &got, sizeof want) != 0) { fclose(f); return NULL; }
    // A database whose newest row is older than the cache is not the one the
    // cache was written from, whatever its name.
    if (db_max_id < got.info.max_id) { fclose(f); return NULL; }
    *info = got.info;
    return f;
}

int viewer_cache_read(FILE *f, void *p, size_t n)
{
    if (n == 0) return 0;
    return fread(p, n, 1, f) == 1 ? 0 : -1;
}

void *viewer_cache_read_alloc(FILE *f, size_t n)
{
    void *p = malloc(n > 0 ? n : 1);
    if (p == NULL) return NULL;
    if (viewer_cache_read(f, p, n) != 0) { free(p); return NULL; }
    return p;
}

int viewer_cache_create(viewer_cache_writer_t *w, const char *name,
                        const char *db_path, const viewer_cache_info_t *info)
{
    memset(w, 0, sizeof *w);
    header_t h;
    if (cache_path(name, w->path, sizeof w->path) != 0) return -1;
    if (expected_header(&h, name, db_path, info) != 0) return -1;
    if (sso_mkdir_p_for_file(w->path) != 0) return -1;
    // Two copies of a viewer writing at once each get their own temporary
    // file; the later rename wins.
    snprintf(w->tmp, sizeof w->tmp, "%s.%ld.tmp", w->path, (long) getpid());
    w->f = fopen(w->tmp, "wb");
    if (w->f == NULL) return -1;
    viewer_cache_write(w, &h, sizeof h);
    return 0;
}

void viewer_cache_write(viewer_cache_writer_t *w, const void *p, size_t n)
{
    if (w->f == NULL || w->failed || n == 0) return;
    if (fwrite(p, n, 1, w->f) != 1) w->failed = 1;
}

int viewer_cache_commit(viewer_cache_writer_t *w)
{
    if (w->f == NULL) return -1;
    if (fclose(w->f) != 0) w->failed = 1;
    w->f = NULL;
    if (!w->failed && rename(w->tmp, w->path) == 0) return 0;
    remove(w->tmp);
    return -1;
}

long long viewer_cache_db_max_id(sqlite3 *db)
{
    sqlite3_stmt *st = NULL;
    long long id = -1;
    if (sqlite3_prepare_v2(db, "SELECT max(id) FROM packet", -1, &st, NULL) != SQLITE_OK)
        return -1;
    if (sqlite3_step(st) == SQLITE_ROW) id = sqlite3_column_int64(st, 0);
    sqlite3_finalize(st);
    return id;
}

double viewer_cache_now_ms(void)
{
    return (double) time(NULL) * 1000.0;
}

void viewer_cache_iso(double ms, char *out, size_t n)
{
    time_t secs = (time_t) (ms / 1000.0);
    struct tm tmv;
    gmtime_r(&secs, &tmv);
    char s[24];
    strftime(s, sizeof s, "%Y-%m-%dT%H:%M:%S", &tmv);
    snprintf(out, n, "%s.000Z", s);
}
