/*

    Simple Satellite Operations  utils/satnogs_browser.c

    Curses TUI over the SatNOGS observation archive: one UTC day at a
    time, showing what SatNOGS holds for the satellite against what this
    ground station has already pulled down, with a way to mark passes and
    fetch them.

    It never talks to the network itself. Every request goes through
    satnogs_pull.sh, which owns the archive lock, the polite delay and
    the rolling-hour request tally -- one path to the API is worth more
    than a tidy program, given that a runaway client got this station's
    address blocked in August 2026. The browser reads the day caches that
    script writes and spawns it to do anything else.

    That split is also what makes browsing cheap. SatNOGS throttles the
    observations endpoint on its `list` action alone, so a day costs
    about fifty requests to list once and nothing at all thereafter,
    however often it is revisited; fetching audio is off the meter
    entirely, because it comes from object storage.

    Layout:

      +-- day, counts and request budget (top reverse-video bar) -------+
      +-- column headings ----------------------------------------------+
      +-- one line per observation, scrolling -------------------------+
      |                                                                 |
      +-- separator ----------------------------------------------------+
      +-- selected observation, or a running job's output --------------+
      |                                                                 |
      +-- key hints (bottom reverse-video bar) -------------------------+

    Keys (vi motions alongside the arrows):
      q | Esc          quit (with a job running, ask first)
      j | k | Up|Down  move the selection
      PgUp / PgDn      move by a page
      gg | Home        first observation
      G | End          last observation
      zz               centre the selection
      h | l | Left|Rt  previous / next day
      t                jump to today
      /                go to a date (YYYY-MM-DD)
      r                list this day from SatNOGS (or re-list it)
      Enter            write a note against the selected observation
      space            mark / unmark the selected observation
      a                mark every observation in view with a recording
      D                mark the ones that carried data
      n                clear all marks
      f | F            cycle the filter forwards / backwards
      d                download the marked observations
      p                decode the marked observations into the packet DB
      c                cancel the running job
      ?                this help

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

// strptime and timegm, which the day arithmetic and the pass-length
// column both need, are outside the C99 standard set on glibc.
#define _GNU_SOURCE

#include "argparse.h"
#include "packet_db.h"
#include "sso_paths.h"
#include "sso_version.h"
#include "ui_textfield.h"

#include <ctype.h>
#include <dirent.h>
#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#if !defined(SATNOGS_BROWSER_HAVE_NCURSES)
int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    fprintf(stderr,
            "satnogs_browser: built without ncurses. Install\n"
            "libncurses-dev and rebuild.\n");
    return 1;
}
#else

#include <ncurses.h>
#include <sqlite3.h>

// A day of one satellite on a busy network runs to a couple of thousand
// observations; this is a generous ceiling on that, and a day that
// somehow exceeded it is truncated with a note rather than refused.
#define MAX_OBS_ROWS   8192
// Ids handed to satnogs_pull.sh in one go. The API returns 25 per page
// whatever we ask for, so this only bounds the query string's length.
#define IDS_PER_BATCH  100
#define JOB_LOG_LINES  200
#define JOB_LINE_LEN   256
// Ceiling on a stored observation record. A pass with a few hundred
// demodulated frames is a few tens of kilobytes; anything past this is
// not the file we think it is, so it is left alone.
#define META_MAX_BYTES (4L * 1024 * 1024)
// A note is one line about one pass, not a log entry.
#define NOTE_MAX       200

enum {
    PAIR_BAR = 1,
    PAIR_HAVE,      // downloaded, nothing decoded from it yet
    PAIR_DECODED,   // downloaded and in the packet database
    PAIR_NOAUDIO,   // SatNOGS holds no recording
    PAIR_MARKED,
    PAIR_SEL,
    PAIR_WARN,
};

typedef struct {
    long   id;
    char   start[32];
    char   end[32];
    char   status[16];
    char   waterfall[16];
    long   station_id;
    char   station[64];
    double max_el;
    int    has_audio;    // SatNOGS holds a recording
    int    downloaded;   // this station already has the audio
    int    decoded;      // and the packet DB has rows from it
    // Frames SatNOGS demodulated from the pass itself, or -1 when
    // nothing here knows. Dozens of them is what a bulk download of
    // stored science data looks like from the outside.
    int    n_data;
    int    marked;
} obs_t;

// Filters, in the order `f` cycles them.
enum { FLT_ALL = 0, FLT_AUDIO, FLT_MISSING, FLT_HAVE, FLT_DECODED, FLT_N };
static const char *const FLT_NAME[FLT_N] = {
    "all", "with audio", "not downloaded", "downloaded", "decoded"
};

// What a running child is doing, which decides how its progress is
// counted and what is re-read when it finishes.
enum { JOB_NONE = 0, JOB_LIST, JOB_DOWNLOAD, JOB_DECODE };

typedef struct {
    int    kind;
    pid_t  pid;
    int    fd;                 // read end of the child's stdout+stderr
    int    running;
    char   title[128];
    char   partial[JOB_LINE_LEN];
    int    partial_len;
    char   lines[JOB_LOG_LINES][JOB_LINE_LEN];
    int    n_lines;
    int    done;               // audio files written
    int    failed;
    int    total;              // expected downloads, 0 while listing
    // Remaining work, fetched a batch at a time so the query string
    // stays short. The next batch is spawned when the child exits.
    long  *queue;
    int    queue_n;
    int    queue_pos;
    int    cancelled;
} job_t;

static obs_t  g_rows[MAX_OBS_ROWS];
static int    g_n_rows = 0;
static int    g_view[MAX_OBS_ROWS];   // indices into g_rows passing the filter
static int    g_n_view = 0;
static int    g_sel = 0;
static int    g_top = 0;
static int    g_filter = FLT_ALL;
static int    g_truncated = 0;

static char   g_day[16] = "";          // YYYY-MM-DD, UTC
static char   g_archive[768] = "";
static char   g_db_path[768] = "";
static char   g_pull[768] = "satnogs_pull.sh";
static char   g_decoder[768] = "decode_passes.sh";
static long   g_norad = 69015;
static int    g_have_color = 0;
static char   g_status[256] = "";      // transient message on the bottom bar
static char   g_cache_note[128] = "";  // cache age, or its absence

static job_t  g_job = {0};

// Observation ids that have produced packets, sorted for bsearch. Read
// once at startup: 200k rows grouped down to a few tens of thousands of
// directories is a second at worst, and it never changes under us in a
// way that matters mid-session.
static long  *g_decoded_ids = NULL;
static int    g_n_decoded = 0;

static int cmp_long(const void *a, const void *b)
{
    long x = *(const long *)a, y = *(const long *)b;
    return (x > y) - (x < y);
}

static int is_decoded(long id)
{
    if (g_decoded_ids == NULL || g_n_decoded == 0) return 0;
    return bsearch(&id, g_decoded_ids, (size_t)g_n_decoded,
                   sizeof *g_decoded_ids, cmp_long) != NULL;
}

static void set_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_status, sizeof g_status, fmt, ap);
    va_end(ap);
}

// ---------------------------------------------------------------- data

// Path of the day cache satnogs_pull.sh --cache-day writes.
static void day_cache_path(const char *day, char *out, size_t outn)
{
    snprintf(out, outn, "%s/.daycache/%ld/%s.tsv", g_archive, g_norad, day);
}

// "2026-08-28T03:25:06Z" -> "03:25:06". Anything unexpected passes
// through as-is so a format change shows on screen instead of blanking.
static const char *hhmmss(const char *iso)
{
    static char buf[16];
    const char *t = strchr(iso, 'T');
    if (t == NULL || strlen(t + 1) < 8) return iso;
    snprintf(buf, sizeof buf, "%.8s", t + 1);
    return buf;
}

// Pass length in seconds from the two ISO timestamps, or -1.
static long pass_seconds(const obs_t *o)
{
    struct tm a = {0}, b = {0};
    if (strptime(o->start, "%Y-%m-%dT%H:%M:%SZ", &a) == NULL) return -1;
    if (strptime(o->end,   "%Y-%m-%dT%H:%M:%SZ", &b) == NULL) return -1;
    time_t ta = timegm(&a), tb = timegm(&b);
    if (ta == (time_t)-1 || tb == (time_t)-1) return -1;
    return (long)(tb - ta);
}

// Does <archive>/<id>/ hold an audio file? The pull script writes
// satnogs_<id>_<start>.<ext> and only moves it into place once the
// download finished, so the name alone is proof enough.
static int have_audio_on_disk(long id)
{
    char dir[900];
    snprintf(dir, sizeof dir, "%s/%ld", g_archive, id);
    DIR *d = opendir(dir);
    if (d == NULL) return 0;

    char prefix[64];
    snprintf(prefix, sizeof prefix, "satnogs_%ld_", id);
    size_t plen = strlen(prefix);

    int found = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, prefix, plen) != 0) continue;
        const char *dot = strrchr(e->d_name, '.');
        if (dot == NULL) continue;
        // Ignore a half-written .part from an interrupted download.
        if (strcmp(dot, ".part") == 0) continue;
        found = 1;
        break;
    }
    closedir(d);
    return found;
}

// Read the ids of every observation that has produced packets. The
// session_dir column holds the archive directory, so its last path
// element is the observation id.
static void load_decoded_ids(void)
{
    // Read again after a decode, so start from nothing rather than
    // adding a second array beside the first.
    free(g_decoded_ids);
    g_decoded_ids = NULL;
    g_n_decoded = 0;

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(g_db_path, &db, SQLITE_OPEN_READWRITE, NULL) != SQLITE_OK) {
        // No database, or one we cannot open: the decoded column just
        // stays empty. It is a convenience, not the point of the tool.
        if (db) sqlite3_close(db);
        return;
    }
    sqlite3_busy_timeout(db, 5000);

    const char *sql =
        "SELECT DISTINCT session_dir FROM packet "
        "WHERE session_dir LIKE ?1 || '/%'";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        sqlite3_close(db);
        return;
    }
    sqlite3_bind_text(st, 1, g_archive, -1, SQLITE_STATIC);

    int cap = 4096;
    long *ids = malloc((size_t)cap * sizeof *ids);
    int n = 0;
    while (ids != NULL && sqlite3_step(st) == SQLITE_ROW) {
        const char *dir = (const char *)sqlite3_column_text(st, 0);
        if (dir == NULL) continue;
        const char *slash = strrchr(dir, '/');
        if (slash == NULL) continue;
        char *endp = NULL;
        long id = strtol(slash + 1, &endp, 10);
        if (endp == slash + 1 || (endp && *endp != '\0')) continue;
        if (n == cap) {
            int ncap = cap * 2;
            long *bigger = realloc(ids, (size_t)ncap * sizeof *ids);
            if (bigger == NULL) break;
            ids = bigger;
            cap = ncap;
        }
        ids[n++] = id;
    }
    sqlite3_finalize(st);
    sqlite3_close(db);

    if (ids != NULL && n > 0) {
        qsort(ids, (size_t)n, sizeof *ids, cmp_long);
        g_decoded_ids = ids;
        g_n_decoded = n;
    } else {
        free(ids);
    }
}

// Split a tab-separated line in place into at most `max` fields.
static int split_tabs(char *line, char **fields, int max)
{
    int n = 0;
    char *p = line;
    while (n < max) {
        fields[n++] = p;
        char *tab = strchr(p, '\t');
        if (tab == NULL) break;
        *tab = '\0';
        p = tab + 1;
    }
    return n;
}

// One entry per observation this station has audio for, built by
// walking the archive once at startup. The archive is filed by
// observation id rather than by date, so without this index a day with
// no SatNOGS listing yet would show nothing at all -- even a day whose
// recordings are all sitting on the disk.
typedef struct {
    long id;
    char day[12];     // YYYY-MM-DD, from the recording's filename
    char start[32];   // the same timestamp as the API writes it
} local_t;

static local_t *g_local = NULL;
static int      g_n_local = 0;

// satnogs_<id>_YYYY-MM-DDTHH-MM-SS.<ext> is what satnogs_pull.sh names
// a recording, so the filename alone dates the pass. Fills day and
// start; returns 0 if the name is not one of ours.
static int parse_recording_name(const char *name, long id,
                                char *day, size_t dayn,
                                char *start, size_t startn)
{
    char prefix[64];
    int plen = snprintf(prefix, sizeof prefix, "satnogs_%ld_", id);
    if (strncmp(name, prefix, (size_t)plen) != 0) return 0;

    const char *ts = name + plen;
    // YYYY-MM-DDTHH-MM-SS is 19 characters.
    if (strlen(ts) < 19 || ts[4] != '-' || ts[7] != '-' || ts[10] != 'T')
        return 0;

    snprintf(day, dayn, "%.10s", ts);
    snprintf(start, startn, "%.10sT%.2s:%.2s:%.2sZ",
             ts, ts + 11, ts + 14, ts + 17);
    return 1;
}

// Walk the archive and note every observation with a recording. Called
// once before the interface starts, because on a real archive this is
// tens of thousands of directories.
static void scan_local_archive(void)
{
    DIR *d = opendir(g_archive);
    if (d == NULL) return;

    int cap = 4096;
    g_local = malloc((size_t)cap * sizeof *g_local);
    if (g_local == NULL) { closedir(d); return; }

    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        char *endp = NULL;
        long id = strtol(e->d_name, &endp, 10);
        if (id <= 0 || endp == e->d_name || *endp != '\0') continue;

        // Room for the archive path and any name the filesystem can
        // hand back, so the join cannot be truncated into a path that
        // names something else.
        char sub[2048];
        snprintf(sub, sizeof sub, "%s/%s", g_archive, e->d_name);
        DIR *s = opendir(sub);
        if (s == NULL) continue;

        struct dirent *f;
        while ((f = readdir(s)) != NULL) {
            const char *dot = strrchr(f->d_name, '.');
            if (dot == NULL || strcmp(dot, ".part") == 0) continue;

            char day[12], start[32];
            if (!parse_recording_name(f->d_name, id, day, sizeof day,
                                      start, sizeof start))
                continue;

            if (g_n_local == cap) {
                int ncap = cap * 2;
                local_t *bigger = realloc(g_local, (size_t)ncap * sizeof *bigger);
                if (bigger == NULL) break;
                g_local = bigger;
                cap = ncap;
            }
            g_local[g_n_local].id = id;
            snprintf(g_local[g_n_local].day,   sizeof g_local[0].day,   "%s", day);
            snprintf(g_local[g_n_local].start, sizeof g_local[0].start, "%s", start);
            g_n_local++;
            break;
        }
        closedir(s);
    }
    closedir(d);
}

// ---------------------------------------------------------------- notes
//
// A line of the operator's own words against an observation -- what a
// pass was, why it matters -- kept in one tab-separated file at the top
// of the archive so it survives the day caches being rewritten and can
// be read without this program.

typedef struct {
    long id;
    char text[NOTE_MAX];
} note_t;

static note_t *g_notes = NULL;
static int     g_n_notes = 0;
static int     g_notes_cap = 0;

static void notes_path(char *out, size_t outn)
{
    snprintf(out, outn, "%s/.notes.tsv", g_archive);
}

static const char *note_for(long id)
{
    for (int i = 0; i < g_n_notes; i++)
        if (g_notes[i].id == id) return g_notes[i].text;
    return "";
}

static void notes_load(void)
{
    char path[800];
    notes_path(path, sizeof path);
    FILE *f = fopen(path, "r");
    if (f == NULL) return;

    char line[NOTE_MAX + 64];
    while (fgets(line, sizeof line, f) != NULL) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            line[--len] = '\0';
        char *tab = strchr(line, '\t');
        if (tab == NULL) continue;
        *tab = '\0';
        long id = strtol(line, NULL, 10);
        if (id <= 0 || tab[1] == '\0') continue;

        if (g_n_notes == g_notes_cap) {
            int ncap = g_notes_cap ? g_notes_cap * 2 : 64;
            note_t *bigger = realloc(g_notes, (size_t)ncap * sizeof *bigger);
            if (bigger == NULL) break;
            g_notes = bigger;
            g_notes_cap = ncap;
        }
        g_notes[g_n_notes].id = id;
        snprintf(g_notes[g_n_notes].text, NOTE_MAX, "%s", tab + 1);
        g_n_notes++;
    }
    fclose(f);
}

// Written whole, through a temporary, so an interrupted save leaves the
// notes that were there rather than half of them.
static int notes_save(void)
{
    char path[800], tmp[820];
    notes_path(path, sizeof path);
    snprintf(tmp, sizeof tmp, "%s.tmp", path);

    FILE *f = fopen(tmp, "w");
    if (f == NULL) return 0;
    for (int i = 0; i < g_n_notes; i++)
        fprintf(f, "%ld\t%s\n", g_notes[i].id, g_notes[i].text);
    if (fclose(f) != 0) { unlink(tmp); return 0; }
    if (rename(tmp, path) != 0) { unlink(tmp); return 0; }
    return 1;
}

// Set, replace or (with empty text) drop one observation's note.
static int note_set(long id, const char *text)
{
    int at = -1;
    for (int i = 0; i < g_n_notes; i++)
        if (g_notes[i].id == id) { at = i; break; }

    if (text[0] == '\0') {
        if (at < 0) return 1;
        g_notes[at] = g_notes[--g_n_notes];
    } else if (at >= 0) {
        snprintf(g_notes[at].text, NOTE_MAX, "%s", text);
    } else {
        if (g_n_notes == g_notes_cap) {
            int ncap = g_notes_cap ? g_notes_cap * 2 : 64;
            note_t *bigger = realloc(g_notes, (size_t)ncap * sizeof *bigger);
            if (bigger == NULL) return 0;
            g_notes = bigger;
            g_notes_cap = ncap;
        }
        g_notes[g_n_notes].id = id;
        snprintf(g_notes[g_n_notes].text, NOTE_MAX, "%s", text);
        g_n_notes++;
    }
    return notes_save();
}

// The meta.json beside a recording is jq's pretty-printed copy of the
// observation record, one top-level field to a line at a two-space
// indent. That is enough structure to pull the few fields the list
// shows without a JSON parser -- and the indent is what keeps "status"
// from matching "waterfall_status" or "transmitter_status".
static int meta_field(const char *json, const char *key, char *out, size_t outn)
{
    char needle[64];
    snprintf(needle, sizeof needle, "\n  \"%s\":", key);
    const char *p = strstr(json, needle);
    if (p == NULL) return 0;
    p += strlen(needle);
    while (*p == ' ') p++;

    size_t n = 0;
    if (*p == '"') {
        p++;
        while (*p != '\0' && *p != '"' && n + 1 < outn) out[n++] = *p++;
    } else {
        while (*p != '\0' && *p != ',' && *p != '\n' && n + 1 < outn) out[n++] = *p++;
    }
    out[n] = '\0';
    return n > 0 && strcmp(out, "null") != 0;
}

// The whole stored record, or NULL. Read entire rather than by the
// headful: a pass with a lot of demodulated data runs past 8 kB on the
// demoddata list alone, and everything the display wants -- the status,
// the station, the elevation -- is written after it.
static char *read_meta(long id)
{
    char path[900];
    snprintf(path, sizeof path, "%s/%ld/satnogs_%ld.meta.json",
             g_archive, id, id);
    FILE *f = fopen(path, "r");
    if (f == NULL) return NULL;

    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long size = ftell(f);
    rewind(f);
    if (size < 0 || size > META_MAX_BYTES) { fclose(f); return NULL; }

    char *json = malloc((size_t)size + 1);
    if (json == NULL) { fclose(f); return NULL; }
    size_t n = fread(json, 1, (size_t)size, f);
    fclose(f);
    json[n] = '\0';
    return json;
}

// One entry per frame SatNOGS demodulated, each carrying a URL to it.
static int count_demoddata(const char *json)
{
    int n = 0;
    for (const char *p = json;
         (p = strstr(p, "\"payload_demod\"")) != NULL;
         p += 15)
        n++;
    return n;
}

// Fill in what the stored record knows about an observation we hold but
// whose day has not been listed. Best effort: a row with nothing behind
// it still shows its id, its time and the fact that we have it.
static void fill_from_meta(obs_t *o)
{
    char *json = read_meta(o->id);
    if (json == NULL) return;

    // Each destination is sized for the value it holds, and a longer
    // one is cut to fit rather than rejected -- these are display
    // columns, and the precision says so explicitly.
    char buf[128];
    if (meta_field(json, "end", buf, sizeof buf))
        snprintf(o->end, sizeof o->end, "%.*s", (int)sizeof o->end - 1, buf);
    if (meta_field(json, "status", buf, sizeof buf))
        snprintf(o->status, sizeof o->status, "%.*s", (int)sizeof o->status - 1, buf);
    if (meta_field(json, "waterfall_status", buf, sizeof buf))
        snprintf(o->waterfall, sizeof o->waterfall, "%.*s", (int)sizeof o->waterfall - 1, buf);
    if (meta_field(json, "station_name", buf, sizeof buf))
        snprintf(o->station, sizeof o->station, "%.*s", (int)sizeof o->station - 1, buf);
    if (meta_field(json, "ground_station", buf, sizeof buf))
        o->station_id = strtol(buf, NULL, 10);
    if (meta_field(json, "max_altitude", buf, sizeof buf))
        o->max_el = atof(buf);
    // What SatNOGS had demodulated when we pulled the recording down.
    o->n_data = count_demoddata(json);
    free(json);
}

static int cmp_obs_start(const void *a, const void *b)
{
    const obs_t *x = a, *y = b;
    int c = strcmp(x->start, y->start);
    if (c != 0) return c;
    return (x->id > y->id) - (x->id < y->id);
}

// Add the observations we hold for this day that the listing does not
// mention -- which, on a day never listed, is all of them. This is what
// makes the archive visible before a single request has been spent.
static void merge_local(void)
{
    for (int i = 0; i < g_n_local && g_n_rows < MAX_OBS_ROWS; i++) {
        if (strcmp(g_local[i].day, g_day) != 0) continue;

        int already = 0;
        for (int j = 0; j < g_n_rows; j++)
            if (g_rows[j].id == g_local[i].id) { already = 1; break; }
        if (already) continue;

        obs_t *o = &g_rows[g_n_rows++];
        memset(o, 0, sizeof *o);
        o->id = g_local[i].id;
        snprintf(o->start,  sizeof o->start,  "%s", g_local[i].start);
        snprintf(o->status, sizeof o->status, "%s", "held");
        o->has_audio  = 1;
        o->downloaded = 1;
        o->decoded    = is_decoded(o->id);
        o->n_data     = -1;
        // The record stored beside the recording fills in the station,
        // the real status and the pass geometry, so a held observation
        // reads the same as a listed one.
        fill_from_meta(o);
    }
    qsort(g_rows, (size_t)g_n_rows, sizeof g_rows[0], cmp_obs_start);
}

// Read the day cache into g_rows, add whatever we hold locally for the
// same day, and work out each observation's state. Returns 0 if the day
// has no SatNOGS listing yet -- the rows can still be non-empty.
static int load_day(void)
{
    g_n_rows = 0;
    g_truncated = 0;
    g_cache_note[0] = '\0';

    char path[900];
    day_cache_path(g_day, path, sizeof path);

    struct stat st;
    if (stat(path, &st) != 0) {
        snprintf(g_cache_note, sizeof g_cache_note, "not listed yet");
        merge_local();
        return 0;
    }

    // Age of the listing, so a day cached before its passes were vetted
    // (or before its audio was uploaded) is visibly stale.
    time_t age = time(NULL) - st.st_mtime;
    if (age < 0) age = 0;
    if (age < 3600)
        snprintf(g_cache_note, sizeof g_cache_note, "listed %ldm ago", (long)(age / 60));
    else if (age < 86400)
        snprintf(g_cache_note, sizeof g_cache_note, "listed %ldh ago", (long)(age / 3600));
    else
        snprintf(g_cache_note, sizeof g_cache_note, "listed %ldd ago", (long)(age / 86400));

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        snprintf(g_cache_note, sizeof g_cache_note, "cache unreadable");
        merge_local();
        return 0;
    }

    char line[2048];
    while (fgets(line, sizeof line, f) != NULL) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            line[--len] = '\0';
        if (len == 0) continue;

        if (g_n_rows >= MAX_OBS_ROWS) { g_truncated = 1; break; }

        char *fld[10] = {0};
        int nf = split_tabs(line, fld, 10);
        if (nf < 9) continue;

        obs_t *o = &g_rows[g_n_rows];
        memset(o, 0, sizeof *o);
        o->id = strtol(fld[0], NULL, 10);
        if (o->id <= 0) continue;
        snprintf(o->start,     sizeof o->start,     "%s", fld[1]);
        snprintf(o->end,       sizeof o->end,       "%s", fld[2]);
        snprintf(o->status,    sizeof o->status,    "%s", fld[3]);
        snprintf(o->waterfall, sizeof o->waterfall, "%s", fld[4]);
        o->station_id = strtol(fld[5], NULL, 10);
        snprintf(o->station,   sizeof o->station,   "%s", fld[6]);
        o->max_el = atof(fld[7]);
        o->has_audio = (fld[8][0] != '\0');
        // Day caches written before the count existed have nine
        // columns. Rather than spend requests re-listing them, read the
        // count off the record stored beside any recording we hold.
        o->n_data = (nf >= 10) ? (int)strtol(fld[9], NULL, 10) : -1;

        o->downloaded = have_audio_on_disk(o->id);
        o->decoded    = o->downloaded && is_decoded(o->id);
        if (o->n_data < 0 && o->downloaded) {
            char *json = read_meta(o->id);
            if (json != NULL) {
                o->n_data = count_demoddata(json);
                free(json);
            }
        }
        g_n_rows++;
    }
    fclose(f);
    merge_local();
    return 1;
}

// A pass is worth fetching when SatNOGS has the audio and we do not.
static int fetchable(const obs_t *o)
{
    return o->has_audio && !o->downloaded;
}

// Worth marking if there is a recording anywhere: here to decode, or on
// SatNOGS to fetch. A pass nobody recorded can only be looked at.
static int markable(const obs_t *o)
{
    return o->has_audio || o->downloaded;
}

static int passes_filter(const obs_t *o)
{
    switch (g_filter) {
        case FLT_AUDIO:   return o->has_audio;
        case FLT_MISSING: return fetchable(o);
        case FLT_HAVE:    return o->downloaded;
        case FLT_DECODED: return o->decoded;
        default:          return 1;
    }
}

// Rebuild the filtered index, keeping the cursor on the same
// observation where it survives the new filter.
static void rebuild_view(void)
{
    long keep = (g_sel >= 0 && g_sel < g_n_view) ? g_rows[g_view[g_sel]].id : -1;

    g_n_view = 0;
    for (int i = 0; i < g_n_rows; i++)
        if (passes_filter(&g_rows[i])) g_view[g_n_view++] = i;

    g_sel = 0;
    if (keep >= 0) {
        for (int i = 0; i < g_n_view; i++) {
            if (g_rows[g_view[i]].id == keep) { g_sel = i; break; }
        }
    }
    if (g_sel >= g_n_view) g_sel = g_n_view > 0 ? g_n_view - 1 : 0;
    g_top = 0;
}

static int count_marked(void)
{
    int n = 0;
    for (int i = 0; i < g_n_rows; i++) if (g_rows[i].marked) n++;
    return n;
}

static void clear_marks(void)
{
    for (int i = 0; i < g_n_rows; i++) g_rows[i].marked = 0;
}

// ----------------------------------------------------------------- day

static void day_shift(int days)
{
    struct tm tm = {0};
    if (strptime(g_day, "%Y-%m-%d", &tm) == NULL) return;
    time_t t = timegm(&tm) + (time_t)days * 86400;
    struct tm out;
    gmtime_r(&t, &out);
    strftime(g_day, sizeof g_day, "%Y-%m-%d", &out);
}

static void day_today(void)
{
    time_t now = time(NULL);
    struct tm out;
    gmtime_r(&now, &out);
    strftime(g_day, sizeof g_day, "%Y-%m-%d", &out);
}

static int day_valid(const char *s)
{
    struct tm tm = {0};
    if (strlen(s) != 10) return 0;
    return strptime(s, "%Y-%m-%d", &tm) != NULL;
}

// Re-read whatever g_day now names. Switching days drops the marks:
// they name observations on the day they were made, and carrying them
// across would make `d` fetch things that scrolled off the screen days
// ago.
static void reload_day(void)
{
    clear_marks();
    load_day();
    rebuild_view();
}

// Only for a day that came from somewhere else -- the `g` prompt, or
// the command line. day_shift() has already written g_day, so its
// callers use reload_day() directly: passing g_day back in here would
// be snprintf copying a buffer onto itself, which is undefined and in
// practice leaves the date empty.
static void go_to_day(const char *day)
{
    snprintf(g_day, sizeof g_day, "%s", day);
    reload_day();
}

// ----------------------------------------------------------------- job

static void job_add_line(const char *text)
{
    if (g_job.n_lines == JOB_LOG_LINES) {
        memmove(g_job.lines[0], g_job.lines[1],
                sizeof g_job.lines[0] * (JOB_LOG_LINES - 1));
        g_job.n_lines--;
    }
    snprintf(g_job.lines[g_job.n_lines], JOB_LINE_LEN, "%s", text);
    g_job.n_lines++;

    // The pull script announces each finished file with an indented
    // arrow and each failure with two bangs; counting those is what
    // drives the progress figure. A decode is counted by its children
    // instead -- one to an observation -- so this only reads download
    // output.
    if (g_job.kind != JOB_DOWNLOAD) return;
    if (strstr(text, "-> ") != NULL && strstr(text, "satnogs_") != NULL)
        g_job.done++;
    else if (strstr(text, "!! ") != NULL)
        g_job.failed++;
}

// Spawn satnogs_pull.sh with the given arguments, reading its output
// through a pipe. argv must be NULL-terminated and starts with the
// script path.
static int job_spawn(char *const argv[], const char *title)
{
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        set_status("cannot create pipe: %s", strerror(errno));
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        set_status("cannot fork: %s", strerror(errno));
        return -1;
    }
    if (pid == 0) {
        // Child: both streams down the pipe, then become the script. A
        // new process group so cancelling kills curl along with it.
        setpgid(0, 0);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[0]);
        close(pipefd[1]);
        execvp(argv[0], argv);
        // Only reached if exec failed; the parent sees it as output.
        fprintf(stderr, "!! cannot run %s: %s\n", argv[0], strerror(errno));
        _exit(127);
    }

    close(pipefd[1]);
    fcntl(pipefd[0], F_SETFL, O_NONBLOCK);
    g_job.pid = pid;
    g_job.fd = pipefd[0];
    g_job.running = 1;
    g_job.partial_len = 0;
    snprintf(g_job.title, sizeof g_job.title, "%s", title);
    return 0;
}

// Hand the next batch of ids to the script. Returns 1 if one started.
// One observation to a child: decode_passes.sh walks a directory, and
// the marked set is a scattering of directories rather than a tree.
static int decode_next_batch(void)
{
    long id = g_job.queue[g_job.queue_pos++];

    // decode_passes.sh takes --root as two words, not as --root=<dir>.
    char root[800], db_arg[800], title[128];
    snprintf(root,   sizeof root,   "%s/%ld", g_archive, id);
    snprintf(db_arg, sizeof db_arg, "--db=%s", g_db_path);
    snprintf(title,  sizeof title,  "decoding %d observation%s",
             g_job.queue_n, g_job.queue_n == 1 ? "" : "s");

    char *argv[] = { g_decoder, (char *)"--root", root, db_arg, NULL };
    return job_spawn(argv, title) == 0;
}

static int job_next_batch(void)
{
    if (g_job.queue == NULL || g_job.queue_pos >= g_job.queue_n) return 0;
    if (g_job.cancelled) return 0;

    if (g_job.kind == JOB_DECODE) return decode_next_batch();

    int n = g_job.queue_n - g_job.queue_pos;
    if (n > IDS_PER_BATCH) n = IDS_PER_BATCH;

    // "--obs-ids=" plus up to IDS_PER_BATCH ids and their commas.
    char ids[IDS_PER_BATCH * 12 + 32];
    int len = snprintf(ids, sizeof ids, "--obs-ids=");
    for (int i = 0; i < n; i++) {
        len += snprintf(ids + len, sizeof ids - (size_t)len, "%s%ld",
                        i ? "," : "", g_job.queue[g_job.queue_pos + i]);
    }
    g_job.queue_pos += n;

    char out_arg[800];
    snprintf(out_arg, sizeof out_arg, "--out=%s", g_archive);
    char norad_arg[64];
    snprintf(norad_arg, sizeof norad_arg, "--norad-id=%ld", g_norad);

    char title[128];
    snprintf(title, sizeof title, "downloading %d observation%s",
             g_job.queue_n, g_job.queue_n == 1 ? "" : "s");

    // --no-local-tle because this browses history: an observation from
    // three months ago should keep the elements SatNOGS actually flew
    // it with, not whichever TLE happens to be newest on this disk.
    char *argv[] = { g_pull, out_arg, norad_arg, "--no-local-tle", ids, NULL };
    return job_spawn(argv, title) == 0;
}

// Marks are a selection, not an instruction: d takes the ones SatNOGS
// can still send us, p takes the ones already on the disk. So each
// starts by picking its own half out of the same set.
static int job_queue_marked(int kind, int (*wanted)(const obs_t *))
{
    int n = count_marked();
    if (n == 0) { set_status("nothing marked"); return 0; }

    free(g_job.queue);
    memset(&g_job, 0, sizeof g_job);
    g_job.kind = kind;
    g_job.queue = malloc((size_t)n * sizeof *g_job.queue);
    if (g_job.queue == NULL) { set_status("out of memory"); return 0; }

    for (int i = 0; i < g_n_rows; i++)
        if (g_rows[i].marked && wanted(&g_rows[i]))
            g_job.queue[g_job.queue_n++] = g_rows[i].id;
    g_job.total = g_job.queue_n;
    return g_job.queue_n;
}

static int already_here(const obs_t *o) { return o->downloaded; }

static void job_start_download(void)
{
    if (job_queue_marked(JOB_DOWNLOAD, fetchable) == 0) {
        if (count_marked() > 0)
            set_status("everything marked is already downloaded");
        return;
    }
    if (!job_next_batch()) set_status("could not start the download");
}

static void job_start_decode(void)
{
    if (job_queue_marked(JOB_DECODE, already_here) == 0) {
        if (count_marked() > 0)
            set_status("nothing marked is downloaded yet -- d fetches it first");
        return;
    }
    if (!job_next_batch()) set_status("could not run %s", g_decoder);
}

static void job_start_listing(void)
{
    free(g_job.queue);
    memset(&g_job, 0, sizeof g_job);
    g_job.kind = JOB_LIST;

    char out_arg[800], norad_arg[64], day_arg[64], title[128];
    snprintf(out_arg,   sizeof out_arg,   "--out=%s", g_archive);
    snprintf(norad_arg, sizeof norad_arg, "--norad-id=%ld", g_norad);
    snprintf(day_arg,   sizeof day_arg,   "--cache-day=%s", g_day);
    snprintf(title,     sizeof title,     "listing %s from SatNOGS", g_day);

    char *argv[] = { g_pull, out_arg, norad_arg, day_arg, NULL };
    job_spawn(argv, title);
}

static void job_cancel(void)
{
    if (!g_job.running) return;
    g_job.cancelled = 1;
    // The child leads its own process group, so this reaches curl too.
    kill(-g_job.pid, SIGTERM);
    set_status("cancelling...");
}

// Drain whatever the child has written and reap it when it exits.
static void job_poll(void)
{
    if (!g_job.running) return;

    char buf[1024];
    ssize_t n;
    while ((n = read(g_job.fd, buf, sizeof buf)) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n' || g_job.partial_len == JOB_LINE_LEN - 1) {
                g_job.partial[g_job.partial_len] = '\0';
                if (g_job.partial_len > 0) job_add_line(g_job.partial);
                g_job.partial_len = 0;
            } else if (c != '\r') {
                g_job.partial[g_job.partial_len++] = c;
            }
        }
    }

    int st = 0;
    pid_t r = waitpid(g_job.pid, &st, WNOHANG);
    if (r != g_job.pid) return;

    // Flush a last line the child left without a newline.
    if (g_job.partial_len > 0) {
        g_job.partial[g_job.partial_len] = '\0';
        job_add_line(g_job.partial);
        g_job.partial_len = 0;
    }
    close(g_job.fd);
    g_job.fd = -1;
    g_job.running = 0;

    // A decode is one child to an observation, so the child's own exit
    // status is the progress: the script's per-file chatter is about
    // frames, not about which observation it belonged to.
    if (g_job.kind == JOB_DECODE) {
        if (WIFEXITED(st) && WEXITSTATUS(st) == 0) g_job.done++;
        else                                       g_job.failed++;
    }

    // More ids waiting means this was one batch of several.
    if (!g_job.cancelled && job_next_batch()) return;

    // Everything the job could change is on disk now, so re-read it.
    // A decode also puts rows in the database, which is where the green
    // in the local column comes from.
    if (g_job.kind == JOB_DECODE) load_decoded_ids();
    load_day();
    rebuild_view();

    if (g_job.cancelled)
        set_status("cancelled after %d of %d", g_job.done, g_job.total);
    else if (g_job.kind == JOB_DECODE)
        set_status("decoded %d of %d%s", g_job.done, g_job.total,
                   g_job.failed ? " (some failed)" : "");
    else if (g_job.total > 0)
        set_status("downloaded %d of %d%s", g_job.done, g_job.total,
                   g_job.failed ? " (some failed)" : "");
    else
        set_status("listing finished");
}

// ---------------------------------------------------------------- draw

static int attr_for(int pair)
{
    return g_have_color ? COLOR_PAIR(pair) : A_NORMAL;
}

static void put(int row, int col, int maxw, int attr, const char *text)
{
    if (row < 0 || col < 0 || maxw <= 0) return;
    char buf[1024];
    snprintf(buf, sizeof buf, "%-*.*s", maxw, maxw, text);
    attron(attr);
    mvaddnstr(row, col, buf, maxw);
    attroff(attr);
}

// Copy text into out, cut to width with a trailing ellipsis when it does
// not fit. Width is in characters, ellipsis included.
static void elide(char *out, size_t outn, const char *text, int width)
{
    if (width < 0) width = 0;
    if ((int)strlen(text) <= width) {
        snprintf(out, outn, "%.*s", width, text);
        return;
    }
    int keep = width - 3;
    if (keep < 0) keep = 0;
    snprintf(out, outn, "%.*s...", keep, text);
}

static const char *local_word(const obs_t *o)
{
    if (o->decoded)    return "decoded";
    if (o->downloaded) return "have";
    if (!o->has_audio) return "no audio";
    return "--";
}

static int local_pair(const obs_t *o)
{
    if (o->decoded)    return PAIR_DECODED;
    if (o->downloaded) return PAIR_HAVE;
    if (!o->has_audio) return PAIR_NOAUDIO;
    return 0;
}

static void draw_top_bar(int cols)
{
    int audio = 0, have = 0, decoded = 0;
    for (int i = 0; i < g_n_rows; i++) {
        if (g_rows[i].has_audio)  audio++;
        if (g_rows[i].downloaded) have++;
        if (g_rows[i].decoded)    decoded++;
    }
    int marked = count_marked();

    // With a filter on, the row numbers stop matching the day's count,
    // so say how many rows they run to.
    char shown[32] = "";
    if (g_n_view != g_n_rows)
        snprintf(shown, sizeof shown, " (%d shown)", g_n_view);

    char line[512];
    snprintf(line, sizeof line,
             " %s UTC   %d obs  %d audio  %d have  %d decoded   filter: %s%s   %s",
             g_day, g_n_rows, audio, have, decoded,
             FLT_NAME[g_filter], shown, g_cache_note);
    if (marked > 0) {
        char more[64];
        snprintf(more, sizeof more, "   %d marked", marked);
        strncat(line, more, sizeof line - strlen(line) - 1);
    }
    put(0, 0, cols, attr_for(PAIR_BAR) | A_BOLD, line);
}

static void draw_headings(int cols)
{
    char line[512];
    snprintf(line, sizeof line,
             "   %4s %-9s %-8s %5s %4s %5s  %-8s %-8s %s",
             "no", "id", "start", "len", "el", "data",
             "status", "local", "station");
    put(1, 0, cols, A_BOLD, line);
}

static void draw_list(int top_row, int height, int cols)
{
    if (g_n_view == 0) {
        const char *msg = (g_n_rows == 0)
            ? "no listing for this day yet -- press r to fetch it from SatNOGS"
            : "nothing matches this filter -- press f to change it";
        put(top_row, 2, cols - 2, attr_for(PAIR_WARN), msg);
        return;
    }

    if (g_sel < g_top) g_top = g_sel;
    if (g_sel >= g_top + height) g_top = g_sel - height + 1;
    if (g_top < 0) g_top = 0;

    for (int r = 0; r < height; r++) {
        int vi = g_top + r;
        if (vi >= g_n_view) break;
        const obs_t *o = &g_rows[g_view[vi]];

        // Wide enough for a nonsense timestamp pair as well as a real
        // pass, so the formatting can't be truncated.
        char len[32] = "  -- ";
        long secs = pass_seconds(o);
        if (secs >= 0) snprintf(len, sizeof len, "%ld:%02ld", secs / 60, secs % 60);

        char el[8];
        snprintf(el, sizeof el, "%.0f", o->max_el);

        // A dash, not a zero: nothing here has been told how many
        // frames the pass yielded, which is a different thing from
        // being told that it yielded none.
        char data[16] = "    -";
        if (o->n_data >= 0) snprintf(data, sizeof data, "%d", o->n_data);

        char line[1024];
        // The number counts rows as displayed, so it follows the
        // filter: with one on, row 1 is the first row you can see, not
        // the first of the day. The station is boxed to a fixed width
        // here so that the note after it starts in the same column on
        // every row; the detail pane shows the name in full.
        int used = snprintf(line, sizeof line,
                            " %c %4d %-9ld %-8s %5s %4s %5s  %-8s %-8s %-14.14s",
                            o->marked ? '*' : ' ', vi + 1,
                            o->id, hhmmss(o->start), len, el, data,
                            o->status, local_word(o), o->station);

        const char *note = note_for(o->id);
        if (note[0] != '\0' && used > 0 && used < (int)sizeof line) {
            // Enough room to say something, or leave it to the detail
            // pane rather than show an ellipsis and nothing else.
            int room = cols - used - 1;
            if (room >= 8) {
                char cut[NOTE_MAX + 8];
                elide(cut, sizeof cut, note, room);
                snprintf(line + used, sizeof line - (size_t)used, " %s", cut);
            }
        }

        int attr;
        if (vi == g_sel)      attr = attr_for(PAIR_SEL) | A_BOLD;
        else if (o->marked)   attr = attr_for(PAIR_MARKED) | A_BOLD;
        else                  attr = attr_for(local_pair(o));
        put(top_row + r, 0, cols, attr, line);
    }
}

static void draw_detail(int top_row, int height, int cols)
{
    if (height <= 0) return;
    if (g_n_view == 0 || g_sel >= g_n_view) return;
    const obs_t *o = &g_rows[g_view[g_sel]];

    // Room for the archive path, which is itself up to 768 bytes, plus
    // the sentence around it. The screen truncates what it cannot show.
    char l[5][1024];
    snprintf(l[0], sizeof l[0], "observation %ld   %s to %s",
             o->id, o->start, o->end);
    snprintf(l[1], sizeof l[1], "station %ld  %s   max elevation %.0f deg",
             o->station_id, o->station, o->max_el);
    char data[64] = "frames demodulated by SatNOGS unknown";
    if (o->n_data >= 0)
        snprintf(data, sizeof data, "%d frame%s demodulated by SatNOGS",
                 o->n_data, o->n_data == 1 ? "" : "s");
    snprintf(l[2], sizeof l[2], "SatNOGS status %s   waterfall %s   audio %s   %s",
             o->status, o->waterfall,
             o->has_audio ? "available" : "none uploaded", data);
    if (o->downloaded)
        snprintf(l[3], sizeof l[3], "local %s/%ld/   %s",
                 g_archive, o->id,
                 o->decoded ? "packets in the database"
                            : "downloaded, no packets recorded from it");
    else if (o->has_audio)
        snprintf(l[3], sizeof l[3], "not downloaded -- space marks it, d fetches");
    else
        snprintf(l[3], sizeof l[3], "nothing to fetch: SatNOGS holds no recording");

    // The row can only show as much of a note as the width allows, so
    // the pane carries it in full -- and says how to write one.
    const char *note = note_for(o->id);
    if (note[0] != '\0')
        snprintf(l[4], sizeof l[4], "note: %s", note);
    else
        snprintf(l[4], sizeof l[4], "no note -- Enter writes one");

    for (int i = 0; i < 5 && i < height; i++)
        put(top_row + i, 1, cols - 1, A_NORMAL, l[i]);
}

static void draw_job(int top_row, int height, int cols)
{
    if (height <= 0) return;

    char head[512];
    if (g_job.total > 0)
        snprintf(head, sizeof head, "%s -- %d of %d done%s%s",
                 g_job.title, g_job.done, g_job.total,
                 g_job.failed ? ", " : "",
                 g_job.failed ? "some failed" : "");
    else
        snprintf(head, sizeof head, "%s", g_job.title);
    put(top_row, 1, cols - 1, A_BOLD, head);

    // A progress bar only means something when we know the total.
    int first_log = top_row + 1;
    if (g_job.total > 0 && height > 2) {
        int barw = cols - 4;
        if (barw > 60) barw = 60;
        if (barw > 4) {
            int fill = (int)((long)barw * g_job.done / g_job.total);
            char bar[80];
            int p = 0;
            bar[p++] = '[';
            for (int i = 0; i < barw && p < (int)sizeof bar - 2; i++)
                bar[p++] = (i < fill) ? '#' : '.';
            bar[p++] = ']';
            bar[p] = '\0';
            put(top_row + 1, 1, cols - 1, A_NORMAL, bar);
            first_log = top_row + 2;
        }
    }

    int log_h = top_row + height - first_log;
    if (log_h <= 0) return;
    int start = g_job.n_lines - log_h;
    if (start < 0) start = 0;
    for (int i = 0; i + start < g_job.n_lines && i < log_h; i++)
        put(first_log + i, 1, cols - 1, A_NORMAL, g_job.lines[start + i]);
}

static void draw_bottom_bar(int rows, int cols)
{
    char line[512];
    if (g_status[0] != '\0')
        snprintf(line, sizeof line, " %s", g_status);
    else if (g_job.running)
        snprintf(line, sizeof line, " c cancel   q quit   ? help");
    else
        snprintf(line, sizeof line,
                 " j/k move  h/l day  t today  / date  r list  "
                 "space mark  a all  D data  n none  f filter  d get  p decode  q quit  ? help");
    put(rows - 1, 0, cols, attr_for(PAIR_BAR), line);
}

static void draw_help(int rows, int cols)
{
    static const char *const help[] = {
        "satnogs_browser -- what SatNOGS has for this satellite, against what we hold",
        "",
        "  j, k / arrows / PgUp / PgDn         move the selection",
        "  gg, G / Home, End                   first, last observation",
        "  zz                                  centre the selection",
        "  h, l / left, right                  previous, next UTC day",
        "  t                                   today",
        "  /                                   go to a date (YYYY-MM-DD)",
        "  r                                   list this day from SatNOGS",
        "  Enter                               write a note on this observation",
        "  space                               mark or unmark an observation",
        "  a                                   mark everything in view with a recording",
        "  D                                   mark the ones that carried data",
        "  n                                   clear all marks",
        "  f, F                                cycle the filter either way",
        "  d                                   download what is marked",
        "  p                                   decode what is marked into the database",
        "  c                                   cancel a running job",
        "  q, Esc                              quit",
        "",
        "Only the day listing costs a SatNOGS request -- about fifty for a busy",
        "day, and nothing at all to revisit it, because the listing is cached on",
        "disk. Downloading audio is not rate limited: it comes from object",
        "storage rather than the API.",
        "",
        "Green rows are finished: the audio is here and packets from it are in",
        "the database. Yellow means the recording is here and nothing has read",
        "it yet -- mark those and press p, which runs the same decoder as the",
        "nightly batch and skips any file already decoded.",
        "",
        "An observation shows as `no audio` when SatNOGS itself holds no",
        "recording, which is roughly one pass in three. Those cannot be fetched",
        "by anyone; they are shown so the day's coverage reads honestly.",
        "",
        "The `data` column counts the frames SatNOGS demodulated from the pass",
        "itself. One or two is a beacon; dozens means the pass carried a bulk",
        "download, which is the quickest way to find the passes worth having. A",
        "dash means nothing here knows the count yet -- press r to list the day.",
        "",
        "A note written with Enter is your own line about the pass -- what it",
        "carried, why it matters. It shows at the right of the row, cut with an",
        "ellipsis when the screen is too narrow for it, and in full below. Notes",
        "live in .notes.tsv at the top of the archive; emptying one deletes it.",
        "",
        "A day with no listing still shows what this station holds for it: the",
        "archive is read at startup and merged into every day. `held` in the",
        "status column means the recording is here but SatNOGS has not been",
        "asked about that day yet.",
        "",
        // Stands in for the archive path, filled in below: knowing
        // which tree is being read is the first thing to check when the
        // rows are not the ones you expected.
        "%ARCHIVE%",
        NULL,
    };
    int n = 0;
    while (help[n] != NULL) n++;

    // A page at a time, because the text is longer than a short
    // terminal and the archive path at the end of it is the one line
    // worth reading when the rows look wrong.
    int per_page = rows - 3;
    if (per_page < 1) per_page = 1;

    nodelay(stdscr, FALSE);
    for (int first = 0; first < n; first += per_page) {
        erase();
        for (int i = first; i < first + per_page && i < n; i++) {
            char line[1024];
            const char *text = help[i];
            if (strcmp(text, "%ARCHIVE%") == 0) {
                snprintf(line, sizeof line, "archive: %s   (%d held, %d decoded)",
                         g_archive, g_n_local, g_n_decoded);
                text = line;
            }
            put(i - first + 1, 2, cols - 4, A_NORMAL, text);
        }
        put(rows - 1, 0, cols, attr_for(PAIR_BAR),
            first + per_page < n ? " any key for more" : " any key to go back");
        refresh();
        getch();
    }
    nodelay(stdscr, TRUE);
}

// Second half of a vi two-key motion: returns 1 when the key that
// follows is the one that completes it. Waits about a second, so a
// dropped keystroke leaves the browser where it was rather than
// hanging; the main loop's timeout is restored by its next pass.
static int second_key(int want)
{
    timeout(1000);
    int ch = getch();
    return ch == want;
}

// Modal one-line prompt on the bottom bar. Returns 1 if the user
// confirmed with Enter, 0 if they backed out.
static int prompt(const char *label, char *buf, size_t cap, int rows, int cols)
{
    int cursor = (int)strlen(buf);
    nodelay(stdscr, FALSE);
    curs_set(1);
    int ok = 0;
    for (;;) {
        char line[512];
        snprintf(line, sizeof line, " %s%s", label, buf);
        put(rows - 1, 0, cols, attr_for(PAIR_BAR), line);
        move(rows - 1, (int)strlen(label) + 1 + cursor);
        refresh();

        int ch = getch();
        if (ch == '\n' || ch == '\r' || ch == KEY_ENTER) { ok = 1; break; }
        if (ch == 27) break;
        if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) ui_tf_backspace(buf, &cursor);
        else if (ch == KEY_LEFT)  ui_tf_left(&cursor);
        else if (ch == KEY_RIGHT) ui_tf_right(buf, &cursor);
        else if (ch >= 32 && ch < 127) ui_tf_insert(buf, cap, &cursor, ch);
    }
    curs_set(0);
    nodelay(stdscr, TRUE);
    return ok;
}

// ------------------------------------------------------------ argument

#define OPTW 22

typedef struct {
    const char *archive;
    const char *db;
    const char *pull;
    const char *decoder;
    const char *day;
    long        norad;
} args_t;

static int parse_args(args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        if (strncmp(arg, "--archive=", 10) == 0 || help) {
            if (help) parse_help_line(OPTW, "--archive=<dir>",
                                      "SatNOGS archive root (default: the "
                                      "FrontierSat data root's satnogs_archive)");
            else a->archive = arg + 10;
            matched = 1;
        }
        if (strncmp(arg, "--norad-id=", 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--norad-id=<n>",
                                      "NORAD catalog id (default 69015, FrontierSat)");
            else a->norad = strtol(arg + 11, NULL, 10);
            matched = 1;
        }
        if (strncmp(arg, "--day=", 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--day=<YYYY-MM-DD>",
                                      "open on this UTC day (default: today)");
            else a->day = arg + 6;
            matched = 1;
        }
        if (strncmp(arg, "--db=", 5) == 0 || help) {
            if (help) parse_help_line(OPTW, "--db=<path>",
                                      "packet database, read to mark which "
                                      "observations decoded");
            else a->db = arg + 5;
            matched = 1;
        }
        if (strncmp(arg, "--pull-script=", 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--pull-script=<path>",
                                      "satnogs_pull.sh to run (default: found on PATH)");
            else a->pull = arg + 14;
            matched = 1;
        }
        if (strncmp(arg, "--decode-script=", 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--decode-script=<path>",
                                      "decode_passes.sh to run (default: found on PATH)");
            else a->decoder = arg + 16;
            matched = 1;
        }
        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "satnogs_browser: unable to parse '%s'\n", arg);
            return PARSE_ERROR;
        }
    }
    if (help >= HELP_FULL) {
        printf("\nBrowses one UTC day of SatNOGS observations at a time and\n"
               "downloads the ones you mark. All network access goes through\n"
               "satnogs_pull.sh.\n");
    }
    return PARSE_OK;
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "satnogs_browser")) return 0;

    args_t a = {0};
    a.norad = 69015;
    switch (parse_args(&a, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }

    g_norad = a.norad;
    snprintf(g_archive, sizeof g_archive, "%s",
             a.archive ? a.archive : sso_satnogs_archive_dir());
    if (a.db != NULL)
        snprintf(g_db_path, sizeof g_db_path, "%s", a.db);
    else if (packet_db_default_path(g_db_path, sizeof g_db_path) != 0)
        g_db_path[0] = '\0';
    if (a.pull != NULL)
        snprintf(g_pull, sizeof g_pull, "%s", a.pull);
    if (a.decoder != NULL)
        snprintf(g_decoder, sizeof g_decoder, "%s", a.decoder);

    struct stat st;
    if (stat(g_archive, &st) != 0 || !S_ISDIR(st.st_mode)) {
        fprintf(stderr, "satnogs_browser: no archive directory at %s\n"
                        "(pass --archive=<dir>)\n", g_archive);
        return 1;
    }

    if (a.day != NULL && day_valid(a.day))
        snprintf(g_day, sizeof g_day, "%s", a.day);
    else
        day_today();

    // The archive is a shared setgid tree that cron writes as another
    // user, so anything fetched here has to stay group-writable. That
    // is a property of the archive rather than a preference of whoever
    // is running the browser, so set it here instead of asking the
    // operator to remember a umask on the command line.
    umask(0002);

    // Both of these walk a lot of the disk, so say what is happening --
    // on a real archive it is tens of thousands of directories and a
    // couple of hundred thousand database rows.
    fprintf(stderr, "satnogs_browser: reading %s ...\n", g_archive);
    scan_local_archive();
    load_decoded_ids();
    notes_load();
    fprintf(stderr, "satnogs_browser: %d observation%s held locally\n",
            g_n_local, g_n_local == 1 ? "" : "s");

    if (initscr() == NULL) {
        fprintf(stderr, "satnogs_browser: ncurses initscr failed\n");
        return 1;
    }
    cbreak();
    noecho();
    nonl();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    set_escdelay(25);
    curs_set(0);
    if (has_colors()) {
        start_color();
        use_default_colors();
        short sel_bg = (COLORS >= 256) ? 240 : COLOR_WHITE;
        init_pair(PAIR_BAR,     COLOR_WHITE,   COLOR_BLUE);
        // Green is the finished state -- audio here and packets in the
        // database. Yellow is the half-done one: the recording is on
        // the disk and nothing has read it yet.
        init_pair(PAIR_DECODED, COLOR_GREEN,   -1);
        init_pair(PAIR_HAVE,    COLOR_YELLOW,  -1);
        init_pair(PAIR_NOAUDIO, COLOR_RED,     -1);
        init_pair(PAIR_MARKED,  COLOR_CYAN,    -1);
        init_pair(PAIR_SEL,     COLOR_WHITE,   sel_bg);
        init_pair(PAIR_WARN,    COLOR_YELLOW,  -1);
        g_have_color = 1;
    }

    load_day();
    rebuild_view();

    int quit = 0;
    while (!quit) {
        job_poll();

        int rows = LINES, cols = COLS;
        // Four rows are not the list or the pane: the day bar, the
        // headings, the separator and the key hints. Counting only
        // three of them handed the pane a row the bottom bar then
        // painted over, which is where the last line of a note -- or of
        // a job's output -- used to go.
        int pane_h = g_job.running || g_job.n_lines > 0 ? 8 : 6;
        int list_h = rows - 4 - pane_h;
        if (list_h < 3) { list_h = 3; pane_h = rows - 4 - list_h; }
        if (pane_h < 1) pane_h = 1;

        erase();
        draw_top_bar(cols);
        draw_headings(cols);
        draw_list(2, list_h, cols);
        mvhline(2 + list_h, 0, ACS_HLINE, cols);
        if (g_job.running || g_job.n_lines > 0)
            draw_job(3 + list_h, pane_h, cols);
        else
            draw_detail(3 + list_h, pane_h, cols);
        draw_bottom_bar(rows, cols);
        refresh();

        // A short wait keeps a running job's output flowing without
        // spinning the CPU when nothing is happening.
        timeout(g_job.running ? 120 : 400);
        int ch = getch();
        if (ch == ERR) continue;
        g_status[0] = '\0';
        // A finished job's output stays on screen until the operator
        // touches something, then the pane goes back to describing
        // whatever the cursor is on.
        if (!g_job.running) g_job.n_lines = 0;

        obs_t *cur = (g_n_view > 0 && g_sel < g_n_view)
                   ? &g_rows[g_view[g_sel]] : NULL;

        switch (ch) {
            case 'q': case 'Q': case 27:
                if (g_job.running) set_status("a job is running -- c cancels it first");
                else quit = 1;
                break;
            case KEY_UP:   case 'k': if (g_sel > 0) g_sel--; break;
            case KEY_DOWN: case 'j': if (g_sel + 1 < g_n_view) g_sel++; break;
            case KEY_PPAGE: g_sel -= list_h; if (g_sel < 0) g_sel = 0; break;
            case KEY_NPAGE: g_sel += list_h;
                            if (g_sel >= g_n_view) g_sel = g_n_view ? g_n_view - 1 : 0;
                            break;
            case KEY_HOME:            g_sel = 0; break;
            case KEY_END:  case 'G':  g_sel = g_n_view ? g_n_view - 1 : 0; break;
            case KEY_LEFT:  case 'h': day_shift(-1); reload_day(); break;
            case KEY_RIGHT: case 'l': day_shift(1);  reload_day(); break;
            case 't':                 day_today();   reload_day(); break;
            // vi's two-key motions. The second key is read here rather
            // than kept as a mode, so a stray g or z does nothing at
            // all instead of arming something the next keystroke fires.
            case 'g':
                if (second_key('g')) g_sel = 0;
                break;
            case 'z':
                if (second_key('z')) {
                    g_top = g_sel - list_h / 2;
                    if (g_top < 0) g_top = 0;
                }
                break;
            case '/': {
                // Empty, not pre-filled with the current day: with the
                // cursor at the end of a seeded date, typing appends to
                // it instead of replacing it, which is never what you
                // meant by asking to go somewhere.
                char buf[16] = "";
                if (prompt("go to date (YYYY-MM-DD): ", buf, sizeof buf, rows, cols)) {
                    if (day_valid(buf)) go_to_day(buf);
                    else set_status("'%s' is not a YYYY-MM-DD date", buf);
                }
                break;
            }
            case '\n': case '\r': case KEY_ENTER: {
                if (cur == NULL) break;
                // Seeded with whatever is already there, so Enter is
                // how a note is written, corrected and (by emptying it)
                // taken away again.
                char buf[NOTE_MAX];
                snprintf(buf, sizeof buf, "%s", note_for(cur->id));
                const char *label = "note: ";
                // The bar does not scroll, so don't accept more than it
                // can show -- the rest would be typed blind.
                size_t cap = sizeof buf;
                if (cols > (int)strlen(label) + 8
                    && (size_t)cols - strlen(label) - 2 < cap)
                    cap = (size_t)cols - strlen(label) - 2;
                if (prompt(label, buf, cap, rows, cols)) {
                    if (note_set(cur->id, buf))
                        set_status(buf[0] ? "note saved" : "note removed");
                    else
                        set_status("could not write %s/.notes.tsv", g_archive);
                }
                break;
            }
            case 'r':
                if (g_job.running) set_status("a job is already running");
                else job_start_listing();
                break;
            case ' ':
                if (cur == NULL) break;
                if (!markable(cur)) {
                    set_status("SatNOGS holds no audio for that pass");
                } else {
                    cur->marked = !cur->marked;
                    if (g_sel + 1 < g_n_view) g_sel++;
                }
                break;
            case 'a': {
                int n = 0;
                for (int i = 0; i < g_n_view; i++) {
                    obs_t *o = &g_rows[g_view[i]];
                    if (markable(o) && !o->marked) { o->marked = 1; n++; }
                }
                set_status("marked %d", n);
                break;
            }
            case 'D': {
                // The passes that carried something. Marking adds, the
                // way a does, so D after a filter or a marks a few more
                // rather than starting the selection over.
                int n = 0, unknown = 0;
                for (int i = 0; i < g_n_view; i++) {
                    obs_t *o = &g_rows[g_view[i]];
                    if (o->n_data < 0) unknown++;
                    if (markable(o) && o->n_data > 0 && !o->marked) {
                        o->marked = 1;
                        n++;
                    }
                }
                if (n == 0 && unknown > 0)
                    set_status("no count for %d of these -- press r to list the day",
                               unknown);
                else
                    set_status("marked %d with data", n);
                break;
            }
            case 'p':
                if (g_job.running) set_status("a job is already running");
                else job_start_decode();
                break;
            case 'n': clear_marks(); set_status("marks cleared"); break;
            case 'f': g_filter = (g_filter + 1) % FLT_N; rebuild_view(); break;
            case 'F': g_filter = (g_filter + FLT_N - 1) % FLT_N; rebuild_view(); break;
            case 'd':
                if (g_job.running) set_status("a job is already running");
                else job_start_download();
                break;
            case 'c': job_cancel(); break;
            case '?': draw_help(rows, cols); break;
            default: break;
        }
    }

    endwin();
    free(g_job.queue);
    free(g_decoded_ids);
    free(g_local);
    free(g_notes);
    return 0;
}

#endif // SATNOGS_BROWSER_HAVE_NCURSES
