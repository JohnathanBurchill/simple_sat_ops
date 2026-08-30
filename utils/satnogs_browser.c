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

    Keys:
      q | Esc          quit (with a job running, ask first)
      Up / Down        move the selection
      PgUp / PgDn      move by a page
      Home / End       first / last observation
      Left / Right     previous / next day
      t                jump to today
      g                go to a date (YYYY-MM-DD)
      r                list this day from SatNOGS (or re-list it)
      space            mark / unmark the selected observation
      a                mark every fetchable observation in view
      n                clear all marks
      f | F            cycle the filter forwards / backwards
      d                download the marked observations
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

enum {
    PAIR_BAR = 1,
    PAIR_HAVE,      // already downloaded
    PAIR_DECODED,   // downloaded and yielded packets
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
    int    marked;
} obs_t;

// Filters, in the order `f` cycles them.
enum { FLT_ALL = 0, FLT_AUDIO, FLT_MISSING, FLT_HAVE, FLT_DECODED, FLT_N };
static const char *const FLT_NAME[FLT_N] = {
    "all", "with audio", "not downloaded", "downloaded", "decoded"
};

typedef struct {
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

// Read the day cache into g_rows and work out the local state of each
// observation. Returns 0 if there is no cache for the day yet.
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
        return 0;
    }

    char line[2048];
    while (fgets(line, sizeof line, f) != NULL) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            line[--len] = '\0';
        if (len == 0) continue;

        if (g_n_rows >= MAX_OBS_ROWS) { g_truncated = 1; break; }

        char *fld[9] = {0};
        int nf = split_tabs(line, fld, 9);
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

        o->downloaded = have_audio_on_disk(o->id);
        o->decoded    = o->downloaded && is_decoded(o->id);
        g_n_rows++;
    }
    fclose(f);
    return 1;
}

// A pass is worth fetching when SatNOGS has the audio and we do not.
static int fetchable(const obs_t *o)
{
    return o->has_audio && !o->downloaded;
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

// Switching days drops the marks: they name observations on the day
// they were made, and carrying them across would make `d` fetch things
// scrolled off the screen days ago.
static void go_to_day(const char *day)
{
    snprintf(g_day, sizeof g_day, "%s", day);
    clear_marks();
    load_day();
    rebuild_view();
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
    // drives the progress figure.
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
static int job_next_batch(void)
{
    if (g_job.queue == NULL || g_job.queue_pos >= g_job.queue_n) return 0;
    if (g_job.cancelled) return 0;

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

static void job_start_download(void)
{
    int n = count_marked();
    if (n == 0) { set_status("nothing marked"); return; }

    free(g_job.queue);
    memset(&g_job, 0, sizeof g_job);
    g_job.queue = malloc((size_t)n * sizeof *g_job.queue);
    if (g_job.queue == NULL) { set_status("out of memory"); return; }

    for (int i = 0; i < g_n_rows; i++)
        if (g_rows[i].marked) g_job.queue[g_job.queue_n++] = g_rows[i].id;
    g_job.total = g_job.queue_n;

    if (!job_next_batch()) set_status("could not start the download");
}

static void job_start_listing(void)
{
    free(g_job.queue);
    memset(&g_job, 0, sizeof g_job);

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

    // More ids waiting means this was one batch of several.
    if (!g_job.cancelled && job_next_batch()) return;

    // Everything the job could change is on disk now, so re-read it.
    load_day();
    rebuild_view();

    if (g_job.cancelled)
        set_status("cancelled after %d of %d", g_job.done, g_job.total);
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

    char line[512];
    snprintf(line, sizeof line,
             " %s UTC   %d obs  %d audio  %d have  %d decoded   filter: %s   %s",
             g_day, g_n_rows, audio, have, decoded,
             FLT_NAME[g_filter], g_cache_note);
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
             "   %-9s %-8s %5s %4s  %-8s %-8s %s",
             "id", "start", "len", "el", "status", "local", "station");
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

        char line[1024];
        snprintf(line, sizeof line,
                 " %c %-9ld %-8s %5s %4s  %-8s %-8s %s",
                 o->marked ? '*' : ' ',
                 o->id, hhmmss(o->start), len, el,
                 o->status, local_word(o), o->station);

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
    char l[4][1024];
    snprintf(l[0], sizeof l[0], "observation %ld   %s to %s",
             o->id, o->start, o->end);
    snprintf(l[1], sizeof l[1], "station %ld  %s   max elevation %.0f deg",
             o->station_id, o->station, o->max_el);
    snprintf(l[2], sizeof l[2], "SatNOGS status %s   waterfall %s   audio %s",
             o->status, o->waterfall,
             o->has_audio ? "available" : "none uploaded");
    if (o->downloaded)
        snprintf(l[3], sizeof l[3], "local %s/%ld/   %s",
                 g_archive, o->id,
                 o->decoded ? "packets in the database"
                            : "downloaded, no packets recorded from it");
    else if (o->has_audio)
        snprintf(l[3], sizeof l[3], "not downloaded -- space marks it, d fetches");
    else
        snprintf(l[3], sizeof l[3], "nothing to fetch: SatNOGS holds no recording");

    for (int i = 0; i < 4 && i < height; i++)
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
                 " arrows move   left/right day   t today  g date  r list   "
                 "space mark  a all  n none  f filter  d download  q quit  ? help");
    put(rows - 1, 0, cols, attr_for(PAIR_BAR), line);
}

static void draw_help(int rows, int cols)
{
    static const char *const help[] = {
        "satnogs_browser -- what SatNOGS has for this satellite, against what we hold",
        "",
        "  arrows / PgUp / PgDn / Home / End   move the selection",
        "  left, right                         previous, next UTC day",
        "  t                                   today",
        "  g                                   go to a date (YYYY-MM-DD)",
        "  r                                   list this day from SatNOGS",
        "  space                               mark or unmark an observation",
        "  a                                   mark every fetchable one in view",
        "  n                                   clear all marks",
        "  f, F                                cycle the filter either way",
        "  d                                   download what is marked",
        "  c                                   cancel a running job",
        "  q, Esc                              quit",
        "",
        "Only the day listing costs a SatNOGS request -- about fifty for a busy",
        "day, and nothing at all to revisit it, because the listing is cached on",
        "disk. Downloading audio is not rate limited: it comes from object",
        "storage rather than the API.",
        "",
        "An observation shows as `no audio` when SatNOGS itself holds no",
        "recording, which is roughly one pass in three. Those cannot be fetched",
        "by anyone; they are shown so the day's coverage reads honestly.",
        "",
        "press any key",
        NULL,
    };
    erase();
    for (int i = 0; help[i] != NULL && i + 1 < rows; i++)
        put(i + 1, 2, cols - 4, A_NORMAL, help[i]);
    refresh();
    nodelay(stdscr, FALSE);
    getch();
    nodelay(stdscr, TRUE);
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

    load_decoded_ids();

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
        init_pair(PAIR_HAVE,    COLOR_GREEN,   -1);
        init_pair(PAIR_DECODED, COLOR_CYAN,    -1);
        init_pair(PAIR_NOAUDIO, COLOR_RED,     -1);
        init_pair(PAIR_MARKED,  COLOR_YELLOW,  -1);
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
        int pane_h = g_job.running || g_job.n_lines > 0 ? 8 : 5;
        int list_h = rows - 3 - pane_h;
        if (list_h < 3) { list_h = 3; pane_h = rows - 3 - list_h; }
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
            case KEY_UP:    if (g_sel > 0) g_sel--; break;
            case KEY_DOWN:  if (g_sel + 1 < g_n_view) g_sel++; break;
            case KEY_PPAGE: g_sel -= list_h; if (g_sel < 0) g_sel = 0; break;
            case KEY_NPAGE: g_sel += list_h;
                            if (g_sel >= g_n_view) g_sel = g_n_view ? g_n_view - 1 : 0;
                            break;
            case KEY_HOME:  g_sel = 0; break;
            case KEY_END:   g_sel = g_n_view ? g_n_view - 1 : 0; break;
            case KEY_LEFT:  day_shift(-1); go_to_day(g_day); break;
            case KEY_RIGHT: day_shift(1);  go_to_day(g_day); break;
            case 't':       day_today();   go_to_day(g_day); break;
            case 'g': {
                char buf[16];
                snprintf(buf, sizeof buf, "%s", g_day);
                if (prompt("go to date (YYYY-MM-DD): ", buf, sizeof buf, rows, cols)) {
                    if (day_valid(buf)) go_to_day(buf);
                    else set_status("'%s' is not a YYYY-MM-DD date", buf);
                }
                break;
            }
            case 'r':
                if (g_job.running) set_status("a job is already running");
                else job_start_listing();
                break;
            case ' ':
                if (cur == NULL) break;
                if (!fetchable(cur)) {
                    set_status(cur->downloaded ? "already downloaded"
                                               : "SatNOGS holds no audio for that pass");
                } else {
                    cur->marked = !cur->marked;
                    if (g_sel + 1 < g_n_view) g_sel++;
                }
                break;
            case 'a': {
                int n = 0;
                for (int i = 0; i < g_n_view; i++) {
                    obs_t *o = &g_rows[g_view[i]];
                    if (fetchable(o) && !o->marked) { o->marked = 1; n++; }
                }
                set_status("marked %d", n);
                break;
            }
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
    return 0;
}

#endif // SATNOGS_BROWSER_HAVE_NCURSES
