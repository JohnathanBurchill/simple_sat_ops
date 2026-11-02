/*

    Simple Satellite Operations  utils/satnogs_passes.c

    Curses grid of when the satellite's SatNOGS observations around now
    start, five minutes to a cell -- UTC and local side by side, how
    many stations, and how many observations begin in that slot --
    filling columns top to bottom and left to right like a timetable. A
    line cuts in where the past ends and the schedule begins. Behind it
    the stations are the ones that got at least one frame out of the
    pass -- SatNOGS's own count of what it demodulated -- and ahead of
    it they are the ones that have booked it; a station books one
    observation per pass, so the observation count is left blank there.
    Green is a slot in which some station received a frame, white a
    past slot in which none did, and cyan one still to come. It is
    there to answer two questions at a glance: is the satellite still
    transmitting, and is the network still scheduled to listen. It
    reads nothing of this station's own.

    The frame counts climb while stations upload, so the hour holding now
    and the one before it are re-listed every quarter of an hour.

    It never talks to the network itself. Every listing goes through
    satnogs_list_hour.sh, which caches one UTC hour per file and shares
    the cron job's archive lock and request tally with satnogs_pull.sh.
    An hour rather than the day satnogs_browser lists: FrontierSat draws
    about 600 observations a day across the network, so a day is twenty-
    odd requests at nearly three seconds each, where an hour is one or
    two -- and listing outward from the hour holding now puts the passes
    that matter on screen in a few seconds.

    What decides when an hour is listed again is how much it can still
    change. An hour that ended more than SETTLE_HOURS before it was last
    listed is final and is never asked about again. Any other hour --
    still being observed, scheduled or uploaded -- is re-listed once its
    listing is older than --ttl. Requests count against a ceiling of 60
    or 240 an hour, and a runaway client got this station's address
    blocked in August 2026, so a listing only starts when it fits inside
    half that ceiling, and one that fails or changes nothing backs off
    before it is tried again.

    Keys (vi motions alongside the arrows):
      q | Esc          quit
      h | l | Left|Rt  scroll a column
      PgUp / PgDn      scroll a screenful
      g | G            first / last column
      n                back to now
      r                re-list every hour in the window from SatNOGS

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

// strptime and timegm are outside the C99 standard set on glibc.
#define _GNU_SOURCE

#include "argparse.h"
#include "sso_paths.h"
#include "sso_version.h"

#include <errno.h>
#include <fcntl.h>
#include <locale.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#if !defined(SATNOGS_PASSES_HAVE_NCURSES)
int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    fprintf(stderr,
            "satnogs_passes: built without ncurses. Install\n"
            "libncurses-dev and rebuild.\n");
    return 1;
}
#else

#include <ncurses.h>

// A day of FrontierSat runs to about 600 observations, scheduled ones
// included, and the window can reach two days.
#define MAX_ROWS        4096
// Hours in the window: 48 of them, and the one either end is cut by.
#define MAX_SLICES      50
#define MAX_WINDOW_H    48
// A listing made this long after its hour ended is final: the audio has
// been uploaded and SatNOGS's frame counts are in.
#define SETTLE_HOURS    12
// How long to leave an hour alone after a listing of it failed, or
// finished without changing the cache (the listing script exits quietly
// when the cron run holds the archive lock).
#define RETRY_BACKOFF_S (10 * 60)
// The hour holding now and the one before it are re-listed this often,
// whatever --ttl says: their passes are still being uploaded, and the
// count of frames SatNOGS demodulated climbs as they are.
#define RECENT_TTL_S    (15 * 60)
// Observations are shown five minutes to a cell: a busy hour has a
// few dozen, from stations all over the network, and what is worth
// seeing is when the satellite was being heard, not each station.
#define BIN_S           300
// A cell is "HH:MM  HH:MM  stations  obs" -- UTC, local, how many
// stations, how many observations start in those five minutes -- and
// the gap after it.
#define CELL_W          27
#define CELL_GAP        3

enum { PAIR_BAR = 1, PAIR_HEARD, PAIR_PAST, PAIR_AHEAD, PAIR_WARN };

typedef struct {
    long   id;
    long   station;     // SatNOGS ground station id
    time_t t_start;
    time_t t_end;
    int    n_frames;    // frames SatNOGS demodulated from it
} obs_t;

// Five minutes of observations, by start time.
typedef struct {
    time_t t0;
    int    n;
    int    stations;        // distinct stations among them
    int    stations_heard;  // of those, with at least one frame
} bin_t;

// One UTC hour the window touches, and what is known about its listing.
// Kept across window moves so a back-off is not forgotten.
typedef struct {
    char   hour[16];     // YYYY-MM-DDTHH, the cache file's name
    time_t t0;           // the top of the hour, UTC
    time_t listed;       // cache file mtime, 0 if never listed
    time_t retry_after;  // no automatic listing before this
    int    forced;       // r asked for it
    int    n_obs;        // observations in the cache, -1 if never listed
} slice_t;

typedef struct {
    pid_t  pid;
    int    fd;
    int    running;
    int    slice;        // index into g_slices
    time_t listed_before;
    char   partial[256];
    int    partial_len;
    char   last[256];    // the script's latest line, for the status bar
} job_t;

static obs_t   g_rows[MAX_ROWS];
static int     g_n_rows = 0;
// The rows that have started, for the counts in the top bar.
static int     g_n_past = 0;
static bin_t   g_bins[MAX_ROWS];
static int     g_n_bins = 0;
// The bins that have begun: where the line between past and future
// falls, drawn as a cell of its own at that point.
static int     g_n_past_bins = 0;
static int     g_first_col = 0;   // the leftmost column on screen

static slice_t g_slices[MAX_SLICES];
static int     g_n_slices = 0;

static char    g_archive[768] = "";
static char    g_lister[768] = "satnogs_list_hour.sh";
static long    g_norad = 69015;
static long    g_back_s  = 6 * 3600;
static long    g_ahead_s = 6 * 3600;
static long    g_ttl_s   = 2 * 3600;
static int     g_have_color = 0;
static char    g_status[256] = "";
// Why no listing is starting although one is due, shown beside the
// listing ages rather than over the key hints.
static char    g_hold[128] = "";

static job_t   g_job = {0};


static void set_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_status, sizeof g_status, fmt, ap);
    va_end(ap);
}

static int attr_for(int pair)
{
    return g_have_color ? COLOR_PAIR(pair) : A_NORMAL;
}

static time_t parse_iso(const char *iso)
{
    struct tm tm = {0};
    if (strptime(iso, "%Y-%m-%dT%H:%M:%S", &tm) == NULL) return (time_t)-1;
    return timegm(&tm);
}

// ---------------------------------------------------------------- hours

static void slice_cache_path(const slice_t *d, char *out, size_t outn)
{
    snprintf(out, outn, "%s/.hourcache/%ld/%s.tsv", g_archive, g_norad, d->hour);
}

// The UTC hours the window touches, oldest first. An hour already known
// keeps its back-off and its forced flag.
static void window_slices(time_t now)
{
    slice_t old[MAX_SLICES];
    int n_old = g_n_slices;
    memcpy(old, g_slices, sizeof old);

    time_t first = ((now - g_back_s) / 3600) * 3600;
    time_t last  = ((now + g_ahead_s) / 3600) * 3600;
    g_n_slices = 0;
    for (time_t t = first; t <= last && g_n_slices < MAX_SLICES; t += 3600) {
        slice_t *d = &g_slices[g_n_slices++];
        memset(d, 0, sizeof *d);
        d->t0 = t;
        d->n_obs = -1;
        struct tm tm;
        gmtime_r(&t, &tm);
        strftime(d->hour, sizeof d->hour, "%Y-%m-%dT%H", &tm);
        const slice_t *was = NULL;
        for (int i = 0; i < n_old; i++)
            if (strcmp(old[i].hour, d->hour) == 0) { was = &old[i]; break; }
        if (was != NULL) {
            d->retry_after = was->retry_after;
            d->forced = was->forced;
        }
        char path[900];
        slice_cache_path(d, path, sizeof path);
        struct stat st;
        d->listed = (stat(path, &st) == 0) ? st.st_mtime : 0;
        if (d->listed == 0) continue;
        // Counted again only when the cache has been rewritten.
        if (was != NULL && was->listed == d->listed) {
            d->n_obs = was->n_obs;
            continue;
        }
        FILE *f = fopen(path, "r");
        if (f == NULL) continue;
        int c, n = 0;
        while ((c = getc(f)) != EOF)
            if (c == '\n') n++;
        fclose(f);
        d->n_obs = n;
    }
}

static int slice_final(const slice_t *d)
{
    return d->listed != 0
        && d->listed >= d->t0 + 3600 + SETTLE_HOURS * 3600;
}

static int slice_stale(const slice_t *d, time_t now)
{
    if (d->listed == 0) return 1;
    if (slice_final(d)) return 0;
    time_t this_hour = now / 3600 * 3600;
    long ttl = (d->t0 >= this_hour - 3600 && d->t0 <= this_hour)
             ? RECENT_TTL_S : g_ttl_s;
    return now - d->listed >= ttl;
}

// ---------------------------------------------------------------- rows

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

static int cmp_obs_start(const void *a, const void *b)
{
    const obs_t *x = a, *y = b;
    if (x->t_start != y->t_start) return (x->t_start > y->t_start) - (x->t_start < y->t_start);
    return (x->id > y->id) - (x->id < y->id);
}

// Read every hour cache in the window into g_rows, keeping the
// observations that overlap the window. Of the columns
// satnogs_list_hour.sh writes, these are the ones used: id, start, end,
// station id and, last, the count of frames SatNOGS demodulated.
static void load_rows(time_t now)
{
    g_n_rows = 0;
    time_t lo = now - g_back_s, hi = now + g_ahead_s;

    for (int s = 0; s < g_n_slices; s++) {
        char path[900];
        slice_cache_path(&g_slices[s], path, sizeof path);
        FILE *f = fopen(path, "r");
        if (f == NULL) continue;

        char line[2048];
        while (fgets(line, sizeof line, f) != NULL && g_n_rows < MAX_ROWS) {
            line[strcspn(line, "\r\n")] = '\0';
            char *fld[10] = {0};
            if (split_tabs(line, fld, 10) < 10) continue;

            obs_t o = {0};
            o.id = strtol(fld[0], NULL, 10);
            o.t_start = parse_iso(fld[1]);
            o.t_end = parse_iso(fld[2]);
            if (o.id <= 0 || o.t_start == (time_t)-1) continue;
            if (o.t_end == (time_t)-1) o.t_end = o.t_start;
            if (o.t_end < lo || o.t_start > hi) continue;
            o.station = strtol(fld[5], NULL, 10);
            o.n_frames = (int)strtol(fld[9], NULL, 10);
            g_rows[g_n_rows++] = o;
        }
        fclose(f);
    }
    qsort(g_rows, (size_t)g_n_rows, sizeof g_rows[0], cmp_obs_start);

    g_n_past = 0;
    while (g_n_past < g_n_rows && g_rows[g_n_past].t_start <= now) g_n_past++;

    // The rows are in start order, so each bin is a run of them. A
    // station is counted once per bin however many observations it
    // has in it, and once among the heard if any of them has a frame.
    g_n_bins = 0;
    int first = 0;
    for (int i = 0; i < g_n_rows; i++) {
        time_t t0 = g_rows[i].t_start / BIN_S * BIN_S;
        if (g_n_bins == 0 || g_bins[g_n_bins - 1].t0 != t0) {
            bin_t b = {0};
            b.t0 = t0;
            g_bins[g_n_bins++] = b;
            first = i;
        }
        bin_t *b = &g_bins[g_n_bins - 1];
        b->n++;
        int seen = 0, seen_heard = 0;
        for (int j = first; j < i; j++) {
            if (g_rows[j].station != g_rows[i].station) continue;
            seen = 1;
            if (g_rows[j].n_frames > 0) seen_heard = 1;
        }
        if (!seen) b->stations++;
        if (g_rows[i].n_frames > 0 && !seen_heard) b->stations_heard++;
    }
    // A bin under way counts as begun: some of its passes are, and the
    // line goes after it.
    g_n_past_bins = 0;
    while (g_n_past_bins < g_n_bins && g_bins[g_n_past_bins].t0 <= now)
        g_n_past_bins++;
}

// ----------------------------------------------------------------- api

// Listing requests and recordings fetched in the trailing hour, from
// the tally satnogs_pull.sh and satnogs_list_hour.sh keep beside the
// archive. Only the listings
// meet the API ceiling, and only they are what this tool spends.
static int api_listings_last_hour(void)
{
    static time_t checked = 0;
    static int    n_list = 0;

    time_t now = time(NULL);
    if (now == checked) return n_list;
    checked = now;
    n_list = 0;

    char path[800];
    snprintf(path, sizeof path, "%s/.api_stats.txt", g_archive);
    FILE *f = fopen(path, "r");
    if (f == NULL) return 0;
    long cutoff = (long)now - 3600;
    char line[256];
    while (fgets(line, sizeof line, f) != NULL) {
        long ts;
        int calls;
        char kind[16] = "list";
        int n = sscanf(line, "%ld %d %15s", &ts, &calls, kind);
        if (n < 2 || ts < cutoff || calls <= 0) continue;
        if (strcmp(kind, "audio") != 0) n_list += calls;
    }
    fclose(f);
    return n_list;
}

// A token lifts the throttle from 60 listings an hour to 240; the
// script looks for one in the environment and then in the archive.
static int api_ceiling(void)
{
    if (getenv("SATNOGS_API_TOKEN") != NULL) return 240;
    char path[800];
    snprintf(path, sizeof path, "%s/.api_token", g_archive);
    return access(path, R_OK) == 0 ? 240 : 60;
}

// ----------------------------------------------------------------- job

static int job_start(int slice)
{
    char out_arg[800], norad_arg[64], hour_arg[64];
    snprintf(out_arg,   sizeof out_arg,   "--out=%s", g_archive);
    snprintf(norad_arg, sizeof norad_arg, "--norad-id=%ld", g_norad);
    snprintf(hour_arg,  sizeof hour_arg,  "--hour=%s", g_slices[slice].hour);
    char *argv[] = { g_lister, out_arg, norad_arg, hour_arg, NULL };

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
        // A process group of its own, so quitting stops curl with it.
        setpgid(0, 0);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[0]);
        close(pipefd[1]);
        execvp(argv[0], argv);
        fprintf(stderr, "cannot run %s: %s\n", argv[0], strerror(errno));
        _exit(127);
    }
    close(pipefd[1]);
    fcntl(pipefd[0], F_SETFL, O_NONBLOCK);

    memset(&g_job, 0, sizeof g_job);
    g_job.pid = pid;
    g_job.fd = pipefd[0];
    g_job.running = 1;
    g_job.slice = slice;
    g_job.listed_before = g_slices[slice].listed;
    // Stays as it is if the script never says anything, which is the
    // cue to report its exit status instead.
    snprintf(g_job.last, sizeof g_job.last, "listing %s", g_slices[slice].hour);
    return 0;
}

// Drain the child's output and reap it. Returns 1 when it has just
// finished, so the caller re-reads the caches.
static int job_poll(void)
{
    if (!g_job.running) return 0;

    char buf[1024];
    ssize_t n;
    while ((n = read(g_job.fd, buf, sizeof buf)) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n' || g_job.partial_len == (int)sizeof g_job.partial - 1) {
                g_job.partial[g_job.partial_len] = '\0';
                if (g_job.partial_len > 0)
                    snprintf(g_job.last, sizeof g_job.last, "%s", g_job.partial);
                g_job.partial_len = 0;
            } else if (c != '\r') {
                g_job.partial[g_job.partial_len++] = c;
            }
        }
    }

    int st = 0;
    if (waitpid(g_job.pid, &st, WNOHANG) != g_job.pid) return 0;
    close(g_job.fd);
    g_job.running = 0;

    slice_t *d = &g_slices[g_job.slice];
    char path[900];
    slice_cache_path(d, path, sizeof path);
    struct stat sb;
    time_t listed = (stat(path, &sb) == 0) ? sb.st_mtime : 0;
    int ok = WIFEXITED(st) && WEXITSTATUS(st) == 0 && listed > g_job.listed_before;
    d->forced = 0;
    if (ok) {
        set_status("listed %s", d->hour);
    } else {
        // Either it failed, or the cron run held the lock and the
        // script stepped aside. Both mean leave the hour for a while.
        d->retry_after = time(NULL) + RETRY_BACKOFF_S;
        if (strncmp(g_job.last, "listing ", 8) == 0)
            set_status("listing %s did not complete (exit status %d)",
                       d->hour, WIFEXITED(st) ? WEXITSTATUS(st) : -1);
        else
            set_status("listing %s did not complete: %s", d->hour, g_job.last);
    }
    return 1;
}

// Listing requests an hour is expected to cost: one page per 25
// observations, by the size of its last listing. An hour never listed
// is guessed at two pages -- FrontierSat averages about 24 observations
// an hour, and the query reaches 20 minutes past the hour besides.
static int slice_pages(const slice_t *d)
{
    return d->n_obs < 0 ? 2 : d->n_obs / 25 + 1;
}

// Start the next listing that is due, if the hour's budget has room
// for the whole of it -- a listing cannot be stopped part way once its
// requests are going out. The hour holding now goes first, then the
// hours outward from it, later before earlier at each step: the passes
// nearest now are the ones being looked for.
static void job_schedule(time_t now)
{
    if (g_job.running) return;
    g_hold[0] = '\0';

    int order[MAX_SLICES], n = 0;
    int cur = 0;
    for (int i = 0; i < g_n_slices; i++)
        if (g_slices[i].t0 <= now) cur = i;
    order[n++] = cur;
    for (int step = 1; n < g_n_slices; step++) {
        if (cur + step < g_n_slices) order[n++] = cur + step;
        if (cur - step >= 0)         order[n++] = cur - step;
    }

    int used = api_listings_last_hour(), ceiling = api_ceiling();
    for (int k = 0; k < n; k++) {
        slice_t *d = &g_slices[order[k]];
        // The automatic refresh keeps to half the ceiling, leaving the
        // rest to the cron run and to anyone browsing. A re-list asked
        // for by hand may go further, but never near enough the ceiling
        // to draw a 429.
        int budget;
        if (d->forced)
            budget = ceiling * 9 / 10;
        else if (slice_stale(d, now) && now >= d->retry_after)
            budget = ceiling / 2;
        else
            continue;
        if (used + slice_pages(d) > budget) {
            snprintf(g_hold, sizeof g_hold,
                     "   waiting for request budget: %d of %d used this hour",
                     used, ceiling);
            return;
        }
        job_start(order[k]);
        return;
    }
}

// ---------------------------------------------------------------- draw

static void put(int row, int col, int maxw, int attr, const char *text)
{
    if (row < 0 || col < 0 || maxw <= 0) return;
    attron(attr);
    mvaddnstr(row, col, text, maxw);
    attroff(attr);
}

// "3h12m" for a span of seconds.
static void span(long s, char *out, size_t outn)
{
    if (s < 0) s = -s;
    if (s < 3600)       snprintf(out, outn, "%ldm", s / 60);
    else if (s < 86400) snprintf(out, outn, "%ldh%02ldm", s / 3600, (s % 3600) / 60);
    else                snprintf(out, outn, "%ldd%02ldh", s / 86400, (s % 86400) / 3600);
}

static void draw_top(int cols, time_t now)
{
    int heard = 0;
    for (int i = 0; i < g_n_past; i++)
        if (g_rows[i].n_frames > 0) heard++;

    char line[512];
    snprintf(line, sizeof line,
             " NORAD %ld   last %gh: %d observations, %d with frames   next %gh: %d scheduled"
             "   API %d/%d per hour",
             g_norad, g_back_s / 3600.0, g_n_past, heard,
             g_ahead_s / 3600.0, g_n_rows - g_n_past,
             api_listings_last_hour(), api_ceiling());
    attron(attr_for(PAIR_BAR) | A_BOLD);
    mvhline(0, 0, ' ', cols);
    attroff(attr_for(PAIR_BAR) | A_BOLD);
    put(0, 0, cols, attr_for(PAIR_BAR) | A_BOLD, line);

    // The state of the window's listings in one line: an hour's worth of
    // ages each would run off the screen, and what matters is whether
    // any are missing and how old the oldest is.
    int never = 0, final = 0;
    time_t oldest = 0;
    for (int i = 0; i < g_n_slices; i++) {
        const slice_t *d = &g_slices[i];
        if (d->listed == 0)      never++;
        else if (slice_final(d)) final++;
        else if (oldest == 0 || d->listed < oldest) oldest = d->listed;
    }
    char ages[512], item[64];
    snprintf(ages, sizeof ages, " %d hours listed", g_n_slices - never);
    if (never > 0) {
        snprintf(item, sizeof item, ", %d not yet", never);
        strncat(ages, item, sizeof ages - strlen(ages) - 1);
    }
    if (oldest != 0) {
        char age[24];
        span(now - oldest, age, sizeof age);
        snprintf(item, sizeof item, ", oldest %s ago", age);
        strncat(ages, item, sizeof ages - strlen(ages) - 1);
    }
    if (final > 0) {
        snprintf(item, sizeof item, ", %d final", final);
        strncat(ages, item, sizeof ages - strlen(ages) - 1);
    }
    if (g_job.running) {
        snprintf(item, sizeof item, "   listing %s:00 UTC...",
                 g_slices[g_job.slice].hour + 5);
        strncat(ages, item, sizeof ages - strlen(ages) - 1);
    }
    strncat(ages, g_hold, sizeof ages - strlen(ages) - 1);
    put(1, 0, cols, A_DIM, ages);
}

// Columns the screen has room for, and how many there are in all: the
// observations plus the cell the line between past and future takes.
static int grid_cols_on_screen(int cols)
{
    int n = (cols + CELL_GAP) / (CELL_W + CELL_GAP);
    return n > 0 ? n : 1;
}

static int grid_cols_total(int height)
{
    return (g_n_bins + 1 + height - 1) / height;
}

// Scroll so the line between past and future is on screen, a column in
// from the left where there is room, so the passes just gone show too.
static void scroll_to_now(int height, int cols)
{
    int col = g_n_past_bins / height;
    int shown = grid_cols_on_screen(cols);
    g_first_col = col - (shown > 2 ? 1 : 0);
    if (g_first_col < 0) g_first_col = 0;
}

static void draw_grid(int top_row, int height, int cols, time_t now)
{
    // The first row is the headings; the cells fill the rest.
    if (height < 2) return;
    int shown = grid_cols_on_screen(cols);
    int total = grid_cols_total(height - 1);
    if (g_first_col > total - shown) g_first_col = total - shown;
    if (g_first_col < 0) g_first_col = 0;
    // Only the columns that have cells in them: five minutes to a cell
    // leaves most of a wide screen empty, and headings over nothing
    // make it look full.
    if (shown > total - g_first_col) shown = total - g_first_col;

    // Column headings, the zone each half of a cell is in.
    char tz[16];
    struct tm ntm;
    localtime_r(&now, &ntm);
    strftime(tz, sizeof tz, "%Z", &ntm);
    char head[64];
    snprintf(head, sizeof head, "%-5s  %-5s  %8s  %3s", "UTC", tz, "stations", "obs");
    for (int c = 0; c < shown; c++)
        put(top_row, c * (CELL_W + CELL_GAP), CELL_W, A_BOLD, head);
    top_row++;
    height--;
    if (height < 1) return;

    if (g_n_rows == 0)
        put(top_row + 1, 2, cols - 2, attr_for(PAIR_WARN),
            g_job.running ? "listing from SatNOGS..."
                          : "nothing listed for this window yet -- press r to list it");

    int per_col = height;
    for (int c = 0; c < shown; c++) {
        int x = c * (CELL_W + CELL_GAP);
        for (int r = 0; r < per_col; r++) {
            int k = (g_first_col + c) * per_col + r;
            int y = top_row + r;
            if (k == g_n_past_bins) {
                // The present, as a rule across the cell with the time
                // on it.
                char lbl[32], hm[16];
                strftime(hm, sizeof hm, "%H:%M", gmtime_r(&now, &ntm));
                snprintf(lbl, sizeof lbl, " now %s ", hm);
                attron(attr_for(PAIR_AHEAD) | A_BOLD);
                mvhline(y, x, ACS_HLINE, CELL_W);
                attroff(attr_for(PAIR_AHEAD) | A_BOLD);
                put(y, x + 3, CELL_W - 3, attr_for(PAIR_AHEAD) | A_BOLD, lbl);
                continue;
            }
            int i = k > g_n_past_bins ? k - 1 : k;
            if (i >= g_n_bins) break;
            const bin_t *b = &g_bins[i];

            // Behind the line the stations are the ones that heard the
            // pass; ahead of it, the ones that have booked it, where
            // the observation count would only say the same again.
            int ahead = i >= g_n_past_bins;
            char utc[16], loc[16], obs[16] = "", cell[64];
            struct tm tm;
            strftime(utc, sizeof utc, "%H:%M", gmtime_r(&b->t0, &tm));
            strftime(loc, sizeof loc, "%H:%M", localtime_r(&b->t0, &tm));
            if (!ahead) snprintf(obs, sizeof obs, "%d", b->n);
            snprintf(cell, sizeof cell, "%s  %s  %8d  %3s", utc, loc,
                     ahead ? b->stations : b->stations_heard, obs);

            int attr;
            if (ahead)                   attr = attr_for(PAIR_AHEAD);
            else if (b->stations_heard)  attr = attr_for(PAIR_HEARD) | A_BOLD;
            else                         attr = attr_for(PAIR_PAST);
            put(y, x, CELL_W, attr, cell);
        }
    }
}

static void draw_bottom(int rows, int cols)
{
    attron(attr_for(PAIR_BAR));
    mvhline(rows - 1, 0, ' ', cols);
    attroff(attr_for(PAIR_BAR));
    if (g_status[0] != '\0') {
        char line[300];
        snprintf(line, sizeof line, " %s", g_status);
        put(rows - 1, 0, cols, attr_for(PAIR_BAR), line);
        return;
    }
    // The key to the colours, written in them.
    const char *keys = " q quit  h/l scroll  n now  r re-list   ";
    int x = (int)strlen(keys);
    put(rows - 1, 0, cols, attr_for(PAIR_BAR), keys);
    put(rows - 1, x, cols - x, A_BOLD | attr_for(PAIR_HEARD), " frames received ");
    x += 17;
    put(rows - 1, x, cols - x, attr_for(PAIR_PAST), " no frames ");
    x += 11;
    put(rows - 1, x, cols - x, attr_for(PAIR_AHEAD), " planned ");
}

// ------------------------------------------------------------ argument

#define OPTW 22

typedef struct {
    const char *archive;
    const char *lister;
    long        norad;
    double      back_h;
    double      ahead_h;
    double      ttl_min;
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
        if (strncmp(arg, "--back=", 7) == 0 || help) {
            if (help) parse_help_line(OPTW, "--back=<hours>",
                                      "how far back to show (default 6)");
            else a->back_h = atof(arg + 7);
            matched = 1;
        }
        if (strncmp(arg, "--ahead=", 8) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ahead=<hours>",
                                      "how far ahead to show (default 6)");
            else a->ahead_h = atof(arg + 8);
            matched = 1;
        }
        if (strncmp(arg, "--ttl=", 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ttl=<minutes>",
                                      "re-list an hour still changing once its "
                                      "listing is this old (default 120)");
            else a->ttl_min = atof(arg + 6);
            matched = 1;
        }
        if (strncmp(arg, "--list-script=", 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--list-script=<path>",
                                      "satnogs_list_hour.sh to run (default: found on PATH)");
            else a->lister = arg + 14;
            matched = 1;
        }
        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "satnogs_passes: unable to parse '%s'\n", arg);
            return PARSE_ERROR;
        }
    }
    return PARSE_OK;
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "satnogs_passes")) return 0;

    args_t a = {0};
    a.norad = 69015;
    a.back_h = 6;
    a.ahead_h = 6;
    a.ttl_min = 120;
    switch (parse_args(&a, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }
    // Each hour in the window is a listing or two to keep fresh, so the
    // window is capped at two days. The floor on the TTL keeps a typo
    // from turning the refresh into a loop against the API.
    if (a.back_h < 0 || a.ahead_h < 0 || a.back_h + a.ahead_h > MAX_WINDOW_H) {
        fprintf(stderr, "satnogs_passes: --back and --ahead must be non-negative "
                        "and add up to at most %d hours\n", MAX_WINDOW_H);
        return 1;
    }
    if (a.ttl_min < 15) {
        fprintf(stderr, "satnogs_passes: --ttl must be at least 15 minutes\n");
        return 1;
    }
    g_norad   = a.norad;
    g_back_s  = (long)(a.back_h * 3600);
    g_ahead_s = (long)(a.ahead_h * 3600);
    g_ttl_s   = (long)(a.ttl_min * 60);

    snprintf(g_archive, sizeof g_archive, "%s",
             a.archive ? a.archive : sso_satnogs_archive_dir());
    if (a.lister != NULL)
        snprintf(g_lister, sizeof g_lister, "%s", a.lister);

    struct stat st;
    if (stat(g_archive, &st) != 0 || !S_ISDIR(st.st_mode)) {
        fprintf(stderr, "satnogs_passes: no archive directory at %s\n"
                        "(pass --archive=<dir>)\n", g_archive);
        return 1;
    }
    // The archive is a shared setgid tree that cron writes as another
    // user, so the caches written from here stay group-writable.
    umask(0002);


    // Before initscr, so the line-drawing characters come out as lines.
    setlocale(LC_ALL, "");
    if (initscr() == NULL) {
        fprintf(stderr, "satnogs_passes: ncurses initscr failed\n");
        return 1;
    }
    cbreak();
    noecho();
    nonl();
    keypad(stdscr, TRUE);
    set_escdelay(25);
    curs_set(0);
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(PAIR_BAR,     COLOR_WHITE,  COLOR_BLUE);
        init_pair(PAIR_HEARD, COLOR_GREEN,  -1);
        init_pair(PAIR_PAST,    COLOR_WHITE,  -1);
        init_pair(PAIR_AHEAD,   COLOR_CYAN,   -1);
        init_pair(PAIR_WARN,    COLOR_YELLOW, -1);
        g_have_color = 1;
    }

    time_t now = time(NULL);
    window_slices(now);
    load_rows(now);
    // Four rows are not the grid: the two at the top, the column
    // headings and the key hints.
    scroll_to_now(LINES - 4, COLS);

    time_t last_load = now;
    int had_rows = g_n_rows > 0;
    int quit = 0;
    while (!quit) {
        now = time(NULL);
        int finished = job_poll();
        // The window slides with the clock, so the rows are re-read
        // every half minute whether or not anything was listed.
        if (finished || now - last_load >= 30) {
            window_slices(now);
            load_rows(now);
            last_load = now;
            // The first listing to land decides where now is on the
            // grid; after that the scrolling is the operator's.
            if (!had_rows && g_n_rows > 0) scroll_to_now(LINES - 4, COLS);
            had_rows = g_n_rows > 0;
        }
        job_schedule(now);

        int rows = LINES, cols = COLS;
        int grid_h = rows - 4;
        if (grid_h < 1) grid_h = 1;

        erase();
        draw_top(cols, now);
        draw_grid(2, grid_h + 1, cols, now);
        draw_bottom(rows, cols);
        refresh();

        timeout(g_job.running ? 150 : 1000);
        int ch = getch();
        if (ch == ERR) continue;
        g_status[0] = '\0';

        int shown = grid_cols_on_screen(cols);
        switch (ch) {
            case 'q': case 'Q': case 27: quit = 1; break;
            case KEY_LEFT:  case 'h': g_first_col--; break;
            case KEY_RIGHT: case 'l': g_first_col++; break;
            case KEY_PPAGE: g_first_col -= shown; break;
            case KEY_NPAGE: g_first_col += shown; break;
            case KEY_HOME: case 'g': g_first_col = 0; break;
            case KEY_END:  case 'G': g_first_col = grid_cols_total(grid_h); break;
            case 'n': scroll_to_now(grid_h, cols); break;
            case 'r':
                for (int i = 0; i < g_n_slices; i++) {
                    g_slices[i].forced = 1;
                    g_slices[i].retry_after = 0;
                }
                set_status("re-listing %d hour%s", g_n_slices, g_n_slices == 1 ? "" : "s");
                break;
            default: break;
        }
        // draw_grid clamps the right-hand end; the left is clamped here.
        if (g_first_col < 0) g_first_col = 0;
    }

    endwin();
    // A listing left running is stopped with the grid rather than left
    // to finish behind a shell prompt; its cache only lands when the
    // whole hour is in, so stopping it part way loses nothing.
    if (g_job.running) {
        kill(-g_job.pid, SIGTERM);
        waitpid(g_job.pid, NULL, 0);
    }
    return 0;
}

#endif // SATNOGS_PASSES_HAVE_NCURSES
