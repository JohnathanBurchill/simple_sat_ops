/*

    Simple Satellite Operations  utils/tcmd_browser.c

    Curses TUI over the sent_tcmd table: a command explorer, the mirror
    of packet_browser. Browse the telecommands simple_sat_ops has
    transmitted (and any backfilled from tx.log by tcmd_import), with the
    count of responses the satellite returned for each. Press Enter on a
    command to open its responses - the tcmd_response packets sharing that
    command's ts_sent, in sequence order. Read-only; safe to run beside a
    live receiver filling the same database.

    Correlation key: the satellite echoes a command's @tssent value
    (ts_sent, unix-ms) in every response, stored as 8 little-endian bytes
    at offset 1 of a tcmd_response payload. Matching is the same
    substr(payload,2,8) blob-equality packet_browser uses.

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

#include "argparse.h"
#include "beacon_cts1.h"
#include "packet_db.h"
#include "sso_version.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#if !defined(WITH_SQLITE3)
int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    fprintf(stderr,
            "tcmd_browser: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#elif !defined(TCMD_BROWSER_HAVE_NCURSES)
int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    fprintf(stderr,
            "tcmd_browser: built without ncurses. Install\n"
            "libncurses-dev and rebuild.\n");
    return 1;
}
#else

#include <ncurses.h>
#include <sqlite3.h>

#define MAX_ROWS  5000   // sent_tcmd rows held in memory
#define MAX_RESP  4000   // responses shown for one command
#define MAX_RESP_TS 100000  // total tcmd_response ts_sent values indexed

// One transmitted telecommand.
typedef struct {
    sqlite3_int64 id;
    uint64_t  ts_sent_ms;
    int       has_tsexec;
    long long tsexec_ms;
    char      ts_tx[40];     // ts_transmitted (ISO-8601 UTC)
    char      tool[24];
    char      run[40];
    int       has_freq;
    long long freq_hz;
    int       has_gain;
    double    gain_db;
    int       resp_count;    // responses seen for this ts_sent
    char      cmd[600];      // command_text
} cmd_row_t;

// One response packet shown in the Enter sub-view.
typedef struct {
    sqlite3_int64 id;
    char ts[40];
    char line[200];          // "[seq/max] code=N(OK) 'text'"
} resp_t;

static cmd_row_t rows[MAX_ROWS];
static int       n_rows = 0;
static int       sel = 0, top = 0;
static char      like_text[128] = "";
static int       show_local_time = 0;
static int       g_have_color = 0;
// Response filter: 0 = all, 1 = answered (resp>0, the "green" ones),
// 2 = unanswered (resp==0). Cycled with `f`.
static int       resp_filter = 0;

static const char *resp_filter_label(void)
{
    switch (resp_filter) {
    case 1:  return "answered";
    case 2:  return "unanswered";
    default: return "all";
    }
}

// Response sub-view (Enter on a command).
static int    in_resp = 0;
static resp_t resps[MAX_RESP];
static int    resp_n = 0, resp_sel = 0, resp_top = 0;
static char   resp_header[768] = "";

// Sorted index of every tcmd_response's ts_sent, so a command's response
// count is a binary-search bounds lookup rather than a per-row scan.
static uint64_t g_resp_ts[MAX_RESP_TS];
static int      g_resp_ts_n = 0;

enum {
    PAIR_BAR = 1, PAIR_SEL, PAIR_DIM, PAIR_OK, PAIR_ERR, PAIR_NONE,
};

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static double monotonic_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

// Render a stored ISO-8601 UTC timestamp into the chosen display mode
// (UTC passthrough, or parsed and re-formatted to local). Same behaviour
// as packet_browser's column.
static void format_ts(const char *iso, char *out, size_t outn)
{
    if (iso == NULL || iso[0] == '\0') { if (outn) out[0] = '\0'; return; }
    if (!show_local_time) { snprintf(out, outn, "%s", iso); return; }
    int yr, mo, dd, hh, mm, ss, ms = 0;
    int got = sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2d.%3d",
                     &yr, &mo, &dd, &hh, &mm, &ss, &ms);
    if (got < 6) { snprintf(out, outn, "%s", iso); return; }
    struct tm utc = {0};
    utc.tm_year = yr - 1900; utc.tm_mon = mo - 1; utc.tm_mday = dd;
    utc.tm_hour = hh; utc.tm_min = mm; utc.tm_sec = ss;
    time_t epoch = timegm(&utc);
    if (epoch == (time_t)-1) { snprintf(out, outn, "%s", iso); return; }
    struct tm local;
    localtime_r(&epoch, &local);
    char base[40];
    strftime(base, sizeof base, "%Y-%m-%d %H:%M:%S", &local);
    const char *tz = tzname[local.tm_isdst > 0 ? 1 : 0];
    if (tz == NULL) tz = "";
    snprintf(out, outn, "%s.%03d %s", base, ms, tz);
}

// Humanize a unix-ms instant as "YYYY-MM-DDTHH:MM:SSZ" (UTC, seconds).
static void fmt_epoch_ms(uint64_t ms, char *out, size_t outn)
{
    time_t t = (time_t)(ms / 1000);
    struct tm tm;
    if (gmtime_r(&t, &tm) == NULL) { snprintf(out, outn, "?"); return; }
    strftime(out, outn, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

static int cmp_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

// Scan every tcmd_response payload once, decode its ts_sent (8 LE bytes
// at offset 1), and build a sorted array so resp_count_for() is a fast
// bounds lookup.
static void build_resp_index(sqlite3 *db)
{
    g_resp_ts_n = 0;
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT payload FROM packet WHERE packet_type=4", -1, &st, NULL)
        != SQLITE_OK) return;
    while (g_resp_ts_n < MAX_RESP_TS && sqlite3_step(st) == SQLITE_ROW) {
        const uint8_t *pl = sqlite3_column_blob(st, 0);
        int n = sqlite3_column_bytes(st, 0);
        if (pl == NULL || n < 9) continue;
        uint64_t v = 0;
        for (int i = 0; i < 8; i++) v |= (uint64_t)pl[1 + i] << (8 * i);
        g_resp_ts[g_resp_ts_n++] = v;
    }
    sqlite3_finalize(st);
    qsort(g_resp_ts, g_resp_ts_n, sizeof g_resp_ts[0], cmp_u64);
}

// Count responses for a ts_sent via lower/upper bound on the sorted index.
static int resp_count_for(uint64_t ts)
{
    int lo = 0, hi = g_resp_ts_n;
    while (lo < hi) { int m = (lo + hi) / 2; if (g_resp_ts[m] < ts) lo = m + 1; else hi = m; }
    int first = lo;
    hi = g_resp_ts_n;
    while (lo < hi) { int m = (lo + hi) / 2; if (g_resp_ts[m] <= ts) lo = m + 1; else hi = m; }
    return lo - first;
}

// Load sent_tcmd (optionally filtered by a LIKE on command_text), newest
// transmitted first, and stamp each row's response count.
static void run_query(sqlite3 *db)
{
    sqlite3_int64 prev_id = (n_rows > 0) ? rows[sel].id : -1;
    build_resp_index(db);

    char sql[512];
    // Clamp off after each append: a truncated snprintf returns the would-be
    // length, leaving off past sizeof sql so the next "sizeof sql - off"
    // (size_t) wraps huge and sql + off goes out of bounds.
    int off = snprintf(sql, sizeof sql,
        "SELECT id, ts_sent_ms, tsexec_ms, command_text, tx_freq_hz, "
        "tx_gain_db, source_tool, source_run, ts_transmitted "
        "FROM sent_tcmd WHERE 1=1");
    if (off < 0 || off > (int) sizeof sql) off = (int) sizeof sql;
    char like_pattern[256];
    int have_like = (like_text[0] != '\0');
    if (have_like) {
        snprintf(like_pattern, sizeof like_pattern, "%%%s%%", like_text);
        off += snprintf(sql + off, sizeof sql - off,
                        " AND command_text LIKE ?1");
        if (off > (int) sizeof sql) off = (int) sizeof sql;
    }
    snprintf(sql + off, sizeof sql - off,
             " ORDER BY ts_transmitted DESC LIMIT %d", MAX_ROWS);

    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) return;
    if (have_like)
        sqlite3_bind_text(st, 1, like_pattern, -1, SQLITE_TRANSIENT);

    int n = 0;
    while (n < MAX_ROWS && sqlite3_step(st) == SQLITE_ROW) {
        cmd_row_t *r = &rows[n];
        memset(r, 0, sizeof *r);
        r->id         = sqlite3_column_int64(st, 0);
        r->ts_sent_ms = (uint64_t)sqlite3_column_int64(st, 1);
        r->has_tsexec = sqlite3_column_type(st, 2) != SQLITE_NULL;
        r->tsexec_ms  = r->has_tsexec ? sqlite3_column_int64(st, 2) : 0;
        const char *cmd = (const char *)sqlite3_column_text(st, 3);
        r->has_freq   = sqlite3_column_type(st, 4) != SQLITE_NULL;
        r->freq_hz    = r->has_freq ? sqlite3_column_int64(st, 4) : 0;
        r->has_gain   = sqlite3_column_type(st, 5) != SQLITE_NULL;
        r->gain_db    = r->has_gain ? sqlite3_column_double(st, 5) : 0.0;
        const char *tool = (const char *)sqlite3_column_text(st, 6);
        const char *run  = (const char *)sqlite3_column_text(st, 7);
        const char *tx   = (const char *)sqlite3_column_text(st, 8);
        snprintf(r->cmd,   sizeof r->cmd,   "%s", cmd  ? cmd  : "");
        snprintf(r->tool,  sizeof r->tool,  "%s", tool ? tool : "");
        snprintf(r->run,   sizeof r->run,   "%s", run  ? run  : "");
        snprintf(r->ts_tx, sizeof r->ts_tx, "%s", tx   ? tx   : "");
        int rc = resp_count_for(r->ts_sent_ms);
        // Apply the response filter here (the count isn't a DB column, so
        // it can't go in the WHERE clause); skipping just leaves the slot
        // for the next row.
        if ((resp_filter == 1 && rc == 0) || (resp_filter == 2 && rc > 0))
            continue;
        r->resp_count = rc;
        n++;
    }
    sqlite3_finalize(st);
    n_rows = n;

    sel = 0;
    if (prev_id >= 0)
        for (int i = 0; i < n_rows; i++)
            if (rows[i].id == prev_id) { sel = i; break; }
    if (sel >= n_rows) sel = n_rows > 0 ? n_rows - 1 : 0;
}

// Build the response sub-view for the selected command.
static void open_responses(sqlite3 *db)
{
    resp_n = 0; resp_sel = 0; resp_top = 0;
    cmd_row_t *c = &rows[sel];
    uint8_t key[8];
    for (int i = 0; i < 8; i++) key[i] = (uint8_t)(c->ts_sent_ms >> (8 * i));

    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT id, ts_received, payload FROM packet "
            "WHERE packet_type=4 AND substr(payload,2,8)=?1 "
            "ORDER BY substr(payload,13,1), ts_received", -1, &st, NULL)
        == SQLITE_OK) {
        sqlite3_bind_blob(st, 1, key, 8, SQLITE_TRANSIENT);
        while (resp_n < MAX_RESP && sqlite3_step(st) == SQLITE_ROW) {
            resp_t *e = &resps[resp_n];
            e->id = sqlite3_column_int64(st, 0);
            const char *ts = (const char *)sqlite3_column_text(st, 1);
            snprintf(e->ts, sizeof e->ts, "%s", ts ? ts : "");
            const uint8_t *pl = sqlite3_column_blob(st, 2);
            int pn = sqlite3_column_bytes(st, 2);
            if (pl == NULL
                || !tcmd_response_summary(pl, (size_t)pn, e->line, sizeof e->line))
                snprintf(e->line, sizeof e->line, "(undecodable response)");
            resp_n++;
        }
        sqlite3_finalize(st);
    }

    char human[40];
    fmt_epoch_ms(c->ts_sent_ms, human, sizeof human);
    snprintf(resp_header, sizeof resp_header,
             "responses to ts_sent=%llu (%s)  %s  | %d packet%s",
             (unsigned long long)c->ts_sent_ms, human, c->cmd,
             resp_n, resp_n == 1 ? "" : "s");
    in_resp = 1;
}

static void draw_top_bar(int cols)
{
    if (g_have_color) attron(COLOR_PAIR(PAIR_BAR)); else attron(A_REVERSE);
    move(0, 0);
    for (int i = 0; i < cols; i++) addch(' ');
    char buf[820];
    if (in_resp)
        snprintf(buf, sizeof buf, " tcmd_browser  %s", resp_header);
    else
        snprintf(buf, sizeof buf,
                 " tcmd_browser  sent telecommands  show=%s  search=\"%s\"  | %d command%s",
                 resp_filter_label(), like_text, n_rows, n_rows == 1 ? "" : "s");
    mvaddnstr(0, 0, buf, cols);
    if (g_have_color) attroff(COLOR_PAIR(PAIR_BAR)); else attroff(A_REVERSE);
}

static void draw_cmd_list(int list_top, int list_h, int cols)
{
    if (g_have_color) attron(A_DIM);
    char header[256];
    snprintf(header, sizeof header, "%6s  %-30s  %-20s  %4s  %-16s  %s",
             "#", show_local_time ? "TRANSMITTED (LOCAL)" : "TRANSMITTED (UTC)",
             "TS_SENT (UTC)", "RESP", "RUN", "COMMAND");
    mvaddnstr(list_top, 0, header, cols);
    if (g_have_color) attroff(A_DIM);

    int data_top = list_top + 1;
    int data_h = list_h - 1;
    if (data_h < 1) return;
    if (sel < top) top = sel;
    if (sel >= top + data_h) top = sel - data_h + 1;

    for (int i = 0; i < data_h; i++) {
        int ridx = top + i;
        move(data_top + i, 0); clrtoeol();
        if (ridx >= n_rows) continue;
        cmd_row_t *r = &rows[ridx];
        char ts_disp[40], human[40];
        format_ts(r->ts_tx, ts_disp, sizeof ts_disp);
        fmt_epoch_ms(r->ts_sent_ms, human, sizeof human);
        // First line of the command only, for the list. The on-screen row
        // is clipped to the column width anyway, so a bounded preview is
        // enough (and keeps the format provably non-truncating).
        char cmd1[256];
        snprintf(cmd1, sizeof cmd1, "%.*s", (int)sizeof cmd1 - 1, r->cmd);
        char *nl = strchr(cmd1, '\n'); if (nl) *nl = '\0';
        char line[400];
        snprintf(line, sizeof line, "%6d  %-30.30s  %-20.20s  %4d  %-16.16s  %.256s",
                 ridx + 1, ts_disp, human, r->resp_count,
                 r->run[0] ? r->run : "-", cmd1);

        int is_sel = (ridx == sel);
        int pair = is_sel ? PAIR_SEL
                          : (r->resp_count > 0 ? PAIR_OK : PAIR_NONE);
        if (is_sel) { if (g_have_color) attron(COLOR_PAIR(PAIR_SEL)); else attron(A_REVERSE); }
        else if (g_have_color && pair != PAIR_NONE) attron(COLOR_PAIR(pair));
        char marker[3] = { is_sel ? '>' : ' ', r->resp_count == 0 ? '?' : ' ', '\0' };
        mvaddstr(data_top + i, 0, marker);
        addnstr(line, cols - 2);
        if (is_sel) { if (g_have_color) attroff(COLOR_PAIR(PAIR_SEL)); else attroff(A_REVERSE); }
        else if (g_have_color && pair != PAIR_NONE) attroff(COLOR_PAIR(pair));
    }
}

static void draw_cmd_detail(int top_y, int height, int cols)
{
    move(top_y, 0); clrtoeol();
    if (g_have_color) attron(A_DIM);
    for (int i = 0; i < cols; i++) mvaddch(top_y, i, '-');
    if (g_have_color) attroff(A_DIM);
    int y = top_y + 1, max_y = top_y + height;
    if (n_rows == 0 || sel < 0 || sel >= n_rows) {
        if (n_rows == 0) mvaddstr(y, 2, "(no telecommands recorded)");
        return;
    }
    cmd_row_t *r = &rows[sel];
    char human[40];
    fmt_epoch_ms(r->ts_sent_ms, human, sizeof human);

    char head[256];
    snprintf(head, sizeof head,
             "ts_sent=%llu (%s)  responses=%d  tool=%s  run=%s",
             (unsigned long long)r->ts_sent_ms, human, r->resp_count,
             r->tool[0] ? r->tool : "-", r->run[0] ? r->run : "-");
    move(y, 0); clrtoeol();
    if (g_have_color) attron(A_BOLD);
    mvaddnstr(y, 2, head, cols - 2);
    if (g_have_color) attroff(A_BOLD);
    y++;

    char meta[256];
    int mp = snprintf(meta, sizeof meta, "transmitted=%s", r->ts_tx);
    if (r->has_tsexec)
        mp += snprintf(meta + mp, sizeof meta - mp, "  tsexec=%lld", r->tsexec_ms);
    if (r->has_freq)
        mp += snprintf(meta + mp, sizeof meta - mp, "  freq=%lldHz", r->freq_hz);
    if (r->has_gain)
        mp += snprintf(meta + mp, sizeof meta - mp, "  gain=%.1fdB", r->gain_db);
    move(y, 0); clrtoeol();
    mvaddnstr(y, 2, meta, cols - 2);
    y++;

    move(y, 0); clrtoeol();
    if (g_have_color) attron(A_DIM);
    mvaddnstr(y, 2, "command:", cols - 2);
    if (g_have_color) attroff(A_DIM);
    y++;

    // Wrap the full command text across as many rows as it needs.
    const char *p = r->cmd;
    int wrap_w = cols - 4; if (wrap_w < 1) wrap_w = 1;
    while (*p != '\0' && y < max_y) {
        int n = (int)strlen(p);
        if (n > wrap_w) n = wrap_w;
        move(y, 0); clrtoeol();
        mvaddnstr(y, 4, p, n);
        y++; p += n;
    }
    if (r->resp_count > 0 && y < max_y) {
        move(y, 0); clrtoeol();
        if (g_have_color) attron(COLOR_PAIR(PAIR_OK));
        mvaddnstr(y, 2, "press Enter to see the responses", cols - 2);
        if (g_have_color) attroff(COLOR_PAIR(PAIR_OK));
    }
    while (y < max_y) { move(y, 0); clrtoeol(); y++; }
}

static void draw_resp_list(int list_top, int list_h, int cols)
{
    if (g_have_color) attron(A_DIM);
    char header[256];
    snprintf(header, sizeof header, "%6s  %-30s  %s",
             "#", show_local_time ? "RECEIVED (LOCAL)" : "RECEIVED (UTC)",
             "RESPONSE");
    mvaddnstr(list_top, 0, header, cols);
    if (g_have_color) attroff(A_DIM);

    int data_top = list_top + 1, data_h = list_h - 1;
    if (data_h < 1) return;
    if (resp_sel < resp_top) resp_top = resp_sel;
    if (resp_sel >= resp_top + data_h) resp_top = resp_sel - data_h + 1;

    for (int i = 0; i < data_h; i++) {
        int ridx = resp_top + i;
        move(data_top + i, 0); clrtoeol();
        if (ridx >= resp_n) continue;
        resp_t *e = &resps[ridx];
        char ts_disp[40];
        format_ts(e->ts, ts_disp, sizeof ts_disp);
        char line[400];
        snprintf(line, sizeof line, "%6d  %-30.30s  %s", ridx + 1, ts_disp, e->line);
        int is_sel = (ridx == resp_sel);
        int err = (strstr(e->line, "(OK)") == NULL && strstr(e->line, "code=0") == NULL);
        if (is_sel) { if (g_have_color) attron(COLOR_PAIR(PAIR_SEL)); else attron(A_REVERSE); }
        else if (g_have_color) attron(COLOR_PAIR(err ? PAIR_ERR : PAIR_OK));
        char marker[2] = { is_sel ? '>' : ' ', '\0' };
        mvaddstr(data_top + i, 0, marker);
        addnstr(line, cols - 1);
        if (is_sel) { if (g_have_color) attroff(COLOR_PAIR(PAIR_SEL)); else attroff(A_REVERSE); }
        else if (g_have_color) attroff(COLOR_PAIR(err ? PAIR_ERR : PAIR_OK));
    }
}

static void draw_bottom_bar(int cols, int rows_total, int searching)
{
    if (g_have_color) attron(COLOR_PAIR(PAIR_BAR)); else attron(A_REVERSE);
    move(rows_total - 1, 0);
    for (int i = 0; i < cols; i++) addch(' ');
    const char *hint;
    if (searching) hint = " enter accept   esc cancel   backspace edits ";
    else if (in_resp) hint = " esc/left/bksp back   up/down scroll   l utc/lt   q quit ";
    else hint = " q quit   up/down scroll   enter responses   f answered   / search   l utc/lt   r reload ";
    mvaddnstr(rows_total - 1, 0, hint, cols);
    if (g_have_color) attroff(COLOR_PAIR(PAIR_BAR)); else attroff(A_REVERSE);
}

static int prompt_search(int rows_total, int cols)
{
    char buf[sizeof like_text];
    snprintf(buf, sizeof buf, "%s", like_text);
    size_t len = strlen(buf);
    nodelay(stdscr, FALSE);
    curs_set(1);
    while (1) {
        if (g_have_color) attron(COLOR_PAIR(PAIR_BAR)); else attron(A_REVERSE);
        move(rows_total - 1, 0);
        for (int i = 0; i < cols; i++) addch(' ');
        char line[256];
        snprintf(line, sizeof line, " /%s", buf);
        mvaddnstr(rows_total - 1, 0, line, cols);
        if (g_have_color) attroff(COLOR_PAIR(PAIR_BAR)); else attroff(A_REVERSE);
        move(rows_total - 1, 2 + (int)len);
        refresh();
        int ch = getch();
        if (ch == '\n' || ch == '\r' || ch == KEY_ENTER) {
            snprintf(like_text, sizeof like_text, "%s", buf);
            curs_set(0); nodelay(stdscr, TRUE); return 1;
        }
        if (ch == 27) { curs_set(0); nodelay(stdscr, TRUE); return 0; }
        if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) { if (len > 0) buf[--len] = '\0'; }
        else if (ch >= 0x20 && ch < 0x7F && len + 1 < sizeof buf) { buf[len++] = (char)ch; buf[len] = '\0'; }
    }
}

// Parsed command-line configuration. parse_args() fills this; main() reads it.
typedef struct {
    const char *db_path;
} tcb_args_t;

// Option column width: the widest label below ("--db=<path>") + a small
// margin. See src/cli/argparse.h for the parse_args convention.
#define OPTW 13

// Parse argv into *a (help == 0), or print one right-aligned help line per
// option and return (help != 0). Each option is one self-contained block whose
// test carries "|| help", so help mode falls through and prints them all.
static int parse_args(tcb_args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "--help-full") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help-full", "this help plus the key-binding reference");
            else { parse_args(a, argc, argv, HELP_FULL); return PARSE_HELP; }
            matched = 1;
        }
        if (starts_with(arg, "--db=") || help) {
            if (help) parse_help_line(OPTW, "--db=<path>", "packet DB path (default $SSO_PACKET_DB, else <root>/packet_db.sqlite)");
            else a->db_path = arg + 5;
            matched = 1;
        }

        if (!matched && !help) {
            // Original behaviour: any unrecognized argument prints usage to
            // stderr and fails. The only option is --db=; anything else lands
            // here.
            fprintf(stderr, "usage: %s [--db=<path>]\n", argv[0]);
            return PARSE_ERROR;
        }
    }
    // --help-full appends the key-binding reference (was in the old usage()).
    if (help >= HELP_FULL) {
        printf("\nKeys:\n"
               "  q | Q | Esc      quit (in the responses view, step back)\n"
               "  arrows / PgUp / PgDn / Home / End   scroll\n"
               "  Enter            open the responses for the selected command\n"
               "                   (the tcmd_response packets sharing its ts_sent)\n"
               "  f                cycle response filter: all -> answered (got a\n"
               "                   response) -> unanswered\n"
               "  /                substring search against the command text\n"
               "  l                toggle timestamp display: UTC (storage) <-> local\n"
               "  r                reload now\n");
    }
    return PARSE_OK;
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "tcmd_browser")) return 0;
    tcb_args_t cfg = {0};
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }
    const char *db_path = cfg.db_path;

    char default_db[1024];
    if (db_path == NULL) {
        if (packet_db_default_path(default_db, sizeof default_db) != 0) {
            fprintf(stderr, "tcmd_browser: cannot resolve default DB path "
                    "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = default_db;
    }

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READWRITE, NULL) != SQLITE_OK) {
        fprintf(stderr, "tcmd_browser: open(%s) failed: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "?");
        if (db) sqlite3_close(db);
        return 1;
    }
    sqlite3_busy_timeout(db, 5000);

    if (initscr() == NULL) {
        sqlite3_close(db);
        fprintf(stderr, "tcmd_browser: ncurses initscr failed\n");
        return 1;
    }
    cbreak(); noecho(); nonl();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    set_escdelay(25);   // a lone Esc shouldn't wait a full second (see packet_browser)
    curs_set(0);
    if (has_colors()) {
        start_color();
        use_default_colors();
        short sel_bg = (COLORS >= 256) ? 240 : COLOR_WHITE;
        init_pair(PAIR_BAR,  COLOR_WHITE,  COLOR_BLUE);
        init_pair(PAIR_SEL,  COLOR_WHITE,  sel_bg);
        init_pair(PAIR_DIM,  COLOR_WHITE,  -1);
        init_pair(PAIR_OK,   COLOR_GREEN,  -1);
        init_pair(PAIR_ERR,  COLOR_RED,    -1);
        init_pair(PAIR_NONE, -1,           -1);
        g_have_color = 1;
    }

    run_query(db);
    double last_query = monotonic_seconds();
    int quit = 0;
    while (!quit) {
        int rows_total = LINES, cols = COLS;
        int header_h = 1, footer_h = 1;
        int avail = rows_total - header_h - footer_h;
        if (avail < 6) avail = 6;
        int list_h = in_resp ? avail : avail / 2;
        if (list_h < 4) list_h = 4;
        int detail_top = header_h + list_h;
        int detail_h = rows_total - footer_h - detail_top;

        erase();
        draw_top_bar(cols);
        if (in_resp) {
            draw_resp_list(header_h, rows_total - header_h - footer_h, cols);
        } else {
            draw_cmd_list(header_h, list_h, cols);
            draw_cmd_detail(detail_top, detail_h, cols);
        }
        draw_bottom_bar(cols, rows_total, 0);
        refresh();

        timeout(250);
        int ch = getch();
        if (ch != ERR) {
            int *psel = in_resp ? &resp_sel : &sel;
            int *ptop = in_resp ? &resp_top : &top;
            int  pn   = in_resp ? resp_n : n_rows;
            switch (ch) {
            case 'q': case 'Q':
                if (in_resp) in_resp = 0; else quit = 1;
                break;
            case 27: case KEY_LEFT: case KEY_BACKSPACE: case 127: case 8: case 'h':
                if (in_resp) in_resp = 0;
                else if (ch == 27) quit = 1;
                break;
            case '\n': case '\r': case KEY_ENTER: case KEY_RIGHT:
                if (!in_resp && n_rows > 0) open_responses(db);
                break;
            case KEY_UP:   case 'k': if (*psel > 0) (*psel)--; break;
            case KEY_DOWN: case 'j': if (*psel < pn - 1) (*psel)++; break;
            case KEY_PPAGE: case 2:
                *psel -= list_h - 1; if (*psel < 0) *psel = 0; break;
            case KEY_NPAGE: case 6:
                *psel += list_h - 1;
                if (*psel >= pn) *psel = pn - 1;
                if (*psel < 0) *psel = 0;
                break;
            case KEY_HOME: case 'g': *psel = 0; break;
            case KEY_END:  case 'G': *psel = pn > 0 ? pn - 1 : 0; break;
            case 'l': show_local_time = !show_local_time; break;
            case 'f':  // cycle response filter: all -> answered -> unanswered
                if (in_resp) break;
                resp_filter = (resp_filter + 1) % 3;
                run_query(db); last_query = monotonic_seconds();
                break;
            case 'r': case 'R': case 18:
                if (in_resp) break;
                run_query(db); last_query = monotonic_seconds();
                break;
            case '/':
                if (in_resp) break;
                if (prompt_search(rows_total, cols)) {
                    run_query(db); last_query = monotonic_seconds();
                }
                break;
            case KEY_RESIZE: break;
            }
            (void)ptop;
        }

        // 1 Hz auto-poll of the command list (suspended in the responses
        // view so it doesn't reshuffle under the operator).
        double now = monotonic_seconds();
        if (!in_resp && now - last_query >= 1.0) {
            run_query(db); last_query = now;
        }
    }

    endwin();
    sqlite3_close(db);
    return 0;
}

#endif
