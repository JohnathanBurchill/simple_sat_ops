/*

    Simple Satellite Operations  utils/packet_browser.c

    Curses TUI over the packet DB written by the live and offline AX100
    receivers. Reads only — never writes — so it's safe to run alongside
    a receiver that's filling the same DB. Polls the DB at ~1 Hz so live
    decodes appear in the list without the operator hitting reload.

    Layout:

      ┌── filter status (top reverse-video bar) ───────────────────────┐
      ├── scrolling packet list (one line per packet) ─────────────────┤
      │                                                                │
      ├── separator                                                    │
      ├── detail panel (firmware-interpreted body for the selection) ──┤
      │                                                                │
      └── key hints (bottom reverse-video bar) ────────────────────────┘

    Keys:
      q | Q | Esc      quit (in the command group, step back to the list)
      ↑ / ↓            move selection by one row
      PgUp / PgDn      move by a page (list height)
      Home / End       jump to top / bottom
      Enter            on a tcmd_response, open the command group (all
                       packets sharing that command's ts_sent, plus the
                       same-run log/bulk_file packets that follow); Esc /
                       Left / Backspace step back
      r                reload now (auto-poll happens every ~1 s anyway)
      t                cycle type filter: all → beacon → tcmd_response →
                       log → bulk_file → all
      v                cycle the detail-pane payload view: hex → ascii →
                       base64 (a bulk_file's ascii/base64 show the file
                       data after the 5-byte type+offset header)
      /                start a LIKE search (substring match against the
                       firmware-interpreted text); Enter applies, Esc
                       cancels, Backspace edits

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

#include "packet_db.h"
#include "sso_paths.h"

#include <dirent.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#if !defined(WITH_SQLITE3)
int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    fprintf(stderr,
            "packet_browser: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#elif !defined(PACKET_BROWSER_HAVE_NCURSES)
int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    fprintf(stderr,
            "packet_browser: built without ncurses. Install\n"
            "libncurses-dev and rebuild.\n");
    return 1;
}
#else

#include <ncurses.h>
#include <sqlite3.h>

#define MAX_ROWS 1000

typedef struct {
    sqlite3_int64 id;
    char ts[40];
    char tool[20];
    char type_name[20];
    char satellite[12];
    char origin[16];
    char run[24];
    int  packet_type;
    int  csp_src, csp_dst, csp_dport, csp_sport, csp_prio, csp_flags;
    int  golay_errs, rs_errs, hmac_ok, crc_status;
    int  has_offset;
    double audio_offset_s;
    // Observer-frame geometry at the moment of reception. Any of these
    // can be NULL in the DB (e.g. when the decoder didn't have a TLE
    // bound for the run), so has_geom records whether at least one
    // value was non-NULL — that gates the detail panel line.
    int    has_geom;
    int    geom_az_valid, geom_el_valid;
    int    geom_range_valid, geom_range_rate_valid, geom_doppler_valid;
    double geom_az_deg, geom_el_deg;
    double geom_range_km, geom_range_rate_km_s, geom_doppler_hz;
    char   session_dir[256];
    char summary[2048];
    int      payload_len;   // full payload length
    uint8_t *payload;       // full payload, malloc'd per row (freed on reload/exit)
} row_t;

// Type cycle: NULL means "all", otherwise filter on packet_type_name.
static const char *const TYPE_CYCLE[] = {
    NULL, "beacon", "tcmd_response", "log", "bulk_file"
};
static const int TYPE_CYCLE_N = sizeof TYPE_CYCLE / sizeof TYPE_CYCLE[0];

// Origin cycle: NULL = all, otherwise filter on capture_origin.
// Mirrors the V4 schema's capture_origin column values; new origins
// (e.g. another partner ground station) get added here.
static const char *const ORIGIN_CYCLE[] = {
    NULL, "cts_ground", "satnogs"
};
static const int ORIGIN_CYCLE_N = sizeof ORIGIN_CYCLE / sizeof ORIGIN_CYCLE[0];

// Two backing stores: the main packet list and the command-group
// sub-view (Enter on a tcmd_response). `rows` points at whichever is
// active, so every render/scroll path that uses rows[]/n_rows/sel/top
// works unchanged for both views. The inactive store keeps its rows and
// payloads alive so returning from the sub-view is instant.
static row_t   main_rows[MAX_ROWS];
static row_t   group_rows[MAX_ROWS];
static row_t  *rows = main_rows;
static int     n_rows = 0;
static int     sel    = 0;
static int     top    = 0;

// Command-group sub-view state. `in_group` is 1 while it's up; the main
// view's scroll position is parked in main_* and restored on return.
// group_confirmed_n is how many leading group rows are the ts_sent-exact
// tcmd_response matches (the rest are the same-run/time heuristic set).
static int     in_group = 0;
static int     group_n = 0;
static int     group_confirmed_n = 0;
static int     main_n = 0, main_sel = 0, main_top = 0;
// Big enough to hold the ~512-char resolved command text plus the
// ts_sent / count framing without format-truncation.
static char    group_header[768] = "";
// The 8 raw ts_sent bytes of the command the sub-view is built around,
// kept so a reload (`r`) can rebuild the same group.
static uint8_t group_key[8];
static char    group_run[24] = "";
static char    group_anchor_ts[40] = "";
static int     type_idx = 0;
static int     origin_idx = 0;
static char    like_text[128] = "";
// Display mode: 0 = UTC (storage form, ISO-8601 Z), 1 = local time
// (parsed back to time_t and re-formatted with tzname). Filtering and
// sorting still happens server-side against the UTC strings; only the
// rendered cells change. Toggle with `L`.
static int     show_local_time = 0;

// Payload view mode in the detail pane, cycled by `v`. Hex is the raw
// view (whole payload, every packet type). Ascii and base64 are content
// views: for a bulk_file they interpret the file data (the bytes after
// the 5-byte type+offset header) so a downloaded text file reads as
// text or copies out as base64; for any other type they cover the whole
// payload. Default hex preserves the long-standing dump.
enum { PV_HEX = 0, PV_ASCII, PV_BASE64 };
static int         payload_view = PV_HEX;
static const char *PV_NAME[] = { "hex", "ascii", "base64" };

static int     g_have_color = 0;
// Toggled by `s`: when on, draw_detail adds a "station: ..." line for
// satnogs rows, pulling station_lat/lng/alt out of the obs's
// meta.json on demand. One-entry cache keyed by session_dir keeps the
// fopen/parse off the hot path when the operator dwells on a row.
static int     g_show_station = 0;
static char    g_station_cache_dir[256] = "";
static int     g_station_cache_ok = 0;
static char    g_station_cache_name[64] = "";
static int     g_station_cache_id = 0;
static double  g_station_cache_lat = 0.0;
static double  g_station_cache_lng = 0.0;
static double  g_station_cache_alt = 0.0;
enum {
    PAIR_BAR    = 1,
    PAIR_BEACON,
    PAIR_TCMD,
    PAIR_LOG,
    PAIR_BULK,
    PAIR_ERROR,
    PAIR_SEL,
    PAIR_SEL_ERR,
    PAIR_SEL_BEACON,
    PAIR_SEL_TCMD,
    PAIR_SEL_LOG,
    PAIR_SEL_BULK,
    PAIR_DIM,
};

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

// Minimal JSON helpers — good enough for the flat top-level fields
// of a SatNOGS observation detail JSON (station_lat, station_lng,
// station_alt, station_name, ground_station). Naive strstr matching
// works here because the keys aren't substrings of each other and the
// JSON has no nested object that re-uses the same key names.
static int json_extract_number(const char *json, const char *key,
                               double *out)
{
    char needle[64];
    int n = snprintf(needle, sizeof needle, "\"%s\"", key);
    if (n < 0 || (size_t)n >= sizeof needle) return -1;
    const char *p = strstr(json, needle);
    if (p == NULL) return -1;
    p += n;
    while (*p == ' ' || *p == ':' || *p == '\t'
        || *p == '\n' || *p == '\r') p++;
    if (*p == '\0') return -1;
    char *endp = NULL;
    double v = strtod(p, &endp);
    if (endp == p) return -1;
    *out = v;
    return 0;
}

static int json_extract_string(const char *json, const char *key,
                               char *out, size_t cap)
{
    char needle[64];
    int n = snprintf(needle, sizeof needle, "\"%s\"", key);
    if (n < 0 || (size_t)n >= sizeof needle) return -1;
    const char *p = strstr(json, needle);
    if (p == NULL) return -1;
    p += n;
    while (*p == ' ' || *p == ':' || *p == '\t'
        || *p == '\n' || *p == '\r') p++;
    if (*p != '"') return -1;
    p++;
    size_t i = 0;
    while (*p != '\0' && *p != '"' && i + 1 < cap) {
        if (*p == '\\' && *(p + 1) != '\0') p++;
        out[i++] = *p++;
    }
    out[i] = '\0';
    return 0;
}

// Refresh g_station_cache_* for `session_dir` if it's not already
// the cached entry. Returns 1 if the cache is populated and valid,
// 0 otherwise (no meta.json, unparseable, or missing fields).
static int station_cache_for(const char *session_dir)
{
    if (session_dir == NULL || session_dir[0] == '\0') return 0;
    if (strcmp(g_station_cache_dir, session_dir) == 0) {
        return g_station_cache_ok;
    }
    snprintf(g_station_cache_dir, sizeof g_station_cache_dir,
             "%s", session_dir);
    g_station_cache_ok = 0;

    // Find the obs id from the session dir tail and assemble the
    // meta.json filename satnogs_pull writes.
    const char *base = strrchr(session_dir, '/');
    base = base ? base + 1 : session_dir;
    if (base[0] == '\0') return 0;

    char meta[512];
    int rc = snprintf(meta, sizeof meta, "%s/satnogs_%s.meta.json",
                      session_dir, base);
    if (rc < 0 || (size_t)rc >= sizeof meta) return 0;

    FILE *f = fopen(meta, "rb");
    if (f == NULL) return 0;
    char buf[16384];
    size_t got = fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[got] = '\0';

    double lat = 0.0, lng = 0.0, alt = 0.0, sid = 0.0;
    if (json_extract_number(buf, "station_lat", &lat) != 0) return 0;
    if (json_extract_number(buf, "station_lng", &lng) != 0) return 0;
    if (json_extract_number(buf, "station_alt", &alt) != 0) return 0;
    if (json_extract_number(buf, "ground_station", &sid) != 0) sid = 0.0;
    if (json_extract_string(buf, "station_name",
                            g_station_cache_name,
                            sizeof g_station_cache_name) != 0) {
        snprintf(g_station_cache_name,
                 sizeof g_station_cache_name, "?");
    }
    g_station_cache_id  = (int)sid;
    g_station_cache_lat = lat;
    g_station_cache_lng = lng;
    g_station_cache_alt = alt;
    g_station_cache_ok  = 1;
    return 1;
}

static const char *type_filter(void)
{
    return TYPE_CYCLE[type_idx];
}

static const char *origin_filter(void)
{
    return ORIGIN_CYCLE[origin_idx];
}

// The column list every row query selects, in the order fill_row()
// reads them. Shared by the main filter query and the command-group
// query so both feed the same row_t loader.
#define PACKET_SELECT_COLS \
    "SELECT id, ts_received, satellite, packet_type, packet_type_name, " \
    "csp_src, csp_dst, csp_dport, csp_sport, csp_prio, csp_flags, " \
    "payload, golay_errs, rs_errs, hmac_ok, crc_status, " \
    "source_tool, source_run, audio_offset_s, decoded_summary, " \
    "capture_origin, az_deg, el_deg, range_km, range_rate_km_s, " \
    "doppler_hz_offset, session_dir "

// Free the malloc'd payloads of `n` rows in `arr` and zero the pointers,
// so the array can be reloaded or torn down without leaking.
static void free_rows(row_t *arr, int n)
{
    for (int i = 0; i < n; i++) {
        free(arr[i].payload);
        arr[i].payload = NULL;
        arr[i].payload_len = 0;
    }
}

// Fill *r from the current row of `stmt`, which must have selected
// PACKET_SELECT_COLS. Allocates r->payload (NULL on alloc failure).
static void fill_row(sqlite3_stmt *stmt, row_t *r)
{
    r->id          = sqlite3_column_int64(stmt, 0);
    const char *ts = (const char *)sqlite3_column_text(stmt, 1);
    const char *sat= (const char *)sqlite3_column_text(stmt, 2);
    r->packet_type = sqlite3_column_int(stmt, 3);
    const char *pn = (const char *)sqlite3_column_text(stmt, 4);
    r->csp_src     = sqlite3_column_int(stmt, 5);
    r->csp_dst     = sqlite3_column_int(stmt, 6);
    r->csp_dport   = sqlite3_column_int(stmt, 7);
    r->csp_sport   = sqlite3_column_int(stmt, 8);
    r->csp_prio    = sqlite3_column_int(stmt, 9);
    r->csp_flags   = sqlite3_column_int(stmt, 10);
    const uint8_t *pl = sqlite3_column_blob(stmt, 11);
    int pln           = sqlite3_column_bytes(stmt, 11);
    r->golay_errs  = sqlite3_column_int(stmt, 12);
    r->rs_errs     = sqlite3_column_int(stmt, 13);
    r->hmac_ok     = sqlite3_column_int(stmt, 14);
    r->crc_status  = sqlite3_column_int(stmt, 15);
    const char *tl = (const char *)sqlite3_column_text(stmt, 16);
    const char *rn = (const char *)sqlite3_column_text(stmt, 17);
    r->has_offset  = sqlite3_column_type(stmt, 18) != SQLITE_NULL;
    r->audio_offset_s = r->has_offset ? sqlite3_column_double(stmt, 18) : 0.0;
    const char *sm = (const char *)sqlite3_column_text(stmt, 19);
    const char *og = (const char *)sqlite3_column_text(stmt, 20);

    r->geom_az_valid         = sqlite3_column_type(stmt, 21) != SQLITE_NULL;
    r->geom_el_valid         = sqlite3_column_type(stmt, 22) != SQLITE_NULL;
    r->geom_range_valid      = sqlite3_column_type(stmt, 23) != SQLITE_NULL;
    r->geom_range_rate_valid = sqlite3_column_type(stmt, 24) != SQLITE_NULL;
    r->geom_doppler_valid    = sqlite3_column_type(stmt, 25) != SQLITE_NULL;
    r->geom_az_deg           = r->geom_az_valid         ? sqlite3_column_double(stmt, 21) : 0.0;
    r->geom_el_deg           = r->geom_el_valid         ? sqlite3_column_double(stmt, 22) : 0.0;
    r->geom_range_km         = r->geom_range_valid      ? sqlite3_column_double(stmt, 23) : 0.0;
    r->geom_range_rate_km_s  = r->geom_range_rate_valid ? sqlite3_column_double(stmt, 24) : 0.0;
    r->geom_doppler_hz       = r->geom_doppler_valid    ? sqlite3_column_double(stmt, 25) : 0.0;
    r->has_geom = r->geom_az_valid || r->geom_el_valid
               || r->geom_range_valid || r->geom_range_rate_valid
               || r->geom_doppler_valid;
    const char *sd = (const char *)sqlite3_column_text(stmt, 26);
    snprintf(r->session_dir, sizeof r->session_dir, "%s", sd ? sd : "");

    snprintf(r->ts,        sizeof r->ts,        "%s", ts ? ts : "");
    snprintf(r->satellite, sizeof r->satellite, "%s", sat ? sat : "");
    snprintf(r->type_name, sizeof r->type_name, "%s", pn ? pn : "");
    snprintf(r->tool,      sizeof r->tool,      "%s", tl ? tl : "");
    snprintf(r->run,       sizeof r->run,       "%s", rn ? rn : "");
    snprintf(r->summary,   sizeof r->summary,   "%s", sm ? sm : "");
    snprintf(r->origin,    sizeof r->origin,    "%s", og ? og : "");

    // Store the FULL payload so the detail view can show all of it.
    r->payload     = NULL;
    r->payload_len = 0;
    if (pln > 0 && pl != NULL) {
        r->payload = (uint8_t *) malloc((size_t)pln);
        if (r->payload != NULL) {
            memcpy(r->payload, pl, (size_t)pln);
            r->payload_len = pln;
        }
    }
}

// Run the current filter against the DB and refresh `rows` / `n_rows`.
// Selection (`sel`) is preserved by row id where possible — feeling
// like the row "stays in place" across reloads is more important than
// always landing on the freshest packet. If the previously-selected id
// is gone, fall back to position 0. Only ever called for the main view
// (the command-group sub-view has its own loader), so `rows` is
// main_rows here.
static void run_query(sqlite3 *db)
{
    sqlite3_int64 prev_id = (n_rows > 0) ? rows[sel].id : -1;

    char sql[1024];
    int off = snprintf(sql, sizeof sql,
        PACKET_SELECT_COLS
        "FROM packet WHERE 1=1");
    int n_params = 0;
    const char *param_text[4] = {0};
    char like_pattern[256];
    if (type_filter() != NULL) {
        off += snprintf(sql + off, sizeof sql - off,
                        " AND packet_type_name = ?%d", n_params + 1);
        param_text[n_params++] = type_filter();
    }
    if (origin_filter() != NULL) {
        off += snprintf(sql + off, sizeof sql - off,
                        " AND capture_origin = ?%d", n_params + 1);
        param_text[n_params++] = origin_filter();
    }
    if (like_text[0] != '\0') {
        snprintf(like_pattern, sizeof like_pattern, "%%%s%%", like_text);
        off += snprintf(sql + off, sizeof sql - off,
                        " AND decoded_summary LIKE ?%d", n_params + 1);
        param_text[n_params++] = like_pattern;
    }
    snprintf(sql + off, sizeof sql - off,
             " ORDER BY ts_received DESC LIMIT %d", MAX_ROWS);

    sqlite3_stmt *stmt = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK) {
        // Leave the existing rows in place rather than clearing — a
        // transient prepare failure shouldn't blank the screen.
        return;
    }
    for (int i = 0; i < n_params; i++) {
        sqlite3_bind_text(stmt, i + 1, param_text[i], -1, SQLITE_TRANSIENT);
    }

    // Free payloads from the previous load (the rows array is reused
    // across reloads) so re-querying doesn't leak.
    free_rows(rows, n_rows);

    int new_n = 0;
    while (new_n < MAX_ROWS && sqlite3_step(stmt) == SQLITE_ROW) {
        fill_row(stmt, &rows[new_n]);
        new_n++;
    }
    sqlite3_finalize(stmt);
    n_rows = new_n;
    main_n = new_n;  // keep the main count current for the sub-view save/restore

    // Re-seat the selection on the same id when possible.
    sel = 0;
    if (prev_id >= 0) {
        for (int i = 0; i < n_rows; i++) {
            if (rows[i].id == prev_id) { sel = i; break; }
        }
    }
    if (sel < 0) sel = 0;
    if (sel >= n_rows) sel = n_rows > 0 ? n_rows - 1 : 0;
}

// ---- Command-group sub-view (Enter on a tcmd_response) -----------------

// How long after a command's first response a same-run log / bulk_file
// packet is still considered "possibly part of this command". A generous
// window: the satellite streams a download right after the command, but
// the firmware doesn't tag those packets with the command's ts_sent, so
// this is a heuristic, not a guarantee.
#define GROUP_WINDOW_S 600

// A tcmd_response payload is [type:1][ts_sent:8 LE][...]. Decode the
// little-endian ts_sent (unix-ms). Returns 1 and writes *out_ms on
// success, 0 if the row isn't a tcmd_response with enough bytes.
static int row_ts_sent(const row_t *r, uint64_t *out_ms)
{
    if (r == NULL || r->payload == NULL || r->payload_len < 9) return 0;
    if (r->packet_type != 0x04) return 0;
    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
        v |= (uint64_t)r->payload[1 + i] << (8 * i);
    *out_ms = v;
    return 1;
}

// Format a unix-ms instant as "YYYY-MM-DDTHH:MM:SSZ" (UTC, second
// resolution). Used for the group header and the heuristic time window.
static void fmt_epoch_ms(uint64_t ms, char *out, size_t outn)
{
    time_t t = (time_t)(ms / 1000);
    struct tm tm;
    if (gmtime_r(&t, &tm) == NULL) { snprintf(out, outn, "?"); return; }
    strftime(out, outn, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

// Search the agenda files under the data root for "@tssent=<ms>" and
// copy the first matching line into `out`. Best-effort fallback used
// when the sent_tcmd table has no row (e.g. a command sent before the
// table existed). Bounded: only files whose name contains "agenda",
// a capped number of files, capped read size. Returns 1 on a hit.
static int scan_agenda_dir(const char *dir, const char *needle,
                           char *out, size_t outn, int depth, int *budget)
{
    if (depth > 4 || *budget <= 0) return 0;
    DIR *d = opendir(dir);
    if (d == NULL) return 0;
    struct dirent *de;
    int found = 0;
    while (!found && (de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char path[1024];
        if ((size_t)snprintf(path, sizeof path, "%s/%s", dir, de->d_name)
            >= sizeof path) continue;
        struct stat st;
        if (stat(path, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            found = scan_agenda_dir(path, needle, out, outn, depth + 1, budget);
            continue;
        }
        if (!S_ISREG(st.st_mode)) continue;
        if (strstr(de->d_name, "agenda") == NULL) continue;
        if (--(*budget) < 0) break;
        FILE *f = fopen(path, "rb");
        if (f == NULL) continue;
        char line[2048];
        while (fgets(line, sizeof line, f) != NULL) {
            if (strstr(line, needle) == NULL) continue;
            line[strcspn(line, "\r\n")] = '\0';
            const char *s = line;
            while (*s == ' ' || *s == '\t') s++;
            snprintf(out, outn, "%s", s);
            found = 1;
            break;
        }
        fclose(f);
    }
    closedir(d);
    return found;
}

// Resolve the command (and arguments) that produced ts_sent_ms into
// `out`. Prefers the sent_tcmd table (exact, recorded at transmit), then
// falls back to scanning agenda files on disk, then "(command unknown)".
static void resolve_command_text(sqlite3 *db, uint64_t ts_sent_ms,
                                 char *out, size_t outn)
{
    out[0] = '\0';
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT command_text FROM sent_tcmd WHERE ts_sent_ms=?1 LIMIT 1",
            -1, &st, NULL) == SQLITE_OK) {
        sqlite3_bind_int64(st, 1, (sqlite3_int64)ts_sent_ms);
        if (sqlite3_step(st) == SQLITE_ROW) {
            const char *txt = (const char *)sqlite3_column_text(st, 0);
            if (txt != NULL && txt[0] != '\0') snprintf(out, outn, "%s", txt);
        }
        sqlite3_finalize(st);
    }
    if (out[0] != '\0') return;

    char needle[40];
    snprintf(needle, sizeof needle, "@tssent=%" PRIu64, ts_sent_ms);
    int budget = 200;
    if (scan_agenda_dir(sso_frontiersat_root(), needle, out, outn, 0, &budget))
        return;
    snprintf(out, outn, "(command unknown)");
}

// Append rows returned by `sql` (bound via the bind callback's params)
// into group_rows starting at group_n. Helper to keep run_group_query
// readable. Returns the number of rows appended.
static int group_append(sqlite3 *db, const char *sql,
                        void (*bind)(sqlite3_stmt *, void *), void *ctx)
{
    sqlite3_stmt *stmt = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK) return 0;
    if (bind) bind(stmt, ctx);
    int added = 0;
    while (group_n < MAX_ROWS && sqlite3_step(stmt) == SQLITE_ROW) {
        fill_row(stmt, &group_rows[group_n]);
        group_n++;
        added++;
    }
    sqlite3_finalize(stmt);
    return added;
}

static void bind_group_confirmed(sqlite3_stmt *stmt, void *ctx)
{
    (void)ctx;
    // The 8 raw ts_sent bytes match the BLOB slice substr(payload,2,8).
    sqlite3_bind_blob(stmt, 1, group_key, 8, SQLITE_TRANSIENT);
}

static void bind_group_related(sqlite3_stmt *stmt, void *ctx)
{
    char *hi = ctx;
    sqlite3_bind_text(stmt, 1, group_run,        -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, group_anchor_ts,  -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, hi,               -1, SQLITE_TRANSIENT);
}

// Build the command-group sub-view into group_rows from the saved
// group_key (8 ts_sent bytes) and group_run: first the ts_sent-exact
// tcmd_response packets (ordered by response seq then time), then the
// same-run log / bulk_file packets in the time window after the command
// (the heuristic set). Main-view filters are ignored. Rebuildable on
// reload because it reads only the saved key/run, not a live row.
static void build_group(sqlite3 *db)
{
    free_rows(group_rows, group_n);
    group_n = 0;
    group_confirmed_n = 0;

    uint64_t ts_ms = 0;
    for (int i = 0; i < 8; i++) ts_ms |= (uint64_t)group_key[i] << (8 * i);

    // Confirmed: every tcmd_response sharing this exact ts_sent, in
    // response-sequence order (substr(payload,13,1) is response_seq_num),
    // then by arrival time.
    group_append(db,
        PACKET_SELECT_COLS
        "FROM packet WHERE packet_type=4 AND substr(payload,2,8)=?1 "
        "ORDER BY substr(payload,13,1), ts_received",
        bind_group_confirmed, NULL);
    group_confirmed_n = group_n;

    // Anchor the heuristic window at the earliest confirmed packet.
    // Entering from a real response guarantees at least one confirmed row.
    group_anchor_ts[0] = '\0';
    if (group_confirmed_n > 0)
        snprintf(group_anchor_ts, sizeof group_anchor_ts, "%s",
                 group_rows[0].ts);
    char hi[40];
    {
        int yr, mo, dd, hh, mm, ss;
        if (sscanf(group_anchor_ts, "%4d-%2d-%2dT%2d:%2d:%2d",
                   &yr, &mo, &dd, &hh, &mm, &ss) == 6) {
            struct tm tm = {0};
            tm.tm_year = yr - 1900; tm.tm_mon = mo - 1; tm.tm_mday = dd;
            tm.tm_hour = hh; tm.tm_min = mm; tm.tm_sec = ss;
            time_t t = timegm(&tm) + GROUP_WINDOW_S;
            struct tm up;
            gmtime_r(&t, &up);
            strftime(hi, sizeof hi, "%Y-%m-%dT%H:%M:%SZ", &up);
        } else {
            snprintf(hi, sizeof hi, "9999");  // no parse -> unbounded upper
        }
    }

    // Possibly related: log (3) + bulk_file (16) from the same run within
    // the window. The firmware doesn't tag these with the command, so
    // this is a time/run heuristic, flagged as such in the header.
    if (group_run[0] != '\0' && group_anchor_ts[0] != '\0') {
        group_append(db,
            PACKET_SELECT_COLS
            "FROM packet WHERE packet_type IN (3,16) AND source_run=?1 "
            "AND ts_received >= ?2 AND ts_received <= ?3 "
            "ORDER BY ts_received",
            bind_group_related, hi);
    }

    // Header: ts_sent (raw + humanized) + resolved command + section
    // counts. Shown in the top bar while the sub-view is up.
    char human[40];
    fmt_epoch_ms(ts_ms, human, sizeof human);
    char cmd_text[512];
    resolve_command_text(db, ts_ms, cmd_text, sizeof cmd_text);
    int related = group_n - group_confirmed_n;
    snprintf(group_header, sizeof group_header,
             "cmd ts_sent=%" PRIu64 " (%s)  %s  | %d response%s, "
             "%d related (same run, <=%dm, unconfirmed)",
             ts_ms, human, cmd_text,
             group_confirmed_n, group_confirmed_n == 1 ? "" : "s",
             related, GROUP_WINDOW_S / 60);
}

// Enter the command-group sub-view for the currently-selected row (must
// be a tcmd_response). Parks the main view's scroll position.
static void enter_group(sqlite3 *db)
{
    uint64_t ts_ms = 0;
    if (!row_ts_sent(&rows[sel], &ts_ms)) return;  // not a decodable response
    // Save the join key + run from the selected response, then build the
    // group from those alone (so a reload doesn't depend on a live row).
    for (int i = 0; i < 8; i++) group_key[i] = rows[sel].payload[1 + i];
    snprintf(group_run, sizeof group_run, "%s", rows[sel].run);
    main_sel = sel; main_top = top; main_n = n_rows;
    build_group(db);
    rows = group_rows; n_rows = group_n; sel = 0; top = 0; in_group = 1;
}

// Return from the sub-view to the main list, restoring its scroll.
static void leave_group(void)
{
    free_rows(group_rows, group_n);
    group_n = 0;
    rows = main_rows; n_rows = main_n; sel = main_sel; top = main_top;
    in_group = 0;
}

// Render a stored ISO-8601 UTC timestamp into the user's chosen
// display mode. UTC mode is a passthrough; local mode parses the
// "YYYY-MM-DDTHH:MM:SS[.fff]Z" form back to a time_t (via timegm)
// then re-formats with localtime_r. Garbage that doesn't match the
// pattern falls through unchanged so the operator can still see what
// the column actually contains. The width is intentionally fixed
// across modes (the list column is sized for it).
static void format_ts(const char *iso, char *out, size_t outn)
{
    if (iso == NULL || iso[0] == '\0') {
        if (outn > 0) out[0] = '\0';
        return;
    }
    if (!show_local_time) {
        snprintf(out, outn, "%s", iso);
        return;
    }
    int yr, mo, dd, hh, mm, ss, ms = 0;
    int got = sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2d.%3d",
                     &yr, &mo, &dd, &hh, &mm, &ss, &ms);
    if (got < 6) {
        snprintf(out, outn, "%s", iso);
        return;
    }
    struct tm utc = {0};
    utc.tm_year = yr - 1900;
    utc.tm_mon  = mo - 1;
    utc.tm_mday = dd;
    utc.tm_hour = hh;
    utc.tm_min  = mm;
    utc.tm_sec  = ss;
    time_t epoch = timegm(&utc);
    if (epoch == (time_t)-1) {
        snprintf(out, outn, "%s", iso);
        return;
    }
    struct tm local;
    localtime_r(&epoch, &local);
    char base[40];
    strftime(base, sizeof base, "%Y-%m-%d %H:%M:%S", &local);
    const char *tz = tzname[local.tm_isdst > 0 ? 1 : 0];
    if (tz == NULL) tz = "";
    snprintf(out, outn, "%s.%03d %s", base, ms, tz);
}

static int color_for_type(const char *name)
{
    if (!g_have_color || name == NULL) return 0;
    if (strcmp(name, "beacon") == 0)         return PAIR_BEACON;
    if (strcmp(name, "tcmd_response") == 0)  return PAIR_TCMD;
    if (strcmp(name, "log") == 0)            return PAIR_LOG;
    if (strcmp(name, "bulk_file") == 0)      return PAIR_BULK;
    return 0;
}

static int row_has_error(const row_t *r)
{
    return (r->rs_errs == -2)
        || (r->hmac_ok == 0)
        || (r->crc_status == 0);
}

// One-line summary for the list. ~80-column friendly; bigger terminals
// just show more whitespace at the end. The decoded_summary's first
// line, after the leading "<type>: " prefix, is the most useful body
// preview to show inline. `index_1based` is the row's position in the
// current sorted view (1 = newest, n_rows = oldest) so the operator
// can read off the total without computing it.
static void format_list_line(const row_t *r, int index_1based,
                              char *out, size_t outn)
{
    const char *summary_first = r->summary;
    size_t prefix_n = strlen(r->type_name);
    if (prefix_n > 0
        && strncmp(summary_first, r->type_name, prefix_n) == 0
        && summary_first[prefix_n] == ':' && summary_first[prefix_n + 1] == ' ') {
        summary_first += prefix_n + 2;
    }
    const char *eol = strchr(summary_first, '\n');
    int body_len = eol ? (int)(eol - summary_first) : (int)strlen(summary_first);
    char ts_disp[40];
    format_ts(r->ts, ts_disp, sizeof ts_disp);
    snprintf(out, outn, "%6d  %-30.30s  %-13s  %-10s  %-13s  %-9s  %.*s",
             index_1based,
             ts_disp, r->tool,
             r->origin[0] ? r->origin : "-",
             r->type_name,
             r->satellite[0] ? r->satellite : "-",
             body_len, summary_first);
}

static void draw_top_bar(int cols)
{
    if (g_have_color) attron(COLOR_PAIR(PAIR_BAR));
    else              attron(A_REVERSE);
    move(0, 0);
    for (int i = 0; i < cols; i++) addch(' ');
    char buf[820];
    if (in_group) {
        snprintf(buf, sizeof buf, " packet_browser  %s", group_header);
    } else {
        snprintf(buf, sizeof buf,
                 " packet_browser  filter: type=%-13s origin=%-10s  search=\"%s\"  | %d row%s",
                 type_filter() ? type_filter() : "all",
                 origin_filter() ? origin_filter() : "all",
                 like_text, n_rows, n_rows == 1 ? "" : "s");
    }
    mvaddnstr(0, 0, buf, cols);
    if (g_have_color) attroff(COLOR_PAIR(PAIR_BAR));
    else              attroff(A_REVERSE);
}

static void draw_list(int list_top, int list_h, int cols)
{
    // Header line for column meaning. In the command-group sub-view it
    // also names the section ordering so the two parts read clearly.
    if (g_have_color) attron(A_DIM);
    char header[256];
    snprintf(header, sizeof header,
             "%6s  %-30s  %-13s  %-10s  %-13s  %-9s  %s",
             "#",
             show_local_time ? "TIMESTAMP (LOCAL)" : "TIMESTAMP (UTC)",
             "TOOL", "ORIGIN", "TYPE", "SATELLITE",
             in_group ? "SUMMARY  (responses first, then same-run/time-window)"
                      : "SUMMARY");
    mvaddnstr(list_top, 0, header, cols);
    if (g_have_color) attroff(A_DIM);

    int data_top = list_top + 1;
    int data_h   = list_h - 1;
    if (data_h < 1) return;
    if (sel < top) top = sel;
    if (sel >= top + data_h) top = sel - data_h + 1;

    for (int i = 0; i < data_h; i++) {
        int ridx = top + i;
        move(data_top + i, 0);
        clrtoeol();
        if (ridx >= n_rows) continue;
        row_t *r = &rows[ridx];
        char line[512];
        format_list_line(r, ridx + 1, line, sizeof line);

        int is_sel = (ridx == sel);
        int color = color_for_type(r->type_name);
        int has_err = row_has_error(r);

        // Selection pair: error wins over type, type wins over the
        // plain black-on-white fallback. Keeps the per-type colour
        // visible when the cursor is on the row, instead of forcing
        // every highlighted row to read as plain black-on-white.
        int sel_pair = PAIR_SEL;
        if (has_err) {
            sel_pair = PAIR_SEL_ERR;
        } else {
            switch (color) {
                case PAIR_BEACON: sel_pair = PAIR_SEL_BEACON; break;
                case PAIR_TCMD:   sel_pair = PAIR_SEL_TCMD;   break;
                case PAIR_LOG:    sel_pair = PAIR_SEL_LOG;    break;
                case PAIR_BULK:   sel_pair = PAIR_SEL_BULK;   break;
                default: /* PAIR_SEL — black on white */ break;
            }
        }
        if (is_sel) {
            if (g_have_color) attron(COLOR_PAIR(sel_pair));
            else              attron(A_REVERSE);
        } else if (has_err && g_have_color) {
            attron(COLOR_PAIR(PAIR_ERROR));
        } else if (color != 0) {
            attron(COLOR_PAIR(color));
        }

        // Col 0: "> " marker for the selected row. Col 1: "!" whenever
        // the row decoded with rs/hmac/crc trouble — visible regardless
        // of selection state or whether the terminal has colour.
        char marker[3] = { is_sel ? '>' : ' ',
                           has_err ? '!' : ' ',
                           '\0' };
        mvaddstr(data_top + i, 0, marker);
        addnstr(line, cols - 2);

        if (is_sel) {
            if (g_have_color) attroff(COLOR_PAIR(sel_pair));
            else              attroff(A_REVERSE);
        } else if (has_err && g_have_color) {
            attroff(COLOR_PAIR(PAIR_ERROR));
        } else if (color != 0) {
            attroff(COLOR_PAIR(color));
        }
    }
}

// Standard base64 alphabet for the detail pane's base64 payload view.
static const char B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// A bulk_file packet is [type:1][file_offset:4 LE][data...]; only the
// data is file content. The content views (ascii / base64) skip that
// 5-byte header so a downloaded file reads as itself. Other packet types
// have no such framing here, so they are interpreted whole.
#define BULK_FILE_PACKET_TYPE 0x10
#define BULK_FILE_HEADER_SIZE 5

// Point *data / *len at the bytes the content views should interpret for
// row r: the file data for a bulk_file, otherwise the whole payload.
// Returns 1 if the header was skipped (bulk_file), 0 otherwise.
static int payload_content_span(const row_t *r, const uint8_t **data, int *len)
{
    int is_bulk = (r->packet_type == BULK_FILE_PACKET_TYPE
                   && r->payload_len > BULK_FILE_HEADER_SIZE);
    int off = is_bulk ? BULK_FILE_HEADER_SIZE : 0;
    *data = (r->payload != NULL) ? r->payload + off : NULL;
    *len  = r->payload_len - off;
    if (*len < 0) *len = 0;
    return is_bulk;
}

// Render r's payload into the detail pane starting at row *yp, in the
// current payload_view, advancing *yp past what it drew. Hex dumps the
// whole payload (every type); ascii and base64 render the content span
// (see payload_content_span). Each mode stops at max_y and, if bytes
// remain unshown, overwrites its last row with a remainder note.
static void draw_payload(const row_t *r, int *yp, int max_y, int cols)
{
    int y = *yp;

    const uint8_t *data = NULL;
    int len = 0;
    int is_bulk = payload_content_span(r, &data, &len);

    char hdr[128];
    if (payload_view == PV_HEX) {
        snprintf(hdr, sizeof hdr, "payload (%d bytes) [hex]:",
                 r->payload_len);
    } else if (is_bulk) {
        snprintf(hdr, sizeof hdr,
                 "payload (%d bytes) [%s; file data %d B, header skipped]:",
                 r->payload_len, PV_NAME[payload_view], len);
    } else {
        snprintf(hdr, sizeof hdr, "payload (%d bytes) [%s]:",
                 r->payload_len, PV_NAME[payload_view]);
    }
    move(y, 0); clrtoeol();
    mvaddnstr(y, 2, hdr, cols - 2);
    y++;

    if (payload_view == PV_HEX) {
        int per_line = (cols - 6) / 3;          // "xx " per byte
        if (per_line < 8)  per_line = 8;
        if (per_line > 32) per_line = 32;
        int shown = 0;
        while (y < max_y && shown < r->payload_len) {
            char line[128];
            int pos = 0;
            int end = shown + per_line;
            if (end > r->payload_len) end = r->payload_len;
            for (int i = shown; i < end && pos + 3 < (int)sizeof line; i++) {
                pos += snprintf(line + pos, sizeof line - pos, "%02x ",
                                r->payload[i]);
            }
            move(y, 0); clrtoeol();
            mvaddnstr(y, 4, line, cols - 4);
            y++;
            shown = end;
        }
        if (shown < r->payload_len && y > 0) {
            move(y - 1, 0); clrtoeol();
            mvprintw(y - 1, 4,
                     "... %d more bytes (packet_query --format=raw)",
                     r->payload_len - shown);
        }
    } else if (payload_view == PV_ASCII) {
        // Sanitised text: printable bytes verbatim, tab -> space, a
        // newline breaks to the next row (so a text file reads as
        // itself), every other byte -> '.'. Long unbroken runs wrap at
        // the pane width.
        if (len == 0) {
            move(y, 0); clrtoeol();
            mvaddnstr(y, 4, "(no content bytes)", cols - 4);
            y++;
        } else {
            int wrap_w = cols - 4;
            if (wrap_w < 8) wrap_w = 8;
            int col = 0, i = 0;
            move(y, 0); clrtoeol();
            for (; i < len && y < max_y; i++) {
                uint8_t b = data[i];
                if (b == '\n') {
                    y++;
                    if (y < max_y) { move(y, 0); clrtoeol(); }
                    col = 0;
                    continue;
                }
                if (col >= wrap_w) {
                    y++;
                    if (y >= max_y) break;
                    move(y, 0); clrtoeol();
                    col = 0;
                }
                char c = (b >= 0x20 && b < 0x7F) ? (char)b
                       : (b == '\t')             ? ' '
                       :                           '.';
                mvaddch(y, 4 + col, (chtype)c);
                col++;
            }
            if (y < max_y) y++;
            if (i < len && y > 0) {
                move(y - 1, 0); clrtoeol();
                mvprintw(y - 1, 4, "... %d more bytes (packet_query)",
                         len - i);
            }
        }
    } else {  // PV_BASE64
        // Standard base64 of the content bytes, wrapped to whole 4-char
        // quanta per row, so the operator can copy the block out and
        // decode it elsewhere.
        if (len == 0) {
            move(y, 0); clrtoeol();
            mvaddnstr(y, 4, "(no content bytes)", cols - 4);
            y++;
        } else {
            char line[256];
            int per_line = cols - 4;
            if (per_line > (int)sizeof line - 4)
                per_line = (int)sizeof line - 4;
            per_line -= per_line % 4;            // whole base64 quanta per row
            if (per_line < 4) per_line = 4;
            int pos = 0, i = 0;
            for (; i < len && y < max_y; i += 3) {
                int b0 = data[i];
                int b1 = (i + 1 < len) ? data[i + 1] : 0;
                int b2 = (i + 2 < len) ? data[i + 2] : 0;
                uint32_t v = ((uint32_t)b0 << 16)
                           | ((uint32_t)b1 << 8)
                           | (uint32_t)b2;
                line[pos++] = B64[(v >> 18) & 0x3F];
                line[pos++] = B64[(v >> 12) & 0x3F];
                line[pos++] = (i + 1 < len) ? B64[(v >> 6) & 0x3F] : '=';
                line[pos++] = (i + 2 < len) ? B64[v & 0x3F]        : '=';
                if (pos >= per_line) {
                    line[pos] = '\0';
                    move(y, 0); clrtoeol();
                    mvaddnstr(y, 4, line, cols - 4);
                    y++;
                    pos = 0;
                }
            }
            if (pos > 0 && y < max_y) {
                line[pos] = '\0';
                move(y, 0); clrtoeol();
                mvaddnstr(y, 4, line, cols - 4);
                y++;
            }
            if (i < len && y > 0) {
                move(y - 1, 0); clrtoeol();
                mvprintw(y - 1, 4, "... %d more bytes (packet_query)",
                         len - i);
            }
        }
    }

    *yp = y;
}

static void draw_detail(int top_y, int height, int cols)
{
    move(top_y, 0);
    clrtoeol();
    if (g_have_color) attron(A_DIM);
    for (int i = 0; i < cols; i++) mvaddch(top_y, i, '-');
    if (g_have_color) attroff(A_DIM);
    int y = top_y + 1;
    int max_y = top_y + height;
    if (n_rows == 0 || sel < 0 || sel >= n_rows) {
        for (int i = y; i < max_y; i++) {
            move(i, 0); clrtoeol();
        }
        if (n_rows == 0) {
            mvaddstr(y, 2, "(no packets match the current filter)");
        }
        return;
    }
    row_t *r = &rows[sel];

    // Header row in detail panel.
    char head[512];
    char ts_disp[40];
    format_ts(r->ts, ts_disp, sizeof ts_disp);
    snprintf(head, sizeof head,
             "id=%lld  ts=%s  type=%s  tool=%s  origin=%s  run=%s",
             (long long)r->id, ts_disp, r->type_name, r->tool,
             r->origin[0] ? r->origin : "-", r->run);
    move(y, 0); clrtoeol();
    if (g_have_color) attron(A_BOLD);
    mvaddnstr(y, 2, head, cols - 2);
    if (g_have_color) attroff(A_BOLD);
    y++;

    // Metadata row.
    char meta[256];
    snprintf(meta, sizeof meta,
             "csp src=%d dst=%d dport=%d sport=%d prio=%d flags=0x%02x  "
             "rs=%d golay=%d hmac=%s crc=%s%s",
             r->csp_src, r->csp_dst, r->csp_dport, r->csp_sport,
             r->csp_prio, r->csp_flags,
             r->rs_errs, r->golay_errs,
             r->hmac_ok == 1 ? "ok" : r->hmac_ok == 0 ? "MISMATCH" : "off",
             r->crc_status == 1 ? "ok" : r->crc_status == 0 ? "MISMATCH" : "n/a",
             r->has_offset ? "" : "");
    move(y, 0); clrtoeol();
    mvaddnstr(y, 2, meta, cols - 2);
    y++;
    if (r->has_offset) {
        char ofs[64];
        snprintf(ofs, sizeof ofs, "audio_offset_s=%.3f", r->audio_offset_s);
        move(y, 0); clrtoeol();
        mvaddnstr(y, 2, ofs, cols - 2);
        y++;
    }
    if (r->has_geom) {
        char geom[256];
        int gp = 0;
        gp += snprintf(geom + gp, sizeof geom - gp, "geom:");
        if (r->geom_az_valid)
            gp += snprintf(geom + gp, sizeof geom - gp, " az=%.2f°", r->geom_az_deg);
        if (r->geom_el_valid)
            gp += snprintf(geom + gp, sizeof geom - gp, " el=%.2f°", r->geom_el_deg);
        if (r->geom_range_valid)
            gp += snprintf(geom + gp, sizeof geom - gp, " range=%.1fkm", r->geom_range_km);
        if (r->geom_range_rate_valid)
            gp += snprintf(geom + gp, sizeof geom - gp, " rate=%+.3fkm/s", r->geom_range_rate_km_s);
        if (r->geom_doppler_valid)
            gp += snprintf(geom + gp, sizeof geom - gp, " doppler=%+.0fHz", r->geom_doppler_hz);
        move(y, 0); clrtoeol();
        mvaddnstr(y, 2, geom, cols - 2);
        y++;
    }
    // station line — only when the operator toggled it on (`s`) and
    // the row is a satnogs capture (others have no meta.json to read).
    if (g_show_station && strcmp(r->origin, "satnogs") == 0) {
        char st[256];
        if (station_cache_for(r->session_dir)) {
            snprintf(st, sizeof st,
                     "station: %s (id=%d) lat=%.4f° lng=%.4f° alt=%dm",
                     g_station_cache_name[0] ? g_station_cache_name : "?",
                     g_station_cache_id,
                     g_station_cache_lat, g_station_cache_lng,
                     (int)g_station_cache_alt);
        } else {
            const char *obs_id = r->session_dir[0]
                ? strrchr(r->session_dir, '/') : NULL;
            obs_id = obs_id ? obs_id + 1
                            : (r->session_dir[0] ? r->session_dir : "?");
            snprintf(st, sizeof st,
                     "station: (no meta.json for obs %.64s)", obs_id);
        }
        move(y, 0); clrtoeol();
        mvaddnstr(y, 2, st, cols - 2);
        y++;
    }

    // Decoded body, line by line. Each logical line is wrapped across
    // as many physical rows as it needs rather than truncated at the
    // right edge — the tcmd_response text in particular routinely runs
    // past the screen width.
    const char *p = r->summary;
    int wrap_w = cols - 2;
    if (wrap_w < 1) wrap_w = 1;
    while (*p != '\0' && y < max_y) {
        const char *eol = strchr(p, '\n');
        int n = eol ? (int)(eol - p) : (int)strlen(p);
        int off = 0;
        do {
            int chunk = n - off;
            if (chunk > wrap_w) chunk = wrap_w;
            move(y, 0); clrtoeol();
            mvaddnstr(y, 2, p + off, chunk);
            y++;
            off += chunk;
        } while (off < n && y < max_y);
        if (!eol) break;
        p = eol + 1;
    }

    // Payload dump filling the rest of the detail pane, in the operator's
    // chosen view mode (hex / ascii / base64; cycle with `v`).
    if (y < max_y - 1) {
        draw_payload(r, &y, max_y, cols);
    }
    while (y < max_y) {
        move(y, 0); clrtoeol(); y++;
    }
}

static void draw_bottom_bar(int cols, int rows_total, int searching)
{
    if (g_have_color) attron(COLOR_PAIR(PAIR_BAR));
    else              attron(A_REVERSE);
    move(rows_total - 1, 0);
    for (int i = 0; i < cols; i++) addch(' ');
    // ASCII only — narrow ncurses' mvaddnstr counts bytes while the
    // terminal renders columns, so multi-byte chars cause stale tail
    // content on the next render.
    const char *hint;
    if (searching) {
        hint = " enter accept   esc cancel   backspace edits ";
    } else if (in_group) {
        hint = " esc/left/bksp back   up/down scroll   r reload   l utc/lt   v view   q quit ";
    } else {
        hint = " q quit   up/down scroll   enter group   t type   o origin   / search   l utc/lt   v view   r reload ";
    }
    mvaddnstr(rows_total - 1, 0, hint, cols);
    if (g_have_color) attroff(COLOR_PAIR(PAIR_BAR));
    else              attroff(A_REVERSE);
}

static double monotonic_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

// Modal LIKE-search prompt. Reads a line into like_text, returns 1 if
// the user accepted (Enter) or 0 if cancelled (Esc). Blocks input
// during the prompt — fine since the rest of the UI is read-only and
// the operator's full attention is on the line editor.
static int prompt_search(int rows_total, int cols)
{
    char buf[sizeof like_text];
    snprintf(buf, sizeof buf, "%s", like_text);
    size_t len = strlen(buf);
    nodelay(stdscr, FALSE);
    curs_set(1);
    while (1) {
        if (g_have_color) attron(COLOR_PAIR(PAIR_BAR));
        else              attron(A_REVERSE);
        move(rows_total - 1, 0);
        for (int i = 0; i < cols; i++) addch(' ');
        char line[256];
        snprintf(line, sizeof line, " /%s", buf);
        mvaddnstr(rows_total - 1, 0, line, cols);
        if (g_have_color) attroff(COLOR_PAIR(PAIR_BAR));
        else              attroff(A_REVERSE);
        move(rows_total - 1, 2 + (int)len);
        refresh();

        int ch = getch();
        if (ch == '\n' || ch == '\r' || ch == KEY_ENTER) {
            snprintf(like_text, sizeof like_text, "%s", buf);
            curs_set(0); nodelay(stdscr, TRUE);
            return 1;
        }
        if (ch == 27) {
            // Esc — cancel without changing like_text.
            curs_set(0); nodelay(stdscr, TRUE);
            return 0;
        }
        if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {
            if (len > 0) buf[--len] = '\0';
        } else if (ch >= 0x20 && ch < 0x7F && len + 1 < sizeof buf) {
            buf[len++] = (char)ch;
            buf[len] = '\0';
        }
        // Anything else: ignore.
    }
}

static void usage(FILE *out, const char *argv0)
{
    fprintf(out,
        "usage: %s [--db=<path>]\n"
        "\n"
        "Curses TUI over the AX100 packet DB. Read-only — safe to run\n"
        "alongside a live receiver that's filling the same DB.\n"
        "\n"
        "Keys:\n"
        "  q | Q | Esc      quit (in the command group, step back to the list)\n"
        "  arrows / PgUp / PgDn / Home / End   scroll the list\n"
        "  Enter            on a tcmd_response, open the command group: every\n"
        "                   packet sharing that command's ts_sent (its ack +\n"
        "                   responses), then same-run log/bulk_file packets in\n"
        "                   the following window (a time heuristic, not\n"
        "                   firmware-confirmed). Filters are ignored in the\n"
        "                   group. Esc / Left / Backspace step back.\n"
        "  r                reload (rebuilds the group when one is open)\n"
        "  t                cycle type filter (all → beacon → tcmd_response\n"
        "                   → log → bulk_file → all)\n"
        "  o                cycle capture-origin filter (all → cts_ground\n"
        "                   → satnogs → all)\n"
        "  /                start a substring search against the firmware-\n"
        "                   interpreted text. Enter applies, Esc cancels.\n"
        "  l                toggle timestamp display: UTC (storage) ↔ local\n"
        "  s                toggle the recording-station summary in the\n"
        "                   detail panel (satnogs rows only; the values\n"
        "                   come from <session>/satnogs_<id>.meta.json).\n"
        "  v                cycle the detail-pane payload view: hex → ascii\n"
        "                   → base64. A bulk_file's ascii/base64 views show\n"
        "                   the file data (after the 5-byte type+offset\n"
        "                   header); hex always shows the whole payload.\n"
        "\n"
        "Options:\n"
        "  --db=<path>      override default DB path. Default, in order:\n"
        "                   $SSO_PACKET_DB, else <root>/packet_db.sqlite\n"
        "                   where <root> is $FRONTIERSAT_ROOT if set,\n"
        "                   else /FrontierSat\n"
        "  --help           this message\n",
        argv0);
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "packet_browser")) return 0;
    const char *db_path = NULL;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (starts_with(a, "--db="))  db_path = a + 5;
        else { usage(stderr, argv[0]); return 1; }
    }

    char default_db[1024];
    if (db_path == NULL) {
        if (packet_db_default_path(default_db, sizeof default_db) != 0) {
            fprintf(stderr, "packet_browser: cannot resolve default DB path "
                    "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = default_db;
    }

    sqlite3 *db = NULL;
    // Open RW (not READONLY) so SQLite can create/mmap the -shm and -wal
    // sidecar files. Under READONLY a WAL database silently falls back to
    // "rollback-journal emulation", whose SHARED read lock blocks the
    // writer (simple_sat_ops) and causes its inserts to time out with
    // "database is locked". We never issue any UPDATE/INSERT/DELETE here.
    if (sqlite3_open_v2(db_path, &db,
                        SQLITE_OPEN_READWRITE, NULL) != SQLITE_OK) {
        fprintf(stderr, "packet_browser: open(%s) failed: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "?");
        if (db) sqlite3_close(db);
        return 1;
    }
    sqlite3_busy_timeout(db, 5000);

    if (initscr() == NULL) {
        sqlite3_close(db);
        fprintf(stderr, "packet_browser: ncurses initscr failed\n");
        return 1;
    }
    cbreak();
    noecho();
    nonl();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    // A lone Esc is also the first byte of every arrow/function-key
    // escape sequence, so ncurses waits ESCDELAY (default 1000 ms) for a
    // continuation before reporting a bare Esc. That made Esc-to-back /
    // Esc-to-quit feel sluggish next to an instant 'q'. 25 ms is long
    // enough for a local or SSH terminal to deliver the rest of a real
    // escape sequence, short enough that a deliberate Esc feels immediate.
    set_escdelay(25);
    curs_set(0);
    if (has_colors()) {
        start_color();
        use_default_colors();
        // Selection background: prefer a mid-tone gray from the
        // xterm 256-colour ramp (index 240 ~ Grey35) when the
        // terminal supports it; fall back to COLOR_WHITE on
        // 8/16-colour terminals where there's no gray slot.
        short sel_bg = (COLORS >= 256) ? 240 : COLOR_WHITE;
        init_pair(PAIR_BAR,    COLOR_WHITE,   COLOR_BLUE);
        init_pair(PAIR_BEACON, COLOR_CYAN,    -1);
        init_pair(PAIR_TCMD,   COLOR_YELLOW,  -1);
        init_pair(PAIR_LOG,    COLOR_GREEN,   -1);
        init_pair(PAIR_BULK,   COLOR_MAGENTA, -1);
        init_pair(PAIR_ERROR,  COLOR_RED,     -1);
        init_pair(PAIR_SEL,        COLOR_WHITE,   sel_bg);
        init_pair(PAIR_SEL_ERR,    COLOR_RED,     sel_bg);
        // Per-type selection pairs preserve the row's type-colour
        // foreground while applying the highlight background.
        init_pair(PAIR_SEL_BEACON, COLOR_CYAN,    sel_bg);
        init_pair(PAIR_SEL_TCMD,   COLOR_YELLOW,  sel_bg);
        init_pair(PAIR_SEL_LOG,    COLOR_GREEN,   sel_bg);
        init_pair(PAIR_SEL_BULK,   COLOR_MAGENTA, sel_bg);
        g_have_color = 1;
    }

    run_query(db);
    double last_query = monotonic_seconds();
    int quit = 0;
    while (!quit) {
        int rows_total = LINES;
        int cols       = COLS;
        int header_h   = 1;
        int footer_h   = 1;
        int avail      = rows_total - header_h - footer_h;
        if (avail < 6) avail = 6;
        int list_h     = avail / 2;
        if (list_h < 4) list_h = 4;
        int detail_top = header_h + list_h;
        int detail_h   = rows_total - footer_h - detail_top;

        erase();
        draw_top_bar(cols);
        draw_list(header_h, list_h, cols);
        draw_detail(detail_top, detail_h, cols);
        draw_bottom_bar(cols, rows_total, /*searching=*/0);
        refresh();

        timeout(250);
        int ch = getch();
        if (ch != ERR) {
            switch (ch) {
            case 'q': case 'Q':
                // In the sub-view, q steps back to the main list; in the
                // main list it quits (Esc/Left/Backspace also step back).
                if (in_group) leave_group(); else quit = 1;
                break;
            case 27: case KEY_LEFT: case KEY_BACKSPACE: case 127: case 8:
            case 'h':
                if (in_group) leave_group();
                else if (ch == 27) quit = 1;
                break;
            case '\n': case '\r': case KEY_ENTER: case KEY_RIGHT:
                // Enter / Right: open the command-group sub-view for the
                // selected tcmd_response (no-op on other rows or when a
                // sub-view is already open).
                if (!in_group) enter_group(db);
                break;
            case KEY_UP:   case 'k': if (sel > 0) sel--; break;
            case KEY_DOWN: case 'j': if (sel < n_rows - 1) sel++; break;
            case KEY_PPAGE:
            case 2:  // Ctrl-B — vim page up
                sel -= list_h - 1;
                if (sel < 0) sel = 0;
                break;
            case KEY_NPAGE:
            case 6:  // Ctrl-F — vim page down
                sel += list_h - 1;
                if (sel >= n_rows) sel = n_rows - 1;
                if (sel < 0) sel = 0;
                break;
            case 4:  // Ctrl-D — half page down
                sel += (list_h - 1) / 2;
                if (sel >= n_rows) sel = n_rows - 1;
                if (sel < 0) sel = 0;
                break;
            case 21: // Ctrl-U — half page up
                sel -= (list_h - 1) / 2;
                if (sel < 0) sel = 0;
                break;
            case 5:  // Ctrl-E — scroll viewport down (keep sel in view)
                if (top < n_rows - 1) {
                    top++;
                    if (sel < top) sel = top;
                }
                break;
            case 25: // Ctrl-Y — scroll viewport up
                if (top > 0) {
                    top--;
                    int data_h = list_h - 1;
                    if (sel >= top + data_h) sel = top + data_h - 1;
                }
                break;
            case 'H': {  // top of viewport
                int data_h = list_h - 1;
                (void) data_h;
                sel = top;
                if (sel >= n_rows) sel = n_rows - 1;
                if (sel < 0) sel = 0;
                break;
            }
            case 'M': {  // middle of viewport
                int data_h = list_h - 1;
                sel = top + data_h / 2;
                if (sel >= n_rows) sel = n_rows - 1;
                if (sel < 0) sel = 0;
                break;
            }
            case KEY_HOME: case 'g': sel = 0; break;
            case KEY_END:  case 'G':
                sel = n_rows > 0 ? n_rows - 1 : 0; break;
            case 'z': {
                // vim z-prefix: zz center, zt top, zb bottom (of viewport).
                // Brief blocking wait so the next keystroke is captured;
                // restore non-blocking mode afterwards.
                timeout(500);
                int next = getch();
                timeout(0);
                int data_h = list_h - 1;
                if (data_h < 1) break;
                if (next == 'z') {
                    top = sel - data_h / 2;
                } else if (next == 't') {
                    top = sel;
                } else if (next == 'b') {
                    top = sel - data_h + 1;
                } else {
                    break;
                }
                if (top < 0) top = 0;
                if (top > n_rows - 1) top = n_rows > 0 ? n_rows - 1 : 0;
                break;
            }
            case 'r': case 'R': case 18: // Ctrl-R
                // Reload: rebuild the group in the sub-view, else re-run
                // the main filter query.
                if (in_group) build_group(db);
                else { run_query(db); last_query = monotonic_seconds(); }
                break;
            case 't':
                // Filter cycles only apply to the main list — the
                // sub-view deliberately ignores filters.
                if (in_group) break;
                type_idx = (type_idx + 1) % TYPE_CYCLE_N;
                run_query(db);
                last_query = monotonic_seconds();
                break;
            case 'o': case 'O':
                if (in_group) break;
                origin_idx = (origin_idx + 1) % ORIGIN_CYCLE_N;
                run_query(db);
                last_query = monotonic_seconds();
                break;
            case 'l':
                show_local_time = !show_local_time;
                break;
            case 's':
                g_show_station = !g_show_station;
                break;
            case 'v':
                // Cycle the detail-pane payload view: hex -> ascii ->
                // base64. Applies to every row; a bulk_file's ascii /
                // base64 views show the file data after the 5-byte
                // header. No requery — the next redraw renders the mode.
                payload_view = (payload_view + 1) % 3;
                break;
            case 'L': {  // bottom of viewport (vim convention)
                int data_h = list_h - 1;
                sel = top + data_h - 1;
                if (sel >= n_rows) sel = n_rows - 1;
                if (sel < 0) sel = 0;
                break;
            }
            case '/':
                if (in_group) break;  // search applies to the main list only
                if (prompt_search(rows_total, cols)) {
                    run_query(db);
                    last_query = monotonic_seconds();
                }
                break;
            case KEY_RESIZE:
                // ncurses already updated LINES/COLS; the next loop
                // iteration redraws.
                break;
            }
        }

        // 1 Hz auto-poll so live decodes from a running receiver appear
        // without manual reload. The dedup in the DB means we always
        // see the same rows in the same order. Suspended while the
        // command-group sub-view is up so it doesn't clobber group_rows
        // (and so the parked main view stays put for an instant return).
        double now = monotonic_seconds();
        if (!in_group && now - last_query >= 1.0) {
            run_query(db);
            last_query = now;
        }
    }

    endwin();
    // Free both backing stores. While the sub-view is up the main store
    // still holds its parked rows; in the main view group_rows was freed
    // on the last leave_group (group_n == 0). free(NULL) is safe either way.
    free_rows(main_rows, in_group ? main_n : n_rows);
    free_rows(group_rows, group_n);
    sqlite3_close(db);
    return 0;
}

#endif
