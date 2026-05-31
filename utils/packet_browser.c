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
      q | Q | Esc      quit
      ↑ / ↓            move selection by one row
      PgUp / PgDn      move by a page (list height)
      Home / End       jump to top / bottom
      r                reload now (auto-poll happens every ~1 s anyway)
      t                cycle type filter: all → beacon → tcmd_response →
                       log → bulk_file → all
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

static row_t   rows[MAX_ROWS];
static int     n_rows = 0;
static int     sel    = 0;
static int     top    = 0;
static int     type_idx = 0;
static int     origin_idx = 0;
static char    like_text[128] = "";
// Display mode: 0 = UTC (storage form, ISO-8601 Z), 1 = local time
// (parsed back to time_t and re-formatted with tzname). Filtering and
// sorting still happens server-side against the UTC strings; only the
// rendered cells change. Toggle with `L`.
static int     show_local_time = 0;

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

// Run the current filter against the DB and refresh `rows` / `n_rows`.
// Selection (`sel`) is preserved by row id where possible — feeling
// like the row "stays in place" across reloads is more important than
// always landing on the freshest packet. If the previously-selected id
// is gone, fall back to position 0.
static void run_query(sqlite3 *db)
{
    sqlite3_int64 prev_id = (n_rows > 0) ? rows[sel].id : -1;

    char sql[1024];
    int off = snprintf(sql, sizeof sql,
        "SELECT id, ts_received, satellite, packet_type, packet_type_name, "
        "csp_src, csp_dst, csp_dport, csp_sport, csp_prio, csp_flags, "
        "payload, golay_errs, rs_errs, hmac_ok, crc_status, "
        "source_tool, source_run, audio_offset_s, decoded_summary, "
        "capture_origin, az_deg, el_deg, range_km, range_rate_km_s, "
        "doppler_hz_offset, session_dir "
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
    for (int i = 0; i < n_rows; i++) {
        free(rows[i].payload);
        rows[i].payload = NULL;
        rows[i].payload_len = 0;
    }

    int new_n = 0;
    while (new_n < MAX_ROWS && sqlite3_step(stmt) == SQLITE_ROW) {
        row_t *r = &rows[new_n];
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

        new_n++;
    }
    sqlite3_finalize(stmt);
    n_rows = new_n;

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
    char buf[256];
    snprintf(buf, sizeof buf,
             " packet_browser  filter: type=%-13s origin=%-10s  search=\"%s\"  | %d row%s",
             type_filter() ? type_filter() : "all",
             origin_filter() ? origin_filter() : "all",
             like_text, n_rows, n_rows == 1 ? "" : "s");
    mvaddnstr(0, 0, buf, cols);
    if (g_have_color) attroff(COLOR_PAIR(PAIR_BAR));
    else              attroff(A_REVERSE);
}

static void draw_list(int list_top, int list_h, int cols)
{
    // Header line for column meaning.
    if (g_have_color) attron(A_DIM);
    char header[256];
    snprintf(header, sizeof header,
             "%6s  %-30s  %-13s  %-10s  %-13s  %-9s  %s",
             "#",
             show_local_time ? "TIMESTAMP (LOCAL)" : "TIMESTAMP (UTC)",
             "TOOL", "ORIGIN", "TYPE", "SATELLITE", "SUMMARY");
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

    // Payload as a multi-line hex dump filling the rest of the detail
    // pane. A single line only ever showed ~cols/2 bytes; this lays the
    // whole payload out, wrapping, and notes any remainder that doesn't
    // fit the visible rows (use packet_query --format=raw for those).
    if (y < max_y - 1) {
        move(y, 0); clrtoeol();
        mvprintw(y, 2, "payload (%d bytes):", r->payload_len);
        y++;
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
            // Overwrite the last drawn row with a remainder note.
            move(y - 1, 0); clrtoeol();
            mvprintw(y - 1, 4,
                     "... %d more bytes (packet_query --format=raw)",
                     r->payload_len - shown);
        }
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
    const char *hint = searching
        ? " enter accept   esc cancel   backspace edits "
        // ASCII only — narrow ncurses' mvaddnstr counts bytes while
        // the terminal renders columns, so multi-byte chars cause
        // stale tail content on the next render.
        : " q quit   up/down scroll   t type   o origin   / search   L utc/lt   r reload ";
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
        "  q | Q | Esc      quit\n"
        "  arrows / PgUp / PgDn / Home / End   scroll the list\n"
        "  r                reload the list now\n"
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
        "\n"
        "Options:\n"
        "  --db=<path>      override default DB path. Default:\n"
        "                   $SSO_PACKET_DB or\n"
        "                   $HOME/.local/share/simple_sat_ops/packets.db\n"
        "  --help           this message\n",
        argv0);
}

int main(int argc, char **argv)
{
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
            case 'q': case 'Q': case 27:
                quit = 1; break;
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
                run_query(db);
                last_query = monotonic_seconds();
                break;
            case 't':
                type_idx = (type_idx + 1) % TYPE_CYCLE_N;
                run_query(db);
                last_query = monotonic_seconds();
                break;
            case 'o': case 'O':
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
            case 'L': {  // bottom of viewport (vim convention)
                int data_h = list_h - 1;
                sel = top + data_h - 1;
                if (sel >= n_rows) sel = n_rows - 1;
                if (sel < 0) sel = 0;
                break;
            }
            case '/':
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
        // see the same rows in the same order.
        double now = monotonic_seconds();
        if (now - last_query >= 1.0) {
            run_query(db);
            last_query = now;
        }
    }

    endwin();
    for (int i = 0; i < n_rows; i++) free(rows[i].payload);
    sqlite3_close(db);
    return 0;
}

#endif
