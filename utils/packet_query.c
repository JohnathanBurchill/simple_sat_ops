/*

    Simple Satellite Operations  utils/packet_query.c

    Non-interactive query CLI over the packet DB written by the live and
    offline AX100 receivers. Reads only — never writes — so it's safe to
    run while a receiver is filling the DB in another shell.

    Examples:
      packet_query --since=24h --type=tcmd_response
      packet_query --type=beacon --like='%eps_mode=SAFETY%' --format=json
      packet_query --type=beacon --limit=1 --format=raw  > beacon.bin
      packet_query --since=7d --source-tool=rx_replay --format=csv > replay.csv

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

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef WITH_SQLITE3
int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    fprintf(stderr,
            "packet_query: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

enum format { FMT_TABLE, FMT_JSON, FMT_CSV, FMT_RAW };

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static void usage(FILE *out, const char *argv0)
{
    fprintf(out,
        "usage: %s [options]\n"
        "\n"
        "Non-interactive query over the AX100 packet DB.\n"
        "\n"
        "Filters (all optional, AND'd together):\n"
        "  --since=<spec>           24h | 7d | 30m | 90s | ISO-8601\n"
        "  --until=<spec>           same syntax as --since (default: now)\n"
        "  --type=<name|0xNN>       beacon|log|tcmd_response|bulk_file|0xNN\n"
        "  --satellite=<name>       e.g. CTS1\n"
        "  --source-tool=<tool>     rx_live|rx_replay|b210_rx_live|rx_decode\n"
        "  --capture-origin=<name>  cts_ground|satnogs (audio provenance)\n"
        "  --like=<pattern>         SQL LIKE pattern matched against the\n"
        "                           decoded_summary text. Use %% as the\n"
        "                           wildcard. Example: --like='%%eps_mode=SAFETY%%'\n"
        "  --limit=<n>              default 100; --limit=0 means unlimited\n"
        "  --order=asc|desc         default desc (newest first)\n"
        "\n"
        "Output:\n"
        "  --format=table|json|csv|raw     default table\n"
        "    table: human-readable, one header line per row plus the\n"
        "           firmware-interpreted body indented underneath\n"
        "    json:  JSON array of objects, one per row, payload hex-encoded\n"
        "    csv:   header row + data rows (decoded_summary collapsed)\n"
        "    raw:   payload bytes to stdout. Requires --limit=1; useful\n"
        "           piped to xxd or rx_decode --ref-hex.\n"
        "  --local-time             render ts in the operator's local TZ\n"
        "                           (filtering / sorting still UTC)\n"
        "\n"
        "Database:\n"
        "  --db=<path>              override default DB path. Default:\n"
        "                           $SSO_PACKET_DB or\n"
        "                           $HOME/.local/share/simple_sat_ops/packets.db\n"
        "  --help                   this message\n",
        argv0);
}

// Convert a stored ISO-8601 UTC timestamp into the operator's local
// time. Accepts both the millisecond ("YYYY-MM-DDTHH:MM:SS.fffZ") and
// second-precision forms; anything else falls through unchanged so a
// malformed cell is still visible in the output.
static void format_display_ts(const char *iso, int local,
                              char *out, size_t outn)
{
    if (iso == NULL || iso[0] == '\0') {
        if (outn > 0) out[0] = '\0';
        return;
    }
    if (!local) {
        snprintf(out, outn, "%s", iso);
        return;
    }
    int yr, mo, dd, hh, mm, ss, ms = 0;
    int got = sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2d.%3d",
                     &yr, &mo, &dd, &hh, &mm, &ss, &ms);
    if (got < 6) { snprintf(out, outn, "%s", iso); return; }
    struct tm utc = {0};
    utc.tm_year = yr - 1900;
    utc.tm_mon  = mo - 1;
    utc.tm_mday = dd;
    utc.tm_hour = hh;
    utc.tm_min  = mm;
    utc.tm_sec  = ss;
    time_t epoch = timegm(&utc);
    if (epoch == (time_t)-1) { snprintf(out, outn, "%s", iso); return; }
    struct tm lt;
    localtime_r(&epoch, &lt);
    char base[40];
    strftime(base, sizeof base, "%Y-%m-%d %H:%M:%S", &lt);
    const char *tz = tzname[lt.tm_isdst > 0 ? 1 : 0];
    if (tz == NULL) tz = "";
    snprintf(out, outn, "%s.%03d %s", base, ms, tz);
}

// Parse a "since/until" spec into ISO-8601 UTC for SQL comparison.
// Accepts:
//   - relative duration ending in s|m|h|d (90s, 30m, 24h, 7d)
//   - any ISO-8601-ish string (passed through as-is — sqlite text
//     comparison sorts ISO-8601 lexicographically so partial strings
//     like "2026-05-08" work as a since-filter too)
// Returns 0 on success, -1 if the spec couldn't be parsed.
static int parse_time_spec(const char *spec, char *out, size_t outn)
{
    if (spec == NULL || spec[0] == '\0') return -1;
    size_t len = strlen(spec);
    char unit = spec[len - 1];
    if (unit == 's' || unit == 'm' || unit == 'h' || unit == 'd') {
        char *endp = NULL;
        long n = strtol(spec, &endp, 10);
        if (endp == spec || endp != spec + len - 1 || n <= 0) return -1;
        long sec = (unit == 's') ? n
                 : (unit == 'm') ? n * 60
                 : (unit == 'h') ? n * 3600
                 :                 n * 86400;
        time_t now = time(NULL);
        time_t cutoff = now - sec;
        struct tm utc;
        gmtime_r(&cutoff, &utc);
        strftime(out, outn, "%Y-%m-%dT%H:%M:%SZ", &utc);
        return 0;
    }
    // ISO-8601 (or partial) — pass through.
    if (len + 1 > outn) return -1;
    memcpy(out, spec, len + 1);
    return 0;
}

// Parse a --type value into either a name (returns name string) or a
// numeric byte (returns NULL, *out_byte set). Used to decide which SQL
// column to filter against.
static int parse_type_filter(const char *spec,
                             const char **out_name, int *out_byte)
{
    *out_name = NULL;
    *out_byte = -1;
    if (strcmp(spec, "beacon") == 0
        || strcmp(spec, "log") == 0
        || strcmp(spec, "tcmd_response") == 0
        || strcmp(spec, "bulk_file") == 0) {
        *out_name = spec;
        return 0;
    }
    if (starts_with(spec, "0x") || starts_with(spec, "0X")) {
        char *endp = NULL;
        long v = strtol(spec + 2, &endp, 16);
        if (endp == spec + 2 || *endp != '\0' || v < 0 || v > 255) return -1;
        *out_byte = (int)v;
        return 0;
    }
    char *endp = NULL;
    long v = strtol(spec, &endp, 10);
    if (endp != spec && *endp == '\0' && v >= 0 && v <= 255) {
        *out_byte = (int)v;
        return 0;
    }
    return -1;
}

// JSON-escape a string into out. Always writes a NUL. Truncates if the
// result wouldn't fit; the truncation is intentional — packet_query is
// for ad-hoc queries, not bulk export.
static void json_escape(const char *in, char *out, size_t outn)
{
    if (outn == 0) return;
    size_t o = 0;
    for (size_t i = 0; in[i] != '\0' && o + 8 < outn; i++) {
        unsigned char c = (unsigned char)in[i];
        switch (c) {
            case '"':  out[o++] = '\\'; out[o++] = '"'; break;
            case '\\': out[o++] = '\\'; out[o++] = '\\'; break;
            case '\n': out[o++] = '\\'; out[o++] = 'n'; break;
            case '\r': out[o++] = '\\'; out[o++] = 'r'; break;
            case '\t': out[o++] = '\\'; out[o++] = 't'; break;
            default:
                if (c < 0x20) {
                    o += snprintf(out + o, outn - o, "\\u%04x", c);
                } else {
                    out[o++] = (char)c;
                }
                break;
        }
    }
    out[o < outn ? o : outn - 1] = '\0';
}

// CSV-quote: wrap in "..." if the field contains comma, quote, or
// newline. Embedded quotes become "". Truncates like json_escape.
static void csv_escape(const char *in, char *out, size_t outn)
{
    if (outn == 0) return;
    int needs_quote = 0;
    for (size_t i = 0; in[i] != '\0'; i++) {
        if (in[i] == ',' || in[i] == '"' || in[i] == '\n' || in[i] == '\r') {
            needs_quote = 1;
            break;
        }
    }
    size_t o = 0;
    if (needs_quote && o + 1 < outn) out[o++] = '"';
    for (size_t i = 0; in[i] != '\0' && o + 2 < outn; i++) {
        if (in[i] == '"') {
            if (o + 2 < outn) { out[o++] = '"'; out[o++] = '"'; }
        } else {
            out[o++] = in[i];
        }
    }
    if (needs_quote && o + 1 < outn) out[o++] = '"';
    out[o < outn ? o : outn - 1] = '\0';
}

// Hex-encode a BLOB into out (NUL-terminated). Truncates at outn-1 hex
// chars if the blob is too big.
static void hex_encode(const uint8_t *bytes, size_t n,
                       char *out, size_t outn)
{
    static const char hex[] = "0123456789abcdef";
    if (outn == 0) return;
    size_t max = (outn - 1) / 2;
    if (n > max) n = max;
    for (size_t i = 0; i < n; i++) {
        out[2 * i + 0] = hex[(bytes[i] >> 4) & 0xF];
        out[2 * i + 1] = hex[bytes[i] & 0xF];
    }
    out[2 * n] = '\0';
}

int main(int argc, char **argv)
{
    const char *db_path = NULL;
    const char *since = NULL;
    const char *until = NULL;
    const char *type_arg = NULL;
    const char *satellite = NULL;
    const char *source_tool = NULL;
    const char *capture_origin = NULL;
    const char *like_arg = NULL;
    long limit = 100;
    int order_desc = 1;
    enum format fmt = FMT_TABLE;
    int local_time = 0;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (starts_with(a, "--db="))           db_path     = a + 5;
        else if (starts_with(a, "--since="))        since       = a + 8;
        else if (starts_with(a, "--until="))        until       = a + 8;
        else if (starts_with(a, "--type="))         type_arg    = a + 7;
        else if (starts_with(a, "--satellite="))    satellite   = a + 12;
        else if (starts_with(a, "--source-tool="))  source_tool = a + 14;
        else if (starts_with(a, "--capture-origin=")) capture_origin = a + 17;
        else if (starts_with(a, "--like="))         like_arg    = a + 7;
        else if (starts_with(a, "--limit=")) {
            limit = strtol(a + 8, NULL, 10);
            if (limit < 0) limit = 0;
        } else if (strcmp(a, "--order=asc") == 0) {
            order_desc = 0;
        } else if (strcmp(a, "--order=desc") == 0) {
            order_desc = 1;
        } else if (strcmp(a, "--format=table") == 0) fmt = FMT_TABLE;
        else if (strcmp(a, "--format=json") == 0)    fmt = FMT_JSON;
        else if (strcmp(a, "--format=csv") == 0)     fmt = FMT_CSV;
        else if (strcmp(a, "--format=raw") == 0)     fmt = FMT_RAW;
        else if (strcmp(a, "--local-time") == 0)     local_time = 1;
        else {
            fprintf(stderr, "packet_query: unknown option '%s'\n", a);
            usage(stderr, argv[0]);
            return 1;
        }
    }

    if (fmt == FMT_RAW && limit != 1) {
        fprintf(stderr, "packet_query: --format=raw needs --limit=1 "
                "(got limit=%ld); refusing to dump multiple BLOBs to stdout\n",
                limit);
        return 1;
    }

    char default_db[1024];
    if (db_path == NULL) {
        if (packet_db_default_path(default_db, sizeof default_db) != 0) {
            fprintf(stderr, "packet_query: cannot resolve default DB path "
                    "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = default_db;
    }

    char since_iso[40] = {0};
    char until_iso[40] = {0};
    if (since != NULL && parse_time_spec(since, since_iso, sizeof since_iso) != 0) {
        fprintf(stderr, "packet_query: bad --since=%s\n", since); return 1;
    }
    if (until != NULL && parse_time_spec(until, until_iso, sizeof until_iso) != 0) {
        fprintf(stderr, "packet_query: bad --until=%s\n", until); return 1;
    }

    const char *type_name = NULL;
    int type_byte = -1;
    if (type_arg != NULL && parse_type_filter(type_arg, &type_name, &type_byte) != 0) {
        fprintf(stderr,
                "packet_query: bad --type=%s (use beacon|log|tcmd_response|"
                "bulk_file|0xNN)\n", type_arg);
        return 1;
    }

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "packet_query: open(%s): %s\n",
                db_path, db ? sqlite3_errmsg(db) : "unknown");
        if (db) sqlite3_close(db);
        return 1;
    }

    char sql[2048];
    int n_params = 0;
    int sql_off = snprintf(sql, sizeof sql,
        "SELECT id, ts_received, satellite, packet_type, packet_type_name, "
        "csp_src, csp_dst, csp_dport, csp_sport, csp_prio, csp_flags, "
        "payload, golay_errs, rs_errs, hmac_ok, crc_status, "
        "source_tool, source_run, audio_offset_s, decoded_summary, "
        "az_deg, el_deg, range_km, range_rate_km_s, doppler_hz_offset, "
        "tle_id, session_dir, capture_origin "
        "FROM packet WHERE 1=1");

    // Slot 0..n_params-1 in these arrays mirror SQL ?1..?n_params.
    enum { TXT, INT_ };
    int param_kind[16];
    const char *param_text[16];
    long param_int[16];

#define ADD_PARAM_TXT(s) do { \
        param_kind[n_params] = TXT; \
        param_text[n_params] = (s); \
        n_params++; \
    } while (0)
#define ADD_PARAM_INT(v) do { \
        param_kind[n_params] = INT_; \
        param_int[n_params] = (v); \
        n_params++; \
    } while (0)

    if (since_iso[0]) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " AND ts_received >= ?%d", n_params + 1);
        ADD_PARAM_TXT(since_iso);
    }
    if (until_iso[0]) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " AND ts_received <= ?%d", n_params + 1);
        ADD_PARAM_TXT(until_iso);
    }
    if (type_name != NULL) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " AND packet_type_name = ?%d", n_params + 1);
        ADD_PARAM_TXT(type_name);
    } else if (type_byte >= 0) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " AND packet_type = ?%d", n_params + 1);
        ADD_PARAM_INT(type_byte);
    }
    if (satellite != NULL) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " AND satellite = ?%d", n_params + 1);
        ADD_PARAM_TXT(satellite);
    }
    if (source_tool != NULL) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " AND source_tool = ?%d", n_params + 1);
        ADD_PARAM_TXT(source_tool);
    }
    if (capture_origin != NULL) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " AND capture_origin = ?%d", n_params + 1);
        ADD_PARAM_TXT(capture_origin);
    }
    if (like_arg != NULL) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " AND decoded_summary LIKE ?%d", n_params + 1);
        ADD_PARAM_TXT(like_arg);
    }

    sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                        " ORDER BY ts_received %s",
                        order_desc ? "DESC" : "ASC");
    if (limit > 0) {
        sql_off += snprintf(sql + sql_off, sizeof sql - sql_off,
                            " LIMIT ?%d", n_params + 1);
        ADD_PARAM_INT(limit);
    }

    sqlite3_stmt *stmt = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK) {
        fprintf(stderr, "packet_query: prepare failed: %s\n",
                sqlite3_errmsg(db));
        sqlite3_close(db);
        return 1;
    }
    for (int i = 0; i < n_params; i++) {
        if (param_kind[i] == TXT) {
            sqlite3_bind_text(stmt, i + 1, param_text[i], -1, SQLITE_TRANSIENT);
        } else {
            sqlite3_bind_int64(stmt, i + 1, (sqlite3_int64)param_int[i]);
        }
    }

    int row_count = 0;
    if (fmt == FMT_JSON) printf("[");
    if (fmt == FMT_CSV) {
        printf("id,ts_received,satellite,packet_type,packet_type_name,"
               "csp_src,csp_dst,csp_dport,csp_sport,csp_prio,csp_flags,"
               "golay_errs,rs_errs,hmac_ok,crc_status,"
               "source_tool,source_run,audio_offset_s,"
               "az_deg,el_deg,range_km,range_rate_km_s,doppler_hz_offset,"
               "tle_id,session_dir,capture_origin,payload_hex,decoded_summary\n");
    }
    if (fmt == FMT_TABLE) {
        printf("%-7s %-30s %-13s %-10s %-15s %-9s %-7s\n",
               "ID",
               local_time ? "TIMESTAMP (LOCAL)" : "TIMESTAMP (UTC)",
               "TOOL", "ORIGIN", "TYPE", "SATELLITE", "RS");
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        long long id     = sqlite3_column_int64(stmt, 0);
        const char *ts   = (const char *)sqlite3_column_text(stmt, 1);
        const char *sat  = (const char *)sqlite3_column_text(stmt, 2);
        int ptype        = sqlite3_column_int(stmt, 3);
        const char *pname= (const char *)sqlite3_column_text(stmt, 4);
        int csp_src      = sqlite3_column_int(stmt, 5);
        int csp_dst      = sqlite3_column_int(stmt, 6);
        int csp_dport    = sqlite3_column_int(stmt, 7);
        int csp_sport    = sqlite3_column_int(stmt, 8);
        int csp_prio     = sqlite3_column_int(stmt, 9);
        int csp_flags    = sqlite3_column_int(stmt, 10);
        const uint8_t *payload = sqlite3_column_blob(stmt, 11);
        int payload_n    = sqlite3_column_bytes(stmt, 11);
        int golay_errs   = sqlite3_column_int(stmt, 12);
        int rs_errs      = sqlite3_column_int(stmt, 13);
        int hmac_ok      = sqlite3_column_int(stmt, 14);
        int crc_status   = sqlite3_column_int(stmt, 15);
        const char *tool = (const char *)sqlite3_column_text(stmt, 16);
        const char *run  = (const char *)sqlite3_column_text(stmt, 17);
        int has_offset   = sqlite3_column_type(stmt, 18) != SQLITE_NULL;
        double aoff      = has_offset ? sqlite3_column_double(stmt, 18) : 0.0;
        const char *summary = (const char *)sqlite3_column_text(stmt, 19);
        int has_az = sqlite3_column_type(stmt, 20) != SQLITE_NULL;
        double az  = has_az ? sqlite3_column_double(stmt, 20) : 0.0;
        int has_el = sqlite3_column_type(stmt, 21) != SQLITE_NULL;
        double el  = has_el ? sqlite3_column_double(stmt, 21) : 0.0;
        int has_range = sqlite3_column_type(stmt, 22) != SQLITE_NULL;
        double range_km = has_range ? sqlite3_column_double(stmt, 22) : 0.0;
        int has_rrate = sqlite3_column_type(stmt, 23) != SQLITE_NULL;
        double range_rate = has_rrate ? sqlite3_column_double(stmt, 23) : 0.0;
        int has_dop = sqlite3_column_type(stmt, 24) != SQLITE_NULL;
        double doppler_hz = has_dop ? sqlite3_column_double(stmt, 24) : 0.0;
        int has_tle = sqlite3_column_type(stmt, 25) != SQLITE_NULL;
        long long tle_id = has_tle ? sqlite3_column_int64(stmt, 25) : 0;
        const char *session_dir = (const char *)sqlite3_column_text(stmt, 26);
        const char *capture_origin_row = (const char *)sqlite3_column_text(stmt, 27);

        char ts_disp[64];
        format_display_ts(ts, local_time, ts_disp, sizeof ts_disp);

        switch (fmt) {
        case FMT_TABLE: {
            char azel[40] = "";
            if (has_az && has_el) {
                snprintf(azel, sizeof azel, "  az=%.1f° el=%+.1f°", az, el);
            }
            printf("%-7lld %-30s %-13s %-10s %-15s %-9s %d%s\n",
                   id, ts_disp[0] ? ts_disp : "?", tool ? tool : "?",
                   capture_origin_row ? capture_origin_row : "-",
                   pname ? pname : "?", sat ? sat : "-", rs_errs, azel);
            if (summary != NULL) {
                // Indent each line of summary under the row header.
                const char *p = summary;
                while (*p != '\0') {
                    const char *eol = strchr(p, '\n');
                    int n = eol ? (int)(eol - p) : (int)strlen(p);
                    printf("        %.*s\n", n, p);
                    if (!eol) break;
                    p = eol + 1;
                    if (*p == '\0') break;
                }
            }
            break;
        }
        case FMT_JSON: {
            char ts_e[64], tool_e[64], pname_e[32], sat_e[32];
            char run_e[64], summary_e[3072];
            char payload_hex[1024];
            json_escape(ts_disp[0] ? ts_disp : "", ts_e, sizeof ts_e);
            json_escape(tool ? tool : "", tool_e, sizeof tool_e);
            json_escape(pname ? pname : "", pname_e, sizeof pname_e);
            json_escape(sat ? sat : "", sat_e, sizeof sat_e);
            json_escape(run ? run : "", run_e, sizeof run_e);
            json_escape(summary ? summary : "", summary_e, sizeof summary_e);
            hex_encode(payload, (size_t)payload_n,
                       payload_hex, sizeof payload_hex);
            if (row_count > 0) printf(",");
            printf("\n  {");
            printf("\"id\":%lld,", id);
            printf("\"ts\":\"%s\",", ts_e);
            printf("\"tool\":\"%s\",", tool_e);
            printf("\"run\":\"%s\",", run_e);
            printf("\"type\":%d,", ptype);
            printf("\"type_name\":\"%s\",", pname_e);
            printf("\"satellite\":%s%s%s,",
                   sat ? "\"" : "", sat ? sat_e : "null", sat ? "\"" : "");
            printf("\"csp\":{\"src\":%d,\"dst\":%d,\"dport\":%d,"
                   "\"sport\":%d,\"prio\":%d,\"flags\":%d},",
                   csp_src, csp_dst, csp_dport, csp_sport,
                   csp_prio, csp_flags);
            printf("\"rs_errs\":%d,\"golay_errs\":%d,"
                   "\"hmac_ok\":%d,\"crc_status\":%d,",
                   rs_errs, golay_errs, hmac_ok, crc_status);
            if (has_offset) printf("\"audio_offset_s\":%.3f,", aoff);
            else            printf("\"audio_offset_s\":null,");
            printf("\"observer\":{");
            if (has_az)    printf("\"az_deg\":%.3f,", az); else printf("\"az_deg\":null,");
            if (has_el)    printf("\"el_deg\":%.3f,", el); else printf("\"el_deg\":null,");
            if (has_range) printf("\"range_km\":%.3f,", range_km); else printf("\"range_km\":null,");
            if (has_rrate) printf("\"range_rate_km_s\":%.4f,", range_rate); else printf("\"range_rate_km_s\":null,");
            if (has_dop)   printf("\"doppler_hz_offset\":%.1f", doppler_hz); else printf("\"doppler_hz_offset\":null");
            printf("},");
            if (has_tle) printf("\"tle_id\":%lld,", tle_id);
            else         printf("\"tle_id\":null,");
            if (session_dir) {
                char sd_e[512];
                json_escape(session_dir, sd_e, sizeof sd_e);
                printf("\"session_dir\":\"%s\",", sd_e);
            } else {
                printf("\"session_dir\":null,");
            }
            if (capture_origin_row) {
                char co_e[64];
                json_escape(capture_origin_row, co_e, sizeof co_e);
                printf("\"capture_origin\":\"%s\",", co_e);
            } else {
                printf("\"capture_origin\":null,");
            }
            printf("\"payload_hex\":\"%s\",", payload_hex);
            printf("\"summary\":\"%s\"", summary_e);
            printf("}");
            break;
        }
        case FMT_CSV: {
            char ts_q[80], tool_q[40], pname_q[40], sat_q[40], run_q[40];
            char summary_q[3072], payload_hex[1024];
            csv_escape(ts_disp[0] ? ts_disp : "", ts_q, sizeof ts_q);
            csv_escape(tool ? tool : "", tool_q, sizeof tool_q);
            csv_escape(pname ? pname : "", pname_q, sizeof pname_q);
            csv_escape(sat ? sat : "", sat_q, sizeof sat_q);
            csv_escape(run ? run : "", run_q, sizeof run_q);
            csv_escape(summary ? summary : "", summary_q, sizeof summary_q);
            hex_encode(payload, (size_t)payload_n,
                       payload_hex, sizeof payload_hex);
            printf("%lld,%s,%s,%d,%s,"
                   "%d,%d,%d,%d,%d,%d,"
                   "%d,%d,%d,%d,"
                   "%s,%s,",
                   id, ts_q, sat_q, ptype, pname_q,
                   csp_src, csp_dst, csp_dport, csp_sport,
                   csp_prio, csp_flags,
                   golay_errs, rs_errs, hmac_ok, crc_status,
                   tool_q, run_q);
            if (has_offset) printf("%.3f,", aoff); else printf(",");
            if (has_az)     printf("%.4f,", az); else printf(",");
            if (has_el)     printf("%.4f,", el); else printf(",");
            if (has_range)  printf("%.3f,", range_km); else printf(",");
            if (has_rrate)  printf("%.4f,", range_rate); else printf(",");
            if (has_dop)    printf("%.1f,", doppler_hz); else printf(",");
            if (has_tle)    printf("%lld,", tle_id); else printf(",");
            char sd_q[512];
            csv_escape(session_dir ? session_dir : "", sd_q, sizeof sd_q);
            printf("%s,", sd_q);
            char co_q[64];
            csv_escape(capture_origin_row ? capture_origin_row : "", co_q, sizeof co_q);
            printf("%s,", co_q);
            printf("%s,%s\n", payload_hex, summary_q);
            break;
        }
        case FMT_RAW:
            (void)fwrite(payload, 1, (size_t)payload_n, stdout);
            break;
        }
        row_count++;
    }
    sqlite3_finalize(stmt);
    sqlite3_close(db);

    if (fmt == FMT_JSON) {
        if (row_count > 0) printf("\n");
        printf("]\n");
    }
    return 0;
}

#endif // WITH_SQLITE3
