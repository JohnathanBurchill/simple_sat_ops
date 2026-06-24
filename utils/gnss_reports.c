/*

    Simple Satellite Operations  utils/gnss_reports.c

    Reassemble the GNSS telecommand responses stored in the packet DB and
    print each one with its NovAtel CRC verified.

    FrontierSat's GNSS receiver (a NovAtel OEM7) answers gnss_send_cmd_ascii
    telecommands with an ASCII log -- BESTXYZA, ITDETECTSTATUSA, RXCONFIGA,
    and so on -- wrapped by the firmware as "GNSS Response (N chars): ...".
    A response longer than 186 bytes is split across several tcmd_response
    packets (response_seq_num 1..max). This tool finds those fragments,
    stitches them back together by sequence number, drops the per-packet CSP
    CRC32 trailer, checks the NovAtel CRC on the recovered log, and prints
    the result. Read-only -- safe to run while a receiver is filling the DB.

    Examples:
      gnss_reports                       # every GNSS response in the DB
      gnss_reports --since=7d            # only the last week
      gnss_reports --type=BESTXYZA       # only position/velocity solutions
      gnss_reports --since=2026-06-12 --until=2026-06-13 --full

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
#include "bestxyz.h"
#include "gnss_frag.h"
#include "packet_db.h"
#include "sso_version.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef WITH_SQLITE3
int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "gnss_reports")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "gnss_reports: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

// tcmd_response packet layout (see src/beacon/beacon_cts1.h): a 14-byte
// header (packet_type, ts_sent[8], code, duration, seq, max_seq) then up to
// 186 data bytes. ts_sent occupies payload[1..8]; seq is payload[12].
#define TCMD_TYPE        0x04
#define TCMD_HDR         COMMS_TCMD_RESPONSE_HEADER_SIZE
#define TCMD_MAXDATA     COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET

// Format a unix-ms timestamp as ISO-8601 UTC with milliseconds.
static void fmt_epoch_ms(uint64_t ms, char *out, size_t outn)
{
    time_t s = (time_t)(ms / 1000);
    struct tm utc;
    gmtime_r(&s, &utc);
    char base[32];
    strftime(base, sizeof base, "%Y-%m-%dT%H:%M:%S", &utc);
    snprintf(out, outn, "%s.%03uZ", base, (unsigned)(ms % 1000));
}

typedef struct {
    const char *db_path;
    const char *since;
    const char *until;
    const char *type;     // filter to one NovAtel log type, else all
    int         full;     // print the whole reassembled log, not just a snippet
    int         summary_only;
    int         reverse;  // sort newest-first instead of oldest-first
} args_t;

#define OPTW 24

static int parse_args(args_t *a, int argc, char **argv, int help)
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
        if (gnss_starts_with(arg, "--db=") || help) {
            if (help) parse_help_line(OPTW, "--db=<path>", "override default DB path ($SSO_PACKET_DB or the default)");
            else a->db_path = arg + 5;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--since=") || help) {
            if (help) parse_help_line(OPTW, "--since=<spec>", "24h | 7d | 30m | ISO-8601 (default: all)");
            else a->since = arg + 8;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--until=") || help) {
            if (help) parse_help_line(OPTW, "--until=<spec>", "same syntax as --since (default: now)");
            else a->until = arg + 8;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--type=") || help) {
            if (help) parse_help_line(OPTW, "--type=<log>", "filter to one NovAtel log, e.g. BESTXYZA");
            else a->type = arg + 7;
            matched = 1;
        }
        if (strcmp(arg, "--full") == 0 || help) {
            if (help) parse_help_line(OPTW, "--full", "print the whole reassembled log, not a snippet");
            else a->full = 1;
            matched = 1;
        }
        if (strcmp(arg, "--summary") == 0 || help) {
            if (help) parse_help_line(OPTW, "--summary", "print only the count-by-type tally");
            else a->summary_only = 1;
            matched = 1;
        }
        if (strcmp(arg, "--reverse") == 0 || help) {
            if (help) parse_help_line(OPTW, "--reverse", "newest first (default: oldest first)");
            else a->reverse = 1;
            matched = 1;
        }
        if ((strcmp(arg, "-V") == 0 || strcmp(arg, "--version") == 0) || help) {
            if (help) parse_help_line(OPTW, "-V, --version", "print version and exit");
            // -V is handled in main via sso_version_handle before parsing.
            matched = 1;
        }
        if (!help && !matched) {
            fprintf(stderr, "gnss_reports: unknown option '%s' (try --help)\n", arg);
            return PARSE_ERROR;
        }
    }
    return help ? PARSE_HELP : PARSE_OK;
}

// All log types we count, plus the two non-log buckets.
typedef struct {
    char name[24];
    int  total, crc_ok, crc_bad, no_crc;
} tally_t;
static tally_t g_tally[32];
static int     g_ntally = 0;

static void tally_add(const char *name, int have_crc, int crc_ok)
{
    tally_t *e = NULL;
    for (int i = 0; i < g_ntally; ++i)
        if (strcmp(g_tally[i].name, name) == 0) { e = &g_tally[i]; break; }
    if (e == NULL && g_ntally < (int)(sizeof g_tally / sizeof g_tally[0])) {
        e = &g_tally[g_ntally++];
        snprintf(e->name, sizeof e->name, "%s", name);
    }
    if (e == NULL) return;
    e->total++;
    if (!have_crc)    e->no_crc++;
    else if (crc_ok)  e->crc_ok++;
    else              e->crc_bad++;
}

// A real NovAtel log name is uppercase letters/digits. Anything else means
// the header took bit errors -- treat the whole response as corrupted.
static int name_is_clean(const char *s)
{
    if (s[0] == '\0' || s[0] == '(') return 0;   // "(empty)"
    for (const char *p = s; *p; ++p)
        if (!isupper((unsigned char)*p) && !isdigit((unsigned char)*p)) return 0;
    return 1;
}

static int all_hex(const char *p, int n)
{
    for (int i = 0; i < n; ++i) if (!isxdigit((unsigned char)p[i])) return 0;
    return 1;
}

// Identify the primary NovAtel log in a reassembled GNSS response and verify
// its CRC. *out_type gets the log name (or "(empty)"), *have_crc / *crc_ok the
// verdict. start/len delimit the recovered log text (for printing).
static void analyze(const char *msg, int msglen,
                    char *out_type, size_t type_sz,
                    int *have_crc, int *crc_ok,
                    unsigned *crc_read, unsigned *crc_calc,
                    int *log_start, int *log_end)
{
    snprintf(out_type, type_sz, "%s", "(empty)");
    *have_crc = 0; *crc_ok = 0; *crc_read = 0; *crc_calc = 0;
    *log_start = 0; *log_end = msglen;

    // The receiver echoes a "[COM1]" prompt; the returned log starts at the
    // first '#' after it. Fall back to the first '#' anywhere if the prompt
    // was lost to bit errors.
    const char *com = strstr(msg, "[COM1]");
    int search = com ? (int)(com - msg) + 6 : 0;
    const char *hash = strchr(msg + search, '#');
    if (hash == NULL) return;
    int h = (int)(hash - msg);

    // The CRC is the last '*XXXXXXXX' in the recovered text.
    int star = -1;
    for (int i = msglen - 9; i > h; --i) {
        if (msg[i] == '*' && all_hex(msg + i + 1, 8)) { star = i; break; }
    }

    // Log name: '#' to the first ',' (the NovAtel header's first field).
    int e = h + 1;
    while (e < msglen && msg[e] != ',' && msg[e] != '*'
           && msg[e] != ' ' && msg[e] != '\r' && msg[e] != '\n')
        e++;
    int nlen = e - (h + 1);
    if (nlen <= 0) return;
    if (nlen >= (int)type_sz) nlen = (int)type_sz - 1;
    memcpy(out_type, msg + h + 1, (size_t)nlen);
    out_type[nlen] = '\0';

    if (star < 0) { *log_start = h; *log_end = msglen; return; }

    *crc_calc = bestxyz_novatel_crc32((const unsigned char *)(msg + h + 1),
                                      (size_t)(star - (h + 1)));
    *crc_read = (unsigned)strtoul(msg + star + 1, NULL, 16);
    *have_crc = 1;
    *crc_ok = (*crc_calc == *crc_read);
    *log_start = h;
    *log_end = star + 9;   // include '*' + 8 hex
}

// Collected output blocks, sorted by reception time before printing (the DB
// walk visits responses in ts_sent-key order, not chronological order).
typedef struct { char ts[40]; char *block; } outmsg_t;
static outmsg_t *g_msgs = NULL;
static int       g_nmsgs = 0, g_capmsgs = 0;

static void store_msg(const char *ts, const char *block)
{
    if (g_nmsgs == g_capmsgs) {
        int cap = g_capmsgs ? g_capmsgs * 2 : 64;
        outmsg_t *t = realloc(g_msgs, (size_t)cap * sizeof *t);
        if (t == NULL) return;   // drop on OOM rather than crash
        g_msgs = t; g_capmsgs = cap;
    }
    snprintf(g_msgs[g_nmsgs].ts, sizeof g_msgs[g_nmsgs].ts, "%s", ts);
    g_msgs[g_nmsgs].block = strdup(block);
    if (g_msgs[g_nmsgs].block) g_nmsgs++;
}

static int cmp_msg_asc(const void *a, const void *b)
{
    return strcmp(((const outmsg_t *)a)->ts, ((const outmsg_t *)b)->ts);
}

// Process one reception: detect a GNSS response, verify the CRC, update the
// tally, and (unless --summary) collect a formatted block for sorted output.
// Returns 1 if it was a GNSS response, else 0.
static int process(const gnss_frag_t *frags, int n, const args_t *a,
                   const char *since_iso, const char *until_iso)
{
    // Time filter on the earliest fragment (frags arrive sorted by ts_received).
    const char *ts = frags[0].ts_received;
    if (since_iso[0] && strcmp(ts, since_iso) < 0) return 0;
    if (until_iso[0] && strcmp(ts, until_iso) > 0) return 0;

    static unsigned char buf[65536];
    int len = gnss_reassemble(frags, n, buf, sizeof buf);
    char *msg = (char *)buf;

    char *marker = strstr(msg, "GNSS Response (");
    if (marker == NULL) {
        // Garbled marker but a recognisable NovAtel log? Treat as a corrupted
        // GNSS response so it's not silently dropped.
        if (!strstr(msg, "BESTXYZA") && !strstr(msg, "ITDETECTSTATUS")
            && !strstr(msg, "RXCONFIG"))
            return 0;
    }

    // Trim to the firmware-declared length so the trailing CSP CRC32 / padding
    // of the last fragment never leaks into the printed log.
    int region = len;
    if (marker) {
        int decl = 0;
        char *colon = strstr(marker, "): ");
        if (sscanf(marker, "GNSS Response (%d chars)", &decl) == 1
            && colon != NULL && decl > 0) {
            int end = (int)(colon + 3 - msg) + decl;
            if (end < region) region = end;
        }
    }
    msg[region] = '\0';

    char type[24];
    int have_crc, crc_ok, ls, le;
    unsigned cr, cc;
    analyze(msg, region, type, sizeof type, &have_crc, &crc_ok, &cr, &cc, &ls, &le);

    if (a->type && strcmp(a->type, type) != 0) return 0;

    int is_empty = (strcmp(type, "(empty)") == 0);
    int corrupt  = (marker == NULL) || (!is_empty && !name_is_clean(type));
    const char *bucket = corrupt ? "(corrupted)" : type;
    tally_add(bucket, have_crc, crc_ok);

    if (a->summary_only) return 1;

    // Build the id list. Clamp p after each append (like APP below): a
    // truncated snprintf returns the would-be length, which would push p past
    // sizeof ids and make the next "sizeof ids - p" (size_t) wrap huge.
    char ids[256]; int p = 0;
    for (int i = 0; i < n && p < (int)sizeof ids - 12; ++i) {
        p += snprintf(ids + p, sizeof ids - p, "%s%lld",
                      i ? "," : "", frags[i].id);
        if (p > (int)sizeof ids) p = (int)sizeof ids;
    }

    char tssent[40];
    uint64_t ts_sent = 0;
    for (int i = 0; i < 8; ++i) ts_sent |= (uint64_t)frags[0].tskey[i] << (8 * i);
    fmt_epoch_ms(ts_sent, tssent, sizeof tssent);

    // Format the whole block into a buffer so the caller can sort by ts before
    // printing. APP guards against overflow (off never passes the buffer size).
    char block[4096]; int off = 0;
    #define APP(...) do { \
        if (off < (int)sizeof block) { \
            int _n = snprintf(block + off, sizeof block - off, __VA_ARGS__); \
            if (_n > 0) off += _n; \
            if (off >= (int)sizeof block) off = (int)sizeof block - 1; \
        } \
    } while (0)

    APP("[%s] ids %s  ts_sent=%s  %d frag%s\n", ts, ids, tssent, n, n == 1 ? "" : "s");
    if (!have_crc)
        APP("  %s  CRC (none -- incomplete or no log)\n", bucket);
    else if (crc_ok)
        APP("  %s  CRC %08x OK\n", bucket, cr);
    else
        APP("  %s  CRC read %08x calc %08x MISMATCH\n", bucket, cr, cc);

    // For BESTXYZA, enrich with the position solution via the project parser.
    if (strcmp(type, "BESTXYZA") == 0) {
        bestxyz_t b; char err[96];
        if (bestxyz_parse(msg, &b, err, sizeof err) == 0)
            APP("    %s wk%d  %s/%s  %d/%d SV\n",
                b.time_status, b.gps_week, b.pos_sol_status, b.pos_type,
                b.num_sol_sv, b.num_sv);
    }

    // The recovered log itself.
    if (ls < le && le <= region) {
        int snip = le - ls;
        if (a->full || snip <= 96) APP("    %.*s\n", snip, msg + ls);
        else APP("    %.*s ...\n", 80, msg + ls);
    }
    APP("\n");
    #undef APP

    store_msg(ts, block);
    return 1;
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "gnss_reports")) return 0;

    args_t cfg = {0};
    switch (parse_args(&cfg, argc, argv, 0)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
        default: break;
    }

    char db_default[1024];
    const char *db_path = cfg.db_path;
    if (db_path == NULL) {
        if (packet_db_default_path(db_default, sizeof db_default) != 0) {
            fprintf(stderr, "gnss_reports: no DB path "
                            "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = db_default;
    }

    char since_iso[40] = {0}, until_iso[40] = {0};
    if (cfg.since && gnss_parse_time_spec(cfg.since, since_iso, sizeof since_iso) != 0) {
        fprintf(stderr, "gnss_reports: bad --since=%s\n", cfg.since); return 1;
    }
    if (cfg.until && gnss_parse_time_spec(cfg.until, until_iso, sizeof until_iso) != 0) {
        fprintf(stderr, "gnss_reports: bad --until=%s\n", cfg.until); return 1;
    }

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "gnss_reports: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 1;
    }

    // Pull every tcmd_response fragment, grouped by ts_sent then ordered by
    // arrival, so we can split duplicate receptions and reassemble each.
    const char *sql =
        "SELECT id, ts_received, payload FROM packet "
        "WHERE packet_type=?1 AND length(payload) >= ?2 "
        "ORDER BY substr(payload,2,8), ts_received, id";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "gnss_reports: query failed: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
        return 1;
    }
    sqlite3_bind_int(st, 1, TCMD_TYPE);
    sqlite3_bind_int(st, 2, TCMD_HDR + 1);

    // Group by ts_sent key; within a group, a new reception starts whenever
    // the sequence number does not advance (seq <= last seq seen).
    gnss_frag_t recv[260];
    int    nrecv = 0;
    unsigned char curkey[8];
    int have_key = 0, last_seq = 0;

    #define FLUSH() do { \
        if (nrecv > 0) { \
            process(recv, nrecv, &cfg, since_iso, until_iso); \
            for (int i = 0; i < nrecv; ++i) { free(recv[i].payload); } \
            nrecv = 0; \
        } \
    } while (0)

    while (sqlite3_step(st) == SQLITE_ROW) {
        const unsigned char *pl = sqlite3_column_blob(st, 2);
        int pl_len = sqlite3_column_bytes(st, 2);
        if (pl == NULL || pl_len < TCMD_HDR + 1) continue;
        int seq = pl[12];
        if (seq < 1) continue;

        unsigned char key[8];
        memcpy(key, pl + 1, 8);

        int new_group = !have_key || memcmp(key, curkey, 8) != 0;
        if (new_group) { FLUSH(); memcpy(curkey, key, 8); have_key = 1; last_seq = 0; }
        else if (seq <= last_seq) { FLUSH(); }   // duplicate reception of same ts_sent

        if (nrecv >= (int)(sizeof recv / sizeof recv[0])) FLUSH();

        gnss_frag_t *f = &recv[nrecv++];
        f->id = sqlite3_column_int64(st, 0);
        snprintf(f->ts_received, sizeof f->ts_received, "%s",
                 (const char *)sqlite3_column_text(st, 1));
        memcpy(f->tskey, key, 8);
        f->seq = seq;
        f->max_seq = pl[13];
        f->payload_len = pl_len;
        f->payload = malloc((size_t)pl_len);
        // On OOM, zero the length so reassemble skips this fragment rather
        // than dereferencing a NULL payload over pl_len bytes.
        if (f->payload) memcpy(f->payload, pl, (size_t)pl_len);
        else            f->payload_len = 0;
        last_seq = seq;
    }
    FLUSH();
    sqlite3_finalize(st);
    sqlite3_close(db);

    // Print the collected responses in chronological order (--reverse flips to
    // newest first), then the tally.
    if (!cfg.summary_only) {
        qsort(g_msgs, (size_t)g_nmsgs, sizeof *g_msgs, cmp_msg_asc);
        printf("GNSS responses%s%s%s:\n\n",
               (since_iso[0] || until_iso[0]) ? " (" : "",
               since_iso[0] ? since_iso : (until_iso[0] ? ".." : ""),
               (since_iso[0] || until_iso[0]) ? ")" : "");
        for (int k = 0; k < g_nmsgs; ++k) {
            int idx = cfg.reverse ? (g_nmsgs - 1 - k) : k;
            fputs(g_msgs[idx].block, stdout);
            free(g_msgs[idx].block);
        }
        free(g_msgs);
    }

    // Count-by-type tally.
    int grand = 0;
    printf("Count by type:\n");
    for (int i = 0; i < g_ntally; ++i) {
        tally_t *e = &g_tally[i];
        grand += e->total;
        if (e->crc_ok || e->crc_bad || e->no_crc)
            printf("  %-18s %4d   (CRC ok %d, bad %d, incomplete %d)\n",
                   e->name, e->total, e->crc_ok, e->crc_bad, e->no_crc);
        else
            printf("  %-18s %4d\n", e->name, e->total);
    }
    printf("  %-18s %4d\n", "TOTAL", grand);
    return 0;
}

#endif
