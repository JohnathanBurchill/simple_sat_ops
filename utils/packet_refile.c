/*

    Simple Satellite Operations  utils/packet_refile.c

    File the packet DB's "unknown" rows under the packet types the decoder
    knows now.

    The decoder names a packet's type when it stores it, and leaves a frame
    it doesn't recognize as type 0, "unknown", with no decoded text. When the
    ground station learns a new type -- the extended beacon (0x20), the GNSS
    ring blob's samples (0x30) -- the rows stored before that stay unknown,
    and decoding the old recordings again doesn't change them: the DB keeps
    the first row for a payload (INSERT OR IGNORE). This tool runs the
    decoder's own classifier (cts1_classify) over every unknown row and, for
    each one it now recognizes, sets the type, its name, the satellite (when
    empty) and the decoded summary, exactly as a fresh decode would store
    them. Rows it still can't name are left alone, so running it again is
    harmless.

    Writes go in short transactions, so a decoder writing to the same DB
    waits at most a moment. Back up the DB first if you want to be able to
    go back.

    Examples:
      packet_refile --dry-run            # what would change, by type
      packet_refile                      # file them
      packet_refile --db=/FrontierSat/packet_db.sqlite

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
#include "gnss_frag.h"
#include "packet_db.h"
#include "sso_version.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef WITH_SQLITE3
int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "packet_refile")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "packet_refile: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

// Rows written per transaction.
#define BATCH 500

typedef struct {
    const char *db_path;
    int         dry_run;
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
        if (strcmp(arg, "--dry-run") == 0 || help) {
            if (help) parse_help_line(OPTW, "--dry-run", "count what would be filed, change nothing");
            else a->dry_run = 1;
            matched = 1;
        }
        if ((strcmp(arg, "-V") == 0 || strcmp(arg, "--version") == 0) || help) {
            if (help) parse_help_line(OPTW, "-V, --version", "print version and exit");
            // -V is handled in main via sso_version_handle before parsing.
            matched = 1;
        }
        if (!help && !matched) {
            fprintf(stderr, "packet_refile: unknown option '%s' (try --help)\n", arg);
            return PARSE_ERROR;
        }
    }
    return help ? PARSE_HELP : PARSE_OK;
}

typedef struct {
    long long      id;
    int            type;
    const char    *name;        // static strings from cts1_classify
    const char    *satellite;
    unsigned char *payload;
    int            len;
} refile_t;

typedef struct { const char *name; long n; } tally_t;

static void tally_add(tally_t *t, int *nt, int cap, const char *name)
{
    for (int i = 0; i < *nt; ++i)
        if (strcmp(t[i].name, name) == 0) { t[i].n++; return; }
    if (*nt < cap) { t[*nt].name = name; t[*nt].n = 1; (*nt)++; }
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "packet_refile")) return 0;

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
            fprintf(stderr, "packet_refile: no DB path "
                            "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = db_default;
    }

    sqlite3 *db = NULL;
    int flags = cfg.dry_run ? SQLITE_OPEN_READONLY : SQLITE_OPEN_READWRITE;
    if (sqlite3_open_v2(db_path, &db, flags, NULL) != SQLITE_OK) {
        fprintf(stderr, "packet_refile: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 1;
    }
    // Wait out a decoder's insert rather than failing on it.
    sqlite3_busy_timeout(db, 5000);

    // The decoder classifies only frames whose CSP header decoded, which it
    // stores CSP-stripped as "unknown"; "unparsed" rows hold the whole frame,
    // header and all, so the classifier doesn't apply to them.
    sqlite3_stmt *st = NULL;
    const char *sel = "SELECT id, payload FROM packet "
                      "WHERE packet_type = 0 AND packet_type_name = 'unknown' ORDER BY id";
    if (sqlite3_prepare_v2(db, sel, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "packet_refile: query failed: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
        return 1;
    }
    refile_t *rows = NULL;
    long nunknown = 0, nrows = 0, cap = 0;
    tally_t tally[16];
    int ntally = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        nunknown++;
        const unsigned char *pl = sqlite3_column_blob(st, 1);
        int len = sqlite3_column_bytes(st, 1);
        const char *name = NULL, *sat = NULL;
        int type = cts1_classify(pl, (size_t) len, &name, &sat);
        if (type == 0 || name == NULL) continue;
        tally_add(tally, &ntally, 16, name);
        if (cfg.dry_run) continue;
        if (nrows == cap) {
            long nc = cap ? cap * 2 : 4096;
            refile_t *t = realloc(rows, (size_t) nc * sizeof *t);
            if (t == NULL) { fprintf(stderr, "packet_refile: out of memory\n"); return 1; }
            rows = t;
            cap = nc;
        }
        refile_t *r = &rows[nrows];
        r->id = sqlite3_column_int64(st, 0);
        r->type = type;
        r->name = name;
        r->satellite = sat;
        r->len = len;
        r->payload = malloc((size_t) len);
        if (r->payload == NULL) { fprintf(stderr, "packet_refile: out of memory\n"); return 1; }
        memcpy(r->payload, pl, (size_t) len);
        nrows++;
    }
    sqlite3_finalize(st);

    long nfound = 0;
    for (int i = 0; i < ntally; ++i) nfound += tally[i].n;
    printf("%s: %ld unknown rows, %ld %s a known type\n", db_path, nunknown, nfound,
           cfg.dry_run ? "would be filed under" : "to file under");
    for (int i = 0; i < ntally; ++i) printf("  %-14s %ld\n", tally[i].name, tally[i].n);
    if (cfg.dry_run || nrows == 0) {
        sqlite3_close(db);
        return 0;
    }

    // WHERE packet_type = 0 keeps a row someone else filed meanwhile as it is.
    const char *upd = "UPDATE packet SET packet_type = ?1, packet_type_name = ?2, "
                      "satellite = COALESCE(satellite, ?3), decoded_summary = ?4 "
                      "WHERE id = ?5 AND packet_type = 0";
    if (sqlite3_prepare_v2(db, upd, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "packet_refile: update failed: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
        return 1;
    }
    long done = 0;
    int rc = 0;
    for (long i = 0; i < nrows && rc == 0; i += BATCH) {
        if (sqlite3_exec(db, "BEGIN IMMEDIATE", NULL, NULL, NULL) != SQLITE_OK) {
            fprintf(stderr, "packet_refile: cannot start a write: %s\n", sqlite3_errmsg(db));
            rc = 1;
            break;
        }
        long end = i + BATCH < nrows ? i + BATCH : nrows, changed = 0;
        for (long k = i; k < end; ++k) {
            // Same 3 KiB buffer the decoder uses for decoded_summary.
            char summary[3072];
            size_t sn = cts1_packet_summary(summary, sizeof summary,
                                            rows[k].payload, (size_t) rows[k].len);
            sqlite3_reset(st);
            sqlite3_bind_int(st, 1, rows[k].type);
            sqlite3_bind_text(st, 2, rows[k].name, -1, SQLITE_STATIC);
            if (rows[k].satellite) sqlite3_bind_text(st, 3, rows[k].satellite, -1, SQLITE_STATIC);
            else                   sqlite3_bind_null(st, 3);
            if (sn > 0) sqlite3_bind_text(st, 4, summary, -1, SQLITE_TRANSIENT);
            else        sqlite3_bind_null(st, 4);
            sqlite3_bind_int64(st, 5, rows[k].id);
            if (sqlite3_step(st) != SQLITE_DONE) {
                fprintf(stderr, "packet_refile: update of row %lld failed: %s\n",
                        rows[k].id, sqlite3_errmsg(db));
                rc = 1;
                break;
            }
            changed += sqlite3_changes(db);
        }
        if (sqlite3_exec(db, rc ? "ROLLBACK" : "COMMIT", NULL, NULL, NULL) != SQLITE_OK) {
            fprintf(stderr, "packet_refile: commit failed: %s\n", sqlite3_errmsg(db));
            rc = 1;
        } else if (rc == 0) {
            done += changed;
        }
    }
    sqlite3_finalize(st);
    sqlite3_close(db);
    for (long k = 0; k < nrows; ++k) free(rows[k].payload);
    free(rows);

    printf("filed %ld rows%s\n", done,
           rc ? " before an error; run it again to finish" : "");
    return rc;
}

#endif
