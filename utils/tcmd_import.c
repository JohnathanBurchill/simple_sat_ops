/*

    Simple Satellite Operations  utils/tcmd_import.c

    Backfill the sent_tcmd table from historical tx.log files.

    simple_sat_ops records every telecommand it transmits into the
    sent_tcmd table going forward, but passes flown before that table
    existed only left their record in the per-pass tx.log. Each tx.log is
    JSON-lines (one encoded event per line); a transmitted command shows
    up as a "tx-command-sent" event whose "ascii" field is the payload
    that went on the air ("ascii:CTS1+...@tssent=...!"). This tool reads
    those lines and inserts the ones carrying an @tssent into sent_tcmd,
    so packet_browser can resolve an old tcmd_response back to its command.

    Only @tssent-bearing commands are imported -- that value is the join
    key the satellite echoes back. Hand-composed commands without an
    @tssent (and binary "hex:" payloads) are skipped, matching the live
    recording rule. Re-running is safe: inserts collide on
    UNIQUE(ts_sent_ms, source_run) and are ignored.

    Usage:
      tcmd_import [--db=<path>] [--source-tool=<name>] [<dir-or-file> ...]

    With no path, the Operations directory under the data root is scanned
    recursively for files named "tx.log".

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

#include "agenda_line.h"
#include "packet_db.h"
#include "sso_paths.h"

#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

// Pull the string value of a top-level JSON field `key` out of one
// encoded event line into `out` (NUL-terminated, minimally unescaped).
// Returns 1 on success, 0 if the key is absent or not a string. Naive
// but sufficient for the flat, machine-written tx.log lines: the keys
// don't nest and none is a tail of another (see header).
static int json_str(const char *line, const char *key, char *out, size_t outn)
{
    if (outn == 0) return 0;
    out[0] = '\0';
    char needle[32];
    if ((size_t) snprintf(needle, sizeof needle, "\"%s\"", key) >= sizeof needle)
        return 0;
    const char *p = strstr(line, needle);
    if (p == NULL) return 0;
    p += strlen(needle);
    while (*p == ' ' || *p == '\t') p++;
    if (*p != ':') return 0;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') return 0;
    p++;
    size_t i = 0;
    while (*p != '\0' && *p != '"' && i + 1 < outn) {
        if (*p == '\\' && p[1] != '\0') {
            p++;
            char c = *p;
            if (c == 'n') c = '\n';
            else if (c == 't') c = '\t';
            // \" \\ \/ and anything else: pass the escaped char through.
            out[i++] = c;
            p++;
            continue;
        }
        out[i++] = *p++;
    }
    out[i] = '\0';
    return 1;
}

// Name of the pass folder holding `txlog_path` (its parent directory's
// basename), used as the sent_tcmd source_run for provenance + dedup.
static void folder_name(const char *txlog_path, char *out, size_t outn)
{
    char tmp[1024];
    snprintf(tmp, sizeof tmp, "%s", txlog_path);
    char *slash = strrchr(tmp, '/');     // drop "/tx.log"
    if (slash) *slash = '\0';
    const char *base = strrchr(tmp, '/');
    base = base ? base + 1 : tmp;
    snprintf(out, outn, "%s", base[0] ? base : "tx_log_import");
}

// Import one tx.log file. Returns the number of @tssent-bearing
// tx-command-sent lines handed to the DB (duplicates included; the DB
// ignores them).
static int import_file(packet_db_t *db, const char *path, const char *tool)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "tcmd_import: cannot open %s\n", path);
        return 0;
    }
    char run[128];
    folder_name(path, run, sizeof run);

    int n = 0;
    char line[4096];
    while (fgets(line, sizeof line, f) != NULL) {
        if (strstr(line, "\"t\":\"tx-command-sent\"") == NULL) continue;

        char ascii[512];
        if (!json_str(line, "ascii", ascii, sizeof ascii)) continue;
        const char *cmd = ascii;
        if (strncmp(cmd, "ascii:", 6) == 0)      cmd += 6;
        else if (strncmp(cmd, "hex:", 4) == 0)   continue;  // binary, no @tssent

        long long ts_sent = -1;
        if (!agenda_parse_directive_ms(cmd, "@tssent=", &ts_sent)) continue;
        long long tsexec = -1;
        agenda_parse_directive_ms(cmd, "@tsexec=", &tsexec);

        char ts[40];
        if (!json_str(line, "ts", ts, sizeof ts)) snprintf(ts, sizeof ts, "(imported)");

        sent_tcmd_record_t rec = {0};
        rec.ts_sent_ms     = ts_sent;
        rec.tsexec_ms      = tsexec;     // -1 -> stored NULL
        rec.command_text   = cmd;
        rec.tx_freq_hz     = 0;          // not recorded in the tx.log event
        rec.tx_gain_db     = NAN;        // -> stored NULL
        rec.source_tool    = tool;
        rec.source_run     = run;
        rec.ts_transmitted = ts;
        if (packet_db_insert_sent_tcmd(db, &rec) == 0) n++;
    }
    fclose(f);
    if (n > 0) fprintf(stderr, "tcmd_import: %-40s %d command(s)\n", path, n);
    return n;
}

// Recursively import every file named "tx.log" under `dir`.
static int import_tree(packet_db_t *db, const char *dir, const char *tool,
                       int depth)
{
    if (depth > 12) return 0;
    DIR *d = opendir(dir);
    if (d == NULL) {
        fprintf(stderr, "tcmd_import: cannot open dir %s\n", dir);
        return 0;
    }
    int total = 0;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char path[1024];
        if ((size_t) snprintf(path, sizeof path, "%s/%s", dir, de->d_name)
            >= sizeof path) continue;
        // lstat (not stat) so symlinks aren't followed: Operations/current
        // points at the active pass folder, which is reached on its own
        // anyway -- following it would import that pass's tx.log twice,
        // once under source_run "current" and once under its real name.
        struct stat st;
        if (lstat(path, &st) != 0) continue;
        if (S_ISLNK(st.st_mode)) continue;
        if (S_ISDIR(st.st_mode))
            total += import_tree(db, path, tool, depth + 1);
        else if (S_ISREG(st.st_mode) && strcmp(de->d_name, "tx.log") == 0)
            total += import_file(db, path, tool);
    }
    closedir(d);
    return total;
}

// Import a path that may be a single tx.log file or a directory tree.
static int import_path(packet_db_t *db, const char *path, const char *tool)
{
    struct stat st;
    if (stat(path, &st) != 0) {
        fprintf(stderr, "tcmd_import: no such path: %s\n", path);
        return 0;
    }
    if (S_ISDIR(st.st_mode)) return import_tree(db, path, tool, 0);
    return import_file(db, path, tool);
}

static void usage(FILE *out, const char *argv0)
{
    fprintf(out,
        "usage: %s [--db=<path>] [--source-tool=<name>] [<dir-or-file> ...]\n"
        "\n"
        "Backfill the sent_tcmd table from historical tx.log files so\n"
        "packet_browser can resolve old command responses to the command\n"
        "that produced them. Each tx.log is scanned for transmitted\n"
        "commands carrying an @tssent (the value the satellite echoes back).\n"
        "Hand-composed commands without an @tssent are skipped. Safe to\n"
        "re-run: duplicates are ignored.\n"
        "\n"
        "Arguments:\n"
        "  <dir-or-file>    a tx.log file, or a directory searched\n"
        "                   recursively for files named tx.log. With none,\n"
        "                   the Operations directory under the data root\n"
        "                   is scanned.\n"
        "\n"
        "Options:\n"
        "  --db=<path>      DB to write. Default: $SSO_PACKET_DB, else\n"
        "                   <root>/packet_db.sqlite ($FRONTIERSAT_ROOT if\n"
        "                   set, else /FrontierSat).\n"
        "  --source-tool=<name>  provenance tag stored on each row\n"
        "                   (default: tx_log_import).\n"
        "  --help           this message\n",
        argv0);
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "tcmd_import")) return 0;
    const char *db_path = NULL;
    const char *tool = "tx_log_import";
    const char *paths[64];
    int n_paths = 0;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (strncmp(a, "--db=", 5) == 0)          db_path = a + 5;
        else if (strncmp(a, "--source-tool=", 14) == 0) tool = a + 14;
        else if (a[0] == '-') { usage(stderr, argv[0]); return 1; }
        else if (n_paths < (int)(sizeof paths / sizeof paths[0]))
            paths[n_paths++] = a;
    }

    char default_db[1024];
    if (db_path == NULL) {
        if (packet_db_default_path(default_db, sizeof default_db) != 0) {
            fprintf(stderr, "tcmd_import: cannot resolve default DB path "
                    "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = default_db;
    }

    packet_db_t *db = packet_db_open(db_path);
    if (db == NULL) {
        fprintf(stderr, "tcmd_import: cannot open DB %s "
                "(built without sqlite3?)\n", db_path);
        return 1;
    }

    int total = 0;
    if (n_paths == 0) {
        const char *ops = sso_operations_dir();
        fprintf(stderr, "tcmd_import: scanning %s\n", ops);
        total = import_tree(db, ops, tool, 0);
    } else {
        for (int i = 0; i < n_paths; i++)
            total += import_path(db, paths[i], tool);
    }

    packet_db_close(db);
    fprintf(stderr,
            "tcmd_import: done -- %d command(s) imported into %s "
            "(duplicates ignored)\n", total, db_path);
    return 0;
}
