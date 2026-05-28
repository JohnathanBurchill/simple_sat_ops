/*

    Simple Satellite Operations  agenda_check.c

    Telecommand-agenda reviewer. Reads a TC file (the same format
    --tc-file= consumes in simple_sat_ops) and prints it with the
    embedded unix-millisecond timestamps converted to human-readable
    times so the operator can confirm tssent / tsexec values before
    keying any of it on air. Also flags verbatim-duplicate lines —
    same command + same times + same response file = somebody pasted
    twice, which is worth knowing.

    Input format (one TC per line; '#' and blank lines preserved):

        CTS1+hello_world()@tssent=<unix_ms>@tsexec=<unix_ms>@resp_fname=<f>!

    Any of the @-directives can be present or omitted. We only touch
    the @tssent= and @tsexec= values; everything else (including
    @resp_fname=, the leading prefix, the trailing '!') is preserved
    verbatim.

    Usage:
        agenda_check [--local-time] [--no-dup-check] [--tle <file>] [<file>]

    --local-time     human times in the host's local TZ (default UTC)
    --no-dup-check   skip the duplicate-line audit (substitute only)
    --tle <file>     (sgp4sdp4 builds only) propagate the first satellite
                     in <file> and prepend the sub-satellite latitude (deg),
                     longitude (deg) and altitude (km) to each command,
                     evaluated at @tsexec= (else @tssent=, else now)
    No <file>        read from stdin

    Duplicate lines are flagged inline with a "DUP(N)>" prefix
    pointing to the line number of the first occurrence. When stdout
    is a TTY the prefix is rendered in red bold so it stands out;
    when piped, the prefix is plain text so grep / less still see it.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifdef WITH_SGP4SDP4
// prediction.h pulls in ephemeres.h -> sgp4sdp4.h (ClearFlag,
// select_ephemeris, the propagators). We reuse next_in_queue's proven
// load_tle + update_satellite_position path rather than driving SGP4 by
// hand, which is easy to get subtly wrong.
#include "prediction.h"
#endif

static void usage(FILE *out, const char *progname)
{
    fprintf(out,
        "Usage: %s [--local-time] [--no-dup-check] [--tle <file>] [<file>]\n"
        "  Replaces @tssent=<unix_ms> and @tsexec=<unix_ms> values with\n"
        "  human-readable timestamps. UTC by default; --local-time uses\n"
        "  the host's local timezone. Flags verbatim-duplicate TC lines.\n"
        "  With --tle <file> (sgp4sdp4 builds), prepends the sub-satellite\n"
        "  lat/lon (deg) and altitude (km) per command at its exec time\n"
        "  (@tsexec=, else @tssent=, else now).\n"
        "  No <file> reads from stdin.\n",
        progname);
}

// Format `unix_ms` as "YYYY-MM-DDTHH:MM:SS.mmmZ" (UTC) or
// "YYYY-MM-DDTHH:MM:SS.mmm±HHMM" (local). Returns bytes written
// excluding NUL.
static int format_ts(long long unix_ms, int local, char *out, size_t cap)
{
    if (cap == 0) return 0;
    time_t secs = (time_t)(unix_ms / 1000);
    int    ms   = (int)(unix_ms % 1000);
    if (ms < 0) { ms += 1000; secs -= 1; }
    struct tm tm;
    if (local) {
        localtime_r(&secs, &tm);
    } else {
        gmtime_r(&secs, &tm);
    }
    char base[40];
    strftime(base, sizeof base, "%Y-%m-%dT%H:%M:%S", &tm);
    char tz[8];
    if (local) {
        if (strftime(tz, sizeof tz, "%z", &tm) == 0) {
            snprintf(tz, sizeof tz, "Z");
        }
    } else {
        snprintf(tz, sizeof tz, "Z");
    }
    return snprintf(out, cap, "%s.%03d%s", base, ms, tz);
}

// Substitute @tssent=<digits> and @tsexec=<digits> with humanized
// timestamps. The output buffer must be large enough — typical line
// growth is ~16 bytes per directive (digits -> ISO-8601 string).
// Returns 0 on success; -1 if the output buffer would overflow.
static int humanize_directives(const char *in, int local,
                                char *out, size_t out_cap)
{
    static const char *const keys[] = {"@tssent=", "@tsexec="};
    static const size_t key_lens[]  = {8u,         8u};
    const size_t n_keys = sizeof keys / sizeof keys[0];

    size_t op = 0;
    const char *p = in;
    while (*p) {
        int matched = 0;
        for (size_t k = 0; k < n_keys; ++k) {
            if (strncmp(p, keys[k], key_lens[k]) != 0) continue;
            const char *digits = p + key_lens[k];
            const char *d = digits;
            // Optional leading sign — accepting it matches the rest
            // of the project's parsers and keeps things forgiving.
            if (*d == '+' || *d == '-') ++d;
            const char *digit_start = d;
            while (isdigit((unsigned char) *d)) ++d;
            if (d == digit_start) break;
            char numbuf[32];
            size_t numlen = (size_t)(d - digits);
            if (numlen >= sizeof numbuf) break;
            memcpy(numbuf, digits, numlen);
            numbuf[numlen] = '\0';
            long long unix_ms = strtoll(numbuf, NULL, 10);
            if (op + key_lens[k] >= out_cap) return -1;
            memcpy(out + op, keys[k], key_lens[k]);
            op += key_lens[k];
            char ts[48];
            int n = format_ts(unix_ms, local, ts, sizeof ts);
            if (n < 0 || (size_t)(op + n) >= out_cap) return -1;
            memcpy(out + op, ts, (size_t) n);
            op += (size_t) n;
            p = d;
            matched = 1;
            break;
        }
        if (matched) continue;
        if (op + 1 >= out_cap) return -1;
        out[op++] = *p++;
    }
    if (op >= out_cap) return -1;
    out[op] = '\0';
    return 0;
}

static void strip_eol(char *buf)
{
    size_t n = strlen(buf);
    while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r')) {
        buf[--n] = '\0';
    }
}

// Linear-scan dedup. Telecommand schedules are short (dozens of
// lines), so the O(N^2) cost is invisible and we keep the binary
// dependency-free.
typedef struct {
    char  **lines;
    int    *line_no;     // 1-based line number where line first appeared
    size_t  n;
    size_t  cap;
} dup_table_t;

static int dup_table_push(dup_table_t *t, const char *line, int lineno,
                           int *out_first_seen)
{
    *out_first_seen = 0;
    for (size_t i = 0; i < t->n; ++i) {
        if (strcmp(t->lines[i], line) == 0) {
            *out_first_seen = t->line_no[i];
            return 0;
        }
    }
    if (t->n == t->cap) {
        size_t nc = t->cap ? t->cap * 2 : 64;
        char **nl   = realloc(t->lines,   nc * sizeof *nl);
        int   *nn   = realloc(t->line_no, nc * sizeof *nn);
        if (nl == NULL || nn == NULL) {
            free(nl); free(nn);
            return -1;
        }
        t->lines   = nl;
        t->line_no = nn;
        t->cap     = nc;
    }
    t->lines[t->n] = strdup(line);
    if (t->lines[t->n] == NULL) return -1;
    t->line_no[t->n] = lineno;
    t->n++;
    return 0;
}

static void dup_table_free(dup_table_t *t)
{
    for (size_t i = 0; i < t->n; ++i) free(t->lines[i]);
    free(t->lines);
    free(t->line_no);
    t->lines = NULL; t->line_no = NULL; t->n = t->cap = 0;
}

#ifdef WITH_SGP4SDP4
// Single-satellite propagation context, loaded once from the --tle file
// and reused for every command line.
static prediction_t g_pred;

// Find the first @<key>=<digits> in `line` and return its integer value.
// A leading +/- is accepted (matching humanize_directives' tolerance).
// Returns 1 on success, 0 if the key is absent or not followed by digits.
static int find_directive_ms(const char *line, const char *key, long long *out)
{
    size_t klen = strlen(key);
    for (const char *p = line; *p != '\0'; ++p) {
        if (strncmp(p, key, klen) != 0) continue;
        const char *digits = p + klen;
        const char *d = digits;
        if (*d == '+' || *d == '-') ++d;
        const char *digit_start = d;
        while (isdigit((unsigned char) *d)) ++d;
        if (d == digit_start) continue;
        char numbuf[32];
        size_t n = (size_t)(d - digits);
        if (n >= sizeof numbuf) continue;
        memcpy(numbuf, digits, n);
        numbuf[n] = '\0';
        *out = strtoll(numbuf, NULL, 10);
        return 1;
    }
    return 0;
}

// Load the first satellite from `path` and prime the SGP4/SDP4 selector.
// Mirrors next_in_queue's setup (load_tle + select_ephemeris). The empty
// satellite name makes load_tle's zero-length prefix match the first TLE
// record in the file. Returns 0 on success, -1 on failure (load_tle has
// already printed the reason).
static int tle_setup(const char *path)
{
    static char empty_name[] = "";
    g_pred.tles_filename = (char *) path;
    g_pred.satellite_ephem.name = empty_name;
    g_pred.observer_ephem.position_geodetic.lat = RAO_LATITUDE  * M_PI / 180.0;
    g_pred.observer_ephem.position_geodetic.lon = RAO_LONGITUDE * M_PI / 180.0;
    g_pred.observer_ephem.position_geodetic.alt = RAO_ALTITUDE  / 1000.0;
    if (load_tle(&g_pred) != 0) {
        return -1;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&g_pred.satellite_ephem.tle);
    return 0;
}

// Sub-satellite geodetic point at a unix-millisecond instant. Longitude is
// normalised from the propagator's [0,360) east to [-180,180).
static void sat_subpoint(long long unix_ms,
                         double *lat_deg, double *lon_deg, double *alt_km)
{
    // The Unix epoch (1970-01-01T00:00:00Z) is Julian Date 2440587.5.
    double jul_utc = 2440587.5 + (double) unix_ms / 86400000.0;
    update_satellite_position(&g_pred, jul_utc);
    double lon = g_pred.satellite_ephem.longitude;
    if (lon > 180.0) lon -= 360.0;
    *lat_deg = g_pred.satellite_ephem.latitude;
    *lon_deg = lon;
    *alt_km  = g_pred.satellite_ephem.altitude_km;
}
#endif // WITH_SGP4SDP4

int main(int argc, char **argv)
{
    int local = 0;
    int dup_check = 1;
    const char *path = NULL;
    const char *tle_path = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--local-time") == 0) {
            local = 1;
        } else if (strcmp(argv[i], "--no-dup-check") == 0) {
            dup_check = 0;
        } else if (strcmp(argv[i], "--tle") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "%s: --tle requires a file path\n", argv[0]);
                return 2;
            }
            tle_path = argv[++i];
        } else if (strcmp(argv[i], "--help") == 0
                || strcmp(argv[i], "-h") == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else if (argv[i][0] == '-') {
            fprintf(stderr, "unknown option %s\n", argv[i]);
            usage(stderr, argv[0]);
            return 2;
        } else if (path == NULL) {
            path = argv[i];
        } else {
            fprintf(stderr, "only one input file supported\n");
            return 2;
        }
    }

#ifdef WITH_SGP4SDP4
    int annotate = 0;
    long long now_ms = (long long) time(NULL) * 1000;
    if (tle_path != NULL) {
        if (tle_setup(tle_path) != 0) {
            return 1;
        }
        annotate = 1;
    }
#else
    if (tle_path != NULL) {
        fprintf(stderr,
                "%s: --tle needs a build with sgp4sdp4; this binary lacks it\n",
                argv[0]);
        return 2;
    }
#endif

    FILE *in = stdin;
    if (path != NULL) {
        in = fopen(path, "r");
        if (in == NULL) {
            fprintf(stderr, "%s: open %s: %s\n", argv[0], path,
                    strerror(errno));
            return 1;
        }
    }

    int tty = isatty(fileno(stdout));
    const char *dup_red    = tty ? "\x1b[1;31m" : "";
    const char *dup_reset  = tty ? "\x1b[0m"    : "";

    dup_table_t dups = {0};
    char buf[4096];
    char out[8192];
    int lineno = 0;
    int n_dups = 0;
    while (fgets(buf, sizeof buf, in) != NULL) {
        ++lineno;
        // Pass through comments and blank lines verbatim — they're
        // documentation in a telecommand schedule, not commands.
        if (buf[0] == '#' || buf[0] == '\n' || buf[0] == '\r'
                          || buf[0] == '\0') {
            fputs(buf, stdout);
            continue;
        }

        // Duplicate check operates on the RAW line (before
        // humanizing) so two lines with identical commands AND
        // identical embedded unix times count as duplicates. EOL is
        // stripped so trailing CRLF differences don't matter.
        int first_seen = 0;
        if (dup_check) {
            char dupbuf[4096];
            snprintf(dupbuf, sizeof dupbuf, "%s", buf);
            strip_eol(dupbuf);
            if (dup_table_push(&dups, dupbuf, lineno, &first_seen) != 0) {
                fprintf(stderr, "agenda_check: out of memory\n");
                return 1;
            }
            if (first_seen) ++n_dups;
        }

        // Sub-satellite point at this command's execution time, prepended
        // to the line. Empty unless a --tle was loaded. The exec instant
        // is @tsexec=, else @tssent=, else the wall-clock time captured at
        // startup.
        char annot[96];
        annot[0] = '\0';
#ifdef WITH_SGP4SDP4
        if (annotate) {
            long long exec_ms;
            if (!find_directive_ms(buf, "@tsexec=", &exec_ms)
                && !find_directive_ms(buf, "@tssent=", &exec_ms)) {
                exec_ms = now_ms;
            }
            double lat, lon, alt;
            sat_subpoint(exec_ms, &lat, &lon, &alt);
            // Fixed-width lat/lon (-90.0 / -180.0 worst case) so the
            // prepended columns line up down the page.
            snprintf(annot, sizeof annot,
                     "lat=%5.1f lon=%6.1f alt=%.1f  ", lat, lon, alt);
        }
#endif

        if (humanize_directives(buf, local, out, sizeof out) != 0) {
            fprintf(stderr,
                "agenda_check: line %d too long to humanize; passing through\n",
                lineno);
            strip_eol(buf);
            if (first_seen) {
                fprintf(stdout, "%s%sDUP(line %d)>%s %s\n",
                        annot, dup_red, first_seen, dup_reset, buf);
            } else {
                fprintf(stdout, "%s%s\n", annot, buf);
            }
            continue;
        }
        strip_eol(out);
        if (first_seen) {
            fprintf(stdout, "%s%sDUP(line %d)>%s %s\n",
                    annot, dup_red, first_seen, dup_reset, out);
        } else {
            fprintf(stdout, "%s%s\n", annot, out);
        }
    }

    if (in != stdin) fclose(in);
    dup_table_free(&dups);

    if (n_dups > 0) {
        fprintf(stderr,
            "agenda_check: %d duplicate line%s detected (see DUP> rows)\n",
            n_dups, n_dups == 1 ? "" : "s");
        return 3;
    }
    return 0;
}
