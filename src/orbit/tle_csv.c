/*

    Simple Satellite Operations  tle_csv.c

    Copyright (C) 2025, 2026  Johnathan K Burchill

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

#define _GNU_SOURCE
#include "tle_csv.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define TLE_CACHE_MAX 8

typedef struct {
    char raw[1024];
    char tmp[1024];
} entry_t;

static entry_t g_cache[TLE_CACHE_MAX];
static int     g_cache_n = 0;
static int     g_atexit_registered = 0;

static void unlink_all_tmp(void)
{
    for (int i = 0; i < g_cache_n; i++) {
        if (g_cache[i].tmp[0]) (void) unlink(g_cache[i].tmp);
    }
}

// Peek the file's first non-blank line. Returns 1 if it starts with
// "OBJECT_NAME" (case-insensitive), 0 otherwise (including unreadable).
static int looks_like_csv(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) return 0;
    char line[256];
    int verdict = 0;
    while (fgets(line, sizeof line, f) != NULL) {
        const char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r') continue;
        verdict = (strncasecmp(p, "OBJECT_NAME", 11) == 0);
        break;
    }
    fclose(f);
    return verdict;
}

// Read the next CSV field at *p, advancing *p past the trailing comma.
// Returns 1 on success, 0 at end-of-line. Strips RFC 4180 quoting.
static int csv_next(const char **p, char *out, size_t cap)
{
    if (cap == 0) return 0;
    out[0] = '\0';
    if (*p == NULL || **p == '\0' || **p == '\n' || **p == '\r') return 0;

    const char *s = *p;
    size_t n = 0;
    if (*s == '"') {
        s++;
        while (*s != '\0') {
            if (*s == '"') {
                if (s[1] == '"') {
                    if (n + 1 < cap) out[n++] = '"';
                    s += 2;
                } else {
                    s++;
                    break;
                }
            } else {
                if (n + 1 < cap) out[n++] = *s;
                s++;
            }
        }
    } else {
        while (*s != '\0' && *s != ',' && *s != '\n' && *s != '\r') {
            if (n + 1 < cap) out[n++] = *s;
            s++;
        }
    }
    while (n > 0 && (out[n-1] == ' ' || out[n-1] == '\t')) n--;
    out[n] = '\0';
    if (*s == ',') s++;
    *p = s;
    return 1;
}

typedef enum {
    F_OBJECT_NAME = 0,
    F_OBJECT_ID,
    F_EPOCH,
    F_MEAN_MOTION,
    F_ECCENTRICITY,
    F_INCLINATION,
    F_RA_OF_ASC_NODE,
    F_ARG_OF_PERICENTER,
    F_MEAN_ANOMALY,
    F_EPHEMERIS_TYPE,
    F_CLASSIFICATION_TYPE,
    F_NORAD_CAT_ID,
    F_ELEMENT_SET_NO,
    F_REV_AT_EPOCH,
    F_BSTAR,
    F_MEAN_MOTION_DOT,
    F_MEAN_MOTION_DDOT,
    F__COUNT
} field_id_t;

static const char *FIELD_NAMES[F__COUNT] = {
    "OBJECT_NAME", "OBJECT_ID", "EPOCH", "MEAN_MOTION",
    "ECCENTRICITY", "INCLINATION", "RA_OF_ASC_NODE", "ARG_OF_PERICENTER",
    "MEAN_ANOMALY", "EPHEMERIS_TYPE", "CLASSIFICATION_TYPE",
    "NORAD_CAT_ID", "ELEMENT_SET_NO", "REV_AT_EPOCH",
    "BSTAR", "MEAN_MOTION_DOT", "MEAN_MOTION_DDOT",
};

// Build a header → field-id map. Returns 0 on success, -1 if any of the
// 17 OMM columns we need is missing.
static int build_header_map(const char *header, int *map_out, int *n_cols)
{
    for (int i = 0; i < F__COUNT; i++) map_out[i] = -1;
    int col = 0;
    const char *p = header;
    char buf[64];
    while (csv_next(&p, buf, sizeof buf)) {
        for (int i = 0; i < F__COUNT; i++) {
            if (strcasecmp(buf, FIELD_NAMES[i]) == 0) {
                map_out[i] = col;
                break;
            }
        }
        col++;
    }
    *n_cols = col;
    for (int i = 0; i < F__COUNT; i++) {
        if (map_out[i] < 0) {
            fprintf(stderr, "tle_csv: header missing column '%s'\n",
                    FIELD_NAMES[i]);
            return -1;
        }
    }
    return 0;
}

// "1998-067A" → "98067A  ", 8 chars (NUL-terminated to fit out_cap >= 9).
static void format_idesg(const char *obj_id, char *out)
{
    int yy = 0, launch = 0;
    char piece[4] = "   ";
    int n = sscanf(obj_id, "%d-%d%3[A-Za-z]", &yy, &launch, piece);
    if (n >= 2) {
        yy %= 100;
        snprintf(out, 9, "%02d%03d%-3s", yy, launch, piece);
    } else {
        snprintf(out, 9, "%-8s", "");
    }
}

// ISO 8601 → 14-char "YYDDD.DDDDDDDD". Returns 0 on success.
static int format_epoch(const char *iso, char *out14)
{
    int year, mon, day, hh, mm, ss;
    char frac_buf[16] = {0};
    int n = sscanf(iso, "%d-%d-%dT%d:%d:%d.%15[0-9]",
                   &year, &mon, &day, &hh, &mm, &ss, frac_buf);
    if (n < 6) {
        if (sscanf(iso, "%d-%d-%dT%d:%d:%d",
                   &year, &mon, &day, &hh, &mm, &ss) != 6) {
            return -1;
        }
    }
    struct tm tm = {0};
    tm.tm_year = year - 1900;
    tm.tm_mon  = mon - 1;
    tm.tm_mday = day;
    if (timegm(&tm) == (time_t) -1) return -1;
    int doy = tm.tm_yday + 1;
    double sec_frac = 0.0;
    if (frac_buf[0]) {
        double f = atof(frac_buf);
        double scale = pow(10.0, (double) strlen(frac_buf));
        if (scale > 0) sec_frac = f / scale;
    }
    double frac_day = ((double) hh * 3600.0
                     + (double) mm * 60.0
                     + (double) ss
                     + sec_frac) / 86400.0;
    double full = (double) doy + frac_day;
    int yy = year % 100;
    char buf[16];
    int w = snprintf(buf, sizeof buf, "%02d%012.8f", yy, full);
    if (w != 14) return -1;
    memcpy(out14, buf, 14);
    return 0;
}

// 10-char signed mantissa with implied "0." prefix: " .NNNNNNNN".
static void format_mdot(double v, char *out10)
{
    char sign = (v < 0) ? '-' : ' ';
    double a = fabs(v);
    long frac = (long) llround(a * 1e8);
    if (frac < 0) frac = 0;
    if (frac > 99999999) frac = 99999999;
    char buf[16];
    snprintf(buf, sizeof buf, "%c.%08ld", sign, frac);
    memcpy(out10, buf, 10);
}

// 8-char decimal-with-exponent in implied ".N" form: " NNNNN-N" / " NNNNN+N".
static void format_decexp(double v, char *out8)
{
    if (v == 0.0 || !isfinite(v)) {
        memcpy(out8, " 00000+0", 8);
        return;
    }
    char sign = (v < 0) ? '-' : ' ';
    double a = fabs(v);
    int exp = 0;
    // The |exp| > 20 guards are infinite-loop backstops for absurd inputs;
    // they can leave `a` un-normalised, but that never reaches the output: any
    // value that hits them lands outside [-9, 9] and the exp-range clamps below
    // override the mantissa (saturate to 99999+9, or emit zero). Real BSTAR /
    // mean-motion-derivative values normalise in a handful of steps.
    while (a < 0.1)  { a *= 10.0;  exp--; if (exp < -20) break; }
    while (a >= 1.0) { a /= 10.0;  exp++; if (exp >  20) break; }
    int mantissa = (int) lround(a * 100000.0);
    if (mantissa >= 100000) { mantissa /= 10; exp++; }
    if (exp > 9) { exp = 9; mantissa = 99999; }
    if (exp < -9) {
        memcpy(out8, " 00000+0", 8);
        out8[0] = sign;
        return;
    }
    char esign = (exp < 0) ? '-' : '+';
    int eabs = (exp < 0) ? -exp : exp;
    char buf[16];
    snprintf(buf, sizeof buf, "%c%05d%c%d", sign, mantissa, esign, eabs);
    memcpy(out8, buf, 8);
}

// 7-char no-decimal: 0.0012345 → "0012345".
static void format_ecc(double e, char *out7)
{
    long n = (long) llround(e * 1e7);
    if (n < 0) n = 0;
    if (n > 9999999) n = 9999999;
    char buf[16];
    snprintf(buf, sizeof buf, "%07ld", n);
    memcpy(out7, buf, 7);
}

static int tle_checksum(const char *line68)
{
    int sum = 0;
    for (int i = 0; i < 68; i++) {
        char c = line68[i];
        if (c >= '0' && c <= '9') sum += c - '0';
        else if (c == '-')        sum += 1;
    }
    return sum % 10;
}

// Construct the two 69-char TLE lines from the parsed CSV fields. Both
// out buffers must hold at least 70 bytes (line + NUL). Returns 0 on
// success, -1 if the epoch can't be parsed.
static int build_lines(int norad, char classification,
                       const char *object_id, const char *epoch,
                       double bstar, double mdot, double mddot,
                       int element_set_no,
                       double inclination, double raan,
                       double eccentricity, double omega,
                       double mean_anomaly, double mean_motion,
                       int rev_at_epoch,
                       char *line1, char *line2)
{
    memset(line1, ' ', 69); line1[69] = '\0';
    memset(line2, ' ', 69); line2[69] = '\0';

    // ── Line 1 ──
    line1[0] = '1';
    {
        char buf[8];
        snprintf(buf, sizeof buf, "%05d", norad % 100000);
        memcpy(&line1[2], buf, 5);
    }
    line1[7] = (classification && isalpha((unsigned char) classification))
               ? (char) toupper((unsigned char) classification) : 'U';
    {
        char idesg[12];
        format_idesg(object_id, idesg);
        memcpy(&line1[9], idesg, 8);
    }
    {
        char ep[16];
        if (format_epoch(epoch, ep) != 0) return -1;
        memcpy(&line1[18], ep, 14);
    }
    {
        char buf[16];
        format_mdot(mdot, buf);
        memcpy(&line1[33], buf, 10);
    }
    {
        char buf[16];
        format_decexp(mddot, buf);
        memcpy(&line1[44], buf, 8);
    }
    {
        char buf[16];
        format_decexp(bstar, buf);
        memcpy(&line1[53], buf, 8);
    }
    // Ephemeris type — always '0' so Good_Elements' " 0 " check passes.
    // Real-world public TLEs use 0; SGP4 itself doesn't read the field.
    line1[62] = '0';
    {
        char buf[8];
        snprintf(buf, sizeof buf, "%4d", element_set_no);
        memcpy(&line1[64], buf, 4);
    }
    line1[68] = (char) ('0' + tle_checksum(line1));

    // ── Line 2 ──
    line2[0] = '2';
    {
        char buf[8];
        snprintf(buf, sizeof buf, "%05d", norad % 100000);
        memcpy(&line2[2], buf, 5);
    }
    {
        char buf[16];
        snprintf(buf, sizeof buf, "%8.4f", inclination);
        memcpy(&line2[8], buf, 8);
    }
    {
        char buf[16];
        snprintf(buf, sizeof buf, "%8.4f", raan);
        memcpy(&line2[17], buf, 8);
    }
    {
        char buf[16];
        format_ecc(eccentricity, buf);
        memcpy(&line2[26], buf, 7);
    }
    {
        char buf[16];
        snprintf(buf, sizeof buf, "%8.4f", omega);
        memcpy(&line2[34], buf, 8);
    }
    {
        char buf[16];
        snprintf(buf, sizeof buf, "%8.4f", mean_anomaly);
        memcpy(&line2[43], buf, 8);
    }
    {
        char buf[16];
        snprintf(buf, sizeof buf, "%11.8f", mean_motion);
        memcpy(&line2[52], buf, 11);
    }
    {
        char buf[8];
        snprintf(buf, sizeof buf, "%5d", rev_at_epoch);
        memcpy(&line2[63], buf, 5);
    }
    line2[68] = (char) ('0' + tle_checksum(line2));
    return 0;
}

// Defensive cross-check that mirrors the sgp4sdp4 Good_Elements column
// audit. Catches any drift between our format strings and what the
// downstream parser expects.
static int looks_valid_tle(const char *line1, const char *line2)
{
    if (line1[0] != '1' || line2[0] != '2') return 0;
    if (line1[23] != '.' || line1[34] != '.') return 0;
    if (line2[11] != '.' || line2[20] != '.') return 0;
    if (line2[37] != '.' || line2[46] != '.') return 0;
    if (line2[54] != '.') return 0;
    if (strncmp(&line1[61], " 0 ", 3) != 0) return 0;
    if (strncmp(&line1[2], &line2[2], 5) != 0) return 0;
    return 1;
}

static int convert_file(const char *csv_path, const char *out_path)
{
    FILE *fin = fopen(csv_path, "r");
    if (fin == NULL) {
        fprintf(stderr, "tle_csv: open %s: %s\n", csv_path, strerror(errno));
        return -1;
    }
    FILE *fout = fopen(out_path, "w");
    if (fout == NULL) {
        fprintf(stderr, "tle_csv: open %s: %s\n", out_path, strerror(errno));
        fclose(fin);
        return -1;
    }

    int map[F__COUNT];
    int n_cols = 0;
    int got_header = 0;
    int rows_out = 0;
    int rows_skipped = 0;

    // Per-column scratch storage. 64 columns of 96 bytes is more than
    // Celestrak ever emits and OBJECT_NAME tops out around 24 bytes.
    enum { MAX_COLS = 64, COL_BUF = 96 };
    char col_bufs[MAX_COLS][COL_BUF];

    char line[4096];
    while (fgets(line, sizeof line, fin) != NULL) {
        size_t n = strlen(line);
        while (n > 0 && (line[n-1] == '\n' || line[n-1] == '\r')) {
            line[--n] = '\0';
        }
        const char *q = line;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == '\0') continue;

        if (!got_header) {
            if (build_header_map(line, map, &n_cols) != 0) {
                fclose(fin);
                fclose(fout);
                return -1;
            }
            got_header = 1;
            continue;
        }

        int col_count = 0;
        const char *p = line;
        while (col_count < MAX_COLS
               && csv_next(&p, col_bufs[col_count], COL_BUF)) {
            col_count++;
        }
        if (col_count < n_cols) {
            rows_skipped++;
            continue;
        }
        const char *obj_name  = col_bufs[map[F_OBJECT_NAME]];
        const char *obj_id    = col_bufs[map[F_OBJECT_ID]];
        const char *epoch_str = col_bufs[map[F_EPOCH]];
        const char *cls_str   = col_bufs[map[F_CLASSIFICATION_TYPE]];

        double mean_motion    = atof(col_bufs[map[F_MEAN_MOTION]]);
        double eccentricity   = atof(col_bufs[map[F_ECCENTRICITY]]);
        double inclination    = atof(col_bufs[map[F_INCLINATION]]);
        double raan           = atof(col_bufs[map[F_RA_OF_ASC_NODE]]);
        double omega          = atof(col_bufs[map[F_ARG_OF_PERICENTER]]);
        double mean_anomaly   = atof(col_bufs[map[F_MEAN_ANOMALY]]);
        double bstar          = atof(col_bufs[map[F_BSTAR]]);
        double mdot           = atof(col_bufs[map[F_MEAN_MOTION_DOT]]);
        double mddot          = atof(col_bufs[map[F_MEAN_MOTION_DDOT]]);
        int    norad          = atoi(col_bufs[map[F_NORAD_CAT_ID]]);
        int    elset          = atoi(col_bufs[map[F_ELEMENT_SET_NO]]);
        int    rev_at_epoch   = atoi(col_bufs[map[F_REV_AT_EPOCH]]);
        char   classification = cls_str[0] ? cls_str[0] : 'U';

        char l1[80], l2[80];
        if (build_lines(norad, classification, obj_id, epoch_str,
                        bstar, mdot, mddot, elset,
                        inclination, raan, eccentricity, omega,
                        mean_anomaly, mean_motion, rev_at_epoch,
                        l1, l2) != 0
            || !looks_valid_tle(l1, l2)) {
            rows_skipped++;
            continue;
        }
        // 24-char left-justified name to match the classic convention
        // (see TLEs/amateur.tle: "OSCAR 7 (AO-7)          ").
        fprintf(fout, "%-24.24s\n%s\n%s\n", obj_name, l1, l2);
        rows_out++;
    }

    fclose(fin);
    fclose(fout);

    if (rows_out == 0) {
        fprintf(stderr, "tle_csv: %s yielded 0 valid TLE rows\n", csv_path);
        return -1;
    }
    fprintf(stderr, "tle_csv: converted %d row%s from %s%s\n",
            rows_out, rows_out == 1 ? "" : "s", csv_path,
            rows_skipped ? " (some malformed rows skipped)" : "");
    return 0;
}

char *tle_path_resolve(const char *raw)
{
    if (raw == NULL || raw[0] == '\0') return (char *) raw;

    for (int i = 0; i < g_cache_n; i++) {
        if (strcmp(g_cache[i].raw, raw) == 0) {
            return g_cache[i].tmp;
        }
    }
    if (!looks_like_csv(raw)) return (char *) raw;

    if (g_cache_n >= TLE_CACHE_MAX) {
        fprintf(stderr, "tle_csv: cache full (%d entries); using raw %s\n",
                TLE_CACHE_MAX, raw);
        return (char *) raw;
    }

    const char *tmpdir = getenv("TMPDIR");
    if (tmpdir == NULL || tmpdir[0] == '\0') tmpdir = "/tmp";
    char tmp_path[1024];
    int n = snprintf(tmp_path, sizeof tmp_path,
                     "%s/sso_tle_XXXXXX.tle", tmpdir);
    if (n < 0 || (size_t) n >= sizeof tmp_path) return (char *) raw;

    int fd = mkstemps(tmp_path, 4);
    if (fd < 0) {
        fprintf(stderr, "tle_csv: mkstemps(%s): %s\n",
                tmp_path, strerror(errno));
        return (char *) raw;
    }
    close(fd);

    if (convert_file(raw, tmp_path) != 0) {
        (void) unlink(tmp_path);
        return (char *) raw;
    }

    if (!g_atexit_registered) {
        atexit(unlink_all_tmp);
        g_atexit_registered = 1;
    }
    snprintf(g_cache[g_cache_n].raw, sizeof g_cache[0].raw, "%s", raw);
    snprintf(g_cache[g_cache_n].tmp, sizeof g_cache[0].tmp, "%s", tmp_path);
    g_cache_n++;
    return g_cache[g_cache_n - 1].tmp;
}
