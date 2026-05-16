/*

    Simple Satellite Operations  unit_tests/tle_csv_selftest.c

    Round-trip tests for tle_csv.c (Celestrak OMM CSV -> classic 3-line
    TLE conversion). Writes synthetic CSV / classic-TLE fixtures to
    tempfiles, invokes tle_path_resolve(), and validates the converted
    output against the known fixture values plus the structural rules
    sgp4sdp4's Good_Elements() audit checks.

    Exit status: 0 = all tests passed, non-zero = failure.

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

#include "tle_csv.h"
#include "tap.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define check(cond, what) tap_ok((cond), (what))

static int approx(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

#define CHECK_APPROX(actual, expected, tol, msg)                           \
    do {                                                                   \
        double _a = (actual);                                              \
        double _e = (expected);                                            \
        int _ok = approx(_a, _e, (tol));                                   \
        tap_ok(_ok, (msg));                                                \
        if (!_ok) tap_diag("expected %.6f +/- %.6f, got %.6f",             \
                           _e, (tol), _a);                                 \
    } while (0)

// Write `content` to a fresh tempfile in TMPDIR. Returns a heap-allocated
// path the caller must free + unlink, or NULL on failure.
static char *write_tempfile(const char *suffix, const char *content)
{
    const char *tmpdir = getenv("TMPDIR");
    if (!tmpdir || !tmpdir[0]) tmpdir = "/tmp";
    char buf[512];
    int n = snprintf(buf, sizeof buf, "%s/sso_tle_test_XXXXXX%s",
                     tmpdir, suffix ? suffix : "");
    if (n < 0 || (size_t)n >= sizeof buf) return NULL;
    int fd = mkstemps(buf, suffix ? (int)strlen(suffix) : 0);
    if (fd < 0) {
        perror("mkstemps");
        return NULL;
    }
    FILE *f = fdopen(fd, "w");
    if (!f) {
        close(fd);
        return NULL;
    }
    if (fputs(content, f) == EOF) {
        fclose(f);
        unlink(buf);
        return NULL;
    }
    fclose(f);
    char *out = strdup(buf);
    return out;
}

// TLE-line checksum: digits sum mod 10, '-' counts as 1.
static int tle_checksum_compute(const char *line68)
{
    int sum = 0;
    for (int i = 0; i < 68; ++i) {
        char c = line68[i];
        if (c >= '0' && c <= '9') sum += c - '0';
        else if (c == '-')        sum += 1;
    }
    return sum % 10;
}

// ---------------------------------------------------------- passthrough

// Contract: NULL / empty input is returned unchanged so callers can
// thread a "no TLE configured" sentinel without crashing.
static void test_null_and_empty_passthrough(void)
{
    fprintf(stderr, "tle_path_resolve (NULL / empty input):\n");
    check(tle_path_resolve(NULL) == NULL, "NULL -> NULL");
    const char *empty = "";
    check(tle_path_resolve(empty) == empty, "\"\" -> same pointer");
}

// Contract: a path that's already a classic 3-line TLE (or anything not
// starting with "OBJECT_NAME") is returned unchanged.
static void test_classic_tle_passthrough(void)
{
    fprintf(stderr, "tle_path_resolve (classic 3-line TLE):\n");
    const char *content =
        "ISS (ZARYA)\n"
        "1 25544U 98067A   25043.50000000  .00010000  00000+0  18000-3 0  9991\n"
        "2 25544  51.6432 123.4567 0001234 234.5678 345.6789 15.50312345 12345\n";
    char *path = write_tempfile(".tle", content);
    check(path != NULL, "tempfile created");
    if (!path) return;

    char *resolved = tle_path_resolve(path);
    check(resolved == path,
          "classic TLE: resolve returns the input pointer unchanged");

    unlink(path);
    free(path);
}

// Contract: a nonexistent path can't be peeked, looks_like_csv returns
// 0, and the function falls through to passthrough.
static void test_nonexistent_path(void)
{
    fprintf(stderr, "tle_path_resolve (nonexistent path):\n");
    const char *bogus = "/tmp/sso_definitely_not_a_real_tle_file_xyzzy.tle";
    char *resolved = tle_path_resolve(bogus);
    check(resolved == bogus, "missing file: passthrough");
}

// --------------------------------------------------------- CSV round trip

// Build a Celestrak-style OMM CSV with one well-known row. Returns a
// tempfile path the caller frees + unlinks.
static char *make_test_csv(void)
{
    // Field values picked so each formatter exercises a non-trivial code
    // path (negative drag, fractional epoch, mid-range inclination).
    const char *content =
        "OBJECT_NAME,OBJECT_ID,EPOCH,MEAN_MOTION,ECCENTRICITY,INCLINATION,"
        "RA_OF_ASC_NODE,ARG_OF_PERICENTER,MEAN_ANOMALY,EPHEMERIS_TYPE,"
        "CLASSIFICATION_TYPE,NORAD_CAT_ID,ELEMENT_SET_NO,REV_AT_EPOCH,"
        "BSTAR,MEAN_MOTION_DOT,MEAN_MOTION_DDOT\n"
        "TEST SAT,2024-001A,2024-01-15T12:00:00.000000,15.50312345,"
        "0.0001234,51.6432,123.4567,234.5678,345.6789,0,U,"
        "99999,123,1234,0.00001234,-.00000123,0.0\n";
    return write_tempfile(".csv", content);
}

static void test_csv_conversion(void)
{
    fprintf(stderr, "tle_path_resolve (CSV -> 3-line TLE):\n");
    char *csv = make_test_csv();
    check(csv != NULL, "test CSV written");
    if (!csv) return;

    char *resolved = tle_path_resolve(csv);
    check(resolved != csv, "CSV input: resolve returns a different path");
    if (resolved == csv) {
        free(csv);
        return;
    }

    // Read the resulting 3-line TLE.
    FILE *f = fopen(resolved, "r");
    check(f != NULL, "converted TLE opens for read");
    if (!f) {
        unlink(csv);
        free(csv);
        return;
    }

    char name[64] = {0};
    char line1[80] = {0};
    char line2[80] = {0};
    int got = (fgets(name,  sizeof name,  f) != NULL)
            + (fgets(line1, sizeof line1, f) != NULL)
            + (fgets(line2, sizeof line2, f) != NULL);
    fclose(f);
    check(got == 3, "converted file has 3 lines (name + L1 + L2)");

    // Strip newlines.
    for (size_t i = strlen(name); i > 0 && (name[i-1] == '\n' || name[i-1] == '\r'); ) name[--i] = '\0';
    for (size_t i = strlen(line1); i > 0 && (line1[i-1] == '\n' || line1[i-1] == '\r'); ) line1[--i] = '\0';
    for (size_t i = strlen(line2); i > 0 && (line2[i-1] == '\n' || line2[i-1] == '\r'); ) line2[--i] = '\0';

    // Name is left-justified, 24 chars wide.
    check(strncmp(name, "TEST SAT", 8) == 0,
          "name line begins with 'TEST SAT'");
    check(strlen(name) == 24,
          "name padded/truncated to 24 chars");

    // Structural Good_Elements-style audit.
    check(line1[0] == '1', "line1[0] = '1'");
    check(line2[0] == '2', "line2[0] = '2'");
    check(strlen(line1) == 69, "line1 is 69 chars");
    check(strlen(line2) == 69, "line2 is 69 chars");
    check(line1[7]  == 'U',   "line1[7] classification = 'U'");
    check(line1[23] == '.',   "line1[23] = '.' (epoch decimal)");
    check(line1[34] == '.',   "line1[34] = '.' (mdot decimal)");
    check(line2[11] == '.',   "line2[11] = '.' (inclination decimal)");
    check(line2[20] == '.',   "line2[20] = '.' (RAAN decimal)");
    check(line2[37] == '.',   "line2[37] = '.' (omega decimal)");
    check(line2[46] == '.',   "line2[46] = '.' (mean anomaly decimal)");
    check(line2[54] == '.',   "line2[54] = '.' (mean motion decimal)");
    check(strncmp(&line1[61], " 0 ", 3) == 0,
          "line1[61..63] = ' 0 ' (ephemeris type, Good_Elements gate)");
    check(strncmp(&line1[2], &line2[2], 5) == 0,
          "NORAD ID matches between line1 and line2");

    // NORAD field decodes to 99999.
    check(strncmp(&line1[2], "99999", 5) == 0,
          "NORAD = 99999 (from CSV NORAD_CAT_ID)");

    // Epoch: 2024 day 015, fraction 0.5 (12:00:00) -> "24015.50000000".
    check(strncmp(&line1[18], "24015.50000000", 14) == 0,
          "epoch encodes as '24015.50000000'");

    // International designator: 2024-001A -> "24001A  ".
    check(strncmp(&line1[9], "24001A  ", 8) == 0,
          "intl designator '24001A  '");

    // Inclination = 51.6432 -> " 51.6432".
    check(strncmp(&line2[8], " 51.6432", 8) == 0,
          "inclination ' 51.6432'");

    // Mean motion = 15.50312345 -> "15.50312345" at columns 52..62.
    check(strncmp(&line2[52], "15.50312345", 11) == 0,
          "mean motion '15.50312345'");

    // Eccentricity = 0.0001234 -> '0001234' (7 chars, implied "0.").
    check(strncmp(&line2[26], "0001234", 7) == 0,
          "eccentricity '0001234'");

    // Mean motion dot = -.00000123 -> '-.00000123' (10 chars at col 33).
    check(strncmp(&line1[33], "-.00000123", 10) == 0,
          "mean motion dot '-.00000123'");

    // Checksums (digits + '-' as 1, mod 10) at column 68.
    int cs1 = tle_checksum_compute(line1);
    int cs2 = tle_checksum_compute(line2);
    check(line1[68] - '0' == cs1, "line1 checksum digit matches recomputed value");
    check(line2[68] - '0' == cs2, "line2 checksum digit matches recomputed value");

    unlink(csv);
    free(csv);
}

// Contract: resolving the same CSV twice yields the same converted
// tempfile path (cached, no redundant conversion).
static void test_csv_caching(void)
{
    fprintf(stderr, "tle_path_resolve (cache):\n");
    char *csv = make_test_csv();
    if (!csv) {
        check(0, "test CSV written");
        return;
    }
    char *first  = tle_path_resolve(csv);
    char *second = tle_path_resolve(csv);
    check(first == second,
          "two calls with identical input return the same cached pointer");
    check(strcmp(first, csv) != 0,
          "cached path is the converted tempfile, not the CSV input");
    unlink(csv);
    free(csv);
}

// Contract: a CSV missing required columns (here: no INCLINATION) is
// detected at header parse and the function falls back to passthrough.
static void test_csv_missing_required_column(void)
{
    fprintf(stderr, "tle_path_resolve (CSV missing required column):\n");
    // Header lacks INCLINATION but otherwise looks like Celestrak.
    const char *content =
        "OBJECT_NAME,OBJECT_ID,EPOCH,MEAN_MOTION,ECCENTRICITY,"
        "RA_OF_ASC_NODE,ARG_OF_PERICENTER,MEAN_ANOMALY,EPHEMERIS_TYPE,"
        "CLASSIFICATION_TYPE,NORAD_CAT_ID,ELEMENT_SET_NO,REV_AT_EPOCH,"
        "BSTAR,MEAN_MOTION_DOT,MEAN_MOTION_DDOT\n"
        "BAD SAT,2024-001A,2024-01-15T12:00:00.000000,15.5,0.0001,"
        "123.4,234.5,345.6,0,U,99999,123,1234,0.00001,0.0,0.0\n";
    char *csv = write_tempfile(".csv", content);
    check(csv != NULL, "malformed CSV written");
    if (!csv) return;
    char *resolved = tle_path_resolve(csv);
    check(resolved == csv, "missing-column CSV: passthrough to the raw path");
    unlink(csv);
    free(csv);
}

int main(void)
{
    test_null_and_empty_passthrough();
    test_classic_tle_passthrough();
    test_nonexistent_path();
    test_csv_conversion();
    test_csv_caching();
    test_csv_missing_required_column();
    return tap_done();
}
