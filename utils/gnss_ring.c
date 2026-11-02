/*

    Simple Satellite Operations  utils/gnss_ring.c

    List the GNSS fixes that FrontierSat's ring blob (gnss_bestxyzb_ring_v2)
    sends down, and compare each with SGP4 from the TLE in force.

    The blob samples the receiver every ~10 s while it has it powered, keeps
    the latest 500 samples in a ring of LittleFS files, and on every run
    downlinks a random run of them as packet type 0x30: one type byte, a
    16-bit downlink sequence number, a 16-bit ring position, then the
    receiver's binary BESTXYZB log with its NovAtel CRC. The packet DB files
    them as "gnss_ring" (rows stored before the decoder knew the type stay
    "unknown" until packet_refile runs), and gnss_reports / gnss_opm (which
    read telecommand responses and firehose files) never see them. This tool
    finds them by their first byte, whatever their filing, checks the CRC,
    keeps one copy of each fix (the ring sends the same sample many times,
    often to several stations), and prints one line per fix in time order.

    The TLE comparison uses the DB's tle table: for each fix, the newest TLE
    of the catalog number whose epoch is at or before the fix (the earliest
    one when none is). The difference is split into radial, along-track and
    cross-track in the inertial frame of the SGP4 state.

    Examples:
      gnss_ring                          # every computed fix in the DB
      gnss_ring --since=24h              # fixes taken in the last day
      gnss_ring --since=2026-10-13 --all # include samples with no solution
      gnss_ring --no-tle > fixes.txt     # skip the SGP4 comparison

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
#include "bestxyz.h"
#include "gnss_frag.h"
#include "packet_db.h"
#include "sso_version.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef WITH_SQLITE3
int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "gnss_ring")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "gnss_ring: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>
#include "prediction.h"

// FrontierSat's NORAD catalog number, the default TLE to compare against.
#define FRONTIERSAT_CATALOG 69015

// The ring blob's downlink packet: type byte, sequence number, ring position.
#define RING_PACKET_TYPE 0x30
#define RING_HDR_LEN     5
#define RING_SAMPLE_LEN  144

#define TLE_LINE_CHARS   69
#define TLE_TWO_LINE_BUF (2 * TLE_LINE_CHARS + 1)

typedef struct {
    const char *db_path;
    const char *since;
    const char *until;
    long long   catalog;
    int         all;      // also print samples without a computed solution
    int         no_tle;
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
            if (help) parse_help_line(OPTW, "--since=<spec>", "fixes taken since: 24h | 7d | 30m | ISO-8601 (default: all)");
            else a->since = arg + 8;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--until=") || help) {
            if (help) parse_help_line(OPTW, "--until=<spec>", "same syntax as --since (default: now)");
            else a->until = arg + 8;
            matched = 1;
        }
        if (strcmp(arg, "--all") == 0 || help) {
            if (help) parse_help_line(OPTW, "--all", "also list samples without a computed solution");
            else a->all = 1;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--catalog=") || help) {
            if (help) parse_help_line(OPTW, "--catalog=<n>", "TLE catalog number to compare with (default 69015)");
            else a->catalog = atoll(arg + 10);
            matched = 1;
        }
        if (strcmp(arg, "--no-tle") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-tle", "skip the comparison with SGP4");
            else a->no_tle = 1;
            matched = 1;
        }
        if ((strcmp(arg, "-V") == 0 || strcmp(arg, "--version") == 0) || help) {
            if (help) parse_help_line(OPTW, "-V, --version", "print version and exit");
            // -V is handled in main via sso_version_handle before parsing.
            matched = 1;
        }
        if (!help && !matched) {
            fprintf(stderr, "gnss_ring: unknown option '%s' (try --help)\n", arg);
            return PARSE_ERROR;
        }
    }
    return help ? PARSE_HELP : PARSE_OK;
}

// ---- fixes -------------------------------------------------------------------

typedef struct {
    double    unix_utc;   // fix time, UTC seconds
    bestxyz_t b;
} fix_t;

static fix_t *g_fixes = NULL;
static int    g_nfix = 0, g_capfix = 0;

static int cmp_fix(const void *pa, const void *pb)
{
    const fix_t *a = pa, *b = pb;
    if (a->unix_utc != b->unix_utc) return a->unix_utc < b->unix_utc ? -1 : 1;
    return 0;
}

static double fix_unix_utc(const bestxyz_t *b)
{
    return 315964800.0 + b->gps_week * 604800.0 + b->gps_sow
           - BESTXYZ_DEFAULT_LEAP_SECONDS;
}

static void fmt_utc(double unix_utc, char *out, size_t outn)
{
    time_t s = (time_t) floor(unix_utc);
    int ms = (int) lround((unix_utc - (double) s) * 1000.0);
    if (ms >= 1000) { s += 1; ms -= 1000; }
    struct tm utc;
    gmtime_r(&s, &utc);
    char base[32];
    strftime(base, sizeof base, "%Y-%m-%dT%H:%M:%S", &utc);
    snprintf(out, outn, "%s.%03dZ", base, ms);
}

// ---- TLEs --------------------------------------------------------------------

typedef struct {
    double unix_epoch;
    char   line1[80];
    char   line2[80];
} tle_row_t;

static tle_row_t *g_tles = NULL;
static int        g_ntles = 0;

static int cmp_tle(const void *pa, const void *pb)
{
    const tle_row_t *a = pa, *b = pb;
    if (a->unix_epoch != b->unix_epoch) return a->unix_epoch < b->unix_epoch ? -1 : 1;
    return 0;
}

static void load_tles(sqlite3 *db, long long catalog)
{
    sqlite3_stmt *st = NULL;
    const char *sql = "SELECT epoch_year, epoch_day, line1, line2 FROM tle "
                      "WHERE catalog_number = ?1 AND epoch_year IS NOT NULL "
                      "AND epoch_day IS NOT NULL";
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) return;
    sqlite3_bind_int64(st, 1, catalog);
    int cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        if (g_ntles == cap) {
            int nc = cap ? cap * 2 : 64;
            tle_row_t *t = realloc(g_tles, (size_t) nc * sizeof *t);
            if (t == NULL) break;
            g_tles = t;
            cap = nc;
        }
        // Epoch day 1.0 is 00:00 UTC on January 1 of the epoch year.
        struct tm jan1 = {0};
        jan1.tm_year = sqlite3_column_int(st, 0) - 1900;
        jan1.tm_mday = 1;
        tle_row_t *r = &g_tles[g_ntles++];
        r->unix_epoch = (double) timegm(&jan1)
                        + (sqlite3_column_double(st, 1) - 1.0) * 86400.0;
        snprintf(r->line1, sizeof r->line1, "%s", (const char *) sqlite3_column_text(st, 2));
        snprintf(r->line2, sizeof r->line2, "%s", (const char *) sqlite3_column_text(st, 3));
    }
    sqlite3_finalize(st);
    qsort(g_tles, (size_t) g_ntles, sizeof *g_tles, cmp_tle);
}

// The newest TLE at or before t, else the earliest; -1 when there are none.
static int tle_in_force(double t)
{
    if (g_ntles == 0) return -1;
    int k = 0;
    while (k + 1 < g_ntles && g_tles[k + 1].unix_epoch <= t) k++;
    return k;
}

// SGP4 inertial (TEME) position and velocity, km and km/s. The elements are
// converted afresh and the module flags reset on every call, because
// select_ephemeris rewrites the units in place and SGP4 caches per-object
// coefficients behind module-level flags. Returns 0, or -1 on bad elements.
static int tle_state(const tle_row_t *row, double unix_utc, double r[3], double v[3])
{
    char tle[TLE_TWO_LINE_BUF] = {0};
    size_t l1 = strlen(row->line1), l2 = strlen(row->line2);
    if (l1 > TLE_LINE_CHARS) l1 = TLE_LINE_CHARS;
    if (l2 > TLE_LINE_CHARS) l2 = TLE_LINE_CHARS;
    memcpy(tle, row->line1, l1);
    memcpy(tle + TLE_LINE_CHARS, row->line2, l2);
    if (!Good_Elements(tle)) return -1;

    prediction_t pred = {0};
    pred.observer_ephem.position_geodetic.lat = RAO_LATITUDE * M_PI / 180.0;
    pred.observer_ephem.position_geodetic.lon = RAO_LONGITUDE * M_PI / 180.0;
    pred.observer_ephem.position_geodetic.alt = RAO_ALTITUDE / 1000.0;
    Convert_Satellite_Data(tle, &pred.satellite_ephem.tle);
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&pred.satellite_ephem.tle);
    update_satellite_position(&pred, julian_date_from_unix_seconds(unix_utc));

    const vector_t *p = &pred.satellite_ephem.position, *u = &pred.satellite_ephem.velocity;
    r[0] = p->x; r[1] = p->y; r[2] = p->z;
    v[0] = u->x; v[1] = u->y; v[2] = u->z;
    return 0;
}

// GNSS minus SGP4, radial / along-track / cross-track in metres. The fix's
// Earth-fixed position is turned into the inertial frame by the Greenwich
// sidereal angle (the inverse of attitude_eci_to_earth_fixed; polar motion,
// a few metres, is ignored).
static void rtn_difference(const double ecef_m[3], double unix_utc,
                           const double r_km[3], const double v_km[3], double out[3])
{
    double th = ThetaG_JD(julian_date_from_unix_seconds(unix_utc));
    double c = cos(th), s = sin(th);
    double g[3] = {c * ecef_m[0] - s * ecef_m[1], s * ecef_m[0] + c * ecef_m[1], ecef_m[2]};
    double d[3] = {g[0] - 1e3 * r_km[0], g[1] - 1e3 * r_km[1], g[2] - 1e3 * r_km[2]};

    double rn = sqrt(r_km[0] * r_km[0] + r_km[1] * r_km[1] + r_km[2] * r_km[2]);
    double R[3] = {r_km[0] / rn, r_km[1] / rn, r_km[2] / rn};
    double h[3] = {r_km[1] * v_km[2] - r_km[2] * v_km[1],
                   r_km[2] * v_km[0] - r_km[0] * v_km[2],
                   r_km[0] * v_km[1] - r_km[1] * v_km[0]};
    double hn = sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
    double N[3] = {h[0] / hn, h[1] / hn, h[2] / hn};
    double T[3] = {N[1] * R[2] - N[2] * R[1], N[2] * R[0] - N[0] * R[2], N[0] * R[1] - N[1] * R[0]};
    out[0] = d[0] * R[0] + d[1] * R[1] + d[2] * R[2];
    out[1] = d[0] * T[0] + d[1] * T[1] + d[2] * T[2];
    out[2] = d[0] * N[0] + d[1] * N[1] + d[2] * N[2];
}

static int cmp_double(const void *pa, const void *pb)
{
    double a = *(const double *) pa, b = *(const double *) pb;
    return (a > b) - (a < b);
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "gnss_ring")) return 0;

    args_t cfg = {0};
    cfg.catalog = FRONTIERSAT_CATALOG;
    switch (parse_args(&cfg, argc, argv, 0)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
        default: break;
    }

    char db_default[1024];
    const char *db_path = cfg.db_path;
    if (db_path == NULL) {
        if (packet_db_default_path(db_default, sizeof db_default) != 0) {
            fprintf(stderr, "gnss_ring: no DB path "
                            "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = db_default;
    }

    char since_iso[40] = {0}, until_iso[40] = {0};
    if (cfg.since && gnss_parse_time_spec(cfg.since, since_iso, sizeof since_iso) != 0) {
        fprintf(stderr, "gnss_ring: bad --since=%s\n", cfg.since); return 1;
    }
    if (cfg.until && gnss_parse_time_spec(cfg.until, until_iso, sizeof until_iso) != 0) {
        fprintf(stderr, "gnss_ring: bad --until=%s\n", cfg.until); return 1;
    }

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "gnss_ring: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 1;
    }

    // Ring samples by their packet-type byte, whatever type the DB gave them.
    const char *sql = "SELECT payload FROM packet "
                      "WHERE length(payload) >= ?1 AND substr(payload, 1, 1) = x'30'";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "gnss_ring: query failed: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
        return 1;
    }
    sqlite3_bind_int(st, 1, RING_HDR_LEN + RING_SAMPLE_LEN);

    int nread = 0, nbad = 0, nother = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const unsigned char *pl = sqlite3_column_blob(st, 0);
        int n = sqlite3_column_bytes(st, 0);
        if (pl == NULL || pl[0] != RING_PACKET_TYPE) continue;
        bestxyz_t b;
        char err[96];
        // Other packets can start with 0x30 too (a bulk-file chunk whose
        // type byte took a bit flip), so only a BESTXYZB frame counts.
        if (bestxyzb_parse(pl + RING_HDR_LEN, (size_t) (n - RING_HDR_LEN), &b, err, sizeof err) != 0) {
            nother++;
            continue;
        }
        nread++;
        if (!b.crc_ok) {
            nbad++;
            continue;
        }
        if (g_nfix == g_capfix) {
            int nc = g_capfix ? g_capfix * 2 : 1024;
            fix_t *t = realloc(g_fixes, (size_t) nc * sizeof *t);
            if (t == NULL) break;
            g_fixes = t;
            g_capfix = nc;
        }
        g_fixes[g_nfix].unix_utc = fix_unix_utc(&b);
        g_fixes[g_nfix].b = b;
        g_nfix++;
    }
    sqlite3_finalize(st);

    if (!cfg.no_tle) load_tles(db, cfg.catalog);
    sqlite3_close(db);

    qsort(g_fixes, (size_t) g_nfix, sizeof *g_fixes, cmp_fix);

    printf("# utc gps_week gps_sow time_status sol_status pos_type x_m y_m z_m"
           " sx_m sy_m sz_m vx_m_s vy_m_s vz_m_s sv_tracked sv_used"
           " tle_age_h dR_m dT_m dN_m\n");

    int ndistinct = 0, nprinted = 0;
    double *dist = malloc((size_t) (g_nfix > 0 ? g_nfix : 1) * sizeof *dist);
    int ndist = 0;
    for (int i = 0; i < g_nfix; ++i) {
        // One line per fix: the ring sends each sample many times.
        if (i > 0 && g_fixes[i].unix_utc == g_fixes[i - 1].unix_utc) continue;
        ndistinct++;

        const bestxyz_t *b = &g_fixes[i].b;
        int computed = (strcmp(b->pos_sol_status, "SOL_COMPUTED") == 0);
        if (!computed && !cfg.all) continue;

        char ts[40];
        fmt_utc(g_fixes[i].unix_utc, ts, sizeof ts);
        if (since_iso[0] && strcmp(ts, since_iso) < 0) continue;
        if (until_iso[0] && strcmp(ts, until_iso) > 0) continue;

        double age_h = NAN, rtn[3] = {NAN, NAN, NAN};
        int k = computed && !cfg.no_tle ? tle_in_force(g_fixes[i].unix_utc) : -1;
        double r[3], v[3];
        if (k >= 0 && tle_state(&g_tles[k], g_fixes[i].unix_utc, r, v) == 0) {
            age_h = (g_fixes[i].unix_utc - g_tles[k].unix_epoch) / 3600.0;
            rtn_difference(b->pos, g_fixes[i].unix_utc, r, v, rtn);
            if (dist) dist[ndist++] = sqrt(rtn[0] * rtn[0] + rtn[1] * rtn[1] + rtn[2] * rtn[2]);
        }

        printf("%s %d %.3f %s %s %s %.3f %.3f %.3f %.3f %.3f %.3f %.4f %.4f %.4f %d %d"
               " %.2f %.1f %.1f %.1f\n",
               ts, b->gps_week, b->gps_sow, b->time_status, b->pos_sol_status, b->pos_type,
               b->pos[0], b->pos[1], b->pos[2],
               b->pos_sigma[0], b->pos_sigma[1], b->pos_sigma[2],
               b->vel[0], b->vel[1], b->vel[2], b->num_sv, b->num_sol_sv,
               age_h, rtn[0], rtn[1], rtn[2]);
        nprinted++;
    }

    fprintf(stderr, "gnss_ring: %d ring samples (%d failed the CRC), %d distinct fixes, %d listed\n"
                    "gnss_ring: %d more packets start with 0x30 but hold no readable BESTXYZB log\n",
            nread, nbad, ndistinct, nprinted, nother);
    if (!cfg.no_tle && ndist > 0) {
        qsort(dist, (size_t) ndist, sizeof *dist, cmp_double);
        fprintf(stderr, "gnss_ring: distance from SGP4 (catalog %lld): median %.0f m, 95th percentile %.0f m\n",
                cfg.catalog, dist[ndist / 2], dist[(int) (0.95 * (ndist - 1))]);
    } else if (!cfg.no_tle && nprinted > 0 && g_ntles == 0) {
        fprintf(stderr, "gnss_ring: no TLE for catalog %lld in the DB, so no SGP4 comparison\n",
                cfg.catalog);
    }
    free(dist);
    free(g_fixes);
    free(g_tles);
    return 0;
}

#endif
