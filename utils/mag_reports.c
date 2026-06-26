/*

    Simple Satellite Operations  utils/mag_reports.c

    Report FrontierSat's ADCS magnetometer readings from the packet DB and
    tie each one to where the satellite was when it was taken.

    The magnetometer is read primarily through the generic-telemetry command
    adcs_generic_telemetry_request(151,6) -- CubeADCS frame 151, the measured
    magnetic field; frame 159 is the IGRF *model* field. Its response is a hex
    byte dump ("55 a 8 f4 6a 1"), little-endian int16 per axis, which the
    firmware scales by 10 to nanotesla. The ADCS also answers a few named
    commands with JSON: adcs_magnetic_field_vector returns the field as
    {"x_nT":..,"y_nT":..,"z_nT":..}; adcs_measurements wraps the same measured
    field (magnetic_field_*_nT) among other sensors; adcs_get_raw_magnetometer_values
    returns uncalibrated counts as {"x":..,"y":..,"z":..}. All land in the packet
    DB as tcmd_response packets (or, when the command carried @resp_fname, inside
    a bulk-downlinked adcs_data log file). This tool finds them, reassembles the
    multi-packet ones, and -- because a frame response is just bytes and the
    measured/model JSON fields share a shape -- uses the originating command
    (resolved via the @tssent each response echoes, or the log's "tcmd"/"args")
    to know the frame id and label the reading measured vs. model.

    For position, it takes the measurement time (the command's @tsexec if it
    was scheduled, else the echoed @tssent, else the ground reception time),
    finds the TLE in the DB whose epoch is closest to that time, and runs the
    same SGP4 path next_in_queue uses to get the sub-satellite lat/lon/alt.
    When a measured reading and an IGRF-model reading sit close together in
    time it reports the |B| residual -- a quick magnetometer health check.

    Read-only on the DB -- safe to run while a receiver is filling it.

    Examples:
      mag_reports                     # every magnetometer reading in the DB
      mag_reports --since=7d          # only the last week
      mag_reports --json              # one JSON object per reading (JSONL)
      mag_reports --catalog=69015     # restrict TLE association to one object

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

#include "adcs_mag.h"
#include "argparse.h"
#include "gnss_frag.h"
#include "packet_db.h"
#include "sso_version.h"
#include "tcmd_response.h"

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef WITH_SQLITE3
int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "mag_reports")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "mag_reports: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

#ifdef WITH_SGP4SDP4
#include "prediction.h"
#endif

// FrontierSat's NORAD catalog number -- the default object to associate
// readings against. The DB's tle table can hold several objects, so without a
// filter the position would be ambiguous; FrontierSat is the mission this tool
// serves. Override with --catalog=<n>, or --catalog=0 / --satellite= for others.
#define FRONTIERSAT_CATALOG 69015

// Format a unix-ms timestamp as ISO-8601 UTC with milliseconds.
static void fmt_epoch_ms(int64_t ms, char *out, size_t outn)
{
    time_t s = (time_t)(ms / 1000);
    struct tm utc;
    gmtime_r(&s, &utc);
    char base[32];
    strftime(base, sizeof base, "%Y-%m-%dT%H:%M:%S", &utc);
    snprintf(out, outn, "%s.%03uZ", base, (unsigned)(((ms % 1000) + 1000) % 1000));
}

// Format a unix-ms timestamp as a bare UTC date (for TLE epochs).
static void fmt_epoch_date(int64_t ms, char *out, size_t outn)
{
    time_t s = (time_t)(ms / 1000);
    struct tm utc;
    gmtime_r(&s, &utc);
    strftime(out, outn, "%Y-%m-%d", &utc);
}

// Parse an ISO-8601 "YYYY-MM-DDTHH:MM:SS[.mmm]Z" string (the form stored in
// packet.ts_received) to unix milliseconds. Returns 0 on success, -1 on a
// shape we don't recognize.
static int iso_to_unix_ms(const char *s, int64_t *out_ms)
{
    int y, mo, d, h, mi, se, ms = 0;
    int got = sscanf(s, "%d-%d-%dT%d:%d:%d.%d", &y, &mo, &d, &h, &mi, &se, &ms);
    if (got < 6) return -1;
    struct tm tm = {0};
    tm.tm_year = y - 1900; tm.tm_mon = mo - 1; tm.tm_mday = d;
    tm.tm_hour = h; tm.tm_min = mi; tm.tm_sec = se;
    time_t t = timegm(&tm);
    if (t == (time_t)-1) return -1;
    *out_ms = (int64_t)t * 1000 + ms;
    return 0;
}

typedef struct {
    const char *db_path;
    const char *since;
    const char *until;
    const char *satellite;   // TLE filter: tle.satellite LIKE %s% (else NULL)
    long long   catalog;     // TLE filter: tle.catalog_number == this (0 = any)
    int         catalog_explicit;  // user passed --catalog (vs the default)
    double      pair_window_s;  // measured<->IGRF pairing tolerance
    double      nt_per_count;   // raw-count -> nT first-order scale (default 1)
    int         no_bulk;     // skip the bulk_file adcs_data logs
    int         json;        // emit JSONL instead of the human report
    int         reverse;     // newest first
} args_t;

#define OPTW 26

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
        if (gnss_starts_with(arg, "--satellite=") || help) {
            if (help) parse_help_line(OPTW, "--satellite=<s>", "associate against TLEs whose name contains <s> (overrides the default catalog)");
            else a->satellite = arg + 12;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--catalog=") || help) {
            if (help) parse_help_line(OPTW, "--catalog=<n>", "NORAD catalog number for TLE association (default 69015, FrontierSat; 0 = any object)");
            else { a->catalog = atoll(arg + 10); a->catalog_explicit = 1; }
            matched = 1;
        }
        if (gnss_starts_with(arg, "--pair-window=") || help) {
            if (help) parse_help_line(OPTW, "--pair-window=<s>", "max seconds between a measured and an IGRF reading to pair them (default 120)");
            else a->pair_window_s = atof(arg + 14);
            matched = 1;
        }
        if (gnss_starts_with(arg, "--nt-per-count=") || help) {
            if (help) parse_help_line(OPTW, "--nt-per-count=<f>", "approx nT per raw count for uncalibrated readings (default 1.0; exact needs adcs_get_magnetometer_config)");
            else a->nt_per_count = atof(arg + 15);
            matched = 1;
        }
        if (strcmp(arg, "--no-bulk") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-bulk", "skip the bulk-downlinked adcs_data/*.log files; read only tcmd_response packets");
            else a->no_bulk = 1;
            matched = 1;
        }
        if (strcmp(arg, "--json") == 0 || help) {
            if (help) parse_help_line(OPTW, "--json", "emit one JSON object per reading (JSONL); diagnostics go to stderr");
            else a->json = 1;
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
            fprintf(stderr, "mag_reports: unknown option '%s' (try --help)\n", arg);
            return PARSE_ERROR;
        }
    }
    return help ? PARSE_HELP : PARSE_OK;
}

// ---- sent_tcmd cache (resolves a response's @tssent to its command) --------

typedef struct {
    int64_t ts_sent_ms;
    int64_t tsexec_ms;     // < 0 means "not scheduled / unknown"
    char    command[160];
} sent_cmd_t;

static sent_cmd_t *g_cmds = NULL;
static int         g_ncmds = 0;

static int cmp_cmd(const void *a, const void *b)
{
    int64_t x = ((const sent_cmd_t *)a)->ts_sent_ms;
    int64_t y = ((const sent_cmd_t *)b)->ts_sent_ms;
    return (x > y) - (x < y);
}

// Load every sent_tcmd row (if the table exists -- older DBs predate it).
static void load_sent_tcmds(sqlite3 *db)
{
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT ts_sent_ms, tsexec_ms, command_text FROM sent_tcmd",
            -1, &st, NULL) != SQLITE_OK)
        return;   // no such table -> command resolution simply unavailable

    int cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        if (g_ncmds == cap) {
            int nc = cap ? cap * 2 : 128;
            sent_cmd_t *t = realloc(g_cmds, (size_t)nc * sizeof *t);
            if (t == NULL) break;
            g_cmds = t; cap = nc;
        }
        sent_cmd_t *c = &g_cmds[g_ncmds];
        c->ts_sent_ms = sqlite3_column_int64(st, 0);
        c->tsexec_ms  = (sqlite3_column_type(st, 1) == SQLITE_NULL)
                        ? -1 : sqlite3_column_int64(st, 1);
        const unsigned char *cmd = sqlite3_column_text(st, 2);
        snprintf(c->command, sizeof c->command, "%s", cmd ? (const char *)cmd : "");
        g_ncmds++;
    }
    sqlite3_finalize(st);
    qsort(g_cmds, (size_t)g_ncmds, sizeof *g_cmds, cmp_cmd);
}

// Find the command sent at ts_sent_ms (the response's echoed @tssent).
// Returns NULL if unknown (manual-compose commands carry no @tssent and
// aren't recorded; pre-table DBs have none at all).
static const sent_cmd_t *find_cmd(int64_t ts_sent_ms)
{
    if (ts_sent_ms == 0 || g_ncmds == 0) return NULL;
    int lo = 0, hi = g_ncmds - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (g_cmds[mid].ts_sent_ms == ts_sent_ms) return &g_cmds[mid];
        if (g_cmds[mid].ts_sent_ms < ts_sent_ms) lo = mid + 1;
        else hi = mid - 1;
    }
    return NULL;
}

// ---- TLE table cache (the "available TLEs" we pick the closest of) ---------

typedef struct {
    long long id;
    long long catalog;
    int64_t   epoch_ms;
    char      line1[80];
    char      line2[80];
} tle_row_t;

static tle_row_t *g_tles = NULL;
static int64_t   *g_tle_epochs = NULL;   // parallel to g_tles, for closest-index
static int        g_ntles = 0;

// Load the TLE rows matching the satellite/catalog filter, computing each
// epoch as unix-ms. Returns the number of distinct catalog numbers seen
// (so the caller can refuse an ambiguous unfiltered run).
static int load_tles(sqlite3 *db, const args_t *a)
{
    char sql[256];
    snprintf(sql, sizeof sql,
             "SELECT id, catalog_number, epoch_year, epoch_day, line1, line2 "
             "FROM tle WHERE epoch_year IS NOT NULL AND epoch_day IS NOT NULL%s%s",
             a->catalog   ? " AND catalog_number=?1" : "",
             a->satellite ? " AND satellite LIKE ?2" : "");
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) return 0;
    if (a->catalog)   sqlite3_bind_int64(st, 1, a->catalog);
    if (a->satellite) {
        char like[80];
        snprintf(like, sizeof like, "%%%s%%", a->satellite);
        sqlite3_bind_text(st, 2, like, -1, SQLITE_TRANSIENT);
    }

    long long catalogs[64]; int ncat = 0;
    int cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        int year   = sqlite3_column_int(st, 2);
        double doy = sqlite3_column_double(st, 3);
        int64_t epoch_ms;
        if (adcs_tle_epoch_unix_ms(year, doy, &epoch_ms) != 0) continue;

        if (g_ntles == cap) {
            int nc = cap ? cap * 2 : 64;
            tle_row_t *tt = realloc(g_tles, (size_t)nc * sizeof *tt);
            int64_t   *et = realloc(g_tle_epochs, (size_t)nc * sizeof *et);
            if (tt) g_tles = tt;
            if (et) g_tle_epochs = et;
            if (!tt || !et) break;
            cap = nc;
        }
        tle_row_t *r = &g_tles[g_ntles];
        r->id      = sqlite3_column_int64(st, 0);
        r->catalog = sqlite3_column_int64(st, 1);
        r->epoch_ms = epoch_ms;
        snprintf(r->line1, sizeof r->line1, "%s", (const char *)sqlite3_column_text(st, 4));
        snprintf(r->line2, sizeof r->line2, "%s", (const char *)sqlite3_column_text(st, 5));
        g_tle_epochs[g_ntles] = epoch_ms;
        g_ntles++;

        int seen = 0;
        for (int i = 0; i < ncat; ++i) if (catalogs[i] == r->catalog) { seen = 1; break; }
        if (!seen && ncat < (int)(sizeof catalogs / sizeof catalogs[0]))
            catalogs[ncat++] = r->catalog;
    }
    sqlite3_finalize(st);
    return ncat;
}

#ifdef WITH_SGP4SDP4
// Mirror prediction.c's two-line buffer geometry.
#define TLE_LINE_CHARS   69
#define TLE_TWO_LINE_BUF (2 * TLE_LINE_CHARS + 1)

// Sub-satellite geodetic point for one TLE at a unix-ms instant. Each call
// reconverts the elements and resets sgp4sdp4's module-level flags before
// select_ephemeris: the flags and select_ephemeris's in-place unit rewrite
// are not per-object, so reusing one converted TLE across different element
// sets would cross-contaminate. Returns 0 on success, -1 on bad elements.
static int tle_subpoint(const char *line1, const char *line2, int64_t unix_ms,
                        double *lat_deg, double *lon_deg, double *alt_km)
{
    char tle[TLE_TWO_LINE_BUF] = {0};
    size_t l1 = strlen(line1), l2 = strlen(line2);
    if (l1 > TLE_LINE_CHARS) l1 = TLE_LINE_CHARS;
    if (l2 > TLE_LINE_CHARS) l2 = TLE_LINE_CHARS;
    memcpy(tle, line1, l1);
    memcpy(tle + TLE_LINE_CHARS, line2, l2);
    if (!Good_Elements(tle)) return -1;

    prediction_t pred = {0};
    pred.observer_ephem.position_geodetic.lat = RAO_LATITUDE  * M_PI / 180.0;
    pred.observer_ephem.position_geodetic.lon = RAO_LONGITUDE * M_PI / 180.0;
    pred.observer_ephem.position_geodetic.alt = RAO_ALTITUDE  / 1000.0;
    Convert_Satellite_Data(tle, &pred.satellite_ephem.tle);
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&pred.satellite_ephem.tle);

    double jul_utc = 2440587.5 + (double)unix_ms / 86400000.0;
    update_satellite_position(&pred, jul_utc);

    double lon = pred.satellite_ephem.longitude;
    if (lon > 180.0) lon -= 360.0;
    *lat_deg = pred.satellite_ephem.latitude;
    *lon_deg = lon;
    *alt_km  = pred.satellite_ephem.altitude_km;
    return 0;
}
#endif // WITH_SGP4SDP4

// ---- collected readings ----------------------------------------------------

typedef enum { REC_MEASURED, REC_IGRF, REC_RAW, REC_UNRESOLVED } rec_kind_t;

typedef struct {
    int64_t    meas_ms;            // chosen measurement time
    char       meas_basis[12];     // "tsexec" | "tssent" | "tsdone" | "received"
    char       ts_received[40];
    int64_t    ts_sent_ms;         // echoed @tssent (0 if none)
    rec_kind_t kind;
    char       command[160];       // command_text, or "" if unresolved
    int        field_measurements; // came from adcs_measurements' block
    int        from_bulk;          // 1 = recovered from a bulk_file adcs_data log
    int        telem_frame;        // generic-telemetry frame id (151/159), else 0
    adcs_mag_vec_t v;
    double     mag;                // |vector| in the reading's native unit (nT or counts)
    int        have_pos;
    double     lat, lon, alt_km;
    long long  tle_id;
    long long  tle_norad;
    int64_t    tle_epoch_ms;
    char       ids[160];
    int        pair;               // index of paired IGRF/measured rec, or -1
} rec_t;

// REC_RAW readings are uncalibrated counts; everything else is nT.
static int rec_is_counts(const rec_t *r) { return r->kind == REC_RAW; }

static rec_t *g_recs = NULL;
static int    g_nrecs = 0, g_caprecs = 0;

static rec_t *new_rec(void)
{
    if (g_nrecs == g_caprecs) {
        int nc = g_caprecs ? g_caprecs * 2 : 128;
        rec_t *t = realloc(g_recs, (size_t)nc * sizeof *t);
        if (t == NULL) return NULL;
        g_recs = t; g_caprecs = nc;
    }
    rec_t *r = &g_recs[g_nrecs++];
    memset(r, 0, sizeof *r);
    r->pair = -1;
    return r;
}

// Build the contributing packet-id list (clamps like gnss_reports does so a
// truncated snprintf can't push the offset past the buffer).
static void build_ids(char *ids, size_t cap, const gnss_frag_t *frags, int n)
{
    int p = 0;
    for (int i = 0; i < n && p < (int)cap - 12; ++i) {
        p += snprintf(ids + p, cap - (size_t)p, "%s%lld", i ? "," : "", frags[i].id);
        if (p > (int)cap) p = (int)cap;
    }
}

// Decide what a field-vector reading is. The command (when known) is
// authoritative -- the measured and IGRF-model fields share a JSON shape, so
// only the originating command separates them. Falls back to the JSON shape.
static rec_kind_t classify(const char *command, const adcs_mag_vec_t *v)
{
    if (command && command[0]) {
        if (strstr(command, "adcs_igrf_magnetic_field_vector"))   return REC_IGRF;
        if (strstr(command, "adcs_magnetic_field_vector")
            || strstr(command, "adcs_measurements"))              return REC_MEASURED;
        if (strstr(command, "adcs_get_raw_magnetometer_values"))  return REC_RAW;
    }
    if (v->kind == ADCS_MAG_MEASUREMENTS) return REC_MEASURED;
    if (v->kind == ADCS_MAG_RAW)          return REC_RAW;
    return REC_UNRESOLVED;   // bare {"x_nT"} with no command: measured or IGRF
}

// Record one reading and associate it with a sub-satellite position via the
// closest TLE by epoch. Deduplicates: the same reading can arrive twice (a
// retransmitted tcmd_response, or a file fetched more than once, or both a
// direct response and its copy in a bulk log).
static void emit_reading(rec_kind_t kind, const adcs_mag_vec_t *v,
                         const char *command, int64_t ts_sent_ms,
                         const char *ts_received, int64_t meas_ms,
                         const char *meas_basis, const char *ids, int from_bulk,
                         int telem_frame)
{
    for (int i = 0; i < g_nrecs; ++i) {
        rec_t *e = &g_recs[i];
        if (e->kind == kind && e->v.x_nT == v->x_nT && e->v.y_nT == v->y_nT
            && e->v.z_nT == v->z_nT && llabs(e->meas_ms - meas_ms) < 1000) {
            // Keep the direct-response copy's provenance over a bulk-log one.
            if (e->from_bulk && !from_bulk) {
                e->from_bulk = 0;
                snprintf(e->ids, sizeof e->ids, "%s", ids ? ids : "");
            }
            return;
        }
    }

    rec_t *r = new_rec();
    if (r == NULL) return;
    r->kind = kind;
    r->v = *v;
    r->mag = adcs_mag_magnitude(v);
    r->field_measurements = (v->kind == ADCS_MAG_MEASUREMENTS);
    r->from_bulk = from_bulk;
    r->telem_frame = telem_frame;
    r->ts_sent_ms = ts_sent_ms;
    snprintf(r->ts_received, sizeof r->ts_received, "%s", ts_received ? ts_received : "");
    if (command) snprintf(r->command, sizeof r->command, "%s", command);
    snprintf(r->ids, sizeof r->ids, "%s", ids ? ids : "");
    r->meas_ms = meas_ms;
    snprintf(r->meas_basis, sizeof r->meas_basis, "%s", meas_basis);

    int idx = adcs_closest_index(g_tle_epochs, g_ntles, r->meas_ms);
    if (idx >= 0) {
        r->tle_id       = g_tles[idx].id;
        r->tle_norad    = g_tles[idx].catalog;
        r->tle_epoch_ms = g_tles[idx].epoch_ms;
#ifdef WITH_SGP4SDP4
        if (tle_subpoint(g_tles[idx].line1, g_tles[idx].line2, r->meas_ms,
                         &r->lat, &r->lon, &r->alt_km) == 0)
            r->have_pos = 1;
#endif
    }
}

// Turn one reassembled tcmd_response reception into a reading (if it carries a
// field vector). since/until already filtered by the caller.
static void process(const gnss_frag_t *frags, int n)
{
    static unsigned char buf[65536];
    gnss_reassemble(frags, n, buf, sizeof buf);

    int64_t ts_sent_ms = 0;
    for (int i = 0; i < 8; ++i)
        ts_sent_ms |= (int64_t)frags[0].tskey[i] << (8 * i);

    const sent_cmd_t *cmd = find_cmd(ts_sent_ms);
    const char *command = cmd ? cmd->command : NULL;

    // The primary path: a response to adcs_generic_telemetry_request(151,6)
    // (magnetometer) or (159,6) (IGRF model). The frame id comes from the
    // resolved command, and the response body is a hex byte dump, not JSON --
    // so it can only be recognized once we know which frame was requested.
    adcs_mag_vec_t v;
    rec_kind_t kind;
    int telem_frame = 0;
    int frame = command ? adcs_generic_telem_frame(command) : -1;
    if (frame == ADCS_TELEM_FRAME_MAG_FIELD || frame == ADCS_TELEM_FRAME_IGRF) {
        if (!adcs_mag_parse_telem_hex((const char *)buf, ADCS_TELEM_NT_PER_LSB, &v)) return;
        kind = (frame == ADCS_TELEM_FRAME_IGRF) ? REC_IGRF : REC_MEASURED;
        telem_frame = frame;
    } else {
        // The named commands (adcs_magnetic_field_vector / _get_raw_magnetometer_values
        // / adcs_measurements) answer with JSON.
        if (!adcs_mag_parse((const char *)buf, &v)) return;   // not a field response
        kind = classify(command, &v);
    }

    char ids[160];
    build_ids(ids, sizeof ids, frags, n);

    // Measurement time: the scheduled execution time is the truest instant;
    // failing that the command's send time; failing that ground reception.
    int64_t meas_ms = 0; char basis[12];
    if (cmd && cmd->tsexec_ms > 0) { meas_ms = cmd->tsexec_ms; snprintf(basis, sizeof basis, "tsexec"); }
    else if (ts_sent_ms > 0)       { meas_ms = ts_sent_ms;     snprintf(basis, sizeof basis, "tssent"); }
    else { iso_to_unix_ms(frags[0].ts_received, &meas_ms);     snprintf(basis, sizeof basis, "received"); }

    emit_reading(kind, &v, command, ts_sent_ms, frags[0].ts_received,
                 meas_ms, basis, ids, 0, telem_frame);
}

// ---- bulk-file adcs_data log scan ------------------------------------------
//
// ADCS telemetry is often stored to an SD file (the command carried
// @resp_fname=adcs_data/...) and later bulk-downlinked, rather than answered
// inline -- which is exactly how the magnetometer measurements arrive. Those
// files are line-structured logs: a header object
//   {"ts_sent":..,"ts_done":<exec ms>,"tcmd":"adcs_generic_telemetry_request",..,"args":"151,6"}
// then the response, then a [END_RESPONSE] marker. ts_done is the precise
// measurement instant. The response shape depends on the command: the named
// commands answer with JSON ({"x":..} / {"x_nT":..}); the generic-telemetry
// frames -- 151 (magnetometer) and 159 (IGRF model) -- answer with a hex byte
// dump ("55 a 8 f4 6a 1"), little-endian int16 x10 nT. We reassemble each
// download and scan it for both forms. Best-effort: a bulk download arrives in
// 195-byte chunks placed by file_offset, often with bit-errored duplicates, so
// garbled entries simply fail to parse and are skipped.

#define BULK_FILE_HDR_SIZE  5             // packet_type(1) + file_offset(4 LE)
#define BULK_FILE_MAX_DATA  195
#define BULK_OFF_CAP        (8L * 1024 * 1024)   // sane reassembled-file cap

static long bulk_off(const uint8_t *pl)
{
    return (long)pl[1] | ((long)pl[2] << 8)
         | ((long)pl[3] << 16) | ((long)pl[4] << 24);
}

// Value of the last occurrence of `key` (e.g. "\"ts_done\":") strictly before
// `before`, parsed as a signed integer. 0 if not found. The buffer has no
// embedded NULs (gaps are spaces), so strstr scans the whole thing.
static int64_t find_i64_back(const char *buf, const char *before, const char *key)
{
    size_t klen = strlen(key);
    const char *best = NULL, *p = buf;
    while ((p = strstr(p, key)) != NULL && p < before) { best = p; p += klen; }
    if (best == NULL) return 0;
    const char *q = best + klen;
    while (*q == ' ' || *q == '\t') q++;
    return (int64_t)strtoll(q, NULL, 10);
}

typedef struct { long off; int score; int len; uint8_t *data; } bchunk_t;

static int cmp_bchunk_score_desc(const void *a, const void *b)
{
    int sa = ((const bchunk_t *)a)->score, sb = ((const bchunk_t *)b)->score;
    return (sa < sb) - (sa > sb);   // dirtiest first, so the cleanest is placed last
}

// Reassemble one download (all type-16 chunks of one source_run) and emit any
// magnetometer readings found in the recovered log text.
static void scan_bulk_run(sqlite3 *db, const char *run,
                          int64_t since_ms, int64_t until_ms)
{
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT payload, golay_errs, rs_errs FROM packet "
            "WHERE packet_type=16 AND ifnull(source_run,'')=?1 "
            "ORDER BY ts_received, id", -1, &st, NULL) != SQLITE_OK)
        return;
    sqlite3_bind_text(st, 1, run, -1, SQLITE_TRANSIENT);

    bchunk_t *cs = NULL; int n = 0, cap = 0; long max_end = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const uint8_t *pl = sqlite3_column_blob(st, 0);
        int pl_len = sqlite3_column_bytes(st, 0);
        if (pl == NULL || pl_len <= BULK_FILE_HDR_SIZE) continue;
        long off = bulk_off(pl);
        int dlen = pl_len - BULK_FILE_HDR_SIZE;
        if (dlen > BULK_FILE_MAX_DATA) dlen = BULK_FILE_MAX_DATA;
        if (off < 0 || off + dlen > BULK_OFF_CAP) continue;

        if (n == cap) {
            int nc = cap ? cap * 2 : 256;
            bchunk_t *t = realloc(cs, (size_t)nc * sizeof *t);
            if (t == NULL) break;
            cs = t; cap = nc;
        }
        int ge = sqlite3_column_int(st, 1), re = sqlite3_column_int(st, 2);
        cs[n].off = off; cs[n].len = dlen;
        cs[n].score = (re == -2 ? 100000 : (re < 0 ? 0 : re)) + (ge < 0 ? 0 : ge);
        cs[n].data = malloc((size_t)dlen);
        if (cs[n].data == NULL) break;
        memcpy(cs[n].data, pl + BULK_FILE_HDR_SIZE, (size_t)dlen);
        if (off + dlen > max_end) max_end = off + dlen;
        n++;
    }
    sqlite3_finalize(st);
    if (n == 0) { free(cs); return; }

    // Place chunks dirtiest-first so a clean copy overwrites a corrupted one at
    // the same offset. Gaps stay as spaces (no embedded NUL breaks the scan).
    char *buf = malloc((size_t)max_end + 1);
    if (buf == NULL) { for (int i = 0; i < n; ++i) free(cs[i].data); free(cs); return; }
    memset(buf, ' ', (size_t)max_end);
    buf[max_end] = '\0';
    qsort(cs, (size_t)n, sizeof *cs, cmp_bchunk_score_desc);
    for (int i = 0; i < n; ++i) {
        long o = cs[i].off, l = cs[i].len;
        if (o + l > max_end) l = max_end - o;
        if (l > 0) memcpy(buf + o, cs[i].data, (size_t)l);
        free(cs[i].data);
    }
    free(cs);

    // The log is ASCII; a bit-errored chunk could carry a stray NUL that would
    // truncate the strstr scan and hide every entry after it. Neutralize them.
    for (long i = 0; i < max_end; ++i) if (buf[i] == '\0') buf[i] = ' ';

    // Scan for log entries whose tcmd is a magnetometer telemetry request.
    const char *p = buf;
    while ((p = strstr(p, "\"tcmd\":\"")) != NULL) {
        const char *name = p + 8;
        const char *nend = strchr(name, '"');
        p = name;
        if (nend == NULL) break;
        int nl = (int)(nend - name);
        if (nl <= 0 || nl >= 64) continue;
        char tcmd[64];
        memcpy(tcmd, name, (size_t)nl); tcmd[nl] = '\0';

        const char *hdr_end = strchr(nend, '}');
        if (hdr_end == NULL) continue;

        // The generic-telemetry frames (151 magnetometer, 159 IGRF) are the
        // primary source; the named magnetometer commands (..._magnetometer...,
        // ..._magnetic_field..., adcs_measurements) are the JSON fallback.
        int is_generic = (strcmp(tcmd, "adcs_generic_telemetry_request") == 0);
        int is_named_mag = (strstr(tcmd, "magnetometer") || strstr(tcmd, "magnetic_field")
                            || strstr(tcmd, "measurements"));
        if (!is_generic && !is_named_mag) continue;

        adcs_mag_vec_t v;
        rec_kind_t kind;
        int telem_frame = 0;

        if (is_generic) {
            // Frame id from the header's "args":"<id>,<len>"; only 151/159 are
            // a magnetic field. The response is the hex dump after the header,
            // up to [END_RESPONSE] (or the next entry if the marker was lost).
            const char *a = strstr(nend, "\"args\":\"");
            int frame = (a && a < hdr_end) ? atoi(a + 8) : -1;
            if (frame != ADCS_TELEM_FRAME_MAG_FIELD && frame != ADCS_TELEM_FRAME_IGRF)
                continue;
            const char *body = hdr_end + 1;
            const char *end_marker = strstr(body, "[END_RESPONSE]");
            const char *next = strstr(body, "\"tcmd\":\"");
            const char *bound = end_marker ? end_marker : next;
            if (bound == NULL || bound <= body) continue;
            int tl = (int)(bound - body);
            if (tl >= 2048) tl = 2047;
            char tmp[2048];
            memcpy(tmp, body, (size_t)tl); tmp[tl] = '\0';
            if (!adcs_mag_parse_telem_hex(tmp, ADCS_TELEM_NT_PER_LSB, &v)) continue;
            kind = (frame == ADCS_TELEM_FRAME_IGRF) ? REC_IGRF : REC_MEASURED;
            telem_frame = frame;
        } else {
            const char *resp = strchr(hdr_end + 1, '{');
            if (resp == NULL) continue;
            const char *brace = strchr(resp, '}');
            const char *end_marker = strstr(resp, "[END_RESPONSE]");
            const char *bound = brace ? brace + 1 : (end_marker ? end_marker : NULL);
            if (bound == NULL || bound <= resp) continue;
            int tl = (int)(bound - resp);
            if (tl >= 2048) continue;
            char tmp[2048];
            memcpy(tmp, resp, (size_t)tl); tmp[tl] = '\0';
            if (!adcs_mag_parse(tmp, &v)) continue;
            kind = classify(tcmd, &v);
        }

        int64_t tsdone = find_i64_back(buf, nend, "\"ts_done\":");
        int64_t tssent = find_i64_back(buf, nend, "\"ts_sent\":");
        int64_t meas = tsdone > 0 ? tsdone : tssent;
        if (meas <= 0) continue;
        if (since_ms && meas < since_ms) continue;
        if (until_ms && meas > until_ms) continue;

        char rxiso[40];
        fmt_epoch_ms(meas, rxiso, sizeof rxiso);
        emit_reading(kind, &v, tcmd, tssent > 0 ? tssent : 0,
                     rxiso, meas, tsdone > 0 ? "tsdone" : "tssent", "", 1, telem_frame);
    }
    free(buf);
}

static void scan_bulk_logs(sqlite3 *db, int64_t since_ms, int64_t until_ms)
{
    // Collect the distinct download runs first (NULL run -> ""), then
    // reassemble each: a second prepared statement can't be live during the
    // per-run query otherwise.
    char (*runs)[256] = NULL; int nruns = 0, cap = 0;
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT DISTINCT ifnull(source_run,'') FROM packet WHERE packet_type=16",
            -1, &st, NULL) != SQLITE_OK)
        return;
    while (sqlite3_step(st) == SQLITE_ROW) {
        if (nruns == cap) {
            int nc = cap ? cap * 2 : 32;
            char (*t)[256] = realloc(runs, (size_t)nc * sizeof *t);
            if (t == NULL) break;
            runs = t; cap = nc;
        }
        snprintf(runs[nruns], 256, "%s", (const char *)sqlite3_column_text(st, 0));
        nruns++;
    }
    sqlite3_finalize(st);
    for (int i = 0; i < nruns; ++i) scan_bulk_run(db, runs[i], since_ms, until_ms);
    free(runs);
}

static int cmp_rec_asc(const void *a, const void *b)
{
    int64_t x = ((const rec_t *)a)->meas_ms;
    int64_t y = ((const rec_t *)b)->meas_ms;
    return (x > y) - (x < y);
}

// Pair each measured reading with the nearest IGRF reading within the window
// (and vice versa), so the report/JSON can show the |B| residual.
static void pair_readings(double window_s)
{
    int64_t window_ms = (int64_t)(window_s * 1000.0);
    for (int i = 0; i < g_nrecs; ++i) {
        if (g_recs[i].kind != REC_MEASURED) continue;
        int best = -1; int64_t bestgap = window_ms + 1;
        for (int j = 0; j < g_nrecs; ++j) {
            if (g_recs[j].kind != REC_IGRF) continue;
            int64_t gap = llabs(g_recs[i].meas_ms - g_recs[j].meas_ms);
            if (gap <= window_ms && gap < bestgap) { bestgap = gap; best = j; }
        }
        if (best >= 0) {
            g_recs[i].pair = best;
            if (g_recs[best].pair < 0) g_recs[best].pair = i;   // mutual link
        }
    }
}

// First-order raw-count -> nT scale for uncalibrated readings (see --help and
// the note printed in summary). 1.0 nT/count unless overridden.
static double g_nt_per_count = 1.0;

static const char *kind_word(rec_kind_t k)
{
    switch (k) {
        case REC_MEASURED:   return "measured";
        case REC_IGRF:       return "igrf";
        case REC_RAW:        return "raw";
        default:             return "unresolved";
    }
}

// ---- output ----------------------------------------------------------------

static void print_human(const rec_t *r)
{
    const char *label = (r->kind == REC_MEASURED) ? "MEASURED"
                      : (r->kind == REC_IGRF)     ? "IGRF"
                      : (r->kind == REC_RAW)      ? "RAW"
                                                  : "MAG?";
    char meas_iso[40];
    fmt_epoch_ms(r->meas_ms, meas_iso, sizeof meas_iso);

    printf("[%s]%s %-8s %s\n", r->ts_received, r->from_bulk ? " (bulk log)" : "",
           label,
           r->command[0] ? r->command : "(command unknown -- measured or IGRF)");
    if (r->ids[0]) printf("  ids %s\n", r->ids);

    if (rec_is_counts(r)) {
        // Raw counts are uncalibrated; the nT figure is a first-order estimate
        // (see the note in the summary), not the on-board calibrated field.
        printf("  raw counts  (x, y, z) = (%ld, %ld, %ld)   |raw| = %.0f counts\n",
               r->v.x_nT, r->v.y_nT, r->v.z_nT, r->mag);
        printf("              ~ |B| = %.0f nT  (approx, %.4g nT/count, uncalibrated)\n",
               r->mag * g_nt_per_count, g_nt_per_count);
    } else {
        char flabel[48];
        if (r->telem_frame)
            snprintf(flabel, sizeof flabel, "%s (frame %d)",
                     r->kind == REC_IGRF ? "IGRF model" : "measured", r->telem_frame);
        else
            snprintf(flabel, sizeof flabel, "%s",
                     r->kind == REC_IGRF ? "IGRF model"
                   : r->field_measurements ? "measured (adcs_measurements)" : "measured");
        printf("  %s  B = (%ld, %ld, %ld) nT   |B| = %.0f nT\n",
               flabel, r->v.x_nT, r->v.y_nT, r->v.z_nT, r->mag);
        if (r->telem_frame)
            printf("              int16 counts (%ld, %ld, %ld)  x%d nT/LSB\n",
                   r->v.x_nT / ADCS_TELEM_NT_PER_LSB, r->v.y_nT / ADCS_TELEM_NT_PER_LSB,
                   r->v.z_nT / ADCS_TELEM_NT_PER_LSB, ADCS_TELEM_NT_PER_LSB);
    }

    if (r->have_pos)
        printf("  position @ %s %s: lat %.3f  lon %.3f  alt %.1f km\n",
               r->meas_basis, meas_iso, r->lat, r->lon, r->alt_km);
    else
        printf("  position @ %s %s: unavailable (no TLE near this time)\n",
               r->meas_basis, meas_iso);

    if (r->tle_id) {
        char ep[16];
        fmt_epoch_date(r->tle_epoch_ms, ep, sizeof ep);
        double gap_h = (double)(r->meas_ms - r->tle_epoch_ms) / 3600000.0;
        printf("  TLE: NORAD %lld  epoch %s  (gap %+.1f h)\n",
               r->tle_norad, ep, gap_h);
    }

    if (r->kind == REC_MEASURED && r->pair >= 0) {
        const rec_t *m = &g_recs[r->pair];
        double dt_s = (double)(m->meas_ms - r->meas_ms) / 1000.0;
        printf("  IGRF pair (%+.0f s): model |B| = %.0f nT   |B|-|model| = %+.0f nT"
               "   d=(%+ld, %+ld, %+ld) nT\n",
               dt_s, m->mag, r->mag - m->mag,
               r->v.x_nT - m->v.x_nT, r->v.y_nT - m->v.y_nT, r->v.z_nT - m->v.z_nT);
    }
    printf("\n");
}

// Minimal JSON string escape for a command line (ASCII, possibly with quotes
// or backslashes); control bytes are dropped to keep one object per line.
static void json_escape(const char *s, char *out, size_t outn)
{
    size_t o = 0;
    for (const char *p = s; *p && o + 2 < outn; ++p) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = (char)c; }
        else if (c >= 0x20)        { out[o++] = (char)c; }
    }
    out[o] = '\0';
}

static void print_json(const rec_t *r)
{
    char meas_iso[40], cmd_esc[320];
    fmt_epoch_ms(r->meas_ms, meas_iso, sizeof meas_iso);
    json_escape(r->command, cmd_esc, sizeof cmd_esc);

    const char *field = (r->kind == REC_RAW)  ? "raw_magnetometer_values"
                      : (r->kind == REC_IGRF)  ? "igrf_magnetic_field_vector"
                      : r->field_measurements  ? "measurements"
                                               : "magnetic_field_vector";
    printf("{\"ts_received\":\"%s\",\"source\":\"%s\",\"kind\":\"%s\",\"field\":\"%s\",",
           r->ts_received, r->from_bulk ? "bulk_log" : "tcmd_response",
           kind_word(r->kind), field);

    if (r->command[0]) printf("\"command\":\"%s\",", cmd_esc);
    else               printf("\"command\":null,");

    if (r->ts_sent_ms) printf("\"ts_sent_ms\":%lld,", r->ts_sent_ms);
    else               printf("\"ts_sent_ms\":null,");

    printf("\"meas_time\":\"%s\",\"meas_basis\":\"%s\",", meas_iso, r->meas_basis);

    if (r->telem_frame) printf("\"telem_frame\":%d,", r->telem_frame);
    else                printf("\"telem_frame\":null,");

    if (rec_is_counts(r)) {
        // Uncalibrated: components/magnitude are counts; nT is a first-order
        // estimate via --nt-per-count (default 1.0), not the calibrated field.
        printf("\"unit\":\"counts\",\"vector\":{\"x\":%ld,\"y\":%ld,\"z\":%ld},"
               "\"magnitude\":%.1f,\"nt_per_count\":%.6g,\"approx_b_mag_nT\":%.1f,",
               r->v.x_nT, r->v.y_nT, r->v.z_nT, r->mag,
               g_nt_per_count, r->mag * g_nt_per_count);
    } else {
        printf("\"unit\":\"nT\",\"vector\":{\"x\":%ld,\"y\":%ld,\"z\":%ld},"
               "\"magnitude\":%.1f,",
               r->v.x_nT, r->v.y_nT, r->v.z_nT, r->mag);
    }

    if (r->have_pos)
        printf("\"position\":{\"lat_deg\":%.4f,\"lon_deg\":%.4f,\"alt_km\":%.2f},",
               r->lat, r->lon, r->alt_km);
    else
        printf("\"position\":null,");

    if (r->tle_id) {
        char ep[16];
        fmt_epoch_date(r->tle_epoch_ms, ep, sizeof ep);
        double gap_h = (double)(r->meas_ms - r->tle_epoch_ms) / 3600000.0;
        printf("\"tle\":{\"id\":%lld,\"norad\":%lld,\"epoch\":\"%s\",\"epoch_gap_h\":%.2f},",
               r->tle_id, r->tle_norad, ep, gap_h);
    } else {
        printf("\"tle\":null,");
    }

    if (r->kind == REC_MEASURED && r->pair >= 0) {
        const rec_t *m = &g_recs[r->pair];
        double dt_s = (double)(m->meas_ms - r->meas_ms) / 1000.0;
        printf("\"igrf\":{\"dt_s\":%.1f,\"model_mag_nT\":%.1f,\"residual_mag_nT\":%.1f,"
               "\"dx_nT\":%ld,\"dy_nT\":%ld,\"dz_nT\":%ld},",
               dt_s, m->mag, r->mag - m->mag,
               r->v.x_nT - m->v.x_nT, r->v.y_nT - m->v.y_nT, r->v.z_nT - m->v.z_nT);
    } else {
        printf("\"igrf\":null,");
    }

    printf("\"ids\":[%s]}\n", r->ids);
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "mag_reports")) return 0;

    args_t cfg = {0};
    cfg.pair_window_s = 120.0;
    cfg.nt_per_count  = 1.0;
    cfg.catalog       = FRONTIERSAT_CATALOG;
    switch (parse_args(&cfg, argc, argv, 0)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
        default: break;
    }
    g_nt_per_count = cfg.nt_per_count;
    // An explicit --satellite= replaces the default catalog (but a user who
    // gives both --catalog and --satellite gets the intersection they asked for).
    if (cfg.satellite && !cfg.catalog_explicit) cfg.catalog = 0;

    char db_default[1024];
    const char *db_path = cfg.db_path;
    if (db_path == NULL) {
        if (packet_db_default_path(db_default, sizeof db_default) != 0) {
            fprintf(stderr, "mag_reports: no DB path "
                            "(set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = db_default;
    }

    char since_iso[40] = {0}, until_iso[40] = {0};
    if (cfg.since && gnss_parse_time_spec(cfg.since, since_iso, sizeof since_iso) != 0) {
        fprintf(stderr, "mag_reports: bad --since=%s\n", cfg.since); return 1;
    }
    if (cfg.until && gnss_parse_time_spec(cfg.until, until_iso, sizeof until_iso) != 0) {
        fprintf(stderr, "mag_reports: bad --until=%s\n", cfg.until); return 1;
    }

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "mag_reports: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 1;
    }

    load_sent_tcmds(db);
    int ncat = load_tles(db, &cfg);
    if (ncat > 1) {
        fprintf(stderr,
                "mag_reports: the TLE table holds %d different objects; positions\n"
                "would be ambiguous. Narrow it with --catalog=<n> or --satellite=<s>.\n",
                ncat);
        sqlite3_close(db);
        return 1;
    }
    if (g_ntles == 0)
        fprintf(stderr, "mag_reports: no matching TLEs in the DB -- readings will "
                        "be shown without an associated position.\n");
#ifndef WITH_SGP4SDP4
    if (g_ntles > 0)
        fprintf(stderr, "mag_reports: built without SGP4 (no sgp4sdp4) -- positions "
                        "cannot be computed.\n");
#endif

    const char *sql =
        "SELECT id, ts_received, payload FROM packet "
        "WHERE packet_type=?1 AND length(payload) >= ?2 "
        "ORDER BY " TCMD_RESP_SQL_TS_SENT ", ts_received, id";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "mag_reports: query failed: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
        return 1;
    }
    sqlite3_bind_int(st, 1, TCMD_RESP_PACKET_TYPE);
    sqlite3_bind_int(st, 2, TCMD_RESP_HDR_LEN + 1);

    // Group response packets by their 8-byte ts_sent key; within a group a
    // new reception starts whenever the sequence number stops advancing.
    gnss_frag_t recv[260];
    int nrecv = 0;
    unsigned char curkey[8];
    int have_key = 0, last_seq = 0;

    #define FLUSH() do { \
        if (nrecv > 0) { \
            const char *ts = recv[0].ts_received; \
            int in_range = (!since_iso[0] || strcmp(ts, since_iso) >= 0) \
                        && (!until_iso[0] || strcmp(ts, until_iso) <= 0); \
            if (in_range) process(recv, nrecv); \
            for (int i = 0; i < nrecv; ++i) free(recv[i].payload); \
            nrecv = 0; \
        } \
    } while (0)

    while (sqlite3_step(st) == SQLITE_ROW) {
        const unsigned char *pl = sqlite3_column_blob(st, 2);
        int pl_len = sqlite3_column_bytes(st, 2);
        if (pl == NULL || pl_len < TCMD_RESP_HDR_LEN + 1) continue;
        int seq = tcmd_resp_seq(pl, (size_t)pl_len);
        if (seq < 1) continue;

        unsigned char key[8];
        tcmd_resp_ts_sent(pl, (size_t)pl_len, key);

        int new_group = !have_key || memcmp(key, curkey, 8) != 0;
        if (new_group) { FLUSH(); memcpy(curkey, key, 8); have_key = 1; last_seq = 0; }
        else if (seq <= last_seq) { FLUSH(); }
        if (nrecv >= (int)(sizeof recv / sizeof recv[0])) FLUSH();

        gnss_frag_t *f = &recv[nrecv++];
        f->id = sqlite3_column_int64(st, 0);
        snprintf(f->ts_received, sizeof f->ts_received, "%s",
                 (const char *)sqlite3_column_text(st, 1));
        memcpy(f->tskey, key, 8);
        f->seq = seq;
        f->max_seq = tcmd_resp_max_seq(pl, (size_t)pl_len);
        f->payload_len = pl_len;
        f->payload = malloc((size_t)pl_len);
        if (f->payload) memcpy(f->payload, pl, (size_t)pl_len);
        else            f->payload_len = 0;
        last_seq = seq;
    }
    FLUSH();
    #undef FLUSH
    sqlite3_finalize(st);

    // The bulk-downlinked adcs_data/*.log files hold readings that were stored
    // to the SD card (commands sent with @resp_fname) rather than answered
    // inline. Scan them too unless asked not to.
    if (!cfg.no_bulk) {
        int64_t since_ms = 0, until_ms = 0;
        if (since_iso[0]) iso_to_unix_ms(since_iso, &since_ms);
        if (until_iso[0]) iso_to_unix_ms(until_iso, &until_ms);
        scan_bulk_logs(db, since_ms, until_ms);
    }
    sqlite3_close(db);

    qsort(g_recs, (size_t)g_nrecs, sizeof *g_recs, cmp_rec_asc);
    pair_readings(cfg.pair_window_s);

    int n_meas = 0, n_igrf = 0, n_raw = 0, n_unres = 0, n_paired = 0;
    for (int i = 0; i < g_nrecs; ++i) {
        switch (g_recs[i].kind) {
            case REC_MEASURED: n_meas++; if (g_recs[i].pair >= 0) n_paired++; break;
            case REC_IGRF:     n_igrf++; break;
            case REC_RAW:      n_raw++;  break;
            default:           n_unres++; break;
        }
    }

    if (!cfg.json) {
        printf("ADCS magnetometer readings%s%s%s:\n\n",
               (since_iso[0] || until_iso[0]) ? " (" : "",
               since_iso[0] ? since_iso : (until_iso[0] ? ".." : ""),
               (since_iso[0] || until_iso[0]) ? ")" : "");
    }
    for (int k = 0; k < g_nrecs; ++k) {
        int idx = cfg.reverse ? (g_nrecs - 1 - k) : k;
        if (cfg.json) print_json(&g_recs[idx]);
        else          print_human(&g_recs[idx]);
    }
    if (!cfg.json) {
        printf("Totals: %d measured (%d paired with IGRF), %d IGRF model, "
               "%d raw, %d unresolved.\n", n_meas, n_paired, n_igrf, n_raw, n_unres);
        if (n_raw > 0)
            printf("Note: raw readings are uncalibrated ADC counts. The ~nT figure "
                   "assumes %.4g nT/count (a first-order scale, see --nt-per-count); the\n"
                   "      exact conversion needs the on-board sensitivity matrix + offsets "
                   "from adcs_get_magnetometer_config, which has not been downlinked.\n",
                   g_nt_per_count);
    }

    free(g_recs); free(g_tles); free(g_tle_epochs); free(g_cmds);
    return 0;
}

#endif
