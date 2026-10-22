/*

    Simple Satellite Operations  utils/beacon_attitude.c

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

#include "beacon_attitude.h"

#include "adcs_sdlog.h"
#include "beacon_cts1.h"
#include "bulk_size.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sqlite3.h>

typedef struct {
    double ts_ms;
    float  roll_deg, pitch_deg, yaw_deg;
    int    from_log;
} att_row_t;

static att_row_t *g_rows = NULL;
static int        g_n    = 0;
static int        g_cap  = 0;

int beacon_attitude_count(void) { return g_n; }

void beacon_attitude_free(void)
{
    free(g_rows);
    g_rows = NULL;
    g_n = 0;
    g_cap = 0;
}

// Room for one more attitude, or NULL when there is none to be had -- in
// which case the caller keeps what it has rather than lose it all.
static att_row_t *new_row(void)
{
    if (g_n == g_cap) {
        int ncap = g_cap ? g_cap * 2 : 512;
        att_row_t *t = (att_row_t *) realloc(g_rows, (size_t) ncap * sizeof *t);
        if (t == NULL) return NULL;
        g_rows = t;
        g_cap = ncap;
    }
    att_row_t *r = &g_rows[g_n++];
    memset(r, 0, sizeof *r);
    return r;
}

static int cmp_row(const void *a, const void *b)
{
    double x = ((const att_row_t *) a)->ts_ms, y = ((const att_row_t *) b)->ts_ms;
    return (x > y) - (x < y);
}

static int load_beacons(sqlite3 *db)
{
    // The same shape test beacon_is_extended makes, pushed into SQL so
    // the scan does not haul every packet in the database across the
    // interface. Extended beacons decoded before the ground station
    // knew the type are stored as "unknown", so the type name alone
    // would miss nearly all of them -- see packet_browser's
    // BEACON_EXT_PREDICATE, which this mirrors. The blob literals are
    // "CTS1" and " X"; a blob never compares equal to a text string in
    // SQLite, so they have to be written as hex.
    const char *sql =
        "SELECT (julianday(ts_received) - 2440587.5) * 86400000.0, payload "
        "FROM packet "
        "WHERE length(payload) IN (198, 202) "
        "  AND substr(payload, 1, 1) = x'20' "
        "  AND (substr(payload, 2, 4) = x'43545331' "
        "       OR substr(payload, 127, 2) = x'2058') "
        "  AND rs_errs != -2 AND crc_status != 0 "
        "ORDER BY ts_received";

    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) return -1;

    while (sqlite3_step(st) == SQLITE_ROW) {
        const uint8_t *pl = (const uint8_t *) sqlite3_column_blob(st, 1);
        int pln = sqlite3_column_bytes(st, 1);
        if (pl == NULL) continue;
        // The SQL narrowed it down; the real decoder decides. Both
        // magic fields have to be exactly right as well -- a pointing
        // direction drawn from a frame of noise is a confident line at
        // the wrong place on the Earth, which is worse than no line.
        if (!beacon_is_extended(pl, (size_t) pln)) continue;
        if (!beacon_magic_intact(pl, (size_t) pln)) continue;
        if (!beacon_ext_attitude_is_valid(pl, (size_t) pln)) continue;

        COMMS_beacon_extended_packet_t b;
        memcpy(&b, pl, sizeof b);

        att_row_t *r = new_row();
        if (r == NULL) break;
        r->ts_ms     = sqlite3_column_double(st, 0);
        r->roll_deg  = b.adcs_estimated_roll_angle_cdeg  / 100.0f;
        r->pitch_deg = b.adcs_estimated_pitch_angle_cdeg / 100.0f;
        r->yaw_deg   = b.adcs_estimated_yaw_angle_cdeg   / 100.0f;
    }
    sqlite3_finalize(st);
    return 0;
}

// ---- the ADCS's own SD-card logs ---------------------------------------
//
// The beacons only say where the satellite was looking while a ground
// station could hear it, and the MPI is run where none can. The ADCS's
// SD-card log records the attitude on board at a fixed period, and comes
// down later as a file; this reads every log in the database that
// records the estimated angles (see adcs_sdlog.h for the format).
//
// Nothing in a bulk_file packet says which file it is from, so a log is
// put back together by content. The chunk at offset 0 of each log holds
// its mask and its first record's time, which together name the log; the
// period comes from a vote over every record stamp in the database; and a
// chunk then belongs to the log when every stamp inside it sits on that
// log's schedule. Only chunks whose CSP CRC32 checked out are used -- every
// one that failed it also failed its Reed-Solomon decode -- for the same
// reason as the beacons' magic test above.
//
// The cost of that rule is a chunk with no whole stamp inside it, which
// cannot be judged and is left out. With records shorter than a chunk that
// is only ever the short last chunk of a file, so a log loses its last
// record.

#define SDLOG_CHUNK_MAX         195
#define SDLOG_PERIOD_CANDIDATES 64

typedef struct {
    long    off;
    int     len;
    uint8_t d[SDLOG_CHUNK_MAX];
} sd_chunk_t;

// Every bulk_file chunk whose CRC32 checked out. A verified packet had its
// 4-byte CRC trailer stripped on the way into the database, so what
// follows the 5-byte header is all file data.
static int load_chunks(sqlite3 *db, sd_chunk_t **out)
{
    *out = NULL;
    const char *sql =
        "SELECT payload FROM packet "
        "WHERE packet_type = ? AND crc_status = 1 AND rs_errs != -2";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) return 0;
    sqlite3_bind_int(st, 1, COMMS_PACKET_TYPE_BULK_FILE_DOWNLINK);

    sd_chunk_t *cs = NULL;
    int n = 0, cap = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        const uint8_t *pl = (const uint8_t *) sqlite3_column_blob(st, 0);
        int len = sqlite3_column_bytes(st, 0) - 5;
        if (pl == NULL || len <= 0) continue;
        if (len > SDLOG_CHUNK_MAX) len = SDLOG_CHUNK_MAX;
        long off = (long) pl[1] | ((long) pl[2] << 8)
                 | ((long) pl[3] << 16) | ((long) pl[4] << 24);
        if (off < 0 || off + len > BULK_SIZE_MAX_PLAUSIBLE) continue;
        if (n == cap) {
            int ncap = cap ? cap * 2 : 4096;
            sd_chunk_t *t = (sd_chunk_t *) realloc(cs, (size_t) ncap * sizeof *t);
            if (t == NULL) break;
            cs = t;
            cap = ncap;
        }
        cs[n].off = off;
        cs[n].len = len;
        memcpy(cs[n].d, pl + 5, (size_t) len);
        n++;
    }
    sqlite3_finalize(st);
    *out = cs;
    return n;
}

// The log's period, by vote: every record stamp in every chunk proposes the
// period that would put it where it sits, and the one proposed most often
// wins. The log's own chunks all propose the same period; a chunk of any
// other file proposes nothing, bar the odd stray.
static long log_period(const adcs_sdlog_t *log, const sd_chunk_t *cs, int nc)
{
    long cand[SDLOG_PERIOD_CANDIDATES];
    int votes[SDLOG_PERIOD_CANDIDATES];
    int ncand = 0;
    for (int k = 0; k < nc; k++) {
        long i0 = 0, i1 = -1;
        if (adcs_sdlog_stamps_in(log, cs[k].off, (size_t) cs[k].len, &i0, &i1) == 0)
            continue;
        for (long i = i0; i <= i1; i++) {
            double t = adcs_sdlog_stamp(cs[k].d + (adcs_sdlog_record_offset(log, i) - cs[k].off));
            long p = adcs_sdlog_period_at(log, i, t);
            if (p == 0) continue;
            int c = 0;
            while (c < ncand && cand[c] != p) c++;
            if (c == ncand) {
                if (ncand == SDLOG_PERIOD_CANDIDATES) continue;
                cand[ncand] = p;
                votes[ncand++] = 0;
            }
            votes[c]++;
        }
    }
    long best = 0;
    int most = 0;
    for (int c = 0; c < ncand; c++)
        if (votes[c] > most) { most = votes[c]; best = cand[c]; }
    return best;
}

static int le16s(const uint8_t *p)
{
    return (int16_t) ((uint16_t) p[0] | ((uint16_t) p[1] << 8));
}

// Every record of one reassembled log that came down whole, as attitudes.
static void read_records(const adcs_sdlog_t *log, long period,
                         const uint8_t *buf, const uint8_t *have, long end)
{
    const int a_off = adcs_sdlog_frame_offset(log, ADCS_SDLOG_FRAME_EST_ANGLES);
    const int s_off = adcs_sdlog_frame_offset(log, ADCS_SDLOG_FRAME_STATE);
    for (long i = 0; adcs_sdlog_record_offset(log, i) + log->rec_len <= end; i++) {
        const long p = adcs_sdlog_record_offset(log, i);
        int whole = 1;
        for (long b = 0; b < log->rec_len; b++)
            if (!have[p + b]) { whole = 0; break; }
        if (!whole) continue;

        // A stamp cut across two chunks was judged by neither of them.
        double t = adcs_sdlog_stamp(buf + p);
        if (t < 0.0 || fabs(t - (log->t0 + (double) i * (double) period))
                       > ADCS_SDLOG_SCHEDULE_TOL_S)
            continue;

        // A log that records the ADCS state says whether the estimator was
        // in a mode that fills the angles in -- the same modes
        // beacon_ext_attitude_is_valid accepts for a beacon.
        if (s_off >= 0) {
            beacon_ext_adcs_state_t st;
            beacon_ext_adcs_state(buf + p + s_off, &st);
            if (!st.reported || st.estimation_mode < 3 || st.estimation_mode > 6)
                continue;
        }

        // One that does not can still say so: all three angles read exactly
        // zero only when nothing estimated them. In the one mode that
        // estimates pitch alone, roll and yaw read zero and pitch does not.
        int roll = le16s(buf + p + a_off);
        int pitch = le16s(buf + p + a_off + 2);
        int yaw = le16s(buf + p + a_off + 4);
        if (roll == 0 && pitch == 0 && yaw == 0) continue;

        att_row_t *r = new_row();
        if (r == NULL) return;
        r->ts_ms     = t * 1000.0;
        r->roll_deg  = roll / 100.0f;
        r->pitch_deg = pitch / 100.0f;
        r->yaw_deg   = yaw / 100.0f;
        r->from_log  = 1;
    }
}

static void load_sd_logs(sqlite3 *db)
{
    sd_chunk_t *cs = NULL;
    int nc = load_chunks(db, &cs);
    adcs_sdlog_t *logs = (adcs_sdlog_t *) malloc((size_t) (nc > 0 ? nc : 1) * sizeof *logs);
    int *mine = (int *) malloc((size_t) (nc > 0 ? nc : 1) * sizeof *mine);
    if (cs == NULL || logs == NULL || mine == NULL) {
        free(cs); free(logs); free(mine);
        return;
    }

    // The logs: every chunk at offset 0 that reads as the start of one that
    // records the estimated angles, once each.
    int nl = 0;
    for (int k = 0; k < nc; k++) {
        adcs_sdlog_t h;
        if (cs[k].off != 0) continue;
        if (!adcs_sdlog_parse_header(cs[k].d, (size_t) cs[k].len, &h)) continue;
        if (adcs_sdlog_frame_offset(&h, ADCS_SDLOG_FRAME_EST_ANGLES) < 0) continue;
        int seen = 0;
        for (int l = 0; l < nl && !seen; l++)
            seen = logs[l].t0 == h.t0 && memcmp(logs[l].mask, h.mask, ADCS_SDLOG_MASK_LEN) == 0;
        if (!seen) logs[nl++] = h;
    }

    for (int l = 0; l < nl; l++) {
        long period = log_period(&logs[l], cs, nc);
        if (period == 0) continue;

        int nm = 0;
        long end = 0;
        for (int k = 0; k < nc; k++) {
            if (adcs_sdlog_chunk_fits(&logs[l], period, cs[k].off,
                                      cs[k].d, (size_t) cs[k].len) <= 0)
                continue;
            mine[nm++] = k;
            if (cs[k].off + cs[k].len > end) end = cs[k].off + cs[k].len;
        }

        uint8_t *buf = (uint8_t *) calloc((size_t) end + 1, 1);
        uint8_t *have = (uint8_t *) calloc((size_t) end + 1, 1);
        if (buf != NULL && have != NULL) {
            for (int m = 0; m < nm; m++) {
                const sd_chunk_t *c = &cs[mine[m]];
                memcpy(buf + c->off, c->d, (size_t) c->len);
                memset(have + c->off, 1, (size_t) c->len);
            }
            read_records(&logs[l], period, buf, have, end);
        }
        free(buf);
        free(have);
    }
    free(cs);
    free(logs);
    free(mine);
}

int beacon_attitude_load(const char *db_path)
{
    beacon_attitude_free();
    if (db_path == NULL) return -1;

    sqlite3 *db = NULL;
    // Read-write for the same reason packet_browser opens it that way:
    // under SQLITE_OPEN_READONLY a WAL database falls back to
    // rollback-journal emulation and its read lock blocks the receiver.
    // Nothing here issues a write.
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READWRITE, NULL) != SQLITE_OK) {
        if (db != NULL) sqlite3_close(db);
        return -1;
    }
    sqlite3_busy_timeout(db, 5000);

    if (load_beacons(db) != 0) {
        sqlite3_close(db);
        return -1;
    }
    load_sd_logs(db);
    sqlite3_close(db);

    // The beacons come out of the query in time order and the logs one log
    // at a time; the lookup bisects, so the two are put in one order.
    if (g_n > 1) qsort(g_rows, (size_t) g_n, sizeof *g_rows, cmp_row);
    return g_n;
}

int beacon_attitude_nearest(double unix_ms, double max_age_s,
                            globe_attitude_t *out)
{
    if (out == NULL) return 0;
    globe_attitude_t none = {0};
    *out = none;
    if (g_n == 0) return 0;

    // Bisect on the sorted times, then look at the two either side of
    // where the moment falls.
    int lo = 0, hi = g_n - 1;
    while (lo < hi) {
        int mid = lo + (hi - lo) / 2;
        if (g_rows[mid].ts_ms < unix_ms) lo = mid + 1;
        else                             hi = mid;
    }
    int best = lo;
    if (lo > 0 && fabs(g_rows[lo - 1].ts_ms - unix_ms)
                < fabs(g_rows[lo].ts_ms - unix_ms)) best = lo - 1;

    const double age_s = (g_rows[best].ts_ms - unix_ms) / 1000.0;
    if (fabs(age_s) > max_age_s) return 0;

    out->have      = 1;
    out->roll_deg  = g_rows[best].roll_deg;
    out->pitch_deg = g_rows[best].pitch_deg;
    out->yaw_deg   = g_rows[best].yaw_deg;
    out->age_s     = age_s;
    out->from_log  = g_rows[best].from_log;
    return 1;
}
