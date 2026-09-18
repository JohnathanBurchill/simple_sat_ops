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

#include "beacon_cts1.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sqlite3.h>

typedef struct {
    double ts_ms;
    float  roll_deg, pitch_deg, yaw_deg;
} att_row_t;

static att_row_t *g_rows = NULL;
static int        g_n    = 0;

int beacon_attitude_count(void) { return g_n; }

void beacon_attitude_free(void)
{
    free(g_rows);
    g_rows = NULL;
    g_n = 0;
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
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        sqlite3_close(db);
        return -1;
    }

    int cap = 0;
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

        if (g_n == cap) {
            int ncap = cap ? cap * 2 : 512;
            att_row_t *t = (att_row_t *) realloc(g_rows,
                                                 (size_t) ncap * sizeof *t);
            if (t == NULL) break;   // keep what we have rather than lose it all
            g_rows = t;
            cap = ncap;
        }
        att_row_t *r = &g_rows[g_n++];
        r->ts_ms     = sqlite3_column_double(st, 0);
        r->roll_deg  = b.adcs_estimated_roll_angle_cdeg  / 100.0f;
        r->pitch_deg = b.adcs_estimated_pitch_angle_cdeg / 100.0f;
        r->yaw_deg   = b.adcs_estimated_yaw_angle_cdeg   / 100.0f;
    }
    sqlite3_finalize(st);
    sqlite3_close(db);
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
    return 1;
}
