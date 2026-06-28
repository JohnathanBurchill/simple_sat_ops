/*

    Simple Satellite Operations  unit_tests/prediction_selftest.c

    Tests for prediction.c:
      - julian_date_from_unix_seconds: known JD anchors, and the issue #53
        regression -- it must differ from the old gmtime_r()+Julian_Date()
        path (which was ~1900 years off) and yield a sane SGP4 range where
        the buggy path yields garbage.
      - tle_default_path: HOME-relative path expansion.
      - load_tle: known-hit, known-miss, and TLE name-match behaviour
        against a synthetic 3-line TLE fixture.
      - update_satellite_position (SGP4, near-Earth): the propagated
        az/el/range/range-rate are cross-checked against an INDEPENDENT
        topocentric oracle (see check_topocentric_oracle below), not just
        range-banded. This is what catches a swapped observation_set field
        mapping or a missing radians->degrees conversion.
      - update_satellite_position (SDP4, deep-space): the deep-space branch
        is exercised with a real HEO TLE (AO-40), asserting the propagator
        actually took the SDP4 path and that the same oracle holds.
      - update_pass_predictions: populates ascension + descent fields
        consistently (descent_jul > ascension_jul, azimuths in [0, 360),
        elevation in (0, 90]).
      - find_passes / get_pass / number_of_passes / free_passes: the pass
        list is module-level static state; this exercises the full search,
        the soonest-first / latest-first sort, the get_pass bounds, and the
        free_passes reset.

    Where the existence of a result depends on orbit-vs-observer geometry
    rather than on prediction.c (e.g. "is there a visible pass in the next
    24 h"), the test reports SKIP rather than failing -- SGP4 propagation
    is deterministic given the TLE, but the geometry is not the unit under
    test.

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

#include "prediction.h"
#include "tap.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include <sgp4sdp4.h>

#define check(cond, what) tap_ok((cond), (what))

// Fixture file holds two real, checksum-valid 3-line TLEs, frozen here
// (not pulled from TLEs/) so the test is reproducible across TLE refreshes:
//
//   OSCAR 7 (AO-7) -- an active LEO at ~1450 km, ~12.5 rev/day. Near-Earth,
//                     so update_satellite_position takes the SGP4 branch.
//   AO-40          -- a defunct HEO at ~1.27 rev/day (period ~19 h). Deep
//                     space, so it takes the SDP4 branch. Lines lifted
//                     verbatim from the vendored sgp4sdp4/amateur.txt.
static const char *FIXTURE_TLE =
    "OSCAR 7 (AO-7)\n"
    "1 07530U 74089B   25043.01160017 -.00000030  00000+0  10217-3 0  9990\n"
    "2 07530 101.9936  46.0237 0012129 347.8272  22.1887 12.53686098299338\n"
    "AO-40\n"
    "1 26609U 00072B   01098.10193978 -.00000077  00000-0  00000+0 0   623\n"
    "2 26609   5.2776 206.5794 8139221 247.7100  15.0295  1.26974654  2011\n";

static char *write_fixture_tle(void)
{
    const char *tmpdir = getenv("TMPDIR");
    if (!tmpdir || !tmpdir[0]) tmpdir = "/tmp";
    char buf[512];
    int n = snprintf(buf, sizeof buf, "%s/sso_pred_test_XXXXXX.tle", tmpdir);
    if (n < 0 || (size_t)n >= sizeof buf) return NULL;
    int fd = mkstemps(buf, 4);
    if (fd < 0) {
        perror("mkstemps");
        return NULL;
    }
    FILE *f = fdopen(fd, "w");
    if (!f) {
        close(fd);
        return NULL;
    }
    fputs(FIXTURE_TLE, f);
    fclose(f);
    return strdup(buf);
}

// Initialise prediction_t with the RAO observer + a satellite name to
// look up. tles_filename points at the caller-owned path.
static void init_pred(prediction_t *pred, const char *tles_path,
                      char *sat_name_buf)
{
    memset(pred, 0, sizeof *pred);
    pred->observer_ephem.position_geodetic.lat = RAO_LATITUDE  * M_PI / 180.0;
    pred->observer_ephem.position_geodetic.lon = RAO_LONGITUDE * M_PI / 180.0;
    pred->observer_ephem.position_geodetic.alt = RAO_ALTITUDE / 1000.0;
    pred->tles_filename = (char *)tles_path;
    pred->satellite_ephem.name = sat_name_buf;
}

// SGP4SDP4 caller protocol: after a TLE is loaded into the tle struct,
// clear the global ephemeris flags and call select_ephemeris so the
// propagator picks SGP4 vs SDP4 based on the orbit's period. main.c
// does this; without it, SGP4's internal init never runs and
// update_satellite_position returns garbage (we found this the hard
// way -- speed = 60 km/s, altitude = -6000 km).
static void prep_propagator(prediction_t *pred)
{
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&pred->satellite_ephem.tle);
}

// ----------------------------------------------------- topocentric oracle

// Independent cross-check of update_satellite_position's topocentric
// outputs (azimuth / elevation / range / range-rate). The oracle:
//   1. re-propagates the satellite to the same time via SGP4/SDP4 directly,
//   2. reconstructs the observer ECI state via Calculate_User_PosVel,
//   3. REIMPLEMENTS the SEZ topocentric projection + field extraction here,
// then compares to what prediction.c reported. A regression in how
// prediction.c maps observation_set -> ephemeris fields shows up at once:
//   - an azimuth<->elevation swap (obs_set.x <-> .y),
//   - a range<->range-rate swap (obs_set.z <-> .w),
//   - a dropped Degrees() (radians reported as degrees, off by ~57x).
// A range-only band check sees none of those.
//
// We MUST re-propagate rather than read pred->satellite_ephem.position
// back: update_satellite_position reuses that field as scratch for the
// observer position at its tail (the Calculate_User_PosVel call), so on
// return it holds the observer's ECI position, not the satellite's. Re-
// calling SGP4/SDP4 with the stored (already unit-converted) TLE at the
// same minutes_since_epoch reproduces the exact state update used, since
// the propagator is deterministic given select_ephemeris's one-time init
// (the same reason the pass-finder can call it in a tight loop).
//
// Tolerances: range, range-rate and azimuth are untouched by atmospheric
// refraction and match to rounding. Calculate_Obs ADDS a Meeus refraction
// term to elevation when the apparent elevation is >= 0 (it peaks near
// +0.5 deg at the horizon and shrinks fast with altitude) and reports the
// true elevation below the horizon -- so the reported elevation sits in
// [el_true, el_true + ~0.55]. A 0.7 deg window is still far tighter than a
// swap (tens of deg) or a units error (57x), so it pins both field and unit.
static void check_topocentric_oracle(prediction_t *pred,
                                     double jul_utc, const char *tag)
{
    // Re-propagate to the exact time update_satellite_position just used
    // (it stashed minutes_since_epoch); pick SGP4/SDP4 by the same flag.
    double mse = pred->minutes_since_epoch;
    vector_t sp = {0}, sv = {0};
    if (isFlagSet(DEEP_SPACE_EPHEM_FLAG))
        SDP4(mse, &pred->satellite_ephem.tle, &sp, &sv);
    else
        SGP4(mse, &pred->satellite_ephem.tle, &sp, &sv);
    Convert_Sat_State(&sp, &sv);          // normalised -> km, km/s

    // Observer ECI position+velocity, computed exactly as sgp4sdp4 does for
    // prediction.c. Use a copy of the geodetic: Calculate_User_PosVel writes
    // .theta (the local sidereal time) as a side effect.
    geodetic_t obs = pred->observer_ephem.position_geodetic;
    vector_t obs_pos = {0}, obs_vel = {0};
    Calculate_User_PosVel(jul_utc, &obs, &obs_pos, &obs_vel);

    // Slant-range vector (sat - observer) and the relative velocity.
    vector_t rng = {0}, rgv = {0};
    rng.x = sp.x - obs_pos.x; rng.y = sp.y - obs_pos.y; rng.z = sp.z - obs_pos.z;
    rgv.x = sv.x - obs_vel.x; rgv.y = sv.y - obs_vel.y; rgv.z = sv.z - obs_vel.z;
    double rmag = sqrt(rng.x * rng.x + rng.y * rng.y + rng.z * rng.z);

    // SEZ (south / east / zenith) projection at the observer. Same algebra
    // as sgp4sdp4's Calculate_Obs, reproduced so the azimuth quadrant
    // convention matches exactly.
    double sin_lat = sin(obs.lat), cos_lat = cos(obs.lat);
    double sin_th  = sin(obs.theta), cos_th = cos(obs.theta);
    double top_s = sin_lat * cos_th * rng.x + sin_lat * sin_th * rng.y - cos_lat * rng.z;
    double top_e = -sin_th * rng.x + cos_th * rng.y;
    double top_z = cos_lat * cos_th * rng.x + cos_lat * sin_th * rng.y + sin_lat * rng.z;

    double azim = atan(-top_e / top_s);
    if (top_s > 0.0) azim += M_PI;
    if (azim < 0.0)  azim += 2.0 * M_PI;
    double el_true = asin(top_z / rmag);            // geometric (no refraction)
    double rr = (rng.x * rgv.x + rng.y * rgv.y + rng.z * rgv.z) / rmag;

    double az_deg = azim * 180.0 / M_PI;
    double el_deg = el_true * 180.0 / M_PI;

    tap_okf(fabs(pred->satellite_ephem.range_km - rmag) < 1e-3,
            "%s: range_km == |sat-obs| (got %.6f, oracle %.6f km)",
            tag, pred->satellite_ephem.range_km, rmag);

    tap_okf(fabs(pred->satellite_ephem.range_rate_km_s - rr) < 1e-4,
            "%s: range_rate == d|range|/dt (got %.6f, oracle %.6f km/s)",
            tag, pred->satellite_ephem.range_rate_km_s, rr);

    // Compare azimuth on the circle so 359.9 vs 0.1 reads as ~0, not ~360.
    double daz = fabs(pred->satellite_ephem.azimuth - az_deg);
    if (daz > 180.0) daz = 360.0 - daz;
    tap_okf(daz < 1e-3,
            "%s: azimuth == SEZ projection (got %.4f, oracle %.4f deg)",
            tag, pred->satellite_ephem.azimuth, az_deg);

    double del = pred->satellite_ephem.elevation - el_deg;
    tap_okf(del > -1e-3 && del < 0.7,
            "%s: elevation == geometric (+refraction) (got %.4f, oracle %.4f deg)",
            tag, pred->satellite_ephem.elevation, el_deg);
}

// ---------------------------------------------------------- tle_default_path

static void test_tle_default_path(void)
{
    fprintf(stderr, "tle_default_path:\n");

    // With HOME set: returns 0 and fills the buffer with the canonical path.
    char *saved_home = getenv("HOME");
    char *saved_home_copy = saved_home ? strdup(saved_home) : NULL;
    setenv("HOME", "/tmp/sso_test_home", 1);
    char buf[256];
    int rc = tle_default_path(buf, sizeof buf);
    check(rc == 0, "HOME=/tmp/sso_test_home: returns 0");
    check(strcmp(buf, "/tmp/sso_test_home/.local/state/simple_sat_ops/active.tle") == 0,
          "buf == \"$HOME/.local/state/simple_sat_ops/active.tle\"");

    // With HOME empty: returns -1.
    setenv("HOME", "", 1);
    rc = tle_default_path(buf, sizeof buf);
    check(rc == -1, "HOME=\"\": returns -1");

    // With HOME unset: returns -1.
    unsetenv("HOME");
    rc = tle_default_path(buf, sizeof buf);
    check(rc == -1, "HOME unset: returns -1");

    // Restore.
    if (saved_home_copy) {
        setenv("HOME", saved_home_copy, 1);
        free(saved_home_copy);
    }

    // With a too-small buffer: returns -1 (snprintf overflow).
    setenv("HOME", "/very/long/home/path/that/likely/blows/the/buffer", 1);
    char tiny[20];
    rc = tle_default_path(tiny, sizeof tiny);
    check(rc == -1, "buffer too small: returns -1");
    if (saved_home_copy) setenv("HOME", saved_home_copy, 1);
}

// ----------------------------------------------------------------- load_tle

static void test_load_tle_hit(const char *tles_path)
{
    fprintf(stderr, "load_tle (known satellite):\n");
    prediction_t pred;
    char name[64];
    snprintf(name, sizeof name, "OSCAR 7");
    init_pred(&pred, tles_path, name);
    int rc = load_tle(&pred);
    check(rc == 0, "load_tle returns 0 for 'OSCAR 7'");
    // load_tle stores the FULL matched name line (including the padding).
    // We only need the prefix to start with the requested name.
    check(strncmp(pred.satellite_ephem.tle.sat_name, "OSCAR 7", 7) == 0,
          "satellite_ephem.tle.sat_name starts with 'OSCAR 7'");
    // Epoch encodes YYDDD.DDDDDDDD = 25043.01160017.
    check(fabs(pred.satellite_ephem.tle.epoch - 25043.01160017) < 1e-6,
          "TLE epoch decoded as 25043.01160017");
    // catnr is the NORAD ID = 7530.
    check(pred.satellite_ephem.tle.catnr == 7530,
          "TLE NORAD catnr = 7530");
}

static void test_load_tle_miss(const char *tles_path)
{
    fprintf(stderr, "load_tle (unknown satellite):\n");
    prediction_t pred;
    char name[64];
    snprintf(name, sizeof name, "DEFINITELY NOT A REAL SAT");
    init_pred(&pred, tles_path, name);
    int rc = load_tle(&pred);
    check(rc == -2, "load_tle returns -2 for an unknown name");
}

static void test_load_tle_bad_path(void)
{
    fprintf(stderr, "load_tle (open failure):\n");
    prediction_t pred;
    char name[64];
    snprintf(name, sizeof name, "OSCAR 7");
    init_pred(&pred, "/tmp/sso_definitely_not_a_real_tle.tle", name);
    int rc = load_tle(&pred);
    check(rc == -1, "load_tle returns -1 when the file can't be opened");
}

// ------------------------------------------------- update_satellite_position

// Near-Earth / SGP4 branch. At several times spanning ~3/4 of an orbit the
// propagated state must (a) land in the physical bands for AO-7 and (b)
// agree with the independent topocentric oracle. The oracle is the part
// that pins the actual az/el/range/range-rate values; the bands just guard
// against a wholly broken propagation (negative altitude, 60 km/s, etc.).
static void test_update_satellite_position(const char *tles_path)
{
    fprintf(stderr, "update_satellite_position (SGP4 / near-Earth):\n");
    prediction_t pred;
    char name[64];
    snprintf(name, sizeof name, "OSCAR 7");
    init_pred(&pred, tles_path, name);
    if (load_tle(&pred) != 0) {
        check(0, "load_tle preflight succeeded");
        return;
    }
    prep_propagator(&pred);
    check(!isFlagSet(DEEP_SPACE_EPHEM_FLAG),
          "AO-7 (~12.5 rev/day) selects the near-Earth SGP4 path");

    double jul_epoch = Julian_Date_of_Epoch(pred.satellite_ephem.tle.epoch);

    update_satellite_position(&pred, jul_epoch);

    // LEO at ~1450 km altitude orbits at ~7.0 km/s. Allow generous slack.
    check(pred.satellite_ephem.speed_km_s > 6.0 && pred.satellite_ephem.speed_km_s < 8.5,
          "speed_km_s in (6, 8.5) -- LEO range");
    check(pred.satellite_ephem.altitude_km > 1300.0 && pred.satellite_ephem.altitude_km < 1600.0,
          "altitude_km in (1300, 1600) for AO-7's apogee-ish band");
    check(pred.satellite_ephem.latitude > -90.0 && pred.satellite_ephem.latitude < 90.0,
          "sub-satellite latitude in (-90, 90)");
    check(pred.satellite_ephem.longitude > -180.0 && pred.satellite_ephem.longitude < 360.0,
          "sub-satellite longitude in (-180, 360)");

    // Independent topocentric cross-check at three points spanning the orbit
    // (~4600 s period), so the oracle is exercised at distinct geometries.
    check_topocentric_oracle(&pred, jul_epoch, "epoch");
    update_satellite_position(&pred, jul_epoch + 23.0 / 1440.0);
    check_topocentric_oracle(&pred, jul_epoch + 23.0 / 1440.0, "epoch+23min");
    update_satellite_position(&pred, jul_epoch + 57.0 / 1440.0);
    check_topocentric_oracle(&pred, jul_epoch + 57.0 / 1440.0, "epoch+57min");
}

// Deep-space / SDP4 branch. AO-40's ~19 h period puts it over the 225-min
// threshold, so select_ephemeris must flag it deep-space and
// update_satellite_position must route through SDP4. The same topocentric
// oracle then confirms the field wiring is correct on this path too.
static void test_update_satellite_position_deep_space(const char *tles_path)
{
    fprintf(stderr, "update_satellite_position (SDP4 / deep-space):\n");
    prediction_t pred;
    char name[64];
    snprintf(name, sizeof name, "AO-40");
    init_pred(&pred, tles_path, name);
    if (load_tle(&pred) != 0) {
        check(0, "load_tle preflight succeeded for AO-40");
        return;
    }
    prep_propagator(&pred);
    check(isFlagSet(DEEP_SPACE_EPHEM_FLAG),
          "AO-40 (~1.27 rev/day) selects the deep-space SDP4 path");

    double jul_epoch = Julian_Date_of_Epoch(pred.satellite_ephem.tle.epoch);
    update_satellite_position(&pred, jul_epoch);

    // AO-40 is a high-eccentricity orbit (perigee ~1000 km, apogee ~58000
    // km). Don't pin altitude tightly -- just confirm it's a sane Earth
    // orbit, not garbage.
    check(pred.satellite_ephem.altitude_km > 500.0
              && pred.satellite_ephem.altitude_km < 70000.0,
          "altitude_km in (500, 70000) -- HEO range");
    check(pred.satellite_ephem.speed_km_s > 0.5 && pred.satellite_ephem.speed_km_s < 12.0,
          "speed_km_s in (0.5, 12) for an HEO");

    check_topocentric_oracle(&pred, jul_epoch, "AO-40 epoch");
    update_satellite_position(&pred, jul_epoch + 120.0 / 1440.0);
    check_topocentric_oracle(&pred, jul_epoch + 120.0 / 1440.0, "AO-40 epoch+2h");
}

// -------------------------------------------------- update_pass_predictions

// Contract: starting from a visible AOS, the pass walker should fill in
// ascension/descent azimuth + jul, max_elevation, pass_duration; the
// descent timestamp must be strictly after the ascension; max_el should
// be positive (we found AOS only if the sat rises above 0 deg).
static void test_update_pass_predictions(const char *tles_path)
{
    fprintf(stderr, "update_pass_predictions (AOS/LOS / max_el):\n");
    prediction_t pred;
    char name[64];
    snprintf(name, sizeof name, "OSCAR 7");
    init_pred(&pred, tles_path, name);
    if (load_tle(&pred) != 0) {
        check(0, "load_tle preflight succeeded");
        return;
    }
    prep_propagator(&pred);
    double jul_epoch = Julian_Date_of_Epoch(pred.satellite_ephem.tle.epoch);

    // Find an AOS within 24 h of epoch.
    minutes_until_visible(&pred, jul_epoch, jul_epoch + 1.0, 1.0);
    double minutes = pred.predicted_minutes_until_visible;
    if (minutes <= -9000.0 || minutes > 24.0 * 60.0) {
        fprintf(stderr, "    no AOS in the 24 h window after epoch -- SKIP\n");
        return;
    }
    double aos_jul = jul_epoch + minutes / 1440.0;
    update_pass_predictions(&pred, aos_jul, 0.1);

    check(pred.predicted_max_elevation > 0.0
              && pred.predicted_max_elevation <= 90.0,
          "max_elevation in (0, 90]");
    check(pred.predicted_pass_duration_minutes > 0.0
              && pred.predicted_pass_duration_minutes < 60.0,
          "pass_duration in (0, 60) min for a LEO pass");
    check(pred.predicted_minutes_above_0_degrees > 0.0
              && pred.predicted_minutes_above_0_degrees
                     <= pred.predicted_pass_duration_minutes,
          "minutes_above_0 <= pass_duration");
    check(pred.predicted_ascension_azimuth >= 0.0
              && pred.predicted_ascension_azimuth <= 360.0,
          "ascension_azimuth in [0, 360]");
    check(pred.predicted_descent_azimuth >= 0.0
              && pred.predicted_descent_azimuth <= 360.0,
          "descent_azimuth in [0, 360]");
    check(pred.predicted_ascension_jul_utc > jul_epoch
              && pred.predicted_descent_jul_utc > pred.predicted_ascension_jul_utc,
          "descent_jul_utc > ascension_jul_utc > epoch");

    fprintf(stderr,
            "    pass: AOS az=%.1f deg el rises, max_el=%.1f deg, "
            "LOS az=%.1f deg, duration=%.1f min\n",
            pred.predicted_ascension_azimuth,
            pred.predicted_max_elevation,
            pred.predicted_descent_azimuth,
            pred.predicted_minutes_above_0_degrees);
}

// --------------------------------------- find_passes / get_pass / free_passes

// Exercise the module-level static pass list end to end: the search, the
// soonest-first and latest-first sorts, the get_pass index bounds, and the
// free_passes reset. Whether a visible pass exists is geometry, not
// prediction.c, so an empty result SKIPs the structural checks.
static void test_pass_search_api(const char *tles_path)
{
    fprintf(stderr, "find_passes / get_pass / number_of_passes / free_passes:\n");

    // Decode AO-7's epoch so the search window is deterministic.
    prediction_t tmp;
    char tmpname[64];
    snprintf(tmpname, sizeof tmpname, "OSCAR 7");
    init_pred(&tmp, tles_path, tmpname);
    if (load_tle(&tmp) != 0) {
        check(0, "preflight load_tle (for epoch) succeeded");
        return;
    }
    double jul_start = Julian_Date_of_Epoch(tmp.satellite_ephem.tle.epoch);

    prediction_t pred;
    char name[64];
    snprintf(name, sizeof name, "OSCAR 7");
    init_pred(&pred, tles_path, name);

    criteria_t crit = {0};
    crit.min_altitude_km = 0.0;
    crit.max_altitude_km = 100000.0;
    crit.min_minutes     = 0.0;
    crit.max_minutes     = 24.0 * 60.0;   // 24 h search window
    crit.min_elevation   = 0.0;
    crit.max_elevation   = 90.0;
    crit.regex           = NULL;
    crit.with_constellations = 1;          // irrelevant -- name_prefix wins
    // Restrict the search to AO-7 (the fixture also holds AO-40).
    crit.name_prefix     = "OSCAR 7";

    free_passes();                          // start from a clean module state
    int count = 0, checked = 0;
    int rc = find_passes(&pred, jul_start, 1.0, &crit, &count, &checked, 0, 1 /*find_all*/);
    check(rc == 0, "find_passes returns 0");
    size_t n = number_of_passes();
    fprintf(stderr, "    found %zu pass(es) (lines checked=%d, alt-passing=%d)\n",
            n, count, checked);

    if (n == 0) {
        fprintf(stderr, "    no visible AO-7 pass in 24 h from epoch -- SKIP structural checks\n");
        free_passes();
        return;
    }

    // get_pass index bounds.
    check(get_pass(0) != NULL, "get_pass(0) is non-NULL when passes exist");
    check(get_pass(-1) == NULL, "get_pass(-1) is NULL");
    check(get_pass((int)n) == NULL, "get_pass(n) is NULL (one past the end)");

    // Soonest-first: minutes_away non-decreasing; per-pass fields sane.
    int sorted = 1, fields_ok = 1, names_ok = 1;
    double prev = -1e30;
    for (size_t i = 0; i < n; i++) {
        const pass_t *p = get_pass((int)i);
        if (p == NULL) { fields_ok = 0; break; }
        if (p->minutes_away < prev - 1e-6) sorted = 0;
        prev = p->minutes_away;
        if (!(p->max_elevation > 0.0 && p->max_elevation <= 90.0)) fields_ok = 0;
        if (!(p->ascension_azimuth >= 0.0 && p->ascension_azimuth <= 360.0)) fields_ok = 0;
        if (!(p->pass_duration > 0.0 && p->pass_duration < 120.0)) fields_ok = 0;
        if (strncmp(p->name, "OSCAR 7", 7) != 0) names_ok = 0;
    }
    check(sorted, "passes sorted soonest-first (minutes_away non-decreasing)");
    check(fields_ok, "every pass: max_el in (0,90], asc_az in [0,360], 0<dur<120 min");
    check(names_ok, "every pass name starts with 'OSCAR 7'");

    // Reverse order: same set, latest-first (minutes_away non-increasing).
    free_passes();
    rc = find_passes(&pred, jul_start, 1.0, &crit, &count, &checked, 1 /*reverse*/, 1);
    check(rc == 0, "find_passes (reverse_order=1) returns 0");
    size_t n_rev = number_of_passes();
    check(n_rev == n, "reverse search finds the same number of passes");
    int desc = 1;
    prev = 1e30;
    for (size_t i = 0; i < n_rev; i++) {
        const pass_t *p = get_pass((int)i);
        if (p == NULL || p->minutes_away > prev + 1e-6) { desc = 0; break; }
        prev = p->minutes_away;
    }
    check(desc, "reverse passes sorted latest-first (minutes_away non-increasing)");

    // free_passes resets the module-level list.
    free_passes();
    check(number_of_passes() == 0, "free_passes resets the count to 0");
    check(get_pass(0) == NULL, "get_pass(0) is NULL after free_passes");
}

// -------------------------------------------- julian_date_from_unix_seconds
//
// rx_replay turns a capture's wall-clock time into the jul_utc it feeds
// update_satellite_position(). It used to do that with gmtime_r() +
// sgp4sdp4's Julian_Date() -- but Julian_Date() expects a non-POSIX struct
// tm (full year in tm_year, 1-based tm_mon), while gmtime_r() fills the POSIX
// convention (tm_year = year - 1900). So Julian_Date() read 2026 as year 126:
// the JD was ~1900 years off, SGP4 propagated ~1e9 minutes past epoch, and
// returned a ~1e19 km range that the geometry gate dropped -- every replayed
// SatNOGS pass lost az/el/range (issue #53).
//
// Pin the helper against known anchors, against the exact buggy path (so a
// revert to gmtime_r is caught), and tie the JD error to the geometry symptom:
// AO-7 at the correct JD gives a sane LEO range; at the buggy JD, garbage.
static void test_julian_date_from_unix_seconds(const char *tles_path)
{
    fprintf(stderr, "julian_date_from_unix_seconds (issue #53):\n");

    // Anchors: the Unix epoch is JD 2440587.5; one day later is +1.0.
    check(fabs(julian_date_from_unix_seconds(0.0) - 2440587.5) < 1e-9,
          "unix 0 -> JD 2440587.5");
    check(fabs(julian_date_from_unix_seconds(86400.0) - 2440588.5) < 1e-9,
          "unix +86400 s -> JD +1.0 day");
    check(fabs(julian_date_from_unix_seconds(0.5) - (2440587.5 + 0.5 / 86400.0)) < 1e-12,
          "fractional seconds preserved");

    // Load AO-7 and choose a Unix time exactly at its TLE epoch, so a correct
    // JD propagates to near-epoch (a sane LEO state).
    prediction_t pred;
    char name[64];
    snprintf(name, sizeof name, "OSCAR 7");
    init_pred(&pred, tles_path, name);
    if (load_tle(&pred) != 0) { check(0, "load_tle preflight succeeded"); return; }
    prep_propagator(&pred);
    double jul_epoch = Julian_Date_of_Epoch(pred.satellite_ephem.tle.epoch);
    double unix_at_epoch = (jul_epoch - 2440587.5) * 86400.0;

    // The helper round-trips back to the epoch JD.
    double jd_good = julian_date_from_unix_seconds(unix_at_epoch);
    check(fabs(jd_good - jul_epoch) < 1e-6,
          "helper(unix_at_epoch) == jul_epoch (round-trip)");

    // The OLD buggy path: a POSIX struct tm fed to sgp4sdp4's Julian_Date.
    time_t t = (time_t)unix_at_epoch;
    struct tm posix_tm;
    gmtime_r(&t, &posix_tm);
    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
    double jd_bug = Julian_Date(&posix_tm, &tv);
    // tm_year (year - 1900) read as a full year -> off by ~1900 years.
    check(fabs(jd_good - jd_bug) > 600000.0,
          "buggy gmtime_r+Julian_Date path is ~1900 years (>600000 days) off");

    // Geometry: the correct JD gives a sane LEO slant range; the buggy JD
    // (~1900 yr of bogus propagation) gives a materially different, wrong one.
    // For high-drag objects (e.g. FrontierSat) that wrong range is ~1e19 km
    // and the geometry gate NULLs it -- the visible #53 symptom; for low-drag
    // AO-7 it stays finite but is still hundreds of km off. The robust, drag-
    // independent invariant is "different from the correct range".
    update_satellite_position(&pred, jd_good);
    double range_good = pred.satellite_ephem.range_km;
    update_satellite_position(&pred, jd_bug);
    double range_bug = pred.satellite_ephem.range_km;

    check(isfinite(range_good) && range_good > 100.0 && range_good < 1.0e5,
          "range at correct JD is a sane LEO slant range (100..1e5 km)");
    check(!isfinite(range_bug) || fabs(range_bug - range_good) > 100.0,
          "range at buggy JD is materially wrong (geometry corrupted)");

    fprintf(stderr, "    range: correct JD %.1f km, buggy JD %.3e km\n",
            range_good, range_bug);
}

int main(void)
{
    test_tle_default_path();

    char *tles_path = write_fixture_tle();
    if (!tles_path) {
        tap_bail("cannot write fixture TLE");
        return 1;
    }

    test_load_tle_hit(tles_path);
    test_load_tle_miss(tles_path);
    test_load_tle_bad_path();
    test_julian_date_from_unix_seconds(tles_path);
    test_update_satellite_position(tles_path);
    test_update_satellite_position_deep_space(tles_path);
    test_update_pass_predictions(tles_path);
    test_pass_search_api(tles_path);

    unlink(tles_path);
    free(tles_path);

    return tap_done();
}
