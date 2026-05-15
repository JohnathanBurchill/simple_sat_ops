/*

    Simple Satellite Operations  utils/prediction_selftest.c

    Smoke tests for prediction.c:
      - tle_default_path: HOME-relative path expansion.
      - load_tle: known-hit, known-miss, and TLE name-match behaviour
        against a synthetic 3-line TLE fixture.
      - update_satellite_position: at TLE epoch, LEO ephemeris fields
        land in physically plausible ranges.
      - update_pass_predictions: populates ascension + descent fields
        consistently (descent_jul > ascension_jul, azimuths in
        [0, 360), elevation in (0, 90]).

    The pass-prediction test searches forward from TLE epoch for the
    first AOS visible from RAO; if none turns up in a 24-hour window
    the test reports SKIP rather than failing -- SGP4 propagation is
    deterministic given the TLE but the existence of a visible pass
    in any given day depends on the geometry, not on prediction.c.

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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sgp4sdp4.h>

static int failures = 0;

static void check(int cond, const char *what)
{
    fprintf(stderr, "  %s: %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) ++failures;
}

// Known-good 3-line TLE: OSCAR 7 (AO-7), an active LEO at ~1450 km
// altitude with a ~12.5 rev/day orbit. Frozen in here (not pulled
// from TLEs/) so the test is reproducible across TLE refreshes.
static const char *FIXTURE_TLE =
    "OSCAR 7 (AO-7)\n"
    "1 07530U 74089B   25043.01160017 -.00000030  00000+0  10217-3 0  9990\n"
    "2 07530 101.9936  46.0237 0012129 347.8272  22.1887 12.53686098299338\n";

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

// Contract: at the TLE epoch the propagated state should land in the
// physical ranges expected for a 1450 km LEO. The exact (az, el, range)
// depend on the orbit and observer geometry and are NOT asserted here
// -- this test guards against gross regressions in the SGP4 plumbing.
static void test_update_satellite_position(const char *tles_path)
{
    fprintf(stderr, "update_satellite_position (LEO sanity):\n");
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
    update_satellite_position(&pred, jul_epoch);

    // LEO at ~1450 km altitude orbits at ~7.0 km/s. Allow generous slack.
    check(pred.satellite_ephem.speed_km_s > 6.0 && pred.satellite_ephem.speed_km_s < 8.5,
          "speed_km_s in (6, 8.5) -- LEO range");
    check(pred.satellite_ephem.altitude_km > 1300.0 && pred.satellite_ephem.altitude_km < 1600.0,
          "altitude_km in (1300, 1600) for AO-7's apogee-ish band");
    check(pred.satellite_ephem.azimuth >= 0.0 && pred.satellite_ephem.azimuth <= 360.0,
          "azimuth in [0, 360]");
    check(pred.satellite_ephem.elevation >= -90.0 && pred.satellite_ephem.elevation <= 90.0,
          "elevation in [-90, 90]");
    check(pred.satellite_ephem.range_km > 0.0 && pred.satellite_ephem.range_km < 20000.0,
          "range_km in (0, 20000) km (observer-to-sat at LEO)");
    check(pred.satellite_ephem.latitude > -90.0 && pred.satellite_ephem.latitude < 90.0,
          "sub-satellite latitude in (-90, 90)");
    check(pred.satellite_ephem.longitude > -180.0 && pred.satellite_ephem.longitude < 360.0,
          "sub-satellite longitude in (-180, 360)");
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

int main(void)
{
    fprintf(stderr, "prediction_selftest: running...\n");

    test_tle_default_path();

    char *tles_path = write_fixture_tle();
    if (!tles_path) {
        fprintf(stderr, "prediction_selftest: cannot write fixture TLE\n");
        return 1;
    }

    test_load_tle_hit(tles_path);
    test_load_tle_miss(tles_path);
    test_load_tle_bad_path();
    test_update_satellite_position(tles_path);
    test_update_pass_predictions(tles_path);

    unlink(tles_path);
    free(tles_path);

    if (failures == 0) {
        fprintf(stderr, "prediction_selftest: OK\n");
        return 0;
    }
    fprintf(stderr, "prediction_selftest: %d failure(s)\n", failures);
    return 1;
}
