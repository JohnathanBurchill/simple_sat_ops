/*

    Simple Satellite Operations  lifetime.c

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

#include "argparse.h"
#include "prediction.h"
#include "tle_csv.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sgp4sdp4.h>

// Returns the first match on prediction->satellite_ephem.name
double lifetime(prediction_t *prediction, double jul_utc_start, double delta_t_minutes, double max_years, double min_alt_km)
{
    (void) load_tle(prediction);
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&prediction->satellite_ephem.tle);
    double jul_utc = jul_utc_start;
    update_satellite_position(prediction, jul_utc);
    double years = 0.0;

    // Sanitize the satellite name before it lands in a /tmp path: a name with
    // '/' or '..' would otherwise escape the intended file. Keep alnum / . / -
    // and map everything else to '_'.
    char safe[64] = {0};
    const char *nm = prediction->satellite_ephem.name ? prediction->satellite_ephem.name : "sat";
    size_t j = 0;
    for (size_t i = 0; nm[i] != '\0' && j + 1 < sizeof safe; ++i) {
        unsigned char ch = (unsigned char) nm[i];
        safe[j++] = (isalnum(ch) || ch == '.' || ch == '-') ? (char) ch : '_';
    }
    if (j == 0) safe[0] = '_';
    char filename[FILENAME_MAX] = {0};
    snprintf(filename, FILENAME_MAX, "/tmp/lifetime_%s.dat", safe);
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        fprintf(stderr, "Unable to open %s for writing\n", filename);
        return -1;
    }

    while (years < max_years && prediction->satellite_ephem.altitude_km > min_alt_km) {
        update_satellite_position(prediction, jul_utc);
        fprintf(file, "%.6f %6.2f\n", years, prediction->satellite_ephem.altitude_km);
        jul_utc += delta_t_minutes / 1440.0;
        // Approx, sufficient for this problem
        years = (jul_utc - jul_utc_start) / 365.25;
    }

    fflush(file);
    fclose(file);

    return years;
}

// Parsed command-line configuration. parse_args() fills this; main() copies
// the fields out into the working locals below so the propagation body is
// unchanged.
typedef struct {
    const char *satellite_name;  // positional 1: name prefix to match in the TLE
    const char *max_years_arg;   // positional 2: stop after this many years
    char *tle_path;              // --tle=<path>, run through tle_path_resolve
} lifetime_args_t;

// Option column width: the widest label below ("<satellite_name>") + a small
// margin. See src/cli/argparse.h for the parse_args convention.
#define OPTW 18

// Parse argv into *a (help == 0), or print one right-aligned help line per
// option and return (help != 0). HELP_FULL also prints the extended notes
// the old --help-full carried. Each option is one self-contained block whose
// test carries "|| help", so help mode falls through and prints them all.
static int parse_args(lifetime_args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // Positionals first so the <...> arguments list above the --options.
        // Two positionals: each claims the first as-yet-unfilled slot. The
        // !matched guard on the second keeps a single token from filling both
        // in the same pass; both still print their line in help mode.
        if ((a->satellite_name == NULL && (arg[0] != '-' || strcmp(arg, "-") == 0)) || help) {
            if (help) parse_help_line(OPTW, "<satellite_name>", "name prefix to match in the TLE");
            else a->satellite_name = arg;
            matched = 1;
        }
        if ((!matched && a->max_years_arg == NULL && (arg[0] != '-' || strcmp(arg, "-") == 0)) || help) {
            if (help) parse_help_line(OPTW, "<max_years>", "stop propagating after this many years");
            else a->max_years_arg = arg;
            matched = 1;
        }
        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "short help (this message)");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "--help-full") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help-full", "detailed help with example and caveats");
            else { parse_args(a, argc, argv, HELP_FULL); return PARSE_HELP; }
            matched = 1;
        }
        if (strncmp(arg, "--tle=", 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tle=<path>", "path to a TLE file (2 or 3-line); default $HOME/.local/state/simple_sat_ops/active.tle");
            else {
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->tle_path = tle_path_resolve(arg + 6);
            }
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "Unable to parse option '%s'\n", arg);
            return PARSE_ERROR;
        }
    }
    if (help >= HELP_FULL) {
        printf(
            "\n"
            "Toy orbit-decay estimator. Propagates the SGP4/SDP4 state forward in\n"
            "time and reports how long until the satellite drops below 100 km.\n"
            "The TLE's empirical drag term is used; do not treat the result as\n"
            "an engineering-grade lifetime prediction.\n"
            "\n"
            "OUTPUT\n"
            "\n"
            "  Prints `Years above 100.0 km: <years>` to stdout.\n"
            "  Writes /tmp/lifetime_<name>.dat with time,altitude samples.\n"
            "\n"
            "EXAMPLE\n"
            "\n"
            "  lifetime 'ISS (ZARYA)' 20 --tle=TLEs/amateur.tle\n"
            "  # then plot the result:\n"
            "  gnuplot -p -e \"plot '/tmp/lifetime_ISS (ZARYA).dat' with lines\"\n"
            "\n"
            "ACCURACY CAVEATS\n"
            "\n"
            "The SGP4/SDP4 drag model uses the BSTAR term recorded in the TLE at\n"
            "epoch. It does not account for:\n"
            "  - Changes in solar activity over the propagation window\n"
            "  - Satellite attitude or drag-area changes\n"
            "  - Orbital manoeuvres\n"
            "  - Long-term BSTAR drift (TLEs are snapshots, not time-series)\n"
            "\n"
            "Treat the result as a `what-if` sanity check. For real end-of-life\n"
            "predictions, use a dedicated propagator with atmospheric density\n"
            "models (NRLMSISE-00 etc.) and updated drag observations.\n");
    }
    return PARSE_OK;
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "lifetime")) return 0;
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;

    prediction_t prediction = {0};

    lifetime_args_t cfg = {0};
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return EXIT_FAILURE;
    }
    if (cfg.satellite_name == NULL || cfg.max_years_arg == NULL) {
        fprintf(stderr, "usage: lifetime <satellite_name> <max_years> [--tle=<path>] (try --help)\n");
        return EXIT_FAILURE;
    }

    prediction.tles_filename = cfg.tle_path;
    if (prediction.tles_filename == NULL) {
        static char default_tle[1024];
        if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
            fprintf(stderr, "HOME unset or path too long; pass --tle=<path>\n");
            return EXIT_FAILURE;
        }
        prediction.tles_filename = tle_path_resolve(default_tle);
    }
    prediction.satellite_ephem.name = (char *)cfg.satellite_name;
    double max_years = atof(cfg.max_years_arg);
    double min_alt_km = 100.0;

    /* Set up observer location */
    prediction.observer_ephem.position_geodetic.lat = site_latitude * M_PI / 180.0;
    prediction.observer_ephem.position_geodetic.lon = site_longitude * M_PI / 180.0;
    prediction.observer_ephem.position_geodetic.alt = site_altitude / 1000.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);

    double years = lifetime(&prediction, jul_utc, 1.0, max_years, min_alt_km);
    printf("Years above %.1f km: %.3f\n", min_alt_km, years);

    return 0;
}

