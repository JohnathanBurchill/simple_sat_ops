/*

    Simple Satellite Operations  lifetime.c

    Copyright (C) 2025  Johnathan K Burchill

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

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sgp4sdp4.h>

// Returns the first match on prediction->satellite_ephem.name
double lifetime(prediction_t *prediction, double jul_utc_start, double delta_t_minutes, double max_years, double min_alt_km)
{
    int status = load_tle(prediction);
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&prediction->satellite_ephem.tle);
    double jul_utc = jul_utc_start;
    update_satellite_position(prediction, jul_utc);
    double years = 0.0;

    char filename[FILENAME_MAX] = {0};
    snprintf(filename, FILENAME_MAX, "/tmp/lifetime_%s.dat", prediction->satellite_ephem.name);
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

void usage(FILE *dest, const char *name, int full)
{
    fprintf(dest,
        "usage: %s <satellite_name> <max_years> [--tle=<path>]\n"
        "\n"
        "Toy orbit-decay estimator. Propagates the SGP4/SDP4 state forward in\n"
        "time and reports how long until the satellite drops below 100 km.\n"
        "The TLE's empirical drag term is used; do not treat the result as\n"
        "an engineering-grade lifetime prediction.\n"
        "\n"
        "Positional arguments:\n"
        "  <satellite_name>             Name prefix to match in the TLE\n"
        "  <max_years>                  Stop propagating after this many years\n"
        "\n"
        "TLE source:\n"
        "  --tle=<path>                 Path to a TLE file (2 or 3-line format).\n"
        "                               Default: $HOME/.local/state/simple_sat_ops/active.tle\n"
        "\n"
        "Output:\n"
        "  Prints `Years above 100.0 km: <years>` to stdout.\n"
        "  Writes /tmp/lifetime_<name>.dat with time,altitude samples.\n"
        "\n"
        "Other:\n"
        "  --help                       Short help (this message)\n"
        "  --help-full                  Detailed help with example and caveats\n",
        name);

    if (!full) return;

    fprintf(dest,
        "\n"
        "EXAMPLE\n"
        "\n"
        "  %s 'ISS (ZARYA)' 20 --tle=TLEs/amateur.tle\n"
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
        "models (NRLMSISE-00 etc.) and updated drag observations.\n",
        name);
}

int main(int argc, char **argv)
{
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;

    int status = 0;
    prediction_t prediction = {0};

    int n_options = 0;

    for (int i = 0; i < argc; i++) {
        if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0], 0);
            return 0;
        } else if (strcmp("--help-full", argv[i]) == 0) {
            usage(stdout, argv[0], 1);
            return 0;
        } else if (strncmp("--tle=", argv[i], 6) == 0) {
            n_options++;
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            prediction.tles_filename = argv[i] + 6;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 1;
        }
    }

    if (argc - n_options != 3) {
        usage(stderr, argv[0], 0);
        return EXIT_FAILURE;
    }

    if (prediction.tles_filename == NULL) {
        static char default_tle[1024];
        if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
            fprintf(stderr, "HOME unset or path too long; pass --tle=<path>\n");
            return EXIT_FAILURE;
        }
        prediction.tles_filename = default_tle;
    }
    prediction.satellite_ephem.name = argv[1];
    double max_years = atof(argv[2]);
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

