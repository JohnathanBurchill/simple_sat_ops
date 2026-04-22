/*

    Simple Satellite Operations  next_in_queue.c

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

#include "state.h"
#include "prediction.h"
#include "oem.h"
#include "satellite_status.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <sgp4sdp4.h>

void usage(FILE *dest, const char *name, int full)
{
    fprintf(dest,
        "usage:\n"
        "  %s <min_alt_km> <max_alt_km> [<satellite_name>] [options]\n"
        "  %s --tle=<path> [<satellite_name>] [options]\n"
        "  %s --trajectory-id=<id> [options]\n"
        "\n"
        "Scan a TLE file (or a propagated SSM trajectory) for upcoming passes\n"
        "over the ground station and report the soonest (or every) match.\n"
        "Read-only; no hardware commands.\n"
        "\n"
        "Positional arguments (default-TLE form only):\n"
        "  <min_alt_km>                 Minimum orbital altitude, km\n"
        "  <max_alt_km>                 Maximum orbital altitude, km\n"
        "  <satellite_name>             Optional. Literal, case-sensitive name\n"
        "                               prefix (e.g. 'ISS (ZARYA)'). Must appear\n"
        "                               before any --options. Bypasses the\n"
        "                               constellation filter and extends the\n"
        "                               search window to one week. For regex\n"
        "                               matching use --regex= instead.\n"
        "\n"
        "Trajectory source (pick one; default = implicit TLE catalog):\n"
        "  --tle=<path>                 Path to a TLE file (2 or 3-line format).\n"
        "                               Default: $HOME/.local/state/simple_sat_ops/active.tle\n"
        "                               With this flag, positional alt limits\n"
        "                               become optional — use --min-altitude-km=\n"
        "                               / --max-altitude-km= flags if needed.\n"
        "  --trajectory-id=<id>         Fetch propagated ephemeris from `ssm\n"
        "                               trajectory <id>` and plan passes against\n"
        "                               it. Mutually exclusive with --tle=.\n"
        "                               Run `ssm trajectories` to list IDs.\n"
        "\n"
        "Output:\n"
        "  --list                       Print all matching passes (default: one)\n"
        "  --reverse                    Sort latest-first\n"
        "  --max-passes=<n>             Limit output to n passes\n"
        "  --show-radio-info            Annotate with amateur-radio info\n"
        "                               from active_radios.txt next to the TLE\n"
        "\n"
        "Pass filter:\n"
        "  --min-altitude-km=<km>       Minimum orbital altitude (default 0)\n"
        "  --max-altitude-km=<km>       Maximum orbital altitude (default unlimited)\n"
        "  --min-minutes=<n>            Minimum minutes until AOS (default 0)\n"
        "  --max-minutes=<n>            Maximum minutes until AOS (default 1440)\n"
        "  --min-elevation=<deg>        Minimum peak elevation (default 0)\n"
        "  --max-elevation=<deg>        Maximum peak elevation (default 90)\n"
        "  --minutes-offset=<n>         Advance `now` by n minutes for planning\n"
        "  --all-satellites             Include Starlink/OneWeb-style swarms\n"
        "                               (default: constellations are excluded)\n"
        "  --regex=<pattern>            Only satellites whose names match\n"
        "  --ignore-case                Case-insensitive regex match\n"
        "\n"
        "Observer location (default RAO Priddis):\n"
        "  --lat=<deg>                  Geodetic latitude\n"
        "  --lon=<deg>                  Geodetic longitude (east positive)\n"
        "  --alt=<m>                    Altitude above ellipsoid, metres\n"
        "\n"
        "Other:\n"
        "  --help                       Short help (this message)\n"
        "  --help-full                  Detailed help with examples\n",
        name, name, name);

    if (!full) return;

    fprintf(dest,
        "\n"
        "EXAMPLES\n"
        "\n"
        "  # Next satellite pass (uses default TLE at ~/.local/state/simple_sat_ops/active.tle)\n"
        "  %s 0 2000\n"
        "\n"
        "  # All passes in the next 3 hours above 30 deg elevation\n"
        "  %s 0 2000 --list \\\n"
        "      --max-minutes=180 --min-elevation=30\n"
        "\n"
        "  # ISS-class only with a specific TLE file, with amateur-radio info annotation\n"
        "  %s 300 500 --list \\\n"
        "      --tle=TLEs/amateur.tle \\\n"
        "      --regex='ISS|ZARYA' --ignore-case --show-radio-info\n"
        "\n"
        "  # Passes for a specific satellite over the next week\n"
        "  %s 0 2000 'ISS (ZARYA)' --list\n"
        "\n"
        "  # Passes against a SSM-propagated trajectory (FrontierSat-style)\n"
        "  %s --trajectory-id=$(ssm trajectories | jq -r '.[0].id') --list\n"
        "\n"
        "NOTES\n"
        "  - The tool never opens the radio or rotator; safe to run on any host.\n"
        "  - `active_radios.txt` (looked for in the same directory as the TLE)\n"
        "    is a community-maintained CSV used by --show-radio-info;\n"
        "    missing satellites are simply omitted.\n"
        "  - `--trajectory-id` needs `ssm` on PATH. The trajectory's window\n"
        "    (from `ssm trajectory-meta <id>`) caps how far ahead passes can\n"
        "    be predicted; `--max-minutes` is clamped if it exceeds the window.\n",
        name, name, name, name, name);
}

void print_radio_info(const char *name, satellite_status_t *sat_info, int n_entries);

int main(int argc, char **argv)
{
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;

    int status = 0;
    int list_all = 0;
    int max_passes = -1;
    int show_radio_info = 0;
    double min_minutes_away = 0.0;
    double max_minutes_away = 1440.0;
    int max_minutes_user_set = 0;
    double min_elevation = 0.0;
    double max_elevation = 90.0;
    state_t state = {0};
    char *regex = NULL;
    int regex_ignore_case = 0;
    int with_constellations = 0;
    int reverse = 0;
    int tle_explicit = 0;
    const char *trajectory_id = NULL;
    double min_altitude_km = 0.0;
    double max_altitude_km = 100000.0;

    int minutes_offset = 0;

    for (int i = 0; i < argc; i++) {
        if (strncmp("--lat=", argv[i], 6) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_latitude = atof(argv[i] + 6);
        } else if (strncmp("--lon=", argv[i], 6) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_longitude = atof(argv[i] + 6);
        } else if (strncmp("--max-passes=", argv[i], 13) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 14) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_passes = atoi(argv[i] + 13);
        } else if (strncmp("--alt=", argv[i], 6) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_altitude = atof(argv[i] + 6);
        } else if (strncmp("--minutes-offset=", argv[i], 17) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 18) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            minutes_offset = atoi(argv[i] + 17);
        } else if (strcmp("--list", argv[i]) == 0) {
            state.n_options++;
            list_all = 1;
        } else if (strcmp("--reverse", argv[i]) == 0) {
            state.n_options++;
            reverse = 1;
        } else if (strcmp("--all-satellites", argv[i]) == 0) {
            state.n_options++;
            with_constellations = 1;
        } else if (strncmp("--min-elevation=", argv[i], 16) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_elevation = atof(argv[i] + 16);
        } else if (strncmp("--max-elevation=", argv[i], 16) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_elevation = atof(argv[i] + 16);
        } else if (strncmp("--min-minutes=", argv[i], 14) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_minutes_away = atof(argv[i] + 14);
        } else if (strncmp("--max-minutes=", argv[i], 14) == 0) {
            state.n_options++;
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_minutes_away = atof(argv[i] + 14);
            max_minutes_user_set = 1;
        } else if (strcmp("--ignore-case", argv[i]) == 0) {
            state.n_options++;
            regex_ignore_case = 1;
        } else if (strncmp("--regex=", argv[i], 8) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 9) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            regex = argv[i] + 8;
        } else if (strcmp("--show-radio-info", argv[i]) == 0) {
            state.n_options++;
            show_radio_info = 1;
        } else if (strncmp("--tle=", argv[i], 6) == 0) {
            state.n_options++;
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state.prediction.tles_filename = argv[i] + 6;
            tle_explicit = 1;
        } else if (strncmp("--trajectory-id=", argv[i], 16) == 0) {
            state.n_options++;
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            trajectory_id = argv[i] + 16;
        } else if (strncmp("--min-altitude-km=", argv[i], 18) == 0) {
            state.n_options++;
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_altitude_km = atof(argv[i] + 18);
        } else if (strncmp("--max-altitude-km=", argv[i], 18) == 0) {
            state.n_options++;
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_altitude_km = atof(argv[i] + 18);
        } else if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0], 0);
            return 0;
        } else if (strcmp("--help-full", argv[i]) == 0) {
            usage(stdout, argv[0], 1);
            return 0;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 1;
        }
    }

    if (trajectory_id != NULL && tle_explicit) {
        fprintf(stderr, "--tle= and --trajectory-id= are mutually exclusive\n");
        return EXIT_FAILURE;
    }

    int n_args = argc - state.n_options;
    int n_pos = n_args - 1;
    char *satellite_name = NULL;

    if (trajectory_id != NULL) {
        // OEM path: no positionals (trajectory is implicit).
        if (n_pos != 0) {
            fprintf(stderr,
                    "--trajectory-id= does not accept positional arguments; "
                    "use --min-altitude-km= / --max-altitude-km= for altitude filtering.\n");
            return EXIT_FAILURE;
        }
    } else if (tle_explicit) {
        // Explicit --tle=: 0 or 1 positional (optional <satellite_name>).
        if (n_pos > 1) {
            fprintf(stderr,
                    "with --tle=, pass at most one positional (<satellite_name>). "
                    "Use --min-altitude-km= / --max-altitude-km= for altitude filtering.\n");
            return EXIT_FAILURE;
        }
        if (n_pos == 1) satellite_name = argv[1];
    } else {
        // Implicit default TLE: require 2 or 3 positionals.
        if (n_pos != 2 && n_pos != 3) {
            usage(stderr, argv[0], 0);
            return EXIT_FAILURE;
        }
        min_altitude_km = atof(argv[1]);
        max_altitude_km = atof(argv[2]);
        if (n_pos == 3) satellite_name = argv[3];
    }
    if (satellite_name != NULL && !max_minutes_user_set) {
        max_minutes_away = 1440 * 7;  // one week when filtering to a specific sat
    }
    if (trajectory_id != NULL && !max_minutes_user_set) {
        max_minutes_away = 1440 * 14;  // two weeks for trajectory planning
    }

    if (trajectory_id == NULL && state.prediction.tles_filename == NULL) {
        static char default_tle[1024];
        if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
            fprintf(stderr, "HOME unset or path too long; pass --tle=<path>\n");
            return EXIT_FAILURE;
        }
        state.prediction.tles_filename = default_tle;
    }

    // Load OEM if --trajectory-id was given. Table lives on the stack and
    // is attached to prediction; freed at exit.
    oem_table_t oem = {0};
    if (trajectory_id != NULL) {
        if (oem_load_from_ssm(trajectory_id, &oem) != 0) {
            return EXIT_FAILURE;
        }
        state.prediction.oem = &oem;
        state.prediction.satellite_ephem.name = oem.object_name[0]
                                              ? oem.object_name : (char *)"UNKNOWN";
        double window_min = (oem.stop_jul_utc - oem.start_jul_utc) * 1440.0;
        if (max_minutes_away > window_min) {
            // Beyond the propagated window we fall back to two-body
            // Kepler extrapolation inside oem_sample_at(). Good enough
            // for ground-segment scheduling; not for conjunction work.
            fprintf(stderr,
                    "note: --max-minutes=%.0f exceeds OEM window (%.1f min / %.2f h); "
                    "passes beyond the window use two-body Kepler extrapolation "
                    "(no J2, no drag — drift of minutes/day for LEO).\n",
                    max_minutes_away, window_min, window_min / 60.0);
        }
    }

    /* Set up observer location */
    state.prediction.observer_ephem.position_geodetic.lat = site_latitude * M_PI / 180.0;
    state.prediction.observer_ephem.position_geodetic.lon = site_longitude * M_PI / 180.0;
    state.prediction.observer_ephem.position_geodetic.alt = site_altitude / 1000.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);
    jul_utc += minutes_offset / 1440.0;
    struct tm utc_ref;
    Date_Time(jul_utc, &utc_ref);

    if (trajectory_id != NULL) {
        printf("Checking SSM trajectory %s (%s) for upcoming ",
               trajectory_id,
               oem.object_name[0] ? oem.object_name : "UNKNOWN");
    } else {
        printf("Checking %s for upcoming ", state.prediction.tles_filename);
    }
    if (satellite_name != NULL) {
        printf("%s", satellite_name);
    } else {
        printf("satellite");
    }
    printf(" passes within ");
    if (max_minutes_away < 120) {
        printf("%.0f minutes", max_minutes_away);
    } else if (max_minutes_away < 2880) {
        printf("%.1f hours", max_minutes_away / 60.0);
    } else {
        printf("%.1f days", max_minutes_away / 1440.0);
    }
    printf(" from %04d-%02d-%02d %02d:%02d:%02d UTC\n", utc_ref.tm_year + 1900, utc_ref.tm_mon + 1, utc_ref.tm_mday, utc_ref.tm_hour, utc_ref.tm_min, utc_ref.tm_sec);
    criteria_t criteria = {
        .min_altitude_km = min_altitude_km,
        .max_altitude_km = max_altitude_km,
        .min_minutes = min_minutes_away,
        .max_minutes = max_minutes_away,
        .min_elevation = min_elevation,
        .max_elevation = max_elevation,
        .regex = regex,
        .regex_ignore_case = regex_ignore_case,
        .with_constellations = with_constellations,
        .name_prefix = satellite_name,
    };

    int count = 0;
    int number_checked = 0;
    int find_all = (satellite_name != NULL || trajectory_id != NULL) ? 1 : 0;
    status = find_passes(&state.prediction, jul_utc, 1.0, &criteria, &count, &number_checked, reverse, find_all);
    const size_t n_passes = number_of_passes();

    // Satellite radio-info annotation. Only loaded if the user asked for
    // it (--show-radio-info) and we have a TLE directory to derive the
    // file path from. Not applicable to the OEM / trajectory path.
    satellite_status_t *sat_info = NULL;
    int n_entries = 0;
    if (show_radio_info && state.prediction.tles_filename != NULL) {
        char radios_file[FILENAME_MAX];
        const char *slash = strrchr(state.prediction.tles_filename, '/');
        if (slash != NULL) {
            int dir_len = (int)(slash - state.prediction.tles_filename);
            snprintf(radios_file, sizeof(radios_file),
                     "%.*s/active_radios.txt", dir_len, state.prediction.tles_filename);
        } else {
            snprintf(radios_file, sizeof(radios_file), "active_radios.txt");
        }
        status = parse_satellite_status_file(radios_file, &sat_info, &n_entries);
    }


    if (n_passes > 0) {
        const pass_t *p = NULL;
        if (max_passes == -1) {
            max_passes = n_passes;
        }
        if (list_all || satellite_name != NULL) {
            if (!reverse) {
                if (satellite_name == NULL) {
                    printf("Found %lu upcoming passes from a total of %d satellites\n", n_passes, count);
                }
                printf("%26s  %8s %8s %8s %9s %9s %9s %9s %9s %25s %9s\n", "Name", "in (min)", "dur (min)", "alt (km)", "azi (deg)", "ele (deg)", "up (MHz)", "down (MHz)", "bcn (MHz)", "mode", "status");
            }
            for (int i = 0; i < max_passes; i++) {
                p = get_pass(i);
                printf("%26s  ", p->name);
                if (p->minutes_away < 120.0) {
                    printf("%8.1f m ", p->minutes_away);
                } else if (p->minutes_away < 2880.0) {
                    printf("%8.1f h ", p->minutes_away / 60.0);
                } else {
                    printf("%8.1f d ", p->minutes_away / 1440.0);
                }
                printf("%9.1f %7.1f %9.1f %9.1f", p->pass_duration, p->max_altitude, p->ascension_azimuth, p->max_elevation);
                if (satellite_name == NULL && show_radio_info == 1) {
                    print_radio_info(p->name, sat_info, n_entries);
                }
                printf("\n");
            }
            if (reverse) {
                printf("%26s  %8s %8s %8s %9s %9s %9s %9s %9s %25s %9s\n", "Name", "in (min)", "dur (min)", "alt (km)", "azi (deg)", "ele (deg)", "up (MHz)", "down (MHz)", "bcn (MHz)", "mode", "status");
                if (satellite_name != NULL) {
                    printf("Found %lu upcoming passes for %s\n", n_passes, satellite_name);
                } else {
                    printf("Found %lu upcoming passes from a total of %d satellites\n", n_passes, count);
                }
            }
        } else {
            printf("Searched %d satellites\n", count);
            p = get_pass(0);
            printf("%26s in %.1f minutes from %04d-%02d-%02d %02d:%02d:%02d UTC at azimuth %.1f deg. for %.1f minutes reaching elevation %.1f deg.\n", p->name, p->minutes_away, utc_ref.tm_year + 1900, utc_ref.tm_mon + 1, utc_ref.tm_mday, utc_ref.tm_hour, utc_ref.tm_min, utc_ref.tm_sec, p->ascension_azimuth, p->pass_duration, p->max_elevation);
        }
    } else {
        printf("No passes found\n");
    }

    free_passes();
    oem_free(&oem);

    return 0;
}

void print_radio_info(const char *name, satellite_status_t *sat_info, int n_entries)
{
    for (int s = 0; s < n_entries; ++s) {
        if (strcmp(name, sat_info[s].name) == 0) {
            if (strlen(sat_info[s].f_uplink_mhz) > 0) {
                printf(" %9s", sat_info[s].f_uplink_mhz);
            } else {
                printf(" %9s", "-");
            }
            if (strlen(sat_info[s].f_downlink_mhz) > 0) {
                printf(" %9s", sat_info[s].f_downlink_mhz);
            } else {
                printf(" %9s", "-");
            }
            if (strlen(sat_info[s].f_beacon_mhz) > 0) {
                printf(" %9s", sat_info[s].f_beacon_mhz);
            } else {
                printf(" %9s", "-");
            }
            if (strlen(sat_info[s].mode) > 0) {
                printf(" %25s", sat_info[s].mode);
            } else {
                printf(" %25s", "-");
            }
            if (strlen(sat_info[s].status) > 0) {
                printf(" %9s", sat_info[s].status);
            } else {
                printf(" %9s", "-");
            }
            break;
        }
    }
}
