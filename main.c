/*

   Simple Satellite Operations  main.c

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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>

// Satellite communication defaults
#define VHF_UPLINK_FREQ_MHZ   145.800000
#define UHF_DOWNLINK_FREQ_MHZ 435.300000

// Update the radio's frequencies when the change 
// associated with Doppler shift exceeds this amount
#define DOPPLER_SHIFT_RESOLUTION_KHZ 2.0 

#define MAX_MINUTES_TO_PREDICT ((7 * 1440))

void usage(FILE *dest, const char *name) 
{
    fprintf(dest, "usage: %s <tles_file> <satellite_id>\n", name);
    return;
}

void init_window(void)
{
    initscr(); cbreak(); noecho();
    nonl();
    timeout(0);
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);
    start_color();
    init_pair(1, COLOR_RED, COLOR_BLACK);

    return;
}

void report_predictions(state_t *state, double jul_utc, int *print_row, int print_col) 
{
    if (print_row == NULL) {
        return;
    }
    int row = *print_row;
    int col = print_col;

    struct tm utc;
    UTC_Calendar_Now(&utc, NULL);
    char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    mvprintw(row++, col, "%15s : %d %s %04d %02d:%02d:%02d UTC", "date", utc.tm_mday, months[utc.tm_mon-1], utc.tm_year, utc.tm_hour, utc.tm_min, utc.tm_sec);
    clrtoeol();

    row++;
    mvprintw(row++, col, "%15s : %s (%s)", "satellite", state->satellite.tle.sat_name, state->satellite.tle.idesg);
    clrtoeol();

    minutes_until_visible(state, 1.0, MAX_MINUTES_TO_PREDICT);
    if (fabs(state->predicted_minutes_until_visible) < 1) {
        minutes_until_visible(state, 1./120.0, 2.0);
    } else if (fabs(state->predicted_minutes_until_visible) < 10) {
        minutes_until_visible(state, 0.1, 20.0);
    }
    if (state->predicted_minutes_until_visible > 0) {
        if (state->predicted_minutes_until_visible < 1) {
            mvprintw(row++, col, "%15s : %.1f seconds", "next pass in", state->predicted_minutes_until_visible * 60.0);
        } else if (state->predicted_minutes_until_visible < 10) {
            mvprintw(row++, col, "%15s : %.1f minutes", "next pass in", state->predicted_minutes_until_visible);
        } else {
            mvprintw(row++, col, "%15s : %.0f minutes", "next pass in", state->predicted_minutes_until_visible);
        }
        clrtoeol();
        update_pass_predictions(state, jul_utc + state->predicted_minutes_until_visible / 1440.0, 0.1);
        mvprintw(row++, col, "%15s : %.1f minutes", "duration", state->predicted_minutes_above_0_degrees);
        clrtoeol();
        mvprintw(row++, col, "%15s : %.1f minutes", "el>30", state->predicted_minutes_above_30_degrees);
        clrtoeol();
    } else {
        if (fabs(state->predicted_minutes_until_visible) < 1) {
            mvprintw(row++, col, "%15s : %.1f seconds ago", "started", -state->predicted_minutes_until_visible * 60.0);
        } else {
            mvprintw(row++, col, "%15s : %.1f minutes ago", "started", -state->predicted_minutes_until_visible);
        }
        clrtoeol();
    }
    mvprintw(row++, col, "%15s : %.1f deg", "max elevation", state->predicted_max_elevation);
    clrtoeol();

    *print_row = row;

    return;
}

void report_status(state_t *state, int *print_row, int print_col)
{
    if (print_row == NULL) {
        return;
    }
    int row = *print_row;
    int col = print_col;

    mvprintw(row++, col, "%15s : %.3f MHz", "UPLINK", state->doppler_uplink_frequency / 1e6);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.3f MHz", "DOWNLINK", state->doppler_downlink_frequency / 1e6);
    clrtoeol();

    if (state->in_pass) {
        mvprintw(row++, col, "%15s : %s", "status", "** IN PASS **");
        if (state->tracking) {
            printw(" (TRACKING)");
        } else {
            attron(COLOR_PAIR(1));
            printw(" (NOT tracking)");
            attroff(COLOR_PAIR(1));
        }
    } else {
        mvprintw(row++, col, "%15s : %s", "status", "** NOT in pass **");
    }
    clrtoeol();
    if (state->have_rig) {
        mvprintw(row++, col, "%15s : %s", "transceiver", state->rig->caps->model_name);
        clrtoeol();
        mvprintw(row++, col, "%15s : %.7f Hz", "VFO Main", state->rig_vfo_main_frequency / 1e6);
        clrtoeol();
        mvprintw(row++, col, "%15s : %.7f Hz", "VFO Sub", state->rig_vfo_sub_frequency / 1e6);
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s : %s", "transceiver", "* not initialized *");
        clrtoeol();
    }
    if (state->have_rotator) {
        mvprintw(row++, col, "%15s : %s", "rotator", state->rot->caps->model_name);
        clrtoeol();
        azimuth_t rot_az = 0.0;
        elevation_t rot_el = 0.0;
        rot_get_position(state->rot, &rot_az, &rot_el);
        mvprintw(row++, col, "%15s : %.2f deg", "elevation", (double)rot_el);
        clrtoeol();
        mvprintw(row++, col, "%15s : %.2f deg", "azimuth", (double)rot_az);
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s : %s", "rotator", "* not initialized *");
        clrtoeol();
    }

    *print_row = row;
}

void report_position(state_t *state, int *print_row, int print_col)
{
    if (print_row == NULL) {
        return;
    }
    int row = *print_row;
    int col = print_col;

    mvprintw(row++, col, "%15s : %.2f deg", "elevation", state->satellite.elevation);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.2f deg", "azimuth", state->satellite.azimuth);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.2f km", "altitude", state->satellite.altitude_km);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.1f deg N", "latitude", state->satellite.latitude);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.1f deg E", "longitude", state->satellite.longitude);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.2f km/s", "speed", state->satellite.speed_km_s);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.1f km", "range", state->satellite.range_km);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.2f km/s", "range rate", state->satellite.range_rate_km_s);
    clrtoeol();

    *print_row = row;
}

int main(int argc, char **argv) 
{

    int ret = 0;
    state_t state = {0};
    state.predicted_max_elevation = -180.0;
    
    int status = 0;
    double tracking_prep_time_minutes = 5.0;
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;
    double min_altitude_km = 0.0;
    double max_altitude_km = 1000.0;
    double min_minutes_away = 0.0;
    double max_minutes_away = 120.0;
    double min_elevation = 0.0;
    double max_elevation = 90.0;
    int with_constellations = 0;
    int auto_sat = 0;

    double nominal_uplink_frequency = VHF_UPLINK_FREQ_MHZ * 1e6;
    double nominal_downlink_frequency = UHF_DOWNLINK_FREQ_MHZ * 1e6;

    state.run_with_rig = 1;
    state.run_with_rotator = 0;

    for (int i = 0; i < argc; i++) {
        if (strncmp("--verbose=", argv[i], 10) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 11) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state.verbose_level = atoi(argv[i] + 10);
        } else if (strcmp("--with-rig", argv[i]) == 0) {
            state.n_options++;
            state.run_with_rig = 1;
        } else if (strcmp("--with-rotator", argv[i]) == 0) {
            state.n_options++;
            state.run_with_rotator = 1;
        } else if (strcmp("--with-hardware", argv[i]) == 0) {
            state.n_options++;
            state.run_with_rig = 1;
            state.run_with_rotator = 1;
        } else if (strncmp("--uplink-freq-mhz=", argv[i], 18) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            nominal_uplink_frequency = atof(argv[i] + 18) * 1e6;
        } else if (strncmp("--downlink-freq-mhz=", argv[i], 20) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 21) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            nominal_downlink_frequency = atof(argv[i] + 20) * 1e6;
        } else if (strncmp("--lat=", argv[i], 6) == 0) {
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
        } else if (strncmp("--alt=", argv[i], 6) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_altitude = atof(argv[i] + 6);
        } else if (strcmp("--include-constellations", argv[i]) == 0) {
            state.n_options++;
            with_constellations = 1;
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
        } else if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 1;
        }

    }
    if (argc - state.n_options != 3) {
        usage(stderr, argv[0]);
        return 1;
    }

    state.doppler_uplink_frequency = nominal_uplink_frequency;
    state.doppler_downlink_frequency = nominal_downlink_frequency;

    state.observer.position_geodetic.lat = site_latitude * M_PI / 180.0;
    state.observer.position_geodetic.lon = site_longitude * M_PI / 180.0;
    state.observer.position_geodetic.alt = site_altitude / 1000.0;

    state.tles_filename = argv[1];
    state.satellite.name = argv[2];

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);

    if (strcmp(state.satellite.name, "auto") == 0) {
        auto_sat = 1;
        criteria_t criteria = {
            .min_altitude_km = min_altitude_km,
            .max_altitude_km = max_altitude_km,
            .min_minutes = min_minutes_away,
            .max_minutes = max_minutes_away,
            .min_elevation = min_elevation,
            .max_elevation = max_elevation,
            .regex = NULL,
            .regex_ignore_case = 0,
            .with_constellations = with_constellations,
        };
        state_t state_tmp = {0};
        state_tmp.tles_filename = state.tles_filename;
        state_tmp.observer.position_geodetic.lat = state.observer.position_geodetic.lat;
        state_tmp.observer.position_geodetic.lon = state.observer.position_geodetic.lon;
        state_tmp.observer.position_geodetic.alt = state.observer.position_geodetic.alt;
        status = find_passes(&state_tmp, jul_utc, 1.0, &criteria, NULL, NULL, 0);
        const size_t n = number_of_passes();
        if (n == 0) {
            fprintf(stderr, "Unable to automatically find next in queue.\n");
            return 1;
        }

        const pass_t *p = get_pass(0);
        state.satellite.name = strdup(p->name);
        printf("Satellite: %s\n", state.satellite.name);
    }

    /* Parse TLE data */
    int tle_status = load_tle(&state);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.satellite.tle);


    if (state.run_with_rig) {
        rig_set_debug(state.verbose_level);
        state.rig = rig_init(RIG_MODEL_IC9700);
        if (!state.rig) {
            fprintf(stderr, "Failed to initialize rig support.\n");
            return 1;
        }
        ret = rig_set_conf(state.rig, rig_token_lookup(state.rig, "rig_pathname"), "/dev/ttyUSB1");
        if (ret != RIG_OK) {
            printf("Error setting pathname\n");
            return 1;
        }
        ret = rig_set_conf(state.rig, rig_token_lookup(state.rig, "serial_speed"), "9600");
        if (ret != RIG_OK) {
            printf("Error setting serial speed\n");
            return 1;
        }
        ret = rig_set_conf(state.rig, rig_token_lookup(state.rig, "no_xchg"), "1");
        if (ret != RIG_OK) {
            printf("Error setting no_xchg = 1\n");
            return 1;
        }
        if (rig_open(state.rig) != RIG_OK) {
            fprintf(stderr, "Error opening rig. Is it plugged into USB and powered?\n");
            if (state.run_with_rig) {
                rig_cleanup(state.rig);
                rot_cleanup(state.rot);
                return 1;
            }
        } else {
            state.have_rig = 1;
            int sat_mode = 1;
            int ret = rig_set_func(state.rig, RIG_VFO_CURR, RIG_FUNC_SATMODE, sat_mode);
            if (ret != RIG_OK) {
                fprintf(stderr, "Failed to enable SATMODE: %s\n", rigerror(ret));
            }
            rig_get_freq(state.rig, RIG_VFO_MAIN, (freq_t *)&state.rig_vfo_main_frequency);
            rig_get_freq(state.rig, RIG_VFO_SUB, (freq_t *)&state.rig_vfo_sub_frequency);
        }
    }
    if (state.run_with_rotator) {
        state.rot = rot_init(ROT_MODEL_SPID_ROT2PROG);
        if (!state.rot) {
            fprintf(stderr, "Failed to initialize rotator support.\n");
            return 1;
        }
        if (rot_open(state.rot) != RIG_OK) {
            fprintf(stderr, "error opening rotator. is it plugged into usb and powered?\n");
            if (state.run_with_rotator) {
                rig_cleanup(state.rig);
                rot_cleanup(state.rot);
                return 1;
            }
        } else {
            state.have_rotator = 1;
        }
    }

    /* Tracking loop */
    double jul_idle_start = 0;  // Time when the satellite was last tracked

    init_window();

    char key = '\0';

    int row = 0;
    state.running = 1;

    // Queue info 

    char *next_in_queue_name = NULL;
    double next_in_queue_minutes_away = -1e10; 

    double current_uplink_frequency = nominal_uplink_frequency;
    double current_downlink_frequency = nominal_downlink_frequency;
    double doppler_delta_uplink = 0.0;
    double doppler_delta_downlink = 0.0;
    double doppler_max_delta = DOPPLER_SHIFT_RESOLUTION_KHZ * 1000.0;

    while (state.running) {
        // Refresh
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state, jul_utc);

        /* Calculate Doppler shift */
        update_doppler_shifted_frequencies(&state, nominal_uplink_frequency, nominal_downlink_frequency);
        doppler_delta_uplink = fabs(state.doppler_uplink_frequency - current_uplink_frequency);
        doppler_delta_downlink = fabs(state.doppler_downlink_frequency - current_downlink_frequency);

        // TODO check for passes that reach a minimum elevation
        if (state.predicted_minutes_until_visible < tracking_prep_time_minutes) {
            if (!state.in_pass) {
                state.in_pass = 1;
            }
            if (state.have_rotator && !state.tracking) {
                state.tracking = 1;
            }

            /* Point rotator to Az/El */
            if (state.have_rotator && state.run_with_rotator) {
                if ((ret = rot_set_position(state.rot, state.satellite.azimuth, state.satellite.azimuth)) != RIG_OK) {
                    fprintf(stderr, "Error setting rotor position: %s\n", rigerror(ret));
                }
            }

            /* Set rig frequencies with Doppler correction*/
            if (state.have_rig && state.run_with_rig && ((doppler_delta_uplink > doppler_max_delta) || (doppler_delta_downlink > doppler_max_delta))) {
                ret = rig_set_freq(state.rig, RIG_VFO_MAIN, state.doppler_uplink_frequency);
                if (ret != RIG_OK) {
                    fprintf(stderr, "Error setting uplink frequency on VFO Main: %s\n", rigerror(ret));
                }
                ret = rig_set_freq(state.rig, RIG_VFO_SUB, state.doppler_downlink_frequency);
                if (ret != RIG_OK) {
                    fprintf(stderr, "Error setting downlink frequency on VFO Sub: %s\n", rigerror(ret));
                }
                current_uplink_frequency = state.doppler_uplink_frequency;
                current_downlink_frequency = state.doppler_downlink_frequency;
                rig_get_freq(state.rig, RIG_VFO_MAIN, (freq_t *)&state.rig_vfo_main_frequency);
                rig_get_freq(state.rig, RIG_VFO_SUB, (freq_t *)&state.rig_vfo_sub_frequency);
                mvprintw(0, 0, "%s: current up: %f current doppler: %f", "Read frequencies", current_uplink_frequency, state.rig_vfo_main_frequency);
                clrtoeol();
            }

            jul_idle_start = 0;  // Reset idle timer
        } else {
            if (state.in_pass) {
                state.in_pass = 0;
                jul_idle_start = jul_utc;  // Start idle timer
            }
            if (state.tracking) {
                state.tracking = 0;
            }
        }

        // Update predictions
        erase();
        row = 1;
        report_predictions(&state, jul_utc, &row, 0);

        // Update status
        row ++;
        report_status(&state, &row, 0);
        row = 3;
        int col = 50;
        report_position(&state, &row, col);

        if (next_in_queue_minutes_away > 0) {
            int n = strlen(next_in_queue_name);
            while (n > 0 && isspace(next_in_queue_name[n-1])) {
                n--;
            }
            next_in_queue_name[n] = '\0';
            row++;
            mvprintw(row++, 0, "%15s : %s (%.0f minutes)", "Next in queue", next_in_queue_name, next_in_queue_minutes_away);
            clrtoeol();
        }

        mvprintw(0, 0, "%s", "");
        clrtoeol();
        refresh();

        key = getch(); 
        switch(key) {
            case 'q':
                state.running = 0;
                break;
            default:
                break;
        }

        // Sleep for a short interval 
        usleep(250000);

    }

    if (next_in_queue_name) {
        free(next_in_queue_name);
    }

    if (auto_sat) {
        free_passes();
    }

    endwin();

    /* Cleanup */
    if (state.rig) {
        rig_close(state.rig);
        rig_cleanup(state.rig);
    }
    if (state.rot) {
        rot_close(state.rot);
        rot_cleanup(state.rot);
    }

    return 0;
}
