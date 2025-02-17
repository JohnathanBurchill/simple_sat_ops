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

#include "antenna_rotator.h"
#include "radio.h"
#include "state.h"
#include "prediction.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// Satellite communication defaults
#define UPLINK_FREQ_MHZ   145.150000
#define DOWNLINK_FREQ_MHZ 436.150000

// Update the radio's frequencies when the change 
// associated with Doppler shift exceeds this amount
#define DOPPLER_SHIFT_RESOLUTION_KHZ 0.1 

// Antenna rotator max angle from target 
#define MAX_DELTA_AZIMUTH_DEGREES 3.0
#define MAX_DELTA_ELEVATION_DEGREES 3.0

#define WARN_DAYS_SINCE_EPOCH 1.0
#define MAX_MINUTES_TO_PREDICT ((7 * 1440))

#define UPDATE_INTERVAL_MICROSEC 500000
#define KEYBOARD_UNLOCK_DURATION_TICKS (1000000 / UPDATE_INTERVAL_MICROSEC * 5)

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
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    curs_set(0);

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
    mvprintw(row++, col, "%15s   %d %s %04d %02d:%02d:%02d UTC", "date", utc.tm_mday, months[utc.tm_mon-1], utc.tm_year, utc.tm_hour, utc.tm_min, utc.tm_sec);
    clrtoeol();

    row++;
    mvprintw(row++, col, "%15s   %s (%s)", "satellite", state->satellite.tle.sat_name, state->satellite.tle.idesg);
    clrtoeol();
    if (state->minutes_since_epoch/1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attron(COLOR_PAIR(2));
    } 
    mvprintw(row++, col, "%15s   %0.1f days", "epoch age", state->minutes_since_epoch / 1440.0);
    if (state->minutes_since_epoch/1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attroff(COLOR_PAIR(2));
    } 
    clrtoeol();

    if (state->in_pass) {
        mvprintw(row++, col, "%15s   %s", "status", "** IN PASS **");
        if (state->tracking) {
            printw(" (TRACKING)");
        } else {
            attron(COLOR_PAIR(1));
            printw(" (NOT tracking)");
            attroff(COLOR_PAIR(1));
        }
    } else {
        mvprintw(row++, col, "%15s   %s", "status", "** NOT in pass **");
    }
    clrtoeol();

    minutes_until_visible(state, 1.0, MAX_MINUTES_TO_PREDICT);
    if (fabs(state->predicted_minutes_until_visible) < 1) {
        minutes_until_visible(state, 1./120.0, 2.0);
    } else if (fabs(state->predicted_minutes_until_visible) < 10) {
        minutes_until_visible(state, 0.1, 20.0);
    }
    if (state->predicted_minutes_until_visible > 0) {
        if (state->predicted_minutes_until_visible < 1) {
            mvprintw(row++, col, "%15s   ", "next pass in");
            attron(COLOR_PAIR(2));
            printw("%.0f seconds", floor(state->predicted_minutes_until_visible * 60.0));
            attroff(COLOR_PAIR(2));
        } else if (state->predicted_minutes_until_visible < 10) {
            mvprintw(row++, col, "%15s   %.1f minutes", "next pass in", state->predicted_minutes_until_visible);
        } else {
            mvprintw(row++, col, "%15s   %.0f minutes", "next pass in", state->predicted_minutes_until_visible);
        }
        clrtoeol();
        update_pass_predictions(state, jul_utc + state->predicted_minutes_until_visible / 1440.0, 0.1);
        mvprintw(row++, col, "%15s   %.1f minutes", "duration", state->predicted_minutes_above_0_degrees);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "el>30", state->predicted_minutes_above_30_degrees);
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   ", "elapsed time");
        attron(COLOR_PAIR(3));
        if (fabs(state->predicted_minutes_until_visible) < 1) {
            printw("%.0f seconds", floor(-state->predicted_minutes_until_visible * 60.0));
        } else {
            printw("%.1f minutes", -state->predicted_minutes_until_visible);
        }
        if (state->predicted_max_elevation == -180.0) {
            update_pass_predictions(state, jul_utc - state->predicted_minutes_until_visible / 1440.0, 0.1);
        }
        attroff(COLOR_PAIR(3));
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "duration", state->predicted_minutes_above_0_degrees);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "el>30", state->predicted_minutes_above_30_degrees);
        clrtoeol();
    }
    mvprintw(row++, col, "%15s   %.1f deg", "max elevation", state->predicted_max_elevation);
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

    if (state->have_radio) {
        mvprintw(row++, col, "%15s   %.6f MHz", "UPLINK", state->radio.doppler_uplink_frequency / 1e6);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.6f MHz", "VFO Main", state->radio.vfo_main_actual_frequency / 1e6);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.6f MHz", "DOWNLINK", state->radio.doppler_downlink_frequency / 1e6);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.6f MHz", "VFO Sub", state->radio.vfo_sub_actual_frequency / 1e6);
        clrtoeol();
        row++;
    } else {
        mvprintw(row++, col, "%15s   %s", "transceiver", "* not initialized *");
        clrtoeol();
    }
    if (state->have_antenna_rotator) {
        double azimuth = 0.0;
        double elevation = 0.0;
        antenna_rotator_command(&state->antenna_rotator, ANTENNA_ROTATOR_STATUS, &azimuth, &elevation);
        mvprintw(row++, col, "%15s   %.1f deg", "target azimuth", state->antenna_rotator.target_azimuth);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "azimuth", azimuth);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "target elevation", state->antenna_rotator.target_elevation);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "elevation", elevation);
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   %s", "antenna rotator", "* not initialized *");
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

    mvprintw(row++, col, "%15s   %.2f deg", "azimuth", state->satellite.azimuth);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f deg", "elevation", state->satellite.elevation);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km", "altitude", state->satellite.altitude_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg N", "latitude", state->satellite.latitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg E", "longitude", state->satellite.longitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "speed", state->satellite.speed_km_s);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f km", "range", state->satellite.range_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "range rate", state->satellite.range_rate_km_s);
    clrtoeol();

    *print_row = row;
}

int apply_args(state_t *state, int argc, char **argv, double jul_utc);

int main(int argc, char **argv) 
{

    int ret = 0;
    state_t state = {0};
    state.predicted_max_elevation = -180.0;
    
    int status = 0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);
    apply_args(&state, argc, argv, jul_utc);

    /* Parse TLE data */
    int tle_status = load_tle(&state);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.satellite.tle);

    int radio_result = 0;
    if (state.run_with_radio) {
        state.radio.is_required = 1;
        radio_result = radio_init(&state.radio);
        if (radio_result != RADIO_OK) {
            fprintf(stderr, "Error initializing radio\n");
            return EXIT_FAILURE;
        }
        state.have_radio = 1;
    }

    int antenna_rotator_result = 0;
    if (state.run_with_antenna_rotator) {
        state.antenna_rotator.is_required = 1;
        antenna_rotator_result = antenna_rotator_init(&state.antenna_rotator);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error initializing antenna rotator\n");
            return EXIT_FAILURE;
        }
        // Check that we can control the antenna position
        antenna_rotator_result = antenna_rotator_command(&state.antenna_rotator, ANTENNA_ROTATOR_STATUS, NULL, NULL);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error commanding the antenna rotator\n");
            state.have_antenna_rotator = 0;
        } else {
            state.have_antenna_rotator = 1;
        }
    }

    /* Tracking loop */
    double jul_idle_start = 0;  // Time when the satellite was last tracked

    init_window();

    char key = '\0';

    int row = 0;
    int col = 2;
    state.running = 1;

    // Queue info 

    char *next_in_queue_name = NULL;
    double next_in_queue_minutes_away = -1e10; 

    double current_uplink_frequency = state.radio.nominal_uplink_frequency;
    double current_downlink_frequency = state.radio.nominal_downlink_frequency;
    double doppler_delta_uplink = 0.0;
    double doppler_delta_downlink = 0.0;
    double doppler_max_delta = DOPPLER_SHIFT_RESOLUTION_KHZ * 1000.0;

    double azimuth = 0.0;
    double elevation = 0.0;
    double delta_az = 0.0;
    double delta_el = 0.0;

    int antenna_should_be_controlled = state.run_with_antenna_rotator && state.have_antenna_rotator;
    int antenna_is_under_control = antenna_should_be_controlled;

    int keyboard_unlocked = 0;
    int keyboard_info_row = 20;

    mvprintw(keyboard_info_row++, 3, "%s", "T - Track satellite");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "s - Stop tracking satellite");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "r - Reset to az=0 el=0");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "q - quit");
    clrtoeol();

    while (state.running) {
        // Refresh
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state, jul_utc);

        /* Calculate Doppler shift */
        update_doppler_shifted_frequencies(&state, state.radio.nominal_uplink_frequency, state.radio.nominal_downlink_frequency);
        doppler_delta_uplink = fabs(state.radio.doppler_uplink_frequency - current_uplink_frequency);
        doppler_delta_downlink = fabs(state.radio.doppler_downlink_frequency - current_downlink_frequency);

        // TODO check for passes that reach a minimum elevation
        if (state.predicted_minutes_until_visible < state.tracking_prep_time_minutes) {
            if (!state.in_pass) {
                state.in_pass = 1;
            }
            if (antenna_should_be_controlled && !state.tracking) {
                state.tracking = 1;
            }

            // Point antenna at satellite
            // TODO remove lag bias by anticipating direction
            if (state.tracking && antenna_is_under_control) {
                delta_az = state.satellite.azimuth - state.antenna_rotator.target_azimuth;
                delta_el = state.satellite.elevation - state.antenna_rotator.target_elevation;

                if (fabs(delta_az) >= MAX_DELTA_AZIMUTH_DEGREES || fabs(delta_el) >= MAX_DELTA_ELEVATION_DEGREES) {
                    state.antenna_rotator.target_azimuth = state.satellite.azimuth;
                    state.antenna_rotator.target_elevation = state.satellite.elevation;
                    azimuth = state.antenna_rotator.target_azimuth;
                    elevation = state.antenna_rotator.target_elevation;
                    antenna_rotator_result = antenna_rotator_command(&state.antenna_rotator, ANTENNA_ROTATOR_SET, &azimuth, &elevation);
                    if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
                        fprintf(stderr, "Error setting antenna rotator position\n");
                    }
                }
            }

            /* Set radio frequencies with Doppler correction*/
            if (state.have_radio && state.run_with_radio && ((doppler_delta_uplink > doppler_max_delta) || (doppler_delta_downlink > doppler_max_delta))) {
                current_uplink_frequency = state.radio.doppler_uplink_frequency;
                current_downlink_frequency = state.radio.doppler_downlink_frequency;
                radio_set_vfo(&state.radio, VFOMain);
                ret = radio_set_frequency(&state.radio, state.radio.doppler_uplink_frequency);
                if (ret != RADIO_OK) {
                    fprintf(stderr, "Error setting uplink frequency on VFO Main\n");
                }
                state.radio.vfo_main_actual_frequency = radio_get_frequency(&state.radio);
                radio_set_vfo(&state.radio, VFOSub);
                ret = radio_set_frequency(&state.radio, state.radio.doppler_downlink_frequency);
                if (ret != RADIO_OK) {
                    fprintf(stderr, "Error setting downlink frequency on VFO Sub\n");
                }
                state.radio.vfo_sub_actual_frequency = radio_get_frequency(&state.radio);
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
        row = 1;
        col = 1;
        report_predictions(&state, jul_utc, &row, col);

        // Update status
        row ++;
        report_status(&state, &row, col);
        row = 5;
        col = 50;
        report_position(&state, &row, col);

        if (next_in_queue_minutes_away > 0) {
            int n = strlen(next_in_queue_name);
            while (n > 0 && isspace(next_in_queue_name[n-1])) {
                n--;
            }
            next_in_queue_name[n] = '\0';
            row++;
            mvprintw(row++, 0, "%15s   %s (%.0f minutes)", "Next in queue", next_in_queue_name, next_in_queue_minutes_away);
            clrtoeol();
        }

        clrtoeol();

        key = getch(); 
        if (key == 'K') {
            keyboard_unlocked = KEYBOARD_UNLOCK_DURATION_TICKS;
        } else if (keyboard_unlocked) {
            switch(key) {
                case 'q':
                    state.running = 0;
                    keyboard_unlocked = 0;
                    break;
                case 'T':
                    antenna_is_under_control = antenna_should_be_controlled;
                    keyboard_unlocked = 0;
                    break;
                case 's':
                    antenna_is_under_control = 0;
                    keyboard_unlocked = 0;
                    break;
                case 'r':
                    antenna_is_under_control = 0;
                    state.antenna_rotator.target_azimuth = 0.0;
                    state.antenna_rotator.target_elevation = 0.0;
                    azimuth = state.antenna_rotator.target_azimuth;
                    elevation = state.antenna_rotator.target_elevation;
                    antenna_rotator_result = antenna_rotator_command(&state.antenna_rotator, ANTENNA_ROTATOR_SET, &azimuth, &elevation);
                    if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
                        fprintf(stderr, "Error setting antenna rotator position\n");
                    }
                    keyboard_unlocked = 0;
                    break;
                default:
                    break;
            }
        }
        if (keyboard_unlocked > 0) {
            --keyboard_unlocked;
        }

        mvprintw(keyboard_info_row, 3, "%s : %s", "Keyboard", keyboard_unlocked ? "unlocked" : "LOCKED");
        clrtoeol();
        refresh();

        // Sleep for a short interval 
        if (state.running) {
            usleep(UPDATE_INTERVAL_MICROSEC);
        }

    }

    if (next_in_queue_name) {
        free(next_in_queue_name);
    }

    if (state.auto_sat) {
        free_passes();
    }

    endwin();

    /* Cleanup */
    if (state.radio.connected) {
        radio_set_satellite_mode(&state.radio, 0);
        radio_disconnect(&state.radio);
    }
    if (state.antenna_rotator.connected) {
        // if (state.have_rotator) {
        //     rot_set_position(state.rot, 0, 0);
        //     rot_close(state.rot);
        // }
        // rot_cleanup(state.rot);
    }

    return 0;
}


int apply_args(state_t *state, int argc, char **argv, double jul_utc)
{
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;
    double min_altitude_km = 0.0;
    double max_altitude_km = 1000.0;
    double min_minutes_away = 30.0 / 60.0;
    double max_minutes_away = 30.0;
    double min_elevation = 0.0;
    double max_elevation = 90.0;
    int with_constellations = 0;
    state->tracking_prep_time_minutes = TRACKING_PREP_TIME_MINUTES;

    state->radio.nominal_uplink_frequency = UPLINK_FREQ_MHZ * 1e6;
    state->radio.nominal_downlink_frequency = DOWNLINK_FREQ_MHZ * 1e6;

    state->run_with_radio = 0;
    state->radio.device_filename = "/dev/ttyUSB1";
    state->radio.serial_speed = 9600;

    state->run_with_antenna_rotator = 0;
    state->antenna_rotator.device_filename = "/dev/ttyUSB0";
    state->antenna_rotator.serial_speed = 960;

    for (int i = 0; i < argc; i++) {

        if (strncmp("--verbose=", argv[i], 10) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 11) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->verbose_level = atoi(argv[i] + 10);
        } else if (strcmp("--with-radio", argv[i]) == 0) {
            state->n_options++;
            state->run_with_radio = 1;
        } else if (strcmp("--with-rotator", argv[i]) == 0) {
            state->n_options++;
            state->run_with_antenna_rotator = 1;
        } else if (strcmp("--with-hardware", argv[i]) == 0) {
            state->n_options++;
            state->run_with_radio = 1;
            state->run_with_antenna_rotator = 1;
        } else if (strncmp("--radio-device=", argv[i], 15) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 16) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]); 
                return EXIT_FAILURE;
            } 
            state->radio.device_filename = argv[i] + 15;
        } else if (strncmp("--radio-serial-speed=", argv[i], 21) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 22) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]); 
                return EXIT_FAILURE;
            } 
            state->radio.serial_speed = atoi(argv[i] + 21);
        } else if (strncmp("--uplink-freq-mhz=", argv[i], 18) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->radio.nominal_uplink_frequency = atof(argv[i] + 18) * 1e6;
        } else if (strncmp("--downlink-freq-mhz=", argv[i], 20) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 21) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->radio.nominal_downlink_frequency = atof(argv[i] + 20) * 1e6;
        } else if (strncmp("--lat=", argv[i], 6) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_latitude = atof(argv[i] + 6);
        } else if (strncmp("--lon=", argv[i], 6) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_longitude = atof(argv[i] + 6);
        } else if (strncmp("--alt=", argv[i], 6) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            site_altitude = atof(argv[i] + 6);
        } else if (strcmp("--include-constellations", argv[i]) == 0) {
            state->n_options++;
            with_constellations = 1;
        } else if (strncmp("--min-altitude-km=", argv[i], 18) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_altitude_km = atof(argv[i] + 18);
        } else if (strncmp("--max-altitude-km=", argv[i], 18) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_altitude_km = atof(argv[i] + 18);
        } else if (strncmp("--min-elevation=", argv[i], 16) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->n_options++; 
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_elevation = atof(argv[i] + 16);
        } else if (strncmp("--min-minutes=", argv[i], 14) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            min_minutes_away = atof(argv[i] + 14);
        } else if (strncmp("--max-minutes=", argv[i], 14) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            max_minutes_away = atof(argv[i] + 14);
        } else if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0]);
            return 2;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 3;
        }

    }
    if (argc - state->n_options != 3) {
        usage(stderr, argv[0]);
        return 1;
    }

    state->radio.doppler_uplink_frequency = state->radio.nominal_uplink_frequency;
    state->radio.doppler_downlink_frequency = state->radio.nominal_downlink_frequency;

    state->observer.position_geodetic.lat = site_latitude * M_PI / 180.0;
    state->observer.position_geodetic.lon = site_longitude * M_PI / 180.0;
    state->observer.position_geodetic.alt = site_altitude / 1000.0;

    state->tles_filename = argv[1];
    state->satellite.name = argv[2];

    if (strcmp(state->satellite.name, "next") == 0) {
        state->auto_sat = 1;
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
        state_tmp.tles_filename = state->tles_filename;
        state_tmp.observer.position_geodetic.lat = state->observer.position_geodetic.lat;
        state_tmp.observer.position_geodetic.lon = state->observer.position_geodetic.lon;
        state_tmp.observer.position_geodetic.alt = state->observer.position_geodetic.alt;
        find_passes(&state_tmp, jul_utc, 0.1, &criteria, NULL, NULL, 0);
        const size_t n = number_of_passes();
        if (n == 0) {
            fprintf(stderr, "Unable to automatically find next in queue.\n");
            return 1;
        }

        const pass_t *p = get_pass(0);
        state->satellite.name = strdup(p->name);
        printf("Satellite: %s\n", state->satellite.name);
    }

    return 0;
}
