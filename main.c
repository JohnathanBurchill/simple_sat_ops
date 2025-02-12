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
#include <string.h>
#include <time.h>
#include <hamlib/rig.h>
#include <hamlib/rotator.h>
#include <sgp4sdp4.h>
#include <ncurses.h>

/* Satellite communication frequencies */
#define VHF_UPLINK_FREQ   145800000.000
#define UHF_DOWNLINK_FREQ 435300000.000

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

    row++;
    mvprintw(row++, col, "%15s : %s (%s)", "satellite", state->satellite.tle.sat_name, state->satellite.tle.idesg);

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
    mvprintw(row++, col, "%15s : %.1f°", "max elevation", state->predicted_max_elevation);
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

    if (state->in_pass) {
        mvprintw(row++, col, "%15s : %s", "status", "** IN PASS **");
        if (state->tracking) {
            printw(" (TRACKING)");
        } else {
            printw(" (NOT tracking)");
        }
    } else {
        mvprintw(row++, col, "%15s : %s", "status", "** NOT in pass **");
    }
    clrtoeol();
    if (state->have_rig) {
        rig_get_freq(state->rig, RIG_VFO_A, (freq_t *)&state->rig_vfo_a_frequency);
        rig_get_freq(state->rig, RIG_VFO_B, (freq_t *)&state->rig_vfo_b_frequency);

        mvprintw(row++, col, "%15s : %s", "transceiver", state->rig->caps->model_name);
        mvprintw(row++, col, "%15s : %.1f Hz", "VFO A", state->rig_vfo_a_frequency);
        mvprintw(row++, col, "%15s : %.1f Hz", "VFO B", state->rig_vfo_b_frequency);
    } else {
        mvprintw(row++, col, "%15s : %s", "transceiver", "* not initialized *");
    }
    if (state->have_rotator) {
        mvprintw(row++, col, "%15s : %s", "rotator", state->rot->caps->model_name);
        azimuth_t rot_az = 0.0;
        elevation_t rot_el = 0.0;
        rot_get_position(state->rot, &rot_az, &rot_el);
        mvprintw(row++, col, "%15s : %.2f°", "elevation", (double)rot_el);
        mvprintw(row++, col, "%15s : %.2f°", "azimuth", (double)rot_az);
    } else {
        mvprintw(row++, col, "%15s : %s", "rotator", "* not initialized *");
    }

    row++;
    mvprintw(row++, col, "%15s : %.1f° N", "latitude", state->satellite.latitude);
    mvprintw(row++, col, "%15s : %.1f° E", "longitude", state->satellite.longitude);
    mvprintw(row++, col, "%15s : %.2f km", "altitude", state->satellite.altitude_km);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.2f km/s", "speed", state->satellite.speed_km_s);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.2f°", "elevation", state->satellite.elevation);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.2f°", "azimuth", state->satellite.azimuth);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.1f km", "range", state->satellite.range_km);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.2f km/s", "range rate", state->satellite.range_rate_km_s);
    clrtoeol();
    row++;
    mvprintw(row++, col, "%15s : %.1f Hz", "UPLINK on", state->doppler_uplink_frequency);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.1f Hz", "DOWNLINK on", state->doppler_downlink_frequency);
    clrtoeol();

    *print_row = row;
}

int main(int argc, char **argv) 
{

    int ret = 0;
    state_t state = {0};
    state.predicted_max_elevation = -180.0;
    state.doppler_uplink_frequency = VHF_UPLINK_FREQ;
    state.doppler_downlink_frequency = UHF_DOWNLINK_FREQ;

    int status = 0;
    double tracking_prep_time_minutes = 5.0;
    double site_latitude = RAO_LATITUDE;
    double site_longitude = RAO_LONGITUDE;
    double site_altitude = RAO_ALTITUDE;


    for (int i = 0; i < argc; i++) {
        if (strncmp("--verbose=", argv[i], 10) == 0) {
            state.n_options++; 
            if (strlen(argv[i]) < 11) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state.verbose_level = atoi(argv[i] + 10);
        } else if (strcmp("--no-rig", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rig = 1;
        } else if (strcmp("--no-rig", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rig = 1;
        } else if (strcmp("--no-rotator", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rotator = 1;
        } else if (strcmp("--no-hardware", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rig = 1;
            state.run_without_rotator = 1;
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

    /* Open TLE file */
    state.tles_filename = argv[1];
    state.satellite.name = argv[2];

    /* Parse TLE data */
    int tle_status = load_tle(&state);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.satellite.tle);


    if (!state.run_without_rig) {
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
        if (rig_open(state.rig) != RIG_OK) {
            fprintf(stderr, "Error opening rig. Is it plugged into USB and powered?\n");
            if (!state.run_without_rig) {
                rig_cleanup(state.rig);
                rot_cleanup(state.rot);
                return 1;
            }
        } else {
            state.have_rig = 1;
        }
    }
    if (!state.run_without_rotator) {
        state.rot = rot_init(ROT_MODEL_SPID_ROT2PROG);
        if (!state.rot) {
            fprintf(stderr, "Failed to initialize rotator support.\n");
            return 1;
        }
        if (rot_open(state.rot) != RIG_OK) {
            fprintf(stderr, "error opening rotator. is it plugged into usb and powered?\n");
            if (!state.run_without_rotator) {
                rig_cleanup(state.rig);
                rot_cleanup(state.rot);
                return 1;
            }
        } else {
            state.have_rotator = 1;
        }
    }

    /* Set up observer location */
    state.observer.position_geodetic.lat = site_latitude * M_PI / 180.0;
    state.observer.position_geodetic.lon = site_longitude * M_PI / 180.0;
    state.observer.position_geodetic.alt = site_altitude / 1000.0;

    /* Tracking loop */
    double jul_idle_start = 0;  // Time when the satellite was last tracked

    struct tm utc;
    struct timeval tv;
    double jul_utc = 0.0;

    init_window();

    char key = '\0';

    int row = 0;
    state.running = 1;

    // Queue info 

    char *next_in_queue_name = NULL;
    double next_in_queue_minutes_away = -1e10; 

    while (state.running) {
        // Refresh
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state, jul_utc);

        /* Calculate Doppler shift */
        update_doppler_shifted_frequencies(&state, VHF_UPLINK_FREQ, UHF_DOWNLINK_FREQ);

        // TODO check for passes that reach a minimum elevation
        if (state.predicted_minutes_until_visible < tracking_prep_time_minutes) {
            if (!state.in_pass) {
                state.in_pass = 1;
            }
            if (state.have_rotator && !state.tracking) {
                state.tracking = 1;
            }

            /* Point rotator to Az/El */
            if (state.have_rotator && !state.run_without_rotator) {
                if ((ret = rot_set_position(state.rot, state.satellite.azimuth, state.satellite.azimuth)) != RIG_OK) {
                    fprintf(stderr, "Error setting rotor position: %s\n", rigerror(ret));
                }
            }

            /* Set rig frequencies with Doppler correction */
            if (state.have_rig && !state.run_without_rig) {
                rig_set_vfo(state.rig, RIG_VFO_A);
                ret = rig_set_freq(state.rig, RIG_VFO_A, state.doppler_uplink_frequency);
                if (ret != RIG_OK) {
                    fprintf(stderr, "Error setting uplink frequency on VFO A: %s\n", rigerror(ret));
                }
                rig_set_vfo(state.rig, RIG_VFO_B);
                ret = rig_set_freq(state.rig, RIG_VFO_B, state.doppler_downlink_frequency);
                if (ret != RIG_OK) {
                    fprintf(stderr, "Error setting downlink frequency on VFO B: %s\n", rigerror(ret));
                }
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

        if (next_in_queue_minutes_away > 0) {
            int n = strlen(next_in_queue_name);
            while (n > 0 && isspace(next_in_queue_name[n-1])) {
                n--;
            }
            next_in_queue_name[n] = '\0';
            row++;
            mvprintw(row++, 0, "%15s : %s (%.0f minutes)", "Next in queue", next_in_queue_name, next_in_queue_minutes_away);
        }

        mvprintw(0, 0, "%s", "");
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
