#include "state.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <hamlib/rig.h>
#include <hamlib/rotator.h>
#include <sgp4sdp4.h>
#include <ncurses.h>

/* RAO site observer location in Priddis, SW of Calgary */
#define RAO_LATITUDE  50.8812  // Latitude in degrees
#define RAO_LONGITUDE -114.2914 // Longitude in degrees
#define RAO_ALTITUDE  1250.0   // Altitude in meters

/* Satellite communication frequencies */
#define VHF_UPLINK_FREQ   145800000.000
#define UHF_DOWNLINK_FREQ 435300000.000

void usage(FILE *dest, const char *name) 
{
    fprintf(dest, "usage: %s <tles_file> <satellite_id>\n", name);
    return;
}

void update_satellite_position(state_t *state, double jul_utc)
{
    // jul times are days
    state->jul_epoch = Julian_Date_of_Epoch(state->satellite.tle.epoch);
    state->minutes_since_epoch = (jul_utc - state->jul_epoch) * 1440.0;

    /* Propagate satellite position */
    /* Call NORAD routines according to deep-space flag */
    if(isFlagSet(DEEP_SPACE_EPHEM_FLAG)) {
        SDP4(state->minutes_since_epoch, &state->satellite.tle, &state->satellite.position, &state->satellite.velocity);
    } else {
        SGP4(state->minutes_since_epoch, &state->satellite.tle, &state->satellite.position, &state->satellite.velocity);
    }

    // pos and vel in km, km/s
    Convert_Sat_State(&state->satellite.position, &state->satellite.velocity);
    Magnitude(&state->satellite.velocity);
    Calculate_Obs(jul_utc, &state->satellite.position, &state->satellite.velocity, &state->observer.position_geodetic, &state->satellite.observation_set);
    Calculate_LatLonAlt(jul_utc, &state->satellite.position, &state->satellite.position_geodetic);
    state->satellite.azimuth = Degrees(state->satellite.observation_set.x);
    state->satellite.elevation = Degrees(state->satellite.observation_set.y);
    state->satellite.range_km = state->satellite.observation_set.z;
    state->satellite.range_rate_km_s = state->satellite.observation_set.w;
    state->satellite.latitude = Degrees(state->satellite.position_geodetic.lat);
    state->satellite.longitude = Degrees(state->satellite.position_geodetic.lon);
    state->satellite.altitude_km = state->satellite.position_geodetic.alt;
    state->satellite.speed_km_s = state->satellite.velocity.w;
    // Assumes ground station (not in a car, drone, balloon, plane, satellite, etc.)
    Calculate_User_PosVel(state->minutes_since_epoch, &state->observer.position_geodetic, &state->satellite.position, &state->observer.velocity);

    return;
}

void update_doppler_shifted_frequencies(state_t *state)
{
    Vec_Sub(&state->satellite.velocity, &state->observer.velocity, &state->observer_satellite_relative_velocity);
    state->observer_satellite_relative_speed = Dot(&state->observer_satellite_relative_velocity, &state->satellite.position) / state->satellite.position.w;  // Radial velocity
    state->doppler_uplink_frequency = VHF_UPLINK_FREQ * (1 + state->observer_satellite_relative_speed / 299792.458);  // Speed of light in km/s
    state->doppler_downlink_frequency = UHF_DOWNLINK_FREQ * (1 + state->observer_satellite_relative_speed / 299792.458);

    return;
}

// Overwrites the current satellite position
void update_pass_predictions(state_t *external_state, double jul_utc_start, double delta_t_minutes)
{
    state_t state = {0};
    memcpy(&state, external_state, sizeof *external_state);
    double jul_utc = jul_utc_start; 
    // Sets prediction to start of pass
    update_satellite_position(&state, jul_utc);
    double current_elevation = state.satellite.elevation;

    double max_elevation = current_elevation;
    double pass_duration = 0.0;
    double minutes_above_0_degrees = 0.0;
    double minutes_above_30_degrees = 0.0;
    while (current_elevation > -5.0) {
        pass_duration += delta_t_minutes;
        if (current_elevation > 0.0) {
            minutes_above_0_degrees += delta_t_minutes;
        }
        if (current_elevation > 30.0) {
            minutes_above_30_degrees += delta_t_minutes;
        }
        update_satellite_position(&state, jul_utc + pass_duration / 1440.0);
        current_elevation = state.satellite.elevation;
        if (max_elevation < current_elevation) {
            max_elevation = current_elevation;
        }
    }
    external_state->predicted_pass_duration_minutes = pass_duration;
    external_state->predicted_minutes_above_0_degrees = minutes_above_0_degrees;
    external_state->predicted_minutes_above_30_degrees = minutes_above_30_degrees;
    external_state->predicted_max_elevation = max_elevation;

    return;
}

void minutes_until_visible(state_t *external_state, double delta_t_minutes)
{
    state_t state = {0};
    memcpy(&state, external_state, sizeof *external_state);
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc_start = Julian_Date(&utc, &tv);
    double jul_utc = jul_utc_start; 
    update_satellite_position(&state, jul_utc);
    double elevation = state.satellite.elevation;
    if (elevation < 0) {
        // How long until it becomes visible?
        while (elevation < 0) {
            jul_utc += delta_t_minutes / 1440.0;
            update_satellite_position(&state, jul_utc);
            elevation = state.satellite.elevation;
        }
    } else {
        // How long since it became visible?
        while (elevation > 0) {
            jul_utc -= delta_t_minutes / 1440.0;
            update_satellite_position(&state, jul_utc);
            elevation = state.satellite.elevation;
        }
    }

    external_state->predicted_minutes_until_visible = (jul_utc - jul_utc_start) * 1440.0;

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

    minutes_until_visible(state, 1.0);
    if (fabs(state->predicted_minutes_until_visible) < 1) {
        minutes_until_visible(state, 1./120.0);
    } else if (fabs(state->predicted_minutes_until_visible) < 10) {
        minutes_until_visible(state, 0.1);
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
        mvprintw(row++, col, "%15s : %s", "transceiver", state->rig->caps->model_name);
        channel_t ch = {0};
        rig_get_channel(state->rig, RIG_VFO_A, &ch, 0);
        mvprintw(row++, col, "%15s : %.3f MHz", "VFO A", ch.freq);
        rig_get_channel(state->rig, RIG_VFO_B, &ch, 0);
        mvprintw(row++, col, "%15s : %.3f MHz", "VFO B", ch.freq);
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
    mvprintw(row++, col, "%15s : %.3f MHz", "UPLINK on", state->doppler_uplink_frequency);
    clrtoeol();
    mvprintw(row++, col, "%15s : %.3f MHz", "DOWNLINK on", state->doppler_downlink_frequency);
    clrtoeol();

    *print_row = row;
}

// Returns the first match on state->satellite.name
int load_tle(state_t *state)
{
    FILE *file = fopen(state->tles_filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error opening %s\n", state->tles_filename);
        return -1;
    }
    
    // 2 69-character lines plus a nul terminator
    char tle[139] = {0};
    char name[128] = {0}; 
    int found_satellite = 0;

    while (fgets(name, 128, file)) {
        // Remove newline
        name[strlen(name) - 1] = '\0';
        if (strncmp(state->satellite.name, name, strlen(state->satellite.name)) == 0) {
            // Errors caught in TLE check
            // Read 70 characters, including the newline
            fgets(tle, 71, file);
            // Read 69 characterers
            fgets(tle + 69, 70, file);
            tle[138] = '\0';
            found_satellite = 1;
            break;
        }
    }
    if (!found_satellite) {
        fprintf(stderr, "Satellite '%s' not found in %s\n", state->satellite.name, state->tles_filename);
        return -2;
    }

    if (!Good_Elements(tle)) {
        fprintf(stderr, "Invalid TLE\n");
        return -3;
    }
    Convert_Satellite_Data(tle, &state->satellite.tle);

    return 0;

}

int main(int argc, char **argv) 
{
    state_t state = {0};
    state.predicted_max_elevation = -180.0;
    state.doppler_uplink_frequency = VHF_UPLINK_FREQ;
    state.doppler_downlink_frequency = UHF_DOWNLINK_FREQ;

    int status = 0;
    double tracking_prep_time_minutes = 5.0;

    for (int i = 0; i < argc; i++) {
        if (strcmp("--no-rig", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rig = 1;
        }
        else if (strcmp("--no-rotator", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rotator = 1;
        }
        else if (strcmp("--no-hardware", argv[i]) == 0) {
            state.n_options++;
            state.run_without_rig = 1;
            state.run_without_rotator = 1;
        }
        else if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0]);
            return 0;
        }
        else if (strncmp("--", argv[i], 2) == 0) {
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

    /* Initialize Hamlib for rig and rotator */
    rig_set_debug(RIG_DEBUG_NONE);
    state.rig = rig_init(RIG_MODEL_IC9700);
    if (!state.rig) {
        fprintf(stderr, "Failed to initialize rig support.\n");
        return 1;
    }
    state.rot = rot_init(ROT_MODEL_GS232A);
    if (!state.rot) {
        fprintf(stderr, "Failed to initialize rotator support.\n");
        return 1;
    }

    strncpy(state.rig->state.rigport.pathname, "/dev/ttyUSB1", sizeof(state.rig->state.rigport.pathname) - 1);
    state.rig->state.rigport.pathname[sizeof(state.rig->state.rigport.pathname) - 1] = '\0';
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

    strncpy(state.rot->state.rotport.pathname, "/dev/ttyUSB0", sizeof(state.rot->state.rotport.pathname) - 1);
    state.rot->state.rotport.pathname[sizeof(state.rot->state.rotport.pathname) - 1] = '\0';
    if (rot_open(state.rot) != RIG_OK) {
        fprintf(stderr, "Error opening rotator. Is it plugged into USB and powered?\n");
        if (!state.run_without_rotator) {
            rig_cleanup(state.rig);
            rot_cleanup(state.rot);
            return 1;
        }
    } else {
        state.have_rotator = 1;
    }

    /* Set up observer location */
    geodetic_t observer_geodetic = {
        .lat = RAO_LATITUDE * M_PI / 180.0,
        .lon = RAO_LONGITUDE * M_PI / 180.0,
        .alt = RAO_ALTITUDE / 1000.0,
    };

    geodetic_t satellite_geodetic = {0};

    /* Tracking loop */
    double jul_idle_start = 0;  // Time when the satellite was last tracked
    double speed = 0.0;
    vector_t observer_set = {0};

    int ret = 0;
    struct tm utc;
    struct timeval tv;
    double jul_utc = 0.0;

    init_window();

    char key = '\0';

    int row = 0;
    state.running = 1;
    while (state.running) {
        // Refresh time
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);

        // Refresh satellite position
        update_satellite_position(&state, jul_utc);

        /* Calculate Doppler shift */
        update_doppler_shifted_frequencies(&state);

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
                if ((ret = rig_set_freq(state.rig, RIG_VFO_A, state.doppler_uplink_frequency)) != RIG_OK ||
                    (ret = rig_set_freq(state.rig, RIG_VFO_B, state.doppler_downlink_frequency)) != RIG_OK) {
                    fprintf(stderr, "Error setting rig frequency: %s\n", rigerror(ret));
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

        mvprintw(0, 0, "");
        refresh();

        key = getch(); 
        switch(key) {
            case 'q':
                state.running = 0;
                break;
            default:
                break;
        }

        // TODO Import TLE every 6 hours?

        // Sleep for a short interval 
        usleep(250000);

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
