#include "sgp4sdp4/sgp4sdp4.h"
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
#define VHF_UPLINK_FREQ   145800000ULL  /* Uplink: 145.800 MHz */
#define UHF_DOWNLINK_FREQ 435300000ULL  /* Downlink: 435.300 MHz */

void usage(FILE *dest, const char *name) 
{
    fprintf(dest, "usage: %s <tle_file>\n", name);
    return;
}

void update_satellite_position(state_t *state, double jul_utc)
{
    // jul times are days
    state->jul_epoch = Julian_Date_of_Epoch(state->tle.epoch);
    state->minutes_since_epoch = (jul_utc - state->jul_epoch) * 1440.0;

    /* Propagate satellite position */
    /* Call NORAD routines according to deep-space flag */
    if(isFlagSet(DEEP_SPACE_EPHEM_FLAG)) {
        SDP4(state->minutes_since_epoch, &state->tle, &state->satellite.position, &state->satellite.velocity);
    } else {
        SGP4(state->minutes_since_epoch, &state->tle, &state->satellite.position, &state->satellite.velocity);
    }

    // pos and vel in km, km/s
    Convert_Sat_State(&state->satellite.position, &state->satellite.velocity);
    Magnitude(&state->satellite.velocity);
    Calculate_Obs(jul_utc, &state->satellite.position, &state->satellite.velocity, &state->observer.position_geodetic, &state->satellite.observation_set);
    Calculate_LatLonAlt(jul_utc, &state->satellite.position, &state->satellite.position_geodetic);
    state->satellite.azimuth = Degrees(state->satellite.observation_set.x);
    state->satellite.elevation = Degrees(state->satellite.observation_set.y);
    state->satellite.range_km = Degrees(state->satellite.observation_set.z);
    state->satellite.range_rate_km_s = Degrees(state->satellite.observation_set.w);
    state->satellite.latitude = Degrees(state->satellite.position_geodetic.lat);
    state->satellite.longitude = Degrees(state->satellite.position_geodetic.lon);
    state->satellite.altitude_km = Degrees(state->satellite.position_geodetic.alt);
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

double minutes_until_visible(state_t *state, double delta_t_minutes)
{
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc_start = Julian_Date(&utc, &tv);
    double jul_utc = jul_utc_start; 
    update_satellite_position(state, jul_utc);
    double elevation = state->satellite.elevation;
    if (elevation < 0) {
        // How long until it becomes visible?
        while (elevation < 0) {
            jul_utc += delta_t_minutes / 1440.0;
            update_satellite_position(state, jul_utc);
            elevation = state->satellite.elevation;
        }
    } else {
        // How long since it became visible?
        while (elevation > 0) {
            jul_utc -= delta_t_minutes / 1440.0;
            update_satellite_position(state, jul_utc);
            elevation = state->satellite.elevation;
        }
    }
    double minutes_away = (jul_utc - jul_utc_start) * 1440.0;

    return minutes_away;
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

int main(int argc, char **argv) 
{
    state_t state = {0};
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
    if (argc - state.n_options != 2) {
        usage(stderr, argv[0]);
        return 1;
    }

    /* Open TLE file */
    state.tle_filename = argv[1];

    /* Parse TLE data */
    int tle_status = Input_Tle_Set(state.tle_filename, &state.tle);
    if (tle_status) {
        fprintf(stderr, "Unable to read TLE from %s: ", state.tle_filename);
        switch (tle_status) {
            case -1:
                fprintf(stderr, "error opening file\n");
                break;
            case -2:
                fprintf(stderr, "invalid TLE format\n");
                break;
            default:
                fprintf(stderr, "unknown reason\n");
        }
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.tle);

    /* Initialize Hamlib for rig and rotator */
    rig_set_debug(RIG_DEBUG_NONE);
    RIG *rig = rig_init(RIG_MODEL_IC9700);
    if (!rig) {
        fprintf(stderr, "Failed to initialize rig support.\n");
        return 1;
    }
    ROT *rot = rot_init(ROT_MODEL_GS232A);
    if (!rot) {
        fprintf(stderr, "Failed to initialize rotator support.\n");
        return 1;
    }

    strncpy(rig->state.rigport.pathname, "/dev/ttyUSB1", sizeof(rig->state.rigport.pathname) - 1);
    rig->state.rigport.pathname[sizeof(rig->state.rigport.pathname) - 1] = '\0';
    if (rig_open(rig) != RIG_OK) {
        fprintf(stderr, "Error opening rig. Is it plugged into USB and powered?.\n");
        if (!state.run_without_rig) {
            rig_cleanup(rig);
            rot_cleanup(rot);
            return 1;
        }
    } else {
        state.have_rig = 1;
    }

    strncpy(rot->state.rotport.pathname, "/dev/ttyUSB0", sizeof(rot->state.rotport.pathname) - 1);
    rot->state.rotport.pathname[sizeof(rot->state.rotport.pathname) - 1] = '\0';
    if (rot_open(rot) != RIG_OK) {
        fprintf(stderr, "Error opening rotator. Is it plugged into USB and powered?.\n");
        if (!state.run_without_rotator) {
            rig_cleanup(rig);
            rot_cleanup(rot);
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
    int tracking = 0;  // 1 if tracking, 0 if idle
    double jul_idle_start = 0;  // Time when the satellite was last tracked
    double speed = 0.0;
    vector_t observer_set = {0};

    int ret = 0;
    struct tm utc;
    struct timeval tv;
    double jul_utc = 0.0;
    double minutes_away = 0.0;

    init_window();

    while (1) {
        
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state, jul_utc);
        mvprintw(5, 0, "EL: %6.2f  Az: %6.2f", state.satellite.elevation, state.satellite.azimuth);
        clrtoeol();
        minutes_away = minutes_until_visible(&state, 1.0);
        mvprintw(6, 0, "Minutes to next pass: ");
        if (fabs(minutes_away) < 10.0) {
            minutes_away = minutes_until_visible(&state, 0.1);
            printw("%.1f", minutes_away);
        } else {
            printw("%.0f", minutes_away);
        }
        clrtoeol();

        // TODO check for passes that reach a minimum elevation
        if (minutes_away < tracking_prep_time_minutes) { // Satellite is within 5 minutes of a pass
            if (!tracking) {
                tracking = 1;
            }

            /* Calculate Doppler shift */
            update_doppler_shifted_frequencies(&state);
            /* Point rotator to Az/El */
            if (state.have_rotator && !state.run_without_rotator) {
                if ((ret = rot_set_position(rot, state.satellite.azimuth, state.satellite.azimuth)) != RIG_OK) {
                    fprintf(stderr, "Error setting rotor position: %s\n", rigerror(ret));
                }
            }

            /* Set rig frequencies with Doppler correction */
            if (state.have_rig && !state.run_without_rig) {
                if ((ret = rig_set_freq(rig, RIG_VFO_A, state.doppler_uplink_frequency)) != RIG_OK ||
                    (ret = rig_set_freq(rig, RIG_VFO_B, state.doppler_downlink_frequency)) != RIG_OK) {
                    fprintf(stderr, "Error setting rig frequency: %s\n", rigerror(ret));
                }
            }

            jul_idle_start = 0;  // Reset idle timer
        } else { // Satellite is below -5 degrees elevation
            if (tracking) {
                tracking = 0;
                jul_idle_start = jul_utc;  // Start idle timer
            }

            /* Check if timeout has elapsed */
            if (jul_idle_start > 0 && ((jul_utc - jul_idle_start)*1440.0) > 10) {  // 10-minute timeout
                printf("Timeout reached. Exiting tracking loop.\n");
                break;
            }
        }
        if (tracking) {
            mvprintw(3, 0, "TRACKING %s", state.tle.sat_name);
        } else {
            mvprintw(3, 0, "NOT tracking %s", state.tle.sat_name);
        }
        clrtoeol();
        refresh();

        /* Sleep for a short interval (e.g., 1 second) */
        sleep(1);
    }

    endwin();

    /* Cleanup */
    if (rig) {
        rig_close(rig);
        rig_cleanup(rig);
    }
    if (rot) {
        rot_close(rot);
        rot_cleanup(rot);
    }

    return 0;
}
