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

#define _GNU_SOURCE

#include "antenna_rotator.h"
#include "audio.h"
#include "radio.h"
#include "radio_device_store.h"
#include "state.h"
#include "prediction.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>

// Satellite communication defaults
#define UPLINK_FREQ_MHZ   145.150000
#define DOWNLINK_FREQ_MHZ 436.150000
#define REFERENCE_DOWNLINK_FREQ_MHZ 432.325000

// Update the radio's frequencies when the change
// associated with Doppler shift exceeds this amount.
// 0.2 kHz keeps the residual offset well inside the
// 9600 GFSK clean-eye band (~±3 kHz tolerance) even
// at the peak ~100 Hz/s slew near TCA, where the
// threshold is crossed every ~2 s — comfortable for
// the 2 Hz UI loop and the FT-991A's ~100 ms FA;
// settling time.
#define DOPPLER_SHIFT_RESOLUTION_KHZ 0.2

// Antenna rotator max angle from target 
#define MAX_DELTA_AZIMUTH_DEGREES 1.0
#define MAX_DELTA_ELEVATION_DEGREES 1.0

#define WARN_DAYS_SINCE_EPOCH 1.0
#define MAX_MINUTES_TO_PREDICT ((7 * 1440))

#define UPDATE_INTERVAL_MICROSEC 500000

void start_tracking(state_t *state);
void stop_tracking(state_t *state);
void enable_wildrose_mode(state_t *state);
int point_to_stationary_target(state_t *state, double azimuth, double elevation);
void update_doppler_shifted_frequencies(state_t *state, double uplink_freq, double downlink_freq);

static speed_t speed_from_bps(int bps)
{
    switch (bps) {
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return 0;
    }
}

static int bps_from_speed(speed_t s)
{
    if (s == B4800)   return 4800;
    if (s == B9600)   return 9600;
    if (s == B19200)  return 19200;
    if (s == B38400)  return 38400;
    if (s == B57600)  return 57600;
    if (s == B115200) return 115200;
    return 0;
}

void usage(FILE *dest, const char *name, int full)
{
    fprintf(dest,
        "usage: %s <satellite_id> [options]\n"
        "\n"
        "Live satellite tracker for the FrontierSat ground station. Predicts\n"
        "passes, drives the SPID rotator, and (with --with-radio) tunes the\n"
        "configured radio onto the simplex UHF carrier with Doppler correction.\n"
        "\n"
        "Positional arguments:\n"
        "  <satellite_id>               Name prefix to match in the TLE, or `next`\n"
        "                               to auto-select the next visible pass\n"
        "\n"
        "TLE source:\n"
        "  --tle=<path>                 Path to a TLE file (2 or 3-line format).\n"
        "                               Default: $HOME/.local/state/simple_sat_ops/active.tle\n"
        "\n"
        "Hardware toggles (all opt-in; no hardware is the default):\n"
        "  --with-radio                 Initialise and command the radio\n"
        "  --with-rotator               Initialise and command the SPID Rot2Prog\n"
        "  --with-hardware              Shortcut: enables both of the above\n"
        "  --uplink-ready               After radio init, run radio_uplink_prep\n"
        "                               (FM + DATA-on + MOD source = ACC) — the\n"
        "                               configuration AX100 uplink expects.\n"
        "  --uplink-mod-level=<0..255>  Also set USB MOD level — how loud the PCM\n"
        "                               is on the modulator (empirical).\n"
        "  --tx-power=<0..100>          RF power, %%. Untouched if omitted (uses\n"
        "                               the radio's current setting).\n"
        "  --allow-high-power           Required for --tx-power above 10%%.\n"
        "  --allow-tx                   Clear the default TX inhibit. Without this,\n"
        "                               PTT-on calls return RADIO_TX_INHIBITED so\n"
        "                               the radio is configured but never keyed.\n"
        "  --radio-type=<id>            yaesu-cat (default) | icom-civ | usrp-b210\n"
        "\n"
        "Radio transport (see --help-full for the supported setups):\n"
        "  --radio-device=<path>        CAT/CI-V tty. Falls back to the saved\n"
        "                               default in ~/.local/share/simple_sat_ops/\n"
        "                               radio_device, then /dev/ttyUSB1.\n"
        "  --radio-serial-speed=<bps>   Serial speed integer. Falls back to the\n"
        "                               saved default, then 4800 (yaesu-cat) or\n"
        "                               115200 (icom-civ).\n"
        "\n"
        "Carrier and modes:\n"
        "  --uplink-freq-mhz=<mhz>      Uplink nominal (default %.6f)\n"
        "  --downlink-freq-mhz=<mhz>    Downlink / simplex carrier nominal\n"
        "                               (default %.6f)\n"
        "  --uplink-mode=<mode>         FM|USB|LSB|CW|AM (default FM)\n"
        "  --downlink-mode=<mode>       FM|USB|LSB|CW|AM (default FM)\n"
        "  --no-doppler-correction      Disable runtime Doppler retuning\n"
        "\n"
        "Rotator overrides:\n"
        "  --rotator-device=<path>           SPID Rot2Prog tty. Default\n"
        "                                    /dev/ttyUSB0 (Linux). On macOS\n"
        "                                    pass the actual port, e.g.\n"
        "                                    /dev/cu.usbserial-XXXXXXXX.\n"
        "  --rotator-target-azimuth=<deg>    Park on a fixed azimuth\n"
        "  --rotator-target-elevation=<deg>  Park on a fixed elevation\n"
        "\n"
        "Observer location (default RAO Priddis):\n"
        "  --lat=<deg>                  Geodetic latitude\n"
        "  --lon=<deg>                  Geodetic longitude (east positive)\n"
        "  --alt=<m>                    Altitude above ellipsoid, metres\n"
        "\n"
        "Pass filter (used when <satellite_id> = `next`):\n"
        "  --include-constellations     Include Starlink/OneWeb-style swarms\n"
        "  --min-altitude-km=<km>       Minimum orbital altitude (default 0)\n"
        "  --max-altitude-km=<km>       Maximum orbital altitude (default 1000)\n"
        "  --min-elevation=<deg>        Minimum peak elevation (default 0)\n"
        "  --min-minutes=<n>            Minimum minutes until AOS (default 1)\n"
        "  --max-minutes=<n>            Maximum minutes until AOS (default 90)\n"
        "\n"
        "Recording:\n"
        "  --without-audio              Skip ALSA capture thread\n"
        "  --radio-audio-output-file=<basename>\n"
        "                               Capture file basename\n"
        "                               (default session/session_pcm_audio)\n"
        "\n"
        "Other:\n"
        "  --verbose=<level>            Verbosity integer\n"
        "  --help                       Short help (this message)\n"
        "  --help-full                  Detailed help with keyboard, transports,\n"
        "                               and operational notes\n",
        name,
        UPLINK_FREQ_MHZ, DOWNLINK_FREQ_MHZ);

    if (!full) return;

    fprintf(dest,
        "\n"
        "KEYBOARD (unlocked by default, press K to toggle lock state)\n"
        "\n"
        "  K         Toggle keyboard lock\n"
        "  T         Start tracking the current satellite\n"
        "  s         Stop tracking\n"
        "  r         Reset rotator to az=0, el=0\n"
        "  [         Nudge antenna azimuth -5 deg\n"
        "  ]         Nudge antenna azimuth +5 deg\n"
        "  *         Enable Wildrose reference mode (432.325 MHz CW)\n"
        "  q         Quit\n"
        "\n"
        "RADIO TRANSPORT SETUPS\n"
        "\n"
        "Pick the radio with --radio-type=. Both supported backends speak over a\n"
        "USB-CAT serial port; the binary is dispatch-agnostic.\n"
        "\n"
        "  A. Yaesu FT-991A (default, --radio-type=yaesu-cat)\n"
        "       --radio-device=/dev/ttyUSBn  --radio-serial-speed=38400\n"
        "       See radio_yaesu_cat.c header for the front-panel one-time setup.\n"
        "\n"
        "  B. ICOM IC-9700 (legacy, --radio-type=icom-civ)\n"
        "       --radio-device=/dev/ttyUSBn or /dev/ttyACM0 (native USB-CDC)\n"
        "       --radio-serial-speed=115200 (or stored default)\n"
        "\n"
        "RADIO STARTUP STATE (when --with-radio)\n"
        "\n"
        "simple_sat_ops assumes exclusive ownership of the configured radio:\n"
        "  - tuned to --downlink-freq-mhz on the active VFO\n"
        "  - FM mode\n"
        "  - With --uplink-ready: FM-DATA + MOD source = --mod-input\n"
        "  - Doppler correction retunes the active VFO only while in-pass\n"
        "\n"
        "EXAMPLES\n"
        "\n"
        "  # Dry-run prediction (uses default TLE at ~/.local/state/simple_sat_ops/active.tle)\n"
        "  %s 'ISS (ZARYA)'\n"
        "\n"
        "  # Auto-pick the next 10-45 min visible pass above 10 deg\n"
        "  %s next --min-elevation=10 --min-minutes=10 --max-minutes=45\n"
        "\n"
        "  # Explicit TLE file\n"
        "  %s next --tle=TLEs/amateur.tle --with-hardware\n"
        "\n"
        "  # Live operation, override the device path for this run\n"
        "  %s next --with-hardware --radio-device=/dev/ttyUSB0\n",
        name, name, name, name);
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
    mvprintw(row++, col, "%15s   %s (%s)", "satellite", state->prediction.satellite_ephem.tle.sat_name, state->prediction.satellite_ephem.tle.idesg);
    clrtoeol();
    if (state->prediction.minutes_since_epoch/1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attron(COLOR_PAIR(2));
    } 
    mvprintw(row++, col, "%15s   %0.1f days", "epoch age", state->prediction.minutes_since_epoch / 1440.0);
    if (state->prediction.minutes_since_epoch/1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attroff(COLOR_PAIR(2));
    } 
    clrtoeol();

    if (state->in_pass) {
        mvprintw(row++, col, "%15s   %s", "status", "** IN PASS **");
        if (state->antenna_rotator.tracking) {
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

    minutes_until_visible(&state->prediction, jul_utc, jul_utc + MAX_MINUTES_TO_PREDICT / 1440.0, 1.0);
    if (fabs(state->prediction.predicted_minutes_until_visible) < 1) {
        minutes_until_visible(&state->prediction, jul_utc, jul_utc + 2.0 / 1440.0, 1./120.0);
    } else if (fabs(state->prediction.predicted_minutes_until_visible) < 10) {
        minutes_until_visible(&state->prediction, jul_utc, jul_utc + 20.0 / 1440.0, 0.1);
    }
    if (state->prediction.predicted_minutes_until_visible > 0) {
        if (state->prediction.predicted_minutes_until_visible < 1) {
            mvprintw(row++, col, "%15s   ", "next pass in");
            attron(COLOR_PAIR(2));
            printw("%.0f seconds", floor(state->prediction.predicted_minutes_until_visible * 60.0));
            attroff(COLOR_PAIR(2));
        } else if (state->prediction.predicted_minutes_until_visible < 10) {
            mvprintw(row++, col, "%15s   %.1f minutes", "next pass in", state->prediction.predicted_minutes_until_visible);
        } else {
            mvprintw(row++, col, "%15s   %.0f minutes", "next pass in", state->prediction.predicted_minutes_until_visible);
        }
        clrtoeol();
        update_pass_predictions(&state->prediction, jul_utc + state->prediction.predicted_minutes_until_visible / 1440.0, 0.1);
        mvprintw(row++, col, "%15s   %.1f minutes", "duration", state->prediction.predicted_minutes_above_0_degrees);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "el>30", state->prediction.predicted_minutes_above_30_degrees);
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   ", "elapsed time");
        attron(COLOR_PAIR(3));
        if (fabs(state->prediction.predicted_minutes_until_visible) < 1) {
            printw("%.0f seconds", floor(-state->prediction.predicted_minutes_until_visible * 60.0));
        } else {
            printw("%.1f minutes", -state->prediction.predicted_minutes_until_visible);
        }
        if (state->prediction.predicted_max_elevation == -180.0) {
            update_pass_predictions(&state->prediction, jul_utc - state->prediction.predicted_minutes_until_visible / 1440.0, 0.1);
        }
        attroff(COLOR_PAIR(3));
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "duration", state->prediction.predicted_minutes_above_0_degrees);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "el>30", state->prediction.predicted_minutes_above_30_degrees);
        clrtoeol();
    }
    mvprintw(row++, col, "%15s   %.1f deg", "max elevation", state->prediction.predicted_max_elevation);
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

    // Check if we have waterfall data
    if (state->radio.waterfall_enabled) {
        mvprintw(0, 0, "%s", "Waterfall");
        // Read the waterfall data
        int radio_result = radio_command(&state->radio, 0x27, 0x00, -1, NULL, 0, NULL, 0);
        // state->radio.result[0] = '\0';
        // ssize_t bytes_received = -1;
        // int remaining_buffer = RADIO_MAX_COMMAND_RESULT_LEN;
        // size_t offset = 0;
        // while (bytes_received != 0) {
        //     bytes_received = read(state->radio.fd, state->radio.result + offset, remaining_buffer);
        //     if (bytes_received == -1) {
        //         break;
        //     }
        //     offset += bytes_received;
        //     remaining_buffer -= bytes_received;
        //     if (remaining_buffer <= 0) {
        //         break;
        //     }
        // }
        // state->radio.result_len = offset;
        printw(" %d bytes received:", state->radio.result_len);
        for (int i = 0; i < state->radio.result_len && i < 50; ++i) {
            printw(" %d", state->radio.result[i]);
        }
    } else {
        mvprintw(0, 0, "%s", "");
    }
    clrtoeol();

    if (state->have_radio) {
        mvprintw(row++, col, "%15s   %.6f MHz", "CARRIER", state->radio.doppler_correction_enabled ? state->radio.doppler_downlink_frequency / 1e6 : state->radio.nominal_downlink_frequency / 1e6);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.6f MHz", "VFO Main", state->radio.vfo_main_actual_frequency / 1e6);
        clrtoeol();
        row++;
    } else {
        mvprintw(row++, col, "%15s   %s", "transceiver", "* not initialized *");
        clrtoeol();
    }
    if (state->have_antenna_rotator) {
        double azimuth = 0.0;
        double elevation = 0.0;
        if (antenna_rotator_command(&state->antenna_rotator, ANTENNA_ROTATOR_STATUS, &azimuth, &elevation) == ANTENNA_ROTATOR_OK) {
            state->antenna_rotator.azimuth = azimuth;
            state->antenna_rotator.elevation = elevation;
        } else {
            azimuth = state->antenna_rotator.azimuth;
            elevation = state->antenna_rotator.elevation;
        }
        double az_display = azimuth;
        if (az_display < 0) {
            az_display += 360.0;
        }
        double target_az_display = state->antenna_rotator.target_azimuth;
        if (target_az_display < 0) {
            target_az_display += 360.0;
        }
        const char *flip_tag = state->antenna_rotator.flip_mode_pass ? " (flip)" : "";
        mvprintw(row++, col, "%15s   %.1f deg%s", "target azimuth", target_az_display, flip_tag);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "azimuth", az_display);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg%s", "target elevation", state->antenna_rotator.target_elevation, flip_tag);
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

    mvprintw(row++, col, "%15s   %.2f deg", "azimuth", state->prediction.satellite_ephem.azimuth);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f deg", "elevation", state->prediction.satellite_ephem.elevation);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km", "altitude", state->prediction.satellite_ephem.altitude_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg N", "latitude", state->prediction.satellite_ephem.latitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg E", "longitude", state->prediction.satellite_ephem.longitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "speed", state->prediction.satellite_ephem.speed_km_s);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f km", "range", state->prediction.satellite_ephem.range_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "range rate", state->prediction.satellite_ephem.range_rate_km_s);
    clrtoeol();

    *print_row = row;
}

int apply_args(state_t *state, int argc, char **argv, double jul_utc);

int main(int argc, char **argv) 
{

    int ret = 0;
    state_t state = {0};
    state.prediction.predicted_max_elevation = -180.0;
    
    int status = 0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);
    status = apply_args(&state, argc, argv, jul_utc);
    if (status != 0) {
        return status;
    }

    /* Parse TLE data */
    int tle_status = load_tle(&state.prediction);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.prediction.satellite_ephem.tle);

    int radio_result = 0;
    if (state.run_with_radio) {
        state.radio.is_required = 1;
        state.radio.nominal_downlink_frequency = state.radio.satellite_downlink_frequency;
        state.radio.nominal_uplink_frequency = state.radio.satellite_uplink_frequency;
        state.radio.tx_inhibit_cleared = state.allow_tx;
        // Show the resolved device + speed so the operator can sanity-
        // check against `radio_ctl identify`. Printed before ncurses
        // takes over the screen so it lands in the shell.
        fprintf(stderr, "simple_sat_ops: radio device=%s speed=%dbps "
                "backend=%s\n",
                state.radio.device_filename ? state.radio.device_filename : "(null)",
                bps_from_speed(state.radio.serial_speed),
                state.radio_backend == RADIO_BACKEND_YAESU_CAT ? "yaesu-cat"
                : state.radio_backend == RADIO_BACKEND_ICOM_CIV ? "icom-civ"
                : state.radio_backend == RADIO_BACKEND_USRP_B210 ? "usrp-b210"
                : "?");
        if (radio_backend_select(&state.radio, state.radio_backend) != RADIO_OK) {
            return EXIT_FAILURE;
        }
        radio_result = radio_init(&state.radio);
        if (radio_result != RADIO_OK) {
            fprintf(stderr, "Error initializing radio\n");
            return EXIT_FAILURE;
        }
        state.have_radio = 1;
        if (state.prepare_uplink) {
            radio_result = radio_uplink_prep(&state.radio);
            if (radio_result != RADIO_OK) {
                fprintf(stderr, "Error configuring radio for uplink (--uplink-ready)\n");
                return EXIT_FAILURE;
            }
            if (state.uplink_mod_level >= 0) {
                radio_result = radio_set_usb_mod_level(&state.radio, state.uplink_mod_level);
                if (radio_result != RADIO_OK) {
                    fprintf(stderr, "Error setting USB MOD level\n");
                    return EXIT_FAILURE;
                }
            }
        }
        if (state.tx_power_pct >= 0) {
            if (state.tx_power_pct > 10 && !state.allow_high_power) {
                fprintf(stderr, "error: --tx-power=%d above 10%% safety threshold; "
                        "add --allow-high-power to override.\n", state.tx_power_pct);
                return EXIT_FAILURE;
            }
            int raw = (int)((state.tx_power_pct * 255 + 50) / 100);
            radio_result = radio_set_rf_power(&state.radio, raw);
            if (radio_result != RADIO_OK) {
                fprintf(stderr, "Error setting RF power\n");
                return EXIT_FAILURE;
            }
        }
        // Optional ALSA capture. Gated on audio_record so --without-audio
        // actually skips the init call; the device names in audio.h
        // (hw:3,0 / hw:4,0) are system-specific and will fail to open on
        // most boxes, which we shouldn't turn into a hard abort when the
        // operator explicitly said they don't want recording.
        if (state.audio.audio_record) {
            state.audio.recording_audio = 1;
            status = init_audio_capture(&state.audio);
            if (status != AUDIO_OK) {
                endwin();
                fprintf(stderr, "Unable to initialize audio capture "
                        "(pass --without-audio to skip)\n");
                return 1;
            }
            int thread_status = pthread_create(&state.audio.audio_thread_main, NULL, capture_audio, &state);
            if (thread_status != 0) {
                endwin();
                fprintf(stderr, "Unable to create an audio recording thread\n");
                return 1;
            }
        }
    }

    int antenna_rotator_result = 0;
    if (state.run_with_antenna_rotator) {
        state.antenna_rotator.is_required = 1;
        antenna_rotator_result = antenna_rotator_init(&state.antenna_rotator);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error initializing antenna rotator\n");
            return EXIT_FAILURE;
        }
        state.have_antenna_rotator = 1;
        // Adopt whatever extended position the SPID is already at so the
        // unwrapped accumulator starts grounded in reality. If this fails,
        // the rotator is likely not in 'A' mode; tracking and 'r' will
        // re-attempt the seed before issuing any motion command.
        if (antenna_rotator_seed_from_status(&state.antenna_rotator) != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Warning: could not read SPID position; check that the Rot2ProG is in 'A' mode\n");
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

    state.antenna_rotator.antenna_should_be_controlled = state.run_with_antenna_rotator && state.have_antenna_rotator;
    state.antenna_rotator.antenna_is_under_control = state.antenna_rotator.antenna_should_be_controlled;

    // Boots unlocked so the operator can drive the rig the moment the
    // UI comes up. Press K to toggle the lock when stepping away.
    int keyboard_unlocked = 1;
    int keyboard_info_row = 20;

    mvprintw(keyboard_info_row++, 3, "%s", "T  - Track satellite");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "s  - Stop antenna immediately (halts jog or tracking)");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "r  - Reset to az=0 el=0 (unwinds along the way it came in)");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "[  - Jog azimuth -5 deg (one step per press)");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "]  - Jog azimuth +5 deg (one step per press)");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "K  - Lock/unlock keyboard");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "q  - Quit");
    clrtoeol();

    double current_az = 0;
    double current_el = 0;
    double last_az = 0;
    double last_el = 0;

    
    while (state.running) {
        // Refresh
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_utc);

        /* Calculate Doppler shift */
        if (state.radio.doppler_correction_enabled) {
            update_doppler_shifted_frequencies(&state, state.radio.nominal_uplink_frequency, state.radio.nominal_downlink_frequency);
            doppler_delta_uplink = fabs(state.radio.doppler_uplink_frequency - current_uplink_frequency);
            doppler_delta_downlink = fabs(state.radio.doppler_downlink_frequency - current_downlink_frequency);
        }

        // TODO check for passes that reach a minimum elevation
        current_az = state.antenna_rotator.azimuth;
        current_el = state.antenna_rotator.elevation;
        // check if antenna reached its target
        if (state.antenna_rotator.antenna_is_moving) {
            if (fabs(current_az - last_az) == 0 && fabs(current_el - last_el) == 0) {
                state.antenna_rotator.antenna_is_moving = 0;
            }
            last_az = current_az;
            last_el = current_el;
        }

        // Drive the second leg of a two-step home once the first leg has stopped.
        // Used when the unwrap delta from the current position to home exceeds
        // 180 deg, where a single SET would leave the SPID's direction-of-rotation
        // ambiguous (and risk wrapping coax around the tower).
        if (state.antenna_rotator.homing_in_progress
            && !state.antenna_rotator.antenna_is_moving
            && state.have_antenna_rotator) {
            double final_az = state.antenna_rotator.home_pending_final_az;
            int rc = antenna_rotator_set_unwrapped(&state.antenna_rotator, final_az, 0.0);
            if (rc == ANTENNA_ROTATOR_OK) {
                state.antenna_rotator.antenna_is_moving = 1;
            }
            state.antenna_rotator.homing_in_progress = 0;
            state.antenna_rotator.home_pending_final_az = 0.0;
        }
        if (state.satellite_tracking && state.prediction.predicted_minutes_until_visible < state.antenna_rotator.tracking_prep_time_minutes) {
            if (!state.in_pass) {
                state.in_pass = 1;
            }
            if (state.antenna_rotator.antenna_should_be_controlled && !state.antenna_rotator.tracking) {
                if (!state.antenna_rotator.fixed_target) {
                    // Decide flip mode for this pass once, before the first
                    // SET goes out, so the antenna can pre-position to the
                    // flipped horizon if needed instead of switching mid-pass.
                    state.antenna_rotator.flip_mode_pass = 0;
                    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION > 90
                        && state.prediction.predicted_max_elevation
                               >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD) {
                        state.antenna_rotator.flip_mode_pass = 1;
                    }
                    state.antenna_rotator.tracking = 1;
                }
            }

            // Point antenna at satellite or fixed target
            // TODO remove lag bias by anticipating direction
            if (state.antenna_rotator.tracking && state.antenna_rotator.antenna_is_under_control) {
                if (!state.antenna_rotator.unwrapped_target_valid) {
                    // Try to recover; refuse to issue motion until grounded.
                    if (antenna_rotator_seed_from_status(&state.antenna_rotator) != ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.tracking = 0;
                    }
                } else if (!state.antenna_rotator.antenna_is_moving) {
                    double pred_az = state.prediction.satellite_ephem.azimuth;
                    double pred_el = state.prediction.satellite_ephem.elevation;
                    double mech_az = pred_az;
                    double mech_el = pred_el;
                    antenna_rotator_to_mech_coords(state.antenna_rotator.flip_mode_pass,
                                                   pred_az, pred_el, &mech_az, &mech_el);
                    double prev_unwrapped = state.antenna_rotator.target_azimuth_unwrapped;
                    double next_az = antenna_rotator_accumulate_unwrapped(prev_unwrapped, mech_az);
                    double next_el = mech_el;
                    if (next_el < ANTENNA_ROTATOR_MINIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MINIMUM_ELEVATION;
                    } else if (next_el > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                    }
                    delta_az = next_az - prev_unwrapped;
                    // In flip mode, mech_el always wants updating: 180 - sky_el
                    // for sky_el < 0 just clamps to MAX, so the antenna parks
                    // at flipped zenith until AOS. In normal mode, leave EL
                    // alone while the satellite is below the horizon.
                    if (state.antenna_rotator.flip_mode_pass
                        || state.prediction.satellite_ephem.elevation >= 0) {
                        delta_el = next_el - state.antenna_rotator.target_elevation;
                    } else {
                        delta_el = 0.0;
                    }

                    if (fabs(delta_az) >= MAX_DELTA_AZIMUTH_DEGREES || fabs(delta_el) >= MAX_DELTA_ELEVATION_DEGREES) {
                        if (next_az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH || next_az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                            // Continuous unwrap would carry the rotator past its mechanical
                            // limits. Stop tracking and let the operator decide whether to
                            // re-seed from the opposite hemisphere.
                            state.antenna_rotator.tracking = 0;
                        } else {
                            int rc = antenna_rotator_set_unwrapped(&state.antenna_rotator, next_az, next_el);
                            if (rc != ANTENNA_ROTATOR_OK) {
                                fprintf(stderr, "Error setting antenna rotator position\n");
                            } else {
                                state.antenna_rotator.antenna_is_moving = 1;
                            }
                        }
                    }
                }
            }

            jul_idle_start = 0;  // Reset idle timer
        } else {
            if (state.in_pass) {
                state.in_pass = 0;
                jul_idle_start = jul_utc;  // Start idle timer
            }
            if (state.antenna_rotator.tracking) {
                state.antenna_rotator.tracking = 0;
                state.antenna_rotator.flip_mode_pass = 0;
            }
        }

        // Doppler retune Main whenever tracking is on. Used to be gated
        // on the prep-window block above (rotator's "<5 min from pass"
        // rule), which meant the radio's actual frequency lagged the
        // CARRIER readout for a long time and the operator couldn't tell
        // whether retunes were happening at all. The rotator gating is
        // intentional (no point pointing at horizon) but the radio is
        // happy to be retuned any time, so this block now follows
        // satellite_tracking only. Sub VFO untouched (simplex UHF; would
        // collide on the IC-9700 if both were retuned).
        if (state.satellite_tracking && state.have_radio && state.run_with_radio
            && doppler_delta_downlink > doppler_max_delta) {
            current_downlink_frequency = state.radio.doppler_downlink_frequency;
            current_uplink_frequency = state.radio.doppler_downlink_frequency;
            ret = radio_set_frequency(&state.radio, state.radio.doppler_downlink_frequency);
            if (ret != RADIO_OK) {
                // stderr would scribble across curses; the UI's CARRIER
                // vs VFO Main divergence is enough of a tell. Silenced.
            }
            state.radio.vfo_main_actual_frequency = radio_get_frequency(&state.radio);
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
            keyboard_unlocked = !keyboard_unlocked;
        } else if (keyboard_unlocked) {
            switch(key) {
                case 'q':
                    state.running = 0;
                    break;
                case 'T':
                    start_tracking(&state);
                    break;
                case 's':
                    stop_tracking(&state);
                    break;
                case 'r':
                    stop_tracking(&state);
                    point_to_stationary_target(&state, 0.0, 0.0);
                    break;
                case '[':
                    // Decrease azimuth 5 degrees, then drop any further queued
                    // jog keystrokes so a held key or fast taps can't stack into
                    // a runaway slew the operator can't easily stop.
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = antenna_rotator_increase_azimuth(&state.antenna_rotator, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ']':
                    // Increase azimuth 5 degrees; same rationale as above.
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = antenna_rotator_increase_azimuth(&state.antenna_rotator, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case 'w':
                    // Broken: waterfall data messes with responses to other commands
                    // radio_result = radio_toggle_waterfall(&state.radio);
                    // if (radio_result != RADIO_OK) {
                    //     fprintf(stderr, "Waterfall error: %d\n", radio_result);
                    // }
                    break;
                case '*':
                    // Wildrose reference station, CW, 432.325 MHz
                    stop_tracking(&state);
                    enable_wildrose_mode(&state);
                    break;
                default:
                    break;
            }
        }

        mvprintw(keyboard_info_row, 3, "%s : %s", "Keyboard", keyboard_unlocked ? "unlocked" : "LOCKED");
        clrtoeol();
        if (state.antenna_rotator.antenna_is_moving) {
            mvprintw(keyboard_info_row + 2, 0, "%s", "Antenna moving");
            clrtoeol();
        } else {
            mvprintw(keyboard_info_row + 2, 0, "%s", "Antenna stationary");
            clrtoeol();
        }

        refresh();

        // Sleep for a short interval 
        if (state.running) {
            usleep(UPDATE_INTERVAL_MICROSEC);
        }

    }

    endwin();

    // stop audio capture
    if (state.audio.audio_record == 1 && state.audio.recording_audio == 1) {
        state.audio.recording_audio = 0;
        // Wait for threads to finish
        pthread_join(state.audio.audio_thread_main, NULL);
        pthread_join(state.audio.audio_thread_sub, NULL);
        audio_capture_cleanup(&state.audio);
    }

    if (next_in_queue_name) {
        free(next_in_queue_name);
    }

    if (state.prediction.auto_sat) {
        free_passes();
    }

    /* Cleanup */
    if (state.radio.connected) {
        // Turn off waterfall output. IC-9700-specific (CI-V `27 10 00`),
        // and only meaningful if we ever started it. Skip on any other
        // backend or if waterfall was never enabled — sending raw CI-V
        // bytes to an FT-991A would just confuse its CAT engine.
        if (state.radio_backend == RADIO_BACKEND_ICOM_CIV
            && state.radio.waterfall_enabled) {
            uint8_t data[1] = {0};
            radio_result = radio_command(&state.radio, 0x27, 0x10, -1,
                                         data, 1, NULL, 0);
            if (radio_result != RADIO_OK) {
                fprintf(stderr, "Unable to set waveform data status\n");
                // Continue cleanup; this is best-effort.
            }
        }

        // set_satellite_mode is NULL on the Yaesu vtable so the dispatcher
        // returns RADIO_NOT_SUPPORTED with a one-line warning. Harmless,
        // but skip the call when we know it won't apply to keep cleanup
        // output clean.
        if (state.radio_backend == RADIO_BACKEND_ICOM_CIV) {
            radio_set_satellite_mode(&state.radio, 0);
        }
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
    double min_minutes_away = 1.0;
    double max_minutes_away = 90.0;
    double min_elevation = 0.0;
    double max_elevation = 90.0;
    int with_constellations = 0;
    state->antenna_rotator.tracking_prep_time_minutes = TRACKING_PREP_TIME_MINUTES;
    state->satellite_tracking = 0;

    state->radio.satellite_uplink_mode = RADIO_MODE_FM;
    state->radio.satellite_downlink_mode = RADIO_MODE_FM;
    state->radio.satellite_uplink_frequency = UPLINK_FREQ_MHZ * 1e6;
    state->radio.satellite_downlink_frequency = DOWNLINK_FREQ_MHZ * 1e6;
    state->radio.reference_downlink_frequency = REFERENCE_DOWNLINK_FREQ_MHZ * 1e6;
    state->radio.sub_park_frequency = RADIO_SUB_PARK_HZ;
    state->radio.doppler_correction_enabled = 1;

    state->run_with_radio = 0;
    state->prepare_uplink = 0;
    state->uplink_mod_level = -1;
    state->tx_power_pct = -1;
    state->allow_high_power = 0;
    state->allow_tx = 0;
    state->radio_backend = RADIO_BACKEND_YAESU_CAT;
    // device_filename and serial_speed are resolved post-argv: explicit
    // flag wins, then ~/.local/share/simple_sat_ops/ store, then a
    // backend-default fallback. Leave NULL/0 here so argv parsing can
    // detect "user didn't pass --radio-device=" cleanly.
    state->radio.device_filename = NULL;
    state->radio.serial_speed = 0;
    state->radio.waterfall_enabled = 0;

    state->run_with_antenna_rotator = 0;
    state->antenna_rotator.device_filename = "/dev/ttyUSB0";
    state->antenna_rotator.serial_speed = B600;
    state->antenna_rotator.fixed_target = 0;

    // Default to recording audio on hosts that have ALSA. On Mac (no
    // ALSA, audio.c replaced with audio_stub.c) skip the capture path
    // so an operator on a laptop doesn't have to pass --without-audio
    // every time just to drive the rotator + radio.
#ifdef SSO_HAVE_ALSA
    state->audio.audio_record = 1;
#else
    state->audio.audio_record = 0;
#endif
    state->audio.audio_output_file_basename = "session/session_pcm_audio";
    // state->audio.audio_device = AUDIO_DEVICE_MAIN;

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
        } else if (strncmp("--tle=", argv[i], 6) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->prediction.tles_filename = argv[i] + 6;
        } else if (strcmp("--uplink-ready", argv[i]) == 0) {
            state->n_options++;
            state->prepare_uplink = 1;
        } else if (strncmp("--uplink-mod-level=", argv[i], 19) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 20) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->uplink_mod_level = atoi(argv[i] + 19);
            if (state->uplink_mod_level < 0 || state->uplink_mod_level > 255) {
                fprintf(stderr, "--uplink-mod-level must be in 0..255\n");
                return EXIT_FAILURE;
            }
            state->prepare_uplink = 1;
        } else if (strncmp("--tx-power=", argv[i], 11) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 12) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->tx_power_pct = atoi(argv[i] + 11);
            if (state->tx_power_pct < 0 || state->tx_power_pct > 100) {
                fprintf(stderr, "--tx-power must be 0..100 (%%)\n");
                return EXIT_FAILURE;
            }
        } else if (strcmp("--allow-high-power", argv[i]) == 0) {
            state->n_options++;
            state->allow_high_power = 1;
        } else if (strcmp("--allow-tx", argv[i]) == 0) {
            state->n_options++;
            state->allow_tx = 1;
        } else if (strncmp("--radio-type=", argv[i], 13) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 14) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            radio_backend_type_t t = radio_backend_type_from_string(argv[i] + 13);
            if (t == RADIO_BACKEND__COUNT) {
                fprintf(stderr, "--radio-type: unknown '%s' "
                        "(icom-civ|yaesu-cat|usrp-b210)\n", argv[i] + 13);
                return EXIT_FAILURE;
            }
            state->radio_backend = t;
        } else if (strncmp("--radio-device=", argv[i], 15) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 16) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->radio.device_filename = argv[i] + 15;
        } else if (strncmp("--rotator-device=", argv[i], 17) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 18) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->antenna_rotator.device_filename = argv[i] + 17;
        } else if (strcmp("--without-audio", argv[i]) == 0) {
            state->n_options++;
            state->audio.audio_record = 0;
        } else if (strncmp("--radio-audio-output-file=", argv[i], 26) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 27) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]); 
                return EXIT_FAILURE;
            } 
            state->audio.audio_output_file_basename = argv[i] + 26;
        } else if (strncmp("--radio-serial-speed=", argv[i], 21) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 22) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            int bps = atoi(argv[i] + 21);
            speed_t s = speed_from_bps(bps);
            if (s == 0) {
                fprintf(stderr, "Unsupported --radio-serial-speed=%d "
                        "(supported: 4800,9600,19200,38400,57600,115200)\n", bps);
                return EXIT_FAILURE;
            }
            state->radio.serial_speed = s;
        } else if (strncmp("--uplink-mode=", argv[i], 14) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 15) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            char *mode = argv[i] + 14;
            if (strcmp("CW", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_CW;
            } else if (strcmp("FM", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_FM;
            } else if (strcmp("AM", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_AM;
            } else if (strcmp("LSB", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_LSB;
            } else if (strcmp("USB", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_USB;
            } else {
                fprintf(stderr, "Unreconized radio mode: %s\n", mode);
                return EXIT_FAILURE;
            }
        } else if (strncmp("--downlink-mode=", argv[i], 16) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 17) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            char *mode = argv[i] + 16;
            if (strcmp("CW", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_CW;
            } else if (strcmp("FM", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_FM;
            } else if (strcmp("AM", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_AM;
            } else if (strcmp("LSB", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_LSB;
            } else if (strcmp("USB", mode) == 0) {
                state->radio.satellite_uplink_mode = RADIO_MODE_USB;
            } else {
                fprintf(stderr, "Unreconized radio mode: %s\n", mode);
                return EXIT_FAILURE;
            }
        } else if (strncmp("--uplink-freq-mhz=", argv[i], 18) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->radio.satellite_uplink_frequency = atof(argv[i] + 18) * 1e6;
        } else if (strncmp("--downlink-freq-mhz=", argv[i], 20) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 21) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->radio.satellite_downlink_frequency = atof(argv[i] + 20) * 1e6;
        } else if (strcmp("--no-doppler-correction", argv[i]) == 0) {
            state->n_options++;
            state->radio.doppler_correction_enabled = 0;
        } else if (strncmp("--rotator-target-elevation=", argv[i], 27) == 0) {
            state->n_options++; 
            if (strlen(argv[i]) < 28) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->antenna_rotator.target_elevation = atof(argv[i] + 27);
            if (state->antenna_rotator.target_elevation < 0.0) {
                state->antenna_rotator.target_elevation = 0.0;
            } else if (state->antenna_rotator.target_elevation > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                state->antenna_rotator.target_elevation = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
            }
            state->antenna_rotator.fixed_target = 1;
        } else if (strncmp("--rotator-target-azimuth=", argv[i], 25) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 26) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            double az = atof(argv[i] + 25);
            if (az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH) {
                az = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
            } else if (az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                az = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
            }
            state->antenna_rotator.target_azimuth = az;
            state->antenna_rotator.target_azimuth_unwrapped = az;
            state->antenna_rotator.unwrapped_target_valid = 1;
            state->antenna_rotator.fixed_target = 1;
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
            usage(stdout, argv[0], 0);
            return 2;
        } else if (strcmp("--help-full", argv[i]) == 0) {
            usage(stdout, argv[0], 1);
            return 2;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 3;
        }

    }
    if (argc - state->n_options != 2) {
        usage(stderr, argv[0], 0);
        return 1;
    }

    // Resolve radio device + speed: explicit flag wins, then the saved
    // default at ~/.local/share/simple_sat_ops/, then a backend-default
    // fallback. Same precedence radio_ctl / tx_tone / tx_white_noise /
    // tx_frame use, so an operator who's run `radio_ctl --store-device
    // --store-serial-speed` once doesn't have to retype the path here.
    static char effective_radio_device[1024];
    if (state->radio.device_filename == NULL) {
        if (radio_device_store_load(effective_radio_device,
                                    sizeof effective_radio_device) == 0) {
            state->radio.device_filename = effective_radio_device;
        } else {
            state->radio.device_filename = "/dev/ttyUSB1";
        }
    }
    if (state->radio.serial_speed == 0) {
        int stored_bps = 0;
        if (radio_device_store_load_speed(&stored_bps) == 0) {
            state->radio.serial_speed = speed_from_bps(stored_bps);
        }
        if (state->radio.serial_speed == 0) {
            state->radio.serial_speed = (state->radio_backend == RADIO_BACKEND_YAESU_CAT)
                                            ? B4800 : B115200;
        }
    }

    state->radio.doppler_uplink_frequency = state->radio.nominal_uplink_frequency;
    state->radio.doppler_downlink_frequency = state->radio.nominal_downlink_frequency;

    state->prediction.observer_ephem.position_geodetic.lat = site_latitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.lon = site_longitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.alt = site_altitude / 1000.0;

    if (state->prediction.tles_filename == NULL) {
        static char default_tle[1024];
        if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
            fprintf(stderr, "HOME unset or path too long; pass --tle=<path>\n");
            return EXIT_FAILURE;
        }
        state->prediction.tles_filename = default_tle;
    }
    state->prediction.satellite_ephem.name = argv[1];

    if (strcmp(state->prediction.satellite_ephem.name, "next") == 0) {
        state->prediction.auto_sat = 1;
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
        prediction_t prediction_tmp = {0};
        prediction_tmp.tles_filename = state->prediction.tles_filename;
        prediction_tmp.observer_ephem.position_geodetic.lat = state->prediction.observer_ephem.position_geodetic.lat;
        prediction_tmp.observer_ephem.position_geodetic.lon = state->prediction.observer_ephem.position_geodetic.lon;
        prediction_tmp.observer_ephem.position_geodetic.alt = state->prediction.observer_ephem.position_geodetic.alt;
        find_passes(&prediction_tmp, jul_utc, 0.5, &criteria, NULL, NULL, 0, 0);
        const size_t n = number_of_passes();
        if (n == 0) {
            fprintf(stderr, "Unable to automatically find next in queue.\n");
            return 1;
        }

        const pass_t *p = get_pass(0);
        state->prediction.satellite_ephem.name = strdup(p->name);
        printf("Satellite: %s\n", state->prediction.satellite_ephem.name);
    }

    return 0;
}

void start_tracking(state_t *state)
{
    int antenna_rotator_result = 0;

    state->satellite_tracking = 1;
    state->radio.nominal_downlink_frequency = state->radio.satellite_downlink_frequency;
    state->radio.doppler_correction_enabled = 1;

    // Don't touch mode / VFO / DATA / MOD source here. The operator
    // configures all that with radio_ctl (uplink-prep, set-mode,
    // set-mod-input) before launching simple_sat_ops; this tool's job
    // during a pass is Doppler retuning, not radio reconfiguration.
    // Anything mode-changing belongs in radio_uplink_prep (--uplink-ready)
    // or the explicit '*' (Wildrose) keystroke.
    state->antenna_rotator.antenna_is_under_control = state->antenna_rotator.antenna_should_be_controlled;
    if (state->antenna_rotator.fixed_target) {
        antenna_rotator_result = antenna_rotator_set_unwrapped(
            &state->antenna_rotator,
            state->antenna_rotator.target_azimuth_unwrapped,
            state->antenna_rotator.target_elevation);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error setting antenna rotator position\n");
        } else {
            state->antenna_rotator.antenna_is_moving = 1;
        }
    }

    return;
}

void stop_tracking(state_t *state)
{
    int antenna_rotator_result = 0;
    double azimuth = 0.0;
    double elevation = 0.0;

    state->satellite_tracking = 0;
    state->radio.doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control = 0;
    if (state->run_with_antenna_rotator) {
        // dummy values
        antenna_rotator_result = antenna_rotator_command(&state->antenna_rotator, ANTENNA_ROTATOR_STOP, &azimuth, &elevation);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error stopping the antenna rotator\n");
        }
        // After a halt the antenna is wherever STOP caught it, not at the
        // last commanded target. Re-read so the next jog or track starts
        // from physical reality instead of a stale runaway target.
        if (antenna_rotator_seed_from_status(&state->antenna_rotator) != ANTENNA_ROTATOR_OK) {
            // leave unwrapped_target_valid as it was; next motion will
            // try the seed again before issuing a SET
        }
    }
    state->antenna_rotator.antenna_is_moving = 0;
    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    state->antenna_rotator.flip_mode_pass = 0;

    return;
}

void enable_wildrose_mode(state_t *state)
{
    int radio_result = 0;
    state->radio.nominal_downlink_frequency = state->radio.reference_downlink_frequency;
    state->radio.doppler_correction_enabled = 0;
    if (state->have_radio) {
        // VFOMain is an IC-9700 sub/main concept; the FT-991A backend has
        // a single VFO and reports RADIO_NOT_SUPPORTED. Treat that as OK.
        radio_result = radio_set_vfo(&state->radio, VFOMain);
        if (radio_result != RADIO_OK && radio_result != RADIO_NOT_SUPPORTED) {
            fprintf(stderr, "Error setting radio VFO to VFOMain\n");
            return;
        }
        radio_result = radio_set_mode(&state->radio, RADIO_MODE_CW, RADIO_FILTER_FIL1);
        if (radio_result != RADIO_OK) {
            fprintf(stderr, "Error setting radio to CW mode\n");
            return;
        }
        radio_result = radio_set_frequency(&state->radio, state->radio.nominal_downlink_frequency);
        if (radio_result != RADIO_OK) {
            fprintf(stderr, "Error setting radio to CW mode\n");
            return;
        }
    }

    return;
}

int point_to_stationary_target(state_t *state, double azimuth, double elevation)
{
    state->satellite_tracking = 0;
    state->antenna_rotator.antenna_is_under_control = 0;
    // The home/park target is a normal-coords sky target; if a flip pass was
    // in progress when the operator hit reset, drop the flag so the next
    // tracking pass re-evaluates from scratch.
    state->antenna_rotator.flip_mode_pass = 0;

    if (!state->antenna_rotator.unwrapped_target_valid) {
        if (antenna_rotator_seed_from_status(&state->antenna_rotator) != ANTENNA_ROTATOR_OK) {
            return ANTENNA_ROTATOR_BAD_RESPONSE;
        }
    }

    double prev = state->antenna_rotator.target_azimuth_unwrapped;
    double final_az = antenna_rotator_home_unwrapped_target(prev, azimuth);
    double delta = final_az - prev;

    if (fabs(delta) > 180.0) {
        // Send a halfway waypoint first to disambiguate the direction of
        // rotation, then finish on a later main-loop tick once the antenna
        // has stopped at the intermediate.
        double mid = prev + delta / 2.0;
        if (mid < ANTENNA_ROTATOR_MINIMUM_AZIMUTH) mid = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
        if (mid > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) mid = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
        state->antenna_rotator.home_pending_final_az = final_az;
        state->antenna_rotator.homing_in_progress = 1;
        int rc = antenna_rotator_set_unwrapped(&state->antenna_rotator, mid, elevation);
        if (rc == ANTENNA_ROTATOR_OK) {
            state->antenna_rotator.antenna_is_moving = 1;
        }
        return rc;
    }

    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    int rc = antenna_rotator_set_unwrapped(&state->antenna_rotator, final_az, elevation);
    if (rc == ANTENNA_ROTATOR_OK) {
        state->antenna_rotator.antenna_is_moving = 1;
    }
    return rc;
}

void update_doppler_shifted_frequencies(state_t *state, double uplink_freq, double downlink_freq)
{
    double doppler_factor = 1.0 - state->prediction.satellite_ephem.range_rate_km_s / 299792.458;
    state->radio.doppler_uplink_frequency = uplink_freq * doppler_factor; // Speed of light in km/s
    state->radio.doppler_downlink_frequency = downlink_freq * doppler_factor;

    return;
}

