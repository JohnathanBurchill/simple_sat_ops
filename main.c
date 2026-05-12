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
#include "state.h"
#include "prediction.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_paths.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <ncurses.h>
#include <unistd.h>

// Carrier defaults (used only for display + IPC publication; the
// actual radio is now driven externally by tx_frame_sdr / b210_rx_live
// over the sso_ipc socket).
#define UPLINK_FREQ_MHZ   145.150000
#define DOWNLINK_FREQ_MHZ 436.150000

// Doppler retune threshold (Hz). Frequencies update on the display
// when the residual drifts past this. 200 Hz keeps the residual offset
// well inside the 9600 GFSK clean-eye band (~±3 kHz) even at the peak
// ~100 Hz/s slew near TCA.
#define DOPPLER_SHIFT_RESOLUTION_KHZ 0.2

// Antenna rotator max angle from target before a new SET is issued.
#define MAX_DELTA_AZIMUTH_DEGREES 1.0
#define MAX_DELTA_ELEVATION_DEGREES 1.0

#define WARN_DAYS_SINCE_EPOCH 1.0
#define MAX_MINUTES_TO_PREDICT ((7 * 1440))

#define UPDATE_INTERVAL_MICROSEC 500000

#define SSO_IPC_MAX_CLIENTS_FOR_ROSTER 16

// --- Operator-mode IPC bookkeeping ---------------------------------
// Set by apply_args when --control is passed. When set, main() opens
// the sso_ipc server on /run/sso/simple_sat_ops.sock and fans out a
// state event on every UI tick. Other operator-aware tools
// (b210_rx_live --control, tx_frame_sdr) verify the operator's Unix
// user matches their own via this socket.
static int g_control_mode = 0;
static sso_ipc_server_t *g_ipc = NULL;
static const char *g_operator_user = NULL;
// SIGUSR1 sets this — used by the force-claim takeover path to nudge
// the operator-mode loop into a graceful exit. (Full in-place
// demotion is a follow-up; for now SIGUSR1 = quit.)
static volatile sig_atomic_t g_yield_requested = 0;
static void on_sigusr1(int sig) {
    (void) sig;
    g_yield_requested = 1;
}

static void ipc_broadcast_state(state_t *s,
                                  double az, double el,
                                  double downlink_freq,
                                  double doppler_delta_dl) {
    if (!g_ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_STATE);
    snprintf(evt.from, sizeof(evt.from), "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(evt.operator_user, sizeof(evt.operator_user), "%s",
             g_operator_user ? g_operator_user : "?");
    evt.has_state = 1;
    if (s->prediction.satellite_ephem.name) {
        snprintf(evt.satellite, sizeof(evt.satellite), "%s",
                 s->prediction.satellite_ephem.name);
    }
    evt.az = az;
    evt.el = el;
    evt.freq_hz = (long) downlink_freq;
    evt.doppler_hz = doppler_delta_dl;
    sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
    size_t n = 0;
    if (n < sizeof(entries) / sizeof(entries[0])) {
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 g_operator_user ? g_operator_user : "?");
        snprintf(entries[n].role, sizeof(entries[n].role), "operator");
        entries[n].since[0] = '\0';
        n++;
    }
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char user[64], role[16], since[40];
    while (n < sizeof(entries) / sizeof(entries[0])
           && sso_ipc_server_next_client(g_ipc, &it, &cid,
                                          user, sizeof(user),
                                          role, sizeof(role),
                                          since, sizeof(since)) == 0) {
        if (!user[0]) continue;
        snprintf(entries[n].user, sizeof(entries[n].user), "%s", user);
        snprintf(entries[n].role, sizeof(entries[n].role), "%s",
                 role[0] ? role : "viewer");
        snprintf(entries[n].since, sizeof(entries[n].since), "%s", since);
        n++;
    }
    sso_event_set_roster(&evt, entries, n);
    char buf[4096];
    if (sso_event_encode(&evt, buf, sizeof(buf)) == 0) {
        sso_ipc_server_broadcast(g_ipc, buf);
    }
}

static void ipc_on_event(sso_ipc_server_t *srv, sso_client_id_t id,
                         const sso_event_t *evt, void *user) {
    (void) user;
    if (evt->type != SSO_EVT_HELLO) return;
    sso_event_t welcome;
    sso_event_init(&welcome, SSO_EVT_WELCOME);
    snprintf(welcome.from, sizeof(welcome.from), "%s",
             g_operator_user ? g_operator_user : "?");
    snprintf(welcome.operator_user, sizeof(welcome.operator_user), "%s",
             g_operator_user ? g_operator_user : "?");
    char buf[1024];
    if (sso_event_encode(&welcome, buf, sizeof(buf)) == 0) {
        sso_ipc_server_send(srv, id, buf);
    }
}

// --- Forward decls -------------------------------------------------

void start_tracking(state_t *state);
void stop_tracking(state_t *state);
int  point_to_stationary_target(state_t *state, double azimuth, double elevation);
void update_doppler_shifted_frequencies(state_t *state, double uplink_freq, double downlink_freq);
int  apply_args(state_t *state, int argc, char **argv, double jul_utc);

// --- TLE auto-discovery helpers (used by --control with no
//     positional satellite name) ------------------------------------

// Recursively scan `dir` for *.tle files, return the path with the
// newest mtime via out_path. Returns 0 on success, -1 if dir is
// unreadable or no .tle file exists in the tree. Caller-allocated
// buffer must be at least PATH_MAX-ish.
static int find_newest_tle_recursive(const char *dir,
                                     char *out_path, size_t out_cap,
                                     time_t *out_mtime)
{
    DIR *d = opendir(dir);
    if (d == NULL) return -1;
    int found = 0;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char child[1024];
        int n = snprintf(child, sizeof child, "%s/%s", dir, de->d_name);
        if (n < 0 || (size_t)n >= sizeof child) continue;
        struct stat st;
        if (stat(child, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            char nested[1024];
            time_t nested_mtime = 0;
            if (find_newest_tle_recursive(child, nested, sizeof nested,
                                          &nested_mtime) == 0) {
                if (!found || nested_mtime > *out_mtime) {
                    snprintf(out_path, out_cap, "%s", nested);
                    *out_mtime = nested_mtime;
                    found = 1;
                }
            }
        } else if (S_ISREG(st.st_mode)) {
            size_t nlen = strlen(de->d_name);
            if (nlen < 4) continue;
            if (strcmp(de->d_name + nlen - 4, ".tle") != 0) continue;
            if (!found || st.st_mtime > *out_mtime) {
                snprintf(out_path, out_cap, "%s", child);
                *out_mtime = st.st_mtime;
                found = 1;
            }
        }
    }
    closedir(d);
    return found ? 0 : -1;
}

// Pull the satellite name out of a 3-line TLE — the first non-blank
// line that doesn't begin with "1 " or "2 ". Trims trailing
// whitespace. Returns 0 on success, -1 if the file is unreadable or
// has no name line.
static int read_tle_name(const char *tle_path,
                         char *out_name, size_t out_cap)
{
    FILE *f = fopen(tle_path, "r");
    if (f == NULL) return -1;
    char line[256];
    int rc = -1;
    while (fgets(line, sizeof line, f) != NULL) {
        size_t n = strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'
                      || line[n - 1] == ' '  || line[n - 1] == '\t')) {
            line[--n] = '\0';
        }
        if (n == 0) continue;
        if ((line[0] == '1' || line[0] == '2') && line[1] == ' ') continue;
        snprintf(out_name, out_cap, "%s", line);
        rc = 0;
        break;
    }
    fclose(f);
    return rc;
}

// mkdir -p for the parent of `path`. Used before copy_file to make
// sure ~/.local/state/simple_sat_ops/ exists.
static int mkdir_p_for_file(const char *path)
{
    char buf[1024];
    snprintf(buf, sizeof buf, "%s", path);
    for (char *p = buf + 1; *p; p++) {
        if (*p != '/') continue;
        *p = '\0';
        if (mkdir(buf, 0755) != 0 && errno != EEXIST) {
            *p = '/';
            return -1;
        }
        *p = '/';
    }
    return 0;
}

// Plain byte-copy. Returns 0 on success, -1 on any I/O error.
static int copy_file(const char *src, const char *dst)
{
    FILE *in = fopen(src, "rb");
    if (in == NULL) return -1;
    if (mkdir_p_for_file(dst) != 0) { fclose(in); return -1; }
    FILE *out = fopen(dst, "wb");
    if (out == NULL) { fclose(in); return -1; }
    char buf[4096];
    size_t n;
    int rc = 0;
    while ((n = fread(buf, 1, sizeof buf, in)) > 0) {
        if (fwrite(buf, 1, n, out) != n) { rc = -1; break; }
    }
    if (ferror(in)) rc = -1;
    fclose(in);
    if (fclose(out) != 0) rc = -1;
    return rc;
}

// --- Usage ---------------------------------------------------------

void usage(FILE *dest, const char *name, int full)
{
    fprintf(dest,
        "usage: %s [--control] [<satellite_id>] [options]\n"
        "\n"
        "Live satellite tracker for the FrontierSat ground station. Predicts\n"
        "passes, drives the SPID rotator, computes Doppler-shifted simplex\n"
        "frequencies for display, and (with --control) acts as the operator\n"
        "coordinator over the sso_ipc socket so b210_rx_live and tx_frame_sdr\n"
        "verify they're owned by the right Unix user.\n"
        "\n"
        "Modes:\n"
        "  %s --control                 Operator: opens the sso_ipc server,\n"
        "                               drives the rotator. With no\n"
        "                               <satellite_id>, the newest *.tle under\n"
        "                               /FrontierSat/TLEs/ is copied to the\n"
        "                               active.tle slot and tracked.\n"
        "  %s <satellite_id>            Standalone tracker (legacy / dev).\n"
        "  %s                           No args: probes for a running operator\n"
        "                               and exits with a hint either way.\n"
        "\n"
        "Positional arguments:\n"
        "  <satellite_id>               Name prefix to match in the TLE, or `next`\n"
        "                               to auto-select the next visible pass.\n"
        "                               Optional when --control is given.\n"
        "\n"
        "TLE source:\n"
        "  --tle=<path>                 Path to a TLE file (2 or 3-line format).\n"
        "                               Default: $HOME/.local/state/simple_sat_ops/active.tle\n"
        "                               (auto-populated by --control without a\n"
        "                               positional satellite_id).\n"
        "\n"
        "Hardware (rotator only — the radio is now the B210, driven\n"
        "externally; there is no audio path in this binary):\n"
        "  --without-rotator            Skip the SPID Rot2Prog. Default is on:\n"
        "                               the tracker initialises and commands\n"
        "                               the rotator unless this flag is given.\n"
        "  --without-hardware           Synonym for --without-rotator\n"
        "\n"
        "Operator coordination:\n"
        "  --control                    Open the sso_ipc server (operator mode).\n"
        "                               Without it, simple_sat_ops runs standalone\n"
        "                               and no other operator-aware tool can verify\n"
        "                               against it.\n"
        "\n"
        "Carrier display (informational; the B210 tools tune themselves):\n"
        "  --uplink-freq-mhz=<mhz>      Uplink nominal (default %.6f)\n"
        "  --downlink-freq-mhz=<mhz>    Downlink / simplex carrier nominal\n"
        "                               (default %.6f)\n"
        "  --no-doppler-correction      Display nominal freqs without Doppler\n"
        "\n"
        "Rotator overrides:\n"
        "  --rotator-device=<path>           SPID Rot2Prog tty. Default\n"
        "                                    /dev/ttyUSB0 (Linux).\n"
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
        "Other:\n"
        "  --verbose=<level>            Verbosity integer\n"
        "  --help                       Short help (this message)\n"
        "  --help-full                  Detailed help with keyboard layout\n",
        name, name, name, name,
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
        "  q         Quit\n"
        "\n"
        "EXAMPLES\n"
        "\n"
        "  # Auto-pick next visible pass above 10 deg (rotator is on by default)\n"
        "  %s next --min-elevation=10 --min-minutes=10 --max-minutes=45\n"
        "\n"
        "  # Dry-run prediction on a dev host (no rotator hardware)\n"
        "  %s 'ISS (ZARYA)' --without-rotator\n"
        "\n"
        "  # Operator coordination (broadcasts state to b210_rx_live + tx_frame_sdr)\n"
        "  %s next --control\n",
        name, name, name);
}

// --- ncurses ------------------------------------------------------

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
}

// --- Reports -------------------------------------------------------

void report_predictions(state_t *state, double jul_utc, int *print_row, int print_col)
{
    if (print_row == NULL) {
        return;
    }
    int row = *print_row;
    int col = print_col;

    struct tm utc;
    UTC_Calendar_Now(&utc, NULL);
    char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    mvprintw(row++, col, "%15s   %d %s %04d %02d:%02d:%02d UTC", "date",
             utc.tm_mday, months[utc.tm_mon - 1], utc.tm_year,
             utc.tm_hour, utc.tm_min, utc.tm_sec);
    clrtoeol();

    row++;
    mvprintw(row++, col, "%15s   %s (%s)", "satellite",
             state->prediction.satellite_ephem.tle.sat_name,
             state->prediction.satellite_ephem.tle.idesg);
    clrtoeol();
    if (state->prediction.minutes_since_epoch / 1440.0 >= WARN_DAYS_SINCE_EPOCH) {
        attron(COLOR_PAIR(2));
    }
    mvprintw(row++, col, "%15s   %0.1f days", "epoch age",
             state->prediction.minutes_since_epoch / 1440.0);
    if (state->prediction.minutes_since_epoch / 1440.0 >= WARN_DAYS_SINCE_EPOCH) {
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

    minutes_until_visible(&state->prediction, jul_utc,
                          jul_utc + MAX_MINUTES_TO_PREDICT / 1440.0, 1.0);
    if (fabs(state->prediction.predicted_minutes_until_visible) < 1) {
        minutes_until_visible(&state->prediction, jul_utc,
                              jul_utc + 2.0 / 1440.0, 1. / 120.0);
    } else if (fabs(state->prediction.predicted_minutes_until_visible) < 10) {
        minutes_until_visible(&state->prediction, jul_utc,
                              jul_utc + 20.0 / 1440.0, 0.1);
    }
    if (state->prediction.predicted_minutes_until_visible > 0) {
        if (state->prediction.predicted_minutes_until_visible < 1) {
            mvprintw(row++, col, "%15s   ", "next pass in");
            attron(COLOR_PAIR(2));
            printw("%.0f seconds",
                   floor(state->prediction.predicted_minutes_until_visible * 60.0));
            attroff(COLOR_PAIR(2));
        } else if (state->prediction.predicted_minutes_until_visible < 10) {
            mvprintw(row++, col, "%15s   %.1f minutes", "next pass in",
                     state->prediction.predicted_minutes_until_visible);
        } else {
            mvprintw(row++, col, "%15s   %.0f minutes", "next pass in",
                     state->prediction.predicted_minutes_until_visible);
        }
        clrtoeol();
        update_pass_predictions(&state->prediction,
            jul_utc + state->prediction.predicted_minutes_until_visible / 1440.0,
            0.1);
        mvprintw(row++, col, "%15s   %.1f minutes", "duration",
                 state->prediction.predicted_minutes_above_0_degrees);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "el>30",
                 state->prediction.predicted_minutes_above_30_degrees);
        clrtoeol();
    } else {
        mvprintw(row++, col, "%15s   ", "elapsed time");
        attron(COLOR_PAIR(3));
        if (fabs(state->prediction.predicted_minutes_until_visible) < 1) {
            printw("%.0f seconds",
                   floor(-state->prediction.predicted_minutes_until_visible * 60.0));
        } else {
            printw("%.1f minutes",
                   -state->prediction.predicted_minutes_until_visible);
        }
        if (state->prediction.predicted_max_elevation == -180.0) {
            // Started mid-pass: walk back to AOS so update_pass_predictions
            // captures the true max elevation rather than just the remainder.
            update_pass_predictions(&state->prediction,
                jul_utc + state->prediction.predicted_minutes_until_visible / 1440.0,
                0.1);
        }
        attroff(COLOR_PAIR(3));
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "duration",
                 state->prediction.predicted_minutes_above_0_degrees);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f minutes", "el>30",
                 state->prediction.predicted_minutes_above_30_degrees);
        clrtoeol();
    }
    mvprintw(row++, col, "%15s   %.1f deg", "max elevation",
             state->prediction.predicted_max_elevation);
    clrtoeol();

    *print_row = row;
}

void report_status(state_t *state, int *print_row, int print_col)
{
    if (print_row == NULL) {
        return;
    }
    int row = *print_row;
    int col = print_col;

    // Header line: operator + viewer count (replaces the old radio status).
    if (g_control_mode) {
        size_t nc = sso_ipc_server_client_count(g_ipc);
        mvprintw(0, 0, "%-15s %s   viewers: %zu",
                 "OPERATOR",
                 g_operator_user ? g_operator_user : "?",
                 nc);
        clrtoeol();
    } else {
        mvprintw(0, 0, "%-15s %s (no --control)", "OPERATOR",
                 g_operator_user ? g_operator_user : "?");
        clrtoeol();
    }

    // Doppler-shifted display freqs (carrier the simplex link is on).
    double display_dl_hz = state->doppler_downlink_frequency_hz;
    if (display_dl_hz == 0.0) display_dl_hz = state->nominal_downlink_frequency_hz;
    mvprintw(row++, col, "%15s   %.6f MHz", "CARRIER",
             display_dl_hz / 1e6);
    clrtoeol();
    row++;

    if (state->have_antenna_rotator) {
        double azimuth = 0.0;
        double elevation = 0.0;
        if (antenna_rotator_command(&state->antenna_rotator,
                                    ANTENNA_ROTATOR_STATUS,
                                    &azimuth, &elevation) == ANTENNA_ROTATOR_OK) {
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
        mvprintw(row++, col, "%15s   %.1f deg%s", "target azimuth",
                 target_az_display, flip_tag);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg", "azimuth", az_display);
        clrtoeol();
        mvprintw(row++, col, "%15s   %.1f deg%s", "target elevation",
                 state->antenna_rotator.target_elevation, flip_tag);
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

    mvprintw(row++, col, "%15s   %.2f deg", "azimuth",
             state->prediction.satellite_ephem.azimuth);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f deg", "elevation",
             state->prediction.satellite_ephem.elevation);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km", "altitude",
             state->prediction.satellite_ephem.altitude_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg N", "latitude",
             state->prediction.satellite_ephem.latitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f deg E", "longitude",
             state->prediction.satellite_ephem.longitude);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "speed",
             state->prediction.satellite_ephem.speed_km_s);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.1f km", "range",
             state->prediction.satellite_ephem.range_km);
    clrtoeol();
    mvprintw(row++, col, "%15s   %.2f km/s", "range rate",
             state->prediction.satellite_ephem.range_rate_km_s);
    clrtoeol();

    *print_row = row;
}

// --- main ---------------------------------------------------------

int main(int argc, char **argv)
{
    state_t state = {0};
    state.prediction.predicted_max_elevation = -180.0;

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);
    int status = apply_args(&state, argc, argv, jul_utc);
    if (status != 0) {
        return status;
    }

    // Audit + operator IPC bring-up.
    g_operator_user = sso_unix_user();
    sso_audit_start("simple_sat_ops",
                    g_control_mode ? "operator" : "standalone");
    if (g_control_mode) {
        g_ipc = sso_ipc_server_open("simple_sat_ops");
        if (g_ipc == NULL) {
            fprintf(stderr, "simple_sat_ops: --control requested but socket "
                            "bind failed (already running? check "
                            "/run/sso/simple_sat_ops.{sock,pid}). "
                            "Continuing without IPC.\n");
            sso_audit_event("ipc-bind-failed", "");
        } else {
            sso_ipc_server_on_event(g_ipc, ipc_on_event, NULL);
            struct sigaction sa;
            memset(&sa, 0, sizeof(sa));
            sa.sa_handler = on_sigusr1;
            sigemptyset(&sa.sa_mask);
            sigaction(SIGUSR1, &sa, NULL);
            fprintf(stderr, "simple_sat_ops: operator=%s ipc=on\n",
                    g_operator_user);
        }
    }

    /* Parse TLE data */
    int tle_status = load_tle(&state.prediction);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.prediction.satellite_ephem.tle);

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
        // unwrapped accumulator starts grounded in reality.
        if (antenna_rotator_seed_from_status(&state.antenna_rotator) != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Warning: could not read SPID position; "
                            "check that the Rot2ProG is in 'A' mode\n");
        }
    }

    /* Tracking loop */
    double jul_idle_start = 0;  // last-tracked timestamp

    init_window();

    char key = '\0';
    int row = 0;
    int col = 2;
    state.running = 1;

    double current_uplink_frequency = state.nominal_uplink_frequency_hz;
    double current_downlink_frequency = state.nominal_downlink_frequency_hz;
    double doppler_delta_uplink = 0.0;
    double doppler_delta_downlink = 0.0;
    double doppler_max_delta = DOPPLER_SHIFT_RESOLUTION_KHZ * 1000.0;
    (void) doppler_delta_uplink;  // tracked for display symmetry / future IPC
    (void) doppler_max_delta;     // threshold for any future on-display retune

    double delta_az = 0.0;
    double delta_el = 0.0;

    state.antenna_rotator.antenna_should_be_controlled =
        state.run_with_antenna_rotator && state.have_antenna_rotator;
    state.antenna_rotator.antenna_is_under_control =
        state.antenna_rotator.antenna_should_be_controlled;

    int keyboard_unlocked = 1;
    int keyboard_info_row = 20;

    mvprintw(keyboard_info_row++, 3, "%s", "T  - Track satellite");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "s  - Stop antenna immediately");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "r  - Reset to az=0 el=0");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "[  - Jog azimuth -5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "]  - Jog azimuth +5 deg");
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
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_utc);

        /* Calculate Doppler shift for the display + IPC publishing */
        if (state.doppler_correction_enabled) {
            update_doppler_shifted_frequencies(&state,
                                                state.nominal_uplink_frequency_hz,
                                                state.nominal_downlink_frequency_hz);
            doppler_delta_uplink   = fabs(state.doppler_uplink_frequency_hz
                                           - current_uplink_frequency);
            doppler_delta_downlink = fabs(state.doppler_downlink_frequency_hz
                                           - current_downlink_frequency);
            current_uplink_frequency   = state.doppler_uplink_frequency_hz;
            current_downlink_frequency = state.doppler_downlink_frequency_hz;
        }

        current_az = state.antenna_rotator.azimuth;
        current_el = state.antenna_rotator.elevation;
        if (state.antenna_rotator.antenna_is_moving) {
            if (fabs(current_az - last_az) == 0
                && fabs(current_el - last_el) == 0) {
                state.antenna_rotator.antenna_is_moving = 0;
            }
            last_az = current_az;
            last_el = current_el;
        }

        // Drive the second leg of a two-step home once the first leg has stopped.
        if (state.antenna_rotator.homing_in_progress
            && !state.antenna_rotator.antenna_is_moving
            && state.have_antenna_rotator) {
            double final_az = state.antenna_rotator.home_pending_final_az;
            int rc = antenna_rotator_set_unwrapped(&state.antenna_rotator,
                                                    final_az, 0.0);
            if (rc == ANTENNA_ROTATOR_OK) {
                state.antenna_rotator.antenna_is_moving = 1;
            }
            state.antenna_rotator.homing_in_progress = 0;
            state.antenna_rotator.home_pending_final_az = 0.0;
        }
        if (state.satellite_tracking
            && state.prediction.predicted_minutes_until_visible
                   < state.antenna_rotator.tracking_prep_time_minutes) {
            if (!state.in_pass) {
                state.in_pass = 1;
            }
            if (state.antenna_rotator.antenna_should_be_controlled
                && !state.antenna_rotator.tracking) {
                if (!state.antenna_rotator.fixed_target
                    && !state.antenna_rotator.flip_decision_made) {
                    state.antenna_rotator.flip_mode_pass = 0;
                    state.antenna_rotator.flip_half = 0;
                    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION > 90
                        && state.prediction.predicted_max_elevation
                               >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD) {
                        state.antenna_rotator.flip_mode_pass = 1;
                        state.antenna_rotator.flip_aos_az =
                            state.prediction.satellite_ephem.azimuth;
                    }
                    state.antenna_rotator.flip_decision_made = 1;
                    state.antenna_rotator.tracking = 1;
                }
            }

            if (state.antenna_rotator.tracking
                && state.antenna_rotator.antenna_is_under_control) {
                if (!state.antenna_rotator.unwrapped_target_valid) {
                    if (antenna_rotator_seed_from_status(&state.antenna_rotator)
                        != ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.tracking = 0;
                    }
                } else if (!state.antenna_rotator.antenna_is_moving) {
                    double pred_az = state.prediction.satellite_ephem.azimuth;
                    double pred_el = state.prediction.satellite_ephem.elevation;
                    double mech_az = pred_az;
                    double mech_el = pred_el;
                    int half = 0;
                    antenna_rotator_to_mech_coords(
                        state.antenna_rotator.flip_mode_pass,
                        state.antenna_rotator.flip_aos_az,
                        pred_az, pred_el,
                        &mech_az, &mech_el, &half);
                    if (state.antenna_rotator.flip_mode_pass
                        && half != state.antenna_rotator.flip_half) {
                        state.antenna_rotator.target_azimuth_unwrapped = mech_az;
                        state.antenna_rotator.flip_half = half;
                    }
                    double prev_unwrapped =
                        state.antenna_rotator.target_azimuth_unwrapped;
                    double next_az = antenna_rotator_accumulate_unwrapped(
                        prev_unwrapped, mech_az);
                    double next_el = mech_el;
                    if (next_el < ANTENNA_ROTATOR_MINIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MINIMUM_ELEVATION;
                    } else if (next_el > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                    }
                    delta_az = next_az - prev_unwrapped;
                    if (state.antenna_rotator.flip_mode_pass
                        || state.prediction.satellite_ephem.elevation >= 0) {
                        delta_el = next_el
                                   - state.antenna_rotator.target_elevation;
                    } else {
                        delta_el = 0.0;
                    }

                    if (fabs(delta_az) >= MAX_DELTA_AZIMUTH_DEGREES
                        || fabs(delta_el) >= MAX_DELTA_ELEVATION_DEGREES) {
                        if (next_az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH
                            || next_az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                            state.antenna_rotator.tracking = 0;
                        } else {
                            int rc = antenna_rotator_set_unwrapped(
                                &state.antenna_rotator, next_az, next_el);
                            if (rc != ANTENNA_ROTATOR_OK) {
                                fprintf(stderr,
                                        "Error setting antenna rotator position\n");
                            } else {
                                state.antenna_rotator.antenna_is_moving = 1;
                            }
                        }
                    }
                }
            }

            jul_idle_start = 0;
        } else {
            if (state.in_pass) {
                state.in_pass = 0;
                jul_idle_start = jul_utc;
            }
            if (state.antenna_rotator.tracking) {
                state.antenna_rotator.tracking = 0;
                state.antenna_rotator.flip_mode_pass = 0;
                state.antenna_rotator.flip_decision_made = 0;
                state.antenna_rotator.flip_half = 0;
            }
        }
        (void) jul_idle_start;  // reserved for any future idle-window behavior

        row = 1;
        col = 1;
        report_predictions(&state, jul_utc, &row, col);

        row++;
        report_status(&state, &row, col);
        row = 5;
        col = 50;
        report_position(&state, &row, col);

        clrtoeol();

        key = getch();
        if (key == 'K') {
            keyboard_unlocked = !keyboard_unlocked;
        } else if (keyboard_unlocked) {
            switch (key) {
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
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = antenna_rotator_increase_azimuth(
                        &state.antenna_rotator, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ']':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = antenna_rotator_increase_azimuth(
                        &state.antenna_rotator, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                default:
                    break;
            }
        }

        mvprintw(keyboard_info_row, 3, "%s : %s", "Keyboard",
                 keyboard_unlocked ? "unlocked" : "LOCKED");
        clrtoeol();
        if (state.antenna_rotator.antenna_is_moving) {
            mvprintw(keyboard_info_row + 2, 0, "%s", "Antenna moving");
            clrtoeol();
        } else {
            mvprintw(keyboard_info_row + 2, 0, "%s", "Antenna stationary");
            clrtoeol();
        }

        refresh();

        // --- IPC: serve clients, fan out state, honour SIGUSR1 yield ---
        if (g_ipc) {
            sso_ipc_server_step(g_ipc, 0);
            ipc_broadcast_state(&state, current_az, current_el,
                                 current_downlink_frequency,
                                 doppler_delta_downlink);
        }
        if (g_yield_requested) {
            sso_audit_event("yield-requested",
                            "SIGUSR1 (--force takeover) — exiting");
            state.running = 0;
        }

        if (state.running) {
            usleep(UPDATE_INTERVAL_MICROSEC);
        }
    }

    endwin();
    if (g_ipc) {
        sso_ipc_server_close(g_ipc);
        g_ipc = NULL;
    }

    if (state.prediction.auto_sat) {
        free_passes();
    }

    return 0;
}

// --- apply_args ---------------------------------------------------

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

    state->nominal_uplink_frequency_hz = UPLINK_FREQ_MHZ * 1e6;
    state->nominal_downlink_frequency_hz = DOWNLINK_FREQ_MHZ * 1e6;
    state->doppler_uplink_frequency_hz = state->nominal_uplink_frequency_hz;
    state->doppler_downlink_frequency_hz = state->nominal_downlink_frequency_hz;
    state->doppler_correction_enabled = 1;

    state->run_with_antenna_rotator = 1;
    state->antenna_rotator.device_filename = "/dev/ttyUSB0";
    state->antenna_rotator.serial_speed = B600;
    state->antenna_rotator.fixed_target = 0;

    for (int i = 0; i < argc; i++) {
        if (strncmp("--verbose=", argv[i], 10) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 11) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->verbose_level = atoi(argv[i] + 10);
        } else if (strcmp("--with-rotator", argv[i]) == 0
                || strcmp("--with-hardware", argv[i]) == 0) {
            // Rotator is on by default now. These flags survive as
            // silent no-ops so existing scripts and muscle memory
            // keep working.
            state->n_options++;
            state->run_with_antenna_rotator = 1;
        } else if (strcmp("--without-rotator", argv[i]) == 0
                || strcmp("--without-hardware", argv[i]) == 0) {
            state->n_options++;
            state->run_with_antenna_rotator = 0;
        } else if (strncmp("--tle=", argv[i], 6) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 7) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->prediction.tles_filename = argv[i] + 6;
        } else if (strncmp("--rotator-device=", argv[i], 17) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 18) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->antenna_rotator.device_filename = argv[i] + 17;
        } else if (strncmp("--uplink-freq-mhz=", argv[i], 18) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 19) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->nominal_uplink_frequency_hz = atof(argv[i] + 18) * 1e6;
        } else if (strncmp("--downlink-freq-mhz=", argv[i], 20) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 21) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->nominal_downlink_frequency_hz = atof(argv[i] + 20) * 1e6;
        } else if (strcmp("--no-doppler-correction", argv[i]) == 0) {
            state->n_options++;
            state->doppler_correction_enabled = 0;
        } else if (strncmp("--rotator-target-elevation=", argv[i], 27) == 0) {
            state->n_options++;
            if (strlen(argv[i]) < 28) {
                fprintf(stderr, "Unable to parse %s\n", argv[i]);
                return EXIT_FAILURE;
            }
            state->antenna_rotator.target_elevation = atof(argv[i] + 27);
            if (state->antenna_rotator.target_elevation < 0.0) {
                state->antenna_rotator.target_elevation = 0.0;
            } else if (state->antenna_rotator.target_elevation
                       > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                state->antenna_rotator.target_elevation =
                    ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
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
            min_elevation = atof(argv[i] + 16);
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
        } else if (strcmp("--control", argv[i]) == 0) {
            state->n_options++;
            g_control_mode = 1;
        } else if (strncmp("--", argv[i], 2) == 0) {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            return 3;
        }
    }
    int n_positional = argc - state->n_options - 1;  // -1 for argv[0]
    if (n_positional > 1) {
        usage(stderr, argv[0], 0);
        return 1;
    }

    // Find the (single) positional, if any. Existing convention is
    // "positional at argv[1]" but loop is robust to options-before /
    // options-after orderings.
    char *positional = NULL;
    for (int i = 1; i < argc; i++) {
        if (strncmp("--", argv[i], 2) == 0) continue;
        positional = argv[i];
        break;
    }

    // Bare invocation (no satellite_id, no --control): the standalone
    // tracker is being phased out in favour of the operator+viewer
    // split. Probe for the running operator and bail with a hint
    // either way.
    if (n_positional == 0 && !g_control_mode) {
        sso_ipc_client_t *probe = sso_ipc_client_connect("simple_sat_ops");
        if (probe == NULL) {
            fprintf(stderr,
                "operator not found: try `simple_sat_ops --control` "
                "to operate FrontierSat\n");
            return 1;
        }
        sso_ipc_client_close(probe);
        fprintf(stderr,
            "simple_sat_ops: operator already running; viewer mode "
            "is not implemented yet — nothing to do here.\n");
        return 0;
    }

    state->prediction.observer_ephem.position_geodetic.lat =
        site_latitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.lon =
        site_longitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.alt =
        site_altitude / 1000.0;

    // --control with no positional: auto-discover the newest TLE
    // under /FrontierSat/TLEs/, copy it to the per-user active.tle
    // slot, and read the satellite name out of it.
    if (n_positional == 0 && g_control_mode) {
        if (state->prediction.tles_filename == NULL) {
            const char *tles_root = sso_tles_dir();
            char src_tle[1024];
            time_t src_mtime = 0;
            if (find_newest_tle_recursive(tles_root, src_tle, sizeof src_tle,
                                          &src_mtime) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: --control wants a TLE under %s, "
                    "but no *.tle was found there. Drop one in "
                    "(or pass --tle=<path>).\n", tles_root);
                return EXIT_FAILURE;
            }
            static char active_tle[1024];
            if (tle_default_path(active_tle, sizeof active_tle) != 0) {
                fprintf(stderr,
                    "HOME unset or path too long; pass --tle=<path>\n");
                return EXIT_FAILURE;
            }
            if (copy_file(src_tle, active_tle) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: copy %s -> %s failed: %s\n",
                    src_tle, active_tle, strerror(errno));
                return EXIT_FAILURE;
            }
            fprintf(stderr, "simple_sat_ops: promoted %s -> %s\n",
                    src_tle, active_tle);
            state->prediction.tles_filename = active_tle;
        }
        static char sat_name[64];
        if (read_tle_name(state->prediction.tles_filename,
                          sat_name, sizeof sat_name) != 0) {
            fprintf(stderr,
                "simple_sat_ops: %s has no name line (2-line TLE?); "
                "pass the satellite name explicitly\n",
                state->prediction.tles_filename);
            return EXIT_FAILURE;
        }
        state->prediction.satellite_ephem.name = sat_name;
        fprintf(stderr, "simple_sat_ops: tracking '%s'\n", sat_name);
    } else {
        if (state->prediction.tles_filename == NULL) {
            static char default_tle[1024];
            if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
                fprintf(stderr,
                    "HOME unset or path too long; pass --tle=<path>\n");
                return EXIT_FAILURE;
            }
            state->prediction.tles_filename = default_tle;
        }
        state->prediction.satellite_ephem.name = positional;
    }

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
        prediction_tmp.observer_ephem.position_geodetic.lat =
            state->prediction.observer_ephem.position_geodetic.lat;
        prediction_tmp.observer_ephem.position_geodetic.lon =
            state->prediction.observer_ephem.position_geodetic.lon;
        prediction_tmp.observer_ephem.position_geodetic.alt =
            state->prediction.observer_ephem.position_geodetic.alt;
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

// --- Tracking helpers ---------------------------------------------

void start_tracking(state_t *state)
{
    int antenna_rotator_result = 0;

    state->satellite_tracking = 1;
    state->doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control =
        state->antenna_rotator.antenna_should_be_controlled;
    // Clear the flip latch so the next tracking-enable re-decides for
    // the upcoming pass.
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;
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
}

void stop_tracking(state_t *state)
{
    int antenna_rotator_result = 0;
    double azimuth = 0.0;
    double elevation = 0.0;

    state->satellite_tracking = 0;
    state->doppler_correction_enabled = 1;
    state->antenna_rotator.antenna_is_under_control = 0;
    if (state->run_with_antenna_rotator) {
        antenna_rotator_result = antenna_rotator_command(
            &state->antenna_rotator, ANTENNA_ROTATOR_STOP, &azimuth, &elevation);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error stopping the antenna rotator\n");
        }
        if (antenna_rotator_seed_from_status(&state->antenna_rotator)
            != ANTENNA_ROTATOR_OK) {
            // leave unwrapped_target_valid as it was
        }
    }
    state->antenna_rotator.antenna_is_moving = 0;
    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;
}

int point_to_stationary_target(state_t *state, double azimuth, double elevation)
{
    state->satellite_tracking = 0;
    state->antenna_rotator.antenna_is_under_control = 0;
    state->antenna_rotator.flip_mode_pass = 0;
    state->antenna_rotator.flip_decision_made = 0;
    state->antenna_rotator.flip_half = 0;

    if (!state->antenna_rotator.unwrapped_target_valid) {
        if (antenna_rotator_seed_from_status(&state->antenna_rotator)
            != ANTENNA_ROTATOR_OK) {
            return ANTENNA_ROTATOR_BAD_RESPONSE;
        }
    }

    double prev = state->antenna_rotator.target_azimuth_unwrapped;
    double final_az = antenna_rotator_home_unwrapped_target(prev, azimuth);
    double delta = final_az - prev;

    if (fabs(delta) > 180.0) {
        // Two-step home: halfway waypoint first to disambiguate the
        // direction of rotation; the main loop drives the second leg
        // once the antenna has stopped at the intermediate.
        double mid = prev + delta / 2.0;
        if (mid < ANTENNA_ROTATOR_MINIMUM_AZIMUTH)
            mid = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
        if (mid > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH)
            mid = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
        state->antenna_rotator.home_pending_final_az = final_az;
        state->antenna_rotator.homing_in_progress = 1;
        int rc = antenna_rotator_set_unwrapped(&state->antenna_rotator,
                                                mid, elevation);
        if (rc == ANTENNA_ROTATOR_OK) {
            state->antenna_rotator.antenna_is_moving = 1;
        }
        return rc;
    }

    state->antenna_rotator.homing_in_progress = 0;
    state->antenna_rotator.home_pending_final_az = 0.0;
    int rc = antenna_rotator_set_unwrapped(&state->antenna_rotator,
                                            final_az, elevation);
    if (rc == ANTENNA_ROTATOR_OK) {
        state->antenna_rotator.antenna_is_moving = 1;
    }
    return rc;
}

void update_doppler_shifted_frequencies(state_t *state,
                                          double uplink_freq,
                                          double downlink_freq)
{
    double doppler_factor = 1.0
        - state->prediction.satellite_ephem.range_rate_km_s / 299792.458;
    state->doppler_uplink_frequency_hz   = uplink_freq   * doppler_factor;
    state->doppler_downlink_frequency_hz = downlink_freq * doppler_factor;
}
