/*

   Simple Satellite Operations  control/pass_session.c

   Copyright (C) 2026  Johnathan K Burchill

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

#include "pass_session.h"
#include "state.h"

#include "prediction.h"
#include "sso_audit.h"
#include "sso_paths.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

// --- TLE auto-discovery helpers (used by --control with no
//     positional satellite name) ------------------------------------

// Recursively scan `dir` for *.tle files, return the path with the
// newest mtime via out_path. Returns 0 on success, -1 if dir is
// unreadable or no .tle file exists in the tree. Caller-allocated
// buffer must be at least PATH_MAX-ish.
int find_newest_tle_recursive(const char *dir,
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
int read_tle_name(const char *tle_path,
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

// --- Pass folder for the upcoming pass -----------------------------

// Julian Date -> Unix epoch seconds. Reference: JD 2440587.5 is
// 1970-01-01 00:00:00 UTC, which is what time_t counts from.
static time_t jul_to_unix(double jd)
{
    return (time_t)((jd - 2440587.5) * 86400.0 + 0.5);
}

// Refresh /FrontierSat/Operations/current so it points at `target`.
// Atomically repoint the "current" symlink. If anything fails we log and
// carry on — the pass folder itself is still created and broadcast over IPC,
// the symlink is just a convenience.
static void update_operations_current_symlink(const char *target)
{
    const char *link = sso_operations_current_symlink();
    if (link == NULL || link[0] == '\0') return;
    // Make sure /FrontierSat/Operations/ exists for the symlink slot.
    sso_mkdir_p_for_file(link);
    // Create the new link under a temp name in the same directory, then
    // rename() it onto the slot. rename() over an existing path is atomic, so
    // a concurrent reader never catches the slot momentarily missing the way
    // the old unlink-then-symlink sequence could.
    char tmp[1100];
    if ((size_t) snprintf(tmp, sizeof tmp, "%s.new", link) >= sizeof tmp) {
        fprintf(stderr, "simple_sat_ops: symlink temp path too long for %s\n", link);
        return;
    }
    unlink(tmp);  // clear any leftover from an interrupted previous swap
    if (symlink(target, tmp) != 0) {
        fprintf(stderr,
                "simple_sat_ops: symlink %s -> %s failed: %s "
                "(non-fatal; pass folder still set)\n",
                tmp, target, strerror(errno));
        return;
    }
    if (rename(tmp, link) != 0) {
        fprintf(stderr,
                "simple_sat_ops: rename %s -> %s failed: %s "
                "(non-fatal; pass folder still set)\n",
                tmp, link, strerror(errno));
        unlink(tmp);
    }
}

// Compute a fresh AOS prediction off `state`'s current position and
// Scan a YYYYMMDD parent dir for an existing HHMMLT folder whose
// HHMM is within `window_minutes` of (target_hh, target_mm).
// Restarting simple_sat_ops near a pass time shifts the predicted
// AOS by a minute or two between launches, which would otherwise
// spawn a fresh 1115LT/ alongside the 1114LT/ the operator was
// already writing to. With this lookup the second start reuses
// the existing folder. Returns 0 + fills out_path on a hit; -1 on
// miss (or any I/O failure — caller should fall back to creating
// a fresh folder).
static int find_nearby_pass_folder(const char *parent_dir,
                                   int target_hh, int target_mm,
                                   int window_minutes,
                                   char *out_path, size_t out_path_cap)
{
    DIR *d = opendir(parent_dir);
    if (d == NULL) return -1;
    int best_diff = INT_MAX;
    char best_name[16] = "";
    int target_mod = target_hh * 60 + target_mm;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        // HHMMLT name pattern: exactly 6 chars, [0-9]{4} then "LT".
        const char *n = de->d_name;
        if (strlen(n) != 6) continue;
        if (!isdigit((unsigned char) n[0]) || !isdigit((unsigned char) n[1])
         || !isdigit((unsigned char) n[2]) || !isdigit((unsigned char) n[3])
         || n[4] != 'L' || n[5] != 'T') continue;
        int hh = (n[0] - '0') * 10 + (n[1] - '0');
        int mm = (n[2] - '0') * 10 + (n[3] - '0');
        if (hh >= 24 || mm >= 60) continue;
        int mod = hh * 60 + mm;
        int diff = abs(mod - target_mod);
        if (diff > 12 * 60) diff = 24 * 60 - diff;   // wrap across midnight
        if (diff <= window_minutes && diff < best_diff) {
            best_diff = diff;
            // Filter above guarantees strlen(n) == 6, but gcc can't
            // prove that — pin the width to keep -Wformat-truncation
            // satisfied with our 16-byte best_name.
            snprintf(best_name, sizeof best_name, "%.6s", n);
        }
    }
    closedir(d);
    if (best_name[0] == '\0') return -1;
    int n = snprintf(out_path, out_path_cap, "%s/%s", parent_dir, best_name);
    return (n > 0 && (size_t) n < out_path_cap) ? 0 : -1;
}

// build /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/. Stashes the
// result in state->op.pass_folder so ipc_broadcast_state can publish it on
// every tick.
void setup_pass_folder(state_t *state, double jul_utc_now)
{
    // Handoff case: --pass-folder seeded state->op.pass_folder before we got
    // here. Honour it — make sure the dir exists, refresh the
    // "current" symlink, and skip AOS-discovery entirely.
    if (state->op.pass_folder[0]) {
        if (sso_mkdir_p(state->op.pass_folder) != 0) {
            fprintf(stderr,
                "simple_sat_ops: mkdir -p %s failed: %s\n",
                state->op.pass_folder, strerror(errno));
        }
        update_operations_current_symlink(state->op.pass_folder);
        fprintf(stderr, "simple_sat_ops: using inherited pass folder %s\n",
                state->op.pass_folder);
        {
            char det[600];
            snprintf(det, sizeof det,
                     "mode=inherited path=\"%.500s\"", state->op.pass_folder);
            sso_audit_event("pass-folder", det);
        }
        return;
    }
    // --testing: bench run, not a pass. Land the folder under the
    // sibling Testing/ tree using the CURRENT local time so we don't
    // need a TLE / prediction at all.
    if (state->app.testing_mode) {
        time_t now = time(NULL);
        struct tm now_local;
        localtime_r(&now, &now_local);
        char folder[256];
        int n = snprintf(folder, sizeof folder,
                         "%s/%04d%02d%02d/%02d%02dLT",
                         sso_testing_dir(),
                         now_local.tm_year + 1900,
                         now_local.tm_mon + 1,
                         now_local.tm_mday,
                         now_local.tm_hour,
                         now_local.tm_min);
        if (n <= 0 || (size_t) n >= sizeof folder) {
            fprintf(stderr,
                "simple_sat_ops: --testing folder path too long; skipping\n");
            return;
        }
        if (sso_mkdir_p(folder) != 0) {
            fprintf(stderr,
                "simple_sat_ops: --testing: mkdir -p %s failed: %s\n",
                folder, strerror(errno));
            return;
        }
        snprintf(state->op.pass_folder, sizeof state->op.pass_folder, "%s", folder);
        // Skip update_operations_current_symlink — keep the
        // Operations/current pointer aimed at real passes, not bench
        // runs (avoids confusing operators who scrub recent activity
        // by looking at the symlink).
        fprintf(stderr,
            "simple_sat_ops: --testing folder %s\n", state->op.pass_folder);
        {
            char det[600];
            snprintf(det, sizeof det,
                     "mode=testing path=\"%.500s\"", state->op.pass_folder);
            sso_audit_event("pass-folder", det);
        }
        return;
    }
    minutes_until_visible(&state->track.prediction, jul_utc_now,
                          jul_utc_now + MAX_MINUTES_TO_PREDICT / 1440.0,
                          1.0);
    // minutes_until_visible sets predicted_minutes_until_visible and
    // uses -9999.0 as the "no AOS in this window" sentinel. Positive
    // values are minutes until AOS; negatives are minutes since AOS
    // when we started mid-pass — either way (now + N) lands on the
    // current pass's AOS, which is what we want for the folder name.
    double minutes = state->track.prediction.predicted_minutes_until_visible;
    if (minutes <= -9000.0) {
        fprintf(stderr,
                "simple_sat_ops: no AOS in the next %d minutes — "
                "pass folder not created\n",
                MAX_MINUTES_TO_PREDICT);
        return;
    }
    double aos_jul = jul_utc_now + minutes / 1440.0;
    time_t aos = jul_to_unix(aos_jul);
    struct tm aos_local;
    localtime_r(&aos, &aos_local);
    char parent_dir[256];
    int pn = snprintf(parent_dir, sizeof parent_dir,
                      "%s/%04d%02d%02d",
                      sso_operations_dir(),
                      aos_local.tm_year + 1900,
                      aos_local.tm_mon + 1,
                      aos_local.tm_mday);
    if (pn <= 0 || (size_t) pn >= sizeof parent_dir) {
        fprintf(stderr,
                "simple_sat_ops: pass folder parent path too long; skipping\n");
        return;
    }
    char folder[256];
    // Look for an existing HHMMLT folder for THIS pass within ±10
    // minutes of the predicted AOS — re-runs of simple_sat_ops near
    // a pass time can drift the prediction by a minute or two, and
    // we want to keep recording into the same folder.
    if (find_nearby_pass_folder(parent_dir,
                                aos_local.tm_hour, aos_local.tm_min,
                                10, folder, sizeof folder) == 0) {
        fprintf(stderr,
                "simple_sat_ops: reusing pass folder %s "
                "(predicted AOS %02d:%02dLT within 10 min)\n",
                folder, aos_local.tm_hour, aos_local.tm_min);
    } else {
        int n = snprintf(folder, sizeof folder,
                         "%s/%02d%02dLT",
                         parent_dir,
                         aos_local.tm_hour, aos_local.tm_min);
        if (n <= 0 || (size_t) n >= sizeof folder) {
            fprintf(stderr,
                "simple_sat_ops: pass folder name too long; skipping\n");
            return;
        }
    }
    if (sso_mkdir_p(folder) != 0) {
        fprintf(stderr,
                "simple_sat_ops: mkdir -p %s failed: %s\n",
                folder, strerror(errno));
        return;
    }

    // Pin the TLE the operator just loaded into the pass folder so
    // any post-pass analysis (rx_replay, packet_browser session-dir
    // lookups, whoever shows up later) can find the exact ephemeris
    // that was being tracked, even if active.tle gets rewritten on
    // the next --control startup.
    if (state->track.prediction.tles_filename
        && state->track.prediction.tles_filename[0]) {
        const char *src = state->track.prediction.tles_filename;
        const char *base = strrchr(src, '/');
        base = (base != NULL) ? base + 1 : src;
        char dst[512];
        int rc = snprintf(dst, sizeof dst, "%s/%s", folder, base);
        if (rc > 0 && (size_t)rc < sizeof dst) {
            if (copy_file(src, dst) != 0) {
                fprintf(stderr,
                        "simple_sat_ops: warning: TLE copy %s -> %s "
                        "failed: %s\n",
                        src, dst, strerror(errno));
            } else {
                fprintf(stderr, "simple_sat_ops: pinned TLE %s\n", dst);
            }
        }
    }

    snprintf(state->op.pass_folder, sizeof state->op.pass_folder, "%s", folder);
    update_operations_current_symlink(folder);
    fprintf(stderr, "simple_sat_ops: pass folder %s\n", folder);
    {
        char det[600];
        snprintf(det, sizeof det,
                 "mode=aos path=\"%.500s\"", state->op.pass_folder);
        sso_audit_event("pass-folder", det);
    }
}

// Sample the upcoming pass on a local prediction_t copy and render a
// polar az/el plot to pass_folder/az_el_plot.png via gnuplot. Two
// traces: the satellite's sky position and the rotator boom's beam
// direction (which match for non-flip passes and diverge near apex on
// flip passes -- a visual sanity check of the flip mapping). The
// raw TSV and the gnuplot script are left in the pass folder so the
// operator can rerun or tweak the plot offline.
void generate_pass_plot(state_t *state, const char *pass_folder,
                               double jul_utc_now)
{
    if (!pass_folder || !pass_folder[0]) {
        return;
    }

    // Work on a local copy: update_pass_predictions / update_satellite_position
    // both mutate the prediction's satellite_ephem and aggregate fields.
    prediction_t pred = state->track.prediction;

    // Defensive: handoff case (setup_pass_folder used inherited
    // state->op.pass_folder) leaves predicted_minutes_until_visible stale.
    // Re-run the search so aos_jul below is well defined.
    minutes_until_visible(&pred, jul_utc_now,
                          jul_utc_now + 180.0 / 1440.0, 1.0);
    if (pred.predicted_minutes_until_visible <= -9000.0) {
        fprintf(stderr,
                "simple_sat_ops: no AOS in the next 3 h — skipping plot\n");
        return;
    }
    double aos_jul = jul_utc_now + pred.predicted_minutes_until_visible / 1440.0;

    // Populate predicted_max_elevation / predicted_ascension_azimuth /
    // predicted_pass_duration_minutes on the local copy.
    update_pass_predictions(&pred, aos_jul, 0.1);

    int flip_mode = 0;
    double aos_az = pred.predicted_ascension_azimuth;
    double los_az = pred.predicted_descent_azimuth;
    double aos_jul_pred = pred.predicted_ascension_jul_utc;
    double los_jul_pred = pred.predicted_descent_jul_utc;
    double pass_jul = los_jul_pred - aos_jul_pred;
    if (ANTENNA_ROTATOR_MAXIMUM_ELEVATION > 90
        && pred.predicted_max_elevation
               >= ANTENNA_ROTATOR_FLIP_ELEVATION_THRESHOLD) {
        flip_mode = 1;
    }

    char dat_path[512];
    int n = snprintf(dat_path, sizeof dat_path, "%s/pass_plot.dat",
                     pass_folder);
    if (n <= 0 || (size_t)n >= sizeof dat_path) return;
    FILE *f = fopen(dat_path, "w");
    if (!f) {
        fprintf(stderr, "simple_sat_ops: fopen %s: %s\n",
                dat_path, strerror(errno));
        return;
    }
    fprintf(f, "# t_min\tsat_az\tsat_el\tbeam_az\tbeam_el\n");

    // 10 s cadence over the visible portion. predicted_pass_duration_minutes
    // includes the -5..0 deg pre-AOS / post-LOS buffer, so step a little
    // wider and let the el-filter below drop the wings.
    const double step_min = 10.0 / 60.0;
    const double duration_min = pred.predicted_pass_duration_minutes > 0
        ? pred.predicted_pass_duration_minutes
        : 15.0;
    const int n_steps = (int)(duration_min / step_min) + 1;

    int wrote = 0;
    for (int i = 0; i <= n_steps; ++i) {
        double t_min = i * step_min;
        double sample_jul = aos_jul + t_min / 1440.0;
        update_satellite_position(&pred, sample_jul);
        double sat_az = pred.satellite_ephem.azimuth;
        double sat_el = pred.satellite_ephem.elevation;
        if (sat_el < 0.0) continue;

        double mech_az = sat_az;
        double mech_el = sat_el;
        int half;
        double progress = 0.0;
        if (pass_jul > 0.0) {
            progress = (sample_jul - aos_jul_pred) / pass_jul;
        }
        antenna_rotator_to_mech_coords(flip_mode, aos_az, los_az,
                                       progress,
                                       sat_az, sat_el,
                                       &mech_az, &mech_el, &half);

        // Convert mech back to where the boom's beam actually points
        // on the sky. mech_el > 90 deg means back-pointing through the
        // rotator: equivalent sky direction is (mech_az + 180, 180 -
        // mech_el).
        double beam_az = mech_az;
        double beam_el = mech_el;
        if (beam_el > 90.0) {
            beam_az = fmod(beam_az + 180.0, 360.0);
            if (beam_az < 0.0) beam_az += 360.0;
            beam_el = 180.0 - beam_el;
        }

        fprintf(f, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
                t_min, sat_az, sat_el, beam_az, beam_el);
        ++wrote;
    }
    fclose(f);

    if (wrote == 0) {
        fprintf(stderr,
                "simple_sat_ops: no visible samples in predicted pass — "
                "skipping plot\n");
        return;
    }

    char gp_path[512];
    n = snprintf(gp_path, sizeof gp_path, "%s/pass_plot.gp", pass_folder);
    if (n <= 0 || (size_t)n >= sizeof gp_path) return;
    FILE *gp = fopen(gp_path, "w");
    if (!gp) {
        fprintf(stderr, "simple_sat_ops: fopen %s: %s\n",
                gp_path, strerror(errno));
        return;
    }

    const char *sat_name =
        (state->track.prediction.satellite_ephem.name
         && state->track.prediction.satellite_ephem.name[0])
            ? state->track.prediction.satellite_ephem.name : "satellite";

    // Mirror scripts/plot_sky_pass.sh's polar style so both plots read
    // the same way (N up, E clockwise, zenith at centre). Solid darker
    // grid + elevation-valued rtics so the reticules are readable.
    fprintf(gp,
        "set terminal pngcairo size 900,900 enhanced font 'Helvetica,11'\n"
        "set output '%s/az_el_plot.png'\n"
        "set polar\n"
        "set angles degrees\n"
        "set theta top clockwise\n"
        "set size square\n"
        // Explicit margins so the W cardinal label isn't clipped on the
        // left and the legend has room on the right.
        "set lmargin 8\n"
        "set rmargin 14\n"
        "set tmargin 4\n"
        "set bmargin 4\n"
        "set grid polar 30 lt 1 lw 0.6 lc rgb '#666666'\n"
        "unset border\n"
        "unset xtics\n"
        "unset ytics\n"
        "set rrange [0:90]\n"
        // Label the rings with the elevation they correspond to (r = 90
        // - el): inner ring = 60 deg el, outer ring = 0 deg el (horizon);
        // centre is zenith (90 deg el) and the title says so.
        "set rtics ('60' 30, '30' 60, '0' 90) "
              "font 'Helvetica,9' textcolor rgb '#333333'\n"
        "set ttics ('N' 0, 'E' 90, 'S' 180, 'W' 270) "
              "font 'Helvetica,11,Bold'\n"
        "set key outside right top\n"
        "set title \"%s  max_el=%.1f deg  flip=%s\\n"
                  "(zenith = centre; ring labels = elevation deg)\" noenhanced\n"
        "plot \\\n"
        "  '%s/pass_plot.dat' using 2:(90-$3) "
            "with lines lw 2 lc rgb '#1f77b4' title 'satellite', \\\n"
        "  '' using 4:(90-$5) "
            "with lines lw 2 lc rgb '#d62728' title 'antenna beam'\n",
        pass_folder,
        sat_name,
        pred.predicted_max_elevation,
        flip_mode ? "yes" : "no",
        pass_folder);
    fclose(gp);

    char cmd[1100];
    n = snprintf(cmd, sizeof cmd, "gnuplot '%s' 2>&1", gp_path);
    if (n <= 0 || (size_t)n >= sizeof cmd) return;
    int rc = system(cmd);
    if (rc != 0) {
        fprintf(stderr,
                "simple_sat_ops: gnuplot failed (rc=%d). Install gnuplot "
                "or run manually:  gnuplot '%s'\n",
                rc, gp_path);
    } else {
        fprintf(stderr,
                "simple_sat_ops: pass plot %s/az_el_plot.png "
                "(%d samples, %s)\n",
                pass_folder, wrote,
                flip_mode ? "flip" : "no flip");
    }
}

int pass_session_load_orbit(state_t *state)
{
    // Parse TLE data.
    int tle_status = load_tle(&state->track.prediction);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state->track.prediction.satellite_ephem.tle);

    // Seed the retarget guard with the startup TLE so a `:retarget` on the
    // same file is correctly a no-op.
    snprintf(state->track.target_tle_path, sizeof state->track.target_tle_path, "%s",
             state->track.prediction.tles_filename
                 ? state->track.prediction.tles_filename : "");

    // With a fresh TLE loaded, find the upcoming pass and stand up
    // /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/ for it before the tracking
    // loop opens ncurses. Only on --control — the standalone-tracker / dev
    // path leaves Operations/ alone.
    if (state->app.control_mode) {
        struct tm utc;
        struct timeval tv;
        UTC_Calendar_Now(&utc, &tv);
        double jul_now = Julian_Date(&utc, &tv);
        update_satellite_position(&state->track.prediction, jul_now);
        setup_pass_folder(state, jul_now);
        if (state->op.pass_folder[0]) {
            generate_pass_plot(state, state->op.pass_folder, jul_now);
        }
    }
    return 0;
}
