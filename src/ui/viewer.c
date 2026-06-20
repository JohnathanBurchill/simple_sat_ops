/*

   Simple Satellite Operations  ui/viewer.c

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

#include "viewer.h"
#include "state.h"

#include "panels.h"
#include "tui.h"
#include "tx_log.h"
#include "ipc_fill.h"        // ipc_fill_state_prediction
#include "prediction.h"      // update_satellite_position, free_passes
#include "pass_schedule.h"   // pass_schedule_t, pass_schedule_encode
#include "tracking.h"        // update_doppler_shifted_frequencies
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_ipc_paths.h"
#include "sso_dirwatch.h"
#include "sso_operator.h"

#include <errno.h>
#include <ncurses.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

// --- Viewer mode --------------------------------------------------
//
// Read-only mirror of the operator instance. The viewer does NOT run
// SGP4 and does NOT load a TLE — it just deposits every broadcast
// field into a state_t and calls the same render helpers the operator
// uses, so the two displays are byte-identical except for the help text.

// All viewer-mode state in one struct, owned as a local in run_viewer and
// threaded by pointer through the viewer helpers (viewer_on_event receives
// it via the IPC callback's user channel). Kept separate from state_t: the
// viewer mirrors an operator's broadcast into `state` and never runs the
// tracker itself.
typedef struct viewer {
    int    event_pending;
    int    has_state;
    char   operator[64];
    char   roster_json[1024];
    time_t last_event;
    int    running;
    // Mirror of the operator's ":" prompt state. cmd_active = 1 between
    // the first cmd-preview after ':' and the cmd-executed that closes it.
    // cmd_buf and cmd_status track the operator's verbatim so the viewer's
    // bottom row matches exactly. Sized to the wire field (cmd_text).
    int    cmd_active;
    char   cmd_buf[160];
    char   cmd_status[160];
    // Mirror of the operator's auto-tcmd run. auto_on = 1 while the
    // operator has a run to show; the render line is "<sent>/<total>
    // sent (<state>)" and disappears when the operator closes the modal.
    int    auto_on;
    int    auto_sent;
    int    auto_total;
    char   auto_state[12];
    // Mirror of the operator's RX panel. Filled from STATE / WELCOME events;
    // render_rx_panel reads it directly during viewer_render.
    rx_panel_data_t rx_panel;
    // state_t whose fields the viewer mirrors from the broadcast each tick.
    state_t state;
    double carrier_hz;
    double jul_utc;
    int    has_rotator;
    char   tle_path[256];
    char   pass_folder[256];
    // Take-control confirmation. Press 'c' once to arm, 'y' within
    // VIEWER_CONFIRM_WINDOW_S seconds to commit. Anything else cancels.
    time_t confirm_until;
} viewer_t;

#define VIEWER_CONFIRM_WINDOW_S 5

static void viewer_on_event(sso_ipc_client_t *cli, const sso_event_t *evt,
                            void *user)
{
    (void) cli;
    viewer_t *v = (viewer_t *) user;
    if (evt->type == SSO_EVT_TX_COMMAND_PREVIEW
     || evt->type == SSO_EVT_TX_COMMAND_SENT
     || evt->type == SSO_EVT_TX_NOT_SENT) {
        tx_log_push(&v->state, evt);
        v->last_event = time(NULL);
        v->event_pending = 1;
        return;
    }
    if (evt->type == SSO_EVT_CMD_PREVIEW) {
        v->cmd_active = 1;
        snprintf(v->cmd_buf, sizeof v->cmd_buf,
                 "%s", evt->cmd_text);
        v->last_event = time(NULL);
        v->event_pending = 1;
        return;
    }
    if (evt->type == SSO_EVT_CMD_EXECUTED) {
        // Empty cmd_text + empty cmd_status = Esc/cancel; clear the row.
        // Otherwise show the executed-command result string just like the
        // operator does after cmd_dispatch returns.
        v->cmd_active = 0;
        v->cmd_buf[0] = '\0';
        snprintf(v->cmd_status, sizeof v->cmd_status,
                 "%s", evt->cmd_status);
        v->last_event = time(NULL);
        v->event_pending = 1;
        return;
    }
    if (evt->type != SSO_EVT_STATE && evt->type != SSO_EVT_WELCOME) {
        return;
    }
    v->last_event = time(NULL);
    if (evt->operator_user[0]) {
        snprintf(v->operator, sizeof v->operator, "%s",
                 evt->operator_user);
    }
    if (evt->roster_json[0]) {
        snprintf(v->roster_json, sizeof v->roster_json, "%s",
                 evt->roster_json);
    }
    if (!evt->has_state) return;

    state_t *s = &v->state;
    snprintf(s->prediction.satellite_ephem.tle.sat_name,
             sizeof s->prediction.satellite_ephem.tle.sat_name, "%s",
             evt->satellite);
    snprintf(s->prediction.satellite_ephem.tle.idesg,
             sizeof s->prediction.satellite_ephem.tle.idesg, "%s",
             evt->idesg);
    s->prediction.minutes_since_epoch              = evt->epoch_min;
    s->prediction.predicted_minutes_until_visible  = evt->min_visible;
    s->prediction.predicted_minutes_above_0_degrees  = evt->min_above_0;
    s->prediction.predicted_minutes_above_30_degrees = evt->min_above_30;
    s->prediction.predicted_max_elevation          = evt->max_el;
    s->prediction.satellite_ephem.azimuth          = evt->pred_az;
    s->prediction.satellite_ephem.elevation        = evt->pred_el;
    s->prediction.satellite_ephem.altitude_km      = evt->alt_km;
    s->prediction.satellite_ephem.latitude         = evt->lat_deg;
    s->prediction.satellite_ephem.longitude        = evt->lon_deg;
    s->prediction.satellite_ephem.speed_km_s       = evt->speed_kms;
    s->prediction.satellite_ephem.range_km         = evt->range_km;
    s->prediction.satellite_ephem.range_rate_km_s  = evt->range_rate_kms;
    s->in_pass                                     = evt->in_pass;
    s->antenna_rotator.tracking                    = evt->tracking;
    s->antenna_rotator.azimuth                     = evt->az;
    s->antenna_rotator.elevation                   = evt->el;
    s->antenna_rotator.target_azimuth              = evt->target_az;
    s->antenna_rotator.target_elevation            = evt->target_el;
    s->antenna_rotator.flip_mode_pass              = evt->flip;

    v->has_rotator = evt->has_rotator;
    v->jul_utc     = evt->jul_utc;
    v->carrier_hz  = (evt->doppler_hz != 0.0)
        ? (double)evt->freq_hz + evt->doppler_hz
        : (double)evt->freq_hz;
    if (evt->tle_path[0]) {
        snprintf(v->tle_path, sizeof v->tle_path,
                 "%s", evt->tle_path);
    }
    if (evt->pass_folder[0]) {
        snprintf(v->pass_folder, sizeof v->pass_folder,
                 "%s", evt->pass_folder);
    }

    // Auto-tcmd progress — stashed unconditionally so a broadcast
    // without the fields (run over, modal closed) clears the line.
    v->auto_on    = evt->auto_tcmd_on;
    v->auto_sent  = evt->auto_tcmd_sent;
    v->auto_total = evt->auto_tcmd_total;
    snprintf(v->auto_state, sizeof v->auto_state,
             "%s", evt->auto_tcmd_state);

    // Mirror the operator's RX panel from the broadcast. Wipe to zero
    // first so a slot that the operator hasn't decoded in this run
    // doesn't carry stale state from a previous event.
    memset(&v->rx_panel, 0, sizeof v->rx_panel);
    v->rx_panel.have_session  = evt->rx_have_session;
    snprintf(v->rx_panel.warning, sizeof v->rx_panel.warning,
             "%s", evt->rx_warning);
    if (evt->rx_have_session) {
        v->rx_panel.rec_active     = evt->rx_rec_active;
        v->rx_panel.rx_freq_hz     = evt->rx_freq_hz;
        v->rx_panel.peak_dbfs      = evt->rx_peak_dbfs;
        v->rx_panel.rms_dbfs       = evt->rx_rms_dbfs;
        v->rx_panel.frames_total   = (uint64_t) evt->rx_frames_total;
        v->rx_panel.frames_pcm     = (uint64_t) evt->rx_frames_pcm;
        v->rx_panel.frames_vit     = (uint64_t) evt->rx_frames_vit;
        snprintf(v->rx_panel.last_frame_summary,
                 sizeof v->rx_panel.last_frame_summary,
                 "%s", evt->rx_last_frame_summary);
        v->rx_panel.age_s = evt->rx_age_s;
        int slots = RX_PANEL_PT_COUNT < SSO_RX_PT_SLOTS
                  ? RX_PANEL_PT_COUNT : SSO_RX_PT_SLOTS;
        for (int s = 0; s < slots; ++s) {
            v->rx_panel.pt_count[s] = (uint64_t) evt->rx_pt_count[s];
            int pl = evt->rx_pt_payload_len[s];
            if (pl < 0) pl = 0;
            int copy = pl;
            if (copy > RX_PANEL_PAYLOAD_MAX) copy = RX_PANEL_PAYLOAD_MAX;
            v->rx_panel.pt_payload_len[s] = pl;
            memcpy(v->rx_panel.pt_payload[s],
                   evt->rx_pt_payload[s], (size_t) copy);
            snprintf(v->rx_panel.pt_summary[s],
                     sizeof v->rx_panel.pt_summary[s],
                     "%.*s",
                     (int)(sizeof v->rx_panel.pt_summary[s] - 1),
                     evt->rx_pt_summary[s]);
        }
        int rn = evt->rx_ribbon_n;
        // Clamp both ends: rx_ribbon_n arrives from a peer broadcast, so a
        // garbled negative value would otherwise become a huge size_t in the
        // memcpy below and a negative ribbon[] index.
        if (rn < 0) rn = 0;
        if (rn > RIBBON_LEN) rn = RIBBON_LEN;
        v->rx_panel.ribbon_n = rn;
        memcpy(v->rx_panel.ribbon, evt->rx_ribbon, (size_t) rn);
        v->rx_panel.ribbon[rn] = '\0';
        memcpy(v->rx_panel.ribbon_peak, evt->rx_ribbon_peak,
               (size_t) rn * sizeof v->rx_panel.ribbon_peak[0]);
    }

    v->has_state   = 1;
    v->event_pending = 1;
}

// Format the roster array into "alice,bob,carol" for the header bar,
// skipping the operator (already shown separately) and any entry whose
// user is empty. The roster JSON is built by sso_event_set_roster with
// the schema [{"user":"...","role":"...","since":"..."},...], so we
// can scan for "user":"..." and "role":"..." pairs.
static void viewer_roster_users(viewer_t *v, char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    const char *p = v->roster_json;
    size_t written = 0;
    while ((p = strstr(p, "\"user\":\"")) != NULL) {
        p += 8;
        const char *uend = strchr(p, '"');
        if (!uend) break;
        char name[64];
        size_t nlen = (size_t)(uend - p);
        if (nlen >= sizeof name) nlen = sizeof name - 1;
        memcpy(name, p, nlen);
        name[nlen] = '\0';
        const char *q = strstr(uend, "\"role\":\"");
        const char *next = strstr(uend, "\"user\":\"");
        // role must belong to this object (before the next "user")
        int is_op = 0;
        if (q && (!next || q < next)) {
            q += 8;
            const char *rend = strchr(q, '"');
            if (rend && (size_t)(rend - q) == 8
                && memcmp(q, "operator", 8) == 0) {
                is_op = 1;
            }
        }
        p = uend;
        if (is_op || nlen == 0) continue;
        size_t need = nlen + (written > 0 ? 1 : 0);
        if (written + need + 1 >= out_size) break;
        if (written > 0) out[written++] = ',';
        memcpy(out + written, name, nlen);
        written += nlen;
        out[written] = '\0';
    }
}

// Read the operator's PID from /run/sso/simple_sat_ops.pid. Returns 0
// and sets *out_pid on success; -1 on missing/unreadable file.
int read_operator_pid(pid_t *out_pid)
{
    char path[256];
    if (sso_ipc_pid_path(path, sizeof path, "simple_sat_ops") != 0)
        return -1;
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int pid = 0;
    int got = fscanf(f, "%d", &pid);
    fclose(f);
    if (got != 1 || pid <= 0) return -1;
    *out_pid = (pid_t) pid;
    return 0;
}

// Hand-off: send SIGUSR1 to the running operator, wait until its
// socket file goes away (= the process has finished cleanup), then
// re-exec this process as `simple_sat_ops --control` with the
// inherited TLE + pass folder so the new operator picks up where the
// old one left off. Does not return on success.
static void viewer_take_control(viewer_t *v, sso_ipc_client_t *cli, const char *argv0)
{
    if (!v->tle_path[0]) {
        fprintf(stderr,
            "simple_sat_ops viewer: no TLE path from operator yet — "
            "wait for a state broadcast and try again.\n");
        return;
    }
    pid_t op_pid = 0;
    if (read_operator_pid(&op_pid) != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: couldn't read operator pid file.\n");
        return;
    }
    if (kill(op_pid, SIGUSR1) != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: kill(%d, SIGUSR1) failed: %s\n",
            (int) op_pid, strerror(errno));
        return;
    }

    // Wait for the old operator to clean up and remove its socket.
    char sock[256];
    if (sso_ipc_socket_path(sock, sizeof sock, "simple_sat_ops") != 0) {
        fprintf(stderr,
            "simple_sat_ops viewer: socket path lookup failed.\n");
        return;
    }
    int waited_ms = 0;
    for (;;) {
        struct stat st;
        if (stat(sock, &st) != 0 && errno == ENOENT) break;
        if (waited_ms >= 5000) {
            fprintf(stderr,
                "simple_sat_ops viewer: timed out waiting for operator "
                "to release the socket (%s)\n", sock);
            return;
        }
        usleep(100000);
        waited_ms += 100;
    }

    // Close our viewer IPC + curses cleanly before exec'ing.
    sso_ipc_client_close(cli);
    endwin();
    tui_release_stderr();

    // Re-exec self as --control with inherited TLE and pass folder.
    // Filename args use the space form so the spawned child sees the
    // same TAB-completable spelling the user types.
    char self_path[1024];
    ssize_t n = readlink("/proc/self/exe", self_path, sizeof self_path - 1);
    const char *exe = (n > 0) ? (self_path[n] = '\0', self_path) : argv0;
    char *new_argv[8];
    int ai = 0;
    new_argv[ai++] = (char *) exe;
    new_argv[ai++] = "--control";
    new_argv[ai++] = "--tle";
    new_argv[ai++] = v->tle_path;
    if (v->pass_folder[0]) {
        new_argv[ai++] = "--pass-folder";
        new_argv[ai++] = v->pass_folder;
    }
    new_argv[ai] = NULL;
    fprintf(stderr,
        "simple_sat_ops: taking control with --tle %s%s%s\n",
        v->tle_path,
        v->pass_folder[0] ? "  --pass-folder " : "",
        v->pass_folder[0] ? v->pass_folder : "");
    execv(exe, new_argv);
    // If we got here exec failed — best to bail loudly.
    fprintf(stderr,
        "simple_sat_ops viewer: execv %s failed: %s\n",
        exe, strerror(errno));
    exit(EXIT_FAILURE);
}

static void viewer_render(viewer_t *v, int connected)
{
    int cols = COLS;
    erase();

    if (!v->has_state) {
        mvprintw(2, 2, "(waiting for state from the operator...)");
    } else {
        int row = 1, col = 1;
        render_predictions_panel(&v->state, v->jul_utc,
                                 &row, col);

        char viewers[160];
        viewer_roster_users(v, viewers, sizeof viewers);
        int srow = row + 1;
        status_panel_t sp;
        memset(&sp, 0, sizeof sp);
        sp.control_mode  = 0;
        sp.operator_user = v->operator;
        sp.viewers       = viewers[0] ? viewers : "(none)";
        sp.carrier_hz    = v->carrier_hz;
        sp.have_rotator  = v->has_rotator;
        sp.current_az    = v->state.antenna_rotator.azimuth;
        sp.current_el    = v->state.antenna_rotator.elevation;
        sp.target_az     = v->state.antenna_rotator.target_azimuth;
        sp.target_el     = v->state.antenna_rotator.target_elevation;
        sp.flip          = v->state.antenna_rotator.flip_mode_pass;
        render_status_panel(&sp, &srow, col);

        // Auto-tcmd run progress, mirrored from the operator's modal.
        // Red while the run is live (the PA is being keyed on a timer),
        // matching the T/R panel's red-while-transmitting convention.
        if (v->auto_on) {
            srow++;
            int at_running = strcmp(v->auto_state, "running") == 0;
            if (at_running) attron(COLOR_PAIR(1));
            mvprintw(srow++, col, "%15s   %d/%d sent (%s)",
                     "auto-tcmd",
                     v->auto_sent, v->auto_total,
                     v->auto_state[0] ? v->auto_state : "?");
            if (at_running) attroff(COLOR_PAIR(1));
            clrtoeol();
        }

        int prow = 5;
        report_position(&v->state, &prow, 50);
        // RX panel directly below position (matches the operator's layout).
        prow++;
        render_rx_panel(&v->rx_panel, &prow, 50);

        // Vertical ribbon on the right edge, same placement as the
        // operator. The wire delivers the same '.'/'-' chars the
        // operator's collector built so both screens crawl in sync.
        int ribbon_col = COLS - 2;
        int ribbon_top = 1;
        int ribbon_bot = LINES - 2;
        if (ribbon_col >= 64 && ribbon_bot > ribbon_top) {
            render_ribbon_vertical(&v->rx_panel,
                                   ribbon_top, ribbon_bot, ribbon_col);
        }

        int tx_log_row = LINES - TX_LOG_SIZE - 2;
        if (tx_log_row >= 17) {
            render_tx_log_panel(&v->state, tx_log_row, 1);
        }
    }

    time_t now = time(NULL);
    long stale_s = v->last_event > 0
        ? (long)(now - v->last_event)
        : -1;
    const char *status = !connected ? "DISCONNECTED"
                                    : (stale_s < 0 ? "WAITING"
                                                   : (stale_s > 5 ? "STALE"
                                                                  : "LIVE"));

    // Bottom row priority: take-control confirm > cmd mirror > footer.
    // The cmd mirror reproduces exactly what the operator sees on its
    // own LINES-1; the footer (connection status + viewer-only shortcuts)
    // is the fallback when neither is active. The mirror does not invert
    // the row — matches the operator's plain ":" prompt rendering.
    int show_confirm = (v->confirm_until > 0
                        && now < v->confirm_until);
    int show_mirror  = !show_confirm
        && (v->cmd_active || v->cmd_status[0]);

    move(LINES - 1, 0);
    clrtoeol();
    if (show_mirror) {
        if (v->cmd_active) {
            mvprintw(LINES - 1, 0, ":%s", v->cmd_buf);
            addch(' ' | A_REVERSE);
        } else {
            mvprintw(LINES - 1, 0, "%s", v->cmd_status);
        }
    } else {
        attron(A_REVERSE);
        char foot[200];
        if (show_confirm) {
            snprintf(foot, sizeof foot,
                " %s     Take control from %s? y/N ",
                status,
                v->operator[0] ? v->operator : "?");
        } else {
            snprintf(foot, sizeof foot,
                " %s     c : take control   q : quit ", status);
        }
        int flen = (int)strlen(foot);
        if (flen > cols) flen = cols;
        mvaddnstr(LINES - 1, 0, foot, flen);
        for (int i = flen; i < cols; i++) mvaddch(LINES - 1, i, ' ');
        attroff(A_REVERSE);
    }

    refresh();
}

int run_viewer(const char *argv0)
{
    viewer_t v = {0};
    v.running = 1;
    sso_ipc_client_t *cli = sso_ipc_client_connect("simple_sat_ops");
    if (cli == NULL) {
        fprintf(stderr,
                "simple_sat_ops viewer: connect failed: %s\n",
                strerror(errno));
        return EXIT_FAILURE;
    }
    sso_ipc_client_on_event(cli, viewer_on_event, &v);

    // Viewer doesn't run SGP4 or load a TLE — it deposits every
    // displayed value into v.state from the broadcast and uses
    // the same render helpers the operator does. zero-init is enough.
    memset(&v.state, 0, sizeof v.state);

    sso_event_t hello;
    sso_event_init(&hello, SSO_EVT_HELLO);
    snprintf(hello.role, sizeof hello.role, "viewer");
    const char *me = getenv("USER");
    if (!me || !me[0]) me = sso_unix_user();
    snprintf(hello.user, sizeof hello.user, "%s", me ? me : "?");
    char buf[1024];
    if (sso_event_encode(&hello, buf, sizeof buf) == 0) {
        sso_ipc_client_send(cli, buf);
    }

    init_window(&v.state);
    int last_connected = -1;
    time_t last_render = 0;
    viewer_render(&v, sso_ipc_client_is_connected(cli));
    last_render = time(NULL);
    int confirm_was_armed = 0;
    while (v.running) {
        int rc = sso_ipc_client_step(cli, 200);
        if (rc < 0) break;
        int connected = sso_ipc_client_is_connected(cli);
        time_t now = time(NULL);
        int confirm_armed = (v.confirm_until > 0
                             && now < v.confirm_until);
        if (!confirm_armed && confirm_was_armed) {
            // Window just expired — re-render to drop the confirm footer.
            v.event_pending = 1;
        }
        confirm_was_armed = confirm_armed;
        if (v.event_pending
            || connected != last_connected
            || (now - last_render) >= 5) {
            viewer_render(&v, connected);
            v.event_pending = 0;
            last_connected = connected;
            last_render = now;
        }
        int key = getch();
        if (key == ERR) continue;
        if (confirm_armed) {
            if (key == 'y' || key == 'Y') {
                // Commits: viewer_take_control re-execs on success and
                // doesn't return; if it returns, the operator wasn't
                // reachable and we just stay as a viewer.
                viewer_take_control(&v, cli, argv0);
                v.confirm_until = 0;
                v.event_pending = 1;
            } else {
                // Anything else cancels the confirm window.
                v.confirm_until = 0;
                v.event_pending = 1;
            }
            continue;
        }
        if (key == 'q' || key == 'Q' || key == 27 /* Esc */) {
            v.running = 0;
        } else if (key == 'c' || key == 'C') {
            v.confirm_until = now + VIEWER_CONFIRM_WINDOW_S;
            v.event_pending = 1;
        }
    }

    endwin();
    tui_release_stderr();
    sso_ipc_client_close(cli);
    return 0;
}

// --- --viewer-stream: headless JSON stream ------------------------
//
// Two modes, auto-switching, writing newline-JSON to stdout (no ncurses,
// no hardware):
//   TLE-only — no operator running. Propagate the already-loaded TLE
//     locally (SGP4) and emit STATE lines tagged source="tle-only" at
//     ~1 Hz. Probe for an operator every 30 s.
//   operator — an operator is up. Connect as an "external" client and
//     relay its broadcast to stdout, re-tagging STATE/WELCOME with
//     source="operator". On disconnect, fall back to TLE-only.
// The source tag on every STATE line is how the reader tells which mode
// produced the data. The TLE-only stream is continuous, so a live link is
// always evident from the flowing lines.

// STATE emit cadence in TLE-only mode (seconds).
#define STREAM_TLE_PERIOD_S    1
// Backstop operator-probe cadence (seconds). The runtime-dir watch is the
// fast path — it wakes us the instant a --control operator binds its socket
// — so this only covers the first attempt, a host with no watch backend,
// and any change the watch happened to miss. A failed connect to a missing
// socket returns immediately, so a short period is cheap.
#define STREAM_PROBE_PERIOD_S  5
// Idle wait between TLE-only ticks (milliseconds). We block here on the
// runtime-dir watch so a newly-bound operator socket wakes us early.
#define STREAM_IDLE_MS         200

static volatile sig_atomic_t g_stream_stop = 0;
static void stream_on_signal(int sig) { (void) sig; g_stream_stop = 1; }

// Write one already-encoded line (it ends in '\n') to stdout and flush.
// A write error — typically the SSH reader closing our stdout — flags a
// stop so the loop unwinds cleanly instead of spinning on a dead pipe.
static void stream_emit(const char *line)
{
    fputs(line, stdout);
    fflush(stdout);
    if (ferror(stdout)) g_stream_stop = 1;
}

// Resolve the Unix user once, for the HELLO / operator_user field.
static const char *stream_user(void)
{
    const char *me = getenv("USER");
    if (!me || !me[0]) me = sso_unix_user();
    return me ? me : "?";
}

// IPC callback for operator-relay mode: re-encode each operator event to
// stdout, stamping STATE/WELCOME with source="operator" so the reader can
// tell relayed data from the local TLE-only stream. Re-encoding (rather
// than forwarding raw bytes) keeps both modes' STATE lines identically
// shaped — the relay is built from the same tree as the operator, so the
// codec round-trip drops nothing.
static void stream_relay_on_event(sso_ipc_client_t *cli,
                                  const sso_event_t *evt, void *user)
{
    (void) cli;
    (void) user;
    sso_event_t e = *evt;
    if (e.type == SSO_EVT_STATE || e.type == SSO_EVT_WELCOME) {
        snprintf(e.source, sizeof e.source, "operator");
    }
    char line[8192];
    if (sso_event_encode(&e, line, sizeof line) == 0) {
        stream_emit(line);
    }
}

// Build + emit one TLE-only STATE line off the current prediction. No
// rotator / radio / RX in this mode: az/el and has_rotator stay 0 and the
// satellite's sky position rides pred_az/pred_el. freq carries the
// Doppler-shifted downlink so the stream shows the same carrier the
// operator would; in_pass reflects the live above-horizon geometry.
static void stream_emit_tle_state(state_t *state, double jul_utc)
{
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_STATE);
    evt.has_state = 1;
    snprintf(evt.source, sizeof evt.source, "tle-only");
    snprintf(evt.operator_user, sizeof evt.operator_user, "%s", stream_user());
    if (state->prediction.tles_filename) {
        snprintf(evt.tle_path, sizeof evt.tle_path, "%s",
                 state->prediction.tles_filename);
    }
    ipc_fill_state_prediction(&state->prediction, &evt);
    evt.jul_utc = jul_utc;
    evt.freq_hz = (long) state->doppler_downlink_frequency_hz;
    evt.in_pass = (state->prediction.satellite_ephem.elevation > 0.0);

    char line[8192];
    if (sso_event_encode(&evt, line, sizeof line) == 0) {
        stream_emit(line);
    }
}

// --- upcoming-passes schedule -------------------------------------
//
// Compute the next week of passes off the loaded TLE and ship them once per
// session as a {"t":"passes"} line, so the standalone viewer can show a pass
// list and fire local alerts with no live link. Independent of operator vs
// tle-only mode (the relay always loads its own orbit), so it's emitted before
// the main loop and refreshed periodically. Read-only: a copy of the
// prediction is used so the relay's live state is never perturbed.

// How far ahead to enumerate passes, and the refresh cadence for long-lived
// tle-only streams (operator sessions are short and parked in the relay, so the
// startup emit covers them).
#define STREAM_PASSES_HORIZON_DAYS 7.0
#define STREAM_PASSES_PERIOD_S     (6 * 3600)
// Step (minutes) for the culmination walk that recovers peak az/time.
#define STREAM_PASS_PEAK_STEP_MIN  0.1

// sgp4sdp4 Julian Date (full JD) -> Unix seconds. Matches next_in_queue's
// format_local_aos and the Date_Time epoch (JD 2440587.5 = 1970-01-01).
static double jul_to_unix(double jul) { return (jul - 2440587.5) * 86400.0; }

// Fill `out` with up to PASS_SCHED_MAX upcoming passes. Returns the count.
static int build_pass_schedule(const state_t *state, double jul_now,
                               pass_schedule_t *out)
{
    memset(out, 0, sizeof *out);

    // Work on a copy so the relay's live prediction is never touched. The TLE
    // was already converted (select_ephemeris ran in pass_session_load_orbit)
    // and the deep-space flag is set globally, so we propagate as-is — exactly
    // like stream_emit_tle_state does (we must NOT select_ephemeris again).
    prediction_t pred;
    memcpy(&pred, &state->prediction, sizeof pred);

    const char *name = pred.oem
        ? (pred.satellite_ephem.name ? pred.satellite_ephem.name : "")
        : pred.satellite_ephem.tle.sat_name;
    if (name == NULL || name[0] == '\0') return 0;   // nothing to schedule
    // Bound the field width so -Wformat-truncation knows it fits: the source
    // (tle.sat_name) is char[128], the wire field is char[64].
    snprintf(out->satellite, sizeof out->satellite, "%.*s",
             (int)(sizeof out->satellite - 1), name);
    if (!pred.oem) {
        snprintf(out->idesg, sizeof out->idesg, "%s",
                 pred.satellite_ephem.tle.idesg);
        double jul_epoch = Julian_Date_of_Epoch(pred.satellite_ephem.tle.epoch);
        out->tle_epoch_min = (jul_now - jul_epoch) * 1440.0;
    }
    out->generated_unix = jul_to_unix(jul_now);

    double jul_stop = jul_now + STREAM_PASSES_HORIZON_DAYS;
    double utc_offset_min = 0.0;
    int n = 0;
    while (n < PASS_SCHED_MAX &&
           get_next_pass(&pred, jul_now + utc_offset_min / 1440.0, jul_stop, 1.0)) {
        // Advance past this pass for the next search (mirrors find_passes):
        // 60 min after this AOS clears the (sub-hour) pass.
        utc_offset_min += pred.predicted_minutes_until_visible + 60.0;
        if (pred.predicted_minutes_above_0_degrees <= 0.0) continue;
        double aos_jul = pred.predicted_ascension_jul_utc;
        double los_jul = pred.predicted_descent_jul_utc;
        if (aos_jul <= 0.0 || los_jul <= aos_jul) continue;

        // get_next_pass yields AOS/LOS + max elevation but not the azimuth or
        // time at culmination; walk the pass to capture them.
        double best_el = -90.0, best_az = 0.0, best_jul = aos_jul;
        for (double j = aos_jul; j <= los_jul + 1e-9;
             j += STREAM_PASS_PEAK_STEP_MIN / 1440.0) {
            update_satellite_position(&pred, j);
            double el = pred.satellite_ephem.elevation;
            if (el > best_el) {
                best_el  = el;
                best_az  = pred.satellite_ephem.azimuth;
                best_jul = j;
            }
        }

        pass_t_wire *p = &out->passes[n++];
        p->aos_unix    = jul_to_unix(aos_jul);
        p->los_unix    = jul_to_unix(los_jul);
        p->peak_unix   = jul_to_unix(best_jul);
        p->peak_el_deg = best_el;
        p->peak_az_deg = best_az;
    }
    out->count = n;
    return n;
}

static void stream_emit_passes(state_t *state)
{
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_now = Julian_Date(&utc, &tv);
    pass_schedule_t sched;
    if (build_pass_schedule(state, jul_now, &sched) <= 0) return;
    char line[16384];
    if (pass_schedule_encode(&sched, line, sizeof line) == 0) {
        stream_emit(line);
    }
}

// One pass through operator-relay mode: HELLO as an "external" client,
// then forward the operator's broadcast until it drops or we're asked to
// stop. Returns when the link is gone so the caller falls back to TLE-only.
static void stream_relay_session(sso_ipc_client_t *cli)
{
    sso_ipc_client_on_event(cli, stream_relay_on_event, NULL);
    sso_event_t hello;
    sso_event_init(&hello, SSO_EVT_HELLO);
    snprintf(hello.role, sizeof hello.role, "external");
    snprintf(hello.user, sizeof hello.user, "%s", stream_user());
    char buf[1024];
    if (sso_event_encode(&hello, buf, sizeof buf) == 0) {
        sso_ipc_client_send(cli, buf);
    }
    while (!g_stream_stop) {
        int rc = sso_ipc_client_step(cli, 500);
        if (rc != 0) break;  // 1 = operator gone, -1 = fatal
    }
}

// Try once to attach to a running operator and relay its broadcast until it
// drops. Returns 1 if we connected (caller resumes TLE-only afterward), 0 if
// no operator was reachable.
static int stream_try_relay(void)
{
    sso_ipc_client_t *cli = sso_ipc_client_connect("simple_sat_ops");
    if (cli == NULL) return 0;
    stream_relay_session(cli);
    sso_ipc_client_close(cli);
    return 1;
}

int run_viewer_stream(state_t *state)
{
    // A closed stdout (SSH reader gone) must surface as a write error we
    // can act on, not a SIGPIPE that kills us mid-line — and we may stream
    // for a while before ever connecting a client, so don't rely on the
    // connect path to have ignored it. SIGTERM/SIGINT stop us cleanly.
    signal(SIGPIPE, SIG_IGN);
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = stream_on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);

    // Watch the runtime dir so we connect the instant a --control operator
    // binds its socket, instead of waiting out the backstop probe. The dir
    // must exist to watch it; the operator would create it anyway, and on a
    // shared ground station it's already present. A NULL watch (no inotify /
    // kqueue backend) just means we lean on the periodic probe alone.
    sso_ipc_ensure_runtime_dir();
    sso_dirwatch_t *watch = sso_dirwatch_open(sso_ipc_runtime_dir());

    time_t last_emit  = 0;   // 0 => emit on the first TLE-only tick
    time_t last_probe = 0;   // 0 => probe on the first iteration

    // Ship the upcoming-passes schedule up front so every viewer (each SSH
    // connection spawns its own stream) gets it near the start, in either
    // mode — operator-relay mode parks inside stream_try_relay, so the loop's
    // periodic refresh below only runs while we're in tle-only mode.
    stream_emit_passes(state);
    time_t last_passes = time(NULL);

    while (!g_stream_stop) {
        time_t now = time(NULL);

        // Refresh the pass schedule periodically (long-lived tle-only streams).
        if (now - last_passes >= STREAM_PASSES_PERIOD_S) {
            stream_emit_passes(state);
            last_passes = now;
        }

        // Backstop probe — also the very first attempt, so that when an
        // operator is already up we go straight to relaying without emitting
        // a stray TLE-only line. The dir-watch below is the fast path.
        if (now - last_probe >= STREAM_PROBE_PERIOD_S) {
            last_probe = now;
            if (stream_try_relay()) {
                // Operator dropped: resume TLE-only promptly.
                last_emit  = 0;
                last_probe = time(NULL);
                continue;
            }
        }

        if (now - last_emit >= STREAM_TLE_PERIOD_S) {
            struct tm utc;
            struct timeval tv;
            UTC_Calendar_Now(&utc, &tv);
            double jul_utc = Julian_Date(&utc, &tv);
            update_satellite_position(&state->prediction, jul_utc);
            compute_predictions(state, jul_utc);
            if (state->doppler_correction_enabled) {
                update_doppler_shifted_frequencies(state,
                    state->nominal_uplink_frequency_hz,
                    state->nominal_downlink_frequency_hz);
            }
            stream_emit_tle_state(state, jul_utc);
            last_emit = now;
        }

        // Idle a beat, but wake early if the runtime dir changes — a
        // --control operator may have just bound its socket. A stop signal
        // interrupts either wait for prompt exit. The watch can report
        // unrelated changes (other tools' sockets, the operator's own
        // teardown); a failed connect costs nothing, so we just re-check.
        if (watch != NULL) {
            int ev = sso_dirwatch_wait(watch, STREAM_IDLE_MS);
            if (ev > 0) {
                if (stream_try_relay()) {
                    last_emit  = 0;
                    last_probe = time(NULL);
                    continue;
                }
            } else if (ev < 0) {
                // The watch broke (not a signal). Drop it and fall back to
                // the periodic probe so we don't busy-spin on the error.
                sso_dirwatch_close(watch);
                watch = NULL;
                usleep(STREAM_IDLE_MS * 1000);
            }
        } else {
            usleep(STREAM_IDLE_MS * 1000);
        }
    }

    sso_dirwatch_close(watch);
    return 0;
}
