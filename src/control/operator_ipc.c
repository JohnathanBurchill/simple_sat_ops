/*

   Simple Satellite Operations  control/operator_ipc.c

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

#include "operator_ipc.h"
#include "state.h"

#include "operator_audio.h"  // operator_audio_handle_ctl
#include "auto_tcmd.h"       // auto_tcmd_progress
#include "ipc_fill.h"        // ipc_fill_state_prediction
#include "panels.h"          // rx_panel_collect_local, rx_panel_data_t
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_operator.h"
#include "tui.h"             // tui_install_yield_handler
#include "viewer.h"          // read_operator_pid

#include <stdio.h>
#include <stdlib.h>          // EXIT_FAILURE
#include <string.h>

// Cap on the number of connected clients listed in the roster broadcast.
#define SSO_IPC_MAX_CLIENTS_FOR_ROSTER 16

// Snapshot the operator's RX panel into the wire-side fields of an
// event. Called for both STATE broadcasts and WELCOME replies so a
// newly-connecting viewer sees the same panel state everyone else does.
static void ipc_fill_rx_panel(state_t *state, sso_event_t *evt)
{
    rx_panel_data_t d;
    rx_panel_collect_local(state, &d);
    evt->rx_have_session = d.have_session;
    // Warning row is operator-wide (e.g. low disk), not gated on the
    // SDR — fill it before the have_session early-return.
    snprintf(evt->rx_warning, sizeof evt->rx_warning, "%s", d.warning);
    if (!d.have_session) return;
    evt->rx_rec_active   = d.rec_active;
    evt->rx_freq_hz      = d.rx_freq_hz;
    evt->rx_peak_dbfs    = d.peak_dbfs;
    evt->rx_rms_dbfs     = d.rms_dbfs;
    evt->rx_frames_total = (long) d.frames_total;
    evt->rx_frames_pcm   = (long) d.frames_pcm;
    evt->rx_frames_vit   = (long) d.frames_vit;
    snprintf(evt->rx_last_frame_summary,
             sizeof evt->rx_last_frame_summary, "%s", d.last_frame_summary);
    evt->rx_age_s = d.age_s;
    int slots = RX_PANEL_PT_COUNT < SSO_RX_PT_SLOTS
              ? RX_PANEL_PT_COUNT : SSO_RX_PT_SLOTS;
    for (int s = 0; s < slots; ++s) {
        evt->rx_pt_count[s]       = (long) d.pt_count[s];
        int pl = d.pt_payload_len[s];
        if (pl < 0) pl = 0;
        int wire_pl = pl;
        if (wire_pl > SSO_RX_PT_PAYLOAD_MAX) wire_pl = SSO_RX_PT_PAYLOAD_MAX;
        evt->rx_pt_payload_len[s] = pl;
        memcpy(evt->rx_pt_payload[s], d.pt_payload[s], (size_t) wire_pl);
        snprintf(evt->rx_pt_summary[s], sizeof evt->rx_pt_summary[s],
                 "%.*s", (int)(sizeof evt->rx_pt_summary[s] - 1),
                 d.pt_summary[s]);
    }
    int rn = d.ribbon_n;
    if (rn > SSO_RIBBON_MAX) rn = SSO_RIBBON_MAX;
    evt->rx_ribbon_n = rn;
    memcpy(evt->rx_ribbon, d.ribbon, (size_t) rn);
    evt->rx_ribbon[rn] = '\0';
    memcpy(evt->rx_ribbon_peak, d.ribbon_peak,
           (size_t) rn * sizeof evt->rx_ribbon_peak[0]);
}

void ipc_broadcast_state(state_t *s,
                                  double az, double el,
                                  double downlink_freq,
                                  double doppler_delta_dl,
                                  double jul_utc) {
    if (!s->ipc) return;
    sso_event_t evt;
    sso_event_init(&evt, SSO_EVT_STATE);
    snprintf(evt.from, sizeof(evt.from), "%s",
             s->operator_user ? s->operator_user : "?");
    snprintf(evt.operator_user, sizeof(evt.operator_user), "%s",
             s->operator_user ? s->operator_user : "?");
    evt.has_state = 1;
    evt.az = az;
    evt.el = el;
    evt.freq_hz = (long) downlink_freq;
    evt.doppler_hz = doppler_delta_dl;
    if (s->pass_folder[0]) {
        snprintf(evt.pass_folder, sizeof(evt.pass_folder), "%s",
                 s->pass_folder);
    }
    if (s->track.prediction.tles_filename) {
        snprintf(evt.tle_path, sizeof(evt.tle_path), "%s",
                 s->track.prediction.tles_filename);
    }
    evt.target_az = s->rot.antenna_rotator.target_azimuth;
    evt.target_el = s->rot.antenna_rotator.target_elevation;
    evt.flip      = s->rot.antenna_rotator.flip_mode_pass;
    evt.in_pass   = s->track.in_pass;
    evt.tracking  = s->rot.antenna_rotator.tracking;
    evt.has_rotator = s->rot.have_antenna_rotator;
    evt.jul_utc   = jul_utc;

    // Prediction snapshot (satellite name, idesg, pass timing, sky
    // position, range) — viewer renders these verbatim. Shared with the
    // headless --viewer-stream relay so both fill the fields identically.
    ipc_fill_state_prediction(&s->track.prediction, &evt);

    // Auto-TCMD progress so viewers can follow the run without the modal.
    {
        int at_sent = 0, at_total = 0;
        const char *at_label = NULL;
        if (auto_tcmd_progress(s, &at_sent, &at_total, &at_label)) {
            evt.auto_tcmd_on    = 1;
            evt.auto_tcmd_sent  = at_sent;
            evt.auto_tcmd_total = at_total;
            snprintf(evt.auto_tcmd_state, sizeof evt.auto_tcmd_state,
                     "%s", at_label);
        }
    }
    sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
    size_t n = 0;
    if (n < sizeof(entries) / sizeof(entries[0])) {
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 s->operator_user ? s->operator_user : "?");
        snprintf(entries[n].role, sizeof(entries[n].role), "operator");
        entries[n].since[0] = '\0';
        n++;
    }
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char user[64], role[16], since[40];
    while (n < sizeof(entries) / sizeof(entries[0])
           && sso_ipc_server_next_client(s->ipc, &it, &cid,
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
    ipc_fill_rx_panel(s, &evt);
    char buf[SSO_IPC_LINE_MAX];
    if (sso_event_encode(&evt, buf, sizeof(buf)) == 0) {
        sso_ipc_server_broadcast(s->ipc, buf);
    } else {
        fprintf(stderr, "operator_ipc: STATE encode overflow -- "
                "dropped (roster too large?)\n");
    }

    // Cache for WELCOME replies so a viewer doesn't have to wait for
    // the next periodic broadcast to see anything.
    snprintf(s->last_state.sat, sizeof s->last_state.sat, "%s", evt.satellite);
    snprintf(s->last_state.tle, sizeof s->last_state.tle, "%s", evt.tle_path);
    s->last_state.az      = evt.az;
    s->last_state.el      = evt.el;
    s->last_state.freq_hz = evt.freq_hz;
    s->last_state.doppler = evt.doppler_hz;
    s->last_state.tgt_az  = evt.target_az;
    s->last_state.tgt_el  = evt.target_el;
    s->last_state.flip    = evt.flip;
    s->last_state.in_pass = evt.in_pass;
    s->last_state.tracking= evt.tracking;
    s->last_state.has_rot = evt.has_rotator;
    s->last_state.jul     = evt.jul_utc;
    snprintf(s->last_state.idesg, sizeof s->last_state.idesg, "%s", evt.idesg);
    s->last_state.epoch_min    = evt.epoch_min;
    s->last_state.min_visible  = evt.min_visible;
    s->last_state.min_above_0  = evt.min_above_0;
    s->last_state.min_above_30 = evt.min_above_30;
    s->last_state.max_el       = evt.max_el;
    s->last_state.pred_az      = evt.pred_az;
    s->last_state.pred_el      = evt.pred_el;
    s->last_state.alt_km       = evt.alt_km;
    s->last_state.lat_deg      = evt.lat_deg;
    s->last_state.lon_deg      = evt.lon_deg;
    s->last_state.speed_kms    = evt.speed_kms;
    s->last_state.range_km     = evt.range_km;
    s->last_state.rrate_kms    = evt.range_rate_kms;
    s->last_state.valid   = 1;
}

void ipc_on_event(sso_ipc_server_t *srv, sso_client_id_t id,
                         const sso_event_t *evt, void *user) {
    state_t *state = (state_t *) user;
    // Live-audio control from a viewer/relay: (un)subscribe and reply with
    // an audio-status. Handled here so the relay's audio-ctl reaches the
    // one process that owns the SDR.
    if (evt->type == SSO_EVT_AUDIO_CTL) {
        operator_audio_handle_ctl(state, id, evt);
        return;
    }
    if (evt->type != SSO_EVT_HELLO) return;
    sso_event_t welcome;
    sso_event_init(&welcome, SSO_EVT_WELCOME);
    snprintf(welcome.from, sizeof(welcome.from), "%s",
             state->operator_user ? state->operator_user : "?");
    snprintf(welcome.operator_user, sizeof(welcome.operator_user), "%s",
             state->operator_user ? state->operator_user : "?");
    if (state->pass_folder[0]) {
        snprintf(welcome.pass_folder, sizeof(welcome.pass_folder), "%s",
                 state->pass_folder);
    }
    if (state->last_state.valid) {
        welcome.has_state   = 1;
        snprintf(welcome.satellite, sizeof welcome.satellite,
                 "%s", state->last_state.sat);
        snprintf(welcome.tle_path, sizeof welcome.tle_path,
                 "%s", state->last_state.tle);
        welcome.az          = state->last_state.az;
        welcome.el          = state->last_state.el;
        welcome.freq_hz     = state->last_state.freq_hz;
        welcome.doppler_hz  = state->last_state.doppler;
        welcome.target_az   = state->last_state.tgt_az;
        welcome.target_el   = state->last_state.tgt_el;
        welcome.flip        = state->last_state.flip;
        welcome.in_pass     = state->last_state.in_pass;
        welcome.tracking    = state->last_state.tracking;
        welcome.has_rotator = state->last_state.has_rot;
        welcome.jul_utc     = state->last_state.jul;
        snprintf(welcome.idesg, sizeof welcome.idesg, "%s", state->last_state.idesg);
        welcome.epoch_min      = state->last_state.epoch_min;
        welcome.min_visible    = state->last_state.min_visible;
        welcome.min_above_0    = state->last_state.min_above_0;
        welcome.min_above_30   = state->last_state.min_above_30;
        welcome.max_el         = state->last_state.max_el;
        welcome.pred_az        = state->last_state.pred_az;
        welcome.pred_el        = state->last_state.pred_el;
        welcome.alt_km         = state->last_state.alt_km;
        welcome.lat_deg        = state->last_state.lat_deg;
        welcome.lon_deg        = state->last_state.lon_deg;
        welcome.speed_kms      = state->last_state.speed_kms;
        welcome.range_km       = state->last_state.range_km;
        welcome.range_rate_kms = state->last_state.rrate_kms;
        // Auto-TCMD progress reads the live modal state (like
        // ipc_fill_rx_panel below) — no state->last_state.* cache needed.
        {
            int at_sent = 0, at_total = 0;
            const char *at_label = NULL;
            if (auto_tcmd_progress(state, &at_sent, &at_total, &at_label)) {
                welcome.auto_tcmd_on    = 1;
                welcome.auto_tcmd_sent  = at_sent;
                welcome.auto_tcmd_total = at_total;
                snprintf(welcome.auto_tcmd_state,
                         sizeof welcome.auto_tcmd_state, "%s", at_label);
            }
        }
        // Roster — operator first, then the existing clients we know
        // of. The newly-connecting client is already in the slot table
        // (slot_dispatch_line ran first) but its role isn't populated
        // until HELLO is processed; that's why we iterate via
        // sso_ipc_server_next_client, which surfaces what HELLO set.
        sso_roster_entry_t entries[SSO_IPC_MAX_CLIENTS_FOR_ROSTER];
        size_t n = 0;
        snprintf(entries[n].user, sizeof(entries[n].user), "%s",
                 state->operator_user ? state->operator_user : "?");
        snprintf(entries[n].role, sizeof(entries[n].role), "operator");
        entries[n].since[0] = '\0';
        n++;
        sso_ipc_iter_t it = {0};
        sso_client_id_t cid;
        char ruser[64], rrole[16], rsince[40];
        while (n < sizeof(entries) / sizeof(entries[0])
               && sso_ipc_server_next_client(srv, &it, &cid,
                                              ruser, sizeof(ruser),
                                              rrole, sizeof(rrole),
                                              rsince, sizeof(rsince)) == 0) {
            if (!ruser[0]) continue;
            snprintf(entries[n].user,  sizeof(entries[n].user),  "%s", ruser);
            snprintf(entries[n].role,  sizeof(entries[n].role),  "%s",
                     rrole[0] ? rrole : "viewer");
            snprintf(entries[n].since, sizeof(entries[n].since), "%s", rsince);
            n++;
        }
        sso_event_set_roster(&welcome, entries, n);
        ipc_fill_rx_panel(state, &welcome);
    }
    char buf[SSO_IPC_LINE_MAX];
    if (sso_event_encode(&welcome, buf, sizeof(buf)) == 0) {
        sso_ipc_server_send(srv, id, buf);
    } else {
        fprintf(stderr, "operator_ipc: WELCOME encode overflow -- "
                "viewer gets no initial state\n");
    }
}

int ipc_operator_startup(state_t *state, int argc, char **argv)
{
    // Audit + operator IPC bring-up.
    state->operator_user = sso_unix_user();
    sso_audit_start("simple_sat_ops",
                    state->control_mode ? "operator" : "standalone");
    // Record the exact command line so post-incident review can tie
    // every operator action back to the flags the session was started
    // with (recording mode, --tx settings, TLE, etc.). One line, tab-
    // safe (sso_audit's sanitiser replaces tabs/newlines with spaces).
    {
        char argv_buf[1024];
        size_t off = 0;
        argv_buf[0] = '\0';
        for (int i = 0; i < argc && off + 2 < sizeof argv_buf; ++i) {
            int n = snprintf(argv_buf + off, sizeof argv_buf - off,
                             "%s%s", (i == 0) ? "" : " ", argv[i]);
            if (n <= 0) break;
            off += (size_t) n;
            if (off >= sizeof argv_buf) { off = sizeof argv_buf - 1; break; }
        }
        sso_audit_event("argv", argv_buf);
    }
    if (state->control_mode) {
        // Refuse if another simple_sat_ops --control is already
        // bound — two operators driving the same SDR / rotator is
        // exactly the failure mode the IPC server existed to avoid.
        // The probe connects as a transient viewer, reads the
        // operator's identity off the welcome reply, and disconnects.
        char existing_user[64]    = {0};
        char existing_folder[256] = {0};
        int op_status = sso_operator_verify("viewer",
                                             existing_folder,
                                             sizeof existing_folder,
                                             existing_user,
                                             sizeof existing_user);
        if (op_status == SSO_OP_OK || op_status == SSO_OP_MISMATCH) {
            pid_t op_pid = 0;
            const char *who = existing_user[0] ? existing_user : "?";
            if (read_operator_pid(&op_pid) == 0) {
                fprintf(stderr,
                    "simple_sat_ops: --control refused: operator already "
                    "running as user=%s pid=%d.\n"
                    "  To take over, run a viewer (no --control) and press\n"
                    "  'c' then 'y' to force-claim; the running operator\n"
                    "  will yield and your viewer will re-exec into --control.\n",
                    who, (int) op_pid);
            } else {
                fprintf(stderr,
                    "simple_sat_ops: --control refused: operator already "
                    "running as user=%s.\n", who);
            }
            char det[96];
            snprintf(det, sizeof det,
                     "existing_user=%s existing_pid=%d",
                     who, (int) op_pid);
            sso_audit_event("control-refused", det);
            return EXIT_FAILURE;
        }

        state->ipc = sso_ipc_server_open("simple_sat_ops");
        if (state->ipc == NULL) {
            // Probe said "no operator" yet bind still failed — most
            // likely a stale socket / pid file from a crashed
            // previous operator (or a vanishingly-rare race with
            // another --control starting at the same instant).
            // Either way, refuse so we don't quietly drive hardware
            // alongside something else.
            fprintf(stderr,
                "simple_sat_ops: --control: socket bind failed. If this is "
                "from a crashed previous operator, remove "
                "/run/sso/simple_sat_ops.{sock,pid} and retry.\n");
            sso_audit_event("ipc-bind-failed", "");
            return EXIT_FAILURE;
        }
        sso_ipc_server_on_event(state->ipc, ipc_on_event, state);
        tui_install_yield_handler();
        fprintf(stderr, "simple_sat_ops: operator=%s ipc=on\n",
                state->operator_user);
    }
    return 0;
}
