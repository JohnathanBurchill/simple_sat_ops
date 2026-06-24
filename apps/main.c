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
#include "antenna_rotator_async.h"
#include "tr_switch.h"
#include "state.h"
#include "prediction.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "operator_audio.h"
#include "frontiersat.h"
#include "sso_time.h"
#include "sso_version.h"
#include "panels.h"
#include "pass_session.h"
#include "operator_ipc.h"
#include "tracking.h"
#include "hw_bringup.h"
#include "spectrogram.h"
#include "tui.h"
#include "auto_tcmd.h"
#include "cmd_line.h"
#include "input.h"
#include "live_waterfall.h"
#include "tx_compose.h"
#include "viewer.h"
#include "tx_log.h"
#include "cli_args.h"
#include "argparse.h"

#ifdef SSO_WITH_SDR
#include "rx_session.h"
#include "tx_burst.h"
#endif

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <time.h>
#include <ncurses.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// Carrier defaults. FrontierSat is simplex on 436.150 MHz — uplink
// and downlink share the same frequency — so both default to the
// same value. Overridable with --uplink-freq-mhz= / --downlink-freq-mhz=
// for any future split-band bird.

// Doppler retune threshold (Hz). Frequencies update on the display
// when the residual drifts past this. 200 Hz keeps the residual offset
// well inside the 9600 GFSK clean-eye band (~±3 kHz) even at the peak
// ~100 Hz/s slew near TCA.
#define DOPPLER_SHIFT_RESOLUTION_KHZ 0.2

// Antenna rotator max angle from target before a new SET is issued.

// WARN_DAYS_SINCE_EPOCH lives in ui/panels.h's renderer; MAX_MINUTES_TO_PREDICT
// in control/pass_session.h (shared with the week-ahead pass search).

#define UPDATE_INTERVAL_MICROSEC 500000

// --- Operator-mode IPC bookkeeping ---------------------------------
// Set by apply_args when --control is passed. When set, main() opens
// the sso_ipc server on /run/sso/simple_sat_ops.sock and fans out a
// state event on every UI tick. Other operator-aware tools
// (b210_rx_tx --control, tx_frame_sdr) verify the operator's Unix
// user matches their own via this socket.
// The IPC fan-out server and the operator's Unix user now live on
// state_t (state.ipc / state.operator_user).

// --self-test: after CLI parse + HMAC keyfile load, print the resolved
// configuration to stdout and exit 0 — BEFORE opening the IPC socket,
// the rotator, the B210, or loading the TLE. Useful for confirming
// "did my command line do what I think?" without keying any hardware
// or claiming any shared resource. Skips the no-arg viewer-probe in
// apply_args too (which is itself a side effect).

// Refuse to fully start when the --tc-file agenda has telecommand lint
// errors. --ignore-at-your-peril-all-tc-errors clears this and lets a
// known-bad agenda through anyway.

// TX dry-run: record the command as not-sent (reason "dry-run")
// instead of pushing the burst through rx_session. Lets the operator
// exercise the auto-tcmd
// state machine + the TX compose modal on a dev host with no B210 (or
// with --without-b210 to skip the device). The allow-tx safety
// checkbox still has to be ticked to enter RUNNING — dry-run is about
// hardware presence, not about the operator's intent to transmit.

// Live raylib waterfall viewer. Off by default; --live-waterfall on the
// command line opts in. The spawn / reap / shutdown of the child viewer and
// the pipe to its stdin live in ui/live_waterfall.c.

// --always-record: start the WAV / IQ / sidecar recording as soon
// as rx_session opens and keep it open until shutdown, ignoring the
// usual per-pass elevation gate. Intended for bench characterisation
// runs (noise floor vs. antenna orientation, no-antenna baseline,
// gain stability over hours) where the operator wants continuous
// capture without a satellite pass to drive AOS / LOS.

// --testing: bench / characterisation runs that aren't tied to a
// pass. Pass folder lands under <root>/Testing/yyyymmdd/hhmmLT/ using
// the CURRENT local time, not a predicted AOS — keeps test captures
// out of the operational Operations/ tree and skips the "no AOS in
// next N minutes" abort in setup_pass_folder.

// --- main ---------------------------------------------------------

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "simple_sat_ops")) return 0;
    // state_t is zero-initialised; every non-zero default is set in one place,
    // apply_args (run below), so there is no half-initialised window here.
    state_t state = {0};

    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    double jul_utc = Julian_Date(&utc, &tv);
    // --help / --help-full historically exited 2; preserve that. Every
    // parse-or-runtime failure inside apply_args collapses to exit 1
    // (the old code returned 1 for too-many-positionals / startup
    // errors and 3 for an unknown --option; both are now PARSE_ERROR ->
    // 1, each still printing its own distinct stderr message).
    int status = apply_args(&state, argc, argv, jul_utc, HELP_OFF);
    if (status != 0) {
        return status;
    }

    // Bare invocation found a running operator — run as a read-only
    // viewer and skip the rest of the operator/standalone bring-up.
    if (state.viewer_mode) {
        return run_viewer(argv[0]);
    }

    // --viewer-stream: headless JSON stream for the remote viewer. Load the
    // orbit (TLE + ephemeris) but open NO hardware, bind NO IPC server, and
    // load NO HMAC key — it only ever reads the sky and writes JSON to
    // stdout, propagating the TLE itself when no operator is up and relaying
    // an operator's broadcast when one is. Returns BEFORE the HMAC keyfile
    // load / lint gate / operator bring-up below so none of those run.
    // --viewer-stream --self-test prints the dry-run config and exits 0.
    if (state.viewer_stream) {
        int s = pass_session_load_orbit(&state);
        if (s == 0) {
            if (state.self_test) {
                self_test_report(&state, stdout, argc, argv);
            } else {
                s = run_viewer_stream(&state);
            }
        }
        if (state.prediction.auto_sat) {
            free_passes();
        }
        return s;
    }

#ifdef SSO_WITH_SDR
    // Pin the SSO+ @tssent dedup key for this session: the startup UTC,
    // truncated to the minute. Constant for the life of the process so the
    // satellite runs an SSO+ time-sync once per pass. See sso_pseudo.h.
    state.sso_pass_tssent_ms = (sso_now_utc_ms() / 60000LL) * 60000LL;
#endif

    // Resolve + load the HMAC keyfile (feeds every TX burst's AX100 frame and
    // lights the operator banner). See apps/cli_args.c.
    cli_load_hmac_keyfile(&state);

    // Telecommand-agenda lint gate: refuse to start on a --tc-file with lint
    // errors, unless --ignore-at-your-peril-all-tc-errors. See cli_args.c.
    if ((status = cli_tcmd_lint_gate(&state)) != 0) {
        return status;
    }

    // Audit-log the session + command line and, in --control mode, bind the
    // operator IPC socket. See control/operator_ipc.c.
    if ((status = ipc_operator_startup(&state, argc, argv)) != 0) {
        return status;
    }

    // Load the TLE, select the ephemeris, and (in --control mode) set up the
    // pass folder for the next pass. See control/pass_session.c.
    int tle_status = pass_session_load_orbit(&state);
    if (tle_status) {
        return tle_status;
    }

    // Hardware bring-up (rotator, T/R switch, SDR), each before ncurses
    // takes the screen so device warnings land on the terminal. See
    // control/hw_bringup.c.
    if ((status = hw_rotator_open(&state)) != 0) {
        return status;
    }
    // --calibrate-rotator runs the one-shot rate measurement and exits.
    if (state.rot.calibrate_rotator) {
        return hw_rotator_calibrate(&state.rot);
    }
    hw_pursuit_rates_load(&state.rot);
    hw_tr_switch_open(&state.trsw);
    hw_sdr_open(&state);

    // --self-test: the full bring-up has now run — TLE load, pass-folder
    // setup, rotator open, SDR open, IPC bind, and the --tc-file lint — so
    // the report below reflects the live, resolved state of each piece of
    // hardware, not just the requested intent. Print it to stdout (we are
    // still BEFORE init_window, so ncurses has not taken the screen), tear
    // the bring-up back down, and exit. This is a true end-to-end dry run:
    // it confirms the session would come up without flying a pass.
    if (state.self_test) {
        self_test_report(&state, stdout, argc, argv);
#ifdef SSO_WITH_SDR
        if (state.sdr.rx_session) {
            rx_session_close(state.sdr.rx_session);
            state.sdr.rx_session = NULL;
        }
#endif
        if (state.rot.rot_async != NULL) {
            antenna_rotator_async_close(state.rot.rot_async);
            state.rot.rot_async = NULL;
        }
        if (state.rot.have_antenna_rotator) {
            antenna_rotator_disconnect(&state.rot.antenna_rotator);
            state.rot.have_antenna_rotator = 0;
        }
        if (state.trsw.have_tr_switch) {
            tr_switch_disconnect(&state.trsw.tr_switch);
            state.trsw.have_tr_switch = 0;
        }
        if (state.ipc) {
            sso_ipc_server_close(state.ipc);
            state.ipc = NULL;
        }
        if (state.prediction.auto_sat) {
            free_passes();
        }
        return 0;
    }

    // Capture the cooked terminal modes BEFORE ncurses switches the tty
    // to raw, so the crash handler can put it back deterministically.
    tui_save_termios();

    init_window(&state);

    // Catch a fatal device fault (or Ctrl-C) now that the screen is up,
    // so it restores the terminal instead of dumping a raw abort on it.
    install_signal_handlers();
    // Let the crash handler reach the live-waterfall child (owned here) so
    // it doesn't orphan when a device-loss abort skips normal teardown.
    tui_register_waterfall_pid(live_waterfall_pid_ref());

    state.running = 1;

    double current_uplink_frequency = state.nominal_uplink_frequency_hz;
    double current_downlink_frequency = state.nominal_downlink_frequency_hz;
    double doppler_delta_uplink = 0.0;
    double doppler_delta_downlink = 0.0;
    double doppler_max_delta = DOPPLER_SHIFT_RESOLUTION_KHZ * 1000.0;
    (void) doppler_delta_uplink;  // tracked for display symmetry / future IPC
    (void) doppler_max_delta;     // threshold for any future on-display retune

    state.rot.antenna_rotator.antenna_should_be_controlled =
        state.rot.run_with_antenna_rotator && state.rot.have_antenna_rotator;
    state.rot.antenna_rotator.antenna_is_under_control =
        state.rot.antenna_rotator.antenna_should_be_controlled;

    int keyboard_unlocked = 1;
    int keyboard_info_row = 20;

    mvprintw(keyboard_info_row++, 3, "%s", "T  - Track satellite");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "s  - Stop antenna immediately");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "r  - Reset to az=0 el=0");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "[/]- Jog azimuth -5/+5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "{/}- Jog azimuth -1/+1 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", ",/.- Jog elevation -5/+5 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "</>- Jog elevation -1/+1 deg");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "t  - Compose TX command");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "A  - Auto-TCMD (needs --tc-file=)");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "K  - Lock/unlock keyboard");
    clrtoeol();
    mvprintw(keyboard_info_row++, 3, "%s", "q  - Quit");
    clrtoeol();

    double current_az = 0;
    double current_el = 0;

    // Slow-cadence work is timestamp-gated so the fast UHD-pump loop
    // doesn't spam viewers or burn CPU on ncurses redraws.
    //
    // Two redraw gates: the "slow" one drives the predictions / status
    // / RX panel / TX log rows and is the one that costs CPU — most
    // notably report_status, which does a blocking read() against the
    // rotator serial port (antenna_rotator.c:142). Keep it at 2 Hz so
    // the rotator isn't hammered. The "fast" path is cmd_render only
    // — runs every loop tick while the operator is typing in the ":"
    // prompt so each keystroke echoes immediately.
    double t_last_ipc_broadcast = 0.0;
    double t_last_redraw        = 0.0;
    // These periods are UPPER BOUNDS, not fixed schedules: each gate is only
    // tested once per loop nap (usleep at the loop tail), so the real cadence
    // is min(this period, nap-bounded tick rate). At the default 2 Hz nap they
    // coincide; while the operator types or audio streams, the nap shortens
    // and the gates fire closer to these periods.
    const double IPC_BROADCAST_PERIOD_S = 0.5;   // 2 Hz
    const double REDRAW_PERIOD_S        = 0.5;   // 2 Hz

    // Per-pass WAV recording: arm 1 min before AOS, hold open through
    // the pass, close 1 min after LOS. Multiple passes during one
    // simple_sat_ops run each get their own file under the pass folder.
    // All three are consumed only inside #ifdef WITH_USRP_B210; the
    // attribute keeps gcc-15 quiet on the non-B210 dev build.
    __attribute__((unused)) const double RECORDING_PREROLL_S  = 60.0;
    __attribute__((unused)) const double RECORDING_POSTROLL_S = 60.0;
    __attribute__((unused)) double t_recording_close_at = 0.0;

    while (state.running) {
        // Ctrl-C / SIGTERM: leave the loop and run the normal teardown
        // (endwin, rotator home, device close) instead of dying raw.
        if (tui_should_quit()) { state.running = 0; break; }
        double t_now = monotonic_seconds();
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_utc);

        // Drain whatever the T/R switch emitted since the last tick.
        // Non-blocking; the firmware beats every ~2.5 s so most ticks
        // read zero bytes.
        if (state.trsw.have_tr_switch) {
            tr_switch_pump(&state.trsw.tr_switch, t_now);
        }

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

#ifdef SSO_WITH_SDR
        // TX-side Doppler: transmit at the frequency that places the
        // nominal carrier at the satellite. Sign: range_rate_km_s > 0
        // when the satellite is receding (LOS end of a pass), so the
        // ground must transmit higher to compensate for redshift at
        // the moving receiver. Mirror of the RX-side correction,
        // applied to FRONTIERSAT_CARRIER_HZ (the actual TX carrier) —
        // state.doppler_uplink_frequency_hz is computed from the 2 m
        // amateur nominal and would give the wrong absolute frequency
        // here. Off when doppler_correction_enabled is false (e.g.
        // bench loopback) so RX and TX share one constant carrier.
        state.tx_freq_hz_doppler = tx_burst_doppler_freq_hz(
            FRONTIERSAT_CARRIER_HZ,
            state.prediction.satellite_ephem.range_rate_km_s,
            state.doppler_correction_enabled);
#endif

#ifdef SSO_WITH_SDR
        // Auto-record per pass: open the WAV 1 min before AOS (or as
        // soon as we're above the horizon, in case simple_sat_ops
        // started mid-pass), keep it open while the satellite is up,
        // close 1 min after LOS. Each pass gets its own auto-named
        // file in the pass folder. Note: this deliberately keys off the
        // satellite geometry (elevation + time-until-AOS) rather than
        // state.in_pass — the latter flips several minutes before AOS
        // (tracking_prep_time_minutes) so the rotator can pre-position,
        // which is far too early to start the WAV.
        //
        // --always-record disables this gate entirely: recording was
        // started once at rx_session_open and stays open until shutdown.
        if (state.sdr.rx_session && !state.sdr.always_record) {
            double sec_to_aos =
                state.prediction.predicted_minutes_until_visible * 60.0;
            int visible   = (state.prediction.satellite_ephem.elevation > 0.0);
            int in_preroll = (sec_to_aos > 0.0
                              && sec_to_aos <= RECORDING_PREROLL_S);
            int active = rx_session_wav_active(state.sdr.rx_session);
            if (!active && (visible || in_preroll)) {
                rx_session_request_wav_start(state.sdr.rx_session);
                t_recording_close_at = 0.0;
                char det[64];
                snprintf(det, sizeof det,
                    "trigger=%s sec_to_aos=%.1f el=%.1f",
                    visible ? "elevation" : "preroll",
                    sec_to_aos,
                    state.prediction.satellite_ephem.elevation);
                sso_audit_event("rec-start", det);
            } else if (active) {
                if (visible) {
                    t_recording_close_at = 0.0;  // cancel any pending close
                } else if (t_recording_close_at == 0.0) {
                    t_recording_close_at = t_now + RECORDING_POSTROLL_S;
                } else if (t_now >= t_recording_close_at) {
                    rx_session_request_wav_stop(state.sdr.rx_session);
                    t_recording_close_at = 0.0;
                    sso_audit_event("rec-stop", "trigger=postroll-expired");
                }
            }
        }
#endif

        // Snapshot the rotator position for the IPC broadcast below, then run
        // the per-tick antenna pointing (settle detect, two-step home, scan,
        // satellite-tracking / pursuit aim). See control/tracking.c.
        current_az = state.rot.antenna_rotator.azimuth;
        current_el = state.rot.antenna_rotator.elevation;
        tracking_tick(&state, jul_utc, t_now);

        int redraw_due = (t_now - t_last_redraw) >= REDRAW_PERIOD_S;
        if (redraw_due) {
            // Paint the whole operator layout for this tick. See ui/panels.c.
            render_operator_screen(&state, jul_utc, t_now, keyboard_info_row);
        }

        // Read one key and route it: modals / command line / keyboard lock /
        // unlocked operator keys (track, jog, modal openers). See ui/input.c.
        input_handle_keys(&state, &keyboard_unlocked);

        if (redraw_due) {
            // Width-padded prints (not clrtoeol) so we don't wipe the
            // signal ribbon that paints over the right edge of these rows.
            mvprintw(keyboard_info_row, 3, "%s : %-8s", "Keyboard",
                     keyboard_unlocked ? "unlocked" : "LOCKED");
            mvprintw(keyboard_info_row + 2, 0, "%-18s",
                     state.rot.antenna_rotator.antenna_is_moving
                         ? "Antenna moving"
                         : "Antenna stationary");
            t_last_redraw = t_now;
        }

        // Pump the modal's debounced preview broadcast before the
        // screen flush so the mirror line is current when we paint.
        tx_compose_pump(&state);
        // Drive the auto-tcmd burst loop. Queues state.tx_request when
        // it's time for the next send; the existing main-loop burst
        // handler below transmits and emits the SENT/NOT_SENT events.
        auto_tcmd_tick(&state);

        // Bottom-row prompt + screen flush. When the operator is typing
        // in the ":" prompt we want this every tick (~50 Hz) so each
        // keystroke echoes immediately. Otherwise piggyback on the slow
        // redraw so the row picks up any post-command status string.
        // When a modal is open we force-redraw it on top of stdscr by
        // touchwin'ing every modal cell as dirty and wrefresh'ing the
        // window after stdscr's own refresh. doupdate's incremental
        // diff is otherwise free to skip "unchanged" modal cells, which
        // is what was letting panel updates (e.g. the antenna status
        // row) bleed through and overwrite the modal.
        if (redraw_due || state.cmd.active || state.tx_compose_active
            || state.auto_tcmd_active) {
            cmd_render(&state);
            refresh();
            int show_hw_cursor = 0;
            if (state.tx_compose_active && state.tx_compose_win) {
                touchwin(state.tx_compose_win);
                wrefresh(state.tx_compose_win);
                tx_field_t f = state.tx_compose.focus;
                show_hw_cursor = (f == TXF_PAYLOAD || f == TXF_POWER);
            } else if (state.auto_tcmd_active && state.auto_tcmd_win) {
                touchwin(state.auto_tcmd_win);
                wrefresh(state.auto_tcmd_win);
                show_hw_cursor = (state.auto_tcmd.state != AUTO_STATE_RUNNING)
                              && auto_field_is_text(state.auto_tcmd.focus);
            } else if (state.cmd.active) {
                show_hw_cursor = 1;
            }
            curs_set(show_hw_cursor ? 1 : 0);
        }

#ifdef SSO_WITH_SDR
        // Signal ribbon sampler: push one peak-dBFS reading per second
        // so the ribbon in the RX panel rolls left in real time. Also
        // grab the iq_burst bright-bin count so the renderer can pick
        // a character that distinguishes broadband packets from a CW
        // carrier at the same peak level.
        if (state.sdr.rx_session && (t_now - state.ribbon_last_t) >= 1.0) {
            double peak = -90.0;
            rx_session_snapshot(state.sdr.rx_session, NULL, &peak, NULL,
                                NULL, NULL, 0);
            int burst_bins = 0;
            rx_session_burst_snapshot(state.sdr.rx_session, &burst_bins, NULL);
            ribbon_push(&state, peak, burst_bins);
            state.ribbon_last_t = t_now;

            // Live waterfall: (re)launch / reap the out-of-process viewer on
            // the same once-per-second cadence as the ribbon. See
            // ui/live_waterfall.c.
            if (state.run_live_waterfall) {
                live_waterfall_poll(state.sdr.rx_session);
            }
        }
        // Software Doppler tracking: the SDR LO stays fixed at the
        // nominal carrier (set once at session open) and we apply the
        // Doppler correction inside the IQ pump as a complex multiply.
        // No PLL glitches, sub-Hz resolution, and the displayed RX freq
        // updates smoothly. The threshold-driven hardware retune that
        // lived here previously fired every 1–10 seconds during a
        // pass and caused brief phase resets in the coherent demod
        // chain; this loop runs every tick at full precision.
        if (state.sdr.rx_session && state.doppler_correction_enabled) {
            double offset = state.doppler_downlink_frequency_hz
                          - state.nominal_downlink_frequency_hz;
            rx_session_set_doppler_offset(state.sdr.rx_session, offset);
        }
        if (state.sdr.rx_session) {
            double doppler_offset =
                state.doppler_downlink_frequency_hz
                - state.nominal_downlink_frequency_hz;
            rx_session_update_observer(state.sdr.rx_session,
                state.rot.antenna_rotator.target_azimuth,
                state.rot.antenna_rotator.target_elevation,
                state.prediction.satellite_ephem.range_km,
                state.prediction.satellite_ephem.range_rate_km_s,
                doppler_offset);
        }

        // Service a pending TX request (dry-run / real burst / reject),
        // emitting the SENT/NOT_SENT event + tx-result audit. See
        // pipeline/tx_burst.c.
        tx_burst_service_request(&state);
#endif

        // --- IPC: serve clients, fan out state, honour SIGUSR1 yield ---
        // Always service the socket (cheap; accepts new viewers) but
        // throttle STATE broadcasts to 2 Hz so viewers don't get
        // hammered when the loop is running at UHD-chunk cadence.
        if (state.ipc) {
            sso_ipc_server_step(state.ipc, 0);
            // Live-audio relay: ship encoded RX audio to any subscribed
            // viewer, then drop subscribers whose client has gone. Both are
            // cheap no-ops when nobody is listening.
            operator_audio_pump(&state);
            operator_audio_prune(&state);
            if ((t_now - t_last_ipc_broadcast) >= IPC_BROADCAST_PERIOD_S) {
                ipc_broadcast_state(&state, current_az, current_el,
                                     current_downlink_frequency,
                                     doppler_delta_downlink,
                                     jul_utc);
                t_last_ipc_broadcast = t_now;
            }
            // Debounced cmd-preview broadcast: viewers see the operator's
            // ":" prompt as it's typed, lagging so we don't fire a packet
            // per keystroke.
            cmd_pump(&state);
        }
        if (tui_yield_requested()) {
            sso_audit_event("yield-requested",
                            "SIGUSR1 (--force takeover) — exiting");
            state.running = 0;
        }

        // Surface a finished spectrum render so the operator sees the
        // outcome (PNG path or ffmpeg error) in the command-line status.
        // The reap only joins the worker thread; status_msg is left
        // alone, so reading it after reap is safe.
        if (state.spec_job.active && state.spec_job.done) {
            if (state.spec_job.status_msg[0]) {
                cmd_set_status(&state, "%s", state.spec_job.status_msg);
            }
            spectrum_job_reap(&state);
        }

        if (state.running) {
            // The B210 worker thread pumps UHD on its own pthread now,
            // so the main loop doesn't pace itself off the radio. Sleep
            // at the historical 2 Hz so rotator-STATUS polls don't ramp
            // up unexpectedly; redraw/IPC gates do their own throttling.
            // Exception: while the operator is typing in the ":" prompt,
            // drop to 20 ms so getch() echoes each keystroke promptly
            // (the 500 ms tick was capping input at ~2 chars/sec).
            useconds_t nap = UPDATE_INTERVAL_MICROSEC;
            if (state.cmd.active || state.tx_compose_active || state.auto_tcmd_active) {
                nap = 20000;
            } else if (operator_audio_active()) {
                // A viewer is listening live — tick at 10 Hz so encoded
                // audio flows with low latency instead of in 0.5 s gulps.
                nap = 100000;
            }
            usleep(nap);
        }
    }

    // Tear down live-audio encoders (and clear the RX tap) while the RX
    // session is still open.
    operator_audio_shutdown(&state);

    endwin();
    tui_release_stderr();
    if (state.trsw.have_tr_switch) {
        tr_switch_disconnect(&state.trsw.tr_switch);
        state.trsw.have_tr_switch = 0;
    }
    // Join the rotator worker before closing the serial FD — otherwise a
    // mid-read in the worker would see EBADF and corrupt the snapshot.
    if (state.rot.rot_async != NULL) {
        antenna_rotator_async_close(state.rot.rot_async);
        state.rot.rot_async = NULL;
    }
    if (state.rot.have_antenna_rotator) {
        antenna_rotator_disconnect(&state.rot.antenna_rotator);
        state.rot.have_antenna_rotator = 0;
    }
    // Free any plan that survived (mid-pass exit / crash on a key
    // before the LOS branch had a chance to clear it).
    main_pursuit_clear_plan(&state.rot);
    if (state.ipc) {
        sso_ipc_server_close(state.ipc);
        state.ipc = NULL;
    }
    // Politely terminate the live raylib waterfall if we spawned one.
    live_waterfall_shutdown();
#ifdef SSO_WITH_SDR
    char final_wav_path[512] = "";
    char final_iq_path[512]  = "";
    int  final_iq_rate       = 0;
    if (state.sdr.rx_session) {
        // Snapshot both sidecar paths before close so the full-pass
        // renderers can find the closed files on disk. Both paths
        // persist across wav_stop in rx_session.
        rx_session_wav_snapshot(state.sdr.rx_session,
                                final_wav_path, sizeof final_wav_path,
                                NULL, NULL, NULL);
        rx_session_iq_snapshot(state.sdr.rx_session,
                               final_iq_path, sizeof final_iq_path,
                               NULL, &final_iq_rate);
        // The worker owns the WAV writer and the B210 core. Closing
        // the session signals the worker to stop, joins it, then
        // tears down both. Any open WAV / .iq gets its header patched
        // (WAV) or its trailer flushed (IQ).
        rx_session_request_wav_stop(state.sdr.rx_session);
        rx_session_close(state.sdr.rx_session);
        state.sdr.rx_session = NULL;
    }

    // Any in-flight `:spectrum N` worker is touching the same WAV / IQ
    // — let it finish before we hand the file to the full-pass render.
    if (state.spec_job.active) {
        pthread_join(state.spec_job.thr, NULL);
        state.spec_job.active = 0;
        if (state.spec_job.status_msg[0]) {
            fprintf(stderr, "simple_sat_ops: %s\n", state.spec_job.status_msg);
        }
    }

    // Full-pass renderer. Prefer the IQ → gen_waterfall path because it
    // gives SatNOGS-style waterfalls (real complex FFT, median-subtracted
    // floor, viridis). Fall back to the FM-demod WAV via ffmpeg when the
    // IQ sidecar isn't on disk (e.g., disk full, mid-pass shutdown).
    int did_iq = 0;
    if (final_iq_path[0] && final_iq_rate > 0) {
        struct stat st;
        if (stat(final_iq_path, &st) == 0 && st.st_size > 0) {
            char png[640];
            if (generate_full_iq_waterfall(final_iq_path, final_iq_rate,
                                            png, sizeof png) == 0) {
                fprintf(stderr, "simple_sat_ops: waterfall -> %s\n", png);
                did_iq = 1;
            } else {
                fprintf(stderr,
                    "simple_sat_ops: gen_waterfall failed for %s "
                    "(gen_waterfall on PATH?)\n", final_iq_path);
            }
        }
    }
    if (!did_iq && final_wav_path[0]) {
        struct stat st;
        if (stat(final_wav_path, &st) == 0 && st.st_size > 44) {
            char png[640];
            if (generate_full_spectrogram(final_wav_path, png, sizeof png) == 0) {
                fprintf(stderr, "simple_sat_ops: pass spectrogram -> %s\n", png);
            } else {
                fprintf(stderr,
                    "simple_sat_ops: ffmpeg spectrogram failed for %s "
                    "(ffmpeg on PATH?)\n", final_wav_path);
            }
        }
    }
#endif

    if (state.prediction.auto_sat) {
        free_passes();
    }

    // Final line: tell the operator whether anything landed in the
    // redirected stderr log during the pass.
    tui_report_errors();
    return 0;
}

