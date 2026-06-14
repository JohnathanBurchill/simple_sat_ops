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
#include "rotator_calibrate.h"
#include "pursuit.h"
#include "tr_switch.h"
#include "state.h"
#include "prediction.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_operator.h"
#include "frontiersat.h"
#include "hmac_keyfile.h"
#include "tcmd_lint.h"
#include "sso_time.h"
#include "sso_version.h"
#include "panels.h"
#include "pass_session.h"
#include "scan_sky.h"
#include "operator_ipc.h"
#include "tracking.h"
#include "spectrogram.h"
#include "tui.h"
#include "auto_tcmd.h"
#include "cmd_line.h"
#include "tx_compose.h"
#include "viewer.h"
#include "tx_log.h"
#include "cli_args.h"
#include "argparse.h"

#ifdef SSO_WITH_SDR
#include "b210_rx_tx_core.h"
#include "carrier_trim.h"
#include "rx_session.h"
#include "tx_burst.h"
#endif

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <locale.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <sys/wait.h>
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

// How close a STATUS azimuth must be to the just-commanded home waypoint to
// be treated as the controller's post-SET target echo -- which it reports
// for a couple of seconds before its feedback shows real motion -- rather
// than a real position reading. Used to gate the two-step home's final leg.
#define HOME_ECHO_TOLERANCE_DEG 2.0

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

// Live raylib waterfall viewer. Off by default; --live-waterfall on
// the command line opts in. When recording starts, fork+exec the
// live_waterfall binary with the active .iq path. Track the child
// PID so we can SIGTERM it on shutdown. The iq-path scratch is only
// referenced inside the WITH_USRP_B210 launch block, so tag it
// unused — the cleanup path at the bottom of main() does use the
// pid, so that one stays unannotated.

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

static pid_t g_live_waterfall_pid = -1;
__attribute__((unused))
static char  g_live_waterfall_iq[512] = "";
// Write end of the pipe whose read end is dup2'd to the viewer's
// stdin. Colon commands like :wf_zoom_khz write line-based commands
// here so the running viewer can adjust without a relaunch. -1 when
// no viewer is alive.
static int   g_live_waterfall_stdin_fd = -1;

// Accessor so the (extracted) command-line :wf_* handlers can write to the
// live-waterfall child's stdin without owning the pipe; main spawns/reaps it.
int live_waterfall_stdin_fd(void) { return g_live_waterfall_stdin_fd; }

// --- main ---------------------------------------------------------

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "simple_sat_ops")) return 0;
    state_t state = {0};
    state.prediction.predicted_max_elevation = -180.0;
    // Seed the TX-compose "remembered" draft. state_t is zero-initialised,
    // so set the first-open defaults explicitly: the CTS1 prefix and 80 dB.
    snprintf(state.tx_last_payload, sizeof state.tx_last_payload, "CTS1+");
    snprintf(state.tx_last_power,   sizeof state.tx_last_power,   "80.0");
    // Non-zero TX-core defaults (state_t is zero-initialised). The Doppler
    // carrier falls back to the bare nominal until SGP4 has a range rate;
    // preroll matches the tx_burst_run fallback. Both may be overridden in
    // apply_args (--tx-preroll-ms) / the per-tick Doppler refresh.
    state.tx_freq_hz_doppler = (long) FRONTIERSAT_CARRIER_HZ;
    state.tx_preroll_ms      = 200;

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

#ifdef SSO_WITH_SDR
    // Pin the SSO+ @tssent dedup key for this session: the startup UTC,
    // truncated to the minute. Constant for the life of the process so the
    // satellite runs an SSO+ time-sync once per pass. See sso_pseudo.h.
    state.sso_pass_tssent_ms = (sso_now_utc_ms() / 60000LL) * 60000LL;
#endif

    // Resolve + load the HMAC keyfile. The bytes feed every TX burst's
    // AX100 frame (CTS1 firmware expects HMAC on every uplink), AND
    // light the operator banner — "(N bytes ok)" means TX is armed,
    // "(MISSING)" / "(BAD)" means the next TX request will be refused
    // before keying the PA. If --hmac-keyfile= wasn't given, fall back
    // to hmac_keyfile_default_path (shared first, per-user second).
    if (state.hmac_keyfile_path[0] == '\0') {
        if (hmac_keyfile_default_path(state.hmac_keyfile_path,
                                      sizeof state.hmac_keyfile_path) != 0) {
            state.hmac_keyfile_path[0] = '\0';
            state.hmac_display_status  = HMAC_DISPLAY_MISSING;
        }
    }
    if (state.hmac_keyfile_path[0] != '\0') {
        struct stat st;
        if (stat(state.hmac_keyfile_path, &st) != 0) {
            state.hmac_display_status = HMAC_DISPLAY_MISSING;
        } else {
            ssize_t got = hmac_keyfile_load(state.hmac_keyfile_path,
                                            state.hmac_key,
                                            sizeof state.hmac_key);
            if (got > 0) {
                state.hmac_display_status = HMAC_DISPLAY_OK;
                state.hmac_key_len        = (size_t) got;
            } else {
                state.hmac_display_status = HMAC_DISPLAY_BAD;
                state.hmac_key_len        = 0;
                memset(state.hmac_key, 0, sizeof state.hmac_key);
            }
        }
    }

    // Telecommand-agenda lint gate. When a --tc-file was given, lint it
    // against the firmware's telecommand set (names, argument counts,
    // CTS1+...! framing, length limits) BEFORE bringing up anything that
    // can key the PA. Lint errors mean a command would be rejected (or
    // worse, mis-parsed) by the satellite, so refuse to start unless the
    // operator explicitly accepts the risk. Warnings (e.g. a command not
    // meant for routine flight operation) are printed but do not block.
    if (state.auto_tcmd_file_path[0] != '\0') {
        int tc_warns = 0;
        int tc_errs = tcmd_lint_file(state.auto_tcmd_file_path, stderr, &tc_warns);
        if (tc_errs > 0 && !state.ignore_tc_errors) {
            fprintf(stderr,
                "simple_sat_ops: %d error%s detected in the --tc-file content (%s).\n"
                "Refusing to start. Fix the agenda, or re-run with\n"
                "--ignore-at-your-peril-all-tc-errors to bypass this check.\n",
                tc_errs, tc_errs == 1 ? "" : "s", state.auto_tcmd_file_path);
            return EXIT_FAILURE;
        }
        if (tc_errs > 0) {
            fprintf(stderr,
                "simple_sat_ops: %d telecommand error%s in %s -- proceeding anyway "
                "(--ignore-at-your-peril-all-tc-errors).\n",
                tc_errs, tc_errs == 1 ? "" : "s", state.auto_tcmd_file_path);
        } else if (tc_warns > 0) {
            fprintf(stderr,
                "simple_sat_ops: %d telecommand warning%s in %s (see above); proceeding.\n",
                tc_warns, tc_warns == 1 ? "" : "s", state.auto_tcmd_file_path);
        }
    }

    // Audit + operator IPC bring-up.
    state.operator_user = sso_unix_user();
    sso_audit_start("simple_sat_ops",
                    state.control_mode ? "operator" : "standalone");
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
    if (state.control_mode) {
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

        state.ipc = sso_ipc_server_open("simple_sat_ops");
        if (state.ipc == NULL) {
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
        sso_ipc_server_on_event(state.ipc, ipc_on_event, &state);
        tui_install_yield_handler();
        fprintf(stderr, "simple_sat_ops: operator=%s ipc=on\n",
                state.operator_user);
    }

    /* Parse TLE data */
    int tle_status = load_tle(&state.prediction);
    if (tle_status) {
        return tle_status;
    }
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&state.prediction.satellite_ephem.tle);

    // Seed the retarget guard with the startup TLE so a `:retarget` on
    // the same file is correctly a no-op.
    snprintf(state.target_tle_path, sizeof state.target_tle_path, "%s",
             state.prediction.tles_filename
                 ? state.prediction.tles_filename : "");

    // With a fresh TLE loaded, find the upcoming pass and stand up
    // /FrontierSat/Operations/<yyyymmdd>/<hhmmLT>/ for it before the
    // tracking loop opens ncurses. Only on --control — the
    // standalone-tracker / dev path leaves Operations/ alone.
    if (state.control_mode) {
        UTC_Calendar_Now(&utc, &tv);
        double jul_now = Julian_Date(&utc, &tv);
        update_satellite_position(&state.prediction, jul_now);
        setup_pass_folder(&state, jul_now);
        if (state.pass_folder[0]) {
            generate_pass_plot(&state, state.pass_folder, jul_now);
        }
    }

    int antenna_rotator_result = 0;
    if (state.run_with_antenna_rotator) {
        state.antenna_rotator.is_required = 1;
        antenna_rotator_result = antenna_rotator_init(&state.antenna_rotator);
        if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
            fprintf(stderr, "Error initializing antenna rotator\n");
            // --self-test is a dry run: a missing rotator must not abort the
            // bring-up, so the configuration report still prints. Outside
            // --self-test this stays fatal.
            if (!state.self_test) {
                return EXIT_FAILURE;
            }
            fprintf(stderr,
                    "--self-test: continuing without the rotator (dry run)\n");
        } else {
            state.have_antenna_rotator = 1;
            // Spawn the async worker. From here on, every serial roundtrip
            // happens on the worker thread; the main loop only reads the
            // snapshot via main_rotator_refresh_targets_from_snapshot() and
            // posts SETs via main_rotator_submit_set().
            if (antenna_rotator_async_open(&state.rot_async,
                                            &state.antenna_rotator, 0.5) != 0) {
                fprintf(stderr, "Error spawning antenna rotator worker\n");
                if (!state.self_test) {
                    return EXIT_FAILURE;
                }
                fprintf(stderr,
                        "--self-test: rotator worker unavailable (dry run)\n");
                state.have_antenna_rotator = 0;
            } else {
                // Adopt whatever extended position the SPID is already at so the
                // unwrapped accumulator starts grounded in reality. We wait
                // briefly for the worker's first STATUS read; the timeout is
                // bounded so a missing controller doesn't hang startup.
                //
                // The seed snapshot also overwrites target_* with the current
                // physical position — fine when nobody asked for a specific park
                // position, but a problem when the operator passed
                // --rotator-target-azimuth / --rotator-target-elevation: those
                // user-specified targets would be silently clobbered before T
                // ever fired. Snapshot them and restore after seeding.
                double sav_az    = state.antenna_rotator.target_azimuth;
                double sav_el    = state.antenna_rotator.target_elevation;
                double sav_az_uw = state.antenna_rotator.target_azimuth_unwrapped;
                int    sav_uw_ok = state.antenna_rotator.unwrapped_target_valid;
                if (antenna_rotator_async_wait_first_status(state.rot_async, 1500) != 0
                    || main_rotator_refresh_targets_from_snapshot(&state) != 0) {
                    fprintf(stderr, "Warning: could not read SPID position; "
                                    "check that the Rot2ProG is in 'A' mode\n");
                }
                if (state.antenna_rotator.fixed_target) {
                    state.antenna_rotator.target_azimuth            = sav_az;
                    state.antenna_rotator.target_elevation          = sav_el;
                    state.antenna_rotator.target_azimuth_unwrapped  = sav_az_uw;
                    state.antenna_rotator.unwrapped_target_valid    = sav_uw_ok;
                }
            }
        }
    }

    // --calibrate-rotator: drive the antenna across known arcs to
    // measure deg/s on each axis, save the result to disk, and exit
    // without entering the operator UI. Requires the safety interlock
    // so a stray flag in a script can't move hardware.
    if (state.calibrate_rotator) {
        if (!state.have_antenna_rotator) {
            fprintf(stderr, "--calibrate-rotator: no rotator open "
                            "(was --without-rotator passed?)\n");
            return EXIT_FAILURE;
        }
        if (!state.confirm_rotator_calibrate) {
            fprintf(stderr,
                    "--calibrate-rotator will physically move the antenna.\n"
                    "Confirm the mast area is clear, then re-run with\n"
                    "  --calibrate-rotator --confirm-rotator-calibrate\n");
            return EXIT_FAILURE;
        }
        double az_dps = 0.0, el_dps = 0.0;
        rotator_calibrate_result_t cres = rotator_calibrate_run(
            state.rot_async, &az_dps, &el_dps, stderr);
        fprintf(stderr, "calibrate: result = %s\n",
                rotator_calibrate_result_name(cres));
        if (cres == ROTATOR_CALIBRATE_OK) {
            fprintf(stderr,
                    "calibrate: saved rates az=%.3f deg/s el=%.3f deg/s\n",
                    az_dps, el_dps);
        }
        // Shutdown cleanly — the operator UI never started, but the
        // rotator FD and worker are open.
        if (state.rot_async != NULL) {
            antenna_rotator_async_close(state.rot_async);
            state.rot_async = NULL;
        }
        if (state.have_antenna_rotator) {
            antenna_rotator_disconnect(&state.antenna_rotator);
            state.have_antenna_rotator = 0;
        }
        return (cres == ROTATOR_CALIBRATE_OK) ? 0 : 1;
    }

    // Normal startup: load saved rotator rates from the calibration
    // file. Missing or malformed file -> pursuit planner stays
    // disabled (Phase 2 hooks this in front of the track loop; Phase
    // 1 just loads + warns so the bench can see the values).
    if (state.have_antenna_rotator) {
        if (pursuit_load_rotator_rates(&state.pursuit_az_dps,
                                        &state.pursuit_el_dps) == 0) {
            fprintf(stderr,
                    "pursuit: loaded slew rates az=%.3f deg/s el=%.3f deg/s\n",
                    state.pursuit_az_dps, state.pursuit_el_dps);
        } else {
            fprintf(stderr,
                    "pursuit: no calibration on disk; run "
                    "`simple_sat_ops --calibrate-rotator "
                    "--confirm-rotator-calibrate` to enable lead-aim\n");
        }
    }

    // T/R antenna switch — auto-probe before ncurses takes the screen,
    // so a "not connected" warning lands on the terminal. Absent or
    // inaccessible hardware is non-fatal; the UI panel reads "not
    // connected" and the program runs normally.
    if (state.run_with_tr_switch) {
        if (tr_switch_init(&state.tr_switch) == 0) {
            state.have_tr_switch = 1;
        } else {
            fprintf(stderr,
                    "T/R switch: could not open %s (skipping; "
                    "pass --without-tr-switch to silence)\n",
                    state.tr_switch.device_filename
                        ? state.tr_switch.device_filename : "?");
        }
    }

#ifdef SSO_WITH_SDR
    // Open the B210 once, here, before ncurses init — soft-fail on any
    // UHD error so a dev host without a device can still run the UI.
    // rx_session takes ownership of the core; we drop our local handle
    // afterwards so main never touches UHD off-thread.
    if (state.control_mode && !state.without_b210) {
        // B210 RX rate doubled from the original 240 kHz / sps=5 to
        // 480 kHz / sps=10 (after the integer-5 decimation FIR). That
        // gives the modem_fsk clock-recovery loop the same oversampling
        // headroom the gr-satellites / AIT chain has (sps=6 with PFB-
        // Gardner) and then some, which is worth ~1-2 dB at marginal
        // SNR on real captures. The post-decim signal still only
        // carries the FrontierSat ±10 kHz FSK, so the decim FIR
        // cutoff stays at 18 kHz — narrower than the new 48 kHz
        // Nyquist, so the filter rejects more noise than it did at
        // the old 24 kHz Nyquist. IQ files double in size; with the
        // sustained-write rate at 96 kHz·2·2 = 384 kB/s, a 10-minute
        // pass produces ~230 MB which the laptop SSD has no trouble
        // with.
        b210_rx_tx_core_params_t cp = {
            // Tune the SDR LO off the nominal carrier so the corrected
            // signal lands well off DC. rx_lo_offset_hz is SIGNED:
            // positive → LO above nominal (signal at negative baseband),
            // negative → LO below (signal at positive baseband). Default
            // -25 kHz keeps existing pipelines unchanged; operator can
            // shift to dodge fixed-pattern noise.
            .freq_hz         = state.nominal_downlink_frequency_hz
                             + state.rx_lo_offset_hz,
            .rate_hz         = 480000.0,
            .gain_db         = state.rx_gain_db,
            .bw_hz           = -1.0,
            .fm_fullscale_hz = 25000.0,
            .rx_antenna      = "RX2",
            // fir_decim budget:
            //   - operator LO offset clamped ±45 kHz (apps/main.c
            //     KEY_LO_OFFSET clamp);
            //   - Doppler swing ±10 kHz for a typical LEO pass;
            //   - FM envelope ±5 kHz around the carrier;
            //   - Nyquist after decim by 5 = ±48 kHz.
            // Cutoff 42 kHz with 256 taps gives ~6 kHz transition
            // before Nyquist and lets the carrier sit anywhere
            // inside the clamp without the LPF rolling off half the
            // beacon. The carrier-at-DC convention moved to the
            // decode-only buffer (see b210_rx_tx_core.c); the IQ
            // tap now carries the carrier at +lo_offset baseband.
            .decim_factor    = 5u,
            .decim_cutoff_hz = 42000.0,
            .decim_taps      = 256u,
            // FM-path LO compensation: the core's second NCO cancels
            // the operator's lo_offset, the UHD-reported tune residual
            // (target − actual, from the AD9361 PLL step), AND the
            // persistent per-host carrier trim (TCXO calibration). The
            // carrier lands at exactly DC for every downstream consumer
            // (.iq sidecar, live waterfall, shadow IQ decoder, FM
            // discriminator).
            .rx_dc_offset_track  = state.rx_dc_offset_track,
            .rx_iq_balance_track = state.rx_iq_balance_track,
            .fm_lo_compensation_hz = state.rx_lo_offset_hz,
            .carrier_trim_hz       = carrier_trim_load_hz(),
            // SDR backend selection: type (default auto), and the UHD
            // clone overrides. --sdr-device routes to the UHD device
            // args when given (e.g. "serial=..."); --uhd-args takes
            // precedence and is passed verbatim.
            .backend_type        = state.sdr_type,
            .device_args         = state.sdr_device[0] ? state.sdr_device : "type=b200",
            .uhd_args_override   = state.uhd_args[0] ? state.uhd_args : NULL,
            .fpga_image_path     = state.sdr_fpga[0] ? state.sdr_fpga : NULL,
            // RTL-SDR dongle index (UHD ignores it; for UHD use --uhd-args).
            .device_index        = state.sdr_device[0] ? atoi(state.sdr_device) : 0,
        };
        b210_rx_tx_core_t *core = NULL;
        if (b210_rx_tx_core_open(&cp, &core) != 0) {
            fprintf(stderr,
                "simple_sat_ops: B210 open failed — continuing without RF "
                "(rotator + UI only). Pass --without-b210 to silence.\n");
            sso_audit_event("b210-open-failed", "");
        } else {
            fprintf(stderr,
                "simple_sat_ops: SDR open at %.6f MHz (post-decim rate %.0f, "
                "tx=%s)\n",
                b210_rx_tx_core_actual_freq(core) / 1e6,
                b210_rx_tx_core_actual_rate(core),
                b210_rx_tx_core_can_tx(core) ? "yes" : "no (RX-only)");
            {
                char det[256];
                snprintf(det, sizeof det,
                    "freq_hz=%.0f rate_hz=%.0f lo_offset_hz=%.0f",
                    b210_rx_tx_core_actual_freq(core),
                    b210_rx_tx_core_actual_rate(core),
                    state.rx_lo_offset_hz);
                sso_audit_event("b210-open", det);
            }
            rx_session_params_t rxp = {
                .bit_rate          = 9600,
                .window_s          = 1.5,
                .slide_s           = 0.5,
                .sync_max_ham      = 4,
                .use_hmac          = 0,
                .use_rs            = 1,
                .force_beacon      = 0,
                .show_packet_headers = 0,
                .csp_crc32         = 0,
                .pass_folder       = state.pass_folder[0] ? state.pass_folder : NULL,
                .want_wav          = 1,
                .tle_path          = state.prediction.tles_filename,
                .sat_name          = state.prediction.satellite_ephem.tle.sat_name[0]
                                     ? state.prediction.satellite_ephem.tle.sat_name
                                     : NULL,
                .session_dir       = state.pass_folder[0] ? state.pass_folder : NULL,
                .lo_offset_hz      = state.rx_lo_offset_hz,
            };
            if (rx_session_open(&state.rx_session, &rxp, core) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: rx_session_open failed — closing B210\n");
                b210_rx_tx_core_close(core);
            }
            // rx_session_open succeeded → it owns `core` now.
            core = NULL;
            // --always-record: start WAV + .iq + sidecars right now,
            // before any pass logic gets a chance to gate them. The
            // per-pass start/stop block in the tracking loop checks
            // state.always_record and skips itself when this is on.
            // Suppressed under --self-test: the dry run opens the SDR to
            // prove it comes up, but must never write capture files.
            if (state.always_record && state.rx_session && !state.self_test) {
                rx_session_request_wav_start(state.rx_session);
                fprintf(stderr,
                    "simple_sat_ops: --always-record on — WAV/IQ "
                    "capture started, pass gating disabled\n");
                sso_audit_event("rec-start", "trigger=always-record");
            }
        }
    }
#endif

    /* Tracking loop */
    double jul_idle_start = 0;  // last-tracked timestamp

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
        if (state.rx_session) {
            rx_session_close(state.rx_session);
            state.rx_session = NULL;
        }
#endif
        if (state.rot_async != NULL) {
            antenna_rotator_async_close(state.rot_async);
            state.rot_async = NULL;
        }
        if (state.have_antenna_rotator) {
            antenna_rotator_disconnect(&state.antenna_rotator);
            state.have_antenna_rotator = 0;
        }
        if (state.have_tr_switch) {
            tr_switch_disconnect(&state.tr_switch);
            state.have_tr_switch = 0;
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
    tui_register_waterfall_pid(&g_live_waterfall_pid);

    // int (not char) — getch returns KEY_* codes well above 127 for
    // arrow keys / function keys / KEY_BACKSPACE etc. A signed char
    // would silently truncate those into bogus low-byte values, which
    // is what made KEY_LEFT (260) look like Ctrl-D (4) in the modal
    // handlers and broke field editing inside the auto-tcmd modal.
    int key = ERR;
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
    double last_az = 0;
    double last_el = 0;

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
        if (state.have_tr_switch) {
            tr_switch_pump(&state.tr_switch, t_now);
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
        if (state.rx_session && !state.always_record) {
            double sec_to_aos =
                state.prediction.predicted_minutes_until_visible * 60.0;
            int visible   = (state.prediction.satellite_ephem.elevation > 0.0);
            int in_preroll = (sec_to_aos > 0.0
                              && sec_to_aos <= RECORDING_PREROLL_S);
            int active = rx_session_wav_active(state.rx_session);
            if (!active && (visible || in_preroll)) {
                rx_session_request_wav_start(state.rx_session);
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
                    rx_session_request_wav_stop(state.rx_session);
                    t_recording_close_at = 0.0;
                    sso_audit_event("rec-stop", "trigger=postroll-expired");
                }
            }
        }
#endif

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

        // Drive the second leg of a two-step home. The first leg drops a mid
        // waypoint to start the antenna unwinding; the final 'go to target'
        // must wait until the antenna has unwound far enough that the
        // controller's SHORT path to the target runs the SAME way as the
        // unwind -- i.e. it has reached the 0..180 zone on the unwinding side.
        // Until then the short path is the opposite (winding) way, so issuing
        // the target now sends it back around and it winds up (330 -> 360).
        //
        // Complication: after a SET the controller's STATUS reports the
        // just-commanded target (the mid waypoint) for a couple of seconds
        // before its feedback shows real motion. So a reading that still
        // equals the commanded mid is that echo, not the real position --
        // ignore it. The first reading that DIFFERS is the antenna's true
        // position; act on that. Mid-slew the real position is far from the
        // mid waypoint, so there's no echo-vs-arrival ambiguity. (Unwinds
        // past a full turn, prev > 360, would need more than one waypoint;
        // a single pass winds < 360, so one mid waypoint suffices.)
        if (state.antenna_rotator.homing_in_progress
            && state.have_antenna_rotator) {
            double final_az  = state.antenna_rotator.home_pending_final_az;
            double mid_az    = state.antenna_rotator.target_azimuth_unwrapped;
            double from_mid  = fabs(antenna_rotator_wrap_to_pm180(current_az - mid_az));
            double unwind    = final_az - mid_az;   // sign = unwind direction
            double remaining = antenna_rotator_wrap_to_pm180(final_az - current_az);
            int in_zone = (remaining == 0.0)
                       || ((remaining > 0.0) == (unwind > 0.0));
            // from_mid > tol => the reading is real feedback, not the post-SET
            // target echo. The two-step always starts out of the unwind zone
            // (|prev| > 180), so the stale pre-SET reading can't fire early.
            if (from_mid > HOME_ECHO_TOLERANCE_DEG && in_zone) {
                int rc = main_rotator_submit_set(&state, final_az, 0.0);
                if (rc == ANTENNA_ROTATOR_OK) {
                    state.antenna_rotator.antenna_is_moving = 1;
                }
                state.antenna_rotator.homing_in_progress = 0;
                state.antenna_rotator.home_pending_final_az = 0.0;
                char det[96];
                snprintf(det, sizeof det, "leg2 fired at az=%.1f -> %.1f",
                         current_az, final_az);
                sso_audit_event("home", det);
            }
        }
        // --scan-sky: drives a sky grid one target at a time, dwelling
        // SCAN_DWELL_S at each. Bypasses the satellite_tracking +
        // pass-timing gate below entirely, so the operator can scan
        // regardless of TLE / pass state. 's' stops mid-scan.
        if (state.scan.active) {
            scan_sky_tick(&state, t_now);
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
                        // Prefer the prediction-derived AOS azimuth (the
                        // satellite_ephem.azimuth here may be a few deg
                        // off as we are still pre-AOS); fall back to the
                        // live position if the pass walk didn't capture
                        // an ascension sample.
                        double aos_az_pred =
                            state.prediction.predicted_ascension_azimuth;
                        state.antenna_rotator.flip_aos_az =
                            (aos_az_pred != 0.0)
                                ? aos_az_pred
                                : state.prediction.satellite_ephem.azimuth;
                        state.antenna_rotator.flip_los_az =
                            state.prediction.predicted_descent_azimuth;
                        state.antenna_rotator.flip_aos_jul =
                            state.prediction.predicted_ascension_jul_utc;
                        state.antenna_rotator.flip_los_jul =
                            state.prediction.predicted_descent_jul_utc;
                    }
                    state.antenna_rotator.flip_decision_made = 1;
                    state.antenna_rotator.tracking = 1;
                    // Pre-sample the trajectory and ask the planner
                    // for a rate-feasible whole-pass aim sequence. On
                    // any failure (no calibration, planner unhappy,
                    // --without-rotator-pursuit) the helper leaves
                    // state.pursuit_plan zero and the track loop below
                    // falls back to today's aim-where-sat-is-now path.
                    main_pursuit_build_plan(&state, jul_utc);
                }
            }

            if (state.antenna_rotator.tracking
                && state.antenna_rotator.antenna_is_under_control) {
                if (!state.antenna_rotator.unwrapped_target_valid) {
                    if (main_rotator_refresh_targets_from_snapshot(&state)
                        != 0) {
                        state.antenna_rotator.tracking = 0;
                        main_pursuit_clear_plan(&state);
                    }
                } else if (!state.antenna_rotator.antenna_is_moving) {
                    double next_az = 0.0, next_el = 0.0;
                    double prev_unwrapped =
                        state.antenna_rotator.target_azimuth_unwrapped;
                    int    used_pursuit = 0;
                    if (state.pursuit_plan.waypoints != NULL
                        && pursuit_aim_at(&state.pursuit_plan, jul_utc,
                                          &next_az, &next_el) == 0) {
                        used_pursuit = 1;
                    }
                    if (!used_pursuit) {
                        double pred_az =
                            state.prediction.satellite_ephem.azimuth;
                        double pred_el =
                            state.prediction.satellite_ephem.elevation;
                        double mech_az = pred_az;
                        double mech_el = pred_el;
                        int half = 0;
                        // AOS->LOS progress: drives the boom-meridian
                        // lerp in flip mode. Clamped to [0, 1] inside
                        // the function. Ignored for non-flip passes.
                        double progress = 0.0;
                        double pass_jul =
                            state.antenna_rotator.flip_los_jul
                            - state.antenna_rotator.flip_aos_jul;
                        if (pass_jul > 0.0) {
                            progress = (jul_utc
                                        - state.antenna_rotator.flip_aos_jul)
                                       / pass_jul;
                        }
                        antenna_rotator_to_mech_coords(
                            state.antenna_rotator.flip_mode_pass,
                            state.antenna_rotator.flip_aos_az,
                            state.antenna_rotator.flip_los_az,
                            progress,
                            pred_az, pred_el,
                            &mech_az, &mech_el, &half);
                        if (state.antenna_rotator.flip_mode_pass
                            && half != state.antenna_rotator.flip_half) {
                            state.antenna_rotator.target_azimuth_unwrapped =
                                mech_az;
                            state.antenna_rotator.flip_half = half;
                            prev_unwrapped =
                                state.antenna_rotator.target_azimuth_unwrapped;
                        }
                        next_az = antenna_rotator_accumulate_unwrapped(
                            prev_unwrapped, mech_az);
                        next_el = mech_el;
                    }
                    if (next_el < ANTENNA_ROTATOR_MINIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MINIMUM_ELEVATION;
                    } else if (next_el > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                        next_el = ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                    }
                    delta_az = next_az - prev_unwrapped;
                    // With a plan in play the elevation is part of the
                    // trajectory; respect it even below the horizon.
                    // The pre-pursuit fallback keeps the existing
                    // "only chase el while the sat is visible" rule.
                    if (used_pursuit
                        || state.antenna_rotator.flip_mode_pass
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
                            main_pursuit_clear_plan(&state);
                        } else {
                            int rc = main_rotator_submit_set(
                                &state, next_az, next_el);
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
                // Released the pass; tear down the planner so the
                // memory comes back and so the next pass / mid-pass
                // 'T' rebuilds against fresh state.
                main_pursuit_clear_plan(&state);
            }
        }
        (void) jul_idle_start;  // reserved for any future idle-window behavior

        int redraw_due = (t_now - t_last_redraw) >= REDRAW_PERIOD_S;
        if (redraw_due) {
            row = 1;
            col = 1;
            report_predictions(&state, jul_utc, &row, col);

            row++;
            report_status(&state, &row, col);
            row = 5;
            col = 50;
            report_position(&state, &row, col);
            row++;
            // Refresh the low-disk warning lazily — statvfs every 30 s
            // is plenty given how slowly disk fills.
            low_disk_refresh(&state, t_now);
            rx_panel_data_t rxd;
            rx_panel_collect_local(&state, &rxd);
            render_rx_panel(&rxd, &row, 50);

            clrtoeol();

            // Vertical ribbon on the right edge — bottom = newest, with
            // a bold '-' tick crawling up one row per second so the
            // timeline is visibly alive even when the signal is flat.
            int ribbon_col = COLS - 2;
            int ribbon_top = 1;
            int ribbon_bot = LINES - 2;
            if (ribbon_col >= 64 && ribbon_bot > ribbon_top) {
                render_ribbon_vertical(&rxd, ribbon_top, ribbon_bot, ribbon_col);
            }

            // TX log lives below the keyboard info / antenna status if
            // the terminal is tall enough to host it without colliding.
            int tx_log_row = LINES - TX_LOG_SIZE - 2;
            if (tx_log_row >= keyboard_info_row + 4) {
                render_tx_log_panel(&state, tx_log_row, 1);
            }
        }

        key = getch();
        if (state.tx_compose_active) {
            if (!tx_compose_handle_key(&state, key)) {
                tx_compose_close(&state);
            }
        } else if (state.auto_tcmd_active) {
            if (!auto_tcmd_handle_key(&state, key)) {
                auto_tcmd_close(&state);
            }
        } else if (state.cmd.active) {
            cmd_handle_key(key, &state);
        } else if (key == 'K') {
            keyboard_unlocked = !keyboard_unlocked;
        } else if (keyboard_unlocked) {
            switch (key) {
                case ':':
                    cmd_enter(&state);
                    break;
                case 'q':
                    state.running = 0;
                    break;
                case 'T':
                    if (state.scan.mode) {
                        scan_sky_start(&state);
                    } else {
                        start_tracking(&state);
                        if (state.antenna_rotator.fixed_target) {
                            char det[128];
                            snprintf(det, sizeof det,
                                "mode=fixed-target az=%.1f el=%.1f",
                                state.antenna_rotator.target_azimuth,
                                state.antenna_rotator.target_elevation);
                            sso_audit_event("track-on", det);
                        } else {
                            sso_audit_event("track-on",
                                state.prediction.satellite_ephem.tle.sat_name[0]
                                    ? state.prediction.satellite_ephem.tle.sat_name : "");
                        }
                    }
                    break;
                case 's':
                    if (state.scan.active) {
                        scan_sky_stop(&state, "user");
                    }
                    stop_tracking(&state);
                    break;
                case 'r':
                    stop_tracking(&state);
                    point_to_stationary_target(&state, 0.0, 0.0);
                    break;
                case '[':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ']':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '{':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, -1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '}':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_azimuth(
                        &state, 1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case ',':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, -5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '.':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, 5.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '<':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, -1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case '>':
                    state.satellite_tracking = 0;
                    state.antenna_rotator.antenna_is_under_control = 0;
                    antenna_rotator_result = main_rotator_increase_elevation(
                        &state, 1.0);
                    if (antenna_rotator_result == ANTENNA_ROTATOR_OK) {
                        state.antenna_rotator.antenna_is_moving = 1;
                    }
                    flushinp();
                    break;
                case 't':
                    tx_compose_open(&state);
                    break;
                case 'A':
                    auto_tcmd_open(&state);
                    break;
                default:
                    break;
            }
        }

        if (redraw_due) {
            // Width-padded prints (not clrtoeol) so we don't wipe the
            // signal ribbon that paints over the right edge of these rows.
            mvprintw(keyboard_info_row, 3, "%s : %-8s", "Keyboard",
                     keyboard_unlocked ? "unlocked" : "LOCKED");
            mvprintw(keyboard_info_row + 2, 0, "%-18s",
                     state.antenna_rotator.antenna_is_moving
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
        if (state.rx_session && (t_now - state.ribbon_last_t) >= 1.0) {
            double peak = -90.0;
            rx_session_snapshot(state.rx_session, NULL, &peak, NULL,
                                NULL, NULL, 0);
            int burst_bins = 0;
            rx_session_burst_snapshot(state.rx_session, &burst_bins, NULL);
            ribbon_push(&state, peak, burst_bins);
            state.ribbon_last_t = t_now;

            // Live waterfall: launch the raylib viewer the first time
            // a recording's .iq path appears, OR if the pass switched
            // to a new path. We poll once per second on the same
            // cadence as the ribbon — cheap, and a single second of
            // lag at viewer-launch is invisible to the operator.
            if (state.run_live_waterfall) {
                char iq_path[512] = "";
                int  iq_rate      = 0;
                rx_session_iq_snapshot(state.rx_session,
                                       iq_path, sizeof iq_path,
                                       NULL, &iq_rate);
                if (iq_path[0]
                    && strcmp(iq_path, g_live_waterfall_iq) != 0) {
                    // Tear down a viewer pointed at a stale path.
                    if (g_live_waterfall_pid > 0) {
                        kill(g_live_waterfall_pid, SIGTERM);
                        waitpid(g_live_waterfall_pid, NULL, 0);
                        g_live_waterfall_pid = -1;
                    }
                    if (g_live_waterfall_stdin_fd >= 0) {
                        close(g_live_waterfall_stdin_fd);
                        g_live_waterfall_stdin_fd = -1;
                    }
                    snprintf(g_live_waterfall_iq,
                             sizeof g_live_waterfall_iq, "%s", iq_path);
                    char rate_arg[32];
                    snprintf(rate_arg, sizeof rate_arg,
                             "--rate=%d",
                             iq_rate > 0 ? iq_rate : 96000);
                    // pipe()+dup2 so the parent can shove
                    // line-based commands (e.g. "zoom 60\n") at the
                    // viewer's stdin.
                    int pfd[2] = {-1, -1};
                    if (pipe(pfd) != 0) { pfd[0] = pfd[1] = -1; }
                    pid_t pid = fork();
                    if (pid == 0) {
                        if (pfd[0] >= 0) {
                            close(pfd[1]);
                            dup2(pfd[0], STDIN_FILENO);
                            close(pfd[0]);
                        }
                        char *args[] = {
                            (char *) "live_waterfall",
                            (char *) g_live_waterfall_iq,
                            rate_arg,
                            NULL
                        };
                        execvp("live_waterfall", args);
                        _exit(127);
                    } else if (pid > 0) {
                        g_live_waterfall_pid = pid;
                        if (pfd[0] >= 0) close(pfd[0]);
                        g_live_waterfall_stdin_fd = pfd[1];
                    } else {
                        if (pfd[0] >= 0) close(pfd[0]);
                        if (pfd[1] >= 0) close(pfd[1]);
                    }
                }
                // Reap a viewer that the operator closed via its
                // window — non-blocking so the main loop never stalls.
                if (g_live_waterfall_pid > 0) {
                    int status;
                    pid_t r = waitpid(g_live_waterfall_pid,
                                      &status, WNOHANG);
                    if (r == g_live_waterfall_pid) {
                        g_live_waterfall_pid = -1;
                        g_live_waterfall_iq[0] = '\0';
                        if (g_live_waterfall_stdin_fd >= 0) {
                            close(g_live_waterfall_stdin_fd);
                            g_live_waterfall_stdin_fd = -1;
                        }
                    }
                }
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
        if (state.rx_session && state.doppler_correction_enabled) {
            double offset = state.doppler_downlink_frequency_hz
                          - state.nominal_downlink_frequency_hz;
            rx_session_set_doppler_offset(state.rx_session, offset);
        }
        if (state.rx_session) {
            double doppler_offset =
                state.doppler_downlink_frequency_hz
                - state.nominal_downlink_frequency_hz;
            rx_session_update_observer(state.rx_session,
                state.antenna_rotator.target_azimuth,
                state.antenna_rotator.target_elevation,
                state.prediction.satellite_ephem.range_km,
                state.prediction.satellite_ephem.range_rate_km_s,
                doppler_offset);
        }

        // Service a pending TX request. Three paths:
        //
        //   1. --tx-dry-run:    synthesize "ok" without touching the
        //                       SDR. Auto-tcmd + compose still exercise
        //                       all their UI state on dev hosts.
        //   2. state.rx_session up: real burst — submitted async to the
        //                       worker, which pauses RX, transmits and
        //                       resumes RX (~1 s). The main loop keeps
        //                       running between submit and poll so the
        //                       rotator, redraw, IPC and the next auto-
        //                       tcmd tick aren't frozen by the burst.
        //                       state.tx_request.pending stays set across
        //                       the in-flight window, so auto-tcmd will
        //                       not queue a second burst on top.
        //   3. neither:         reject so auto-tcmd can move on. The
        //                       operator must have started simple_sat_ops
        //                       --without-b210 without also passing
        //                       --tx-dry-run; just clear the pending
        //                       slot rather than deadlocking.
        if (state.tx_request.pending) {
            char summary[SSO_TX_TEXT_MAX];
            const char *outcome = NULL;
            int  on_air = 0;
            int  finished = 0;        // emit the result + clear pending this tick
            if (state.tx_dry_run) {
                snprintf(summary, sizeof summary, "%s",
                         state.tx_request.summary);
                outcome = "dry-run";   // composed but deliberately not keyed
                finished = 1;
            } else if (state.hmac_key_len == 0) {
                // CTS1 expects HMAC on every uplink. Without a valid
                // key the burst would go out unsigned and the satellite
                // would silently drop it. Refuse here so the operator
                // sees a clear error instead of letting it go out unsigned.
                snprintf(summary, sizeof summary, "%s",
                         state.tx_request.summary);
                outcome = "rejected: no HMAC key (see banner)";
                finished = 1;
            } else if (state.rx_session != NULL && !rx_session_can_tx(state.rx_session)) {
                // RX-only backend (e.g. RTL-SDR): never reaches the air.
                // Backstop for a stale queued burst that slipped past the
                // compose / auto-tcmd gates.
                snprintf(summary, sizeof summary, "%s", state.tx_request.summary);
                outcome = "rejected: RX-only SDR";
                finished = 1;
            } else if (state.rx_session != NULL) {
                if (!state.tx_inflight) {
                    if (rx_session_submit_burst(state.rx_session, &state.tx_request,
                                                 state.hmac_key, state.hmac_key_len) == 0) {
                        state.tx_inflight = 1;
                        // Stay pending; we'll poll on subsequent ticks.
                    } else {
                        // Worker refused (slot already busy or rxs error).
                        snprintf(summary, sizeof summary, "%s",
                                 state.tx_request.summary);
                        outcome = "rejected: rx_session busy";
                        finished = 1;
                    }
                } else {
                    rx_burst_result_t br;
                    int done = rx_session_poll_burst(state.rx_session, &br,
                                                      summary, sizeof summary);
                    if (done == 1) {
                        switch (br) {
                            case RX_BURST_OK:                 outcome = "ok"; on_air = 1; break;
                            case RX_BURST_NO_CORE:            outcome = "rejected: no B210"; break;
                            case RX_BURST_FRAME_BUILD_FAILED: outcome = "rejected: frame build"; break;
                            case RX_BURST_UHD_ERROR:          outcome = "uhd-err"; break;
                        }
                        state.tx_inflight = 0;
                        finished = 1;
                    }
                    // else: still in flight; fall through and let the
                    // rest of the main loop run.
                }
            } else {
                snprintf(summary, sizeof summary, "%s",
                         state.tx_request.summary);
                outcome = "rejected: no B210";
                finished = 1;
            }
            if (finished) {
                // A command that made it on the air gets a plain TX
                // record, nothing more: the ground station can confirm
                // it transmitted, but only the satellite can acknowledge,
                // and that arrives on the downlink, not here. Anything
                // that did NOT reach the air (rejected, dry-run, uhd-err)
                // gets a not-sent note carrying the reason.
                if (on_air) {
                    emit_tx_event_local(&state, SSO_EVT_TX_COMMAND_SENT, summary, NULL);
                } else {
                    emit_tx_event_local(&state, SSO_EVT_TX_NOT_SENT, summary, outcome);
                }
                // Audit: the result of every queued TX burst, so post-
                // incident review can see each tx-commit and whether it
                // reached the air (on_air=1 means the burst left the radio).
                {
                    char det[512];
                    snprintf(det, sizeof det,
                             "outcome=\"%.80s\" on_air=%d summary=\"%.300s\"",
                             outcome ? outcome : "?", on_air, summary);
                    sso_audit_event("tx-result", det);
                }
                state.tx_request.pending = 0;
            }
        }
#endif

        // --- IPC: serve clients, fan out state, honour SIGUSR1 yield ---
        // Always service the socket (cheap; accepts new viewers) but
        // throttle STATE broadcasts to 2 Hz so viewers don't get
        // hammered when the loop is running at UHD-chunk cadence.
        if (state.ipc) {
            sso_ipc_server_step(state.ipc, 0);
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
            usleep((state.cmd.active || state.tx_compose_active || state.auto_tcmd_active)
                   ? 20000 : UPDATE_INTERVAL_MICROSEC);
        }
    }

    endwin();
    tui_release_stderr();
    if (state.have_tr_switch) {
        tr_switch_disconnect(&state.tr_switch);
        state.have_tr_switch = 0;
    }
    // Join the rotator worker before closing the serial FD — otherwise a
    // mid-read in the worker would see EBADF and corrupt the snapshot.
    if (state.rot_async != NULL) {
        antenna_rotator_async_close(state.rot_async);
        state.rot_async = NULL;
    }
    if (state.have_antenna_rotator) {
        antenna_rotator_disconnect(&state.antenna_rotator);
        state.have_antenna_rotator = 0;
    }
    // Free any plan that survived (mid-pass exit / crash on a key
    // before the LOS branch had a chance to clear it).
    main_pursuit_clear_plan(&state);
    if (state.ipc) {
        sso_ipc_server_close(state.ipc);
        state.ipc = NULL;
    }
    // Politely terminate the live raylib waterfall if we spawned one.
    // 5 s timeout via WNOHANG polling so the operator doesn't wait on
    // a hung viewer at shutdown.
    if (g_live_waterfall_pid > 0) {
        kill(g_live_waterfall_pid, SIGTERM);
        for (int t = 0; t < 50; ++t) {
            int status;
            pid_t r = waitpid(g_live_waterfall_pid, &status, WNOHANG);
            if (r == g_live_waterfall_pid) {
                g_live_waterfall_pid = -1;
                break;
            }
            usleep(100000);
        }
        if (g_live_waterfall_pid > 0) {
            kill(g_live_waterfall_pid, SIGKILL);
            waitpid(g_live_waterfall_pid, NULL, 0);
            g_live_waterfall_pid = -1;
        }
        if (g_live_waterfall_stdin_fd >= 0) {
            close(g_live_waterfall_stdin_fd);
            g_live_waterfall_stdin_fd = -1;
        }
    }
#ifdef SSO_WITH_SDR
    char final_wav_path[512] = "";
    char final_iq_path[512]  = "";
    int  final_iq_rate       = 0;
    if (state.rx_session) {
        // Snapshot both sidecar paths before close so the full-pass
        // renderers can find the closed files on disk. Both paths
        // persist across wav_stop in rx_session.
        rx_session_wav_snapshot(state.rx_session,
                                final_wav_path, sizeof final_wav_path,
                                NULL, NULL, NULL);
        rx_session_iq_snapshot(state.rx_session,
                               final_iq_path, sizeof final_iq_path,
                               NULL, &final_iq_rate);
        // The worker owns the WAV writer and the B210 core. Closing
        // the session signals the worker to stop, joins it, then
        // tears down both. Any open WAV / .iq gets its header patched
        // (WAV) or its trailer flushed (IQ).
        rx_session_request_wav_stop(state.rx_session);
        rx_session_close(state.rx_session);
        state.rx_session = NULL;
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

