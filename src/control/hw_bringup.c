/*

   Simple Satellite Operations  control/hw_bringup.c

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

#include "hw_bringup.h"
#include "state.h"

#include "antenna_rotator.h"
#include "antenna_rotator_async.h"
#include "pursuit.h"
#include "rotator_calibrate.h"
#include "tracking.h"          // main_rotator_refresh_targets_from_snapshot
#include "tr_switch.h"

#include <stdio.h>
#include <stdlib.h>            // EXIT_FAILURE, atoi

#ifdef SSO_WITH_SDR
#include "b210_rx_tx_core.h"
#include "carrier_trim.h"
#include "rx_session.h"
#include "sso_audit.h"
#endif

int hw_rotator_open(state_t *state)
{
    if (!state->rot.run_with_antenna_rotator) {
        return 0;
    }
    state->rot.antenna_rotator.is_required = 1;
    int antenna_rotator_result = antenna_rotator_init(&state->rot.antenna_rotator);
    if (antenna_rotator_result != ANTENNA_ROTATOR_OK) {
        fprintf(stderr, "Error initializing antenna rotator\n");
        // --self-test is a dry run: a missing rotator must not abort the
        // bring-up, so the configuration report still prints. Outside
        // --self-test this stays fatal.
        if (!state->self_test) {
            return EXIT_FAILURE;
        }
        fprintf(stderr,
                "--self-test: continuing without the rotator (dry run)\n");
        return 0;
    }
    state->rot.have_antenna_rotator = 1;
    // Spawn the async worker. From here on, every serial roundtrip happens on
    // the worker thread; the main loop only reads the snapshot via
    // main_rotator_refresh_targets_from_snapshot() and posts SETs via
    // main_rotator_submit_set().
    if (antenna_rotator_async_open(&state->rot.rot_async,
                                    &state->rot.antenna_rotator, 0.5) != 0) {
        fprintf(stderr, "Error spawning antenna rotator worker\n");
        if (!state->self_test) {
            return EXIT_FAILURE;
        }
        fprintf(stderr,
                "--self-test: rotator worker unavailable (dry run)\n");
        state->rot.have_antenna_rotator = 0;
        return 0;
    }
    // Adopt whatever extended position the SPID is already at so the unwrapped
    // accumulator starts grounded in reality. We wait briefly for the worker's
    // first STATUS read; the timeout is bounded so a missing controller
    // doesn't hang startup.
    //
    // The seed snapshot also overwrites target_* with the current physical
    // position — fine when nobody asked for a specific park position, but a
    // problem when the operator passed --rotator-target-azimuth /
    // --rotator-target-elevation: those user-specified targets would be
    // silently clobbered before T ever fired. Snapshot them and restore after
    // seeding.
    double sav_az    = state->rot.antenna_rotator.target_azimuth;
    double sav_el    = state->rot.antenna_rotator.target_elevation;
    double sav_az_uw = state->rot.antenna_rotator.target_azimuth_unwrapped;
    int    sav_uw_ok = state->rot.antenna_rotator.unwrapped_target_valid;
    if (antenna_rotator_async_wait_first_status(state->rot.rot_async, 1500) != 0
        || main_rotator_refresh_targets_from_snapshot(&state->rot) != 0) {
        fprintf(stderr, "Warning: could not read SPID position; "
                        "check that the Rot2ProG is in 'A' mode\n");
    }
    if (state->rot.antenna_rotator.fixed_target) {
        state->rot.antenna_rotator.target_azimuth            = sav_az;
        state->rot.antenna_rotator.target_elevation          = sav_el;
        state->rot.antenna_rotator.target_azimuth_unwrapped  = sav_az_uw;
        state->rot.antenna_rotator.unwrapped_target_valid    = sav_uw_ok;
    }
    return 0;
}

int hw_rotator_calibrate(rot_t *rot)
{
    // --calibrate-rotator: drive the antenna across known arcs to measure
    // deg/s on each axis, save the result to disk, and exit without entering
    // the operator UI. Requires the safety interlock so a stray flag in a
    // script can't move hardware.
    if (!rot->have_antenna_rotator) {
        fprintf(stderr, "--calibrate-rotator: no rotator open "
                        "(was --without-rotator passed?)\n");
        return EXIT_FAILURE;
    }
    if (!rot->confirm_rotator_calibrate) {
        fprintf(stderr,
                "--calibrate-rotator will physically move the antenna.\n"
                "Confirm the mast area is clear, then re-run with\n"
                "  --calibrate-rotator --confirm-rotator-calibrate\n");
        return EXIT_FAILURE;
    }
    double az_dps = 0.0, el_dps = 0.0;
    rotator_calibrate_result_t cres = rotator_calibrate_run(
        rot->rot_async, &az_dps, &el_dps, stderr);
    fprintf(stderr, "calibrate: result = %s\n",
            rotator_calibrate_result_name(cres));
    if (cres == ROTATOR_CALIBRATE_OK) {
        fprintf(stderr,
                "calibrate: saved rates az=%.3f deg/s el=%.3f deg/s\n",
                az_dps, el_dps);
    }
    // Shutdown cleanly — the operator UI never started, but the rotator FD and
    // worker are open.
    if (rot->rot_async != NULL) {
        antenna_rotator_async_close(rot->rot_async);
        rot->rot_async = NULL;
    }
    if (rot->have_antenna_rotator) {
        antenna_rotator_disconnect(&rot->antenna_rotator);
        rot->have_antenna_rotator = 0;
    }
    return (cres == ROTATOR_CALIBRATE_OK) ? 0 : 1;
}

void hw_pursuit_rates_load(rot_t *rot)
{
    // Normal startup: load saved rotator rates from the calibration file.
    // Missing or malformed file -> pursuit planner stays disabled.
    if (!rot->have_antenna_rotator) {
        return;
    }
    if (pursuit_load_rotator_rates(&rot->pursuit_az_dps,
                                    &rot->pursuit_el_dps) == 0) {
        fprintf(stderr,
                "pursuit: loaded slew rates az=%.3f deg/s el=%.3f deg/s\n",
                rot->pursuit_az_dps, rot->pursuit_el_dps);
    } else {
        fprintf(stderr,
                "pursuit: no calibration on disk; run "
                "`simple_sat_ops --calibrate-rotator "
                "--confirm-rotator-calibrate` to enable lead-aim\n");
    }
}

void hw_tr_switch_open(trsw_t *trsw)
{
    // T/R antenna switch — auto-probe before ncurses takes the screen, so a
    // "not connected" warning lands on the terminal. Absent or inaccessible
    // hardware is non-fatal; the UI panel reads "not connected" and the
    // program runs normally.
    if (!trsw->run_with_tr_switch) {
        return;
    }
    if (tr_switch_init(&trsw->tr_switch) == 0) {
        trsw->have_tr_switch = 1;
    } else {
        fprintf(stderr,
                "T/R switch: could not open %s (skipping; "
                "pass --without-tr-switch to silence)\n",
                trsw->tr_switch.device_filename
                    ? trsw->tr_switch.device_filename : "?");
    }
}

void hw_sdr_open(state_t *state)
{
#ifdef SSO_WITH_SDR
    // Open the B210 once, here, before ncurses init — soft-fail on any UHD
    // error so a dev host without a device can still run the UI. rx_session
    // takes ownership of the core; we drop our local handle afterwards so main
    // never touches UHD off-thread.
    if (state->control_mode && !state->sdr.without_b210) {
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
            .freq_hz         = state->track.nominal_downlink_frequency_hz
                             + state->sdr.rx_lo_offset_hz,
            .rate_hz         = 480000.0,
            .gain_db         = state->sdr.rx_gain_db,
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
            .rx_dc_offset_track  = state->sdr.rx_dc_offset_track,
            .rx_iq_balance_track = state->sdr.rx_iq_balance_track,
            .fm_lo_compensation_hz = state->sdr.rx_lo_offset_hz,
            .carrier_trim_hz       = carrier_trim_load_hz(),
            // SDR backend selection: type (default auto), and the UHD
            // clone overrides. --sdr-device routes to the UHD device
            // args when given (e.g. "serial=..."); --uhd-args takes
            // precedence and is passed verbatim.
            .backend_type        = state->sdr.sdr_type,
            .device_args         = state->sdr.sdr_device[0] ? state->sdr.sdr_device : "type=b200",
            .uhd_args_override   = state->sdr.uhd_args[0] ? state->sdr.uhd_args : NULL,
            .fpga_image_path     = state->sdr.sdr_fpga[0] ? state->sdr.sdr_fpga : NULL,
            // RTL-SDR dongle index (UHD ignores it; for UHD use --uhd-args).
            .device_index        = state->sdr.sdr_device[0] ? atoi(state->sdr.sdr_device) : 0,
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
                    state->sdr.rx_lo_offset_hz);
                sso_audit_event("b210-open", det);
            }
            rx_session_params_t rxp = {
                .bit_rate          = 9600,
                .window_s          = 1.5,
                .slide_s           = 0.5,
                .sync_max_ham      = 4,
                .use_rs            = 1,
                .force_beacon      = 0,
                .show_packet_headers = 0,
                .pass_folder       = state->op.pass_folder[0] ? state->op.pass_folder : NULL,
                .want_wav          = 1,
                .tle_path          = state->track.prediction.tles_filename,
                .sat_name          = state->track.prediction.satellite_ephem.tle.sat_name[0]
                                     ? state->track.prediction.satellite_ephem.tle.sat_name
                                     : NULL,
                .session_dir       = state->op.pass_folder[0] ? state->op.pass_folder : NULL,
                .lo_offset_hz      = state->sdr.rx_lo_offset_hz,
            };
            if (rx_session_open(&state->sdr.rx_session, &rxp, core) != 0) {
                fprintf(stderr,
                    "simple_sat_ops: rx_session_open failed — closing B210\n");
                b210_rx_tx_core_close(core);
            }
            // rx_session_open succeeded → it owns `core` now.
            core = NULL;
            // --always-record: start WAV + .iq + sidecars right now,
            // before any pass logic gets a chance to gate them. The
            // per-pass start/stop block in the tracking loop checks
            // state->sdr.always_record and skips itself when this is on.
            // Suppressed under --self-test: the dry run opens the SDR to
            // prove it comes up, but must never write capture files.
            if (state->sdr.always_record && state->sdr.rx_session && !state->self_test) {
                rx_session_request_wav_start(state->sdr.rx_session);
                fprintf(stderr,
                    "simple_sat_ops: --always-record on — WAV/IQ "
                    "capture started, pass gating disabled\n");
                sso_audit_event("rec-start", "trigger=always-record");
            }
        }
    }
#else
    (void) state;
#endif
}
