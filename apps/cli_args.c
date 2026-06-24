/*

   Simple Satellite Operations  cli_args.c

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

#include "cli_args.h"
#include "state.h"

#include "antenna_rotator.h"
#include "antenna_rotator_async.h"
#include "argparse.h"
#include "frontiersat.h"
#include "hmac_keyfile.h"
#include "pass_session.h"
#include "prediction.h"
#include "sdr_backend.h"
#include "sso_audit.h"
#include "sso_ipc.h"
#include "sso_ipc_paths.h"
#include "sso_operator.h"
#include "sso_paths.h"
#include "sso_version.h"
#include "tcmd_lint.h"
#include "tle_csv.h"
#include "tr_switch.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#ifdef SSO_WITH_SDR
#include "b210_rx_tx_core.h"
#include "carrier_trim.h"
#include "rx_session.h"
#include "tx_burst.h"
#endif

// Default uplink/downlink carriers (MHz) when --uplink-freq / --downlink-freq
// aren't given. FrontierSat is simplex on 436.150.
#define UPLINK_FREQ_MHZ   436.150000
#define DOWNLINK_FREQ_MHZ 436.150000

// --- --self-test report -------------------------------------------
//
// Prints the resolved configuration in a stable key: value layout so
// test harnesses can grep it. Called at the very end of the bring-up
// (after the TLE load, pass-folder setup, rotator + SDR open, IPC bind
// and --tc-file lint), so the hardware lines report the live opened
// state, not just the requested intent. Every line is "key: value"
// with no surrounding quoting; values are short enough to fit on one
// line. The "self-test:" header line is the contract — downstream
// scripts can use it as a sentinel.

static const char *hmac_status_str(hmac_display_status_t s)
{
    switch (s) {
        case HMAC_DISPLAY_OK:      return "ok";
        case HMAC_DISPLAY_MISSING: return "missing";
        case HMAC_DISPLAY_BAD:     return "bad";
        case HMAC_DISPLAY_UNSET:   /* fall through */
        default:                   return "unset";
    }
}

static const char *baud_str(int speed_const)
{
    // antenna_rotator stores the serial speed as the POSIX termios
    // constant (B600 etc), not the integer baud rate. Map the ones
    // the rotator actually uses; "?" everything else so a change to
    // antenna_rotator.c shows up in the report instead of crashing
    // it.
    switch (speed_const) {
        case B600:    return "600";
        case B1200:   return "1200";
        case B2400:   return "2400";
        case B4800:   return "4800";
        case B9600:   return "9600";
        case B19200:  return "19200";
        case B38400:  return "38400";
        case B57600:  return "57600";
        case B115200: return "115200";
        default:      return "?";
    }
}

// Parse the numeric tail of a "--opt=<value>" token (pass arg + prefix_len).
// Unlike atoi/atof these reject an empty value and any trailing non-numeric
// characters, so a fat-fingered "--rx-gain=3o" or "--verbose=" is an error
// rather than a silent 3 / 0. Trailing ASCII blanks are tolerated. Returns
// 0 on success, -1 on a malformed or out-of-range value.
static int parse_arg_double(const char *s, double *out)
{
    if (s == NULL || *s == '\0') return -1;
    errno = 0;
    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s) return -1;
    while (*end == ' ' || *end == '\t') ++end;
    if (*end != '\0') return -1;
    if (errno == ERANGE) return -1;
    *out = v;
    return 0;
}

static int parse_arg_long(const char *s, long *out)
{
    if (s == NULL || *s == '\0') return -1;
    errno = 0;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s) return -1;
    while (*end == ' ' || *end == '\t') ++end;
    if (*end != '\0') return -1;
    if (errno == ERANGE) return -1;
    *out = v;
    return 0;
}

static const char *sdr_type_str(sdr_backend_type_t t)
{
    switch (t) {
        case SDR_TYPE_UHD:    return "uhd (B2xx)";
        case SDR_TYPE_RTLSDR: return "rtl-sdr";
        case SDR_TYPE_AUTO:   /* fall through */
        default:              return "auto (probe UHD, then RTL-SDR)";
    }
}

void self_test_report(const state_t *state, FILE *out, int argc, char **argv)
{
    fprintf(out, "self-test: simple_sat_ops configuration snapshot\n");
    fprintf(out, "version: %s\n", sso_version_string());

    // Echo the command line so the report is self-describing — the
    // reader can see at a glance which flags produced this snapshot.
    fprintf(out, "argv:");
    for (int i = 1; i < argc; ++i) {
        fprintf(out, " %s", argv[i]);
    }
    fprintf(out, "\n");

    // Mode. apply_args has already set state->control_mode / state->viewer_mode
    // (the latter only via the auto-probe path, which --self-test
    // skips). Standalone is the default.
    const char *mode = state->control_mode  ? "operator (--control)"
                     : state->viewer_stream ? "viewer-stream (headless JSON)"
                     : state->viewer_mode   ? "viewer (auto-detected)"
                                       : "standalone";
    fprintf(out, "mode: %s\n", mode);

#ifdef SSO_WITH_SDR
    int sdr_compiled = 1;
#else
    int sdr_compiled = 0;
#endif
#ifdef WITH_USRP_B210
    int uhd_compiled = 1;
#else
    int uhd_compiled = 0;
#endif
#ifdef WITH_RTL_SDR
    int rtl_compiled = 1;
#else
    int rtl_compiled = 0;
#endif
    fprintf(out, "build: sdr=%s (uhd=%s, rtl-sdr=%s)\n",
            sdr_compiled ? "on" : "off",
            uhd_compiled ? "on" : "off",
            rtl_compiled ? "on" : "off");

    // TLE — loaded by the time this prints, so report the satellite the
    // bring-up actually selected plus the file it came from.
    fprintf(out, "tle: %s (file=%s)\n",
            state->prediction.satellite_ephem.tle.sat_name[0]
                ? state->prediction.satellite_ephem.tle.sat_name
                : "(no satellite loaded)",
            state->prediction.tles_filename
                ? state->prediction.tles_filename
                : "(none)");

    // HMAC --- the operator's banner-and-sign state. CTS1 firmware
    // expects every uplink to be HMAC-signed; the dispatcher refuses
    // to key the PA if state->hmac_key_len == 0, so this line is the
    // single most-important pre-flight check.
    fprintf(out,
            "hmac: %s (path=%s, status=%s, bytes=%zu)\n",
            state->hmac_key_len > 0 ? "enabled (default)" : "DISABLED",
            state->hmac_keyfile_path[0] ? state->hmac_keyfile_path : "(unresolved)",
            hmac_status_str(state->hmac_display_status),
            state->hmac_key_len);

    // Doppler --- both the display correction and the TX-side burst
    // staging key off state->doppler_correction_enabled. On by
    // default; --no-doppler-correction clears it. Report RX and TX
    // separately so the operator can see where the correction is
    // applied: RX is software (sw_nco on post-decim IQ, no hardware
    // LO retune mid-pass — the threshold-driven retune was removed
    // because it caused phase resets in the coherent demod), TX is
    // hardware (b210_rx_tx_core_burst tunes the B210 LO to the
    // Doppler-corrected frequency for every burst).
    fprintf(out, "doppler-correction: %s\n",
            state->doppler_correction_enabled ? "enabled (default)"
                                              : "DISABLED (--no-doppler-correction)");
    fprintf(out, "doppler-rx: %s (software sw_nco on post-decim IQ; hardware LO fixed)\n",
            state->doppler_correction_enabled ? "enabled" : "disabled");
    fprintf(out, "doppler-tx: %s (hardware SDR LO retune per burst, f=carrier/(1-rr/c))\n",
            (!sdr_compiled || state->sdr.without_b210)
                ? "n/a (no SDR)"
                : (state->doppler_correction_enabled ? "enabled" : "disabled"));
    fprintf(out, "uplink-nominal-mhz: %.6f\n",
            state->nominal_uplink_frequency_hz / 1e6);
    fprintf(out, "downlink-nominal-mhz: %.6f\n",
            state->nominal_downlink_frequency_hz / 1e6);
    fprintf(out, "rx-lo-offset-khz: %+.3f\n", state->sdr.rx_lo_offset_hz / 1000.0);

    // TX safety / staging gates the operator might have set.
    fprintf(out, "tx-no-tx: %s\n", state->no_tx ? "on (--no-tx)" : "off");
    fprintf(out, "tx-dry-run: %s\n", state->tx_dry_run ? "on (--tx-dry-run)" : "off");
    fprintf(out, "tx-auto-tcmd-file: %s\n",
            state->auto_tcmd_file_path[0] ? state->auto_tcmd_file_path : "(none)");

    // Hardware — reported live: the rotator and SDR have already been
    // opened by the time --self-test prints this, so these lines show the
    // real opened state (device, current position, slew rates, SDR
    // backend, TX capability), not just the requested intent.
    if (state->rot.have_antenna_rotator) {
        double az = 0.0, el = 0.0;
        int ok = 0, stale_ms = 0, inflight = 0;
        if (state->rot.rot_async) {
            antenna_rotator_async_snapshot(state->rot.rot_async, &az, &el,
                                           &ok, &stale_ms, &inflight);
        }
        fprintf(out,
                "rotator: open (device=%s, baud=%s, az=%.1f el=%.1f, "
                "slew az=%.3f el=%.3f deg/s)\n",
                state->rot.antenna_rotator.device_filename,
                baud_str(state->rot.antenna_rotator.serial_speed),
                az, el, state->rot.pursuit_az_dps, state->rot.pursuit_el_dps);
    } else {
        fprintf(out, "rotator: %s (device=%s, baud=%s)\n",
                state->rot.run_with_antenna_rotator
                    ? "enabled but not opened (no controller?)"
                    : "disabled (--without-rotator)",
                state->rot.antenna_rotator.device_filename,
                baud_str(state->rot.antenna_rotator.serial_speed));
    }

    fprintf(out, "sdr-type: %s\n", sdr_type_str(state->sdr.sdr_type));
#ifdef SSO_WITH_SDR
    if (state->sdr.rx_session) {
        double actual_freq_hz = 0.0;
        rx_session_snapshot(state->sdr.rx_session, NULL, NULL, NULL,
                            &actual_freq_hz, NULL, 0);
        fprintf(out, "sdr: open (%s, tx=%s, rx-freq-mhz=%.6f)\n",
                rx_session_sdr_name(state->sdr.rx_session),
                rx_session_can_tx(state->sdr.rx_session) ? "yes" : "no (RX-only)",
                actual_freq_hz / 1e6);
    } else if (state->sdr.without_b210) {
        fprintf(out, "sdr: disabled (--without-b210)\n");
    } else if (!state->control_mode) {
        fprintf(out, "sdr: not opened (standalone; SDR opens only with --control)\n");
    } else {
        fprintf(out, "sdr: not opened (open failed — see stderr)\n");
    }
#else
    fprintf(out, "sdr: disabled (built without SDR support)\n");
#endif

    fprintf(out, "live-waterfall: %s\n",
            state->run_live_waterfall ? "on (--live-waterfall)" : "off");

    fprintf(out, "pass-folder-seed: %s\n",
            state->pass_folder[0] ? state->pass_folder : "(auto)");

    // Observer location. apply_args stored these in radians on the
    // ephem struct — convert back to degrees for the report.
    fprintf(out, "observer-lat-deg: %.6f\n",
            state->prediction.observer_ephem.position_geodetic.lat * 180.0 / M_PI);
    fprintf(out, "observer-lon-deg: %.6f\n",
            state->prediction.observer_ephem.position_geodetic.lon * 180.0 / M_PI);
    fprintf(out, "observer-alt-m: %.1f\n",
            state->prediction.observer_ephem.position_geodetic.alt * 1000.0);

    fprintf(out, "self-test: ok\n");
    fflush(out);
}

// --- startup wiring -----------------------------------------------

void cli_load_hmac_keyfile(state_t *state)
{
    // Resolve + load the HMAC keyfile. The bytes feed every TX burst's
    // AX100 frame (CTS1 firmware expects HMAC on every uplink), AND
    // light the operator banner — "(N bytes ok)" means TX is armed,
    // "(MISSING)" / "(BAD)" means the next TX request will be refused
    // before keying the PA. If --hmac-keyfile= wasn't given, fall back
    // to hmac_keyfile_default_path (shared first, per-user second).
    if (state->hmac_keyfile_path[0] == '\0') {
        if (hmac_keyfile_default_path(state->hmac_keyfile_path,
                                      sizeof state->hmac_keyfile_path) != 0) {
            state->hmac_keyfile_path[0] = '\0';
            state->hmac_display_status  = HMAC_DISPLAY_MISSING;
        }
    }
    if (state->hmac_keyfile_path[0] != '\0') {
        struct stat st;
        if (stat(state->hmac_keyfile_path, &st) != 0) {
            state->hmac_display_status = HMAC_DISPLAY_MISSING;
        } else {
            ssize_t got = hmac_keyfile_load(state->hmac_keyfile_path,
                                            state->hmac_key,
                                            sizeof state->hmac_key);
            if (got > 0) {
                state->hmac_display_status = HMAC_DISPLAY_OK;
                state->hmac_key_len        = (size_t) got;
            } else {
                state->hmac_display_status = HMAC_DISPLAY_BAD;
                state->hmac_key_len        = 0;
                memset(state->hmac_key, 0, sizeof state->hmac_key);
            }
        }
    }
}

int cli_tcmd_lint_gate(const state_t *state)
{
    // Telecommand-agenda lint gate. When a --tc-file was given, lint it
    // against the firmware's telecommand set (names, argument counts,
    // CTS1+...! framing, length limits) BEFORE bringing up anything that
    // can key the PA. Lint errors mean a command would be rejected (or
    // worse, mis-parsed) by the satellite, so refuse to start unless the
    // operator explicitly accepts the risk. Warnings (e.g. a command not
    // meant for routine flight operation) are printed but do not block.
    if (state->auto_tcmd_file_path[0] == '\0') {
        return 0;
    }
    int tc_warns = 0;
    int tc_errs = tcmd_lint_file(state->auto_tcmd_file_path, stderr, &tc_warns);
    if (tc_errs > 0 && !state->ignore_tc_errors) {
        fprintf(stderr,
            "simple_sat_ops: %d error%s detected in the --tc-file content (%s).\n"
            "Refusing to start. Fix the agenda, or re-run with\n"
            "--ignore-at-your-peril-all-tc-errors to bypass this check.\n",
            tc_errs, tc_errs == 1 ? "" : "s", state->auto_tcmd_file_path);
        return EXIT_FAILURE;
    }
    if (tc_errs > 0) {
        fprintf(stderr,
            "simple_sat_ops: %d telecommand error%s in %s -- proceeding anyway "
            "(--ignore-at-your-peril-all-tc-errors).\n",
            tc_errs, tc_errs == 1 ? "" : "s", state->auto_tcmd_file_path);
    } else if (tc_warns > 0) {
        fprintf(stderr,
            "simple_sat_ops: %d telecommand warning%s in %s (see above); proceeding.\n",
            tc_warns, tc_warns == 1 ? "" : "s", state->auto_tcmd_file_path);
    }
    return 0;
}

// --- apply_args ---------------------------------------------------

// One self-contained block per option, each testing "... || help" so
// that in help mode (help != HELP_OFF) every block prints its one-line
// help and falls through to the next. In parse mode only the matching
// block runs its body and writes its result straight into *state (there
// is no separate config struct; apply_args has always filled state).
// See src/cli/argparse.h for the convention.
//
// Option column width: widest label below is
// "--ignore-at-your-peril-all-tc-errors" (36) + a small margin.
#define OPTW 38

int apply_args(state_t *state, int argc, char **argv, double jul_utc, int help)
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
    char *positional = NULL;
    // Set when a second bare (non-option) token appears. The authoritative
    // "too many positionals" signal -- the old argc - n_options heuristic
    // miscounts the moment any option's n_options bookkeeping is off by one.
    int extra_positional = 0;

    if (!help) {
        // Non-zero state defaults all live here, in one place, so a caller
        // that runs apply_args can't get a half-initialised struct (these
        // used to be split between here and main()). Set before the parse
        // loop below, which overrides whatever the operator passed.
        state->prediction.predicted_max_elevation = -180.0;
        // Seed the TX-compose "remembered" draft: the CTS1 prefix and 80 dB.
        snprintf(state->tx_last_payload, sizeof state->tx_last_payload, "CTS1+");
        snprintf(state->tx_last_power,   sizeof state->tx_last_power,   "80.0");
        // The Doppler carrier falls back to the bare nominal until SGP4 has a
        // range rate; preroll matches the tx_burst_run fallback. Both may be
        // overridden below (--tx-preroll-ms) or by the per-tick Doppler refresh.
        state->tx_freq_hz_doppler = (long) FRONTIERSAT_CARRIER_HZ;
        state->tx_preroll_ms      = 200;

        state->rot.antenna_rotator.tracking_prep_time_minutes = TRACKING_PREP_TIME_MINUTES;
        state->satellite_tracking = 0;

        state->nominal_uplink_frequency_hz = UPLINK_FREQ_MHZ * 1e6;
        state->nominal_downlink_frequency_hz = DOWNLINK_FREQ_MHZ * 1e6;
        state->doppler_uplink_frequency_hz = state->nominal_uplink_frequency_hz;
        state->doppler_downlink_frequency_hz = state->nominal_downlink_frequency_hz;
        state->doppler_correction_enabled = 1;
        // SIGNED LO offset from the nominal carrier. Positive → LO ABOVE
        // nominal (signal lands at negative baseband). Negative → LO
        // BELOW nominal (signal at positive baseband). Default -25 kHz
        // puts the corrected signal at +25 kHz baseband, away from the
        // B210's DC null. Operator can shift it to dodge fixed-pattern
        // spurs — comfortable range is roughly ±5..±35 kHz: at least
        // 5 kHz to clear DC, and at most ~35 kHz so the ±10 kHz Doppler
        // swing stays inside the 48 kHz post-decim half-band.
        state->sdr.rx_lo_offset_hz = -25000.0;
        state->sdr.rx_gain_db      = 30.0;
        // AD9361 background tracking. The visible ~51 Hz comb of impulsive
        // spikes at mid-range gain is from the IQ-balance loop (discrete
        // phase-rotation steps applied to the captured IQ); the DC-offset
        // loop is a slow continuous IIR notch that DOESN'T produce
        // spikes but DOES suppress the AD9361's static ADC DC bias.
        // Turn IQ tracking off by default (kills the spikes), leave DC
        // tracking on (otherwise the static DC bias rotates into a strong
        // +lo_offset_hz sinusoid via fm_lo_nco on the decode path, which
        // dominates the IQ time series).
        state->sdr.rx_dc_offset_track  = 1;
        state->sdr.rx_iq_balance_track = 0;

        state->rot.run_with_antenna_rotator = 1;
        state->rot.antenna_rotator.device_filename = "/dev/ttyUSB0";
        state->rot.antenna_rotator.serial_speed = B600;
        state->rot.antenna_rotator.fixed_target = 0;

        // T/R antenna switch: auto-probe /dev/ttyACM0. Failure is a
        // one-line warning, not an error.
        state->trsw.run_with_tr_switch = 1;
        state->trsw.have_tr_switch     = 0;
        state->trsw.tr_switch.device_filename = "/dev/ttyACM0";
        state->trsw.tr_switch.serial_speed    = B115200;
    }

    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // Positional first so <satellite_id> lists above the options.
        // A token that is not "--"-prefixed is the positional. Capture it
        // here, inside the loop, where t has already advanced past every
        // space-form option value (each value-taking option does ++t), so
        // a bare token at this point is genuinely the <satellite_id> and
        // not, say, the path after "--tle". The first one wins; a second
        // bare token falls through to the post-loop n_positional > 1 check.
        if (strncmp("--", arg, 2) != 0 || help) {
            if (help) parse_help_line(OPTW, "<satellite_id>",
                "name prefix in the TLE, or `next` to auto-pick the next pass");
            else if (positional == NULL) positional = (char *) arg;
            else extra_positional = 1;   // a second bare token -> too many
            matched = 1;
        }
        if (strcmp("--help", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "short help (this message)");
            else { apply_args(state, argc, argv, jul_utc, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp("--help-full", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--help-full", "detailed help with keyboard layout");
            else { apply_args(state, argc, argv, jul_utc, HELP_FULL); return PARSE_HELP; }
            matched = 1;
        }
        if (strncmp("--verbose=", arg, 10) == 0 || help) {
            if (help) parse_help_line(OPTW, "--verbose=<level>", "verbosity integer");
            else {
                state->n_options++;
                long v;
                if (parse_arg_long(arg + 10, &v) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->verbose_level = (int) v;
            }
            matched = 1;
        }
        if (strcmp("--with-rotator", arg) == 0
                || strcmp("--with-hardware", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--with-rotator",
                "drive the rotator (default; --with-hardware synonym, no-op)");
            else {
                // Rotator is on by default now. These flags survive as
                // silent no-ops so existing scripts and muscle memory
                // keep working.
                state->n_options++;
                state->rot.run_with_antenna_rotator = 1;
            }
            matched = 1;
        }
        if (strcmp("--without-rotator", arg) == 0
                || strcmp("--without-hardware", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-rotator",
                "skip the SPID Rot2Prog (--without-hardware synonym)");
            else {
                state->n_options++;
                state->rot.run_with_antenna_rotator = 0;
            }
            matched = 1;
        }
        if (strcmp("--calibrate-rotator", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--calibrate-rotator",
                "measure rotator slew rates then exit (needs --confirm-rotator-calibrate)");
            else { state->n_options++; state->rot.calibrate_rotator = 1; }
            matched = 1;
        }
        if (strcmp("--confirm-rotator-calibrate", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--confirm-rotator-calibrate",
                "safety interlock for --calibrate-rotator (antenna moves)");
            else { state->n_options++; state->rot.confirm_rotator_calibrate = 1; }
            matched = 1;
        }
        if (strcmp("--without-rotator-pursuit", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-rotator-pursuit",
                "disable the pursuit / lead-aim planner even if calibrated");
            else { state->n_options++; state->rot.without_rotator_pursuit = 1; }
            matched = 1;
        }
        if (strcmp("--without-tr-switch", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-tr-switch",
                "skip the T/R switch probe entirely");
            else { state->n_options++; state->trsw.run_with_tr_switch = 0; }
            matched = 1;
        }
        if (strncmp("--tr-switch-device=", arg, 19) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tr-switch-device=<path>",
                "UHF T/R antenna switch tty (default /dev/ttyACM0)");
            else {
                state->n_options++;
                if (strlen(arg) < 20) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->trsw.tr_switch.device_filename = arg + 19;
            }
            matched = 1;
        }
        if (strcmp("--without-b210", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--without-b210",
                "skip the USRP B210 (UI + rotator only)");
            else { state->n_options++; state->sdr.without_b210 = 1; }
            matched = 1;
        }
        if (strcmp("--no-audio", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-audio",
                "refuse viewer live-audio (--viewer-stream) requests");
            else { state->n_options++; state->sdr.no_audio = 1; }
            matched = 1;
        }
#ifdef SSO_WITH_SDR
        if (strncmp("--sdr-type=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-type=uhd|rtlsdr|auto",
                "SDR backend (default auto; RTL-SDR is RX-only)");
            else {
                state->n_options++;
                if (sdr_backend_type_from_string(arg + 11, &state->sdr.sdr_type) != 0) {
                    fprintf(stderr, "--sdr-type: unknown '%s' "
                            "(want uhd | rtlsdr | auto)\n", arg + 11);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--sdr-device=", arg, 13) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-device=<sel>",
                "backend device selector (RTL-SDR index; UHD use --uhd-args)");
            else {
                state->n_options++;
                snprintf(state->sdr.sdr_device, sizeof state->sdr.sdr_device, "%s", arg + 13);
            }
            matched = 1;
        }
        if (strncmp("--uhd-args=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--uhd-args=<args>",
                "UHD device-args verbatim; overrides detection");
            else {
                state->n_options++;
                snprintf(state->sdr.uhd_args, sizeof state->sdr.uhd_args, "%s", arg + 11);
            }
            matched = 1;
        }
        if (strncmp("--sdr-fpga=", arg, 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--sdr-fpga=<path>",
                "force a UHD FPGA image (B2xx clone with non-stock bitstream)");
            else {
                state->n_options++;
                snprintf(state->sdr.sdr_fpga, sizeof state->sdr.sdr_fpga, "%s", arg + 11);
            }
            matched = 1;
        }
#endif
        if (strcmp("--no-tx", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-tx",
                "open the B210 for RX but block the TX compose modal from keying the PA");
            else { state->n_options++; state->no_tx = 1; }
            matched = 1;
        }
        if (strcmp("--live-waterfall", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--live-waterfall",
                "auto-launch the raylib live_waterfall viewer when recording starts");
            else { state->n_options++; state->run_live_waterfall = 1; }
            matched = 1;
        }
        if (strcmp("--always-record", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--always-record",
                "record from B210 open until shutdown (skip per-pass start/stop)");
            else { state->n_options++; state->sdr.always_record = 1; }
            matched = 1;
        }
        if (strcmp("--testing", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--testing",
                "bench mode: pass folder under Testing/ at current local time, no TLE");
            else { state->n_options++; state->testing_mode = 1; }
            matched = 1;
        }
        if (strcmp("--scan-sky", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--scan-sky",
                "rebind T to walk the rotator through a sky grid, dwelling 5 s each");
            else { state->n_options++; state->scan.mode = 1; }
            matched = 1;
        }
        if (strncmp("--scan-step=", arg, 12) == 0 || help) {
            if (help) parse_help_line(OPTW, "--scan-step=<deg>",
                "elevation ring spacing for --scan-sky (default 15, clamped [1,45])");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 12, &state->scan.step_deg) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                if (state->scan.step_deg < 1.0)  state->scan.step_deg = 1.0;
                if (state->scan.step_deg > 45.0) state->scan.step_deg = 45.0;
            }
            matched = 1;
        }
        if (strcmp("--tx-dry-run", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tx-dry-run",
                "record every TX burst as not-sent instead of routing it through the SDR");
            else { state->n_options++; state->tx_dry_run = 1; }
            matched = 1;
        }
        if (strncmp("--tx-preroll-ms=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tx-preroll-ms=<n>",
                "modulated 0xAA carrier before each TX burst (default 200, [0,5000])");
            else {
                state->n_options++;
                long v;
                if (parse_arg_long(arg + 16, &v) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                if (v < 0)    v = 0;
                if (v > 5000) v = 5000;
                state->tx_preroll_ms = (int) v;
            }
            matched = 1;
        }
        // Filename args use the space form (--foo <path>) so bash
        // tab-completion works. The old --foo=<path> form is rejected
        // with a one-line hint pointing at the new spelling.
        if (strcmp("--tc-file", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tc-file <path>",
                "load ASCII telecommands (one per line; 'A' / `:auto` in the UI to send)");
            else {
                // arg is argv[t + 1]; its value is the next token,
                // argv[t + 2]. Consume it and step t past it.
                if (t + 2 >= argc) {
                    fprintf(stderr, "--tc-file: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                snprintf(state->auto_tcmd_file_path, sizeof state->auto_tcmd_file_path,
                         "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--tc-file=", arg, 10) == 0) {
            fprintf(stderr,
                "--tc-file=<path> is no longer accepted; "
                "use `--tc-file <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--ignore-at-your-peril-all-tc-errors", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ignore-at-your-peril-all-tc-errors",
                "start even if the --tc-file agenda has telecommand lint errors");
            else { state->n_options++; state->ignore_tc_errors = 1; }
            matched = 1;
        }
        if (strcmp("--hmac-keyfile", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--hmac-keyfile <path>",
                "HMAC key file shown on the operator banner (default shared, then user)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--hmac-keyfile: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                snprintf(state->hmac_keyfile_path, sizeof state->hmac_keyfile_path,
                         "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--hmac-keyfile=", arg, 15) == 0) {
            fprintf(stderr,
                "--hmac-keyfile=<path> is no longer accepted; "
                "use `--hmac-keyfile <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--tle", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tle <path>",
                "path to a TLE file (default: newest *.tle under the FrontierSat TLEs dir)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--tle: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                state->prediction.tles_filename = tle_path_resolve(argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--tle=", arg, 6) == 0) {
            fprintf(stderr,
                "--tle=<path> is no longer accepted; "
                "use `--tle <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--pass-folder", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--pass-folder <path>",
                "pre-seed the pass folder (handoff: inherit a previous operator's folder)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--pass-folder: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                // Pre-seed state->pass_folder; setup_pass_folder() then skips
                // its AOS-driven auto-discovery and uses the inherited
                // path (handoff case: new operator picks up the previous
                // operator's pass folder).
                snprintf(state->pass_folder, sizeof state->pass_folder, "%s", argv[t + 2]);
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--pass-folder=", arg, 14) == 0) {
            fprintf(stderr,
                "--pass-folder=<path> is no longer accepted; "
                "use `--pass-folder <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strcmp("--rotator-device", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-device <path>",
                "SPID Rot2Prog tty (default /dev/ttyUSB0)");
            else {
                if (t + 2 >= argc) {
                    fprintf(stderr, "--rotator-device: missing <path>\n");
                    return PARSE_ERROR;
                }
                state->n_options += 2;
                state->rot.antenna_rotator.device_filename = argv[t + 2];
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--rotator-device=", arg, 17) == 0) {
            fprintf(stderr,
                "--rotator-device=<path> is no longer accepted; "
                "use `--rotator-device <path>` (TAB-completes the filename)\n");
            return PARSE_ERROR;
        }
        if (strncmp("--uplink-freq-mhz=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--uplink-freq-mhz=<mhz>",
                "uplink nominal carrier, MHz (informational)");
            else {
                state->n_options++;
                double mhz;
                if (parse_arg_double(arg + 18, &mhz) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->nominal_uplink_frequency_hz = mhz * 1e6;
            }
            matched = 1;
        }
        if (strncmp("--downlink-freq-mhz=", arg, 20) == 0 || help) {
            if (help) parse_help_line(OPTW, "--downlink-freq-mhz=<mhz>",
                "downlink / simplex carrier nominal, MHz (informational)");
            else {
                state->n_options++;
                double mhz;
                if (parse_arg_double(arg + 20, &mhz) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->nominal_downlink_frequency_hz = mhz * 1e6;
            }
            matched = 1;
        }
        if (strcmp("--no-doppler-correction", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-doppler-correction",
                "display nominal freqs without Doppler");
            else { state->n_options++; state->doppler_correction_enabled = 0; }
            matched = 1;
        }
        if (strncmp("--lo-offset=", arg, 12) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lo-offset=<kHz>",
                "park the SDR LO this far off the nominal carrier (signed, default -25)");
            else {
                state->n_options++;
                // Argument is kHz so an integer is easy to type; we store Hz.
                double khz;
                if (parse_arg_double(arg + 12, &khz) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                state->sdr.rx_lo_offset_hz = khz * 1000.0;
            }
            matched = 1;
        }
        if (strncmp("--rx-gain=", arg, 10) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rx-gain=<dB>",
                "AD9361 RX gain at session open, dB (default 30, range [0,76])");
            else {
                state->n_options++;
                double g;
                if (parse_arg_double(arg + 10, &g) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                // AD9361 RX gain range is 0-76 dB; UHD coerces values outside
                // this and prints a warning, but we clip here so the value in
                // state matches what the hardware will use.
                if (g < 0.0)       g = 0.0;
                else if (g > 76.0) g = 76.0;
                state->sdr.rx_gain_db = g;
            }
            matched = 1;
        }
        if (strncmp("--ad9361-dc-track=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ad9361-dc-track=on|off",
                "AD9361 background DC-offset tracking (default on)");
            else {
                // on|off|true|false|1|0
                const char *v = arg + 18;
                state->n_options++;
                state->sdr.rx_dc_offset_track =
                    (strcmp(v, "on")   == 0
                     || strcmp(v, "true") == 0
                     || strcmp(v, "1")  == 0) ? 1 : 0;
            }
            matched = 1;
        }
        if (strncmp("--ad9361-iq-track=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ad9361-iq-track=on|off",
                "AD9361 background IQ-balance tracking (default off; ~51 Hz spike comb)");
            else {
                const char *v = arg + 18;
                state->n_options++;
                state->sdr.rx_iq_balance_track =
                    (strcmp(v, "on")   == 0
                     || strcmp(v, "true") == 0
                     || strcmp(v, "1")  == 0) ? 1 : 0;
            }
            matched = 1;
        }
        if (strncmp("--rotator-target-elevation=", arg, 27) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-target-elevation=<deg>",
                "park on a fixed elevation");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 27,
                        &state->rot.antenna_rotator.target_elevation) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                if (state->rot.antenna_rotator.target_elevation < 0.0) {
                    state->rot.antenna_rotator.target_elevation = 0.0;
                } else if (state->rot.antenna_rotator.target_elevation
                           > ANTENNA_ROTATOR_MAXIMUM_ELEVATION) {
                    state->rot.antenna_rotator.target_elevation =
                        ANTENNA_ROTATOR_MAXIMUM_ELEVATION;
                }
                state->rot.antenna_rotator.fixed_target = 1;
            }
            matched = 1;
        }
        if (strncmp("--rotator-target-azimuth=", arg, 25) == 0 || help) {
            if (help) parse_help_line(OPTW, "--rotator-target-azimuth=<deg>",
                "park on a fixed azimuth");
            else {
                state->n_options++;
                double az;
                if (parse_arg_double(arg + 25, &az) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                if (az < ANTENNA_ROTATOR_MINIMUM_AZIMUTH) {
                    az = ANTENNA_ROTATOR_MINIMUM_AZIMUTH;
                } else if (az > ANTENNA_ROTATOR_MAXIMUM_AZIMUTH) {
                    az = ANTENNA_ROTATOR_MAXIMUM_AZIMUTH;
                }
                state->rot.antenna_rotator.target_azimuth = az;
                state->rot.antenna_rotator.target_azimuth_unwrapped = az;
                state->rot.antenna_rotator.unwrapped_target_valid = 1;
                state->rot.antenna_rotator.fixed_target = 1;
            }
            matched = 1;
        }
        if (strncmp("--lat=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lat=<deg>", "geodetic latitude (default RAO Priddis)");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 6, &site_latitude) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--lon=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lon=<deg>", "geodetic longitude, east positive");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 6, &site_longitude) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--alt=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--alt=<m>", "altitude above ellipsoid, metres");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 6, &site_altitude) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strcmp("--include-constellations", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--include-constellations",
                "include Starlink/OneWeb-style swarms in the `next` pass filter");
            else { state->n_options++; with_constellations = 1; }
            matched = 1;
        }
        if (strncmp("--min-altitude-km=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-altitude-km=<km>",
                "minimum orbital altitude (default 0)");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 18, &min_altitude_km) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--max-altitude-km=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-altitude-km=<km>",
                "maximum orbital altitude (default 1000)");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 18, &max_altitude_km) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--min-elevation=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-elevation=<deg>",
                "minimum peak elevation (default 0)");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 16, &min_elevation) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--min-minutes=", arg, 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-minutes=<n>",
                "minimum minutes until AOS (default 1)");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 14, &min_minutes_away) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strncmp("--max-minutes=", arg, 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-minutes=<n>",
                "maximum minutes until AOS (default 90)");
            else {
                state->n_options++;
                if (parse_arg_double(arg + 14, &max_minutes_away) != 0) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
            }
            matched = 1;
        }
        if (strcmp("--control", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--control",
                "open the sso_ipc server (operator mode)");
            else { state->n_options++; state->control_mode = 1; }
            matched = 1;
        }
        if (strcmp("--viewer-stream", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--viewer-stream",
                "headless: stream satellite data as newline-JSON to stdout (no hardware)");
            else { state->n_options++; state->viewer_stream = 1; }
            matched = 1;
        }
        if (strcmp("--self-test", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--self-test",
                "print the settings simple_sat_ops would run with, then exit 0");
            else { state->n_options++; state->self_test = 1; }
            matched = 1;
        }

        // Unknown token. Only "--"-prefixed tokens are an error here
        // (the old code's final `strncmp("--", ...) == 0` branch);
        // a bare extra positional falls through and is caught by the
        // n_positional > 1 check after the loop.
        if (!matched && !help) {
            if (strncmp("--", arg, 2) == 0) {
                fprintf(stderr, "Unable to parse option '%s'\n", arg);
                return PARSE_ERROR;
            }
        }
    }

    // Full-help epilog: the keyboard layout + examples, printed once
    // after all the option lines (only for --help-full).
    if (help >= HELP_FULL) {
        printf(
            "\n"
            "KEYBOARD (unlocked by default, press K to toggle lock state)\n"
            "\n"
            "  K         Toggle keyboard lock\n"
            "  T         Start tracking the current satellite\n"
            "  s         Stop tracking\n"
            "  r         Reset rotator to az=0, el=0\n"
            "  [ / ]     Nudge antenna azimuth -5 / +5 deg\n"
            "  { / }     Nudge antenna azimuth -1 / +1 deg (fine)\n"
            "  , / .     Nudge antenna elevation -5 / +5 deg\n"
            "  < / >     Nudge antenna elevation -1 / +1 deg (fine)\n"
            "  q         Quit\n"
            "\n"
            "EXAMPLES\n"
            "\n"
            "  # Auto-pick next visible pass above 10 deg (rotator on by default)\n"
            "  simple_sat_ops next --min-elevation=10 --min-minutes=10 --max-minutes=45\n"
            "\n"
            "  # Dry-run prediction on a dev host (no rotator hardware)\n"
            "  simple_sat_ops 'ISS (ZARYA)' --without-rotator\n"
            "\n"
            "  # Operator coordination (broadcasts state to viewers over sso_ipc)\n"
            "  simple_sat_ops next --control\n");
    }
    if (help) return PARSE_OK;

    if (extra_positional) {
        fprintf(stderr,
            "simple_sat_ops: too many positional arguments "
            "(expected at most one <satellite_id>)\n");
        return PARSE_ERROR;
    }

    // positional was captured in the parse loop above (which correctly
    // skips space-form option values), so there is no second re-scan here.

    // --control drives hardware; --viewer-stream must never touch it.
    // Allowing both would bind the operator socket AND try to stream as a
    // detached client from the same process — refuse the combination.
    if (state->control_mode && state->viewer_stream) {
        fprintf(stderr,
            "simple_sat_ops: --control and --viewer-stream cannot be used "
            "together (one operates the hardware, the other only streams).\n");
        return PARSE_ERROR;
    }

    // Any invocation without --control: the standalone tracker is being
    // phased out in favour of the operator+viewer split, so there is no
    // longer a "track this on my own" path. Probe for the running
    // operator and either attach as a viewer or bail with a hint.
    // This holds whether or not a satellite name was given on the
    // command line - a viewer mirrors whatever the operator is tracking,
    // so any positional is ignored here. --self-test skips the probe (a
    // side effect) so the config dump runs cleanly with no live operator.
    // --viewer-stream also skips it: it loads its own TLE and streams
    // predictions even when no operator is running, so a missing operator
    // is not an error — it just means TLE-only mode.
    if (!state->control_mode && !state->self_test && !state->viewer_stream) {
        sso_ipc_client_t *probe = sso_ipc_client_connect("simple_sat_ops");
        if (probe == NULL) {
            fprintf(stderr,
                "operator not found: try `simple_sat_ops --control` "
                "to operate FrontierSat\n");
            return PARSE_ERROR;
        }
        sso_ipc_client_close(probe);
        // Operator is up — main() will dispatch into run_viewer()
        // instead of the standalone-tracker path.
        state->viewer_mode = 1;
        return PARSE_OK;
    }

    state->prediction.observer_ephem.position_geodetic.lat =
        site_latitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.lon =
        site_longitude * M_PI / 180.0;
    state->prediction.observer_ephem.position_geodetic.alt =
        site_altitude / 1000.0;

    // Resolve the TLE file when it wasn't given with --tle: the ground
    // station keeps its current elements under the FrontierSat TLEs dir, so
    // load the newest *.tle there. This is the one default for every mode
    // that gets here (--control, --viewer-stream, --self-test) — there is no
    // ~/.local/state/active.tle fallback. For --control, setup_pass_folder
    // later pins this source file (under its original tle-YYYYMMDD.tle name)
    // into the pass folder once AOS is known; --viewer-stream just
    // propagates it for the stream.
    if (state->prediction.tles_filename == NULL) {
        const char *tles_root = sso_tles_dir();
        static char src_tle[1024];
        time_t src_mtime = 0;
        if (find_newest_tle_recursive(tles_root, src_tle, sizeof src_tle,
                                      &src_mtime) != 0) {
            fprintf(stderr,
                "simple_sat_ops: no *.tle found under %s. Drop one in "
                "(or pass --tle <path>).\n", tles_root);
            return PARSE_ERROR;
        }
        fprintf(stderr, "simple_sat_ops: using TLE %s\n", src_tle);
        state->prediction.tles_filename = tle_path_resolve(src_tle);
    }

    // Resolve the satellite: an explicit name on the command line wins;
    // otherwise track whatever the TLE's first name line names. Reading the
    // name here (rather than leaving it NULL) is what keeps --self-test and
    // the no-name forms from ever feeding a NULL into the TLE search.
    if (positional != NULL) {
        state->prediction.satellite_ephem.name = positional;
    } else {
        static char sat_name[64];
        if (read_tle_name(state->prediction.tles_filename,
                          sat_name, sizeof sat_name) != 0) {
            fprintf(stderr,
                "simple_sat_ops: %s has no name line (2-line TLE?); "
                "pass the satellite name explicitly\n",
                state->prediction.tles_filename);
            return PARSE_ERROR;
        }
        state->prediction.satellite_ephem.name = sat_name;
        fprintf(stderr, "simple_sat_ops: tracking '%s'\n", sat_name);
    }

    // An explicit "next" picks the soonest matching pass. The name is always
    // set by the block above; the NULL guard is just belt-and-braces.
    if (state->prediction.satellite_ephem.name != NULL
        && strcmp(state->prediction.satellite_ephem.name, "next") == 0) {
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
            return PARSE_ERROR;
        }

        const pass_t *p = get_pass(0);
        // Deliberate one-shot, process-lifetime allocation: the chosen name
        // must outlive the pass list (freed just below) and persist for the
        // whole run. ephemeres_t.name has no consistent owner (elsewhere it
        // points at argv or a static), so we don't free it here — it is
        // reclaimed at process exit.
        state->prediction.satellite_ephem.name = strdup(p->name);
        printf("Satellite: %s\n", state->prediction.satellite_ephem.name);
    }

    return PARSE_OK;
}
