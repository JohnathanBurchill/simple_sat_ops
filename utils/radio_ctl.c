/*

   Simple Satellite Operations  utils/radio_ctl.c

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

// Subcommand-style CLI for the radio_t abstraction. No audio, no ncurses,
// no SGP4 — just enough to drive an FT-991A or IC-9700 over a serial
// port for control-protocol verification. Builds on macOS, Linux, and
// any other POSIX host with a serial driver.
//
// Examples (Mac, FT-991A on USB-CAT):
//
//   radio_ctl --radio-device=/dev/cu.usbserial-FT991A init
//   radio_ctl --radio-device=/dev/cu.usbserial-FT991A get-freq
//   radio_ctl --radio-device=/dev/cu.usbserial-FT991A set-freq 436150000
//   radio_ctl --radio-device=/dev/cu.usbserial-FT991A set-power 10
//   radio_ctl --radio-device=/dev/cu.usbserial-FT991A --allow-tx ptt on
//   radio_ctl --radio-device=/dev/cu.usbserial-FT991A --allow-tx ptt off

#include "radio.h"
#include "radio_backend.h"
#include "radio_device_store.h"

#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <strings.h>

static const char *g_argv0 = "radio_ctl";

// For the SIGINT handler. Set to point at the live radio_t while we
// might have TX keyed; cleared back to NULL afterwards. Only the raw
// PTT-off bytes are written, via a single async-signal-safe write().
static radio_t *g_signal_radio = NULL;

static void on_sigint(int sig)
{
    (void)sig;
    if (g_signal_radio != NULL && g_signal_radio->connected
        && g_signal_radio->ptt_off_raw_len > 0) {
        (void)!write(g_signal_radio->fd,
                     g_signal_radio->ptt_off_raw,
                     g_signal_radio->ptt_off_raw_len);
    }
    _exit(130);
}

static void usage(FILE *f)
{
    fprintf(f,
        "usage: %s [global-flags] <command> [args]\n"
        "\n"
        "Global flags:\n"
        "  --radio-device=<path>      Serial device. If omitted, the default\n"
        "                             stored at\n"
        "                             ~/.local/share/simple_sat_ops/radio_device\n"
        "                             is loaded; if that's missing too,\n"
        "                             /dev/ttyUSB0 is used.\n"
        "  --store-device             Persist --radio-device= as the default\n"
        "                             above. Prompts before overwriting an\n"
        "                             existing different value.\n"
        "  --radio-type=<id>          yaesu-cat (default) | icom-civ | usrp-b210\n"
        "  --radio-serial-speed=<bps> Override the speed for this run. If\n"
        "                             omitted, the value at\n"
        "                             ~/.local/share/simple_sat_ops/radio_serial_speed\n"
        "                             is loaded; if missing, 4800 (yaesu-cat)\n"
        "                             or 115200 (icom-civ) is used.\n"
        "  --store-serial-speed       Persist --radio-serial-speed= as the\n"
        "                             default above. Same overwrite-prompt\n"
        "                             behaviour as --store-device.\n"
        "  --freq-hz=<hz>             Override radio_t.nominal_downlink_frequency\n"
        "                             used by 'init' (default %.0f).\n"
        "  --tx-power=<value>         Set RF power before the subcommand runs.\n"
        "                             Plain number or 'N%%' = percent of band max\n"
        "                             (10%% safety gate, --allow-high-power to\n"
        "                             override). 'NW' = absolute watts (clamped\n"
        "                             up to the radio's hardware minimum, 5 W on\n"
        "                             FT-991A; not subject to the percent gate).\n"
        "  --allow-tx                 Required to actually key TX (ptt on).\n"
        "  --allow-high-power         Required for set-power above 10%%.\n"
        "  --allow-hf-tx              Required to key TX below 100 MHz. Blocks\n"
        "                             accidental TX into an HF/50 connector\n"
        "                             when only a VHF/UHF dummy load or\n"
        "                             antenna is wired up.\n"
        "  --debug-output             Print wire bytes as hex (default is\n"
        "                             ASCII for the FT-991A's text CAT;\n"
        "                             always hex for the IC-9700's binary\n"
        "                             CI-V).\n"
        "  --verify                   After set-freq / set-mode / set-power,\n"
        "                             read the value back and report whether\n"
        "                             it matches what was requested.\n"
        "  --duration=<seconds>       For 'ptt on': key, sleep this many\n"
        "                             seconds, then release. Fractional\n"
        "                             seconds are accepted. Ctrl-C during the\n"
        "                             sleep releases TX cleanly.\n"
        "  -h, --help                 This message.\n"
        "\n"
        "Commands:\n"
        "  init                       Run backend init() and exit.\n"
        "  uplink-prep                radio_uplink_prep() and exit.\n"
        "  get-freq                   Print current frequency in Hz.\n"
        "  set-freq <hz>              Tune the active VFO.\n"
        "  set-mode <fm|usb|lsb|am|cw>          Operating mode.\n"
        "  set-data-mode <on|off>     DATA flag (icom) / mode-swap (yaesu).\n"
        "  set-mod-input <usb|mic|acc|mic_acc|mic_usb|lan>\n"
        "                             Modulator audio input. Yaesu mapping:\n"
        "                             usb=REAR+USB CODEC, acc=REAR+DATA jack,\n"
        "                             mic=front MIC. Also pins Yaesu PKT MODE\n"
        "                             to 9600 so DATA-FM stays wide.\n"
        "  set-power <value>          RF power. Plain number or 'N%%' = percent;\n"
        "                             'NW' = absolute watts. Examples: 10, 10%%, 5W.\n"
        "  set-mod-level <0..100>     USB MOD level, %%.\n"
        "  set-moni-level <0..100>    Monitor level, %% (icom).\n"
        "  ptt <on|off>               Key / unkey.\n"
        "  power <on|off>             Soft power the radio on or off via CAT.\n"
        "                             Requires DC supply applied. Power-on\n"
        "                             includes the wake-up sequence and takes\n"
        "                             ~3 s to issue; the radio then needs\n"
        "                             ~5-10 s to fully boot before further\n"
        "                             CAT commands respond reliably.\n"
        "  identify                   Run init() then disconnect; loud about model.\n"
        "\n"
        "Exit code is RADIO_OK (0) on success, the RADIO_STATUS code otherwise.\n",
        g_argv0, FRONTIERSAT_CARRIER_HZ);
}

static int parse_pct(const char *s, int *out)
{
    if (s == NULL) return -1;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || v < 0 || v > 100) return -1;
    *out = (int)v;
    return 0;
}

// Parses a power expression: "<n>" or "<n>%" → percent (0..100),
// "<n>W" or "<n>w" → watts (>= 0). Sets *out_value and *out_is_watts.
// Bash treats '%' as literal in argv (it only has special meaning inside
// ${} parameter expansion), so 'radio_ctl --tx-power=10%' works
// unquoted on the command line.
static int parse_power(const char *s, int *out_value, int *out_is_watts)
{
    if (s == NULL || *s == '\0') return -1;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || v < 0) return -1;
    if (*end == '\0' || *end == '%') {
        if (v > 100) return -1;
        *out_is_watts = 0;
    } else if (*end == 'W' || *end == 'w') {
        *out_is_watts = 1;
    } else {
        return -1;
    }
    *out_value = (int)v;
    return 0;
}

static int parse_mode(const char *s)
{
    if (strcasecmp(s, "fm")  == 0) return RADIO_MODE_FM;
    if (strcasecmp(s, "usb") == 0) return RADIO_MODE_USB;
    if (strcasecmp(s, "lsb") == 0) return RADIO_MODE_LSB;
    if (strcasecmp(s, "am")  == 0) return RADIO_MODE_AM;
    if (strcasecmp(s, "cw")  == 0) return RADIO_MODE_CW;
    return -1;
}

static int parse_mod_source(const char *s)
{
    if (strcasecmp(s, "mic")     == 0) return RADIO_DATA_MOD_SRC_MIC;
    if (strcasecmp(s, "acc")     == 0) return RADIO_DATA_MOD_SRC_ACC;
    if (strcasecmp(s, "mic_acc") == 0) return RADIO_DATA_MOD_SRC_MIC_ACC;
    if (strcasecmp(s, "usb")     == 0) return RADIO_DATA_MOD_SRC_USB;
    if (strcasecmp(s, "mic_usb") == 0) return RADIO_DATA_MOD_SRC_MIC_USB;
    if (strcasecmp(s, "lan")     == 0) return RADIO_DATA_MOD_SRC_LAN;
    return -1;
}

static speed_t default_speed_for(radio_backend_type_t t)
{
    switch (t) {
        case RADIO_BACKEND_ICOM_CIV:  return B115200;
        // FT-991A factory default is 4800 bps. Persisted via
        // --store-serial-speed once you've changed Menu 031.
        case RADIO_BACKEND_YAESU_CAT: return B4800;
        default:                      return B115200;
    }
}

static speed_t speed_from_int(int bps)
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

int main(int argc, char **argv)
{
    g_argv0 = argv[0];

    const char *device_arg = NULL;  // explicit --radio-device=, NULL if absent
    radio_backend_type_t backend = RADIO_BACKEND_YAESU_CAT;
    int speed_override = -1;
    double nominal_hz = FRONTIERSAT_CARRIER_HZ;
    int allow_tx = 0;
    int allow_high_power = 0;
    int store_device = 0;
    int store_serial_speed = 0;
    int debug_wire = 0;
    int verify = 0;
    int allow_hf_tx = 0;
    double duration_s = 0.0;
    int tx_power_value = -1;     // < 0 = leave radio's current power alone
    int tx_power_is_watts = 0;

    int i = 1;
    for (; i < argc; ++i) {
        const char *a = argv[i];
        if (strncmp(a, "--radio-device=", 15) == 0)              device_arg = a + 15;
        else if (strcmp(a, "--store-device") == 0)               store_device = 1;
        else if (strncmp(a, "--radio-type=", 13) == 0) {
            radio_backend_type_t t = radio_backend_type_from_string(a + 13);
            if (t == RADIO_BACKEND__COUNT) {
                fprintf(stderr, "--radio-type: unknown '%s'\n", a + 13);
                return RADIO_ERROR;
            }
            backend = t;
        }
        else if (strncmp(a, "--radio-serial-speed=", 21) == 0)   speed_override = atoi(a + 21);
        else if (strcmp(a, "--store-serial-speed") == 0)         store_serial_speed = 1;
        else if (strncmp(a, "--freq-hz=", 10) == 0)              nominal_hz = atof(a + 10);
        else if (strcmp(a, "--allow-tx") == 0)                   allow_tx = 1;
        else if (strcmp(a, "--allow-high-power") == 0)           allow_high_power = 1;
        else if (strcmp(a, "--debug-output") == 0)               debug_wire = 1;
        else if (strcmp(a, "--verify") == 0)                     verify = 1;
        else if (strcmp(a, "--allow-hf-tx") == 0)                allow_hf_tx = 1;
        else if (strncmp(a, "--duration=", 11) == 0)             duration_s = atof(a + 11);
        else if (strncmp(a, "--tx-power=", 11) == 0) {
            int v, is_w;
            if (parse_power(a + 11, &v, &is_w) < 0) {
                fprintf(stderr, "--tx-power: expected '<n>', '<n>%%', or '<n>W'\n");
                return RADIO_ERROR;
            }
            tx_power_value = v;
            tx_power_is_watts = is_w;
        }
        else if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(stdout);
            return 0;
        }
        else if (a[0] != '-') break;  // start of subcommand
        else {
            fprintf(stderr, "unknown option: %s\n", a);
            usage(stderr);
            return RADIO_ERROR;
        }
    }
    if (i >= argc) {
        usage(stderr);
        return RADIO_ERROR;
    }
    const char *cmd = argv[i++];

    // --store-device handling. Three cases:
    //   1) --radio-device= absent: warn, no-op.
    //   2) Stored value matches new path: no-op.
    //   3) No stored value yet: silent fresh save.
    //   4) Stored value differs: prompt "Overwrite? [y,N]".
    if (store_device) {
        if (device_arg == NULL) {
            fprintf(stderr, "--store-device: nothing to store "
                    "(no --radio-device= given).\n");
        } else {
            char existing[1024] = {0};
            int load_rc = radio_device_store_load(existing, sizeof existing);
            if (load_rc == -1) {
                if (radio_device_store_save(device_arg) == 0) {
                    fprintf(stderr, "radio device default saved: %s\n", device_arg);
                } else {
                    perror("warning: could not save radio device default");
                }
            } else if (load_rc == 0 && strcmp(existing, device_arg) == 0) {
                // Already stored — silent no-op.
            } else if (load_rc == 0) {
                fprintf(stderr, "Overwrite stored radio device '%s' -> '%s'? [y,N] ",
                        existing, device_arg);
                fflush(stderr);
                char ans[8] = {0};
                if (fgets(ans, sizeof ans, stdin) != NULL
                    && (ans[0] == 'y' || ans[0] == 'Y')) {
                    if (radio_device_store_save(device_arg) == 0) {
                        fprintf(stderr, "radio device default updated: %s\n", device_arg);
                    } else {
                        perror("warning: could not save radio device default");
                    }
                } else {
                    fprintf(stderr, "kept existing default; using new device for "
                            "this run only.\n");
                }
            } else {
                fprintf(stderr, "warning: could not read existing default "
                        "(rc=%d); not saving.\n", load_rc);
            }
        }
    }

    // Same logic for --store-serial-speed.
    if (store_serial_speed) {
        if (speed_override <= 0) {
            fprintf(stderr, "--store-serial-speed: nothing to store "
                    "(no --radio-serial-speed= given).\n");
        } else {
            int existing = 0;
            int load_rc = radio_device_store_load_speed(&existing);
            if (load_rc == -1) {
                if (radio_device_store_save_speed(speed_override) == 0) {
                    fprintf(stderr, "radio serial speed default saved: %d\n",
                            speed_override);
                } else {
                    perror("warning: could not save serial speed default");
                }
            } else if (load_rc == 0 && existing == speed_override) {
                // Already stored — silent no-op.
            } else if (load_rc == 0) {
                fprintf(stderr, "Overwrite stored radio serial speed %d -> %d? [y,N] ",
                        existing, speed_override);
                fflush(stderr);
                char ans[8] = {0};
                if (fgets(ans, sizeof ans, stdin) != NULL
                    && (ans[0] == 'y' || ans[0] == 'Y')) {
                    if (radio_device_store_save_speed(speed_override) == 0) {
                        fprintf(stderr, "radio serial speed default updated: %d\n",
                                speed_override);
                    } else {
                        perror("warning: could not save serial speed default");
                    }
                } else {
                    fprintf(stderr, "kept existing default; using new speed for "
                            "this run only.\n");
                }
            } else {
                fprintf(stderr, "warning: could not read existing speed "
                        "(rc=%d); not saving.\n", load_rc);
            }
        }
    }

    // Resolve the effective device for this run:
    //   --radio-device= wins; else stored default; else /dev/ttyUSB0.
    char effective_device[1024];
    if (device_arg != NULL) {
        snprintf(effective_device, sizeof effective_device, "%s", device_arg);
    } else if (radio_device_store_load(effective_device,
                                       sizeof effective_device) != 0) {
        snprintf(effective_device, sizeof effective_device, "/dev/ttyUSB0");
    }

    // Resolve the effective serial speed for this run:
    //   --radio-serial-speed= wins; else stored default; else backend default.
    int effective_speed_int = -1;
    speed_t effective_speed = 0;
    if (speed_override > 0) {
        effective_speed_int = speed_override;
        effective_speed = speed_from_int(speed_override);
    } else {
        int stored = 0;
        if (radio_device_store_load_speed(&stored) == 0) {
            effective_speed_int = stored;
            effective_speed = speed_from_int(stored);
        } else {
            effective_speed = default_speed_for(backend);
        }
    }
    if (effective_speed == 0) {
        fprintf(stderr, "unsupported serial speed %d\n", effective_speed_int);
        return RADIO_ERROR;
    }

    radio_t r = {0};
    r.device_filename = effective_device;
    r.serial_speed = effective_speed;
    r.nominal_downlink_frequency = nominal_hz;
    r.tx_inhibit_cleared = allow_tx;
    r.debug_wire = debug_wire;

    if (radio_backend_select(&r, backend) != RADIO_OK) {
        return RADIO_ERROR;
    }

    int rc = radio_init(&r);
    if (rc != RADIO_OK) {
        fprintf(stderr, "radio_init failed (rc=%d)\n", rc);
        radio_disconnect(&r);
        return rc;
    }

    // --tx-power= applies before the subcommand. Most useful with 'ptt
    // on' so the power is staged before the radio keys. Two paths:
    //   percent (10% safety gate, --allow-high-power to override)
    //   absolute watts (no percent gate; backend clamps to its minimum
    //                   and there's nothing wired to the front panel
    //                   knob's > 5 W warning territory yet)
    if (tx_power_value >= 0) {
        int p_rc;
        if (tx_power_is_watts) {
            p_rc = radio_set_rf_power_watts(&r, tx_power_value);
        } else {
            if (tx_power_value > 10 && !allow_high_power) {
                fprintf(stderr, "--tx-power=%d%% above 10%% safety threshold; "
                        "add --allow-high-power.\n", tx_power_value);
                radio_disconnect(&r);
                return RADIO_ERROR;
            }
            int raw = (tx_power_value * 255 + 50) / 100;
            p_rc = radio_set_rf_power(&r, raw);
        }
        if (p_rc != RADIO_OK) {
            fprintf(stderr, "warning: --tx-power: rf_power set failed (rc=%d)\n", p_rc);
        }
    }

    if (strcmp(cmd, "init") == 0 || strcmp(cmd, "identify") == 0) {
        fprintf(stderr, "radio_ctl: %s ok (transceiver_id=0x%04X)\n",
                cmd, r.transceiver_id);
    }
    else if (strcmp(cmd, "uplink-prep") == 0) {
        rc = radio_uplink_prep(&r);
    }
    else if (strcmp(cmd, "get-freq") == 0) {
        double f = radio_get_frequency(&r);
        if (f < 0) { rc = RADIO_GET_FREQUENCY; }
        else { printf("%.0f\n", f); }
    }
    else if (strcmp(cmd, "set-freq") == 0) {
        if (i >= argc) { fprintf(stderr, "set-freq: missing <hz>\n"); rc = RADIO_ERROR; }
        else {
            double want = atof(argv[i]);
            rc = radio_set_frequency(&r, want);
            if (rc == RADIO_OK && verify) {
                double got = radio_get_frequency(&r);
                if (got < 0) {
                    fprintf(stderr, "set-freq: verify read failed\n");
                    rc = RADIO_BAD_RESPONSE;
                } else if (llround(got) != llround(want)) {
                    fprintf(stderr, "set-freq: MISMATCH requested=%.0f got=%.0f\n", want, got);
                    rc = RADIO_BAD_RESPONSE;
                } else {
                    fprintf(stderr, "set-freq: verified %.0f\n", got);
                }
            }
        }
    }
    else if (strcmp(cmd, "set-mode") == 0) {
        if (i >= argc) { fprintf(stderr, "set-mode: missing <mode>\n"); rc = RADIO_ERROR; }
        else {
            int m = parse_mode(argv[i]);
            if (m < 0) { fprintf(stderr, "set-mode: unknown '%s'\n", argv[i]); rc = RADIO_ERROR; }
            else {
                rc = radio_set_mode(&r, m, RADIO_FILTER_FIL1);
                if (rc == RADIO_OK && verify) {
                    fprintf(stderr, "set-mode: verify not implemented "
                            "(no portable get-mode op yet)\n");
                }
            }
        }
    }
    else if (strcmp(cmd, "set-data-mode") == 0) {
        if (i >= argc) { fprintf(stderr, "set-data-mode: missing <on|off>\n"); rc = RADIO_ERROR; }
        else {
            int on = (strcasecmp(argv[i], "on") == 0 || strcmp(argv[i], "1") == 0);
            rc = radio_set_data_mode(&r, on, RADIO_FILTER_FIL1);
        }
    }
    else if (strcmp(cmd, "set-mod-input") == 0) {
        if (i >= argc) { fprintf(stderr, "set-mod-input: missing <src>\n"); rc = RADIO_ERROR; }
        else {
            int s = parse_mod_source(argv[i]);
            if (s < 0) { fprintf(stderr, "set-mod-input: unknown '%s'\n", argv[i]); rc = RADIO_ERROR; }
            else { rc = radio_set_data_mod_source(&r, s); }
        }
    }
    else if (strcmp(cmd, "set-power") == 0) {
        int v, is_w;
        if (i >= argc || parse_power(argv[i], &v, &is_w) < 0) {
            fprintf(stderr, "set-power: expected '<n>', '<n>%%', or '<n>W'\n");
            rc = RADIO_ERROR;
        } else if (is_w) {
            rc = radio_set_rf_power_watts(&r, v);
        } else if (v > 10 && !allow_high_power) {
            fprintf(stderr, "set-power=%d%% above 10%% safety threshold; "
                    "add --allow-high-power.\n", v);
            rc = RADIO_ERROR;
        } else {
            int raw = (v * 255 + 50) / 100;
            rc = radio_set_rf_power(&r, raw);
        }
        if (rc == RADIO_OK && verify) {
            fprintf(stderr, "set-power: verify not implemented "
                    "(no portable get-rf-power op yet)\n");
        }
    }
    else if (strcmp(cmd, "set-mod-level") == 0) {
        int pct;
        if (i >= argc || parse_pct(argv[i], &pct) < 0) {
            fprintf(stderr, "set-mod-level: need <0..100>\n");
            rc = RADIO_ERROR;
        } else {
            int raw = (pct * 255 + 50) / 100;
            rc = radio_set_usb_mod_level(&r, raw);
        }
    }
    else if (strcmp(cmd, "set-moni-level") == 0) {
        int pct;
        if (i >= argc || parse_pct(argv[i], &pct) < 0) {
            fprintf(stderr, "set-moni-level: need <0..100>\n");
            rc = RADIO_ERROR;
        } else {
            int raw = (pct * 255 + 50) / 100;
            rc = radio_set_moni_level(&r, raw);
        }
    }
    else if (strcmp(cmd, "power") == 0) {
        if (i >= argc) { fprintf(stderr, "power: missing <on|off>\n"); rc = RADIO_ERROR; }
        else {
            int on = (strcasecmp(argv[i], "on") == 0 || strcmp(argv[i], "1") == 0);
            rc = radio_power(&r, on);
            if (rc == RADIO_OK && on) {
                fprintf(stderr, "power: PS1; sent. Radio takes ~5-10 s to boot; "
                        "wait before issuing further CAT.\n");
            }
        }
    }
    else if (strcmp(cmd, "ptt") == 0) {
        if (i >= argc) { fprintf(stderr, "ptt: missing <on|off>\n"); rc = RADIO_ERROR; }
        else {
            int on = (strcasecmp(argv[i], "on") == 0 || strcmp(argv[i], "1") == 0);
            // Before keying, check the radio is on a band that goes to the
            // V/U connector. The FT-991A's HF/50 connector handles 1.8 MHz
            // through 50 MHz; anything < 100 MHz routes there. If your
            // dummy load / antenna is only on V/U, an HF TX into an
            // unconnected jack is a serious risk to the PA.
            int hf_blocked = 0;
            if (on && !allow_hf_tx) {
                double f = radio_get_frequency(&r);
                if (f > 0.0 && f < 100e6) {
                    fprintf(stderr,
                            "ptt: refusing — current freq %.6f MHz routes to "
                            "the HF/50 connector. Add --allow-hf-tx if you "
                            "really mean it.\n", f / 1e6);
                    rc = RADIO_ERROR;
                    hf_blocked = 1;
                }
            }
            if (!hf_blocked) {
                if (on && duration_s > 0.0) {
                    // Auto-release path: install a Ctrl-C handler so the
                    // radio is unkeyed even if the user kills us mid-sleep,
                    // then key, sleep, release.
                    g_signal_radio = &r;
                    struct sigaction sa = {0};
                    sa.sa_handler = on_sigint;
                    sigemptyset(&sa.sa_mask);
                    sigaction(SIGINT,  &sa, NULL);
                    sigaction(SIGTERM, &sa, NULL);

                    rc = radio_ptt(&r, 1);
                    if (rc == RADIO_OK) {
                        fprintf(stderr, "ptt: keyed for %.3f s ...\n", duration_s);
                        usleep((useconds_t)(duration_s * 1.0e6));
                        rc = radio_ptt(&r, 0);
                    }
                    g_signal_radio = NULL;
                } else {
                    rc = radio_ptt(&r, on);
                }
            }
        }
    }
    else {
        fprintf(stderr, "unknown command: %s\n", cmd);
        usage(stderr);
        rc = RADIO_ERROR;
    }

    // No unconditional TX0; cleanup. None of the subcommands above can
    // leave the radio keyed unless 'ptt on' was just run (and that path
    // is intentionally left keyed for the user's follow-up 'ptt off').
    // An auto-TX0; here was preempting the previous command in flight —
    // for example, a set-freq's FA<freq>; would still be processing
    // inside the radio when TX0; arrived ~3 ms later, causing the freq
    // change to be silently discarded.
    radio_disconnect(&r);

    if (rc != RADIO_OK) {
        fprintf(stderr, "radio_ctl: %s failed (rc=%d)\n", cmd, rc);
    }
    return rc;
}
