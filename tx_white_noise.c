/*

    Simple Satellite Operations  tx_white_noise.c

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

// Sibling of tx_tone: keys the radio and plays uniformly random samples
// across the full 0..Nyquist (24 kHz at 48 kHz/2) baseband. The captured
// loopback or off-air spectrogram of the resulting transmission is the
// combined TX+RX bandshape — what the modulator and demodulator pass,
// independent of any specific tone or modulation scheme.

#include "radio.h"
#include "radio_device_store.h"
#include "audio.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>

static radio_t g_radio = {0};
static int g_no_ptt = 0;
static pid_t g_record_pid = 0;

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

static void stop_recorder(void)
{
    if (g_record_pid <= 0) return;
    kill(g_record_pid, SIGINT);
    int status = 0;
    waitpid(g_record_pid, &status, 0);
    g_record_pid = 0;
}

static void on_signal(int sig)
{
    (void)sig;
    stop_recorder();
    if (g_radio.connected) {
        if (!g_no_ptt) {
            radio_ptt(&g_radio, 0);
        }
        radio_disconnect(&g_radio);
    }
    _exit(130);
}

static void usage(FILE *dest, const char *name)
{
    fprintf(dest,
        "usage: %s [options]\n"
        "\n"
        "Key the radio and play full-band white noise (0..Nyquist) through the\n"
        "TX audio chain. Capture the loopback or off-air spectrogram to read off\n"
        "the combined TX+RX inherent frequency response.\n"
        "\n"
        "Audio:\n"
        "  --signalink-audio            (default) Auto-detect the SignaLink USB\n"
        "                               (TI Burr-Brown PCM2901/PCM2904) and use\n"
        "                               its plughw:N,0. Implies --mod-input=acc.\n"
        "  --radio-audio                Auto-detect the radio's native USB CODEC\n"
        "                               (Yaesu/ICOM) and use its plughw:N,0.\n"
        "                               Implies --mod-input=usb.\n"
        "  --audio-device=<name>        Explicit ALSA PCM device, overriding\n"
        "                               either auto-detect (e.g. plughw:1,0)\n"
        "\n"
        "Radio transport:\n"
        "  --radio-device=<path>        CAT/CI-V tty. Falls back to the saved\n"
        "                               default in ~/.local/share/simple_sat_ops/\n"
        "                               radio_device, then /dev/ttyUSB1.\n"
        "  --radio-serial-speed=<bps>   Serial speed integer. Falls back to the\n"
        "                               saved default, then 4800 (yaesu-cat) or\n"
        "                               115200 (icom-civ).\n"
        "  --radio-type=<id>            yaesu-cat (default) | icom-civ | usrp-b210\n"
        "\n"
        "Carrier and signal:\n"
        "  --freq-hz=<hz>               UHF simplex carrier (default %.0f\n"
        "                               = %.6f MHz, FRONTIERSAT_CARRIER_HZ)\n"
        "  --duration-s=<seconds>       Noise burst duration (default 5.0)\n"
        "  --amplitude=<0..1>           Peak amplitude into S16 full-scale\n"
        "                               (default 0.2; uniform PRNG, so RMS is\n"
        "                               ~0.577 of peak)\n"
        "  --seed=<u64>                 PRNG seed (default 1 = repeatable runs;\n"
        "                               pass 0 to seed from the wall clock)\n"
        "  --bandwidth-hz=<hz>          Band-limit the noise to <hz> via a 127-tap\n"
        "                               windowed-sinc low-pass FIR. Default 0 = no\n"
        "                               filtering (full Nyquist = 24 kHz at\n"
        "                               48 kHz fs). Use e.g. 15000 to test the\n"
        "                               radio's response over just 0..15 kHz.\n"
        "  --mode=<fm|fm-data>          TX mode (default fm-data). Use fm to\n"
        "                               drive the radio's voice-FM mod chain\n"
        "                               (subject to mic-EQ / pre-emphasis /\n"
        "                               front-panel MOD INPUT) instead of the\n"
        "                               flat FM-DATA path.\n"
        "  --mod-input=<src>            Modulator audio input: usb|acc|mic|\n"
        "                               mic+acc|mic+usb|lan (default acc).\n"
        "                               Yaesu mapping: usb=REAR+USB CODEC,\n"
        "                               acc=REAR+DATA jack, mic=front MIC.\n"
        "                               Default targets SignaLink-on-rear-DATA;\n"
        "                               pass --mod-input=usb for native USB.\n"
        "                               Ignored when --mode=fm.\n"
        "  --filter=<fil1|fil2|fil3>    IC-9700 IF filter slot. Overrides the\n"
        "                               FIL1 default set by radio_uplink_prep;\n"
        "                               useful for comparing FIL1 vs FIL2/3\n"
        "                               passband shapes against the same noise\n"
        "                               source. Ignored on backends without\n"
        "                               per-slot filters (e.g. FT-991A).\n"
        "\n"
        "Behaviour flags:\n"
        "  --no-ptt                     Skip PTT; play audio only (bench test)\n"
        "  --record=<path>              Capture audio device to <path>\n"
        "                               (headerless S16_LE; default: auto-named\n"
        "                               tx_white_noise_UT=YYYYMMDDTHHMMSS.sss.raw)\n"
        "  --no-record                  Disable auto-recording\n"
        "  --record-warmup-ms=<ms>      Delay between arecord start and PTT on\n"
        "                               (default 600)\n"
        "  --moni-level=<0..100>        MONI loopback gain, %%\n"
        "  --uplink-mod-level=<0..100>  USB MOD level, %%\n"
        "  --tx-power=<0..100>          RF power, %%. Untouched if omitted.\n"
        "  --allow-high-power           Required for --tx-power above 10%%.\n"
        "  --allow-tx                   Clear default TX inhibit; required to\n"
        "                               actually key the radio.\n"
        "  --help                       Show this help\n",
        name, FRONTIERSAT_CARRIER_HZ, FRONTIERSAT_CARRIER_HZ / 1e6);
}

static int start_recorder(const char *device, const char *out_path,
                          int warmup_ms)
{
    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "error: fork() for arecord failed: %s\n", strerror(errno));
        return -1;
    }
    if (pid == 0) {
        char rate_str[16], chan_str[8];
        snprintf(rate_str, sizeof rate_str, "%u", AUDIO_RATE_HZ);
        snprintf(chan_str, sizeof chan_str, "%u", AUDIO_CHANNELS);
        char *argv_rec[] = {
            "arecord",
            "-q",
            "-D", (char *)device,
            "-f", "S16_LE",
            "-r", rate_str,
            "-c", chan_str,
            "-t", "raw",
            (char *)out_path,
            NULL,
        };
        execvp("arecord", argv_rec);
        fprintf(stderr, "error: execvp(arecord) failed: %s\n", strerror(errno));
        _exit(127);
    }
    g_record_pid = pid;
    if (warmup_ms > 0) {
        usleep((useconds_t)warmup_ms * 1000);
    }
    return 0;
}

static int make_auto_record_path(const char *tool, char *out, size_t out_size)
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) {
        return -1;
    }
    struct tm utc;
    if (gmtime_r(&tv.tv_sec, &utc) == NULL) {
        return -1;
    }
    int n = snprintf(out, out_size,
                     "%s_UT=%04d%02d%02dT%02d%02d%02d.%03ld.raw",
                     tool,
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec,
                     (long)(tv.tv_usec / 1000));
    return (n > 0 && (size_t)n < out_size) ? 0 : -1;
}

static int derive_wav_path(const char *raw_path, char *out, size_t out_size)
{
    if (raw_path == NULL) return -1;
    size_t len = strlen(raw_path);
    int n;
    if (len >= 4 && strcmp(raw_path + len - 4, ".raw") == 0) {
        n = snprintf(out, out_size, "%.*s.wav", (int)(len - 4), raw_path);
    } else {
        n = snprintf(out, out_size, "%s.wav", raw_path);
    }
    return (n > 0 && (size_t)n < out_size) ? 0 : -1;
}

int main(int argc, char **argv)
{
    const char *radio_device = NULL;
    int radio_speed_bps = -1;
    const char *audio_device = NULL;
    char audio_device_buf[64] = {0};
    int audio_pick = 1;  // 0=explicit, 1=signalink (default), 2=radio
    const char *record_path = NULL;
    char auto_record_path[256];
    int no_record = 0;
    double freq_hz = FRONTIERSAT_CARRIER_HZ;
    double duration_s = 5.0;
    double amplitude = 0.2;
    uint64_t seed = 1;
    double bandwidth_hz = 0.0;  // 0 = full Nyquist, no filtering
    int data_mod_source = -1;
    int use_data_mode = 1;  // 1 = FM-DATA (default); 0 = plain FM
    int filter = -1;  // < 0 = leave whatever radio_uplink_prep picked (FIL1)
    int moni_level_pct = -1;
    int uplink_mod_level = -1;
    int tx_power_pct = -1;
    int allow_high_power = 0;
    int allow_tx = 0;
    radio_backend_type_t radio_backend = RADIO_BACKEND_YAESU_CAT;
    int record_warmup_ms = 600;

    for (int i = 1; i < argc; i++) {
        if (strncmp("--radio-device=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            radio_device = argv[i] + 15;
        } else if (strncmp("--radio-serial-speed=", argv[i], 21) == 0) {
            if (strlen(argv[i]) < 22) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            radio_speed_bps = atoi(argv[i] + 21);
        } else if (strncmp("--audio-device=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            audio_device = argv[i] + 15;
            audio_pick = 0;
        } else if (strcmp("--signalink-audio", argv[i]) == 0) {
            audio_pick = 1;
        } else if (strcmp("--radio-audio", argv[i]) == 0) {
            audio_pick = 2;
        } else if (strncmp("--freq-hz=", argv[i], 10) == 0) {
            if (strlen(argv[i]) < 11) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            freq_hz = atof(argv[i] + 10);
        } else if (strncmp("--duration-s=", argv[i], 13) == 0) {
            if (strlen(argv[i]) < 14) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            duration_s = atof(argv[i] + 13);
        } else if (strncmp("--amplitude=", argv[i], 12) == 0) {
            if (strlen(argv[i]) < 13) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            amplitude = atof(argv[i] + 12);
        } else if (strncmp("--seed=", argv[i], 7) == 0) {
            if (strlen(argv[i]) < 8) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            seed = strtoull(argv[i] + 7, NULL, 0);
        } else if (strncmp("--bandwidth-hz=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            bandwidth_hz = atof(argv[i] + 15);
            if (bandwidth_hz < 0.0) {
                fprintf(stderr, "--bandwidth-hz must be >= 0\n");
                return EXIT_FAILURE;
            }
        } else if (strncmp("--mod-input=", argv[i], 12) == 0) {
            if (strlen(argv[i]) < 13) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            const char *s = argv[i] + 12;
            if      (strcmp(s, "mic")     == 0) data_mod_source = RADIO_DATA_MOD_SRC_MIC;
            else if (strcmp(s, "acc")     == 0) data_mod_source = RADIO_DATA_MOD_SRC_ACC;
            else if (strcmp(s, "mic+acc") == 0) data_mod_source = RADIO_DATA_MOD_SRC_MIC_ACC;
            else if (strcmp(s, "usb")     == 0) data_mod_source = RADIO_DATA_MOD_SRC_USB;
            else if (strcmp(s, "mic+usb") == 0) data_mod_source = RADIO_DATA_MOD_SRC_MIC_USB;
            else if (strcmp(s, "lan")     == 0) data_mod_source = RADIO_DATA_MOD_SRC_LAN;
            else {
                fprintf(stderr, "--mod-input: unknown '%s' (mic|acc|mic+acc|usb|mic+usb|lan)\n", s);
                return EXIT_FAILURE;
            }
        } else if (strncmp("--mode=", argv[i], 7) == 0) {
            if (strlen(argv[i]) < 8) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            const char *s = argv[i] + 7;
            if      (strcmp(s, "fm-data") == 0) use_data_mode = 1;
            else if (strcmp(s, "fm")      == 0) use_data_mode = 0;
            else {
                fprintf(stderr, "--mode: unknown '%s' (fm|fm-data)\n", s);
                return EXIT_FAILURE;
            }
        } else if (strncmp("--filter=", argv[i], 9) == 0) {
            if (strlen(argv[i]) < 10) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            const char *s = argv[i] + 9;
            if      (strcmp(s, "fil1") == 0) filter = RADIO_FILTER_FIL1;
            else if (strcmp(s, "fil2") == 0) filter = RADIO_FILTER_FIL2;
            else if (strcmp(s, "fil3") == 0) filter = RADIO_FILTER_FIL3;
            else {
                fprintf(stderr, "--filter: unknown '%s' (fil1|fil2|fil3)\n", s);
                return EXIT_FAILURE;
            }
        } else if (strcmp("--no-ptt", argv[i]) == 0) {
            g_no_ptt = 1;
        } else if (strncmp("--record=", argv[i], 9) == 0) {
            if (strlen(argv[i]) < 10) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            record_path = argv[i] + 9;
        } else if (strcmp("--no-record", argv[i]) == 0) {
            no_record = 1;
        } else if (strncmp("--moni-level=", argv[i], 13) == 0) {
            if (strlen(argv[i]) < 14) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            moni_level_pct = atoi(argv[i] + 13);
            if (moni_level_pct < 0 || moni_level_pct > 100) {
                fprintf(stderr, "--moni-level must be 0..100 (%%)\n");
                return EXIT_FAILURE;
            }
        } else if (strncmp("--uplink-mod-level=", argv[i], 19) == 0) {
            if (strlen(argv[i]) < 20) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            uplink_mod_level = atoi(argv[i] + 19);
            if (uplink_mod_level < 0 || uplink_mod_level > 100) {
                fprintf(stderr, "--uplink-mod-level must be 0..100 (%%)\n");
                return EXIT_FAILURE;
            }
        } else if (strncmp("--tx-power=", argv[i], 11) == 0) {
            if (strlen(argv[i]) < 12) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            tx_power_pct = atoi(argv[i] + 11);
            if (tx_power_pct < 0 || tx_power_pct > 100) {
                fprintf(stderr, "--tx-power must be 0..100 (%%)\n");
                return EXIT_FAILURE;
            }
        } else if (strcmp("--allow-high-power", argv[i]) == 0) {
            allow_high_power = 1;
        } else if (strcmp("--allow-tx", argv[i]) == 0) {
            allow_tx = 1;
        } else if (strncmp("--radio-type=", argv[i], 13) == 0) {
            if (strlen(argv[i]) < 14) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            radio_backend_type_t t = radio_backend_type_from_string(argv[i] + 13);
            if (t == RADIO_BACKEND__COUNT) {
                fprintf(stderr, "--radio-type: unknown '%s' "
                        "(icom-civ|yaesu-cat|usrp-b210)\n", argv[i] + 13);
                return EXIT_FAILURE;
            }
            radio_backend = t;
        } else if (strncmp("--record-warmup-ms=", argv[i], 19) == 0) {
            if (strlen(argv[i]) < 20) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            record_warmup_ms = atoi(argv[i] + 19);
            if (record_warmup_ms < 0) record_warmup_ms = 0;
        } else if (strcmp("--help", argv[i]) == 0) {
            usage(stdout, argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            usage(stderr, argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (audio_device == NULL) {
        const char *backend_hint =
            (radio_backend == RADIO_BACKEND_ICOM_CIV) ? "icom" :
            (radio_backend == RADIO_BACKEND_YAESU_CAT) ? "yaesu" : NULL;
        int rc;
        if (audio_pick == 2) {
            rc = audio_find_radio_device(backend_hint, audio_device_buf,
                                         sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "error: --radio-audio: no matching ALSA card "
                        "found (rc=%d)\n", rc);
                return EXIT_FAILURE;
            }
            if (data_mod_source < 0) data_mod_source = RADIO_DATA_MOD_SRC_USB;
        } else {
            rc = audio_find_signalink_device(audio_device_buf,
                                             sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "error: --signalink-audio: SignaLink not "
                        "found (rc=%d). Pass --audio-device= or "
                        "--radio-audio.\n", rc);
                return EXIT_FAILURE;
            }
            if (data_mod_source < 0) data_mod_source = RADIO_DATA_MOD_SRC_ACC;
        }
        audio_device = audio_device_buf;
        fprintf(stderr, "tx_white_noise: auto-detected audio device %s\n",
                audio_device);
    }

    char effective_device[1024];
    if (radio_device != NULL) {
        snprintf(effective_device, sizeof effective_device, "%s", radio_device);
    } else if (radio_device_store_load(effective_device,
                                       sizeof effective_device) != 0) {
        snprintf(effective_device, sizeof effective_device, "/dev/ttyUSB1");
    }
    int effective_bps = radio_speed_bps;
    if (effective_bps <= 0) {
        if (radio_device_store_load_speed(&effective_bps) != 0) {
            effective_bps = (radio_backend == RADIO_BACKEND_YAESU_CAT)
                                ? 4800 : 115200;
        }
    }
    speed_t radio_speed = speed_from_bps(effective_bps);
    if (radio_speed == 0) {
        fprintf(stderr, "error: unsupported serial speed %d\n", effective_bps);
        return EXIT_FAILURE;
    }

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    g_radio.device_filename = effective_device;
    g_radio.serial_speed = radio_speed;
    g_radio.nominal_downlink_frequency = freq_hz;
    g_radio.sub_park_frequency = RADIO_SUB_PARK_HZ;
    g_radio.tx_inhibit_cleared = allow_tx;
    if (radio_backend_select(&g_radio, radio_backend) != RADIO_OK) {
        return EXIT_FAILURE;
    }
    int rc = radio_init(&g_radio);
    if (rc != RADIO_OK) {
        fprintf(stderr, "error: radio_init failed (rc=%d)\n", rc);
        if (g_radio.connected) radio_disconnect(&g_radio);
        return EXIT_FAILURE;
    }
    const int eff_filter = (filter >= 0) ? filter : RADIO_FILTER_FIL1;
    if (use_data_mode) {
        rc = radio_uplink_prep(&g_radio);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: radio_uplink_prep failed (rc=%d); "
                    "check MODE/DATA/DATA-MOD on the front panel.\n", rc);
        }
        if (data_mod_source >= 0) {
            rc = radio_set_data_mod_source(&g_radio, data_mod_source);
            if (rc != RADIO_OK) {
                fprintf(stderr, "warning: could not override DATA MOD source (rc=%d)\n", rc);
            }
        }
        // Filter override. radio_uplink_prep set FM + DATA on with FIL1; if
        // the user asked for a different slot, re-issue both calls.
        if (filter >= 0) {
            rc = radio_set_mode(&g_radio, RADIO_MODE_FM, filter);
            if (rc != RADIO_OK && rc != RADIO_NOT_SUPPORTED) {
                fprintf(stderr, "warning: could not set filter (set_mode rc=%d)\n", rc);
            }
            rc = radio_set_data_mode(&g_radio, 1, filter);
            if (rc != RADIO_OK && rc != RADIO_NOT_SUPPORTED) {
                fprintf(stderr, "warning: could not set filter (set_data_mode rc=%d)\n", rc);
            }
        }
    } else {
        // Plain FM: tune, set mode, and clear the DATA flag in case the
        // radio was previously left in FM-DATA. DATA-MOD source doesn't
        // apply in plain FM (the radio uses the front-panel MOD INPUT
        // setting), so we don't issue it.
        rc = radio_set_frequency(&g_radio, freq_hz);
        if (rc != RADIO_OK && rc != RADIO_NOT_SUPPORTED) {
            fprintf(stderr, "warning: set_frequency rc=%d\n", rc);
        }
        rc = radio_set_mode(&g_radio, RADIO_MODE_FM, eff_filter);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: set_mode FM rc=%d\n", rc);
        }
        rc = radio_set_data_mode(&g_radio, 0, eff_filter);
        if (rc != RADIO_OK && rc != RADIO_NOT_SUPPORTED) {
            fprintf(stderr, "warning: clearing DATA flag rc=%d\n", rc);
        }
        if (data_mod_source >= 0) {
            fprintf(stderr, "warning: --mod-input ignored in --mode=fm "
                    "(plain FM uses the front-panel MOD INPUT)\n");
        }
    }
    if (tx_power_pct >= 0) {
        if (tx_power_pct > 10 && !allow_high_power) {
            fprintf(stderr, "error: --tx-power=%d above 10%% safety threshold; "
                    "add --allow-high-power to override.\n", tx_power_pct);
            radio_disconnect(&g_radio);
            return EXIT_FAILURE;
        }
        int raw = (int)((tx_power_pct * 255 + 50) / 100);
        rc = radio_set_rf_power(&g_radio, raw);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: could not set RF power (rc=%d)\n", rc);
        }
    }
    if (uplink_mod_level >= 0) {
        int raw = (int)((uplink_mod_level * 255 + 50) / 100);
        rc = radio_set_usb_mod_level(&g_radio, raw);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: could not set USB MOD level (rc=%d)\n", rc);
        }
    }
    if (moni_level_pct >= 0) {
        int raw = (int)((moni_level_pct * 255 + 50) / 100);
        rc = radio_set_moni_level(&g_radio, raw);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: could not set MONI level (rc=%d)\n", rc);
        }
    }

    fprintf(stderr,
            "tx_white_noise: %.6f MHz %s simplex, white noise %.2f s, amp %.2f, "
            "seed=%llu, audio %s, ptt %s\n",
            freq_hz / 1e6, use_data_mode ? "FM-DATA" : "FM", duration_s, amplitude,
            (unsigned long long)seed, audio_device, g_no_ptt ? "off" : "on");

    if (record_path == NULL && !no_record) {
        if (make_auto_record_path("tx_white_noise", auto_record_path,
                                  sizeof auto_record_path) != 0) {
            fprintf(stderr, "warning: could not build auto-record filename; "
                    "recording disabled for this run\n");
        } else {
            record_path = auto_record_path;
        }
    }

    if (record_path != NULL) {
        fprintf(stderr, "tx_white_noise: recording %s <- %s (warmup %d ms)\n",
                record_path, audio_device, record_warmup_ms);
        if (start_recorder(audio_device, record_path, record_warmup_ms) != 0) {
            goto fail_radio;
        }
    }

    snd_pcm_t *playback = NULL;
    int arc = audio_playback_open(&playback, audio_device, AUDIO_RATE_HZ, AUDIO_CHANNELS);
    if (arc != AUDIO_OK || playback == NULL) {
        fprintf(stderr, "error: cannot open ALSA playback device '%s' (rc=%d)\n", audio_device, arc);
        stop_recorder();
        goto fail_radio;
    }

    // Synth WAV: same base name as the loopback .raw, .wav extension, in
    // CWD. Always written; --no-record only suppresses the loopback side.
    char synth_wav_path[300] = {0};
    const char *synth_wav_base = record_path;
    char synth_wav_auto[256];
    if (synth_wav_base == NULL) {
        if (make_auto_record_path("tx_white_noise", synth_wav_auto,
                                  sizeof synth_wav_auto) == 0) {
            synth_wav_base = synth_wav_auto;
        }
    }
    audio_wav_writer_t *synth_wav = NULL;
    if (synth_wav_base != NULL
        && derive_wav_path(synth_wav_base, synth_wav_path,
                           sizeof synth_wav_path) == 0) {
        synth_wav = audio_wav_writer_open(synth_wav_path,
                                          AUDIO_RATE_HZ, AUDIO_CHANNELS);
        if (synth_wav == NULL) {
            fprintf(stderr, "warning: could not open synth WAV %s\n",
                    synth_wav_path);
        } else {
            fprintf(stderr, "tx_white_noise: synth WAV -> %s\n",
                    synth_wav_path);
        }
    }

    if (!g_no_ptt) {
        rc = radio_ptt(&g_radio, 1);
        if (rc != RADIO_OK) {
            fprintf(stderr, "error: PTT on (rc=%d)\n", rc);
            audio_wav_writer_close(synth_wav);
            stop_recorder();
            audio_playback_close(playback);
            goto fail_radio;
        }
        usleep(100000);
    }

    arc = audio_play_white_noise(playback, synth_wav, amplitude, duration_s,
                                 AUDIO_RATE_HZ, AUDIO_CHANNELS, seed,
                                 bandwidth_hz);
    audio_wav_writer_close(synth_wav);
    audio_playback_close(playback);
    if (synth_wav_path[0] != '\0') {
        audio_generate_spectrogram(synth_wav_path);
    }

    if (!g_no_ptt) {
        usleep(200000);
        radio_ptt(&g_radio, 0);
    }
    stop_recorder();
    radio_disconnect(&g_radio);

    if (arc != AUDIO_OK) {
        fprintf(stderr, "error: noise generation failed (rc=%d)\n", arc);
        return EXIT_FAILURE;
    }
    return 0;

fail_radio:
    stop_recorder();
    if (!g_no_ptt && g_radio.connected) {
        radio_ptt(&g_radio, 0);
    }
    radio_disconnect(&g_radio);
    return EXIT_FAILURE;
}
