/*

    Simple Satellite Operations  tx_tone.c

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

static void usage(FILE *dest, const char *name, int full)
{
    fprintf(dest,
        "usage: %s [options]\n"
        "\n"
        "Key the configured radio on a simplex UHF carrier and play a sine tone\n"
        "through the TX chain. Uplink bring-up tool; does no modulation yet.\n"
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
        "\n"
        "Carrier and tone:\n"
        "  --freq-hz=<hz>               UHF simplex carrier (default %.0f\n"
        "                               = %.6f MHz, FRONTIERSAT_CARRIER_HZ)\n"
        "  --tone-hz=<hz>               Tone frequency / step start (default 1000)\n"
        "  --tone-stop-hz=<hz>          Stepped sweep stop frequency. If set, plays\n"
        "                               steps from --tone-hz to here in --tone-step-hz\n"
        "                               increments, --duration-s seconds per step.\n"
        "  --tone-step-hz=<hz>          Step size for stepped sweep (default 500)\n"
        "  --duration-s=<seconds>       Tone duration (default 3.0). Per-step when\n"
        "                               --tone-stop-hz is set.\n"
        "  --amplitude=<0..1>           Amplitude into S16 full-scale (default 0.3)\n"
        "  --mod-input=<src>            Modulator audio input: usb|acc|mic|\n"
        "                               mic+acc|mic+usb|lan (default acc).\n"
        "                               Yaesu mapping: usb=REAR+USB CODEC,\n"
        "                               acc=REAR+DATA jack, mic=front MIC.\n"
        "                               Default targets the SignaLink-on-rear\n"
        "                               -DATA path; pass --mod-input=usb for\n"
        "                               the radio's native USB audio.\n"
        "\n"
        "Behaviour flags:\n"
        "  --no-ptt                     Skip PTT; play audio only (bench test)\n"
        "  --record=<path>              Capture the audio device to <path>\n"
        "                               (headerless S16_LE; default: auto-named\n"
        "                               tx_tone_UT=YYYYMMDDTHHMMSS.sss.raw in CWD)\n"
        "  --no-record                  Disable auto-recording\n"
        "  --record-warmup-ms=<ms>      Delay between arecord start and PTT on so\n"
        "                               its DMA/CODEC startup transient lands\n"
        "                               before TX audio (default 600)\n"
        "  --moni-level=<0..100>        MONI loopback gain, %%. Backend-specific;\n"
        "                               IC-9700 only on this branch.\n"
        "  --uplink-mod-level=<0..100>  USB MOD level, %% — how loud your PCM is\n"
        "                               on the modulator.\n"
        "  --tx-power=<0..100>          RF power, %%. Untouched if omitted (uses\n"
        "                               the radio's current setting).\n"
        "  --allow-high-power           Required for --tx-power above 10%%.\n"
        "  --allow-tx                   Clear the default TX inhibit. Without this\n"
        "                               flag PTT is gated and the radio is\n"
        "                               configured but never keyed.\n"
        "  --radio-type=<id>            yaesu-cat (default) | icom-civ | usrp-b210\n"
        "  --help                       Short help (this message)\n"
        "  --help-full                  Detailed help with setup and verification\n",
        name, FRONTIERSAT_CARRIER_HZ, FRONTIERSAT_CARRIER_HZ / 1e6);

    if (!full) return;

    fprintf(dest,
        "\n"
        "TRANSPORT SETUPS\n"
        "\n"
        "All supported transparently; pick the radio with --radio-type=.\n"
        "\n"
        "  A. SignaLink (or similar external USB soundcard) into rear DATA jack\n"
        "       --radio-type=yaesu-cat --radio-device=/dev/ttyUSBn\n"
        "       --audio-device=plughw:<signalink-card>,0 --mod-input=acc\n"
        "       Default mod path; works on FT-991A and IC-9700.\n"
        "\n"
        "  B. Radio's native USB CODEC (audio over USB Audio Class)\n"
        "       --radio-type=yaesu-cat --radio-device=/dev/ttyUSBn\n"
        "       --audio-device=plughw:<radio-card>,0 --mod-input=usb\n"
        "       FT-991A and IC-9700 both expose a USB CODEC; on the FT-991A\n"
        "       the codec has been observed to pass tones up to 12 kHz.\n"
        "\n"
        "Run `aplay -l` on the target to enumerate ALSA cards and pick the right\n"
        "hw:N,M string for --audio-device.\n"
        "\n"
        "RADIO CONFIGURATION AT STARTUP\n"
        "\n"
        "tx_tone configures the radio fully every run via radio_uplink_prep:\n"
        "  - tune to --freq-hz on the active VFO\n"
        "  - FM mode\n"
        "  - DATA MOD source = --mod-input (default acc = rear DATA jack)\n"
        "  - DATA mode ON (FM-DATA / DATA-FM depending on backend)\n"
        "  - PTT asserted via the active backend (unless --no-ptt or TX is\n"
        "    inhibited; see --allow-tx)\n"
        "  - On exit (clean or signal) PTT is released before disconnect.\n"
        "\n"
        "Operator-set front-panel items (not driven by CAT) — see the comment\n"
        "block at the top of radio_yaesu_cat.c for the FT-991A checklist.\n"
        "\n"
        "VERIFICATION SEQUENCE\n"
        "\n"
        "  1. Bench audio, no radio:\n"
        "       %s --no-ptt --audio-device=hw:Loopback,0 --duration-s=2\n"
        "     Confirms NCO + ALSA playback via snd-aloop.\n"
        "\n"
        "  2. PTT only, dummy load, no audio:\n"
        "       %s --duration-s=0.5 --amplitude=0 --allow-tx\n"
        "     Radio keys TX briefly then releases. Confirms CAT-side PTT.\n"
        "\n"
        "  3. Full end-to-end on a dummy load, monitor on a 2nd receiver:\n"
        "       %s --duration-s=5 --amplitude=0.3 --allow-tx\n"
        "\n"
        "RECORDING THE TX\n"
        "\n"
        "  Recording is ON by default; each run drops a headerless S16_LE\n"
        "  raw file into the current working directory named\n"
        "  `tx_tone_UT=YYYYMMDDTHHMMSS.sss.raw`. Pass --record=<path> to\n"
        "  override the filename, or --no-record to disable entirely.\n"
        "\n"
        "  To hear anything in the capture, the radio's MONI function must\n"
        "  be on (MONI key + MONITOR LEVEL non-zero); otherwise the USB AF\n"
        "  output is muted during TX and the file is silence.\n"
        "\n"
        "  Replay: `ffplay -f s16le -ar 48000 -ac 2 <file>.raw`, or convert\n"
        "  to WAV with `ffmpeg -f s16le -ar 48000 -ac 2 -i <file>.raw ...`.\n"
        "\n"
        "CAVEATS\n"
        "\n"
        "  - FM-DATA mode (flat TX audio, no mic EQ) — same path AX100 frames\n"
        "    use. Radio is left in FM-DATA on exit.\n"
        "  - --mod-input picks the modulator audio source via CAT, but a few\n"
        "    front-panel menus are operator-set (FT-991A: 071=DAKY, 072=DATA;\n"
        "    see radio_yaesu_cat.c for the full checklist).\n"
        "  - SIGINT / SIGTERM handler releases PTT before exit, so ^C is safe.\n",
        name, name, name);
}

// Spawn `arecord` capturing the USB CODEC into a headerless S16_LE raw
// file. Child PID is stashed in g_record_pid so on_signal() / the main
// flow can SIGINT it. Headerless so the file lines up byte-for-byte
// with the output of `rtl_fm -M fm` (also headerless S16_LE) for decoder
// diffing; replay/inspect with `ffplay -f s16le -ar 48000 -ac N <file>`.
// warmup_ms: sleep after fork so arecord's DMA/CODEC startup transient
// (first ~100 ms of noise + silence) is past before PTT fires.
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

// Build an auto-record filename "<tool>_UT=YYYYMMDDTHHMMSS.sss.raw" in the
// current working directory. Timestamp is UTC at the instant of generation.
// Returns 0 on success, -1 on overflow.
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

int main(int argc, char **argv)
{
    const char *radio_device = NULL;     // explicit --radio-device=, NULL if absent
    int radio_speed_bps = -1;            // explicit --radio-serial-speed=, < 0 if absent
    const char *audio_device = NULL;
    char audio_device_buf[64] = {0};     // backing store for autodetect result
    int audio_pick = 1;  // 0=explicit --audio-device, 1=signalink, 2=radio
    const char *record_path = NULL;
    char auto_record_path[256];
    int no_record = 0;
    double freq_hz = FRONTIERSAT_CARRIER_HZ;
    double tone_hz = 1000.0;
    double tone_stop_hz = -1.0;  // < 0 = single-tone mode
    double tone_step_hz = 500.0;
    double duration_s = 3.0;
    double amplitude = 0.3;
    int data_mod_source = -1;  // < 0 = leave whatever radio_uplink_prep set
    int moni_level_pct = -1;  // < 0 = don't touch
    int uplink_mod_level = -1;  // < 0 = don't touch (% 0..100)
    int tx_power_pct = -1;  // < 0 = don't touch (% 0..100)
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
        } else if (strncmp("--tone-hz=", argv[i], 10) == 0) {
            if (strlen(argv[i]) < 11) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            tone_hz = atof(argv[i] + 10);
        } else if (strncmp("--tone-stop-hz=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            tone_stop_hz = atof(argv[i] + 15);
        } else if (strncmp("--tone-step-hz=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            tone_step_hz = atof(argv[i] + 15);
            if (tone_step_hz <= 0) {
                fprintf(stderr, "--tone-step-hz must be > 0\n");
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
        } else if (strncmp("--duration-s=", argv[i], 13) == 0) {
            if (strlen(argv[i]) < 14) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            duration_s = atof(argv[i] + 13);
        } else if (strncmp("--amplitude=", argv[i], 12) == 0) {
            if (strlen(argv[i]) < 13) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            amplitude = atof(argv[i] + 12);
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
            usage(stdout, argv[0], 0);
            return 0;
        } else if (strcmp("--help-full", argv[i]) == 0) {
            usage(stdout, argv[0], 1);
            return 0;
        } else {
            fprintf(stderr, "Unable to parse option '%s'\n", argv[i]);
            usage(stderr, argv[0], 0);
            return EXIT_FAILURE;
        }
    }

    // Audio device resolution: explicit --audio-device= wins; otherwise
    // autodetect based on --signalink-audio (default) or --radio-audio
    // by reading /proc/asound/cards. Wrong-device-of-the-day mistakes
    // are how we lost most of yesterday — make the auto path the default.
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
                        "found in /proc/asound/cards (rc=%d)\n", rc);
                return EXIT_FAILURE;
            }
            // --radio-audio implies USB CODEC routing on the radio.
            if (data_mod_source < 0) data_mod_source = RADIO_DATA_MOD_SRC_USB;
        } else {
            rc = audio_find_signalink_device(audio_device_buf,
                                             sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "error: --signalink-audio: SignaLink not "
                        "found in /proc/asound/cards (rc=%d). Pass "
                        "--audio-device= explicitly or --radio-audio for the "
                        "rig's native USB CODEC.\n", rc);
                return EXIT_FAILURE;
            }
            // --signalink-audio implies rear DATA jack routing.
            if (data_mod_source < 0) data_mod_source = RADIO_DATA_MOD_SRC_ACC;
        }
        audio_device = audio_device_buf;
        fprintf(stderr, "tx_tone: auto-detected audio device %s\n", audio_device);
    }

    // Resolve device + speed: explicit flag wins, else stored default,
    // else compiled-in fallbacks. Same precedence radio_ctl uses.
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
    // radio_init leaves Main in plain FM (CI-V 0x06 clears the DATA flag).
    // radio_uplink_prep sets FM + DATA on + DATA MOD source = USB so audio
    // from our playback actually reaches the modulator. Without DATA MOD
    // forced to USB, a front-panel misconfig causes a clean carrier with
    // no deviation — the TX spectrum shows an unmodulated CW line.
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
        // Percent -> 0..255 scale (IC-9700 monitor-level range). MONI
        // ON/OFF is still a front-panel toggle.
        int raw = (int)((moni_level_pct * 255 + 50) / 100);
        rc = radio_set_moni_level(&g_radio, raw);
        if (rc != RADIO_OK) {
            fprintf(stderr, "warning: could not set MONI level (rc=%d)\n", rc);
        }
    }

    if (tone_stop_hz > 0) {
        fprintf(stderr, "tx_tone: %.6f MHz FM simplex, sweep %.1f -> %.1f Hz step %.1f, %.2f s/step, amp %.2f, audio %s, ptt %s\n",
                freq_hz / 1e6, tone_hz, tone_stop_hz, tone_step_hz, duration_s, amplitude,
                audio_device, g_no_ptt ? "off" : "on");
    } else {
        fprintf(stderr, "tx_tone: %.6f MHz FM simplex, tone %.1f Hz, %.2f s, amp %.2f, audio %s, ptt %s\n",
                freq_hz / 1e6, tone_hz, duration_s, amplitude, audio_device, g_no_ptt ? "off" : "on");
    }

    // Auto-generate a record path if recording is on and the user didn't
    // pass --record=<path>. Writes tx_tone_UT=YYYYMMDDTHHMMSS.sss.raw in
    // the current working directory so every run leaves a searchable
    // breadcrumb without the operator having to remember a filename.
    if (record_path == NULL && !no_record) {
        if (make_auto_record_path("tx_tone", auto_record_path,
                                  sizeof auto_record_path) != 0) {
            fprintf(stderr, "warning: could not build auto-record filename; "
                    "recording disabled for this run\n");
        } else {
            record_path = auto_record_path;
        }
    }

    // Start the recorder first so arecord owns the device at open time.
    // Opening playback afterwards won't reset the capture clock the way
    // the reverse order can (PCM2901 behaves oddly with playback-first
    // opens, producing level glitches mid-capture).
    if (record_path != NULL) {
        fprintf(stderr, "tx_tone: recording %s <- %s (warmup %d ms)\n",
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

    if (!g_no_ptt) {
        rc = radio_ptt(&g_radio, 1);
        if (rc != RADIO_OK) {
            fprintf(stderr, "error: PTT on (rc=%d)\n", rc);
            stop_recorder();
            audio_playback_close(playback);
            goto fail_radio;
        }
        usleep(100000);
    }

    if (tone_stop_hz > 0) {
        // Stepped sweep: ascending if stop > start, descending otherwise.
        // Inclusive of stop frequency (stops once next step would overshoot).
        double step = (tone_stop_hz >= tone_hz) ? tone_step_hz : -tone_step_hz;
        double f = tone_hz;
        arc = AUDIO_OK;
        while ((step > 0 && f <= tone_stop_hz + 1e-6) ||
               (step < 0 && f >= tone_stop_hz - 1e-6)) {
            fprintf(stderr, "tx_tone: step %.1f Hz\n", f);
            arc = audio_play_tone(playback, f, amplitude, duration_s,
                                  AUDIO_RATE_HZ, AUDIO_CHANNELS);
            if (arc != AUDIO_OK) break;
            f += step;
        }
    } else {
        arc = audio_play_tone(playback, tone_hz, amplitude, duration_s, AUDIO_RATE_HZ, AUDIO_CHANNELS);
    }
    audio_playback_close(playback);

    if (!g_no_ptt) {
        // Tail: capture the receiver's post-TX recovery before stopping.
        usleep(200000);
        radio_ptt(&g_radio, 0);
    }
    stop_recorder();
    radio_disconnect(&g_radio);

    if (arc != AUDIO_OK) {
        fprintf(stderr, "error: tone generation failed (rc=%d)\n", arc);
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
