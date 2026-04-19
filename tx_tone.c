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
#include "audio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

static radio_t g_radio = {0};
static int g_no_ptt = 0;

static void on_signal(int sig)
{
    (void)sig;
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
        "usage: %s --audio-device=<alsa> [options]\n"
        "\n"
        "Required:\n"
        "  --audio-device=<name>       ALSA PCM device, e.g. hw:3,0 (run `aplay -l`)\n"
        "\n"
        "Options:\n"
        "  --radio-device=<path>       CI-V serial device (default /dev/ttyUSB1)\n"
        "  --radio-serial-speed=<bps>  Serial speed integer (default 115200)\n"
        "  --freq-hz=<hz>              UHF simplex carrier (default %.0f = %.6f MHz)\n"
        "  --tone-hz=<hz>              Tone frequency (default 1000)\n"
        "  --duration-s=<seconds>      Tone duration (default 3.0)\n"
        "  --amplitude=<0..1>          Amplitude into S16 full-scale (default 0.3)\n"
        "  --no-ptt                    Skip CI-V PTT; play audio only\n"
        "  --help                      This message\n",
        name, FRONTIERSAT_CARRIER_HZ, FRONTIERSAT_CARRIER_HZ / 1e6);
}

int main(int argc, char **argv)
{
    const char *radio_device = "/dev/ttyUSB1";
    speed_t radio_speed = B115200;
    const char *audio_device = NULL;
    double freq_hz = FRONTIERSAT_CARRIER_HZ;
    double tone_hz = 1000.0;
    double duration_s = 3.0;
    double amplitude = 0.3;

    for (int i = 1; i < argc; i++) {
        if (strncmp("--radio-device=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            radio_device = argv[i] + 15;
        } else if (strncmp("--radio-serial-speed=", argv[i], 21) == 0) {
            if (strlen(argv[i]) < 22) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            radio_speed = (speed_t)atoi(argv[i] + 21);
        } else if (strncmp("--audio-device=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            audio_device = argv[i] + 15;
        } else if (strncmp("--freq-hz=", argv[i], 10) == 0) {
            if (strlen(argv[i]) < 11) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            freq_hz = atof(argv[i] + 10);
        } else if (strncmp("--tone-hz=", argv[i], 10) == 0) {
            if (strlen(argv[i]) < 11) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            tone_hz = atof(argv[i] + 10);
        } else if (strncmp("--duration-s=", argv[i], 13) == 0) {
            if (strlen(argv[i]) < 14) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            duration_s = atof(argv[i] + 13);
        } else if (strncmp("--amplitude=", argv[i], 12) == 0) {
            if (strlen(argv[i]) < 13) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            amplitude = atof(argv[i] + 12);
        } else if (strcmp("--no-ptt", argv[i]) == 0) {
            g_no_ptt = 1;
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
        fprintf(stderr, "error: --audio-device is required. Run `aplay -l` on the target to list devices.\n");
        usage(stderr, argv[0]);
        return EXIT_FAILURE;
    }

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    g_radio.device_filename = (char *)radio_device;
    g_radio.serial_speed = radio_speed;
    g_radio.nominal_downlink_frequency = freq_hz;
    g_radio.sub_park_frequency = RADIO_SUB_PARK_HZ;
    int rc = radio_init(&g_radio);
    if (rc != RADIO_OK) {
        fprintf(stderr, "error: radio_init failed (rc=%d)\n", rc);
        if (g_radio.connected) radio_disconnect(&g_radio);
        return EXIT_FAILURE;
    }

    fprintf(stderr, "tx_tone: %.6f MHz FM simplex, tone %.1f Hz, %.2f s, amp %.2f, audio %s, ptt %s\n",
            freq_hz / 1e6, tone_hz, duration_s, amplitude, audio_device, g_no_ptt ? "off" : "on");

    snd_pcm_t *playback = NULL;
    int arc = audio_playback_open(&playback, audio_device, AUDIO_RATE_HZ, AUDIO_CHANNELS);
    if (arc != AUDIO_OK || playback == NULL) {
        fprintf(stderr, "error: cannot open ALSA playback device '%s' (rc=%d)\n", audio_device, arc);
        goto fail_radio;
    }

    if (!g_no_ptt) {
        rc = radio_ptt(&g_radio, 1);
        if (rc != RADIO_OK) {
            fprintf(stderr, "error: PTT on (rc=%d)\n", rc);
            audio_playback_close(playback);
            goto fail_radio;
        }
        usleep(100000);
    }

    arc = audio_play_tone(playback, tone_hz, amplitude, duration_s, AUDIO_RATE_HZ, AUDIO_CHANNELS);
    audio_playback_close(playback);

    if (!g_no_ptt) {
        radio_ptt(&g_radio, 0);
    }
    radio_disconnect(&g_radio);

    if (arc != AUDIO_OK) {
        fprintf(stderr, "error: tone generation failed (rc=%d)\n", arc);
        return EXIT_FAILURE;
    }
    return 0;

fail_radio:
    if (!g_no_ptt && g_radio.connected) {
        radio_ptt(&g_radio, 0);
    }
    radio_disconnect(&g_radio);
    return EXIT_FAILURE;
}
