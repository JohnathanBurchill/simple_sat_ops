/*

    Simple Satellite Operations  rx_capture.c

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

// Receive-side counterpart of tx_tone / tx_white_noise: capture the
// SignaLink (default) or radio's native USB CODEC into a WAV in CWD and
// auto-render a 1920x1080 spectrogram PNG. No radio dispatcher — this
// tool is purely audio-side. Configure the radio (frequency, mode, etc.)
// separately with radio_ctl, then point rx_capture at the SignaLink to
// see what's coming out of the rear DATA-OUT jack, or at the radio's
// native USB CODEC to see the AF demod.

#include "audio.h"

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void usage(FILE *dest, const char *name)
{
    fprintf(dest,
        "usage: %s [options]\n"
        "\n"
        "Capture audio from the SignaLink (default) or radio's native USB\n"
        "CODEC, write a WAV in CWD, and render a 1920x1080 spectrogram PNG.\n"
        "Mirrors tx_tone / tx_white_noise on the receive side: feed the\n"
        "off-air RX (or SDR-via-radio audio) into the SignaLink and read\n"
        "the bandshape off the resulting spectrogram.\n"
        "\n"
        "Audio source:\n"
        "  --signalink-audio        (default) Auto-detect the SignaLink USB\n"
        "                           (TI Burr-Brown PCM2901/PCM2904) and\n"
        "                           capture from its plughw:N,0.\n"
        "  --radio-audio            Auto-detect the radio's native USB\n"
        "                           CODEC (Yaesu/ICOM) and capture from\n"
        "                           its plughw:N,0.\n"
        "  --audio-device=<name>    Explicit ALSA PCM device, overriding\n"
        "                           either auto-detect (e.g. plughw:1,0).\n"
        "  --backend-hint=<id>      yaesu|icom — only matters with\n"
        "                           --radio-audio. Default: try both.\n"
        "\n"
        "Capture:\n"
        "  --duration-s=<seconds>   Capture duration (default 10.0). Pass\n"
        "                           0 to capture until Ctrl-C; either way\n"
        "                           Ctrl-C cleanly truncates the WAV at\n"
        "                           the last chunk read.\n"
        "  --output=<path>          WAV path (default: auto-named\n"
        "                           rx_capture_UT=YYYYMMDDTHHMMSS.sss.wav\n"
        "                           in CWD).\n"
        "  --no-spectrogram         Skip the ffmpeg spectrogram step.\n"
        "  --help                   Show this help.\n",
        name);
}

static int make_auto_wav_path(char *out, size_t out_size)
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) return -1;
    struct tm utc;
    if (gmtime_r(&tv.tv_sec, &utc) == NULL) return -1;
    int n = snprintf(out, out_size,
                     "rx_capture_UT=%04d%02d%02dT%02d%02d%02d.%03ld.wav",
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec,
                     (long)(tv.tv_usec / 1000));
    return (n > 0 && (size_t)n < out_size) ? 0 : -1;
}

int main(int argc, char **argv)
{
    const char *audio_device = NULL;
    char audio_device_buf[64] = {0};
    int audio_pick = 1;  // 0=explicit, 1=signalink (default), 2=radio
    const char *backend_hint = NULL;
    const char *output_path = NULL;
    char auto_path[256];
    double duration_s = 10.0;
    int do_spectrogram = 1;

    for (int i = 1; i < argc; i++) {
        if (strncmp("--audio-device=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            audio_device = argv[i] + 15;
            audio_pick = 0;
        } else if (strcmp("--signalink-audio", argv[i]) == 0) {
            audio_pick = 1;
        } else if (strcmp("--radio-audio", argv[i]) == 0) {
            audio_pick = 2;
        } else if (strncmp("--backend-hint=", argv[i], 15) == 0) {
            if (strlen(argv[i]) < 16) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            backend_hint = argv[i] + 15;
        } else if (strncmp("--duration-s=", argv[i], 13) == 0) {
            if (strlen(argv[i]) < 14) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            duration_s = atof(argv[i] + 13);
            if (duration_s < 0.0) duration_s = 0.0;
        } else if (strncmp("--output=", argv[i], 9) == 0) {
            if (strlen(argv[i]) < 10) { fprintf(stderr, "Unable to parse %s\n", argv[i]); return EXIT_FAILURE; }
            output_path = argv[i] + 9;
        } else if (strcmp("--no-spectrogram", argv[i]) == 0) {
            do_spectrogram = 0;
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
        int rc;
        if (audio_pick == 2) {
            rc = audio_find_radio_device(backend_hint, audio_device_buf,
                                         sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "error: --radio-audio: no matching ALSA "
                        "card found (rc=%d)\n", rc);
                return EXIT_FAILURE;
            }
        } else {
            rc = audio_find_signalink_device(audio_device_buf,
                                              sizeof audio_device_buf);
            if (rc != 0) {
                fprintf(stderr, "error: --signalink-audio: SignaLink not "
                        "found (rc=%d). Pass --audio-device= or "
                        "--radio-audio.\n", rc);
                return EXIT_FAILURE;
            }
        }
        audio_device = audio_device_buf;
        fprintf(stderr, "rx_capture: auto-detected audio device %s\n",
                audio_device);
    }

    if (output_path == NULL) {
        if (make_auto_wav_path(auto_path, sizeof auto_path) != 0) {
            fprintf(stderr, "error: could not build auto WAV filename\n");
            return EXIT_FAILURE;
        }
        output_path = auto_path;
    }

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    snd_pcm_t *capture = NULL;
    int arc = audio_capture_open(&capture, audio_device,
                                 AUDIO_RATE_HZ, AUDIO_CHANNELS);
    if (arc != AUDIO_OK || capture == NULL) {
        fprintf(stderr, "error: cannot open ALSA capture device '%s' "
                "(rc=%d)\n", audio_device, arc);
        return EXIT_FAILURE;
    }

    audio_wav_writer_t *wav = audio_wav_writer_open(output_path,
                                                    AUDIO_RATE_HZ,
                                                    AUDIO_CHANNELS);
    if (wav == NULL) {
        fprintf(stderr, "error: could not open WAV %s\n", output_path);
        audio_capture_close(capture);
        return EXIT_FAILURE;
    }

    if (duration_s > 0.0) {
        fprintf(stderr, "rx_capture: %.2f s from %s -> %s\n",
                duration_s, audio_device, output_path);
    } else {
        fprintf(stderr, "rx_capture: open-ended from %s -> %s "
                "(Ctrl-C to stop)\n", audio_device, output_path);
    }

    arc = audio_capture_to_wav(capture, wav, duration_s,
                               AUDIO_RATE_HZ, AUDIO_CHANNELS, &g_stop);
    audio_wav_writer_close(wav);
    audio_capture_close(capture);

    if (arc != AUDIO_OK) {
        fprintf(stderr, "error: capture failed (rc=%d)\n", arc);
        return EXIT_FAILURE;
    }

    if (g_stop) {
        fprintf(stderr, "rx_capture: stopped on signal\n");
    }

    if (do_spectrogram) {
        audio_generate_spectrogram(output_path);
    }

    return 0;
}
