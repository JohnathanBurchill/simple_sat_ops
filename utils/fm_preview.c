/*

    Simple Satellite Operations  utils/fm_preview.c

    Converts a baseband WAV (e.g. from uplink_test) into an audible
    FM-modulated WAV by integrating an instantaneous frequency
    (carrier + deviation * sample) and emitting cos(phase). Same
    operation an FM receiver does to the AX100 signal on-air — makes
    the bit pattern audible as a two-tone warble.

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

#include "modem.h"
#include "utils/wav_read.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static void usage(FILE *out, const char *argv0)
{
    fprintf(out,
        "usage: %s --in=<in.wav> --out=<out.wav> [options]\n"
        "\n"
        "FM-modulate a baseband WAV into an audible preview:\n"
        "  phase += 2π · (carrier + deviation · sample) / samp_rate\n"
        "  out    = cos(phase)\n"
        "Input must be 16-bit PCM mono; output is written at the same rate.\n"
        "\n"
        "Options:\n"
        "  --carrier=<Hz>       Audio carrier (default 1000)\n"
        "  --deviation=<Hz>     Peak frequency deviation (default 500)\n"
        "  --time-stretch=<N>   Stretch output duration by integer factor N\n"
        "                       while keeping the carrier pitch unchanged\n"
        "                       (default 1, no stretch). Each input baseband\n"
        "                       sample drives N output samples of cosine at\n"
        "                       the instantaneous frequency, so the carrier\n"
        "                       stays at `carrier` Hz and the bit pattern\n"
        "                       plays N× slower. For bit-by-bit aural inspection.\n"
        "  --help               This message\n",
        argv0);
}

int main(int argc, char **argv)
{
    const char *in_path = NULL;
    const char *out_path = NULL;
    double carrier = 1000.0;
    double deviation = 500.0;
    int stretch = 1;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if (strcmp(a, "--help") == 0) { usage(stdout, argv[0]); return 0; }
        else if (starts_with(a, "--in="))        in_path   = a + 5;
        else if (starts_with(a, "--out="))       out_path  = a + 6;
        else if (starts_with(a, "--carrier="))   carrier   = atof(a + 10);
        else if (starts_with(a, "--deviation=")) deviation = atof(a + 12);
        else if (starts_with(a, "--time-stretch=")) stretch   = atoi(a + 15);
        else {
            fprintf(stderr, "unknown option: %s\n", a);
            usage(stderr, argv[0]);
            return 1;
        }
    }
    if (in_path == NULL || out_path == NULL) {
        fprintf(stderr, "missing --in or --out\n");
        usage(stderr, argv[0]);
        return 1;
    }
    if (!(carrier > 0.0) || !(deviation >= 0.0)) {
        fprintf(stderr, "--carrier must be > 0, --deviation must be >= 0\n");
        return 1;
    }
    if (stretch < 1) {
        fprintf(stderr, "--stretch must be >= 1\n");
        return 1;
    }

    int16_t *bb = NULL;
    size_t n = 0;
    int samp_rate = 0, channels = 0;
    if (wav_read_pcm16(in_path, &bb, &n, &samp_rate, &channels) != 0) {
        return 1;
    }
    if (channels != 1) {
        fprintf(stderr,
                "fm_preview: only mono input supported (got %d channels)\n",
                channels);
        free(bb);
        return 1;
    }
    if (n == 0) {
        fprintf(stderr, "fm_preview: input has no samples\n");
        free(bb);
        return 1;
    }

    size_t n_out = n * (size_t)stretch;
    int16_t *out = (int16_t *)malloc(n_out * sizeof(int16_t));
    if (out == NULL) {
        fprintf(stderr, "fm_preview: out of memory for %zu samples\n", n_out);
        free(bb);
        return 1;
    }

    double phase = 0.0;
    double two_pi_over_fs = 2.0 * M_PI / (double)samp_rate;
    size_t o = 0;
    for (size_t i = 0; i < n; ++i) {
        double x = (double)bb[i] / 32768.0;
        double inst_freq = carrier + deviation * x;
        double step = two_pi_over_fs * inst_freq;
        for (int k = 0; k < stretch; ++k) {
            phase += step;
            double y = cos(phase);
            long v = lrint(y * 32767.0);
            if (v > 32767)      v = 32767;
            else if (v < -32768) v = -32768;
            out[o++] = (int16_t)v;
        }
    }

    int rc = pcm16_write_wav(out_path, out, n_out, samp_rate);
    free(out);
    free(bb);
    if (rc != 0) return 1;

    fprintf(stderr,
            "fm_preview: wrote %s (%zu samples @ %d Hz, carrier=%.1f, deviation=%.1f, stretch=%d)\n",
            out_path, n_out, samp_rate, carrier, deviation, stretch);
    return 0;
}
