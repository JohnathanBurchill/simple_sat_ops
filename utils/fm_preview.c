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

#include "argparse.h"
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

// Parsed command-line configuration. parse_args() fills this; main() copies
// the fields out into working locals so the modulation body is unchanged.
typedef struct {
    const char *in_path;
    const char *out_path;
    double carrier;
    double deviation;
    int stretch;
} fm_preview_args_t;

// Option column width: the widest label below ("--time-stretch=<N>") + a small
// margin. See src/cli/argparse.h for the parse_args convention.
#define OPTW 20

// Parse argv into *a (help == 0), or print one right-aligned help line per
// option and return (help != 0). Each option is one self-contained block whose
// test carries "|| help", so help mode falls through and prints them all.
static int parse_args(fm_preview_args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (starts_with(arg, "--in=") || help) {
            if (help) parse_help_line(OPTW, "--in=<in.wav>", "input baseband WAV (16-bit PCM mono)");
            else a->in_path = arg + 5;
            matched = 1;
        }
        if (starts_with(arg, "--out=") || help) {
            if (help) parse_help_line(OPTW, "--out=<out.wav>", "output FM-modulated WAV (same rate as input)");
            else a->out_path = arg + 6;
            matched = 1;
        }
        if (starts_with(arg, "--carrier=") || help) {
            if (help) parse_help_line(OPTW, "--carrier=<Hz>", "audio carrier (default 1000)");
            else a->carrier = atof(arg + 10);
            matched = 1;
        }
        if (starts_with(arg, "--deviation=") || help) {
            if (help) parse_help_line(OPTW, "--deviation=<Hz>", "peak frequency deviation (default 500)");
            else a->deviation = atof(arg + 12);
            matched = 1;
        }
        if (starts_with(arg, "--time-stretch=") || help) {
            if (help) parse_help_line(OPTW, "--time-stretch=<N>", "stretch output duration by integer factor N at the same pitch (default 1)");
            else a->stretch = atoi(arg + 15);
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "unknown option: %s\n", arg);
            return PARSE_ERROR;
        }
    }
    return PARSE_OK;
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "fm_preview")) return 0;
    fm_preview_args_t cfg = {
        .in_path = NULL,
        .out_path = NULL,
        .carrier = 1000.0,
        .deviation = 500.0,
        .stretch = 1,
    };
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }
    const char *in_path = cfg.in_path;
    const char *out_path = cfg.out_path;
    double carrier = cfg.carrier;
    double deviation = cfg.deviation;
    int stretch = cfg.stretch;

    if (in_path == NULL || out_path == NULL) {
        fprintf(stderr, "missing --in or --out\n");
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
