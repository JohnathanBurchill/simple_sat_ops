/*

    Simple Satellite Operations  utils/ham_speak.c

    Record a short narrowband-FM voice message from the microphone, then
    transmit it on a UHF amateur frequency (default 436.15 MHz). Run it
    before a satellite ops session to announce that you're about to start
    ("...please stand by"), and again afterward to say you've finished
    ("...enjoy the airwaves") — the talk half of the pre-/post-ops
    courtesy ritual (ham_listen is the listen half).

    Record-then-transmit: you speak, press Ctrl-C, and the whole clip
    goes out as ONE continuous FM burst through the existing half-duplex
    burst path (b210_rx_tx_core_burst). No live streaming, so no changes
    to the transmitter backend. Identify by voice with your callsign as
    licence requires; this tool sends no automatic station ID.

    TX is inhibited unless --allow-tx is given. --dump-iq renders the IQ
    to a file instead of keying, for a no-RF dry run (works without a
    transmit-capable SDR, e.g. on a dev host).

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

#include "argparse.h"
#include "audio_io.h"
#include "b210_rx_tx_core.h"
#include "carrier_trim.h"
#include "fm_mod.h"
#include "frontiersat.h"
#include "resample.h"

#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Mic capture rate; the SDR transmits at TX_RATE, so audio is upsampled
// by TX_RATE / MIC_RATE before FM modulation.
#define MIC_RATE  48000
#define TX_RATE   480000
#define UPSAMPLE  (TX_RATE / MIC_RATE)   // 10

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void) sig; g_stop = 1; }

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

typedef struct {
    double      freq_mhz;
    double      tx_level_db;
    double      deviation_hz;
    int         preemphasis;     // 1 = apply pre-emphasis high-shelf
    double      max_talk_s;
    int         review;
    int         allow_tx;
    const char *dump_iq;
    const char *sdr_type;
    const char *uhd_args;
    const char *fpga_path;
    int         device_index;
} hs_args_t;

// Option column width: widest label below ("--no-preemphasis") + margin.
#define OPTW 22

static int parse_args(hs_args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0 || help) {
            if (help) parse_help_line(OPTW, "-h, --help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (starts_with(arg, "--freq-mhz=") || help) {
            if (help) parse_help_line(OPTW, "--freq-mhz=<f>", "TX frequency in MHz (default 436.15)");
            else a->freq_mhz = atof(arg + 11);
            matched = 1;
        }
        if (starts_with(arg, "--tx-level=") || help) {
            if (help) parse_help_line(OPTW, "--tx-level=<dB>", "TX gain in dB (default 50)");
            else a->tx_level_db = atof(arg + 11);
            matched = 1;
        }
        if (starts_with(arg, "--deviation-hz=") || help) {
            if (help) parse_help_line(OPTW, "--deviation-hz=<hz>", "NBFM peak deviation (default 5000)");
            else a->deviation_hz = atof(arg + 15);
            matched = 1;
        }
        if (strcmp(arg, "--no-preemphasis") == 0 || help) {
            if (help) parse_help_line(OPTW, "--no-preemphasis", "disable the TX pre-emphasis high-shelf");
            else a->preemphasis = 0;
            matched = 1;
        }
        if (starts_with(arg, "--max-talk-s=") || help) {
            if (help) parse_help_line(OPTW, "--max-talk-s=<s>", "max recording length, seconds (default 60)");
            else a->max_talk_s = atof(arg + 13);
            matched = 1;
        }
        if (strcmp(arg, "--review") == 0 || help) {
            if (help) parse_help_line(OPTW, "--review", "play the recording back and confirm before TX");
            else a->review = 1;
            matched = 1;
        }
        if (strcmp(arg, "--allow-tx") == 0 || help) {
            if (help) parse_help_line(OPTW, "--allow-tx", "required to actually key the transmitter");
            else a->allow_tx = 1;
            matched = 1;
        }
        if (starts_with(arg, "--dump-iq=") || help) {
            if (help) parse_help_line(OPTW, "--dump-iq=<path>", "render TX IQ to a file instead of keying (dry run)");
            else a->dump_iq = arg + 10;
            matched = 1;
        }
        if (starts_with(arg, "--sdr-type=") || help) {
            if (help) parse_help_line(OPTW, "--sdr-type=<t>", "uhd | rtlsdr | auto (default auto)");
            else a->sdr_type = arg + 11;
            matched = 1;
        }
        if (starts_with(arg, "--uhd-args=") || help) {
            if (help) parse_help_line(OPTW, "--uhd-args=<str>", "verbatim UHD device args (escape hatch)");
            else a->uhd_args = arg + 11;
            matched = 1;
        }
        if (starts_with(arg, "--sdr-fpga=") || help) {
            if (help) parse_help_line(OPTW, "--sdr-fpga=<path>", "force a UHD FPGA image");
            else a->fpga_path = arg + 11;
            matched = 1;
        }
        if (starts_with(arg, "--sdr-device=") || help) {
            if (help) parse_help_line(OPTW, "--sdr-device=<idx>", "RTL-SDR dongle index (default 0)");
            else a->device_index = atoi(arg + 13);
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "unknown option: %s\n", arg);
            return PARSE_ERROR;
        }
    }
    return PARSE_OK;
}

static sdr_backend_type_t backend_from_str(const char *s)
{
    if (s == NULL) return SDR_TYPE_AUTO;
    if (strcmp(s, "uhd") == 0)    return SDR_TYPE_UHD;
    if (strcmp(s, "rtlsdr") == 0) return SDR_TYPE_RTLSDR;
    return SDR_TYPE_AUTO;
}

// Record mono mic audio until Ctrl-C or max_frames. Returns a malloc'd
// int16 buffer (caller frees) and the frame count via *out_n; NULL on
// error or if nothing was recorded.
static int16_t *record_mic(size_t max_frames, size_t *out_n)
{
    *out_n = 0;
    audio_capture_t *cap = audio_capture_open(MIC_RATE, 1);
    if (cap == NULL) {
        fprintf(stderr, "ham_speak: could not open the microphone\n");
        return NULL;
    }

    size_t   cap_frames = (size_t) MIC_RATE * 5;   // grow from ~5 s
    if (cap_frames > max_frames) cap_frames = max_frames;
    int16_t *buf = malloc(cap_frames * sizeof(int16_t));
    if (buf == NULL) { audio_capture_close(cap); return NULL; }

    size_t  n = 0;
    int16_t block[4096];
    long    status_acc = 0;

    fprintf(stderr, "ham_speak: recording — speak now, Ctrl-C to finish.\n");
    while (!g_stop && n < max_frames) {
        ssize_t got = audio_capture_read(cap, block,
                                         sizeof block / sizeof block[0]);
        if (got < 0) break;
        if (got == 0) {
            struct timespec ts = { 0, 10 * 1000000L };
            nanosleep(&ts, NULL);
            continue;
        }
        if (n + (size_t) got > max_frames) got = (ssize_t)(max_frames - n);

        if (n + (size_t) got > cap_frames) {
            size_t want = cap_frames * 2;
            if (want < n + (size_t) got) want = n + (size_t) got;
            if (want > max_frames) want = max_frames;
            int16_t *grown = realloc(buf, want * sizeof(int16_t));
            if (grown == NULL) break;
            buf = grown;
            cap_frames = want;
        }

        int16_t peak = 0;
        for (ssize_t i = 0; i < got; ++i) {
            int16_t s = block[i];
            int16_t a = (int16_t)(s < 0 ? -s : s);
            if (a > peak) peak = a;
        }
        memcpy(buf + n, block, (size_t) got * sizeof(int16_t));
        n += (size_t) got;

        status_acc += got;
        if (status_acc >= MIC_RATE / 4) {
            status_acc = 0;
            int bars = peak / 1000;
            if (bars > 30) bars = 30;
            char meter[32];
            memset(meter, '#', (size_t) bars);
            meter[bars] = '\0';
            fprintf(stderr, "\r  %5.1f s  [%-30s]", (double) n / MIC_RATE, meter);
            fflush(stderr);
        }
    }
    fprintf(stderr, "\n");
    audio_capture_close(cap);

    if (n == 0) { free(buf); return NULL; }
    *out_n = n;
    return buf;
}

// In-place pre-emphasis (high-shelf: y = x + k*(x - lpf(x))) and peak
// normalize to ~0.7 full scale, via a float scratch pass so the
// emphasis boost can't clip before the level is set.
static int shape_audio(int16_t *pcm, size_t n, int preemphasis)
{
    float *f = malloc(n * sizeof(float));
    if (f == NULL) return -1;

    const double a = 1.0 - exp(-2.0 * M_PI * 2000.0 / (double) MIC_RATE);
    const double k = 1.0;  // +6 dB high-shelf
    double lp = 0.0, peak = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double x = (double) pcm[i];
        if (preemphasis) {
            lp += a * (x - lp);
            x = x + k * (x - lp);
        }
        f[i] = (float) x;
        double mag = fabs(x);
        if (mag > peak) peak = mag;
    }

    double scale = (peak > 1.0) ? (0.7 * 32767.0 / peak) : 1.0;
    for (size_t i = 0; i < n; ++i) {
        double v = (double) f[i] * scale;
        if (v >  32767.0) v =  32767.0;
        if (v < -32768.0) v = -32768.0;
        pcm[i] = (int16_t) lround(v);
    }
    free(f);
    return 0;
}

#include "sso_version.h"

int main(int argc, char *argv[])
{
    if (sso_version_handle(argc, argv, "ham_speak")) return 0;

    hs_args_t cfg = {
        .freq_mhz     = FRONTIERSAT_CARRIER_HZ / 1e6,
        .tx_level_db  = 50.0,
        .deviation_hz = 5000.0,
        .preemphasis  = 1,
        .max_talk_s   = 60.0,
        .review       = 0,
        .allow_tx     = 0,
        .dump_iq      = NULL,
        .sdr_type     = "auto",
        .uhd_args     = NULL,
        .fpga_path    = NULL,
        .device_index = 0,
    };
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 2;
    }

    int dry_run = (cfg.dump_iq != NULL);

    // TX-inhibit gate: refuse to key the transmitter without explicit
    // consent. A --dump-iq dry run never keys, so it's exempt.
    if (!dry_run && !cfg.allow_tx) {
        fprintf(stderr,
                "ham_speak: TX inhibited. Re-run with --allow-tx to key the "
                "transmitter, or --dump-iq=<path> for a no-RF dry run.\n");
        return 1;
    }
    if (cfg.max_talk_s <= 0.0 || cfg.max_talk_s > 600.0) {
        fprintf(stderr, "ham_speak: --max-talk-s must be in (0, 600]\n");
        return 2;
    }

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);
    setenv("UHD_LOG_CONSOLE_LEVEL", "error", 0);

    // --- Open the SDR up front (skipped for a dry run) so we fail fast
    //     on a missing or RX-only device, before recording. ---
    b210_rx_tx_core_t *core = NULL;
    if (!dry_run) {
        double trim_hz = carrier_trim_load_hz();
        b210_rx_tx_core_params_t p = {
            .freq_hz                = cfg.freq_mhz * 1e6,
            .rate_hz                = 480000.0,
            .gain_db                = 30.0,        // RX gain, unused here
            .bw_hz                  = -1.0,
            .fm_fullscale_hz        = 25000.0,
            .device_args            = NULL,
            .rx_antenna             = "RX2",
            .decim_factor           = 5u,
            .decim_cutoff_hz        = 0.0,
            .decim_taps             = 0u,
            .fm_lo_compensation_hz  = 0.0,
            .rx_dc_offset_track     = 1,
            .rx_iq_balance_track    = 0,
            .carrier_trim_hz        = trim_hz,
            .backend_type           = backend_from_str(cfg.sdr_type),
            .uhd_args_override      = cfg.uhd_args,
            .fpga_image_path        = cfg.fpga_path,
            .device_index           = cfg.device_index,
        };
        if (b210_rx_tx_core_open(&p, &core) != 0) {
            fprintf(stderr, "ham_speak: could not open an SDR\n");
            return 1;
        }
        if (!b210_rx_tx_core_can_tx(core)) {
            fprintf(stderr,
                    "ham_speak: %s is receive-only — it cannot transmit. "
                    "ham_speak needs a transmit-capable SDR (UHD/B210).\n",
                    b210_rx_tx_core_sdr_name(core));
            b210_rx_tx_core_close(core);
            return 1;
        }
    }

    // --- Record the mic. ---
    size_t   max_frames = (size_t)(cfg.max_talk_s * MIC_RATE);
    size_t   n_mic = 0;
    int16_t *mic = record_mic(max_frames, &n_mic);
    if (mic == NULL) {
        fprintf(stderr, "ham_speak: nothing recorded; aborting.\n");
        if (core) b210_rx_tx_core_close(core);
        return 1;
    }
    fprintf(stderr, "ham_speak: recorded %.1f s\n", (double) n_mic / MIC_RATE);

    // --- Optional review: play it back and confirm. ---
    if (cfg.review) {
        audio_play_t *pb = audio_play_open(MIC_RATE, 1);
        if (pb != NULL) {
            audio_play_write(pb, mic, n_mic);
            audio_play_close(pb);
        }
        fprintf(stderr, "Transmit this message? [y/N] ");
        int c = getchar();
        if (c != 'y' && c != 'Y') {
            fprintf(stderr, "ham_speak: cancelled.\n");
            free(mic);
            if (core) b210_rx_tx_core_close(core);
            return 0;
        }
    }

    // --- Shape + normalize, upsample x10, FM-modulate, ramp. ---
    if (shape_audio(mic, n_mic, cfg.preemphasis) != 0) {
        fprintf(stderr, "ham_speak: out of memory shaping audio\n");
        free(mic); if (core) b210_rx_tx_core_close(core); return 1;
    }

    size_t   n_tx   = n_mic * (size_t) UPSAMPLE;
    int16_t *pcm_up = malloc(n_tx * sizeof(int16_t));
    if (pcm_up == NULL) {
        fprintf(stderr, "ham_speak: out of memory (upsample)\n");
        free(mic); if (core) b210_rx_tx_core_close(core); return 1;
    }
    size_t got = resample_up_linear(mic, n_mic, UPSAMPLE, pcm_up, n_tx);
    free(mic);
    n_tx = got;

    int16_t *iq = malloc(n_tx * 2 * sizeof(int16_t));
    if (iq == NULL) {
        fprintf(stderr, "ham_speak: out of memory (IQ)\n");
        free(pcm_up); if (core) b210_rx_tx_core_close(core); return 1;
    }
    fm_mod_t fm;
    fm_mod_init(&fm);
    fm_mod_block(&fm, pcm_up, n_tx, cfg.deviation_hz, (double) TX_RATE, iq);
    free(pcm_up);
    fm_apply_ramp(iq, n_tx, (size_t)(0.005 * TX_RATE));   // 5 ms key ramp

    // --- Dry run: dump IQ and exit (no device, no RF). ---
    if (dry_run) {
        FILE *fp = fopen(cfg.dump_iq, "wb");
        if (fp == NULL) {
            fprintf(stderr, "ham_speak: cannot open %s\n", cfg.dump_iq);
            free(iq); return 1;
        }
        size_t wrote = fwrite(iq, sizeof(int16_t), n_tx * 2, fp);
        fclose(fp);
        free(iq);
        fprintf(stderr,
                "ham_speak: wrote %zu IQ samples (%.1f s @ %d Hz) to %s\n",
                wrote / 2, (double) n_tx / TX_RATE, TX_RATE, cfg.dump_iq);
        return 0;
    }

    // --- Transmit one continuous burst. ---
    fprintf(stderr, "ham_speak: transmitting %.1f s on %.4f MHz at %.0f dB...\n",
            (double) n_tx / TX_RATE, cfg.freq_mhz, cfg.tx_level_db);
    b210_rx_tx_core_burst_params_t bp = {
        .iq                = iq,
        .n_samps           = n_tx,
        .tx_rate_hz        = (double) TX_RATE,
        .tx_freq_hz        = cfg.freq_mhz * 1e6,
        .tx_gain_db        = cfg.tx_level_db,
        .start_delay_s     = 0.5,
        .rx_resume_freq_hz = cfg.freq_mhz * 1e6,
    };
    int rc = b210_rx_tx_core_burst(core, &bp);
    free(iq);
    b210_rx_tx_core_close(core);
    if (rc != 0) {
        fprintf(stderr, "ham_speak: transmit failed\n");
        return 1;
    }
    fprintf(stderr, "ham_speak: done.\n");
    return 0;
}
