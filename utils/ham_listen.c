/*

    Simple Satellite Operations  utils/ham_listen.c

    Listen to live narrowband-FM voice on a UHF amateur frequency and
    pipe it to the speakers until Ctrl-C. Run it before a satellite ops
    session to confirm the simplex carrier (default 436.15 MHz) is clear
    before you announce that you're about to start — the listen half of
    the pre-/post-ops courtesy ritual (see ham_speak for the talk half).

    The SDR + FM demodulator are the same b210_rx_tx_core the operational
    receiver uses, so this is just "pump demodulated PCM -> de-emphasis
    -> optional carrier squelch -> soundcard". Works on any compiled-in
    backend (UHD or RTL-SDR; both receive).

    --ogg-stdout swaps the soundcard for an Ogg/Vorbis stream on stdout, so
    a remote ground station can be monitored over ssh:

        ssh rao ham_listen --ogg-stdout | ffplay -nodisp -i -

    (needs libsndfile at build time; Ogg is a streaming container so it
    survives the non-seekable ssh pipe.)

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
#include "frontiersat.h"

#ifdef HAVE_SNDFILE
#include <sndfile.h>
#endif

#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void) sig; g_stop = 1; }

static int starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

typedef struct {
    double      freq_mhz;
    double      gain_db;
    double      fm_fullscale_hz;
    double      deemphasis_hz;   // one-pole audio LPF corner; 0 = off
    double      squelch;         // IQ-magnitude threshold; 0 = off
    const char *rx_antenna;
    const char *sdr_type;
    const char *uhd_args;
    const char *fpga_path;
    int         device_index;
    int         ogg_stdout;   // encode Ogg/Vorbis to stdout instead of speakers
    double      ogg_quality;  // Vorbis VBR quality 0..1 (only with --ogg-stdout)
} hl_args_t;

// Option column width: widest label below ("--fm-fullscale-hz=<hz>") + margin.
#define OPTW 24

static int parse_args(hl_args_t *a, int argc, char **argv, int help)
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
            if (help) parse_help_line(OPTW, "--freq-mhz=<f>", "RX frequency in MHz (default 436.15)");
            else a->freq_mhz = atof(arg + 11);
            matched = 1;
        }
        if (starts_with(arg, "--gain-db=") || help) {
            if (help) parse_help_line(OPTW, "--gain-db=<n>", "RX gain in dB (default 50)");
            else a->gain_db = atof(arg + 10);
            matched = 1;
        }
        if (starts_with(arg, "--fm-fullscale-hz=") || help) {
            if (help) parse_help_line(OPTW, "--fm-fullscale-hz=<hz>", "deviation mapped to full scale (default 5000, NBFM)");
            else a->fm_fullscale_hz = atof(arg + 18);
            matched = 1;
        }
        if (starts_with(arg, "--deemphasis-hz=") || help) {
            if (help) parse_help_line(OPTW, "--deemphasis-hz=<hz>", "audio de-emphasis/LPF corner (default 3000; 0 off)");
            else a->deemphasis_hz = atof(arg + 16);
            matched = 1;
        }
        if (starts_with(arg, "--squelch=") || help) {
            if (help) parse_help_line(OPTW, "--squelch=<level>", "mute below this IQ magnitude (default 0 = off)");
            else a->squelch = atof(arg + 10);
            matched = 1;
        }
        if (strcmp(arg, "--ogg-stdout") == 0 || help) {
            if (help) parse_help_line(OPTW, "--ogg-stdout", "encode Ogg/Vorbis to stdout (no speakers); pipe to ffplay");
            else a->ogg_stdout = 1;
            matched = 1;
        }
        if (starts_with(arg, "--ogg-quality=") || help) {
            if (help) parse_help_line(OPTW, "--ogg-quality=<q>", "Vorbis VBR quality 0..1 for --ogg-stdout (default 0.4)");
            else a->ogg_quality = atof(arg + 14);
            matched = 1;
        }
        if (starts_with(arg, "--antenna=") || help) {
            if (help) parse_help_line(OPTW, "--antenna=<name>", "RX antenna (default RX2)");
            else a->rx_antenna = arg + 10;
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

#include "sso_version.h"

int main(int argc, char *argv[])
{
    if (sso_version_handle(argc, argv, "ham_listen")) return 0;

    hl_args_t cfg = {
        .freq_mhz        = FRONTIERSAT_CARRIER_HZ / 1e6,
        .gain_db         = 50.0,
        .fm_fullscale_hz = 5000.0,
        .deemphasis_hz   = 3000.0,
        .squelch         = 0.0,
        .rx_antenna      = "RX2",
        .sdr_type        = "auto",
        .uhd_args        = NULL,
        .fpga_path       = NULL,
        .device_index    = 0,
        .ogg_stdout      = 0,
        .ogg_quality     = 0.4,
    };
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 2;
    }

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);

    // Keep UHD's INFO banners off the operator's terminal (overwrite=0
    // leaves a deliberately-set debug level alone).
    setenv("UHD_LOG_CONSOLE_LEVEL", "error", 0);

    double trim_hz = carrier_trim_load_hz();

    // Same RX chain as the operational receiver: 480 kHz in, decimate by
    // 5 to 96 kHz, FIR cutoff narrowed to an NBFM channel (~16 kHz Carson
    // bandwidth -> +/-8 kHz passband). fm_fullscale tuned for voice.
    b210_rx_tx_core_params_t p = {
        .freq_hz                = cfg.freq_mhz * 1e6,
        .rate_hz                = 480000.0,
        .gain_db                = cfg.gain_db,
        .bw_hz                  = -1.0,
        .fm_fullscale_hz        = cfg.fm_fullscale_hz,
        .device_args            = NULL,
        .rx_antenna             = cfg.rx_antenna,
        .decim_factor           = 5u,
        .decim_cutoff_hz        = 8000.0,
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

    b210_rx_tx_core_t *core = NULL;
    if (b210_rx_tx_core_open(&p, &core) != 0) {
        fprintf(stderr, "ham_listen: could not open an SDR\n");
        return 1;
    }

    int    audio_rate = (int) lround(b210_rx_tx_core_actual_rate(core));
    size_t max_chunk  = b210_rx_tx_core_max_chunk(core);
    if (max_chunk == 0) max_chunk = 4096;

    int16_t *pcm = (int16_t *) malloc(max_chunk * sizeof(int16_t));
    if (pcm == NULL) {
        fprintf(stderr, "ham_listen: out of memory\n");
        b210_rx_tx_core_close(core);
        return 1;
    }

    // Output sink: either the local soundcard, or — with --ogg-stdout — an
    // Ogg/Vorbis stream on stdout, so a remote ground station can be
    // monitored over ssh (`ssh rao ham_listen --ogg-stdout | ffplay -i -`).
    // Ogg is a streaming container, so it survives a non-seekable pipe.
    audio_play_t *play = NULL;
#ifdef HAVE_SNDFILE
    SNDFILE *ogg = NULL;
#endif
    if (cfg.ogg_stdout) {
#ifdef HAVE_SNDFILE
        SF_INFO info = {0};
        info.samplerate = audio_rate;
        info.channels   = 1;
        info.format     = SF_FORMAT_OGG | SF_FORMAT_VORBIS;
        // close_desc = 0: libsndfile must not close stdout out from under us;
        // sf_close() below flushes the trailing Ogg pages, process exit closes
        // the fd.
        ogg = sf_open_fd(STDOUT_FILENO, SFM_WRITE, &info, 0);
        if (ogg == NULL) {
            fprintf(stderr, "ham_listen: cannot open Ogg/Vorbis on stdout: %s\n",
                    sf_strerror(NULL));
            free(pcm);
            b210_rx_tx_core_close(core);
            return 1;
        }
        double q = cfg.ogg_quality;
        if (q < 0.0) q = 0.0;
        if (q > 1.0) q = 1.0;
        sf_command(ogg, SFC_SET_VBR_ENCODING_QUALITY, &q, sizeof(q));
#else
        fprintf(stderr, "ham_listen: --ogg-stdout needs libsndfile, which this build lacks.\n");
        free(pcm);
        b210_rx_tx_core_close(core);
        return 1;
#endif
    } else {
        play = audio_play_open(audio_rate, 1);
        if (play == NULL) {
            fprintf(stderr, "ham_listen: could not open the audio output device\n");
            free(pcm);
            b210_rx_tx_core_close(core);
            return 1;
        }
    }

    fprintf(stderr,
            "ham_listen: %s on %.4f MHz, %d Hz %s. Ctrl-C to stop.\n",
            b210_rx_tx_core_sdr_name(core), cfg.freq_mhz, audio_rate,
            cfg.ogg_stdout ? "Ogg/Vorbis -> stdout" : "audio");

    // One-pole de-emphasis / voice LPF coefficient.
    double deemph_a = (cfg.deemphasis_hz > 0.0)
        ? 1.0 - exp(-2.0 * M_PI * cfg.deemphasis_hz / (double) audio_rate)
        : 0.0;
    double deemph_y = 0.0;

    long status_acc = 0;     // samples since last status line
    long hang_left  = 0;     // squelch hang, in samples

    while (!g_stop) {
        ssize_t n = b210_rx_tx_core_pump(core, pcm, max_chunk,
                                         NULL, 0, NULL, NULL, 0, NULL);
        if (n < 0) { fprintf(stderr, "\nham_listen: RX fatal error\n"); break; }
        if (n == 0) continue;

        if (deemph_a > 0.0) {
            for (ssize_t i = 0; i < n; ++i) {
                deemph_y += deemph_a * ((double) pcm[i] - deemph_y);
                pcm[i] = (int16_t) lround(deemph_y);
            }
        }

        // Carrier-power squelch off the post-FIR IQ envelope (not the
        // audio, whose RMS is loudest on open-channel noise).
        double rms_sq = 0.0;
        b210_rx_tx_core_iq_levels(core, NULL, &rms_sq);
        double mag = sqrt(rms_sq);
        int open = 1;
        if (cfg.squelch > 0.0) {
            if (mag >= cfg.squelch) hang_left = (long)(0.3 * audio_rate);
            else                    hang_left -= n;
            if (hang_left < 0) hang_left = 0;
            open = (mag >= cfg.squelch) || (hang_left > 0);
            if (!open) memset(pcm, 0, (size_t) n * sizeof(int16_t));
        }

#ifdef HAVE_SNDFILE
        if (cfg.ogg_stdout) {
            sf_writef_short(ogg, pcm, n);
        } else
#endif
        {
            audio_play_write(play, pcm, (size_t) n);
        }

        status_acc += n;
        if (status_acc >= audio_rate / 2) {
            status_acc = 0;
            fprintf(stderr, "\rlevel %6.0f   %s        ",
                    mag, (cfg.squelch > 0.0)
                           ? (open ? "[open]" : "[squelched]")
                           : "");
            fflush(stderr);
        }
    }

    fprintf(stderr, "\nham_listen: stopping\n");
#ifdef HAVE_SNDFILE
    if (ogg) sf_close(ogg);
#endif
    if (play) audio_play_close(play);
    free(pcm);
    b210_rx_tx_core_close(core);
    return 0;
}
