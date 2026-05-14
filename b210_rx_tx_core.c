/*

   Simple Satellite Operations  b210_rx_core.c

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "b210_rx_core.h"
#include "fir_decim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include <uhd/types/tune_request.h>
#include <uhd/types/metadata.h>
#include <uhd/error.h>

struct b210_rx_core {
    uhd_usrp_handle         dev;
    uhd_rx_streamer_handle  stream;
    uhd_rx_metadata_handle  md;

    // UHD streamer side
    int16_t                *iq_chunk;        // sc16 interleaved, max_iq_in pairs
    size_t                  max_iq_in;       // UHD's max_num_samps

    // Optional IQ decimator (FIR + downsample). When NULL, the FM demod
    // runs directly on iq_chunk; when non-NULL, iq_chunk is filtered into
    // iq_decim and the demod runs on that. Either way the post-FIR
    // sample rate equals actual_rate.
    fir_decim_iq_t         *decim;
    int16_t                *iq_decim;        // sc16 interleaved, max_iq_out pairs
    size_t                  max_iq_out;      // ceil(max_iq_in / M) + 1

    double                  input_rate;      // UHD's coerced sample rate
    double                  actual_rate;     // post-decim rate (==input_rate if no decim)
    double                  actual_freq;
    double                  fm_fullscale_hz;
    double                  k_scale;         // dphi_rad → int16 PCM, calibrated at actual_rate
    double                  prev_I;
    double                  prev_Q;
    int                     have_prev;
    int                     stream_running;

    // Post-FIR IQ level meter. Updated every pump on the same iq_demod
    // stream the FM discriminator sees, so the readback reflects what's
    // inside the decoded passband — out-of-band carriers stay invisible.
    // Fast-attack peak (instant rise to max(|I|,|Q|), 0.5 s exponential
    // release) drives a "blip" indicator; rms_sq is a 30 ms smoothing
    // of I²+Q² for a steady RMS magnitude reading.
    double                  iq_peak_env;
    double                  iq_rms_sq;
    double                  iq_peak_release_alpha;
    double                  iq_rms_alpha;
};

static int log_uhd(uhd_error e, const char *what)
{
    if (e == UHD_ERROR_NONE) return 0;
    char errbuf[256] = {0};
    (void)uhd_get_last_error(errbuf, sizeof errbuf);
    fprintf(stderr, "b210_rx_core: %s: UHD error %d: %s\n",
            what, (int)e, errbuf[0] ? errbuf : "(no detail)");
    return 1;
}

// Pull the actual freq from UHD (after a tune) and update the cached value.
// Best-effort; on UHD failure leave the cache unchanged.
static void refresh_actual_freq(b210_rx_core_t *c)
{
    double f = c->actual_freq;
    if (uhd_usrp_get_rx_freq(c->dev, 0, &f) == UHD_ERROR_NONE) {
        c->actual_freq = f;
    }
}

int b210_rx_core_open(const b210_rx_core_params_t *p, b210_rx_core_t **out)
{
    if (out == NULL || p == NULL) return -1;
    *out = NULL;

    const char *device_args = p->device_args ? p->device_args : "type=b200";
    const char *rx_antenna  = p->rx_antenna  ? p->rx_antenna  : "RX2";
    double bw_hz = p->bw_hz > 0.0 ? p->bw_hz : p->rate_hz;
    double fm_fullscale = p->fm_fullscale_hz > 0.0 ? p->fm_fullscale_hz : 25000.0;
    unsigned decim_M     = (p->decim_factor >= 1u) ? p->decim_factor : 1u;
    unsigned decim_taps  = p->decim_taps  > 0u   ? p->decim_taps    : 96u;

    b210_rx_core_t *c = (b210_rx_core_t *)calloc(1, sizeof(*c));
    if (c == NULL) {
        fprintf(stderr, "b210_rx_core: out of memory\n");
        return -1;
    }
    c->fm_fullscale_hz = fm_fullscale;

    if (log_uhd(uhd_usrp_make(&c->dev, device_args), "uhd_usrp_make")) goto fail;

    if (log_uhd(uhd_usrp_set_rx_rate(c->dev, p->rate_hz, 0), "set_rx_rate")) goto fail;
    c->input_rate = p->rate_hz;
    (void)uhd_usrp_get_rx_rate(c->dev, 0, &c->input_rate);
    if (fabs(c->input_rate - p->rate_hz) / p->rate_hz > 0.01) {
        fprintf(stderr, "b210_rx_core: requested %.0f S/s, got %.0f S/s "
                        "(B210 coerced)\n", p->rate_hz, c->input_rate);
    }
    // actual_rate (post-decimation) is what the FM demod runs at, so it
    // sets k_scale. dphi-per-sample for a given Hz of frequency deviation
    // is rate-dependent — running atan2 at 48 kHz vs 240 kHz gives 5×
    // larger dphi for the same modulation, so k_scale shrinks 5× to keep
    // the same int16 PCM amplitude.
    c->actual_rate = c->input_rate / (double)decim_M;
    c->k_scale = c->actual_rate * 32767.0 / (c->fm_fullscale_hz * 2.0 * M_PI);
    // Per-sample alphas at the post-FIR rate. 0.5 s peak release →
    // visible "blip" decay over ~half a second; 30 ms RMS smoothing →
    // steady reading without sample-to-sample jitter.
    c->iq_peak_release_alpha = exp(-1.0 / (0.5   * c->actual_rate));
    c->iq_rms_alpha          = exp(-1.0 / (0.030 * c->actual_rate));

    if (log_uhd(uhd_usrp_set_rx_gain(c->dev, p->gain_db, 0, ""), "set_rx_gain")) goto fail;
    if (log_uhd(uhd_usrp_set_rx_bandwidth(c->dev, bw_hz, 0), "set_rx_bandwidth")) goto fail;
    if (log_uhd(uhd_usrp_set_rx_antenna(c->dev, rx_antenna, 0), "set_rx_antenna")) goto fail;

    {
        uhd_tune_request_t req = {
            .target_freq     = p->freq_hz,
            .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
            .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
            .args            = NULL,
        };
        uhd_tune_result_t res = {0};
        if (log_uhd(uhd_usrp_set_rx_freq(c->dev, &req, 0, &res), "set_rx_freq")) goto fail;
    }
    c->actual_freq = p->freq_hz;
    refresh_actual_freq(c);

    if (log_uhd(uhd_rx_streamer_make(&c->stream), "rx_streamer_make")) goto fail;
    {
        size_t channels[1] = { 0 };
        uhd_stream_args_t args = {
            .cpu_format   = "sc16",
            .otw_format   = "sc16",
            .args         = "",
            .channel_list = channels,
            .n_channels   = 1,
        };
        if (log_uhd(uhd_usrp_get_rx_stream(c->dev, &args, c->stream),
                    "get_rx_stream")) goto fail;
    }

    if (log_uhd(uhd_rx_streamer_max_num_samps(c->stream, &c->max_iq_in),
                "rx_max_num_samps")) goto fail;
    if (c->max_iq_in == 0) c->max_iq_in = 2040;

    c->iq_chunk = (int16_t *)malloc(c->max_iq_in * 2 * sizeof(int16_t));
    if (c->iq_chunk == NULL) {
        fprintf(stderr, "b210_rx_core: out of memory for IQ chunk buf\n");
        goto fail;
    }

    if (decim_M > 1u) {
        double fc_hz = p->decim_cutoff_hz > 0.0
            ? p->decim_cutoff_hz
            : 0.4 * c->actual_rate;
        c->decim = fir_decim_iq_new(c->input_rate, fc_hz, decim_taps, decim_M);
        if (c->decim == NULL) {
            fprintf(stderr, "b210_rx_core: failed to build IQ decimator "
                            "(fs=%.0f fc=%.0f taps=%u M=%u)\n",
                    c->input_rate, fc_hz, decim_taps, decim_M);
            goto fail;
        }
        // Output IQ buffer is sized to the worst-case post-decim count
        // for a single recv. +1 for the case where the decimator's phase
        // happened to align exactly at the start of the chunk.
        c->max_iq_out = c->max_iq_in / decim_M + 1u;
        c->iq_decim = (int16_t *)malloc(c->max_iq_out * 2 * sizeof(int16_t));
        if (c->iq_decim == NULL) {
            fprintf(stderr, "b210_rx_core: out of memory for decim IQ buf\n");
            goto fail;
        }
    } else {
        c->max_iq_out = c->max_iq_in;
    }

    if (log_uhd(uhd_rx_metadata_make(&c->md), "rx_metadata_make")) goto fail;

    {
        uhd_stream_cmd_t cmd = {
            .stream_mode          = UHD_STREAM_MODE_START_CONTINUOUS,
            .num_samps            = 0,
            .stream_now           = true,
            .time_spec_full_secs  = 0,
            .time_spec_frac_secs  = 0.0,
        };
        if (log_uhd(uhd_rx_streamer_issue_stream_cmd(c->stream, &cmd),
                    "issue_start_stream")) goto fail;
        c->stream_running = 1;
    }

    *out = c;
    return 0;

fail:
    b210_rx_core_close(c);
    return -1;
}

void b210_rx_core_close(b210_rx_core_t *c)
{
    if (c == NULL) return;
    if (c->stream_running && c->stream != NULL) {
        uhd_stream_cmd_t cmd = {
            .stream_mode = UHD_STREAM_MODE_STOP_CONTINUOUS,
            .num_samps   = 0,
            .stream_now  = true,
        };
        (void)uhd_rx_streamer_issue_stream_cmd(c->stream, &cmd);
    }
    if (c->md     != NULL) uhd_rx_metadata_free(&c->md);
    if (c->stream != NULL) uhd_rx_streamer_free(&c->stream);
    if (c->dev    != NULL) uhd_usrp_free(&c->dev);
    if (c->decim  != NULL) fir_decim_iq_free(c->decim);
    free(c->iq_chunk);
    free(c->iq_decim);
    free(c);
}

ssize_t b210_rx_core_pump(b210_rx_core_t *c, int16_t *pcm_out, size_t pcm_cap)
{
    if (c == NULL || pcm_out == NULL || pcm_cap == 0) return -1;

    // Decide how much IQ to ask UHD for. With decimation by M, each
    // output PCM sample corresponds to M input IQ samples, so to fill
    // pcm_cap outputs we need roughly pcm_cap*M inputs (capped at the
    // streamer's max recv chunk).
    size_t want_iq;
    if (c->decim != NULL) {
        unsigned M = fir_decim_iq_M(c->decim);
        want_iq = pcm_cap * (size_t)M;
        if (want_iq > c->max_iq_in) want_iq = c->max_iq_in;
    } else {
        want_iq = pcm_cap < c->max_iq_in ? pcm_cap : c->max_iq_in;
    }

    void  *bufs[1] = { c->iq_chunk };
    size_t n_recv = 0;
    uhd_error e = uhd_rx_streamer_recv(c->stream, bufs, want_iq, &c->md,
                                       /*timeout=*/3.0,
                                       /*one_packet=*/false,
                                       &n_recv);
    if (e != UHD_ERROR_NONE) {
        // Treat as transient: log once, return 0 so caller keeps looping.
        // Truly fatal errors here would prevent the streamer from ever
        // delivering samples again, in which case the caller's outer
        // duration / signal handling is the right place to bail.
        log_uhd(e, "rx_recv");
        return 0;
    }
    uhd_rx_metadata_error_code_t mderr = 0;
    if (uhd_rx_metadata_error_code(c->md, &mderr) == UHD_ERROR_NONE
        && mderr != UHD_RX_METADATA_ERROR_CODE_NONE) {
        char errbuf[128] = {0};
        (void)uhd_rx_metadata_strerror(c->md, errbuf, sizeof errbuf);
        fprintf(stderr, "b210_rx_core: RX metadata error %d: %s\n",
                (int)mderr, errbuf[0] ? errbuf : "(no detail)");
        // Overflow / late-packet are non-fatal — keep going. The samples
        // we DID get this round are already in iq_chunk; demod them as
        // usual rather than dropping the chunk.
    }
    if (n_recv == 0) return 0;

    // Push through the IQ decimator if configured. The result is a
    // narrowband sc16 IQ stream at actual_rate (== input_rate / M).
    // If the decimator is bypassed the demod runs on the UHD stream
    // directly, same as the pre-FIR design.
    const int16_t *iq_demod = c->iq_chunk;
    size_t n_demod = n_recv;
    if (c->decim != NULL) {
        n_demod = fir_decim_iq_push(c->decim, c->iq_chunk, n_recv,
                                    c->iq_decim, c->max_iq_out);
        iq_demod = c->iq_decim;
        if (n_demod == 0) return 0;
    }

    // IQ level meter. Run before the discriminator so the reading
    // reflects RF input (rises on incoming signal) rather than demod
    // output (whose noise floor *drops* on FM capture).
    {
        double env = c->iq_peak_env;
        double rms = c->iq_rms_sq;
        const double pa  = c->iq_peak_release_alpha;
        const double ra  = c->iq_rms_alpha;
        const double inv_pa = 1.0 - pa;
        const double inv_ra = 1.0 - ra;
        for (size_t i = 0; i < n_demod; i++) {
            double I  = (double)iq_demod[i * 2 + 0];
            double Q  = (double)iq_demod[i * 2 + 1];
            double aI = fabs(I), aQ = fabs(Q);
            double m  = (aI > aQ) ? aI : aQ;
            // Fast attack: instant rise to chunk peak. Slow release:
            // 1-pole IIR back toward the per-sample magnitude.
            if (m > env) env = m;
            else         env = pa * env + inv_pa * m;
            rms = ra * rms + inv_ra * (I * I + Q * Q);
        }
        c->iq_peak_env = env;
        c->iq_rms_sq   = rms;
    }

    // FM discriminator: pcm[k] = arg(z[k] * conj(z[k-1])) * k_scale,
    // clamped to int16 range. prev_I/prev_Q carry the phase reference
    // across pump calls so chunk seams don't pop. Output count equals
    // the post-decim sample count — the very first sample of the very
    // first pump can't be demoded (no prev) and is emitted as 0.
    size_t out_n = 0;
    size_t k     = 0;
    if (!c->have_prev) {
        c->prev_I    = (double)iq_demod[0];
        c->prev_Q    = (double)iq_demod[1];
        c->have_prev = 1;
        pcm_out[out_n++] = 0;
        k = 1;
    }
    for (; k < n_demod && out_n < pcm_cap; k++) {
        double I = (double)iq_demod[k * 2 + 0];
        double Q = (double)iq_demod[k * 2 + 1];
        double dphi = atan2(Q * c->prev_I - I * c->prev_Q,
                            I * c->prev_I + Q * c->prev_Q);
        double pcm_d = dphi * c->k_scale;
        if (pcm_d >  32767.0) pcm_d =  32767.0;
        if (pcm_d < -32768.0) pcm_d = -32768.0;
        pcm_out[out_n++] = (int16_t)lround(pcm_d);
        c->prev_I = I;
        c->prev_Q = Q;
    }
    return (ssize_t)out_n;
}

int b210_rx_core_set_freq(b210_rx_core_t *c, double freq_hz)
{
    if (c == NULL) return -1;
    uhd_tune_request_t req = {
        .target_freq     = freq_hz,
        .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
        .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
        .args            = NULL,
    };
    uhd_tune_result_t res = {0};
    if (log_uhd(uhd_usrp_set_rx_freq(c->dev, &req, 0, &res), "set_rx_freq")) {
        return -1;
    }
    refresh_actual_freq(c);
    return 0;
}

double b210_rx_core_actual_rate(const b210_rx_core_t *c) { return c ? c->actual_rate : 0.0; }
double b210_rx_core_input_rate (const b210_rx_core_t *c) { return c ? c->input_rate  : 0.0; }
double b210_rx_core_actual_freq(const b210_rx_core_t *c) { return c ? c->actual_freq : 0.0; }
size_t b210_rx_core_max_chunk  (const b210_rx_core_t *c) { return c ? c->max_iq_out  : 0; }

int b210_rx_core_iq_levels(const b210_rx_core_t *c,
                           double *peak_env_out, double *rms_sq_out)
{
    if (c == NULL) return -1;
    if (peak_env_out != NULL) *peak_env_out = c->iq_peak_env;
    if (rms_sq_out   != NULL) *rms_sq_out   = c->iq_rms_sq;
    return 0;
}
