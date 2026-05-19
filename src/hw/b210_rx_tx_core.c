/*

   Simple Satellite Operations  b210_rx_tx_core.c

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "b210_rx_tx_core.h"
#include "fir_decim.h"
#include "iq_burst.h"
#include "sw_nco.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include <uhd/types/tune_request.h>
#include <uhd/types/metadata.h>
#include <uhd/error.h>

struct b210_rx_tx_core {
    uhd_usrp_handle         dev;
    uhd_rx_streamer_handle  stream;
    uhd_rx_metadata_handle  md;

    // Lazy-built TX streamer. Cached for the daemon's lifetime so
    // repeated bursts during one pass don't pay the rebuild tax.
    uhd_tx_streamer_handle  tx_stream;
    size_t                  tx_max_per_buff;
    double                  tx_rate_cached;
    double                  tx_gain_cached;

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

    // Software Doppler NCO. Applied to iq_demod after decimation so a
    // carrier at offset sw_nco.freq_hz is rotated back to DC for the
    // FM-discriminator, the .iq tap, and the shadow IQ decoder. Phase
    // is owned by the NCO and persists across pump calls and across
    // set_freq() so a smooth Doppler trajectory stays phase-coherent.
    // Lives in src/dsp/sw_nco for unit-testability.
    sw_nco_t                sw_nco;
    // Second NCO that runs ONLY on the FM-discriminator path. Cancels
    // the constant lo_offset so the FSK signal lands at DC for the
    // discriminator (which is calibrated for ±fm_fullscale_hz around
    // DC; a non-zero baseband offset clips the FSK upper level). The
    // public IQ tap, .iq writer, and broadband-burst detector all run
    // BEFORE this NCO so they keep the LO-offset signal visible.
    sw_nco_t                fm_lo_nco;
    int                     fm_lo_nco_active;
    int16_t                *iq_fm_scratch;     // scratch buf for the FM path
    size_t                  iq_fm_scratch_cap_pairs;

    // Broadband-burst detector — FFTs the post-NCO IQ and counts how
    // many bins simultaneously exceed their per-bin running floor.
    // Narrowband (carrier) ⇒ 1-6 bins lit; wideband (packet) ⇒ many.
    // Lets the operator ribbon visually distinguish the two. Owned by
    // the core and pumped inline so the live and recorded-IQ paths
    // see the same numbers. Allocated in open(), freed in close().
    struct iq_burst        *iq_burst_det;
    int                     last_burst_bright_bins;
    double                  last_burst_peak_excess_db;
};

static int log_uhd(uhd_error e, const char *what)
{
    if (e == UHD_ERROR_NONE) return 0;
    char errbuf[256] = {0};
    (void)uhd_get_last_error(errbuf, sizeof errbuf);
    fprintf(stderr, "b210_rx_tx_core: %s: UHD error %d: %s\n",
            what, (int)e, errbuf[0] ? errbuf : "(no detail)");
    return 1;
}

// (Software-Doppler NCO moved to src/dsp/sw_nco.{c,h} for unit-testing.
// The pump path below composes a sw_nco_t into the core struct and calls
// sw_nco_apply() right after fir_decim_iq_push.)

// Pull the actual freq from UHD (after a tune) and update the cached value.
// Best-effort; on UHD failure leave the cache unchanged.
static void refresh_actual_freq(b210_rx_tx_core_t *c)
{
    double f = c->actual_freq;
    if (uhd_usrp_get_rx_freq(c->dev, 0, &f) == UHD_ERROR_NONE) {
        c->actual_freq = f;
    }
}

int b210_rx_tx_core_open(const b210_rx_tx_core_params_t *p, b210_rx_tx_core_t **out)
{
    if (out == NULL || p == NULL) return -1;
    *out = NULL;

    const char *device_args = p->device_args ? p->device_args : "type=b200";
    const char *rx_antenna  = p->rx_antenna  ? p->rx_antenna  : "RX2";
    double bw_hz = p->bw_hz > 0.0 ? p->bw_hz : p->rate_hz;
    double fm_fullscale = p->fm_fullscale_hz > 0.0 ? p->fm_fullscale_hz : 25000.0;
    unsigned decim_M     = (p->decim_factor >= 1u) ? p->decim_factor : 1u;
    unsigned decim_taps  = p->decim_taps  > 0u   ? p->decim_taps    : 96u;

    b210_rx_tx_core_t *c = (b210_rx_tx_core_t *)calloc(1, sizeof(*c));
    if (c == NULL) {
        fprintf(stderr, "b210_rx_tx_core: out of memory\n");
        return -1;
    }
    c->fm_fullscale_hz = fm_fullscale;

    if (log_uhd(uhd_usrp_make(&c->dev, device_args), "uhd_usrp_make")) goto fail;

    if (log_uhd(uhd_usrp_set_rx_rate(c->dev, p->rate_hz, 0), "set_rx_rate")) goto fail;
    c->input_rate = p->rate_hz;
    (void)uhd_usrp_get_rx_rate(c->dev, 0, &c->input_rate);
    if (fabs(c->input_rate - p->rate_hz) / p->rate_hz > 0.01) {
        fprintf(stderr, "b210_rx_tx_core: requested %.0f S/s, got %.0f S/s "
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
    // Software-Doppler NCO runs at the post-decimation rate.
    sw_nco_init(&c->sw_nco, c->actual_rate);

    // FM-path LO-compensation NCO: shifts the post-Doppler IQ to DC
    // for the discriminator. fm_lo_compensation_hz is the SIGNED
    // offset of the hardware LO from nominal — the NCO subtracts it
    // (rotates by exp(-j 2π · (-comp) · n/fs) = exp(+j 2π · comp · n/fs))
    // to put the carrier back at 0 Hz baseband. 0 → disable, FM path
    // runs straight off the post-Doppler buffer.
    sw_nco_init(&c->fm_lo_nco, c->actual_rate);
    if (p->fm_lo_compensation_hz != 0.0) {
        // sw_nco_apply rotates by exp(-j 2π · f · n/fs). To shift a
        // tone at +offset to DC we set f = +offset. The operator's
        // lo_offset_hz is "LO minus nominal", so the signal lives at
        // -lo_offset_hz baseband. Setting f = -lo_offset_hz brings
        // it to DC.
        sw_nco_set_freq(&c->fm_lo_nco, -p->fm_lo_compensation_hz);
        c->fm_lo_nco_active = 1;
    }
    c->iq_fm_scratch = NULL;
    c->iq_fm_scratch_cap_pairs = 0;

    // Broadband-burst detector at the post-decim rate. N=512 → ~187 Hz
    // bin width at 96 kS/s. 10 dB threshold + 2 s floor τ is the same
    // pair the unit test pinned. Allocation failure is non-fatal: we
    // just leave the pointer NULL and the pump skips the FFT.
    c->iq_burst_det = iq_burst_new(512u, c->actual_rate, 10.0, 2.0);
    if (c->iq_burst_det == NULL) {
        fprintf(stderr, "b210_rx_tx_core: iq_burst_new failed; "
                "ribbon will not report broadband-burst counts.\n");
    }
    c->last_burst_bright_bins   = 0;
    c->last_burst_peak_excess_db = 0.0;

    if (log_uhd(uhd_usrp_set_rx_gain(c->dev, p->gain_db, 0, ""), "set_rx_gain")) goto fail;
    if (log_uhd(uhd_usrp_set_rx_bandwidth(c->dev, bw_hz, 0), "set_rx_bandwidth")) goto fail;
    if (log_uhd(uhd_usrp_set_rx_antenna(c->dev, rx_antenna, 0), "set_rx_antenna")) goto fail;

    {
        // mode_n=fractional forces the AD9361's fractional-N PLL so the LO
        // resolution is ~2 Hz; without it UHD defaults to integer-N on
        // some builds and the actual tune snaps to multi-kHz boundaries.
        // The fine offset (target - chosen LO) is taken up by the DSP NCO,
        // which has ~fs/2^32 ≈ 0.0001 Hz step at 480 kSPS.
        char tune_args[] = "mode_n=fractional";
        uhd_tune_request_t req = {
            .target_freq     = p->freq_hz,
            .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
            .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
            .args            = tune_args,
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
        fprintf(stderr, "b210_rx_tx_core: out of memory for IQ chunk buf\n");
        goto fail;
    }

    if (decim_M > 1u) {
        double fc_hz = p->decim_cutoff_hz > 0.0
            ? p->decim_cutoff_hz
            : 0.4 * c->actual_rate;
        c->decim = fir_decim_iq_new(c->input_rate, fc_hz, decim_taps, decim_M);
        if (c->decim == NULL) {
            fprintf(stderr, "b210_rx_tx_core: failed to build IQ decimator "
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
            fprintf(stderr, "b210_rx_tx_core: out of memory for decim IQ buf\n");
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
    b210_rx_tx_core_close(c);
    return -1;
}

void b210_rx_tx_core_close(b210_rx_tx_core_t *c)
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
    if (c->md        != NULL) uhd_rx_metadata_free(&c->md);
    if (c->stream    != NULL) uhd_rx_streamer_free(&c->stream);
    if (c->tx_stream != NULL) uhd_tx_streamer_free(&c->tx_stream);
    if (c->dev       != NULL) uhd_usrp_free(&c->dev);
    if (c->decim     != NULL) fir_decim_iq_free(c->decim);
    if (c->iq_burst_det != NULL) iq_burst_free(c->iq_burst_det);
    free(c->iq_chunk);
    free(c->iq_decim);
    free(c->iq_fm_scratch);
    free(c);
}

ssize_t b210_rx_tx_core_pump(b210_rx_tx_core_t *c, int16_t *pcm_out, size_t pcm_cap,
                             int16_t *iq_out, size_t iq_cap,
                             size_t *out_iq_pairs)
{
    if (out_iq_pairs) *out_iq_pairs = 0;
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
        fprintf(stderr, "b210_rx_tx_core: RX metadata error %d: %s\n",
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
    int16_t *iq_demod_buf = c->iq_chunk;
    size_t n_demod = n_recv;
    if (c->decim != NULL) {
        n_demod = fir_decim_iq_push(c->decim, c->iq_chunk, n_recv,
                                    c->iq_decim, c->max_iq_out);
        iq_demod_buf = c->iq_decim;
        if (n_demod == 0) return 0;
    }
    // Software Doppler correction — applied in place on the post-decim
    // buffer so every downstream consumer (.iq writer, FM discriminator,
    // shadow IQ decoder, IQ tap) sees the same Doppler-corrected stream.
    sw_nco_apply(&c->sw_nco, iq_demod_buf, n_demod);
    const int16_t *iq_demod = iq_demod_buf;

    // Broadband-burst detector: feed the post-NCO IQ in and snapshot
    // the latest count. Internally it accumulates into 512-sample
    // FFT frames so one push usually triggers 0-2 frame completions
    // at typical UHD chunk sizes.
    if (c->iq_burst_det != NULL) {
        iq_burst_push(c->iq_burst_det, iq_demod, n_demod);
        c->last_burst_bright_bins    = iq_burst_bright_bins(c->iq_burst_det);
        c->last_burst_peak_excess_db = iq_burst_peak_excess_db(c->iq_burst_det);
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

    // IQ tap: copy the post-decim IQ stream to the caller's buffer
    // before we run the discriminator on it. Same sample timing as
    // the PCM output (one IQ pair per PCM sample) so a downstream
    // recorder can pair them. Caller decides the cap; the count we
    // actually delivered lands in *out_iq_pairs.
    if (iq_out != NULL && iq_cap >= 2) {
        size_t pairs = n_demod;
        if (pairs * 2 > iq_cap) pairs = iq_cap / 2;
        memcpy(iq_out, iq_demod, pairs * 2 * sizeof(int16_t));
        if (out_iq_pairs) *out_iq_pairs = pairs;
    }

    // FM-path LO compensation: shift the Doppler-corrected stream by
    // an additional fixed offset so the carrier lands at DC for the
    // discriminator. We DON'T mutate iq_demod_buf in place because
    // the IQ tap above just sampled it — we need it intact for the
    // .iq file (which is supposed to keep the LO-offset baseband for
    // operator-visible waterfalls). Copy into a scratch buffer and
    // run the second NCO on the copy.
    if (c->fm_lo_nco_active) {
        if (c->iq_fm_scratch_cap_pairs < n_demod) {
            free(c->iq_fm_scratch);
            c->iq_fm_scratch = (int16_t *) malloc(n_demod * 2
                                                  * sizeof(int16_t));
            c->iq_fm_scratch_cap_pairs = c->iq_fm_scratch ? n_demod : 0;
        }
        if (c->iq_fm_scratch != NULL) {
            memcpy(c->iq_fm_scratch, iq_demod,
                   n_demod * 2 * sizeof(int16_t));
            sw_nco_apply(&c->fm_lo_nco, c->iq_fm_scratch, n_demod);
            iq_demod = c->iq_fm_scratch;
        }
        // If the scratch alloc failed we silently fall through to the
        // un-compensated stream — at worst the FM discriminator clips
        // again, same as before this fix.
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

int b210_rx_tx_core_set_freq(b210_rx_tx_core_t *c, double freq_hz)
{
    if (c == NULL) return -1;
    // See init for why we pass mode_n=fractional; without it the LO
    // snaps to multi-kHz boundaries and Doppler retunes are wasted.
    char tune_args[] = "mode_n=fractional";
    uhd_tune_request_t req = {
        .target_freq     = freq_hz,
        .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
        .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
        .args            = tune_args,
    };
    uhd_tune_result_t res = {0};
    if (log_uhd(uhd_usrp_set_rx_freq(c->dev, &req, 0, &res), "set_rx_freq")) {
        return -1;
    }
    refresh_actual_freq(c);
    return 0;
}

double b210_rx_tx_core_actual_rate(const b210_rx_tx_core_t *c) { return c ? c->actual_rate : 0.0; }
double b210_rx_tx_core_input_rate (const b210_rx_tx_core_t *c) { return c ? c->input_rate  : 0.0; }
double b210_rx_tx_core_actual_freq(const b210_rx_tx_core_t *c) { return c ? c->actual_freq : 0.0; }
size_t b210_rx_tx_core_max_chunk  (const b210_rx_tx_core_t *c) { return c ? c->max_iq_out  : 0; }

void b210_rx_tx_core_set_doppler_offset(b210_rx_tx_core_t *c, double offset_hz)
{
    if (c == NULL) return;
    // Frequency-only update. Phase is preserved across the change so a
    // smooth Doppler trajectory stays phase-continuous.
    sw_nco_set_freq(&c->sw_nco, offset_hz);
}

double b210_rx_tx_core_get_doppler_offset(const b210_rx_tx_core_t *c)
{
    return c ? sw_nco_get_freq(&c->sw_nco) : 0.0;
}

void b210_rx_tx_core_set_fm_lo_compensation(b210_rx_tx_core_t *c,
                                            double lo_offset_hz)
{
    if (c == NULL) return;
    if (lo_offset_hz == 0.0) {
        c->fm_lo_nco_active = 0;
        sw_nco_set_freq(&c->fm_lo_nco, 0.0);
    } else {
        // Frequency-only update — preserve phase so a small adjustment
        // doesn't pop the discriminator output.
        sw_nco_set_freq(&c->fm_lo_nco, -lo_offset_hz);
        c->fm_lo_nco_active = 1;
    }
}

int b210_rx_tx_core_iq_levels(const b210_rx_tx_core_t *c,
                           double *peak_env_out, double *rms_sq_out)
{
    if (c == NULL) return -1;
    if (peak_env_out != NULL) *peak_env_out = c->iq_peak_env;
    if (rms_sq_out   != NULL) *rms_sq_out   = c->iq_rms_sq;
    return 0;
}

int b210_rx_tx_core_burst_snapshot(const b210_rx_tx_core_t *c,
                                   int *out_bright_bins,
                                   double *out_peak_excess_db)
{
    if (c == NULL) return -1;
    if (out_bright_bins    != NULL) *out_bright_bins    = c->last_burst_bright_bins;
    if (out_peak_excess_db != NULL) *out_peak_excess_db = c->last_burst_peak_excess_db;
    return 0;
}

// Tear the RX streamer down to idle. Drain any in-flight packet so the
// next START_CONTINUOUS doesn't pick up a stale chunk. FM-demod phase
// state is cleared as well so the post-TX RX seam doesn't pop.
static int rx_pause_for_tx(b210_rx_tx_core_t *c)
{
    if (c == NULL) return -1;
    if (c->stream_running) {
        uhd_stream_cmd_t stop = {
            .stream_mode = UHD_STREAM_MODE_STOP_CONTINUOUS,
            .num_samps   = 0,
            .stream_now  = true,
        };
        if (log_uhd(uhd_rx_streamer_issue_stream_cmd(c->stream, &stop),
                    "rx_stop_for_tx")) return -1;
        c->stream_running = 0;
    }
    // Drain residual packets so the next START_CONTINUOUS sees a clean queue.
    if (c->iq_chunk != NULL && c->stream != NULL && c->md != NULL) {
        void *bufs[1] = { c->iq_chunk };
        for (int i = 0; i < 16; ++i) {
            size_t n_recv = 0;
            uhd_error e = uhd_rx_streamer_recv(c->stream, bufs, c->max_iq_in,
                                               &c->md, 0.1, false, &n_recv);
            if (e != UHD_ERROR_NONE) break;
            if (n_recv == 0) break;
        }
    }
    c->have_prev   = 0;
    c->iq_peak_env = 0.0;
    c->iq_rms_sq   = 0.0;
    return 0;
}

static int rx_resume_after_tx(b210_rx_tx_core_t *c, double rx_freq_hz)
{
    if (c == NULL || c->stream == NULL) return -1;
    if (rx_freq_hz > 0.0) {
        if (b210_rx_tx_core_set_freq(c, rx_freq_hz) != 0) return -1;
    }
    uhd_stream_cmd_t start = {
        .stream_mode         = UHD_STREAM_MODE_START_CONTINUOUS,
        .num_samps           = 0,
        .stream_now          = true,
        .time_spec_full_secs = 0,
        .time_spec_frac_secs = 0.0,
    };
    if (log_uhd(uhd_rx_streamer_issue_stream_cmd(c->stream, &start),
                "rx_resume_after_tx")) return -1;
    c->stream_running = 1;
    return 0;
}

// Build the TX streamer on first use, cache rate/gain so subsequent
// bursts only retune freq+gain when they actually changed.
static int tx_streamer_lazy_build(b210_rx_tx_core_t *c, double tx_rate_hz)
{
    if (c->tx_stream != NULL && fabs(c->tx_rate_cached - tx_rate_hz) < 1.0) {
        return 0;
    }
    if (c->tx_stream != NULL) {
        // Rate changed — rebuild. UHD doesn't let us retune the rate on
        // a live streamer.
        uhd_tx_streamer_free(&c->tx_stream);
        c->tx_stream = NULL;
        c->tx_max_per_buff = 0;
    }
    if (log_uhd(uhd_usrp_set_tx_antenna(c->dev, "TX/RX", 0),
                "set_tx_antenna")) return -1;
    if (log_uhd(uhd_usrp_set_tx_rate(c->dev, tx_rate_hz, 0),
                "set_tx_rate")) return -1;
    c->tx_rate_cached = tx_rate_hz;
    if (log_uhd(uhd_tx_streamer_make(&c->tx_stream),
                "tx_streamer_make")) return -1;
    size_t channels[1] = { 0 };
    uhd_stream_args_t args = {
        .cpu_format   = "sc16",
        .otw_format   = "sc16",
        .args         = "",
        .channel_list = channels,
        .n_channels   = 1,
    };
    if (log_uhd(uhd_usrp_get_tx_stream(c->dev, &args, c->tx_stream),
                "get_tx_stream")) return -1;
    if (log_uhd(uhd_tx_streamer_max_num_samps(c->tx_stream,
                                               &c->tx_max_per_buff),
                "tx_max_num_samps")) return -1;
    if (c->tx_max_per_buff == 0) c->tx_max_per_buff = 1024;
    c->tx_gain_cached = -1.0;  // force a set on first burst
    return 0;
}

int b210_rx_tx_core_burst(b210_rx_tx_core_t *c,
                          const b210_rx_tx_core_burst_params_t *p)
{
    if (c == NULL || p == NULL || p->iq == NULL || p->n_samps == 0) return -1;

    int rc = -1;
    if (rx_pause_for_tx(c) != 0) goto resume;
    if (tx_streamer_lazy_build(c, p->tx_rate_hz) != 0) goto resume;

    if (fabs(c->tx_gain_cached - p->tx_gain_db) > 0.05) {
        if (log_uhd(uhd_usrp_set_tx_gain(c->dev, p->tx_gain_db, 0, ""),
                    "set_tx_gain")) goto resume;
        c->tx_gain_cached = p->tx_gain_db;
    }
    // Reset the device clock so the time_spec on the first chunk gives
    // the FPGA TX FIFO room to buffer before the scheduled start.
    if (log_uhd(uhd_usrp_set_time_now(c->dev, 0, 0.0, 0),
                "tx_set_time_now")) goto resume;
    {
        uhd_tune_request_t req = {
            .target_freq     = p->tx_freq_hz,
            .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO, .rf_freq = 0.0,
            .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO, .dsp_freq = 0.0,
            .args            = NULL,
        };
        uhd_tune_result_t res = {0};
        if (log_uhd(uhd_usrp_set_tx_freq(c->dev, &req, 0, &res),
                    "set_tx_freq")) goto resume;
    }

    uhd_tx_metadata_handle md = NULL;
    if (log_uhd(uhd_tx_metadata_make(&md, false, 0, 0.0, true, false),
                "tx_metadata_make")) goto resume;

    size_t sent_total = 0;
    while (sent_total < p->n_samps) {
        size_t remaining  = p->n_samps - sent_total;
        size_t this_chunk = (remaining < c->tx_max_per_buff)
                          ? remaining : c->tx_max_per_buff;
        int is_first = (sent_total == 0);
        int is_last  = (this_chunk == remaining);
        uhd_tx_metadata_free(&md);
        if (log_uhd(uhd_tx_metadata_make(&md,
                          /*has_time_spec=*/is_first ? true : false,
                          /*full_secs=*/(int64_t) p->start_delay_s,
                          /*frac_secs=*/p->start_delay_s
                                        - (double)(int64_t) p->start_delay_s,
                          /*start_of_burst=*/is_first ? true : false,
                          /*end_of_burst=*/is_last ? true : false),
                    "tx_metadata_make (loop)")) {
            uhd_tx_metadata_free(&md);
            goto resume;
        }
        const void *bufs[1] = { p->iq + sent_total * 2 };
        size_t items_sent = 0;
        double timeout = p->start_delay_s + 1.0;
        if (timeout < 1.0) timeout = 1.0;
        uhd_error e = uhd_tx_streamer_send(c->tx_stream, bufs, this_chunk,
                                           &md, timeout, &items_sent);
        if (e != UHD_ERROR_NONE) {
            log_uhd(e, "tx_streamer_send");
            uhd_tx_metadata_free(&md);
            goto resume;
        }
        if (items_sent == 0) {
            fprintf(stderr, "b210_rx_tx_core: TX accepted 0 samples — backpressure?\n");
            uhd_tx_metadata_free(&md);
            goto resume;
        }
        sent_total += items_sent;
    }
    if (md != NULL) uhd_tx_metadata_free(&md);

    // Let the FPGA FIFO drain — the host call returned when the FIFO
    // accepted the last sample, not when the antenna stopped emitting.
    double on_air_s = (double) p->n_samps / p->tx_rate_hz;
    double drain_s  = p->start_delay_s + on_air_s + 0.05;
    if (drain_s > 0.0) {
        usleep((useconds_t) (drain_s * 1e6));
    }
    rc = 0;

resume:
    // Always try to put RX back into service, even if the TX leg failed.
    if (rx_resume_after_tx(c, p->rx_resume_freq_hz) != 0) {
        fprintf(stderr,
            "b210_rx_tx_core: WARNING — RX did not resume after TX burst\n");
    }
    return rc;
}
