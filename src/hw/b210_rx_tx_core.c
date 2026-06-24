/*

   Simple Satellite Operations  b210_rx_tx_core.c

   Device-agnostic RX/TX chain. Pulls raw IQ from a pluggable SDR
   backend (src/hw/sdr_backend.h) and runs the shared DSP: optional FIR
   decimation, software Doppler NCO, a carrier-to-DC NCO, an atan2 FM
   discriminator, an IQ level meter, and a broadband-burst detector. TX
   bursts are delegated to the backend (RX-only backends have none).

   The UHD-specific device I/O that used to live here now lives in
   src/hw/sdr_uhd.c; this file keeps its name and public API so callers
   (rx_session, tx_burst, simple_sat_ops) are unchanged.

   FM demod is the same per-sample atan2 discriminator as
   utils/b210_rx_capture.c — phase reference (prev_I, prev_Q) is held
   inside the core across pump calls so chunk boundaries don't pop.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "b210_rx_tx_core.h"
#include "sdr_backend.h"
#include "fir_decim.h"
#include "iq_burst.h"
#include "sw_nco.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct b210_rx_tx_core {
    sdr_backend_t          *backend;         // device I/O (UHD, RTL-SDR, ...)

    // Raw-IQ input buffer (the backend's read_iq fills this).
    int16_t                *iq_chunk;        // sc16 interleaved, max_iq_in pairs
    size_t                  max_iq_in;       // backend's max IQ pairs per read

    // Optional IQ decimator (FIR + downsample). When NULL, the FM demod
    // runs directly on iq_chunk; when non-NULL, iq_chunk is filtered into
    // iq_decim and the demod runs on that. Either way the post-FIR
    // sample rate equals actual_rate.
    fir_decim_iq_t         *decim;
    int16_t                *iq_decim;        // sc16 interleaved, max_iq_out pairs
                                              // post-Doppler, PRE-fm_lo_nco
                                              // (carrier at +lo_offset baseband)
    int16_t                *iq_for_decode;   // sc16 interleaved, max_iq_out pairs
                                              // post-Doppler, POST-fm_lo_nco
                                              // (carrier at DC).
    size_t                  max_iq_out;      // ceil(max_iq_in / M) + 1

    double                  input_rate;      // backend's coerced sample rate
    double                  actual_rate;     // post-decim rate (==input_rate if no decim)
    double                  actual_freq;
    double                  fm_fullscale_hz;
    double                  k_scale;         // dphi_rad → int16 PCM, calibrated at actual_rate
    double                  prev_I;
    double                  prev_Q;
    int                     have_prev;

    // Post-FIR IQ level meter (see header). Fast-attack peak + smoothed
    // RMS, both at the post-decim rate.
    double                  iq_peak_env;
    double                  iq_rms_sq;
    double                  iq_peak_release_alpha;
    double                  iq_rms_alpha;

    // Software Doppler NCO + the FM-path LO-compensation NCO. See the
    // header / the original design notes; both run at actual_rate.
    sw_nco_t                sw_nco;
    sw_nco_t                fm_lo_nco;
    int                     fm_lo_nco_active;
    double                  fm_lo_compensation_hz; // operator's rx_lo_offset
    double                  tune_residual_hz;      // target − actual, per retune
    double                  carrier_trim_hz;       // per-host TCXO calibration

    // Broadband-burst detector — FFTs the post-NCO IQ and counts bright
    // bins. Narrowband ⇒ few; wideband ⇒ many.
    struct iq_burst        *iq_burst_det;
    int                     last_burst_bright_bins;
    double                  last_burst_peak_excess_db;
};

// Recompute the fm_lo_nco rotation from the current operator offset
// + the SDR tune residual + the per-host carrier trim, so the
// satellite carrier lands at DC. sw_nco_set_freq(f) shifts signals DOWN
// by f, so set f to the carrier's current baseband location (which is
// −fm_lo_compensation_hz + tune_residual_hz + carrier_trim_hz) and it
// ends up at exactly 0.
static void refresh_fm_lo_nco(b210_rx_tx_core_t *c)
{
    double f = -c->fm_lo_compensation_hz + c->tune_residual_hz + c->carrier_trim_hz;
    sw_nco_set_freq(&c->fm_lo_nco, f);
    c->fm_lo_nco_active = (f != 0.0);
}

int b210_rx_tx_core_open(const b210_rx_tx_core_params_t *p, b210_rx_tx_core_t **out)
{
    if (out == NULL || p == NULL) return -1;
    *out = NULL;

    double fm_fullscale = p->fm_fullscale_hz > 0.0 ? p->fm_fullscale_hz : 25000.0;
    unsigned req_decim_M = (p->decim_factor >= 1u) ? p->decim_factor : 1u;
    unsigned decim_taps  = p->decim_taps  > 0u   ? p->decim_taps    : 96u;
    // The post-decim rate the caller is asking for (rate_hz / decim).
    // A backend with a fixed/quantized native rate (RTL-SDR) coerces to
    // a valid native that is an integer multiple of this; the actual
    // decimation factor is then derived from the backend's native rate
    // after open, so the chain lands on the same post-decim rate
    // regardless of which SDR opened.
    double target_rate = (req_decim_M >= 1u)
                       ? p->rate_hz / (double)req_decim_M
                       : p->rate_hz;

    b210_rx_tx_core_t *c = (b210_rx_tx_core_t *)calloc(1, sizeof(*c));
    if (c == NULL) {
        fprintf(stderr, "b210_rx_tx_core: out of memory\n");
        return -1;
    }
    c->fm_fullscale_hz = fm_fullscale;

    // Open the device backend. backend_type selects UHD / RTL-SDR / auto;
    // the UHD overrides let the caller force device args or an FPGA image
    // for a B2xx clone.
    sdr_open_params_t sp = {
        .freq_hz             = p->freq_hz,
        .rate_hz             = p->rate_hz,
        .target_post_decim_hz = target_rate,
        .gain_db             = p->gain_db,
        .bw_hz               = p->bw_hz,
        .rx_antenna          = p->rx_antenna,
        .device_args         = p->device_args,
        .rx_dc_offset_track  = p->rx_dc_offset_track,
        .rx_iq_balance_track = p->rx_iq_balance_track,
        .uhd_args_override   = p->uhd_args_override,
        .fpga_image_path     = p->fpga_image_path,
        .device_index        = p->device_index,
    };
    if (sdr_backend_open(p->backend_type, &sp, &c->backend) != 0) goto fail;

    const sdr_caps_t *caps = sdr_backend_caps(c->backend);
    c->input_rate = caps->native_rate_hz;
    c->max_iq_in  = caps->max_rx_pairs;
    if (c->max_iq_in == 0) c->max_iq_in = 2040;

    // Derive the decimation factor from the backend's ACTUAL native rate
    // so the post-decim rate matches target_rate whichever SDR opened
    // (UHD 480k/5, RTL-SDR 1.92M/20 -> both 96 kHz).
    unsigned decim_M = req_decim_M;
    if (target_rate > 0.0 && c->input_rate > 0.0) {
        long m = lround(c->input_rate / target_rate);
        if (m < 1) m = 1;
        decim_M = (unsigned)m;
    }

    // actual_rate (post-decimation) is what the FM demod runs at, so it
    // sets k_scale. dphi-per-sample for a given Hz of deviation is
    // rate-dependent, so k_scale scales with actual_rate to keep the
    // same int16 PCM amplitude.
    c->actual_rate = c->input_rate / (double)decim_M;
    c->k_scale = c->actual_rate * 32767.0 / (c->fm_fullscale_hz * 2.0 * M_PI);
    c->iq_peak_release_alpha = exp(-1.0 / (0.5   * c->actual_rate));
    c->iq_rms_alpha          = exp(-1.0 / (0.030 * c->actual_rate));

    sw_nco_init(&c->sw_nco, c->actual_rate);
    sw_nco_init(&c->fm_lo_nco, c->actual_rate);
    c->fm_lo_compensation_hz = p->fm_lo_compensation_hz;
    c->tune_residual_hz      = 0.0;
    c->carrier_trim_hz       = p->carrier_trim_hz;

    // Broadband-burst detector at the post-decim rate. N=512 → ~187 Hz
    // bin width at 96 kS/s. Allocation failure is non-fatal.
    c->iq_burst_det = iq_burst_new(512u, c->actual_rate, 10.0, 2.0);
    if (c->iq_burst_det == NULL) {
        fprintf(stderr, "b210_rx_tx_core: iq_burst_new failed; "
                "ribbon will not report broadband-burst counts.\n");
    }
    c->last_burst_bright_bins    = 0;
    c->last_burst_peak_excess_db = 0.0;

    // Initial tune residual: the backend tuned to p->freq_hz in open;
    // read back the achieved LO and fold the difference into fm_lo_nco.
    c->actual_freq      = sdr_backend_get_actual_freq(c->backend);
    c->tune_residual_hz = p->freq_hz - c->actual_freq;
    refresh_fm_lo_nco(c);
    if (fabs(c->tune_residual_hz) >= 1.0) {
        fprintf(stderr,
                "b210_rx_tx_core: tune residual %.1f Hz (target %.6f MHz, "
                "actual %.6f MHz) — folded into fm_lo_nco\n",
                c->tune_residual_hz, p->freq_hz / 1e6, c->actual_freq / 1e6);
    }

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
        c->max_iq_out = c->max_iq_in / decim_M + 1u;
        c->iq_decim = (int16_t *)malloc(c->max_iq_out * 2 * sizeof(int16_t));
        if (c->iq_decim == NULL) {
            fprintf(stderr, "b210_rx_tx_core: out of memory for decim IQ buf\n");
            goto fail;
        }
    } else {
        c->max_iq_out = c->max_iq_in;
    }
    c->iq_for_decode = (int16_t *)malloc(c->max_iq_out * 2 * sizeof(int16_t));
    if (c->iq_for_decode == NULL) {
        fprintf(stderr,
            "b210_rx_tx_core: out of memory for decode IQ buf\n");
        goto fail;
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
    if (c->backend != NULL) sdr_backend_close(c->backend);
    if (c->decim   != NULL) fir_decim_iq_free(c->decim);
    if (c->iq_burst_det != NULL) iq_burst_free(c->iq_burst_det);
    free(c->iq_chunk);
    free(c->iq_decim);
    free(c->iq_for_decode);
    free(c);
}

ssize_t b210_rx_tx_core_pump(b210_rx_tx_core_t *c, int16_t *pcm_out, size_t pcm_cap,
                             int16_t *iq_raw_out,    size_t iq_raw_cap,
                             size_t  *out_iq_raw_pairs,
                             int16_t *iq_decode_out, size_t iq_decode_cap,
                             size_t  *out_iq_decode_pairs)
{
    if (out_iq_raw_pairs)    *out_iq_raw_pairs    = 0;
    if (out_iq_decode_pairs) *out_iq_decode_pairs = 0;
    if (c == NULL || pcm_out == NULL || pcm_cap == 0) return -1;

    // Decide how much IQ to ask the backend for. With decimation by M,
    // each output PCM sample corresponds to M input IQ samples.
    size_t want_iq;
    if (c->decim != NULL) {
        unsigned M = fir_decim_iq_M(c->decim);
        want_iq = pcm_cap * (size_t)M;
        if (want_iq > c->max_iq_in) want_iq = c->max_iq_in;
    } else {
        want_iq = pcm_cap < c->max_iq_in ? pcm_cap : c->max_iq_in;
    }

    ssize_t got = sdr_backend_read_iq(c->backend, c->iq_chunk, want_iq);
    if (got < 0) return -1;   // fatal
    if (got == 0) return 0;   // transient — keep looping
    size_t n_recv = (size_t)got;

    // Push through the IQ decimator if configured. The result is a
    // narrowband sc16 IQ stream at actual_rate (== input_rate / M).
    int16_t *iq_demod_buf = c->iq_chunk;
    size_t n_demod = n_recv;
    if (c->decim != NULL) {
        n_demod = fir_decim_iq_push(c->decim, c->iq_chunk, n_recv,
                                    c->iq_decim, c->max_iq_out);
        iq_demod_buf = c->iq_decim;
        if (n_demod == 0) return 0;
    }
    // Software Doppler correction — applied in place on the post-decim
    // buffer so both the IQ tap and the decode path see the same
    // Doppler-tracked stream. The carrier is parked at the operator's
    // +lo_offset baseband in iq_demod_buf after this step.
    sw_nco_apply(&c->sw_nco, iq_demod_buf, n_demod);

    // Decode-path buffer: rotated to DC for FM discriminator + shadow
    // IQ decoder + IQ-burst detector + level meter. When fm_lo_nco is
    // inactive, alias iq_for_decode to iq_demod_buf to skip the copy.
    int16_t *iq_decode = iq_demod_buf;
    if (c->fm_lo_nco_active) {
        memcpy(c->iq_for_decode, iq_demod_buf,
               n_demod * 2 * sizeof(int16_t));
        sw_nco_apply(&c->fm_lo_nco, c->iq_for_decode, n_demod);
        iq_decode = c->iq_for_decode;
    }
    const int16_t *iq_demod = iq_decode;

    // Broadband-burst detector: reads the decode-path buffer so the
    // FFT bins line up with carrier-at-DC.
    if (c->iq_burst_det != NULL) {
        iq_burst_push(c->iq_burst_det, iq_demod, n_demod);
        c->last_burst_bright_bins    = iq_burst_bright_bins(c->iq_burst_det);
        c->last_burst_peak_excess_db = iq_burst_peak_excess_db(c->iq_burst_det);
    }

    // IQ level meter on the decode-path buffer.
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
            if (m > env) env = m;
            else         env = pa * env + inv_pa * m;
            rms = ra * rms + inv_ra * (I * I + Q * Q);
        }
        c->iq_peak_env = env;
        c->iq_rms_sq   = rms;
    }

    // Raw IQ tap (carrier at +lo_offset baseband).
    if (iq_raw_out != NULL && iq_raw_cap >= 2) {
        size_t pairs = n_demod;
        if (pairs * 2 > iq_raw_cap) pairs = iq_raw_cap / 2;
        memcpy(iq_raw_out, iq_demod_buf, pairs * 2 * sizeof(int16_t));
        if (out_iq_raw_pairs) *out_iq_raw_pairs = pairs;
    }
    // Decode-path IQ tap (carrier at DC).
    if (iq_decode_out != NULL && iq_decode_cap >= 2) {
        size_t pairs = n_demod;
        if (pairs * 2 > iq_decode_cap) pairs = iq_decode_cap / 2;
        memcpy(iq_decode_out, iq_demod, pairs * 2 * sizeof(int16_t));
        if (out_iq_decode_pairs) *out_iq_decode_pairs = pairs;
    }

    // FM discriminator: pcm[k] = arg(z[k] * conj(z[k-1])) * k_scale.
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
    if (sdr_backend_set_freq(c->backend, freq_hz) != 0) return -1;
    c->actual_freq      = sdr_backend_get_actual_freq(c->backend);
    c->tune_residual_hz = freq_hz - c->actual_freq;
    refresh_fm_lo_nco(c);
    return 0;
}

int b210_rx_tx_core_set_gain(b210_rx_tx_core_t *c, double gain_db)
{
    if (c == NULL) return -1;
    return sdr_backend_set_gain(c->backend, gain_db);
}

double b210_rx_tx_core_actual_rate(const b210_rx_tx_core_t *c) { return c ? c->actual_rate : 0.0; }
double b210_rx_tx_core_input_rate (const b210_rx_tx_core_t *c) { return c ? c->input_rate  : 0.0; }
double b210_rx_tx_core_actual_freq(const b210_rx_tx_core_t *c) { return c ? c->actual_freq : 0.0; }
size_t b210_rx_tx_core_max_chunk  (const b210_rx_tx_core_t *c) { return c ? c->max_iq_out  : 0; }

void b210_rx_tx_core_set_doppler_offset(b210_rx_tx_core_t *c, double offset_hz)
{
    if (c == NULL) return;
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
    c->fm_lo_compensation_hz = lo_offset_hz;
    refresh_fm_lo_nco(c);
}

// Display-only meters. These read fields the pump thread writes without a
// lock, so a caller can observe a torn double or a peak/rms pair from
// adjacent pumps. That is intentional: the values drive a level meter, a
// stale or half-updated reading is cosmetic, and locking here would stall
// the pump for a UI poll. Do NOT reuse these for anything that needs a
// coherent snapshot.
int b210_rx_tx_core_iq_levels(const b210_rx_tx_core_t *c,
                           double *peak_env_out, double *rms_sq_out)
{
    if (c == NULL) return -1;
    if (peak_env_out != NULL) *peak_env_out = c->iq_peak_env;
    if (rms_sq_out   != NULL) *rms_sq_out   = c->iq_rms_sq;
    return 0;
}

// Display-only, unsynchronised read — see b210_rx_tx_core_iq_levels above.
int b210_rx_tx_core_burst_snapshot(const b210_rx_tx_core_t *c,
                                   int *out_bright_bins,
                                   double *out_peak_excess_db)
{
    if (c == NULL) return -1;
    if (out_bright_bins    != NULL) *out_bright_bins    = c->last_burst_bright_bins;
    if (out_peak_excess_db != NULL) *out_peak_excess_db = c->last_burst_peak_excess_db;
    return 0;
}

int b210_rx_tx_core_burst(b210_rx_tx_core_t *c,
                          const b210_rx_tx_core_burst_params_t *p)
{
    if (c == NULL || p == NULL) return -1;
    sdr_tx_burst_params_t sp = {
        .iq                = p->iq,
        .n_samps           = p->n_samps,
        .tx_rate_hz        = p->tx_rate_hz,
        .tx_freq_hz        = p->tx_freq_hz,
        .tx_gain_db        = p->tx_gain_db,
        .start_delay_s     = p->start_delay_s,
        .rx_resume_freq_hz = p->rx_resume_freq_hz,
    };
    // Full-duplex: the burst runs on the TX thread while the RX worker
    // keeps pumping. RX is never paused, so there is no demod seam to
    // reset here — and touching the DSP phase/level state from this
    // thread would race the worker's pump. Leave the RX chain untouched.
    return sdr_backend_tx_burst(c->backend, &sp);
}

int b210_rx_tx_core_can_tx(const b210_rx_tx_core_t *c)
{
    return c ? sdr_backend_can_tx(c->backend) : 0;
}

const char *b210_rx_tx_core_sdr_name(const b210_rx_tx_core_t *c)
{
    const sdr_caps_t *caps = c ? sdr_backend_caps(c->backend) : NULL;
    return caps ? caps->name : "";
}
