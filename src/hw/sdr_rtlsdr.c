/*

   Simple Satellite Operations  sdr_rtlsdr.c

   RTL-SDR (RTL2832U + tuner) backend for the SDR abstraction. RX-only:
   tx_burst is NULL, so the rest of the program disables transmit when
   this backend is active. Yields raw IQ (converted from the dongle's
   offset-binary uint8 to int16) at a native rate chosen so the chain's
   decimator lands on the same post-decim rate the B210 path uses
   (96 kHz). The device-agnostic DSP in b210_rx_tx_core.c does the rest.

   Synchronous reads only (rtlsdr_read_sync) so the rx_session worker
   stays the single owner of the device — no async callback thread.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "sdr_backend.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rtl-sdr.h>

struct sdr_rtl {
    rtlsdr_dev_t *dev;
    uint32_t      native_rate;   // achieved sample rate
    double        actual_freq;   // last read-back center freq
    size_t        max_pairs;     // IQ pairs per read_sync
    uint8_t      *u8buf;         // read_sync byte buffer (max_pairs * 2)
};

static void rtl_close(sdr_backend_t *be);

// Snap the requested gain (dB) to the nearest value the tuner supports
// (librtlsdr reports gains in tenths of a dB) and set manual gain.
static int rtl_apply_gain(struct sdr_rtl *r, double gain_db)
{
    int target = (int)lround(gain_db * 10.0);
    int n = rtlsdr_get_tuner_gains(r->dev, NULL);
    if (n > 0) {
        int *gains = (int *)malloc((size_t)n * sizeof(int));
        if (gains != NULL && rtlsdr_get_tuner_gains(r->dev, gains) == n) {
            int best = gains[0];
            for (int i = 1; i < n; i++) {
                if (abs(gains[i] - target) < abs(best - target)) best = gains[i];
            }
            target = best;
        }
        free(gains);
    }
    (void)rtlsdr_set_tuner_gain_mode(r->dev, 1);  // 1 = manual
    return rtlsdr_set_tuner_gain(r->dev, target) == 0 ? 0 : -1;
}

// Pick a valid RTL-SDR sample rate that is an integer multiple of the
// chain's target post-decim rate and sits in the dongle's usable band
// (~0.9 .. 3.2 MS/s), preferring ~1.92 MS/s for generous oversampling.
static uint32_t rtl_pick_rate(double target_hz)
{
    double target = target_hz > 0.0 ? target_hz : 96000.0;
    long k = lround(1920000.0 / target);
    if (k < 1) k = 1;
    double native = target * (double)k;
    while (native > 3200000.0 && k > 1) { k--; native = target * (double)k; }
    while (native < 900001.0)           { k++; native = target * (double)k; }
    return (uint32_t)llround(native);
}

static int rtl_open(sdr_backend_t *be, const sdr_open_params_t *p, sdr_caps_t *caps)
{
    if (be == NULL || p == NULL || caps == NULL) return -1;

    struct sdr_rtl *r = (struct sdr_rtl *)calloc(1, sizeof *r);
    if (r == NULL) {
        fprintf(stderr, "sdr_rtlsdr: out of memory\n");
        return -1;
    }
    be->priv = r;

    uint32_t idx = (p->device_index >= 0) ? (uint32_t)p->device_index : 0u;
    if (rtlsdr_get_device_count() == 0) {
        fprintf(stderr, "sdr_rtlsdr: no RTL-SDR devices found\n");
        goto fail;
    }
    if (rtlsdr_open(&r->dev, idx) != 0 || r->dev == NULL) {
        fprintf(stderr, "sdr_rtlsdr: open(index=%u) failed\n", idx);
        goto fail;
    }

    uint32_t rate = rtl_pick_rate(p->target_post_decim_hz);
    if (rtlsdr_set_sample_rate(r->dev, rate) != 0) {
        fprintf(stderr, "sdr_rtlsdr: set_sample_rate(%u) failed\n", rate);
        goto fail;
    }
    r->native_rate = rtlsdr_get_sample_rate(r->dev);

    (void)rtl_apply_gain(r, p->gain_db);

    (void)rtlsdr_set_center_freq(r->dev, (uint32_t)llround(p->freq_hz));
    r->actual_freq = (double)rtlsdr_get_center_freq(r->dev);

    r->max_pairs = 16384;  // 32 KiB per read — a clean USB bulk size
    r->u8buf = (uint8_t *)malloc(r->max_pairs * 2);
    if (r->u8buf == NULL) {
        fprintf(stderr, "sdr_rtlsdr: out of memory for read buffer\n");
        goto fail;
    }
    (void)rtlsdr_reset_buffer(r->dev);

    caps->can_tx             = 0;     // RX-only — no tx_burst op
    caps->native_rate_hz     = (double)r->native_rate;
    caps->tune_resolution_hz = 1.0;
    caps->has_hw_lo_offset   = 1;     // can tune off-carrier (helps dodge the DC spur)
    caps->sc16_native        = 0;     // we convert uint8 -> int16
    caps->max_rx_pairs       = r->max_pairs;
    {
        const char *nm = rtlsdr_get_device_name(idx);
        snprintf(caps->name, sizeof caps->name, "RTL-SDR %.20s", nm ? nm : "");
    }
    fprintf(stderr,
            "sdr_rtlsdr: open index=%u rate=%u Hz freq=%.6f MHz (RX-only)\n",
            idx, r->native_rate, r->actual_freq / 1e6);
    return 0;

fail:
    rtl_close(be);
    return -1;
}

static void rtl_close(sdr_backend_t *be)
{
    if (be == NULL) return;
    struct sdr_rtl *r = (struct sdr_rtl *)be->priv;
    if (r == NULL) return;
    if (r->dev != NULL) rtlsdr_close(r->dev);
    free(r->u8buf);
    free(r);
    be->priv = NULL;
}

static ssize_t rtl_read_iq(sdr_backend_t *be, int16_t *out, size_t cap_pairs)
{
    struct sdr_rtl *r = (struct sdr_rtl *)be->priv;
    if (r == NULL || r->dev == NULL || out == NULL) return -1;

    size_t pairs = cap_pairs < r->max_pairs ? cap_pairs : r->max_pairs;
    // libusb bulk transfers want a length that is a multiple of 512
    // bytes; floor to that (256 pairs), keeping at least one block.
    size_t bytes = (pairs * 2) & ~((size_t)511);
    if (bytes == 0) bytes = 512;

    int n_read = 0;
    int rc = rtlsdr_read_sync(r->dev, r->u8buf, (int)bytes, &n_read);
    if (rc != 0) {
        // Transient (USB hiccup); log and let the caller keep looping.
        fprintf(stderr, "sdr_rtlsdr: read_sync error %d\n", rc);
        return 0;
    }
    if (n_read <= 0) return 0;

    size_t got = (size_t)n_read / 2;       // IQ pairs actually read
    if (got > cap_pairs) got = cap_pairs;
    // Offset-binary uint8 [0,255] (128 = 0) -> signed int16. << 7 maps
    // ±128 to roughly ±16384, leaving headroom below sc16 full-scale.
    for (size_t i = 0; i < got; i++) {
        out[2 * i + 0] = (int16_t)(((int)r->u8buf[2 * i + 0] - 128) << 7);
        out[2 * i + 1] = (int16_t)(((int)r->u8buf[2 * i + 1] - 128) << 7);
    }
    return (ssize_t)got;
}

static int rtl_set_freq(sdr_backend_t *be, double freq_hz)
{
    struct sdr_rtl *r = (struct sdr_rtl *)be->priv;
    if (r == NULL) return -1;
    if (rtlsdr_set_center_freq(r->dev, (uint32_t)llround(freq_hz)) != 0) return -1;
    r->actual_freq = (double)rtlsdr_get_center_freq(r->dev);
    return 0;
}

static double rtl_get_actual_freq(sdr_backend_t *be)
{
    struct sdr_rtl *r = (struct sdr_rtl *)be->priv;
    return r ? r->actual_freq : 0.0;
}

static int rtl_set_gain(sdr_backend_t *be, double gain_db)
{
    struct sdr_rtl *r = (struct sdr_rtl *)be->priv;
    if (r == NULL) return -1;
    return rtl_apply_gain(r, gain_db);
}

static const sdr_backend_ops_t rtl_ops = {
    .name            = "rtlsdr",
    .open            = rtl_open,
    .close           = rtl_close,
    .read_iq         = rtl_read_iq,
    .set_freq        = rtl_set_freq,
    .get_actual_freq = rtl_get_actual_freq,
    .set_gain        = rtl_set_gain,
    .tx_burst        = NULL,   // RX-only
};

const sdr_backend_ops_t *sdr_backend_rtlsdr_ops(void)
{
    return &rtl_ops;
}
