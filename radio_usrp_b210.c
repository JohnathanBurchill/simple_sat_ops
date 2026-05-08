/*

   Simple Satellite Operations  radio_usrp_b210.c

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
*/

// Ettus USRP B210 backend. Control plane only — no streaming yet.
//
// The B210 is a software-defined radio, not a transceiver. There is no
// PTT line, no AM/FM/USB mode switch, no MOD input selector. The
// "transmit" knob is "start a TX streamer feeding IQ samples to the
// FPGA at the configured center frequency and gain." Until a TX
// modulator pipeline is wired up, `radio_ctl --radio-type=usrp-b210
// ptt on` just stores a flag in backend state — no RF leaves the
// connector. That gate is intentional so the rest of the abstraction
// (set_frequency, set_rf_power) is exercised on real hardware before
// the streaming work begins.
//
// Compilation gating:
//   WITH_USRP_B210=ON  -> link libuhd, fill in real ops below
//   WITH_USRP_B210=OFF -> file still compiles, init() returns
//                         RADIO_NOT_IMPLEMENTED with a CMake hint
//
// One-time host setup (Mac, Apple Silicon):
//   brew install uhd                           # libuhd + utilities
//   uhd_images_downloader.py                   # FX3 firmware + FPGA bitstream
//                                              # (lives under
//                                              #  /opt/homebrew/Cellar/uhd/<ver>/...
//                                              #  use `which` to locate)
//   uhd_find_devices                           # smoke test
//
// libuhd performs FX3-firmware and FPGA-bitstream upload transparently
// inside uhd_usrp_make() each time the device is freshly enumerated.
// We don't write that loader; we just need the image files reachable
// by libuhd's search path. If they're missing, uhd_usrp_make returns
// an error containing "image" / "firmware" / "FX3"; init() detects
// that substring and prints a friendly downloader hint.

#include "radio.h"
#include "radio_backend.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WITH_USRP_B210

#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include <uhd/types/tune_request.h>
#include <uhd/types/usrp_info.h>
#include <uhd/error.h>

// Private state hung off radio->backend_state. Allocated in init,
// freed in disconnect.
struct b210_state {
    uhd_usrp_handle h;
    double rx_freq_hz;
    double tx_freq_hz;
    double tx_gain_db;
    double tx_gain_max_db;
    int    ptt_on;
    char   mboard_name[64];
    char   serial[32];
};

// Pull the most recent UHD error string into a stack buffer for logging.
// Always NUL-terminates. uhd_get_last_error reads from a thread-local
// store, so this is fine across our single-threaded usage.
static void b210_last_error(char *buf, size_t buflen)
{
    if (buflen == 0) return;
    buf[0] = '\0';
    (void)uhd_get_last_error(buf, buflen);
}

// Map a UHD error code + last-error string to one of the project's
// RADIO_STATUS values. Recognises the "missing FPGA image / firmware"
// failure mode so the operator gets a one-shot hint instead of a
// confusing UHD stack trace.
static int b210_map_error(uhd_error e, const char *errstr)
{
    if (e == UHD_ERROR_NONE) return RADIO_OK;
    // libuhd prints its own "EnvironmentError ... Please run
    // uhd_images_downloader" guidance to stderr before returning, so we
    // don't duplicate that here. We only translate the error code into
    // one of our RADIO_STATUS values. UHD_ERROR_INVALID_DEVICE and
    // UHD_ERROR_KEY (the "no devices found" failure mode that surfaces
    // both for missing-images and for unplugged-device) map to
    // RADIO_OPEN. UHD_ERROR_NOT_IMPLEMENTED maps to NOT_SUPPORTED.
    // Everything else falls through to BAD_RESPONSE with the UHD
    // error string for the operator's eyeballs.
    if (e == UHD_ERROR_INVALID_DEVICE || e == UHD_ERROR_KEY) {
        return RADIO_OPEN;
    }
    if (e == UHD_ERROR_NOT_IMPLEMENTED) return RADIO_NOT_SUPPORTED;
    if (errstr != NULL && errstr[0] != '\0') {
        fprintf(stderr, "radio_usrp_b210: UHD error %d: %s\n", (int)e, errstr);
    } else {
        fprintf(stderr, "radio_usrp_b210: UHD error %d\n", (int)e);
    }
    return RADIO_BAD_RESPONSE;
}

static int b210_init(radio_t *radio)
{
    struct b210_state *st = calloc(1, sizeof *st);
    if (st == NULL) return RADIO_ERROR;

    uhd_error e = uhd_usrp_make(&st->h, "type=b200");
    if (e != UHD_ERROR_NONE) {
        char errbuf[256];
        b210_last_error(errbuf, sizeof errbuf);
        int rc = b210_map_error(e, errbuf);
        free(st);
        return rc;
    }

    // Best-effort device introspection. Failures here are non-fatal —
    // the radio is open and ops will work; we just won't have a serial
    // string to print in the trace.
    e = uhd_usrp_get_mboard_name(st->h, 0, st->mboard_name,
                                 sizeof st->mboard_name);
    if (e != UHD_ERROR_NONE) snprintf(st->mboard_name, sizeof st->mboard_name, "B210");

    uhd_usrp_rx_info_t info = {0};
    if (uhd_usrp_get_rx_info(st->h, 0, &info) == UHD_ERROR_NONE) {
        if (info.mboard_serial != NULL) {
            snprintf(st->serial, sizeof st->serial, "%s", info.mboard_serial);
        }
        uhd_usrp_rx_info_free(&info);
    }

    // Read the TX gain range so set_rf_power can scale 0..255 across
    // the device's reported maximum (B210: 0..89.75 dB).
    uhd_meta_range_handle gr = NULL;
    if (uhd_meta_range_make(&gr) == UHD_ERROR_NONE) {
        if (uhd_usrp_get_tx_gain_range(st->h, "", 0, gr) == UHD_ERROR_NONE) {
            double stop = 0.0;
            if (uhd_meta_range_stop(gr, &stop) == UHD_ERROR_NONE && stop > 0.0) {
                st->tx_gain_max_db = stop;
            }
        }
        uhd_meta_range_free(&gr);
    }
    if (st->tx_gain_max_db <= 0.0) st->tx_gain_max_db = 89.75;

    // Conservative defaults: 1 Msps RX/TX, RX gain 30 dB, TX gain 0 dB.
    // Stream-side work later will override per-tool. set_*_rate must
    // come before the gain calls or the AD9361 PLL hasn't settled.
    (void)uhd_usrp_set_rx_rate(st->h, 1.0e6, 0);
    (void)uhd_usrp_set_tx_rate(st->h, 1.0e6, 0);
    (void)uhd_usrp_set_rx_gain(st->h, 30.0, 0, "");
    (void)uhd_usrp_set_tx_gain(st->h, 0.0, 0, "");
    st->tx_gain_db = 0.0;

    // Hand the populated state back to the radio_t.
    radio->backend_state = st;
    radio->connected = 1;
    radio->transceiver_id = 0xB210;
    radio->ptt_off_raw_len = 0;

    fprintf(stderr,
            "radio_usrp_b210: opened %s sn=%s tx_gain_max=%.2f dB\n",
            st->mboard_name[0] ? st->mboard_name : "B210",
            st->serial[0] ? st->serial : "?", st->tx_gain_max_db);
    return RADIO_OK;
}

static void b210_disconnect(radio_t *radio)
{
    if (radio == NULL) return;
    struct b210_state *st = (struct b210_state *)radio->backend_state;
    if (st != NULL) {
        if (st->h != NULL) {
            (void)uhd_usrp_free(&st->h);
        }
        free(st);
        radio->backend_state = NULL;
    }
    radio->connected = 0;
}

static double b210_get_frequency(radio_t *radio)
{
    struct b210_state *st = (struct b210_state *)radio->backend_state;
    if (st == NULL || st->h == NULL) return -1.0;
    double f = -1.0;
    if (uhd_usrp_get_rx_freq(st->h, 0, &f) != UHD_ERROR_NONE) return -1.0;
    st->rx_freq_hz = f;
    return f;
}

static int b210_set_frequency(radio_t *radio, double freq_hz)
{
    struct b210_state *st = (struct b210_state *)radio->backend_state;
    if (st == NULL || st->h == NULL) return RADIO_BAD_RESPONSE;

    // Tune both RX and TX center freqs in lockstep. The operator's
    // mental model on a "radio" is one knob; split-VFO can come later
    // as a separate set_split_frequency op.
    uhd_tune_request_t req = {
        .target_freq     = freq_hz,
        .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO,
        .rf_freq         = 0.0,
        .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO,
        .dsp_freq        = 0.0,
        .args            = NULL,
    };
    uhd_tune_result_t res = {0};

    uhd_error e = uhd_usrp_set_rx_freq(st->h, &req, 0, &res);
    if (e != UHD_ERROR_NONE) {
        char errbuf[256];
        b210_last_error(errbuf, sizeof errbuf);
        b210_map_error(e, errbuf);
        return RADIO_SET_FREQUENCY;
    }
    e = uhd_usrp_set_tx_freq(st->h, &req, 0, &res);
    if (e != UHD_ERROR_NONE) {
        char errbuf[256];
        b210_last_error(errbuf, sizeof errbuf);
        b210_map_error(e, errbuf);
        return RADIO_SET_FREQUENCY;
    }
    st->rx_freq_hz = freq_hz;
    st->tx_freq_hz = freq_hz;
    return RADIO_OK;
}

static int b210_set_rf_power(radio_t *radio, int level_0_to_255)
{
    struct b210_state *st = (struct b210_state *)radio->backend_state;
    if (st == NULL || st->h == NULL) return RADIO_BAD_RESPONSE;

    if (level_0_to_255 < 0)   level_0_to_255 = 0;
    if (level_0_to_255 > 255) level_0_to_255 = 255;
    double gain_db = ((double)level_0_to_255 / 255.0) * st->tx_gain_max_db;

    uhd_error e = uhd_usrp_set_tx_gain(st->h, gain_db, 0, "");
    if (e != UHD_ERROR_NONE) {
        char errbuf[256];
        b210_last_error(errbuf, sizeof errbuf);
        return b210_map_error(e, errbuf);
    }
    st->tx_gain_db = gain_db;
    fprintf(stderr,
            "radio_usrp_b210: TX gain = %.2f dB (no calibrated absolute-watts "
            "available on B210)\n", gain_db);
    return RADIO_OK;
}

static int b210_ptt(radio_t *radio, int on)
{
    struct b210_state *st = (struct b210_state *)radio->backend_state;
    if (st == NULL) return RADIO_BAD_RESPONSE;
    // Until a TX streamer + IQ generator is wired up, this is just a
    // state flag. The upstream tx_inhibit_cleared gate already vetted
    // a real "ptt on", so honour it. PTT-off is always allowed.
    st->ptt_on = on ? 1 : 0;
    if (on) {
        fprintf(stderr,
                "radio_usrp_b210: ptt on (state flag only; no TX streamer "
                "yet — no RF will leave the connector).\n");
    }
    return RADIO_OK;
}

#else  // !WITH_USRP_B210

// All the slot bodies below are placeholders so the file compiles
// without UHD. init() returns RADIO_NOT_IMPLEMENTED so any tool that
// selects --radio-type=usrp-b210 fails fast with a clear hint.

#endif

static int b210_init_stub(radio_t *radio)
{
    (void)radio;
#ifdef WITH_USRP_B210
    return b210_init(radio);
#else
    fprintf(stderr, "radio: USRP B210 backend not compiled in. "
            "Re-run cmake with -DWITH_USRP_B210=ON (requires UHD).\n");
    return RADIO_NOT_IMPLEMENTED;
#endif
}

static void b210_disconnect_stub(radio_t *radio)
{
#ifdef WITH_USRP_B210
    b210_disconnect(radio);
#else
    (void)radio;
#endif
}

static const radio_backend_ops_t usrp_b210_ops = {
    .name                  = "usrp-b210",
    .init                  = b210_init_stub,
    .disconnect            = b210_disconnect_stub,
    .command               = NULL,
    .set_vfo               = NULL,
#ifdef WITH_USRP_B210
    .get_frequency         = b210_get_frequency,
    .set_frequency         = b210_set_frequency,
#else
    .get_frequency         = NULL,
    .set_frequency         = NULL,
#endif
    .get_satellite_mode    = NULL,
    .set_satellite_mode    = NULL,
    .set_mode              = NULL,
    .set_data_mode         = NULL,
    .set_data_mod_source   = NULL,
    .set_rx_clean          = NULL,
    .set_usb_mod_level     = NULL,
    .set_moni_level        = NULL,
#ifdef WITH_USRP_B210
    .set_rf_power          = b210_set_rf_power,
#else
    .set_rf_power          = NULL,
#endif
    .set_squelch           = NULL,
    .get_squelch           = NULL,
    .cat_send              = NULL,
    .set_rf_power_watts    = NULL,
#ifdef WITH_USRP_B210
    .ptt                   = b210_ptt,
#else
    .ptt                   = NULL,
#endif
    .power                 = NULL,
    .get_band_selection    = NULL,
    .set_band_selection    = NULL,
    .toggle_waterfall      = NULL,
};

const radio_backend_ops_t *radio_backend_usrp_b210_ops(void)
{
    return &usrp_b210_ops;
}
