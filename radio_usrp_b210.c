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

// Ettus USRP B210 backend. SCAFFOLD ONLY.
//
// The B210 is a software-defined radio, not a transceiver. There is no
// PTT, no operating-mode switch, no DATA MOD source. To transmit AX100
// FM-DATA via the B210 we'd need an in-software FM modulator pipeline
// feeding IQ samples to UHD's TX streamer. That's a separate, larger
// project. For this branch, the B210 backend exists so:
//
//   - the abstraction in radio_backend.h is exercised by more than one
//     concrete radio (catches accidental IC-9700 assumptions in the
//     dispatcher);
//   - --radio-type=usrp-b210 produces a clean error rather than a
//     segfault until WITH_USRP_B210 is turned on and the actual UHD
//     code is dropped in here.
//
// Compilation gating: this file is always compiled, but most of its
// implementation is wrapped in an #ifdef WITH_USRP_B210 block. Without
// that build option, init() returns RADIO_NOT_IMPLEMENTED with a hint
// pointing at the CMake option.

#include "radio.h"
#include "radio_backend.h"

#include <stdio.h>

#ifdef WITH_USRP_B210
// Real UHD-backed implementation goes here. Not yet written; the project
// builds a synthetic IC-9700 / FT-991A test bed first so that the AX100
// frame and modem code is independently verified. When ready:
//   - init():           uhd::usrp::multi_usrp::make("type=b200")
//   - set_frequency():  set_tx_freq()
//   - set_rf_power():   set_tx_gain()
//   - ptt():            start/stop a TX streamer; needs an FM-modulated
//                       IQ generator to actually emit anything.
#endif

static int b210_init(radio_t *radio)
{
    (void)radio;
#ifdef WITH_USRP_B210
    fprintf(stderr, "radio: USRP B210 backend stub — operations will return "
            "RADIO_NOT_IMPLEMENTED until the UHD bring-up lands.\n");
    return RADIO_OK;
#else
    fprintf(stderr, "radio: USRP B210 backend not compiled in. "
            "Re-run cmake with -DWITH_USRP_B210=ON (requires UHD).\n");
    return RADIO_NOT_IMPLEMENTED;
#endif
}

static void b210_disconnect(radio_t *radio)
{
    (void)radio;
}

static const radio_backend_ops_t usrp_b210_ops = {
    .name                  = "usrp-b210",
    .init                  = b210_init,
    .disconnect            = b210_disconnect,
    .command               = NULL,
    .set_vfo               = NULL,
    .get_frequency         = NULL,
    .set_frequency         = NULL,
    .get_satellite_mode    = NULL,
    .set_satellite_mode    = NULL,
    .set_mode              = NULL,
    .set_data_mode         = NULL,
    .set_data_mod_source   = NULL,
    .set_usb_mod_level     = NULL,
    .set_moni_level        = NULL,
    .set_rf_power          = NULL,
    .ptt                   = NULL,
    .get_band_selection    = NULL,
    .set_band_selection    = NULL,
    .toggle_waterfall      = NULL,
};

const radio_backend_ops_t *radio_backend_usrp_b210_ops(void)
{
    return &usrp_b210_ops;
}
