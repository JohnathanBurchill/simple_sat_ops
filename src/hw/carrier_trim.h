/*

    Simple Satellite Operations  src/hw/carrier_trim.h

    Persistent carrier-trim calibration. The B210's TCXO drifts by a
    few ppm, which lands on the RX baseband as a fixed Hz-scale bias
    that UHD doesn't know about (it reports actual_freq as whatever
    the AD9361 PLL was asked to land on, not what it actually emits
    once the reference is off). Operators measure the residual once
    on a known carrier and stash it in
    ~/.local/share/simple_sat_ops/carrier-trim-hz; every B210-using
    program reads it at startup and folds it into the LO-cancel NCO
    so the carrier sits at DC.

    File format: one decimal Hz value (signed) on a single line.
    Sign matches the additive correction to the fm_lo_nco frequency
    — if the carrier sits at -700 Hz after the UHD-residual fix, the
    file should contain "-700".

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

#ifndef SSO_CARRIER_TRIM_H
#define SSO_CARRIER_TRIM_H

#ifdef __cplusplus
extern "C" {
#endif

// Read the persistent carrier-trim file and return its value in Hz.
// If the file does not exist, create it with "0\n" and return 0.0.
// On any I/O or parse error, logs to stderr and returns 0.0 so the
// caller can proceed without calibration. Safe to call once per
// process at startup.
double carrier_trim_load_hz(void);

// Absolute path of the trim file (in a static buffer; not thread-safe).
// Exposed so programs can mention the location in --help / status text.
const char *carrier_trim_path(void);

#ifdef __cplusplus
}
#endif

#endif
