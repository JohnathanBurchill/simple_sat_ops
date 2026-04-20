/*

    Simple Satellite Operations  modem.h

    Baseband modulator for the AX100 uplink audio path:
    bytes -> MSB-first bit unpack -> oversample (sps = samp_rate/bit_rate)
    -> BPSK {0,1} -> {-1,+1} -> optional Gaussian pulse shape -> gain
    -> 16-bit mono PCM.

    Default parameters match the FrontierSat link: samp_rate=48000,
    bit_rate=9600, Gaussian BT=0.5 across 4 symbols, gain=0 dB.

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

#ifndef MODEM_H
#define MODEM_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

typedef struct modem_params {
    int samp_rate;           // output sample rate, Hz; must be multiple of bit_rate
    int bit_rate;            // baud rate on-air, bits per second
    double gain_db;          // output gain in dB; 0 means unity at the filter output
    double gauss_bt;         // Gaussian BT product; 0 disables filter (pure NRZ)
    int gauss_symbol_span;   // filter length in symbol periods (e.g. 4)
} modem_params_t;

// Initializes p to the FrontierSat defaults (9600 bps at 48 kHz,
// Gaussian BT=0.5, 4-symbol span, unity gain).
void modem_params_defaults(modem_params_t *p);

// Modulates frame bytes into mono 16-bit PCM samples at p->samp_rate.
// Writes n = frame_len * 8 * (samp_rate/bit_rate) samples to `out`.
// Returns number of samples written, or -1 on bad args / buffer too small.
ssize_t modem_bytes_to_pcm16(const uint8_t *frame, size_t frame_len,
                             const modem_params_t *p,
                             int16_t *out, size_t out_cap);

// Writes a mono 16-bit PCM WAV file (RIFF header + samples).
// Returns 0 on success, -1 on error.
int pcm16_write_wav(const char *path,
                    const int16_t *samples, size_t n_samples,
                    int samp_rate);

#endif // MODEM_H
