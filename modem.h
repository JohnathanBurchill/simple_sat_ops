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

// Demodulator: recover bit stream from mono int16 PCM and find the AX100
// ASM (0x930B51DE) sync word. Pipeline:
//   1. High-pass (DC-block, α=0.995 IIR) — defeats rtl_fm discriminator drift
//   2. Bipolar threshold at 0: sample mid-bit, bit = (x > 0)
//   3. Try all sps = samp_rate/bit_rate phase offsets; pick the one whose
//      resulting bit stream contains the ASM within sync_max_ham bit errors.
//   4. Copy the bit stream starting AT the ASM into out_bits; record the
//      bit-stream start offset for diagnostics.
//
// invert_polarity: flip the threshold sign (bit = (x < 0)). Use when FM
// deviation polarity is inverted end-to-end.
// sync_max_ham: 0 = strict match, 1..8 = tolerate up to that many errors
// in the 32-bit ASM.
//
// out_bits: caller-allocated, at least (n_samples/sps - sync_offset) bits.
// *n_bits_out: on success, number of bits written.
// *sync_bit_offset: on success, where in the pre-alignment raw bit stream
// the ASM was found (bit 0 of ASM = first bit copied to out_bits).
//
// Returns 0 on success, -1 on no sync or bad args.
int modem_pcm16_to_bits(const int16_t *samples, size_t n_samples,
                        const modem_params_t *p,
                        int invert_polarity,
                        int sync_max_ham,
                        uint8_t *out_bits, size_t *n_bits_out,
                        size_t *sync_bit_offset);

// Pack a bit stream (bit[i] = 0 or 1, MSB-first) into bytes. Last partial
// byte is zero-padded on the LSB side. n_bits need not be a multiple of 8.
// Returns the number of bytes written (ceil(n_bits / 8)).
size_t modem_bits_to_bytes(const uint8_t *bits, size_t n_bits, uint8_t *out);

#endif // MODEM_H
