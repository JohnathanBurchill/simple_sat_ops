/*

    Simple Satellite Operations  modem.h

    Baseband modulator for the AX100 uplink audio path:
    bytes -> MSB-first bit unpack -> oversample (sps = samp_rate/bit_rate)
    -> BPSK {0,1} -> {-1,+1} -> optional Gaussian pulse shape -> gain
    -> 16-bit mono PCM.

    Default parameters match the FrontierSat link: samp_rate=48000,
    bit_rate=9600, no Gaussian filter (rectangular NRZ — the gr-satellites
    fsk_demodulator the gold-reference receiver uses is matched to NRZ
    FSK), gain=0 dB.

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
    // RX-side DC-block bypass. The default IIR high-pass (α=0.995) was
    // added to defeat rtl_fm's discriminator drift, but it also adds
    // group delay and per-burst baseline transients that can shift
    // bit-slicer thresholds enough to cost a few Hamming bits in the
    // ASM detector. For radio paths with no DC offset (e.g. FT-991A
    // discriminator output), set this to 1 to slice the raw samples
    // directly. Default 0 (DC-block enabled).
    int rx_disable_dc_block;
    // IQ-chain (modem_fsk) low-pass cutoff in Hz. 0 = use the built-in
    // default (12 kHz, or the $FSK_IQ_LPF_HZ override for offline sweeps).
    // Surfaced here so the cutoff is a visible parameter rather than a knob
    // buried in a DSP leaf. Unused by the PCM/Viterbi chains.
    double fsk_iq_lpf_hz;
} modem_params_t;

// Initializes p to the FrontierSat defaults (9600 bps at 48 kHz,
// no Gaussian filter (rectangular NRZ), 4-symbol span (irrelevant
// when BT=0), unity gain).
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
//   1. High-pass (DC-block, α=0.995 IIR) — defeats discriminator drift
//   2. AGC: divide by signal RMS so downstream gains are scale-free
//   3. Matched filter: sliding boxcar of length sps = samp_rate/bit_rate.
//      For rectangular NRZ this is the integrate-and-dump optimum
//      (~10·log10(sps) dB SNR gain over single-sample slicing).
//   4. Mueller-Müller decision-directed symbol-timing recovery: one strobe
//      per symbol, linearly interpolated from the matched-filter output;
//      proportional loop filter pulls in within a few symbols.
//   5. Hard slicer at 0 (sign of the strobe sample). M&M is sign-invariant
//      under polarity flip, so a single timing loop produces the strobe
//      samples that we then slice under each polarity to handle the
//      radio-side FM-discriminator convention.
//   6. ASM search over the resulting bit stream; pick lowest-Hamming match.
//
// invert_polarity: preferred polarity to try first (0 normal, 1 inverted).
// Both are always tried; this just changes the order.
// sync_max_ham: 0 = strict match, 1..8 = tolerate up to that many errors
// in the 32-bit ASM.
// min_bit_offset: only consider sync matches at this phase-relative bit
// index or later. Set to 0 for "find first match anywhere"; bump it
// past a previously-tried position to look for the next candidate
// (used for multi-hypothesis HMAC validation in rx_decode).
//
// out_bits: caller-allocated, at least (n_samples/sps - sync_offset) bits.
// *n_bits_out: on success, number of bits written.
// *sync_bit_offset: on success, where in the pre-alignment raw bit stream
// the ASM was found (bit 0 of ASM = first bit copied to out_bits).
// *polarity_used: optional; on success, 0 if normal polarity matched, 1 if
// inverted. NULL to ignore.
//
// Returns 0 on success, -1 on no sync or bad args.
int modem_pcm16_to_bits(const int16_t *samples, size_t n_samples,
                        const modem_params_t *p,
                        int invert_polarity,
                        int sync_max_ham,
                        size_t min_bit_offset,
                        uint8_t *out_bits, size_t *n_bits_out,
                        size_t *sync_bit_offset,
                        int *polarity_used);

// Pack a bit stream (bit[i] = 0 or 1, MSB-first) into bytes. Last partial
// byte is zero-padded on the LSB side. n_bits need not be a multiple of 8.
// Returns the number of bytes written (ceil(n_bits / 8)).
size_t modem_bits_to_bytes(const uint8_t *bits, size_t n_bits, uint8_t *out);

#endif // MODEM_H
