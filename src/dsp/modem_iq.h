/*

    Simple Satellite Operations  modem_iq.h

    Coherent / quasi-coherent AX100 demodulator operating directly on
    interleaved int16 I,Q baseband samples — the post-FIR IQ stream
    that the discriminator path consumes in b210_rx_tx_core. Sits next
    to modem_pcm16_to_bits as a parallel decode chain so the operator
    can A/B-compare against the existing FM-audio path on the same RF.

    Pipeline (first cut — pre-Viterbi):

      1. AGC on the complex baseband (divide by RMS magnitude).
      2. Boxcar matched filter of length sps on I and Q separately.
         For roughly-NRZ-shaped 9600 GFSK at sps=5 this captures most
         of the matched-filter SNR gain (~7 dB) but does it BEFORE the
         non-linear arg(), where the noise is still Gaussian.
      3. Per-sample differential phase: arg(z[k]·conj(z[k-1])). At
         modulation index h=0.5 the noise-free per-sample phase advance
         is ±πh/sps; integrated across a symbol period the differential
         phase is ±π/2.
      4. Same Mueller-Müller decision-directed timing recovery as
         modem_pcm16_to_bits — strobe the differential-phase stream at
         the symbol rate.
      5. Hard slicer at 0 under each polarity; ASM (0x930B51DE) search
         with up to sync_max_ham bit errors; return the lowest-Hamming
         match.

    A follow-up adds a Costas-style phase tracker plus a 4-state
    Viterbi MLSE decoder to wring another ~1–2 dB out of the GMSK
    pulse memory. Kept out of the first commit so the simpler chain
    can be validated on real-RF captures first.

    Copyright (C) 2026  Johnathan K Burchill

    GPLv3 or later.
*/

#ifndef MODEM_IQ_H
#define MODEM_IQ_H

#include "modem.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Demodulator: recover bits from interleaved int16 I,Q baseband and
// find the AX100 ASM. Mirrors modem_pcm16_to_bits's contract so
// try_decode_window can swap chains at the call site.
//
//   iq_pairs:        interleaved I,Q pairs; total int16 count = 2 * n_pairs
//   n_pairs:         number of complex samples
//   p:               same modem_params_t the PCM chain uses; samp_rate is
//                    the post-decim IQ rate (e.g. 48000), bit_rate is 9600
//   invert_polarity: try this polarity first (0 normal, 1 inverted)
//   sync_max_ham:    0..8 — tolerate this many ASM bit errors
//   min_bit_offset:  earliest bit index to consider for sync
//   out_bits:        caller buffer; at least (n_pairs/sps - offset) bits
//   *n_bits_out:     bits written
//   *sync_bit_offset: position of ASM bit 0 in the raw bit stream
//   *polarity_used:  0/1 (optional)
//
// Returns 0 on success, -1 on no sync / bad args.
int modem_iq_to_bits(const int16_t *iq_pairs, size_t n_pairs,
                     const modem_params_t *p,
                     int invert_polarity,
                     int sync_max_ham,
                     size_t min_bit_offset,
                     uint8_t *out_bits, size_t *n_bits_out,
                     size_t *sync_bit_offset,
                     int *polarity_used);

#ifdef __cplusplus
}
#endif

#endif // MODEM_IQ_H
