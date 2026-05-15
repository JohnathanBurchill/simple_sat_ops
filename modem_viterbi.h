/*

    Simple Satellite Operations  modem_viterbi.h

    MSK-MLSE Viterbi demodulator. Same int16 I,Q baseband input and
    same output contract as modem_iq_to_bits, but the per-symbol slicer
    is replaced by a maximum-likelihood sequence detector running on a
    4-state CPM phase trellis (states ∈ {0, π/2, π, 3π/2} — the natural
    MSK alphabet at h=1/2).

    Pipeline:
      1. AGC on the complex baseband (RMS-normalise).
      2. Boxcar matched filter of length sps on I and Q separately.
      3. Symbol-rate differential phase: arg(z_mf[i+sps]·conj(z_mf[i]))
         — identical to modem_iq.c so the timing-recovery loop sees the
         same signal that already works there.
      4. Mueller-Müller decision-directed timing. As a side product the
         loop also strobes z_mf[strobe + sps] (with the same fractional
         interpolation) so we get a clean symbol-rate complex sequence
         y[n] to feed the Viterbi.
      5. Carrier-phase estimate via fourth-power: for MSK at h=1/2,
         y[n]^4 has constant phase 4·φ₀ irrespective of the data bits,
         so φ̂₀ = arg(Σ y[n]^4) / 4. The 4-fold ambiguity in φ̂₀ is
         irrelevant — Viterbi transitions are invariant under a rotation
         of the absolute phase reference, so only the residual polarity
         is left, and that is resolved the usual way (ASM-search under
         both polarities).
      6. Viterbi on 4 phase states with branch metric
            bm(s) = Re{y[n] · exp(-j·s·π/2)}
         and predecessors (s+3)%4 [via +π/2 transition] and (s+1)%4
         [via -π/2 transition]. Path metrics carry over symbols, back-
         tracking yields the bit decisions.

    For pure-AWGN MSK this picks up most of the ~3 dB gap between the
    differential receiver and the coherent floor — at very low SNR the
    Viterbi additionally cleans up isolated errors that the slicer
    cannot recover.

    Same signature as modem_iq_to_bits so a caller can A/B-swap chains
    at the call site.

    Copyright (C) 2026  Johnathan K Burchill

    GPLv3 or later.
*/

#ifndef MODEM_VITERBI_H
#define MODEM_VITERBI_H

#include "modem.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int modem_iq_viterbi_to_bits(const int16_t *iq_pairs, size_t n_pairs,
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

#endif // MODEM_VITERBI_H
