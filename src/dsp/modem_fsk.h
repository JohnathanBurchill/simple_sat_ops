/*

    Simple Satellite Operations  modem_fsk.h

    FSK demodulator on int16 I,Q baseband. Takes the same input format
    as modem_iq_to_bits / modem_iq_viterbi_to_bits and has the same
    output contract, but the demod chain is:

      1. FM-discriminate the IQ stream — i.e. compute instantaneous
         frequency as the differential phase between adjacent samples.
         This collapses the IQ to a scalar (the FM-demod-output WAV
         that an FM receiver would feed us if we had one).
      2. Run the same DC-block + AGC + boxcar matched-filter + M&M
         timing + slicer pipeline that modem_pcm16_to_bits uses on
         actual FM-demod audio — that pipeline is mature and works
         well on G3RUH-style scrambled FSK regardless of the
         modulation index (the slicer is sign-based on a DC-blocked
         signal, so it doesn't care whether the FSK frequency
         separation is h=0.5 or h=2/3 or h=1).

    This is the chain gr_satellites uses (quadrature-demod → clock
    sync → slicer) and is the right primary chain for FrontierSat,
    whose downlink is FSK at h≈2/3 (deviation 3200, baud 9600) rather
    than the MSK h=0.5 that modem_iq.c's commentary still assumes.

    Same signature as modem_iq_to_bits so callers can A/B-swap chains
    at the call site.

    Copyright (C) 2026  Johnathan K Burchill

    GPLv3 or later.
*/

#ifndef MODEM_FSK_H
#define MODEM_FSK_H

#include "modem.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int modem_fsk_iq_to_bits(const int16_t *iq_pairs, size_t n_pairs,
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

#endif // MODEM_FSK_H
