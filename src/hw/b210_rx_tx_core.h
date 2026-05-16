/*

   Simple Satellite Operations  b210_rx_tx_core.h

   USRP B210 RX core: UHD streamer + FM-discriminator demod, packaged
   so a binary just pumps demoded PCM and asks for retunes when the
   orbit math says the carrier moved.

   Used by utils/b210_rx_tx.c. Designed so simple_sat_ops can pull
   it in later without the live binary's CLI / decode-loop scaffolding
   coming along for the ride.

   FM demod is the same per-sample atan2 discriminator as
   utils/b210_rx_capture.c — phase reference (prev_I, prev_Q) is held
   inside the core across pump calls so chunk boundaries don't pop.

   Copyright (C) 2026  Johnathan K Burchill

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

#ifndef B210_RX_TX_CORE_H
#define B210_RX_TX_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

typedef struct b210_rx_tx_core_params {
    double      freq_hz;          // initial center freq
    double      rate_hz;          // requested sample rate (UHD will coerce)
    double      gain_db;          // RX gain
    double      bw_hz;            // analog filter bw; <0 → use rate_hz
    double      fm_fullscale_hz;  // PCM scale: ±this Hz → ±32767 (0/neg → 25 kHz)
    const char *device_args;      // NULL → "type=b200"
    const char *rx_antenna;       // NULL → "RX2" (RF A RX-only)

    // Optional digital IQ decimator. Inserted between the UHD streamer
    // and the FM discriminator: a Hamming-windowed-sinc lowpass FIR
    // narrows the IQ to (rate_hz / decim_factor), drastically reducing
    // out-of-band noise that would otherwise smear the discriminator
    // output. Without this stage, raw 240 kHz IQ + atan2 buries any
    // 9600 GMSK carrier under ~14 dB of avoidable noise.
    //
    // decim_factor:    integer ≥ 1; 0 or 1 → no decimation
    // decim_cutoff_hz: FIR -6 dB cutoff at the input rate; <=0 →
    //                  default 0.4 * (rate_hz / decim_factor)
    // decim_taps:      number of FIR taps; 0 → default 96
    unsigned    decim_factor;
    double      decim_cutoff_hz;
    unsigned    decim_taps;
} b210_rx_tx_core_params_t;

typedef struct b210_rx_tx_core b210_rx_tx_core_t;

// Open the device, set rate/gain/bw/antenna/freq, build the RX streamer,
// and issue START_CONTINUOUS. On success *out is non-NULL; on failure
// any partial state is cleaned up and *out is NULL.
//
// Return: 0 on success, -1 on UHD error (stderr already has detail).
int b210_rx_tx_core_open(const b210_rx_tx_core_params_t *p, b210_rx_tx_core_t **out);

// Stop the continuous stream and release all UHD handles.
void b210_rx_tx_core_close(b210_rx_tx_core_t *core);

// Pump up to one UHD recv chunk and FM-demod it into pcm_out. Maintains
// the prev-IQ phase reference across calls. pcm_cap should be at least
// b210_rx_tx_core_max_chunk(core) — short caps just truncate the chunk.
//
// iq_out is an optional tap on the post-decimation IQ stream — the same
// samples the FM discriminator runs on. When non-NULL, the call copies
// up to (iq_cap / 2) IQ pairs (interleaved I,Q int16) into the buffer,
// returning the IQ count in *out_iq_pairs. Pass NULL for both iq_out
// and out_iq_pairs to keep the existing PCM-only behaviour. Use this
// to write a sidecar IQ recording without re-running the FIR.
//
// Return:
//   > 0  number of PCM samples written
//   = 0  transient UHD error (overflow / timeout) — keep looping
//   < 0  fatal UHD error — bail
ssize_t b210_rx_tx_core_pump(b210_rx_tx_core_t *core,
                          int16_t *pcm_out, size_t pcm_cap,
                          int16_t *iq_out, size_t iq_cap,
                          size_t *out_iq_pairs);

// Issue a tune request on RX channel 0. The streamer keeps running.
// Return: 0 on success, -1 on UHD error.
int b210_rx_tx_core_set_freq(b210_rx_tx_core_t *core, double freq_hz);

// Read-back accessors (after b210_rx_tx_core_open succeeded).
//
// actual_rate:  post-decimation rate (== input_rate when decim_factor is
//               <=1). This is what the FM-demodded PCM stream is at, so
//               downstream modems / WAV writers should use it.
// input_rate:   UHD's coerced sample rate (pre-decimation). Same as
//               actual_rate when no decimation is configured.
// max_chunk:    upper bound on PCM samples produced by one pump call.
//               Already accounts for decimation.
double b210_rx_tx_core_actual_rate(const b210_rx_tx_core_t *core);
double b210_rx_tx_core_input_rate (const b210_rx_tx_core_t *core);
double b210_rx_tx_core_actual_freq(const b210_rx_tx_core_t *core);
size_t b210_rx_tx_core_max_chunk  (const b210_rx_tx_core_t *core);

// Post-FIR IQ level snapshot. Both values are sc16-scale (full-scale
// PCM amplitude at ±32767), updated every pump:
//
//   peak_env_out:  fast-attack / slow-release envelope of max(|I|,|Q|).
//                  Jumps instantly on incoming signal, decays at a
//                  ~0.5 s exponential time constant. Driven by the
//                  in-band IQ stream (post-FIR), so out-of-band
//                  carriers don't inflate it.
//   rms_sq_out:    smoothed I²+Q² (~30 ms time constant). Take sqrt
//                  for an RMS magnitude in PCM units.
//
// Either pointer may be NULL. Returns 0 on success, -1 if core is NULL.
int b210_rx_tx_core_iq_levels(const b210_rx_tx_core_t *core,
                           double *peak_env_out, double *rms_sq_out);

typedef struct b210_rx_tx_core_burst_params {
    const int16_t *iq;          // pre-built sc16 interleaved, n_samps pairs
    size_t         n_samps;
    double         tx_rate_hz;
    double         tx_freq_hz;
    double         tx_gain_db;
    double         start_delay_s;
    double         rx_resume_freq_hz;  // RX center freq to restore after TX
} b210_rx_tx_core_burst_params_t;

// Half-duplex switch-over: stop RX, drain residue, lazy-build the TX
// streamer (cached for the daemon lifetime), set freq/gain/rate, send
// the IQ burst in max-chunk pieces with SOB on the first and EOB on
// the last, drain the FPGA FIFO, restore RX freq, restart the RX
// stream. Returns 0 on success, -1 on UHD/streamer error (stderr has
// detail). RX is left running on return regardless of outcome.
int b210_rx_tx_core_burst(b210_rx_tx_core_t *core,
                          const b210_rx_tx_core_burst_params_t *p);

#endif // B210_RX_TX_CORE_H
