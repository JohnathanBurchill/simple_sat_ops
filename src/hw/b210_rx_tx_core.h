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

    // FM-demod LO compensation. With a non-zero LO offset (the
    // hardware LO is tuned off the nominal carrier to dodge the DC
    // null), the Doppler-corrected signal sits at +lo_offset_hz of
    // baseband — well outside the ±fm_fullscale_hz the discriminator
    // is calibrated for. The result is a clipped, asymmetric FSK
    // waveform that won't bit-slice cleanly.
    //
    // The pump applies a SECOND NCO to the post-decim/post-Doppler
    // stream IN PLACE that cancels both this lo_offset AND the UHD-
    // reported tune residual (target_freq − actual_freq, from the
    // AD9361's PLL step), so the carrier lands at exactly DC. The
    // rotation happens BEFORE the IQ tap so every downstream consumer
    // (.iq sidecar, live waterfall, shadow IQ decoder, FM
    // discriminator) sees the centered signal. The spectrum is
    // periodic in fs so the rotation wraps the far edge — for a
    // single-carrier sat that's invisible; for a noisy neighbour at
    // original +/-(fs/2 - lo_offset) baseband it'll show up mirrored
    // on the opposite edge.
    //
    // Pass the operator's lo_offset_hz here (i.e. nominal − actual_LO).
    // The tune residual is read back from UHD on every set_freq and
    // folded in automatically.
    //
    // 0 lo_offset with 0 residual → NCO disabled. Otherwise it stays
    // active to cancel whatever residual is left.
    double      fm_lo_compensation_hz;

    // AD9361 automatic DC-offset tracking + IQ-balance tracking. Both
    // run as periodic background loops inside the AD9361 driver: each
    // tick steps an IIR notch / a correction register, which kicks an
    // impulsive transient into the IQ stream. At ~51 Hz update rate
    // those transients look like a comb of vertical spikes in the
    // waterfall — visible at moderate gain, suppressed at gain=0 (no
    // input to chase) and at very high gain (loop saturates and the
    // driver bails on convergence). FM signals near DC fall inside
    // the tracking bandwidth and make the artifact worse.
    //
    // Pass false to disable. Tradeoff: a small static DC bias and
    // residual IQ imbalance remain in the captured IQ, both of which
    // the rest of the chain tolerates (carrier parked off-DC at
    // +lo_offset_hz, FM disc is per-sample, decoder is differential
    // phase). UHD's default is true on B210.
    int         rx_dc_offset_track;
    int         rx_iq_balance_track;

    // Persistent per-host carrier-trim calibration. Absorbs whatever's
    // left after the UHD-reported tune residual is cancelled — in
    // practice this is the B210 TCXO error (a few ppm × tuned RF =
    // hundreds of Hz at 70 cm). Loaded once at program startup from
    // ~/.local/share/simple_sat_ops/carrier-trim-hz (see carrier_trim.h);
    // pass 0 if you don't want a trim. Added on top of the operator
    // offset and the UHD residual inside the second NCO.
    double      carrier_trim_hz;
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
// Two optional IQ taps on the post-decimation stream:
//
//   * iq_raw_out: post-Doppler, PRE-fm_lo_nco. The carrier sits at
//     +lo_offset_hz baseband (i.e. wherever the operator's LO offset
//     placed it). This is what gets written to the .iq sidecar so the
//     waterfall shows the beacon away from DC and the operator's
//     offset choice survives offline replay.
//
//   * iq_decode_out: post-Doppler, POST-fm_lo_nco. The carrier sits at
//     DC, which is what the FM discriminator's calibration and the
//     shadow IQ decoder both expect. This is the same shape the old
//     single-tap output had, just plumbed out separately.
//
// Each tap copies up to (cap / 2) IQ pairs (interleaved I,Q int16) and
// reports the pair count in *out_*_pairs. Either tap may be NULL with
// the corresponding cap = 0; pass nulls for both to keep the existing
// PCM-only behaviour.
//
// Return:
//   > 0  number of PCM samples written
//   = 0  transient UHD error (overflow / timeout) — keep looping
//   < 0  fatal UHD error — bail
ssize_t b210_rx_tx_core_pump(b210_rx_tx_core_t *core,
                          int16_t *pcm_out, size_t pcm_cap,
                          int16_t *iq_raw_out,    size_t iq_raw_cap,
                          size_t  *out_iq_raw_pairs,
                          int16_t *iq_decode_out, size_t iq_decode_cap,
                          size_t  *out_iq_decode_pairs);

// Issue a tune request on RX channel 0. The streamer keeps running.
// Return: 0 on success, -1 on UHD error.
int b210_rx_tx_core_set_freq(b210_rx_tx_core_t *core, double freq_hz);

// Update the AD9361 RX gain on RX channel 0. The streamer keeps
// running; the AD9361 accepts the change inline and the next pump
// returns samples at the new gain after a brief settle.  Clipped
// to [0, 76] dB by the AD9361 itself; pass values outside that and
// UHD will coerce / warn.  Return: 0 on success, -1 on UHD error.
int b210_rx_tx_core_set_gain(b210_rx_tx_core_t *core, double gain_db);

// Software Doppler NCO. After the FIR decimator and before the IQ tap /
// FM discriminator, every sample is multiplied by exp(-j 2π Δf · t)
// so a carrier sitting at offset Δf Hz from the LO is rotated back to
// DC. The phase accumulator is continuous across pump calls and across
// frequency updates — set_doppler_offset DOES NOT reset the phase, so
// smooth Doppler trajectories produce a phase-coherent baseband signal.
//
// Pass 0.0 to disable (the multiply turns into a no-op fast path).
void   b210_rx_tx_core_set_doppler_offset(b210_rx_tx_core_t *core,
                                          double offset_hz);
double b210_rx_tx_core_get_doppler_offset(const b210_rx_tx_core_t *core);

// Update the FM-path LO-compensation NCO at runtime. lo_offset_hz is
// the SIGNED operator offset of the hardware LO from nominal (the
// same value plumbed via fm_lo_compensation_hz at open). Used by the
// :lo_offset colon command to chase a noisier baseband band — pair
// with a hardware retune via b210_rx_tx_core_set_freq so the SDR LO
// and the demod-path compensation stay consistent. The UHD tune
// residual is refreshed automatically inside set_freq and stays
// folded into the NCO across this call. 0 lo_offset + 0 residual →
// the NCO goes dormant; non-zero residual keeps it running.
void b210_rx_tx_core_set_fm_lo_compensation(b210_rx_tx_core_t *core,
                                            double lo_offset_hz);

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

// Broadband-burst snapshot from the iq_burst detector running on the
// post-NCO IQ. out_bright_bins: number of FFT bins above floor +
// threshold in the latest completed frame — a CW carrier reads 1-6
// (target bin + Hann sidelobes), wideband packets read tens to
// hundreds, stationary noise reads ~5-30 (false-positive rate of the
// 10 dB threshold). out_peak_excess_db: brightest bin's dB excess
// above its floor. Either pointer may be NULL. Returns 0 on success,
// -1 if core is NULL.
int b210_rx_tx_core_burst_snapshot(const b210_rx_tx_core_t *core,
                                   int *out_bright_bins,
                                   double *out_peak_excess_db);

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
//
// RX-only backends (e.g. RTL-SDR) have no transmit path: this returns
// -1 and logs. Callers should gate on b210_rx_tx_core_can_tx() first.
int b210_rx_tx_core_burst(b210_rx_tx_core_t *core,
                          const b210_rx_tx_core_burst_params_t *p);

// 1 if the active SDR backend can transmit, 0 if it is RX-only (or core
// is NULL). The TX UI and the burst path gate on this.
int b210_rx_tx_core_can_tx(const b210_rx_tx_core_t *core);

#endif // B210_RX_TX_CORE_H
