/*

    Simple Satellite Operations  decode_loop.h

    Shared multi-frame AX100 decode helpers used by rx_live (ALSA-fed
    real-time decoder) and rx_replay (offline file-fed equivalent).
    The decode loop is identical in both — only the source of audio
    samples differs — so the window-walking, partial-RS rescue, and
    output formatting live here once and rx_live / rx_replay only
    drive sample acquisition.

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

#ifndef DECODE_LOOP_H
#define DECODE_LOOP_H

#include "ax100.h"
#include "modem.h"

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

// Multi-hypothesis sync search on a single window of mono int16 samples.
// Same logic as rx_decode.c's main decode loop — try sync, on failure
// advance min_offset past the bad match and try again. With HMAC enabled,
// HMAC mismatch is treated as a false-positive sync (advance + retry).
// Returns 1 on a successful decode (packet/packet_len/* filled), 0 if no
// frame found. The caller may loop on this function with min_offset_in
// advanced past the previously-returned sync_off to extract multiple
// frames per window (e.g. when bursts arrive faster than the slide
// interval); *out_sync_off receives the offset of the accepted sync
// match so the caller can compute the next min_offset_in.
//
// rs_errs sentinels (*out_rs_errs):
//   >= 0  RS-corrected; value is the count of corrected byte errors
//   -1    RS off / not exercised
//   -2    RS uncorrectable, descrambled-but-uncorrected bytes returned
//         via packet/packet_len so the operator can still see the data
//         (only happens when allow_partial_rs is on and HMAC is off).
int try_decode_window(const int16_t *samples, size_t n_samples,
                      const modem_params_t *mp,
                      const ax100_opts_t *opts,
                      int sync_max_ham, int use_hmac,
                      int allow_partial_rs,
                      size_t min_offset_in,
                      uint8_t *bits_scratch, size_t bits_cap,
                      uint8_t *bytes_scratch, size_t bytes_cap,
                      uint8_t *packet, size_t packet_cap,
                      ssize_t *out_packet_len,
                      int *out_golay_errs, int *out_hmac_ok,
                      int *out_rs_errs, int *out_used_golay_len,
                      size_t *out_sync_off);

// Append-only log line writer. Re-opens the log per frame so log
// rotation works (mv the log mid-run, next frame creates a fresh
// file). Mirrored to stdout if not quiet. ts is the timestamp string
// the caller picks — UTC wall clock for rx_live, file-relative
// "t=NN.NNNs" for rx_replay — always wrapped in [...] in the output.
void emit_frame(const char *log_path, int quiet, const char *ts,
                const uint8_t *packet, size_t packet_len,
                int golay_errs, int hmac_ok, int use_hmac,
                int rs_errs, int used_golay_len,
                int crc_status,
                uint32_t crc_computed, uint32_t crc_le, uint32_t crc_be);

#endif // DECODE_LOOP_H
