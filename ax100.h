/*

    Simple Satellite Operations  ax100.h

    AX100 frame encoder matching the CalgaryToSpace pycsplink.AX100 encoder:

      [32x 0xAA preamble]
      [4-byte sync word 0x93 0x0B 0x51 0xDE]
      [3-byte Golay(12,24)-encoded length]
      [CCSDS scrambled payload (payload = CSP_packet || optional HMAC trailer)]
      [1x 0xAA postamble]

    Reed-Solomon (255,223) and CRC-32C are not yet implemented; they are
    off by default in the reference and not used by pycsp_tx.py.

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

#ifndef AX100_H
#define AX100_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

// AX100 sync word (attached sync marker) — big-endian bytes on the wire.
#define AX100_ASM_0 0x93u
#define AX100_ASM_1 0x0Bu
#define AX100_ASM_2 0x51u
#define AX100_ASM_3 0xDEu

typedef struct ax100_opts {
    // If non-NULL, a 4-byte HMAC trailer (HMAC-SHA1, truncated) is appended
    // to the packet before scrambling. Key is first SHA-1'd, first 16 bytes
    // used as the effective key (matches pycsp.HMACEngine).
    const uint8_t *hmac_key;
    size_t hmac_key_len;

    int randomize;   // 1: apply CCSDS scrambler (default); 0: skip
    int len_field;   // 1: prepend 3-byte Golay24 length (default); 0: skip
    int syncword;    // 1: prepend 4-byte ASM (default); 0: skip
    int prefill;     // number of 0xAA preamble bytes (default 32)
    int tailfill;    // number of 0xAA postamble bytes (default 1)
} ax100_opts_t;

// Initializes opts to the pycsplink defaults (randomize=1, len_field=1,
// syncword=1, prefill=32, tailfill=1, no HMAC).
void ax100_opts_defaults(ax100_opts_t *opts);

// Computes the AX100 HMAC trailer (4 bytes) for the given data.
//   out_trailer: 4-byte buffer.
// Returns 0 on success, -1 on error.
int ax100_hmac(const uint8_t *key, size_t key_len,
               const uint8_t *data, size_t data_len,
               uint8_t out_trailer[4]);

// Frames a CSP packet for the AX100. Returns total frame length written to
// out_buf, or -1 on error. Writes (prefill + 4 + 3 + payload_len + 4 + tailfill)
// bytes in the worst case (HMAC on, all options on).
//
// `packet` is the raw CSP packet bytes (header + payload).
ssize_t ax100_frame(const uint8_t *packet, size_t packet_len,
                    const ax100_opts_t *opts,
                    uint8_t *out_buf, size_t out_buf_size);

// Reverse of ax100_frame. Input `bytes` must start AT the 4-byte ASM sync
// word — the decoder's timing-recovery stage finds that in the bit stream
// and byte-aligns the remainder.
//
// Honors the same ax100_opts_t used on the encode side: if opts->len_field
// is set, the 3-byte Golay24-encoded length is consumed; if opts->randomize,
// the payload is descrambled; if opts->hmac_key, the trailing 4-byte HMAC
// is verified.
//
// Writes the inner CSP packet (with HMAC trailer stripped, if applicable)
// to out_packet. On success returns the inner packet length.
// *out_golay_errors (optional) gets 0..3 on success, or the uncorrectable
// bit count (>3) on failure.
// *out_hmac_ok (optional) is 1 if the HMAC matched, 0 if it did not, or
// -1 if HMAC verification wasn't attempted (no key supplied).
//
// Returns -1 on any fatal error (missing ASM, uncorrectable Golay, not
// enough bytes to satisfy the decoded length, invalid args).
ssize_t ax100_unframe(const uint8_t *bytes, size_t n_bytes,
                      const ax100_opts_t *opts,
                      uint8_t *out_packet, size_t out_packet_cap,
                      int *out_golay_errors,
                      int *out_hmac_ok);

#endif // AX100_H
