/*

    Simple Satellite Operations  ax100.h

    AX100 frame encoder matching the CalgaryToSpace pycsplink.AX100 encoder:

      [32x 0xAA preamble]
      [4-byte sync word 0x93 0x0B 0x51 0xDE]
      [3-byte Golay(12,24)-encoded length]
      [CCSDS scrambled payload (payload = CSP_packet || optional HMAC
        trailer || optional 32-byte Reed-Solomon parity)]
      [1x 0xAA postamble]

    Reed-Solomon (255,223) is implemented (see rs.c) and matches
    reed_solomon_ccsds.encode(x, False, 1) semantics: left-zero-pad to 223,
    encode to 255, strip padding. Enabled via ax100_opts_t.reed_solomon.
    CRC-32C is still not implemented — pycsplink.AX100 enables it only on
    the downlink (reed_solomon=False, crc=True), which is not our TX path.

    Copyright (C) 2025, 2026  Johnathan K Burchill

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
    int reed_solomon;// 1: RS(255,223) encode post-HMAC, decode pre-HMAC;
                     // 0: skip (default, for backwards compatibility).
                     // pycsplink uplink enables this; enable it whenever
                     // TX and RX sides agree.
    int len_field;   // 1: prepend 3-byte Golay24 length (default); 0: skip
    int syncword;    // 1: prepend 4-byte ASM (default); 0: skip
    int prefill;     // number of 0xAA preamble bytes (default 32)
    int tailfill;    // number of 0xAA postamble bytes (default 1)
} ax100_opts_t;

// Initializes opts to the pycsplink defaults (randomize=1, len_field=1,
// syncword=1, prefill=32, tailfill=1, no HMAC, no Reed-Solomon).
// Callers that want the pycsp uplink profile should also set
// opts->reed_solomon = 1 after calling this.
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
// Honors the same ax100_opts_t used on the encode side: Golay24 length,
// scrambler, HMAC, and Reed-Solomon are each toggled by the corresponding
// opts field.
//
// When opts->reed_solomon AND opts->hmac_key are BOTH set, and the frame
// length decoded from the Golay header does not produce a valid payload
// (RS uncorrectable, or RS ok but HMAC mismatch), ax100_unframe brute-
// forces alternative inner_len candidates in [33, 255] and accepts the
// first one that RS-decodes cleanly and HMAC-validates. This recovers
// frames where the 3-byte Golay header took >3 bit errors and misdecoded
// the length field — an outcome RS alone cannot fix because RS sits
// inside the Golay-length envelope.
//
// Writes the inner CSP packet (with HMAC trailer and RS parity stripped,
// if applicable) to out_packet. On success returns the inner packet length.
// *out_golay_errors (optional) gets 0..3 on success, or the uncorrectable
// bit count (>3) on failure.
// *out_hmac_ok (optional) is 1 if the HMAC matched, 0 if it did not, or
// -1 if HMAC verification wasn't attempted (no key supplied).
// *out_rs_errors (optional) gets the number of byte errors RS corrected
// (0..16); unused (-1) if opts->reed_solomon is not set.
// *out_used_golay_len (optional) is 1 if the Golay-decoded length was
// accepted, 0 if the brute-force length search found a different length,
// -1 if not applicable. Useful for debugging weak-signal captures.
// out_rs_locs (optional): if non-NULL, must have at least 32 slots
// (RS_NROOTS). On success with reed_solomon enabled, the first
// *out_rs_errors entries hold byte offsets of corrected bytes relative
// to the start of the on-wire scrambled payload (i.e., the byte right
// after the Golay24 length header). The last 32 of those positions are
// the RS parity tail; lower positions are the data portion. Negative
// offsets indicate RS placed a (likely false) correction in the
// synthetic zero-pad region. Useful for spotting timing-drift signatures
// (errors clustering at high offsets) vs. uniform BER.
//
// Returns -1 on any fatal error (missing ASM, uncorrectable Golay AND
// no RS fallback, no candidate satisfying RS+HMAC, invalid args).
ssize_t ax100_unframe(const uint8_t *bytes, size_t n_bytes,
                      const ax100_opts_t *opts,
                      uint8_t *out_packet, size_t out_packet_cap,
                      int *out_golay_errors,
                      int *out_hmac_ok,
                      int *out_rs_errors,
                      int *out_used_golay_len,
                      int *out_rs_locs);

#endif // AX100_H
