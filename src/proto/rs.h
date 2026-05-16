/*

    Simple Satellite Operations  rs.h

    Reed-Solomon (255, 223) codec — CCSDS conventional basis.
    Matches the Python `reed_solomon_ccsds` package (v1.0.3) when invoked
    with dual_basis=False, interleaving=1, which in turn is what
    pycsplink.AX100 uses for FrontierSat uplink frames.

    GF(2^8), primitive polynomial p(x) = x^8 + x^7 + x^2 + x + 1 (0x187).
    Code: (n=255, k=223, 2t=32 roots) — corrects up to 16 symbol errors
    per block. Fcr=112, Prim=11, IPrim=116.

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

#ifndef RS_H
#define RS_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

#define RS_N 255      // codeword length
#define RS_K 223      // data length
#define RS_NROOTS 32  // parity length

// Encode 223 data bytes to a 255-byte codeword. data_out[0..222] receives
// the data (copied from data_in); data_out[223..254] receives the parity.
void rs_encode(const uint8_t data_in[RS_K], uint8_t codeword_out[RS_N]);

// Decode a 255-byte codeword in place. On success returns the number of
// corrected byte errors (0..16). On failure (uncorrectable) returns -1.
// Only codeword[0..222] (the data portion) is guaranteed meaningful after
// decode; the parity tail is overwritten.
//
// out_locs (optional): if non-NULL, must have at least RS_NROOTS slots.
// On a non-negative return, the first `return` entries hold the codeword
// indices (0..RS_N-1) of corrected bytes, in the order the Forney
// algorithm produced them.
int rs_decode(uint8_t codeword[RS_N], int *out_locs);

// pycsplink-compatible encode: left-zero-pad input to 223 bytes, RS-encode,
// strip the leading padding. Output length = in_len + 32 (the 32-byte
// parity always tacked on). in_len must be <= 223.
// Returns output length on success, -1 on bad args.
ssize_t rs_pycsp_encode(const uint8_t *in, size_t in_len,
                        uint8_t *out, size_t out_cap);

// pycsplink-compatible decode: left-zero-pad input to 255 bytes, RS-decode,
// strip the leading padding and parity. Output length = in_len - 32.
// in_len must satisfy 32 < in_len <= 255.
// *out_errors (optional): number of corrected byte errors.
// out_locs (optional): if non-NULL, must have at least RS_NROOTS slots.
// On success, the first *out_errors entries hold byte offsets relative
// to the start of `in` (range 0..in_len-1; the last 32 are the RS parity
// tail). RS-solver false-corrections that land in the synthetic
// zero-pad region are emitted as negative offsets so the operator can
// flag them; this is rare in practice.
// Returns output length on success, -1 on uncorrectable / bad args.
ssize_t rs_pycsp_decode(const uint8_t *in, size_t in_len,
                        uint8_t *out, size_t out_cap,
                        int *out_errors,
                        int *out_locs);

#endif // RS_H
