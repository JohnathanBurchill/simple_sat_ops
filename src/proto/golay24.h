/*

    Simple Satellite Operations  golay24.h

    Extended Golay(24,12) encoder used by the AX100 framer to protect
    the 12-bit length field. Systematic form: output = (parity << 12) | data.
    Matches the H-matrix in pycsplink.Golay24.

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

#ifndef GOLAY24_H
#define GOLAY24_H

#include <stdint.h>

// Encodes a 12-bit value (0..4095) into a 24-bit Golay codeword.
// Output layout: (12-bit parity << 12) | (12-bit data).
uint32_t golay24_encode(uint16_t data12);

// Decodes a 24-bit Golay codeword back to its 12-bit data value, correcting
// up to 3 bit errors. Implementation is a brute-force minimum-Hamming-
// distance search over all 4096 codewords — trivial for this code size and
// easy to convince yourself is correct. On success returns 0, writes the
// recovered 12-bit data to *out_data12, and (if non-NULL) writes the number
// of bit errors (0..3) to *out_errors_corrected. Returns -1 if the closest
// codeword differs by > 3 bits (uncorrectable).
int golay24_decode(uint32_t word24, uint16_t *out_data12,
                   int *out_errors_corrected);

#endif // GOLAY24_H
