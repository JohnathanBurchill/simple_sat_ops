/*

    Simple Satellite Operations  golay24.h

    Extended Golay(24,12) encoder used by the AX100 framer to protect
    the 12-bit length field. Systematic form: output = (parity << 12) | data.
    Matches the H-matrix in pycsplink.Golay24.

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

#ifndef GOLAY24_H
#define GOLAY24_H

#include <stdint.h>

// Encodes a 12-bit value (0..4095) into a 24-bit Golay codeword.
// Output layout: (12-bit parity << 12) | (12-bit data).
uint32_t golay24_encode(uint16_t data12);

#endif // GOLAY24_H
