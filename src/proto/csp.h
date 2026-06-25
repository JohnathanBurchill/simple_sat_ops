/*

    Simple Satellite Operations  csp.h

    CSP v1 packet header serialization. Matches the layout used by libcsp
    and the CalgaryToSpace pycsp reference: 4-byte big-endian header,
    followed by payload. No HMAC/CRC/XTEA at the CSP layer — those, if
    used, are applied externally (e.g. at the AX100 framing layer).

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

#ifndef CSP_H
#define CSP_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

// CSP v1 priority field
#define CSP_PRIO_CRITICAL 0
#define CSP_PRIO_HIGH     1
#define CSP_PRIO_NORM     2
#define CSP_PRIO_LOW      3

// CSP v1 flag bits (lower 4 bits of the 8-bit flags field)
#define CSP_FLAG_HMAC 0x08
#define CSP_FLAG_XTEA 0x04
#define CSP_FLAG_RDP  0x02
#define CSP_FLAG_CRC  0x01

// CSP v1 header fields. prio is 2 bits, src/dst 5 bits, dport/sport 6 bits,
// flags 8 bits (upper 4 reserved = 0, lower 4 are HMAC/XTEA/RDP/CRC).
typedef struct csp_v1_header {
    uint8_t prio;
    uint8_t src;
    uint8_t dst;
    uint8_t dport;
    uint8_t sport;
    uint8_t flags;
} csp_v1_header_t;

// Serialize a CSP v1 packet into out_buf:
//   [4-byte big-endian header][payload]
// Returns number of bytes written on success, -1 on bad args / buffer too small.
ssize_t csp_v1_encode(const csp_v1_header_t *hdr,
                      const uint8_t *payload, size_t payload_len,
                      uint8_t *out_buf, size_t out_buf_size);

// Parse the 4-byte big-endian CSP v1 header. Always fills out_hdr if bytes
// is non-NULL. Returns 0 on success, -1 on bad args.
int csp_v1_decode(const uint8_t bytes[4], csp_v1_header_t *out_hdr);

// CRC-32C (Castagnoli): polynomial 0x1EDC6F41, reflected, init 0xFFFFFFFF,
// final XOR 0xFFFFFFFF. This is libcsp's downlink CRC-mode integrity trailer
// (NOT the zlib / IEEE 802.3 CRC-32). Returns the CRC; caller compares against
// the trailing 4 bytes of the frame (try both big-endian and little-endian —
// the wire is big-endian but little-endian acceptance stays lenient).
uint32_t csp_crc32c(const uint8_t *data, size_t len);

#endif // CSP_H
