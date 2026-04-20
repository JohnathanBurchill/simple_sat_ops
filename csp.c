/*

    Simple Satellite Operations  csp.c

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

#include "csp.h"

#include <string.h>

ssize_t csp_v1_encode(const csp_v1_header_t *hdr,
                      const uint8_t *payload, size_t payload_len,
                      uint8_t *out_buf, size_t out_buf_size)
{
    if (hdr == NULL || out_buf == NULL) {
        return -1;
    }
    if (hdr->prio  > 3  || hdr->src   > 31 || hdr->dst   > 31 ||
        hdr->dport > 63 || hdr->sport > 63) {
        return -1;
    }
    if (payload_len > 0 && payload == NULL) {
        return -1;
    }
    if (out_buf_size < 4 + payload_len) {
        return -1;
    }

    uint32_t h = 0;
    h |= ((uint32_t)(hdr->prio  & 0x03)) << 30;
    h |= ((uint32_t)(hdr->src   & 0x1F)) << 25;
    h |= ((uint32_t)(hdr->dst   & 0x1F)) << 20;
    h |= ((uint32_t)(hdr->dport & 0x3F)) << 14;
    h |= ((uint32_t)(hdr->sport & 0x3F)) << 8;
    h |= (uint32_t)hdr->flags;

    out_buf[0] = (uint8_t)((h >> 24) & 0xFF);
    out_buf[1] = (uint8_t)((h >> 16) & 0xFF);
    out_buf[2] = (uint8_t)((h >>  8) & 0xFF);
    out_buf[3] = (uint8_t)( h        & 0xFF);

    if (payload_len > 0) {
        memcpy(out_buf + 4, payload, payload_len);
    }

    return (ssize_t)(4 + payload_len);
}
