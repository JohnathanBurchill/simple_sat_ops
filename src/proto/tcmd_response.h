/*

   Simple Satellite Operations  tcmd_response.h

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

// On-wire layout of a telecommand-response (tcmd_response) downlink packet,
// plus the SQL fragments that group its packets back into one command's reply.
//
// The firmware emits one tcmd_response packet per response fragment. Every
// fragment of one command shares the same 8-byte ts_sent (the join key) and
// carries its position in response_seq_num (1..response_max_seq_num).
// Reassembling a reply means selecting every packet_type==4 row with a
// matching ts_sent, ordered by response_seq_num.
//
// These constants and offsets mirror COMMS_tcmd_response_packet_t in
// beacon_cts1.h (from firmware comms_tx.h). They are repeated here rather
// than pulled from beacon_cts1.h so the SQL/DB tools (e.g. packet_browser)
// can share the layout without including the firmware-struct header. Where
// beacon_cts1.h is also in scope, the static asserts below tie the two
// together so they can never drift apart.
//
// Header-only (static inline) so each translation unit can include it with
// no link dependency.

#ifndef TCMD_RESPONSE_H
#define TCMD_RESPONSE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Packet geometry (mirrors beacon_cts1.h / firmware comms_tx.h).
#define TCMD_RESP_PACKET_TYPE  0x04   // COMMS_PACKET_TYPE_TCMD_RESPONSE
#define TCMD_RESP_HDR_LEN      14     // type + ts_sent(8) + code + dur(2) + seq + max_seq
#define TCMD_RESP_MAX_DATA     186    // response data bytes per packet

// C byte offsets into the payload.
#define TCMD_RESP_OFF_TS_SENT  1      // 8 LE bytes: command timestamp, the join key
#define TCMD_RESP_OFF_SEQ      12     // response_seq_num (1..max)
#define TCMD_RESP_OFF_MAXSEQ   13     // response_max_seq_num

#ifdef COMMS_TCMD_RESPONSE_HEADER_SIZE
_Static_assert(TCMD_RESP_PACKET_TYPE == COMMS_PACKET_TYPE_TCMD_RESPONSE,
               "tcmd_response type disagrees with firmware header");
_Static_assert(TCMD_RESP_HDR_LEN == COMMS_TCMD_RESPONSE_HEADER_SIZE,
               "tcmd_response header size disagrees with firmware header");
_Static_assert(TCMD_RESP_MAX_DATA == COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET,
               "tcmd_response max-data disagrees with firmware header");
#endif

// SQL fragments for grouping a command's response packets. sqlite's substr()
// is 1-based, so C offset N is substr(payload, N+1, len). These string
// literals are meant for compile-time concatenation into a query, e.g.
//   "... WHERE " TCMD_RESP_SQL_IS " AND " TCMD_RESP_SQL_TS_SENT "=?1 "
//   "ORDER BY " TCMD_RESP_SQL_SEQ ", ts_received"
#define TCMD_RESP_SQL_IS       "packet_type=4"        // a tcmd_response row
#define TCMD_RESP_SQL_TS_SENT  "substr(payload,2,8)"  // the 8-byte ts_sent join key
#define TCMD_RESP_SQL_SEQ      "substr(payload,13,1)"  // response_seq_num

// Copy the 8-byte ts_sent join key out of a payload. Returns 0 on success,
// -1 if the payload is too short to contain it.
static inline int tcmd_resp_ts_sent(const uint8_t *payload, size_t len,
                                     uint8_t out_key[8])
{
    if (payload == NULL || len < TCMD_RESP_OFF_TS_SENT + 8) return -1;
    memcpy(out_key, payload + TCMD_RESP_OFF_TS_SENT, 8);
    return 0;
}

// Decode ts_sent as a little-endian uint64 (unix-ms). Returns 0 on success,
// -1 if the payload is too short.
static inline int tcmd_resp_ts_sent_u64(const uint8_t *payload, size_t len,
                                         uint64_t *out_ms)
{
    if (payload == NULL || len < TCMD_RESP_OFF_TS_SENT + 8) return -1;
    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
        v |= (uint64_t) payload[TCMD_RESP_OFF_TS_SENT + i] << (8 * i);
    *out_ms = v;
    return 0;
}

// Little-endian pack of a ts_sent unix-ms value back into its 8-byte key
// (the inverse of tcmd_resp_ts_sent_u64), for binding to the SQL join key.
static inline void tcmd_resp_key_from_u64(uint64_t ms, uint8_t out_key[8])
{
    for (int i = 0; i < 8; i++) out_key[i] = (uint8_t)(ms >> (8 * i));
}

// Decode an already-extracted 8-byte key as a little-endian uint64 (unix-ms).
static inline uint64_t tcmd_resp_key_to_u64(const uint8_t key[8])
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t) key[i] << (8 * i);
    return v;
}

// response_seq_num (1..max), or -1 if the payload is too short.
static inline int tcmd_resp_seq(const uint8_t *payload, size_t len)
{
    if (payload == NULL || len <= TCMD_RESP_OFF_SEQ) return -1;
    return payload[TCMD_RESP_OFF_SEQ];
}

// response_max_seq_num, or -1 if the payload is too short.
static inline int tcmd_resp_max_seq(const uint8_t *payload, size_t len)
{
    if (payload == NULL || len <= TCMD_RESP_OFF_MAXSEQ) return -1;
    return payload[TCMD_RESP_OFF_MAXSEQ];
}

// Response data bytes carried by one fragment: payload length minus the
// header, capped at the per-packet maximum (a full fragment may carry a
// trailing CSP CRC32 the ground leaves in). Returns 0 for a header-only or
// too-short payload.
static inline size_t tcmd_resp_data_len(size_t payload_len)
{
    if (payload_len <= TCMD_RESP_HDR_LEN) return 0;
    size_t dl = payload_len - TCMD_RESP_HDR_LEN;
    return dl > TCMD_RESP_MAX_DATA ? (size_t) TCMD_RESP_MAX_DATA : dl;
}

#endif // TCMD_RESPONSE_H
