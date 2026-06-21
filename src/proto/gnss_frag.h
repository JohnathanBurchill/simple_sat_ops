/*

   Simple Satellite Operations  gnss_frag.h

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

#ifndef GNSS_FRAG_H
#define GNSS_FRAG_H

#include <stddef.h>

// A GNSS report is downlinked split across several tcmd_response packets.
// gnss_opm and gnss_reports both pull those fragments out of packet_db,
// group them by ts_sent, and stitch them back together; this is the shared
// fragment type and the reassembly they have in common.
typedef struct {
    long long      id;
    char           ts_received[40];
    unsigned char *payload;
    int            payload_len;
    unsigned char  tskey[8];   // ts_sent bytes, the join key for one response
    int            seq;
    int            max_seq;     // fragment count from the header (gnss_reports only)
} gnss_frag_t;

// "does s start with p?"
int gnss_starts_with(const char *s, const char *p);

// Parse a since/until spec (90s|30m|24h|7d relative, or an ISO-8601 / partial
// string passed straight through) into ISO-8601 UTC for lexicographic SQL
// comparison. Returns 0 on success, -1 on a bad spec.
int gnss_parse_time_spec(const char *spec, char *out, size_t outn);

// Reassemble one reception (fragments sorted by seq) into buf, capping each
// fragment's contribution so the trailing per-packet CSP CRC32 is dropped.
// Returns the reassembled length (NUL-terminated within bufcap).
int gnss_reassemble(const gnss_frag_t *frags, int n, unsigned char *buf, int bufcap);

#endif
