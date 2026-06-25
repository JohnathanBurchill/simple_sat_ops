/*

   Simple Satellite Operations  gnss_frag.c

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

#include "gnss_frag.h"

#include "beacon_cts1.h"
#include "tcmd_response.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

int gnss_starts_with(const char *s, const char *p)
{
    return strncmp(s, p, strlen(p)) == 0;
}

int gnss_parse_time_spec(const char *spec, char *out, size_t outn)
{
    if (spec == NULL || spec[0] == '\0') return -1;
    size_t len = strlen(spec);
    char unit = spec[len - 1];
    if (unit == 's' || unit == 'm' || unit == 'h' || unit == 'd') {
        char *endp = NULL;
        long n = strtol(spec, &endp, 10);
        if (endp == spec || endp != spec + len - 1 || n <= 0) return -1;
        long sec = (unit == 's') ? n
                 : (unit == 'm') ? n * 60
                 : (unit == 'h') ? n * 3600
                 :                 n * 86400;
        time_t cutoff = time(NULL) - sec;
        struct tm utc;
        gmtime_r(&cutoff, &utc);
        strftime(out, outn, "%Y-%m-%dT%H:%M:%SZ", &utc);
        return 0;
    }
    if (len + 1 > outn) return -1;
    memcpy(out, spec, len + 1);
    return 0;
}

int gnss_reassemble(const gnss_frag_t *frags, int n, unsigned char *buf, int bufcap)
{
    int total = 0;
    memset(buf, 0, (size_t)bufcap);
    for (int i = 0; i < n; ++i) {
        int dl = frags[i].payload_len - TCMD_RESP_HDR_LEN;
        if (dl < 0) dl = 0;
        if (dl > TCMD_RESP_MAX_DATA) dl = TCMD_RESP_MAX_DATA;
        int off = (frags[i].seq - 1) * TCMD_RESP_MAX_DATA;
        if (off < 0 || off + dl > bufcap) continue;
        memcpy(buf + off, frags[i].payload + TCMD_RESP_HDR_LEN, (size_t)dl);
        if (off + dl > total) total = off + dl;
    }
    if (total >= bufcap) total = bufcap - 1;
    buf[total] = '\0';
    return total;
}
