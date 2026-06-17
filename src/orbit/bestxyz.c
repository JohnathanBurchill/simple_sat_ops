/*

    Simple Satellite Operations  bestxyz.c

    Parse a NovAtel OEM7 BESTXYZA ASCII log into an Earth-fixed
    position/velocity state. See bestxyz.h. This is the pure-C stand-in
    for the team's novatel_edie snippet: locate the log inside whatever
    wrapper the telecommand response carries, check the message CRC, and
    pull out the fields tle_from_state needs.

    BESTXYZA layout (commas separate fields, ';' ends the header, '*'
    precedes the CRC):

      #BESTXYZA,port,seq,idle,timestatus,WEEK,SECONDS,rxstat,resv,ver;
      Psolstat,postype,X,Y,Z,sdX,sdY,sdZ,
      Vsolstat,veltype,vX,vY,vZ,sdvX,sdvY,sdvZ,
      stnid,vlatency,diffage,solage,#SV,#solSV,...*CRC

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include "bestxyz.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ---- NovAtel 32-bit CRC (CalculateBlockCRC32 from the OEM7 manual) ----------

static uint32_t crc32_value(uint32_t v)
{
    for (int j = 0; j < 8; ++j)
        v = (v & 1) ? (v >> 1) ^ 0xEDB88320u : (v >> 1);
    return v;
}

uint32_t bestxyz_novatel_crc32(const unsigned char *buf, size_t len)
{
    uint32_t crc = 0;
    for (size_t k = 0; k < len; ++k) {
        uint32_t t1 = (crc >> 8) & 0x00FFFFFFu;
        uint32_t t2 = crc32_value((crc ^ buf[k]) & 0xFFu);
        crc = t1 ^ t2;
    }
    return crc;
}

// ---- field splitting --------------------------------------------------------

#define MAX_FIELDS 64

// Split s in place on commas that are not inside double quotes. Stores up
// to max field pointers and returns the count.
static int split_fields(char *s, char **fields, int max)
{
    if (max <= 0) return 0;
    int n = 0, in_quote = 0;
    fields[n++] = s;
    for (char *p = s; *p; ++p) {
        if (*p == '"') {
            in_quote = !in_quote;
        } else if (*p == ',' && !in_quote) {
            *p = '\0';
            if (n < max) fields[n++] = p + 1;
            else break;
        }
    }
    return n;
}

// Copy src into dst (capacity cap), trimming surrounding whitespace and a
// pair of enclosing double quotes. Manual bounded copy, so no snprintf
// truncation diagnostic.
static void copy_field(char *dst, size_t cap, const char *src)
{
    while (*src == ' ' || *src == '\t') src++;
    size_t len = strlen(src);
    while (len && (src[len - 1] == ' ' || src[len - 1] == '\t')) len--;
    if (len >= 2 && src[0] == '"' && src[len - 1] == '"') { src++; len -= 2; }
    if (len >= cap) len = cap - 1;
    memcpy(dst, src, len);
    dst[len] = '\0';
}

// Copy n bytes of src into dst (capacity cap), undoing JSON string
// escaping. A telecommand response is sometimes pasted as JSON, which
// turns the message's stn-id "" into \"\" and may escape \r\n; a genuine
// NovAtel log has no backslashes, so this leaves a raw paste untouched
// and restores the true bytes for a JSON-wrapped one. Returns the length
// written (excluding the terminator).
static size_t json_unescape(const char *src, size_t n, char *dst, size_t cap)
{
    size_t o = 0;
    for (size_t i = 0; i < n && o + 1 < cap; ++i) {
        char c = src[i];
        if (c == '\\' && i + 1 < n) {
            char e = src[++i];
            switch (e) {
                case 'n': c = '\n'; break;
                case 'r': c = '\r'; break;
                case 't': c = '\t'; break;
                default:  c = e;    break;   // \" \\ \/ and anything else
            }
        }
        dst[o++] = c;
    }
    dst[o] = '\0';
    return o;
}

// ---- parse ------------------------------------------------------------------

int bestxyz_parse(const char *text, bestxyz_t *out, char *err, size_t errsz)
{
    memset(out, 0, sizeof *out);
#define FAIL(msg) do { if (errsz) snprintf(err, errsz, "%s", (msg)); return -1; } while (0)

    // Find the log inside any wrapper. NovAtel frames it with a leading
    // '#'; the CRC is computed from just after that '#' to just before
    // the '*', which is the same span as the start of "BESTXYZA".
    const char *name = strstr(text, "BESTXYZA");
    if (!name) FAIL("no BESTXYZA log found in input");

    const char *semi0 = strchr(name, ';');
    if (!semi0) FAIL("BESTXYZA header has no ';' terminator");

    // The CRC follows '*'; without one, stop the message at end of line.
    const char *star = strchr(semi0, '*');
    const char *raw_end = star;
    if (!raw_end) {
        raw_end = strpbrk(semi0, "\r\n");
        if (!raw_end) raw_end = semi0 + strlen(semi0);
    }

    // Working copy of the message (the bytes a '#'-framed NovAtel log
    // puts between '#' and '*'), JSON-unescaped so a response pasted as
    // JSON validates and parses the same as a raw one.
    char msg[1024];
    size_t mlen = json_unescape(name, (size_t) (raw_end - name), msg, sizeof msg);

    if (star) {
        out->crc_calc = bestxyz_novatel_crc32((const unsigned char *) msg, mlen);
        out->crc_read = (unsigned) strtoul(star + 1, NULL, 16);
        out->crc_present = 1;
        out->crc_ok = (out->crc_calc == out->crc_read);
    }

    // Split the working copy at ';' into header and body.
    char *semi = strchr(msg, ';');
    if (!semi) FAIL("BESTXYZA header has no ';' terminator");
    *semi = '\0';

    char *hf[MAX_FIELDS];
    int hn = split_fields(msg, hf, MAX_FIELDS);
    if (hn < 7) FAIL("BESTXYZA header has too few fields");
    copy_field(out->time_status, sizeof out->time_status, hf[4]);
    out->gps_week = atoi(hf[5]);
    out->gps_sow = strtod(hf[6], NULL);

    char *bf[MAX_FIELDS];
    int bn = split_fields(semi + 1, bf, MAX_FIELDS);
    if (bn < 16) FAIL("BESTXYZA body has too few fields");

    copy_field(out->pos_sol_status, sizeof out->pos_sol_status, bf[0]);
    copy_field(out->pos_type, sizeof out->pos_type, bf[1]);
    out->pos[0] = strtod(bf[2], NULL);
    out->pos[1] = strtod(bf[3], NULL);
    out->pos[2] = strtod(bf[4], NULL);
    out->pos_sigma[0] = strtod(bf[5], NULL);
    out->pos_sigma[1] = strtod(bf[6], NULL);
    out->pos_sigma[2] = strtod(bf[7], NULL);

    copy_field(out->vel_sol_status, sizeof out->vel_sol_status, bf[8]);
    copy_field(out->vel_type, sizeof out->vel_type, bf[9]);
    out->vel[0] = strtod(bf[10], NULL);
    out->vel[1] = strtod(bf[11], NULL);
    out->vel[2] = strtod(bf[12], NULL);
    out->vel_sigma[0] = strtod(bf[13], NULL);
    out->vel_sigma[1] = strtod(bf[14], NULL);
    out->vel_sigma[2] = strtod(bf[15], NULL);

    if (bn > 17) out->vel_latency = strtod(bf[17], NULL);
    if (bn > 19) out->sol_age = strtod(bf[19], NULL);
    if (bn > 20) out->num_sv = atoi(bf[20]);
    if (bn > 21) out->num_sol_sv = atoi(bf[21]);

    return 0;
#undef FAIL
}

// ---- GPS time -> UTC --------------------------------------------------------

void bestxyz_gps_to_utc(int gps_week, double gps_sow, int leap_seconds,
                        int *year, int *mon, int *day,
                        int *hh, int *mm, double *ss)
{
    // GPS epoch is 1980-01-06 00:00:00 UTC = Unix 315964800. GPS time has
    // no leap seconds; subtracting the GPS-UTC offset puts us on the UTC
    // (Unix) timescale that gmtime understands.
    double unix_utc = 315964800.0 + (double) gps_week * 604800.0
                      + gps_sow - (double) leap_seconds;
    time_t whole = (time_t) floor(unix_utc);
    double frac = unix_utc - (double) whole;

    struct tm g;
    gmtime_r(&whole, &g);
    *year = g.tm_year + 1900;
    *mon  = g.tm_mon + 1;
    *day  = g.tm_mday;
    *hh   = g.tm_hour;
    *mm   = g.tm_min;
    *ss   = g.tm_sec + frac;
}
