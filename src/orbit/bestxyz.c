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

// ---- binary BESTXYZB parse --------------------------------------------------

// NovAtel enumerations (OEM7 Commands and Logs Reference Manual) that the
// binary log carries as numbers and the ASCII log spells out.
typedef struct { unsigned v; const char *name; } enum_name_t;

static const enum_name_t TIME_STATUS[] = {
    {20, "UNKNOWN"}, {60, "APPROXIMATE"}, {80, "COARSEADJUSTING"},
    {100, "COARSE"}, {120, "COARSESTEERING"}, {130, "FREEWHEELING"},
    {140, "FINEADJUSTING"}, {160, "FINE"}, {170, "FINEBACKUPSTEERING"},
    {180, "FINESTEERING"}, {200, "SATTIME"},
};

static const enum_name_t SOL_STATUS[] = {
    {0, "SOL_COMPUTED"}, {1, "INSUFFICIENT_OBS"}, {2, "NO_CONVERGENCE"},
    {3, "SINGULARITY"}, {4, "COV_TRACE"}, {5, "TEST_DIST"}, {6, "COLD_START"},
    {7, "V_H_LIMIT"}, {8, "VARIANCE"}, {9, "RESIDUALS"},
    {13, "INTEGRITY_WARNING"}, {18, "PENDING"}, {19, "INVALID_FIX"},
    {20, "UNAUTHORIZED"}, {22, "INVALID_RATE"},
};

static const enum_name_t POS_TYPE[] = {
    {0, "NONE"}, {1, "FIXEDPOS"}, {2, "FIXEDHEIGHT"}, {8, "DOPPLER_VELOCITY"},
    {16, "SINGLE"}, {17, "PSRDIFF"}, {18, "WAAS"}, {19, "PROPAGATED"},
    {32, "L1_FLOAT"}, {34, "NARROW_FLOAT"}, {48, "L1_INT"}, {49, "WIDE_INT"},
    {50, "NARROW_INT"}, {68, "PPP_CONVERGING"}, {69, "PPP"},
};

// The enum's name, or the bare number when the table doesn't know it.
static void enum_name(const enum_name_t *t, size_t n, unsigned v,
                      char *out, size_t outn)
{
    for (size_t i = 0; i < n; ++i)
        if (t[i].v == v) { snprintf(out, outn, "%s", t[i].name); return; }
    snprintf(out, outn, "%u", v);
}

static uint16_t rd_u16(const unsigned char *p) { return (uint16_t) (p[0] | p[1] << 8); }
static uint32_t rd_u32(const unsigned char *p)
{
    return (uint32_t) p[0] | (uint32_t) p[1] << 8 | (uint32_t) p[2] << 16 | (uint32_t) p[3] << 24;
}
// The receiver writes IEEE-754 little-endian, as the hosts we build on are.
static double rd_f64(const unsigned char *p) { double v; memcpy(&v, p, 8); return v; }
static double rd_f32(const unsigned char *p) { float v; memcpy(&v, p, 4); return (double) v; }

#define BESTXYZB_MSG_ID   241
#define BESTXYZB_BODY_LEN 112

int bestxyzb_parse(const unsigned char *msg, size_t len, bestxyz_t *out,
                   char *err, size_t errsz)
{
    memset(out, 0, sizeof *out);
#define FAIL(m) do { if (errsz) snprintf(err, errsz, "%s", (m)); return -1; } while (0)

    if (len < 10) FAIL("too short for a NovAtel binary header");
    if (msg[0] != 0xAA || msg[1] != 0x44 || msg[2] != 0x12)
        FAIL("no NovAtel binary sync (AA 44 12)");
    size_t hdr = msg[3];
    if (hdr < 28) FAIL("binary header shorter than 28 bytes");
    if (rd_u16(msg + 4) != BESTXYZB_MSG_ID) FAIL("not a BESTXYZB log (message ID != 241)");
    size_t body = rd_u16(msg + 8);
    if (body < BESTXYZB_BODY_LEN) FAIL("BESTXYZB body shorter than 112 bytes");
    if (len < hdr + body + 4) FAIL("BESTXYZB log truncated");

    // The CRC covers the header and body (sync bytes included) and follows them.
    out->crc_calc = bestxyz_novatel_crc32(msg, hdr + body);
    out->crc_read = rd_u32(msg + hdr + body);
    out->crc_present = 1;
    out->crc_ok = (out->crc_calc == out->crc_read);

    enum_name(TIME_STATUS, sizeof TIME_STATUS / sizeof TIME_STATUS[0], msg[13],
              out->time_status, sizeof out->time_status);
    out->gps_week = rd_u16(msg + 14);
    out->gps_sow = rd_u32(msg + 16) / 1000.0;

    const unsigned char *b = msg + hdr;
    enum_name(SOL_STATUS, sizeof SOL_STATUS / sizeof SOL_STATUS[0], rd_u32(b),
              out->pos_sol_status, sizeof out->pos_sol_status);
    enum_name(POS_TYPE, sizeof POS_TYPE / sizeof POS_TYPE[0], rd_u32(b + 4),
              out->pos_type, sizeof out->pos_type);
    for (int k = 0; k < 3; ++k) {
        out->pos[k] = rd_f64(b + 8 + 8 * k);
        out->pos_sigma[k] = rd_f32(b + 32 + 4 * k);
    }
    enum_name(SOL_STATUS, sizeof SOL_STATUS / sizeof SOL_STATUS[0], rd_u32(b + 44),
              out->vel_sol_status, sizeof out->vel_sol_status);
    enum_name(POS_TYPE, sizeof POS_TYPE / sizeof POS_TYPE[0], rd_u32(b + 48),
              out->vel_type, sizeof out->vel_type);
    for (int k = 0; k < 3; ++k) {
        out->vel[k] = rd_f64(b + 52 + 8 * k);
        out->vel_sigma[k] = rd_f32(b + 76 + 4 * k);
    }
    // b + 88 is the 4-byte base station ID, b + 96 the differential age.
    out->vel_latency = rd_f32(b + 92);
    out->sol_age = rd_f32(b + 100);
    out->num_sv = b[104];
    out->num_sol_sv = b[105];

    return 0;
#undef FAIL
}

// ---- TIMEA parse ------------------------------------------------------------

// TIMEA layout: the header matches every NovAtel log (name through ';'),
// then the body is:
//   clockstatus,offset,offsetstd,utcoffset,utcyear,utcmon,utcday,
//   utchour,utcmin,utcms,utcstatus*CRC
int timea_parse(const char *text, timea_t *out, char *err, size_t errsz)
{
    memset(out, 0, sizeof *out);
#define FAIL(msg) do { if (errsz) snprintf(err, errsz, "%s", (msg)); return -1; } while (0)

    const char *name = strstr(text, "TIMEA");
    if (!name) FAIL("no TIMEA log found in input");

    const char *semi0 = strchr(name, ';');
    if (!semi0) FAIL("TIMEA header has no ';' terminator");

    const char *star = strchr(semi0, '*');
    const char *raw_end = star;
    if (!raw_end) {
        raw_end = strpbrk(semi0, "\r\n");
        if (!raw_end) raw_end = semi0 + strlen(semi0);
    }

    char msg[1024];
    size_t mlen = json_unescape(name, (size_t) (raw_end - name), msg, sizeof msg);

    if (star) {
        out->crc_calc = bestxyz_novatel_crc32((const unsigned char *) msg, mlen);
        out->crc_read = (unsigned) strtoul(star + 1, NULL, 16);
        out->crc_present = 1;
        out->crc_ok = (out->crc_calc == out->crc_read);
    }

    char *semi = strchr(msg, ';');
    if (!semi) FAIL("TIMEA header has no ';' terminator");
    *semi = '\0';

    char *hf[MAX_FIELDS];
    int hn = split_fields(msg, hf, MAX_FIELDS);
    if (hn < 7) FAIL("TIMEA header has too few fields");
    copy_field(out->time_status, sizeof out->time_status, hf[4]);
    out->gps_week = atoi(hf[5]);
    out->gps_sow = strtod(hf[6], NULL);

    char *bf[MAX_FIELDS];
    int bn = split_fields(semi + 1, bf, MAX_FIELDS);
    if (bn < 11) FAIL("TIMEA body has too few fields");

    copy_field(out->clock_status, sizeof out->clock_status, bf[0]);
    out->clock_offset = strtod(bf[1], NULL);
    out->clock_offset_std = strtod(bf[2], NULL);
    out->utc_offset = strtod(bf[3], NULL);
    copy_field(out->utc_status, sizeof out->utc_status, bf[10]);

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
