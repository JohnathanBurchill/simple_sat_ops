/*

    Simple Satellite Operations  unit_tests/gnss_frag_selftest.c

    Exercises src/proto/gnss_frag.c: the prefix test, the since/until time
    spec parser, and the fragment reassembly that gnss_opm / gnss_reports
    use to stitch a multi-packet GNSS report back together.

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

// strptime/timegm on glibc need this; harmless on macOS where both are
// always visible. They are the independent oracle for the time parser.
#define _GNU_SOURCE 1

#include "tap.h"
#include "gnss_frag.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

// The two beacon_cts1.h sizes gnss_frag.c compiles against, restated here
// as literals so this test is an independent oracle for the reassembly
// geometry: if the header values ever change, the offsets these fixtures
// assume no longer match and the test goes red, flagging the layout shift.
#define HDR      14
#define MAXDATA  186

// ---- gnss_starts_with --------------------------------------------------

static void test_starts_with(void)
{
    tap_ok(gnss_starts_with("CTS1+abc", "CTS1+") == 1, "prefix matches");
    tap_ok(gnss_starts_with("hello", "hello") == 1, "whole-string prefix matches");
    tap_ok(gnss_starts_with("anything", "") == 1, "empty prefix always matches");
    tap_ok(gnss_starts_with("he", "hello") == 0, "prefix longer than string does not match");
    tap_ok(gnss_starts_with("Hello", "hello") == 0, "match is case-sensitive");
    tap_ok(gnss_starts_with("abc", "abd") == 0, "differing prefix does not match");
}

// ---- gnss_parse_time_spec ---------------------------------------------

// Independent reverse of the module's strftime("%Y-%m-%dT%H:%M:%SZ"): parse
// the produced string back to a time_t. A malformed format string fails here.
static int parse_iso(const char *s, time_t *out)
{
    struct tm tm;
    memset(&tm, 0, sizeof tm);
    if (strptime(s, "%Y-%m-%dT%H:%M:%SZ", &tm) == NULL) return -1;
    *out = timegm(&tm);
    return 0;
}

static void check_relative(const char *spec, long sec)
{
    char out[64] = {0};
    time_t t0 = time(NULL);
    int rc = gnss_parse_time_spec(spec, out, sizeof out);
    time_t t1 = time(NULL);
    tap_okf(rc == 0, "relative spec '%s' parses", spec);

    time_t got = 0;
    int prc = parse_iso(out, &got);
    tap_okf(prc == 0, "relative spec '%s' yields well-formed ISO-8601 ('%s')", spec, out);

    // The module computes time(NULL) - sec, and time(NULL) was somewhere in
    // [t0, t1]. Without relying on the module's own arithmetic, the embedded
    // cutoff must fall in [t0 - sec, t1 - sec]. A wrong unit multiplier (e.g.
    // hours scaled by 60 instead of 3600) lands well outside this window.
    tap_okf(prc == 0 && got >= t0 - sec && got <= t1 - sec,
            "relative spec '%s' cutoff is now minus %ld s", spec, sec);
}

static void test_time_spec_relative(void)
{
    check_relative("90s", 90);
    check_relative("30m", 30 * 60);
    check_relative("24h", 24 * 3600);
    check_relative("7d", 7 * 86400);
    check_relative("1h", 3600);
}

static void test_time_spec_passthrough(void)
{
    char out[64] = {0};

    // Anything not ending in s/m/h/d is copied through verbatim for the SQL
    // lexicographic comparison the callers do.
    const char *iso = "2026-06-22T18:30:00Z";
    int rc = gnss_parse_time_spec(iso, out, sizeof out);
    tap_okf(rc == 0 && strcmp(out, iso) == 0, "ISO-8601 string passes through verbatim");

    const char *partial = "2026-06-22";
    rc = gnss_parse_time_spec(partial, out, sizeof out);
    tap_okf(rc == 0 && strcmp(out, partial) == 0, "partial date passes through verbatim");

    // Pass-through needs strlen+1 bytes. The 20-char string needs cap 21.
    rc = gnss_parse_time_spec(iso, out, 21);
    tap_okf(rc == 0, "pass-through with exact-fit buffer (21) succeeds");
    rc = gnss_parse_time_spec(iso, out, 20);
    tap_okf(rc == -1, "pass-through with one-too-small buffer (20) is rejected");
}

static void test_time_spec_errors(void)
{
    char out[64] = {0};
    tap_ok(gnss_parse_time_spec(NULL, out, sizeof out) == -1, "NULL spec rejected");
    tap_ok(gnss_parse_time_spec("", out, sizeof out) == -1, "empty spec rejected");
    tap_ok(gnss_parse_time_spec("0s", out, sizeof out) == -1, "zero count rejected");
    tap_ok(gnss_parse_time_spec("-5m", out, sizeof out) == -1, "negative count rejected");
    tap_ok(gnss_parse_time_spec("s", out, sizeof out) == -1, "bare unit with no number rejected");
    tap_ok(gnss_parse_time_spec("1.5h", out, sizeof out) == -1, "non-integer count rejected");
    tap_ok(gnss_parse_time_spec("abch", out, sizeof out) == -1, "non-numeric before unit rejected");
}

// ---- gnss_reassemble ---------------------------------------------------

// Build a fragment: HDR bytes of 0xEE (the per-packet header that must be
// stripped) followed by `datalen` bytes of `fill`.
static gnss_frag_t mkfrag(int seq, int datalen, unsigned char fill)
{
    gnss_frag_t f;
    memset(&f, 0, sizeof f);
    f.seq = seq;
    f.payload_len = HDR + datalen;
    f.payload = malloc((size_t)f.payload_len);
    memset(f.payload, 0xEE, HDR);
    memset(f.payload + HDR, fill, (size_t)datalen);
    return f;
}

static void freefrags(gnss_frag_t *f, int n)
{
    for (int i = 0; i < n; ++i) free(f[i].payload);
}

static void test_reassemble_basic(void)
{
    gnss_frag_t f[2];
    f[0] = mkfrag(1, 10, 'A');
    f[1] = mkfrag(2, 10, 'B');
    unsigned char buf[512];
    int total = gnss_reassemble(f, 2, buf, sizeof buf);

    tap_okf(total == MAXDATA + 10,
            "two frags: length is one full slot + second payload (got %d)", total);
    tap_okf(buf[0] == 'A' && buf[9] == 'A',
            "frag 1 data lands at offset 0 with its 14-byte header stripped");
    tap_okf(buf[10] == 0 && buf[MAXDATA - 1] == 0,
            "the short tail of slot 1 is zero-filled up to slot 2");
    tap_okf(buf[MAXDATA] == 'B' && buf[MAXDATA + 9] == 'B',
            "frag 2 data lands at offset (seq-1)*MAXDATA");
    tap_okf(buf[total] == '\0',
            "output is NUL-terminated at the reassembled length");
    freefrags(f, 2);
}

static void test_reassemble_out_of_order(void)
{
    // Same two fragments, seq 2 passed before seq 1. Placement is by offset,
    // so the result must match in-order input exactly.
    gnss_frag_t f[2];
    f[0] = mkfrag(2, 10, 'B');
    f[1] = mkfrag(1, 10, 'A');
    unsigned char buf[512];
    int total = gnss_reassemble(f, 2, buf, sizeof buf);
    tap_okf(total == MAXDATA + 10, "out-of-order input: same length (got %d)", total);
    tap_okf(buf[0] == 'A' && buf[MAXDATA] == 'B',
            "out-of-order input: bytes placed by seq, not arrival order");
    freefrags(f, 2);
}

static void test_reassemble_clamp_maxdata(void)
{
    // A fragment claiming more than one slot of data is clamped to MAXDATA.
    gnss_frag_t f = mkfrag(1, MAXDATA + 50, 'X');
    unsigned char buf[512];
    int total = gnss_reassemble(&f, 1, buf, sizeof buf);
    tap_okf(total == MAXDATA, "oversized fragment is clamped to MAXDATA (got %d)", total);
    tap_okf(buf[MAXDATA - 1] == 'X', "clamped fragment fills exactly MAXDATA bytes");
    tap_okf(buf[MAXDATA] == '\0', "nothing is written past the clamp");
    free(f.payload);
}

static void test_reassemble_short_fragment(void)
{
    // A payload shorter than the header gives a negative data length, which
    // is clamped to 0 so the fragment contributes nothing.
    gnss_frag_t f;
    memset(&f, 0, sizeof f);
    f.seq = 1;
    f.payload_len = HDR - 5;
    f.payload = malloc(HDR);   // >= payload_len; payload+HDR is not deref'd at dl=0
    memset(f.payload, 0xEE, HDR);
    unsigned char buf[512];
    int total = gnss_reassemble(&f, 1, buf, sizeof buf);
    tap_okf(total == 0, "sub-header fragment contributes no bytes (got %d)", total);
    tap_okf(buf[0] == '\0', "empty reassembly is an empty string");
    free(f.payload);
}

static void test_reassemble_overflow_skip(void)
{
    // bufcap too small to hold frag 2's slot: it is skipped entirely (not
    // truncated) and must not disturb frag 1's bytes.
    gnss_frag_t f[2];
    f[0] = mkfrag(1, 10, 'A');
    f[1] = mkfrag(2, 10, 'B');   // wants offset 186, past the cap of 100
    unsigned char buf[512];
    int total = gnss_reassemble(f, 2, buf, 100);
    tap_okf(total == 10, "fragment that would overflow bufcap is skipped (got %d)", total);
    tap_okf(buf[0] == 'A' && buf[9] == 'A',
            "in-bounds fragment is preserved when a later one is skipped");
    tap_okf(buf[10] == '\0', "terminator sits at the end of the kept data");
    freefrags(f, 2);
}

static void test_reassemble_cap_terminator(void)
{
    // frag 2 fits exactly to bufcap (offset 186 + 14 == 200). The length
    // reaches bufcap, which is capped to bufcap-1 so the NUL fits, clobbering
    // the final data byte.
    gnss_frag_t f[2];
    f[0] = mkfrag(1, 10, 'A');
    f[1] = mkfrag(2, 14, 'Z');
    unsigned char buf[512];
    int total = gnss_reassemble(f, 2, buf, 200);
    tap_okf(total == 199, "length reaching bufcap is capped to bufcap-1 (got %d)", total);
    tap_okf(buf[199] == '\0', "terminator occupies the last buffer byte");
    tap_okf(buf[198] == 'Z', "the second-to-last data byte survives the cap");
    tap_okf(buf[MAXDATA] == 'Z', "frag 2 is still placed at its offset");
    freefrags(f, 2);
}

static void test_reassemble_gap(void)
{
    // seq 1 and seq 3 present, seq 2 missing: the seq-2 slot stays zero and
    // the length reaches the end of seq 3.
    gnss_frag_t f[2];
    f[0] = mkfrag(1, MAXDATA, 'A');   // fills slot 0 completely
    f[1] = mkfrag(3, 5, 'C');          // offset 2*MAXDATA
    unsigned char buf[1024];
    int total = gnss_reassemble(f, 2, buf, sizeof buf);
    tap_okf(total == 2 * MAXDATA + 5,
            "gap: length spans to the end of the last present fragment (got %d)", total);
    tap_okf(buf[MAXDATA] == 0 && buf[2 * MAXDATA - 1] == 0,
            "the missing middle slot is left zero-filled");
    tap_okf(buf[2 * MAXDATA] == 'C', "seq 3 lands at offset 2*MAXDATA");
    freefrags(f, 2);
}

static void test_reassemble_empty(void)
{
    unsigned char buf[64];
    memset(buf, 0xFF, sizeof buf);
    int total = gnss_reassemble(NULL, 0, buf, sizeof buf);
    tap_okf(total == 0, "zero fragments reassemble to length 0 (got %d)", total);
    tap_okf(buf[0] == '\0' && buf[10] == '\0',
            "the buffer is zeroed even with no fragments");
}

int main(void)
{
    test_starts_with();
    test_time_spec_relative();
    test_time_spec_passthrough();
    test_time_spec_errors();
    test_reassemble_basic();
    test_reassemble_out_of_order();
    test_reassemble_clamp_maxdata();
    test_reassemble_short_fragment();
    test_reassemble_overflow_skip();
    test_reassemble_cap_terminator();
    test_reassemble_gap();
    test_reassemble_empty();
    return tap_done();
}
