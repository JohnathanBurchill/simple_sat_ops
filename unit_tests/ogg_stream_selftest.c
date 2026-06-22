/*

    Simple Satellite Operations  unit_tests/ogg_stream_selftest.c

    Coverage for src/audio/ogg_stream.c — the shared Ogg/Vorbis encoder used
    by ham_listen (--ogg-stdout) and the operator's live-audio relay. The
    module wraps libsndfile behind a sink callback; what's actually under
    test is OUR wiring: the forward-only virtual-I/O sink, the byte counter,
    and the sink error/contract paths.

    Oracles:
      - The first four sink bytes must be the Ogg page-capture pattern "OggS"
        (the Ogg container spec — independent of libsndfile).
      - Feeding the captured byte stream back to libsndfile's *decoder* must
        reproduce a VORBIS-in-OGG file at the same rate/channels and a
        comparable frame count. A corrupt sink (dropped/duplicated pages,
        miscounted offsets) would fail to decode — so this exercises the
        plumbing without trusting the encoder to grade itself.

    Without libsndfile the module is stubs; the test then just pins that
    open returns NULL and the accessors degrade safely.

    Exit status: 0 = all tests passed, non-zero = failure.

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

#include "ogg_stream.h"
#include "tap.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef HAVE_SNDFILE
#include <sndfile.h>

// Read-side virtual I/O over an in-memory buffer, so libsndfile can decode
// the bytes our encoder produced (the round-trip oracle).
typedef struct { const uint8_t *buf; sf_count_t len; sf_count_t pos; } memrd_t;

static sf_count_t mr_filelen(void *u) { return ((memrd_t *) u)->len; }
static sf_count_t mr_tell   (void *u) { return ((memrd_t *) u)->pos; }
static sf_count_t mr_write(const void *p, sf_count_t c, void *u)
{ (void) p; (void) c; (void) u; return 0; }

static sf_count_t mr_seek(sf_count_t off, int whence, void *u)
{
    memrd_t *m = (memrd_t *) u;
    sf_count_t np = (whence == SEEK_SET) ? off
                  : (whence == SEEK_CUR) ? m->pos + off
                                         : m->len + off;
    if (np < 0) np = 0;
    if (np > m->len) np = m->len;
    m->pos = np;
    return m->pos;
}

static sf_count_t mr_read(void *p, sf_count_t c, void *u)
{
    memrd_t *m = (memrd_t *) u;
    sf_count_t avail = m->len - m->pos;
    if (c > avail) c = avail;
    memcpy(p, m->buf + m->pos, (size_t) c);
    m->pos += c;
    return c;
}
#endif

// Growable capture sink. fail_after >= 0 makes the (fail_after+1)-th call
// (and every later one) return -1, to exercise the error path.
typedef struct {
    uint8_t *data;
    size_t   len;
    size_t   cap;
    long     calls;
    int      fail_after;   // <0 = never fail
} cap_t;

static long cap_sink(const uint8_t *b, size_t n, void *user)
{
    cap_t *c = (cap_t *) user;
    if (c->fail_after >= 0 && c->calls >= c->fail_after) { c->calls++; return -1; }
    c->calls++;
    if (c->len + n > c->cap) {
        size_t nc = c->cap ? c->cap : 4096;
        while (nc < c->len + n) nc *= 2;
        uint8_t *nb = (uint8_t *) realloc(c->data, nc);
        if (!nb) return -1;
        c->data = nb;
        c->cap  = nc;
    }
    memcpy(c->data + c->len, b, n);
    c->len += n;
    return (long) n;
}

int main(void)
{
    // --- Argument guards (apply to stub + real builds) ----------------
    tap_ok(ogg_stream_open(24000, 1, 0.3, NULL, NULL) == NULL,
           "open with NULL sink returns NULL");
    {
        cap_t cap = { 0, 0, 0, 0, -1 };
        tap_ok(ogg_stream_open(0, 1, 0.3, cap_sink, &cap) == NULL,
               "open with rate<=0 returns NULL");
        tap_ok(ogg_stream_open(24000, 0, 0.3, cap_sink, &cap) == NULL,
               "open with channels<=0 returns NULL");
        free(cap.data);
    }
    tap_ok(ogg_stream_write(NULL, NULL, 0) == -1, "write(NULL) returns -1");
    tap_ok(ogg_stream_bytes(NULL) == 0, "bytes(NULL) == 0");
    ogg_stream_close(NULL);   // must not crash
    tap_ok(1, "close(NULL) is a no-op");

#ifdef HAVE_SNDFILE
    const int SR = 24000, CH = 1, NFR = 24000;   // 1 s mono

    // --- Encode a sine and capture the Ogg stream ---------------------
    cap_t cap = { 0, 0, 0, 0, -1 };
    ogg_stream_t *s = ogg_stream_open(SR, CH, 0.3, cap_sink, &cap);
    tap_ok(s != NULL, "open returns a handle (libsndfile build)");
    if (s) {
        int16_t chunk[1024];
        int ok_writes = 1;
        for (int i = 0; i < NFR; ) {
            int m = (NFR - i < 1024) ? (NFR - i) : 1024;
            for (int k = 0; k < m; ++k) {
                double t = (double) (i + k) / SR;
                chunk[k] = (int16_t) lround(8000.0 * sin(2.0 * M_PI * 1000.0 * t));
            }
            if (ogg_stream_write(s, chunk, (size_t) m) != 0) ok_writes = 0;
            i += m;
        }
        tap_ok(ok_writes, "all writes returned 0 (sink healthy)");
        tap_ok(ogg_stream_bytes(s) > 0, "encoder reports a non-zero byte count");
        ogg_stream_close(s);   // flush remaining pages
    }

    tap_ok(cap.len >= 4, "sink captured a non-empty stream");
    tap_ok(cap.len >= 4 && memcmp(cap.data, "OggS", 4) == 0,
           "stream begins with the Ogg page-capture pattern \"OggS\"");

    // --- Oracle: libsndfile decodes our bytes back to VORBIS/OGG ------
    {
        SF_INFO info; memset(&info, 0, sizeof info);
        memrd_t mr = { cap.data, (sf_count_t) cap.len, 0 };
        SF_VIRTUAL_IO vio = { mr_filelen, mr_seek, mr_read, mr_write, mr_tell };
        SNDFILE *rd = sf_open_virtual(&vio, SFM_READ, &info, &mr);
        tap_ok(rd != NULL, "libsndfile re-opens the captured stream for read");
        if (rd) {
            tap_okf(info.samplerate == SR, "decoded sample rate matches (%d)", info.samplerate);
            tap_okf(info.channels == CH, "decoded channel count matches (%d)", info.channels);
            tap_ok((info.format & SF_FORMAT_TYPEMASK) == SF_FORMAT_OGG
                   && (info.format & SF_FORMAT_SUBMASK) == SF_FORMAT_VORBIS,
                   "decoded container/codec is OGG/VORBIS");
            int16_t buf[4096];
            sf_count_t total = 0, got;
            while ((got = sf_readf_short(rd, buf, 4096)) > 0) total += got;
            tap_okf(total >= NFR / 2,
                    "decoded a comparable frame count: %ld of %d in",
                    (long) total, NFR);
            sf_close(rd);
        }
    }
    free(cap.data);

    // --- Sink failure surfaces (no silent success) --------------------
    {
        cap_t bad = { 0, 0, 0, 0, 0 };   // fail on the very first sink call
        ogg_stream_t *b = ogg_stream_open(SR, CH, 0.2, cap_sink, &bad);
        int surfaced;
        if (!b) {
            surfaced = 1;   // failure surfaced at open (header write rejected)
        } else {
            int16_t z[512];
            memset(z, 0, sizeof z);
            // Write enough to force the encoder to flush at least one page.
            int rc = 0;
            for (int i = 0; i < 64 && rc == 0; ++i) rc = ogg_stream_write(b, z, 512);
            surfaced = (rc < 0);
            ogg_stream_close(b);
        }
        tap_ok(surfaced, "a failing sink surfaces as open==NULL or write<0");
        free(bad.data);
    }
#else
    tap_diag("built without libsndfile — exercising stub paths only");
    {
        cap_t cap = { 0, 0, 0, 0, -1 };
        tap_ok(ogg_stream_open(24000, 1, 0.3, cap_sink, &cap) == NULL,
               "stub open returns NULL without libsndfile");
        free(cap.data);
    }
#endif

    return tap_done();
}
