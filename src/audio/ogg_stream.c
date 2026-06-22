/*

    Simple Satellite Operations  audio/ogg_stream.c

    Shared Ogg/Vorbis encoder — see ogg_stream.h. The libsndfile
    virtual-I/O write callback forwards every encoded Ogg page to the
    caller's sink and keeps a running byte count.

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

#include "ogg_stream.h"

#include <stdlib.h>

#ifdef HAVE_SNDFILE
#include <sndfile.h>

struct ogg_stream {
    SNDFILE            *sf;
    ogg_stream_sink_fn  sink;
    void               *user;
    sf_count_t          offset;   // logical write position (Ogg is forward-only)
    unsigned long long  total;    // total bytes the sink accepted
    int                 failed;   // sink returned an error; latch it
};

// libsndfile virtual I/O. Ogg write is forward-only, so "seek" just
// tracks a logical offset — the underlying sink is never seeked.
static sf_count_t vio_filelen(void *u) { return ((ogg_stream_t *) u)->offset; }
static sf_count_t vio_tell   (void *u) { return ((ogg_stream_t *) u)->offset; }
static sf_count_t vio_read(void *p, sf_count_t c, void *u)
{ (void) p; (void) c; (void) u; return 0; }

static sf_count_t vio_seek(sf_count_t off, int whence, void *u)
{
    ogg_stream_t *s = (ogg_stream_t *) u;
    s->offset = (whence == SEEK_SET) ? off : s->offset + off;  // CUR/END: relative
    return s->offset;
}

static sf_count_t vio_write(const void *p, sf_count_t c, void *u)
{
    ogg_stream_t *s = (ogg_stream_t *) u;
    long w = s->sink((const uint8_t *) p, (size_t) c, s->user);
    if (w < 0) {
        // Sink failed (pipe closed / send queue full): latch it so
        // ogg_stream_write can report the error and the caller stops.
        s->failed = 1;
        return 0;
    }
    s->offset += w;
    s->total  += (unsigned long long) w;
    return w;
}

ogg_stream_t *ogg_stream_open(int sample_rate, int channels,
                              double vbr_quality,
                              ogg_stream_sink_fn sink, void *user)
{
    if (!sink || sample_rate <= 0 || channels <= 0) return NULL;
    ogg_stream_t *s = (ogg_stream_t *) calloc(1, sizeof *s);
    if (!s) return NULL;
    s->sink = sink;
    s->user = user;

    SF_INFO info = {0};
    info.samplerate = sample_rate;
    info.channels   = channels;
    info.format     = SF_FORMAT_OGG | SF_FORMAT_VORBIS;
    // libsndfile copies the struct, so this local can go out of scope,
    // but `s` (the user pointer) must outlive the SNDFILE.
    SF_VIRTUAL_IO vio = {
        vio_filelen, vio_seek, vio_read, vio_write, vio_tell,
    };
    s->sf = sf_open_virtual(&vio, SFM_WRITE, &info, s);
    if (!s->sf) {
        free(s);
        return NULL;
    }
    double q = vbr_quality;
    if (q < 0.0) q = 0.0;
    if (q > 1.0) q = 1.0;
    sf_command(s->sf, SFC_SET_VBR_ENCODING_QUALITY, &q, sizeof q);
    return s;
}

int ogg_stream_write(ogg_stream_t *s, const int16_t *pcm, size_t frames)
{
    if (!s || !s->sf) return -1;
    if (frames > 0 && pcm) {
        sf_writef_short(s->sf, pcm, (sf_count_t) frames);
    }
    return s->failed ? -1 : 0;
}

unsigned long long ogg_stream_bytes(const ogg_stream_t *s)
{
    return s ? s->total : 0;
}

void ogg_stream_close(ogg_stream_t *s)
{
    if (!s) return;
    if (s->sf) sf_close(s->sf);   // flushes remaining pages through vio_write
    free(s);
}

#else  // !HAVE_SNDFILE — stubs so callers link and degrade gracefully.

ogg_stream_t *ogg_stream_open(int sample_rate, int channels,
                              double vbr_quality,
                              ogg_stream_sink_fn sink, void *user)
{
    (void) sample_rate; (void) channels; (void) vbr_quality;
    (void) sink; (void) user;
    return NULL;
}

int ogg_stream_write(ogg_stream_t *s, const int16_t *pcm, size_t frames)
{
    (void) s; (void) pcm; (void) frames;
    return -1;
}

unsigned long long ogg_stream_bytes(const ogg_stream_t *s)
{
    (void) s;
    return 0;
}

void ogg_stream_close(ogg_stream_t *s) { (void) s; }

#endif // HAVE_SNDFILE
