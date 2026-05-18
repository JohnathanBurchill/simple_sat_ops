/*

    Simple Satellite Operations  utils/gen_waterfall.c

    Build a SatNOGS-style waterfall PNG from a raw interleaved int16
    I,Q file. The pipeline is:

      1. Slice the IQ stream into overlapping FFT frames.
      2. Hann-window each frame, run a radix-2 complex FFT.
      3. Magnitude² in dB, fftshift so DC sits in the middle.
      4. Bin the FFT frames into output rows by averaging in dB-space.
      5. Per-frequency median subtraction so the noise floor flattens
         out, the way the SatNOGS waterfalls do — without that step
         our spectra look "muddy" even with a real complex FFT.
      6. Map the resulting per-pixel dB into the viridis colormap.
      7. Emit a PNG (RGB, 8 bpc) with a hand-rolled stored-DEFLATE
         encoder so we don't need libz or libpng.

    Usage:

        gen_waterfall <iq_path> <sample_rate_hz> <out_png>
                      [--fft=N] [--rows=N] [--db-min=X] [--db-max=X]
                      [--center-hz=F]

    Defaults: --fft=1024, --rows=1080, --db-min=-3, --db-max=20.

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

#define _GNU_SOURCE

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------------------------------------------------------------------
// Radix-2 in-place complex FFT (Cooley-Tukey, iterative).
// Operates on a pair of float arrays (re, im) of length n where n must
// be a power of two. Forward transform; we don't need the inverse.
// --------------------------------------------------------------------

static int is_pow2(unsigned n) { return n > 0 && (n & (n - 1)) == 0; }

static void fft_bit_reverse(float *re, float *im, unsigned n)
{
    unsigned j = 0;
    for (unsigned i = 1; i < n; ++i) {
        unsigned bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
}

static void fft_forward(float *re, float *im, unsigned n)
{
    fft_bit_reverse(re, im, n);
    for (unsigned len = 2; len <= n; len <<= 1) {
        double ang = -2.0 * M_PI / (double) len;
        double wr_step = cos(ang);
        double wi_step = sin(ang);
        unsigned half = len >> 1;
        for (unsigned i = 0; i < n; i += len) {
            double wr = 1.0, wi = 0.0;
            for (unsigned k = 0; k < half; ++k) {
                unsigned a = i + k;
                unsigned b = a + half;
                double tr = wr * re[b] - wi * im[b];
                double ti = wr * im[b] + wi * re[b];
                re[b] = (float)(re[a] - tr);
                im[b] = (float)(im[a] - ti);
                re[a] = (float)(re[a] + tr);
                im[a] = (float)(im[a] + ti);
                double nwr = wr * wr_step - wi * wi_step;
                double nwi = wr * wi_step + wi * wr_step;
                wr = nwr;
                wi = nwi;
            }
        }
    }
}

// --------------------------------------------------------------------
// Viridis colormap (256 RGB entries, generated from matplotlib's
// table). Embedded as a constant so we don't need matplotlib or any
// runtime dependency. Each row is R,G,B in 0..255.
// --------------------------------------------------------------------

static const uint8_t VIRIDIS[256][3] = {
    { 68,  1, 84},{ 68,  2, 86},{ 69,  4, 87},{ 69,  5, 89},{ 70,  7, 90},
    { 70,  8, 92},{ 70, 10, 93},{ 70, 11, 94},{ 71, 13, 96},{ 71, 14, 97},
    { 71, 16, 99},{ 71, 17,100},{ 71, 19,101},{ 72, 20,103},{ 72, 22,104},
    { 72, 23,105},{ 72, 24,106},{ 72, 26,108},{ 72, 27,109},{ 72, 28,110},
    { 72, 29,111},{ 72, 31,112},{ 72, 32,113},{ 72, 33,115},{ 72, 35,116},
    { 72, 36,117},{ 72, 37,118},{ 72, 38,119},{ 72, 40,120},{ 72, 41,121},
    { 71, 42,122},{ 71, 44,122},{ 71, 45,123},{ 71, 46,124},{ 71, 47,125},
    { 70, 48,126},{ 70, 50,126},{ 70, 51,127},{ 70, 52,128},{ 69, 53,129},
    { 69, 55,129},{ 69, 56,130},{ 68, 57,131},{ 68, 58,131},{ 68, 59,132},
    { 67, 61,132},{ 67, 62,133},{ 66, 63,133},{ 66, 64,134},{ 66, 65,134},
    { 65, 66,135},{ 65, 68,135},{ 64, 69,136},{ 64, 70,136},{ 63, 71,136},
    { 63, 72,137},{ 62, 73,137},{ 62, 74,137},{ 62, 76,138},{ 61, 77,138},
    { 61, 78,138},{ 60, 79,138},{ 60, 80,139},{ 59, 81,139},{ 59, 82,139},
    { 58, 83,139},{ 58, 84,140},{ 57, 85,140},{ 57, 86,140},{ 56, 88,140},
    { 56, 89,140},{ 55, 90,140},{ 55, 91,141},{ 54, 92,141},{ 54, 93,141},
    { 53, 94,141},{ 53, 95,141},{ 52, 96,141},{ 52, 97,141},{ 51, 98,141},
    { 51, 99,141},{ 50,100,142},{ 50,101,142},{ 49,102,142},{ 49,103,142},
    { 49,104,142},{ 48,105,142},{ 48,106,142},{ 47,107,142},{ 47,108,142},
    { 46,109,142},{ 46,110,142},{ 46,111,142},{ 45,112,142},{ 45,113,142},
    { 44,113,142},{ 44,114,142},{ 44,115,142},{ 43,116,142},{ 43,117,142},
    { 42,118,142},{ 42,119,142},{ 42,120,142},{ 41,121,142},{ 41,122,142},
    { 41,123,142},{ 40,124,142},{ 40,125,142},{ 39,126,142},{ 39,127,142},
    { 39,128,142},{ 38,129,142},{ 38,130,142},{ 38,130,142},{ 37,131,142},
    { 37,132,142},{ 37,133,142},{ 36,134,142},{ 36,135,142},{ 35,136,142},
    { 35,137,142},{ 35,138,141},{ 34,139,141},{ 34,140,141},{ 34,141,141},
    { 33,142,141},{ 33,143,141},{ 33,144,141},{ 33,145,140},{ 32,146,140},
    { 32,146,140},{ 32,147,140},{ 31,148,140},{ 31,149,139},{ 31,150,139},
    { 31,151,139},{ 31,152,139},{ 31,153,138},{ 31,154,138},{ 30,155,138},
    { 30,156,137},{ 30,157,137},{ 31,158,137},{ 31,159,136},{ 31,160,136},
    { 31,161,136},{ 31,161,135},{ 31,162,135},{ 32,163,134},{ 32,164,134},
    { 33,165,133},{ 33,166,133},{ 34,167,133},{ 34,168,132},{ 35,169,131},
    { 36,170,131},{ 37,171,130},{ 37,172,130},{ 38,173,129},{ 39,173,129},
    { 40,174,128},{ 41,175,127},{ 42,176,127},{ 44,177,126},{ 45,178,125},
    { 46,179,124},{ 47,180,124},{ 49,181,123},{ 50,182,122},{ 52,182,121},
    { 53,183,121},{ 55,184,120},{ 56,185,119},{ 58,186,118},{ 59,187,117},
    { 61,188,116},{ 63,188,115},{ 64,189,114},{ 66,190,113},{ 68,191,112},
    { 70,192,111},{ 72,193,110},{ 74,193,109},{ 76,194,108},{ 78,195,107},
    { 80,196,106},{ 82,197,105},{ 84,197,104},{ 86,198,103},{ 88,199,101},
    { 90,200,100},{ 92,200, 99},{ 94,201, 98},{ 96,202, 96},{ 99,203, 95},
    {101,203, 94},{103,204, 92},{105,205, 91},{108,205, 90},{110,206, 88},
    {112,207, 87},{115,208, 86},{117,208, 84},{119,209, 83},{122,209, 81},
    {124,210, 80},{127,211, 78},{129,211, 77},{132,212, 75},{134,213, 73},
    {137,213, 72},{139,214, 70},{142,214, 69},{144,215, 67},{147,215, 65},
    {149,216, 64},{152,216, 62},{155,217, 60},{157,217, 59},{160,218, 57},
    {162,218, 55},{165,219, 54},{168,219, 52},{170,220, 50},{173,220, 48},
    {176,221, 47},{178,221, 45},{181,222, 43},{184,222, 41},{186,222, 40},
    {189,223, 38},{192,223, 37},{194,223, 35},{197,224, 33},{200,224, 32},
    {202,225, 31},{205,225, 29},{208,225, 28},{210,226, 27},{213,226, 26},
    {216,226, 25},{218,227, 25},{221,227, 24},{223,227, 24},{226,228, 24},
    {229,228, 25},{231,228, 25},{234,229, 26},{236,229, 27},{239,229, 28},
    {241,229, 29},{244,230, 30},{246,230, 32},{248,230, 33},{251,231, 35},
    {253,231, 37}
};

// --------------------------------------------------------------------
// Minimal PNG writer (stored DEFLATE, no compression).
// PNG output is larger than zlib-compressed but every byte is valid
// per the PNG spec, and we don't need a runtime zlib dependency.
// --------------------------------------------------------------------

static uint32_t crc32_iso_hdlc(const uint8_t *buf, size_t len)
{
    static uint32_t table[256];
    static int initialised = 0;
    if (!initialised) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k)
                c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        initialised = 1;
    }
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i)
        crc = table[(crc ^ buf[i]) & 0xFFu] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

static void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >>  8);
    p[3] = (uint8_t)(v      );
}

// Build the PNG payload for IDAT: zlib header + stored DEFLATE blocks +
// Adler-32 trailer. `raw` is the filter-prefixed scanline stream.
static uint8_t *zlib_stored_wrap(const uint8_t *raw, size_t raw_len,
                                 size_t *out_len)
{
    // Stored blocks: 5-byte header (BFINAL/BTYPE + LEN + NLEN) per ≤65535
    // bytes of payload. Worst-case output = raw + 2 (zlib header) + 4
    // (adler) + ceil(raw/65535) * 5.
    size_t n_blocks = (raw_len + 65534) / 65535;
    if (n_blocks == 0) n_blocks = 1;
    size_t cap = 2 + 4 + n_blocks * 5 + raw_len;
    uint8_t *buf = (uint8_t *) malloc(cap);
    if (buf == NULL) return NULL;
    size_t off = 0;
    buf[off++] = 0x78;  // CM=8 deflate, CINFO=7 (32k window)
    buf[off++] = 0x01;  // FCHECK so (0x78<<8 | 0x01) % 31 == 0; FLEVEL=0
    size_t left = raw_len;
    size_t read = 0;
    while (left > 0 || (left == 0 && raw_len == 0)) {
        size_t take = left > 65535 ? 65535 : left;
        int is_last = (left == take);
        buf[off++] = (uint8_t)(is_last ? 0x01 : 0x00);  // BFINAL, BTYPE=00
        buf[off++] = (uint8_t)(take & 0xFFu);
        buf[off++] = (uint8_t)((take >> 8) & 0xFFu);
        uint16_t nlen = (uint16_t)(~take);
        buf[off++] = (uint8_t)(nlen & 0xFFu);
        buf[off++] = (uint8_t)((nlen >> 8) & 0xFFu);
        if (take > 0) {
            memcpy(buf + off, raw + read, take);
            off += take;
            read += take;
            left -= take;
        }
        if (raw_len == 0) break;
    }
    // Adler-32 over the uncompressed raw.
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < raw_len; ++i) {
        a = (a + raw[i]) % 65521u;
        b = (b + a)      % 65521u;
    }
    uint32_t adler = (b << 16) | a;
    buf[off++] = (uint8_t)(adler >> 24);
    buf[off++] = (uint8_t)(adler >> 16);
    buf[off++] = (uint8_t)(adler >>  8);
    buf[off++] = (uint8_t)(adler      );
    *out_len = off;
    return buf;
}

static int write_chunk(FILE *fp, const char *type, const uint8_t *data,
                       size_t data_len)
{
    uint8_t hdr[8];
    put_be32(hdr, (uint32_t) data_len);
    memcpy(hdr + 4, type, 4);
    if (fwrite(hdr, 1, 8, fp) != 8) return -1;
    if (data_len > 0 && fwrite(data, 1, data_len, fp) != data_len) return -1;
    // CRC32 covers chunk type + data.
    size_t crc_buf_len = 4 + data_len;
    uint8_t *crc_buf = (uint8_t *) malloc(crc_buf_len);
    if (crc_buf == NULL) return -1;
    memcpy(crc_buf, type, 4);
    if (data_len > 0) memcpy(crc_buf + 4, data, data_len);
    uint32_t crc = crc32_iso_hdlc(crc_buf, crc_buf_len);
    free(crc_buf);
    uint8_t crc_be[4];
    put_be32(crc_be, crc);
    return (fwrite(crc_be, 1, 4, fp) == 4) ? 0 : -1;
}

static int write_png_rgb(const char *path,
                         const uint8_t *rgb, int width, int height)
{
    FILE *fp = fopen(path, "wb");
    if (fp == NULL) return -1;
    static const uint8_t sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    if (fwrite(sig, 1, 8, fp) != 8) { fclose(fp); return -1; }

    uint8_t ihdr[13];
    put_be32(ihdr + 0, (uint32_t) width);
    put_be32(ihdr + 4, (uint32_t) height);
    ihdr[8]  = 8;   // bit depth
    ihdr[9]  = 2;   // color type: truecolor RGB
    ihdr[10] = 0;   // compression: deflate
    ihdr[11] = 0;   // filter: standard
    ihdr[12] = 0;   // interlace: none
    if (write_chunk(fp, "IHDR", ihdr, sizeof ihdr) != 0) {
        fclose(fp); return -1;
    }

    // Build the filter-prefixed scanline stream.
    size_t row_bytes = (size_t) width * 3;
    size_t raw_len = (size_t) height * (1 + row_bytes);
    uint8_t *raw = (uint8_t *) malloc(raw_len);
    if (raw == NULL) { fclose(fp); return -1; }
    for (int y = 0; y < height; ++y) {
        uint8_t *dst = raw + (size_t) y * (1 + row_bytes);
        dst[0] = 0x00;  // filter: None
        memcpy(dst + 1, rgb + (size_t) y * row_bytes, row_bytes);
    }

    size_t idat_len = 0;
    uint8_t *idat = zlib_stored_wrap(raw, raw_len, &idat_len);
    free(raw);
    if (idat == NULL) { fclose(fp); return -1; }
    int rc = write_chunk(fp, "IDAT", idat, idat_len);
    free(idat);
    if (rc != 0) { fclose(fp); return -1; }

    if (write_chunk(fp, "IEND", NULL, 0) != 0) { fclose(fp); return -1; }
    fclose(fp);
    return 0;
}

// --------------------------------------------------------------------
// Minimal PDF 1.4 writer. Embeds the spectrogram + colorbar as raster
// XObjects (FlateDecode-wrapped via the same stored-DEFLATE helper the
// PNG writer uses) and draws axes / ticks / labels with vector PDF ops
// so the text stays sharp at any zoom — that's the point of the PDF
// export. Only depends on libc; no libharu, no Cairo.
// --------------------------------------------------------------------

// wf_opts_t holds every knob the spectrogram pipeline reads; defined
// here (rather than next to build_waterfall) so the PDF writer below
// can access its members. The axis-formatting helpers are forward-
// declared too — definitions live further down the file.
typedef struct wf_opts {
    int    fft_size;
    int    hop;
    int    out_rows;
    float  db_min;
    float  db_max;
    int    auto_clip;     // 1 = ignore db_min/db_max, derive from percentiles
    int    sample_rate;   // for the time + frequency axis labels
    double center_hz;     // baseband centre frequency for the freq-axis label
    double zoom_hz;       // visible bandwidth around DC; 0 or negative = full
    int    dc_notch;      // 1 = interpolate over DC bin (B210 LO spike); 0 = off
    int    dc_notch_bins; // half-width of the notch in FFT bins (each side)
    double display_bw_hz; // filled in by build_waterfall — the actual rendered BW
    time_t start_utc;     // capture start in UTC; 0 = unknown, render elapsed time
    float  display_db_lo;    // filled in by build_waterfall — the dB range mapped
    float  display_db_hi;    //   onto colormap indices [0..255]; lights the colorbar.
    float  display_db_floor; // estimated noise floor in dBFS (median of per-bin
                             // medians, FFT-window normalised). Added to the
                             // colorbar tick values so the operator reads
                             // absolute power rather than "delta above floor".
    float  power_offset_db;  // optional CLI shift (e.g. dBFS -> dBm); applied
                             // on top of display_db_floor at label time.
    char   power_unit[8];    // "dBFS" by default, "dBm" if power_offset_db set.
} wf_opts_t;
static double pick_tick_step(double range, int target_ticks);
static double pick_time_step(double range_s, int target_ticks);
static int    fmt_freq(double hz, char *out, size_t out_cap);
static int    fmt_time(time_t base_utc, double sec, char *out, size_t out_cap);

typedef struct {
    FILE   *fp;
    size_t  off;
    size_t  obj_offsets[64];  // byte offset of each object body
    int     n_objs;
} pdf_writer_t;

static void pdf_write(pdf_writer_t *w, const void *buf, size_t n)
{
    fwrite(buf, 1, n, w->fp);
    w->off += n;
}
static void pdf_printf(pdf_writer_t *w, const char *fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) pdf_write(w, buf, (size_t) n);
}
static int pdf_begin_obj(pdf_writer_t *w)
{
    int id = ++w->n_objs;
    w->obj_offsets[id] = w->off;
    pdf_printf(w, "%d 0 obj\n", id);
    return id;
}
static void pdf_end_obj(pdf_writer_t *w)
{
    pdf_printf(w, "\nendobj\n");
}

// Build the FlateDecode-wrapped image stream for one RGB raster and
// emit the corresponding /XObject /Subtype /Image object.
static int pdf_emit_image_obj(pdf_writer_t *w,
                              const uint8_t *rgb, int img_w, int img_h)
{
    size_t raw_len = (size_t) img_w * (size_t) img_h * 3;
    size_t z_len = 0;
    uint8_t *z = zlib_stored_wrap(rgb, raw_len, &z_len);
    if (z == NULL) return -1;
    int id = pdf_begin_obj(w);
    pdf_printf(w,
        "<< /Type /XObject /Subtype /Image /Width %d /Height %d "
        "/ColorSpace /DeviceRGB /BitsPerComponent 8 "
        "/Filter /FlateDecode /Length %zu >>\nstream\n",
        img_w, img_h, z_len);
    pdf_write(w, z, z_len);
    free(z);
    pdf_printf(w, "\nendstream");
    pdf_end_obj(w);
    return id;
}

// PDF-escape a label so parentheses / backslashes don't break the
// surrounding (...) string literal. Tick labels are short ASCII; small
// out buffer is fine.
static void pdf_escape(const char *in, char *out, size_t out_cap)
{
    size_t o = 0;
    for (const char *p = in; *p && o + 2 < out_cap; ++p) {
        if (*p == '(' || *p == ')' || *p == '\\') out[o++] = '\\';
        out[o++] = *p;
    }
    out[o] = '\0';
}

// Vertically-flipped copy of `src` (row 0 of out = last row of src).
// PDF image XObjects render scanline 0 at the TOP of the destination
// rectangle, but spec_rgb stores oldest-first → we need to feed it
// flipped so "start of recording" lands at the BOTTOM of the page, the
// same convention we adopted for the PNG output.
static uint8_t *flip_rgb_vertical(const uint8_t *src, int w, int h)
{
    size_t row_bytes = (size_t) w * 3;
    uint8_t *out = (uint8_t *) malloc(row_bytes * (size_t) h);
    if (out == NULL) return NULL;
    for (int r = 0; r < h; ++r) {
        memcpy(out + (size_t) r * row_bytes,
               src + (size_t)(h - 1 - r) * row_bytes,
               row_bytes);
    }
    return out;
}

// Build the colorbar pixels (16 wide × spec_h tall). The bar runs from
// high dB at the top to low dB at the bottom — same orientation as the
// PNG colorbar in render_with_axes.
static uint8_t *build_colorbar_rgb(int height, int *out_w)
{
    const int W = 16;
    uint8_t *rgb = (uint8_t *) malloc((size_t) W * (size_t) height * 3);
    if (rgb == NULL) return NULL;
    for (int y = 0; y < height; ++y) {
        float t = 1.0f - (float) y / (float)(height > 1 ? height - 1 : 1);
        int idx = (int)(t * 255.0f + 0.5f);
        if (idx < 0) idx = 0;
        if (idx > 255) idx = 255;
        uint8_t r = VIRIDIS[idx][0];
        uint8_t g = VIRIDIS[idx][1];
        uint8_t b = VIRIDIS[idx][2];
        for (int x = 0; x < W; ++x) {
            uint8_t *p = rgb + ((size_t) y * (size_t) W + (size_t) x) * 3;
            p[0] = r; p[1] = g; p[2] = b;
        }
    }
    *out_w = W;
    return rgb;
}

// Write a one-page PDF with the spectrogram + colorbar embedded as
// XObjects and all axes / ticks / labels emitted as vector PDF
// operators. Uses Helvetica (standard 14, no embedding). Page size in
// points = image size in pixels, so screen-zooming the PDF preserves
// the on-screen scale of the PNG.
static int write_pdf_with_axes(const char *path,
                                const uint8_t *spec_rgb, int spec_w, int spec_h,
                                const wf_opts_t *opt, double duration_s)
{
    if (path == NULL || spec_rgb == NULL || spec_w <= 0 || spec_h <= 0) return -1;

    const int LM = 80, TM = 12, BM = 28;
    const int CB_GAP = 12;
    const int CB_W   = 16;
    const int CB_LABEL_SPACE = 60;
    const int W = LM + spec_w + CB_GAP + CB_W + CB_LABEL_SPACE;
    const int H = TM + spec_h + BM;

    FILE *fp = fopen(path, "wb");
    if (fp == NULL) return -1;
    pdf_writer_t w = { fp, 0, {0}, 0 };

    pdf_printf(&w, "%%PDF-1.4\n%%\xE2\xE3\xCF\xD3\n");

    // Pre-flip the spectrogram so row 0 is the newest sample (top of
    // page), matching the PNG layout where start-of-recording is at
    // the bottom of the image.
    uint8_t *flip = flip_rgb_vertical(spec_rgb, spec_w, spec_h);
    if (flip == NULL) { fclose(fp); return -1; }
    int cb_w = 0;
    uint8_t *cb_rgb = build_colorbar_rgb(spec_h, &cb_w);
    if (cb_rgb == NULL) { free(flip); fclose(fp); return -1; }

    int spec_id = pdf_emit_image_obj(&w, flip, spec_w, spec_h);
    free(flip);
    int cb_id   = pdf_emit_image_obj(&w, cb_rgb, cb_w, spec_h);
    free(cb_rgb);
    if (spec_id < 0 || cb_id < 0) { fclose(fp); return -1; }

    // Helvetica (standard 14 PDF font, no embedding needed).
    int font_id = pdf_begin_obj(&w);
    pdf_printf(&w,
        "<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica "
        "/Encoding /WinAnsiEncoding >>");
    pdf_end_obj(&w);

    // Build the content stream in memory so we know its length before
    // writing the dictionary.
    size_t cs_cap = 65536;
    size_t cs_len = 0;
    char *cs = (char *) malloc(cs_cap);
    if (cs == NULL) { fclose(fp); return -1; }
#define CS_APPEND(...) do {                                                   \
    if (cs_len + 512 > cs_cap) {                                              \
        cs_cap *= 2;                                                          \
        char *_nc = (char *) realloc(cs, cs_cap);                             \
        if (_nc == NULL) { free(cs); fclose(fp); return -1; }                 \
        cs = _nc;                                                             \
    }                                                                         \
    int _n = snprintf(cs + cs_len, cs_cap - cs_len, __VA_ARGS__);             \
    if (_n > 0) cs_len += (size_t) _n;                                        \
} while (0)

    // 1. Spectrogram raster. PDF y-up: place at (LM, BM) → (LM+spec_w, BM+spec_h).
    CS_APPEND("q %d 0 0 %d %d %d cm /Im1 Do Q\n",
              spec_w, spec_h, LM, BM);
    // 2. Colorbar raster.
    int cb_x = LM + spec_w + CB_GAP;
    CS_APPEND("q %d 0 0 %d %d %d cm /Im2 Do Q\n",
              CB_W, spec_h, cb_x, BM);

    // 3. Axes: thin gray rule between margins and image. PDF default
    // line color is black; set to mid-gray for the axes.
    CS_APPEND("0.65 G 0.5 w\n");
    // Spectrogram bottom edge (frequency axis baseline)
    CS_APPEND("%d %d m %d %d l S\n", LM, BM, LM + spec_w, BM);
    // Spectrogram left edge (time axis baseline)
    CS_APPEND("%d %d m %d %d l S\n", LM, BM, LM, BM + spec_h);
    // Colorbar frame
    CS_APPEND("%d %d %d %d re S\n", cb_x, BM, CB_W, spec_h);

    // 4. Frequency-axis ticks + labels (X axis, below the spectrogram).
    CS_APPEND("0.7 G\n");
    double fs = (opt->display_bw_hz > 0.0)
                ? opt->display_bw_hz : (double) opt->sample_rate;
    double f_lo = -fs / 2.0, f_hi = +fs / 2.0;
    double f_step = pick_tick_step(fs, 8);
    double f0 = ceil(f_lo / f_step) * f_step;
    CS_APPEND("BT /F1 8 Tf 0 g\n");
    for (double f = f0; f <= f_hi + 0.5 * f_step; f += f_step) {
        double frac = (f - f_lo) / fs;
        double x = (double) LM + frac * (double) spec_w;
        // Tick below the axis.
        CS_APPEND("ET 0.65 G %.1f %d m %.1f %d l S BT /F1 8 Tf 0 g\n",
                  x, BM, x, BM - 4);
        char buf[24], esc[40];
        double f_abs = (opt->center_hz != 0.0) ? (opt->center_hz + f) : f;
        if (opt->center_hz != 0.0) fmt_freq(f_abs, buf, sizeof buf);
        else                       fmt_freq(f,     buf, sizeof buf);
        pdf_escape(buf, esc, sizeof esc);
        // Centre the label under the tick.
        int label_w_pt = (int) strlen(buf) * 4;  // rough Helvetica 8pt width
        CS_APPEND("1 0 0 1 %.1f %d Tm (%s) Tj\n",
                  x - label_w_pt / 2.0, BM - 14, esc);
    }

    // 5. Time-axis ticks + labels (Y axis, left of the spectrogram).
    // Major ticks with labels, minor 20 s ticks unlabeled.
    double t_step = pick_time_step(duration_s, 10);
    for (double t = 0.0; t <= duration_s + 0.5; t += 20.0) {
        double mod = fmod(t, t_step);
        if (fabs(mod) < 1.0 || fabs(mod - t_step) < 1.0) continue;
        double frac = 1.0 - t / duration_s;
        double y = BM + frac * (double) spec_h;
        // Time runs upward in the PDF (y-up), but the start-of-recording
        // is at the bottom of the spectrogram → flip: y = BM + (1-t/d)*h
        // means t=0 → y = BM+h (top). Wait, we want t=0 at BOTTOM (BM).
        // Recompute: bottom = oldest (t=0), top = newest (t=d).
        y = BM + (t / duration_s) * (double) spec_h;
        CS_APPEND("ET 0.65 G %d %.1f m %d %.1f l S BT /F1 8 Tf 0 g\n",
                  LM - 3, y, LM, y);
    }
    for (double t = 0.0; t <= duration_s + 0.5 * t_step; t += t_step) {
        double y = BM + (t / duration_s) * (double) spec_h;
        CS_APPEND("ET 0.65 G %d %.1f m %d %.1f l S BT /F1 8 Tf 0 g\n",
                  LM - 6, y, LM, y);
        char buf[24], esc[40];
        fmt_time(opt->start_utc, t, buf, sizeof buf);
        pdf_escape(buf, esc, sizeof esc);
        int label_w_pt = (int) strlen(buf) * 5;
        CS_APPEND("1 0 0 1 %d %.1f Tm (%s) Tj\n",
                  LM - 10 - label_w_pt, y - 3, esc);
    }

    // 6. Colorbar dB ticks + labels. Same offset trick as the PNG: tick
    // positions are in median-subtracted space (matching the colormap),
    // tick labels are absolute power (dBFS by default, dBm if the user
    // passed --power-offset).
    float db_lo = opt->display_db_lo, db_hi = opt->display_db_hi;
    float db_range = db_hi - db_lo;
    double label_offset_pdf = (double) opt->display_db_floor
                            + (double) opt->power_offset_db;
    if (db_range > 0.0f) {
        double db_step = pick_tick_step(db_range, 6);
        double db0 = ceil(db_lo / db_step) * db_step;
        for (double v = db0; v <= db_hi + 0.5 * db_step; v += db_step) {
            float frac = (float)((v - db_lo) / db_range);
            double y = BM + frac * (double) spec_h;
            CS_APPEND("ET 0.65 G %d %.1f m %d %.1f l S BT /F1 8 Tf 0 g\n",
                      cb_x + CB_W, y, cb_x + CB_W + 4, y);
            char buf[24], esc[40];
            snprintf(buf, sizeof buf, "%.0f", v + label_offset_pdf);
            pdf_escape(buf, esc, sizeof esc);
            CS_APPEND("1 0 0 1 %d %.1f Tm (%s) Tj\n",
                      cb_x + CB_W + 7, y - 3, esc);
        }
    }
    // Unit label ("dBFS" / "dBm") above the colorbar in Helvetica.
    {
        char esc[16];
        pdf_escape(opt->power_unit, esc, sizeof esc);
        CS_APPEND("1 0 0 1 %d %d Tm (%s) Tj\n",
                  cb_x, BM + spec_h + 4, esc);
    }

    CS_APPEND("ET\n");
#undef CS_APPEND

    // Content stream object.
    int cs_id = pdf_begin_obj(&w);
    pdf_printf(&w, "<< /Length %zu >>\nstream\n", cs_len);
    pdf_write(&w, cs, cs_len);
    pdf_printf(&w, "\nendstream");
    pdf_end_obj(&w);
    free(cs);

    // Page object.
    int page_id = pdf_begin_obj(&w);
    pdf_printf(&w,
        "<< /Type /Page /Parent %%PARENT%% /MediaBox [0 0 %d %d] "
        "/Resources << /Font << /F1 %d 0 R >> "
        "/XObject << /Im1 %d 0 R /Im2 %d 0 R >> >> "
        "/Contents %d 0 R >>",
        W, H, font_id, spec_id, cb_id, cs_id);
    pdf_end_obj(&w);

    // Pages object.
    int pages_id = pdf_begin_obj(&w);
    pdf_printf(&w,
        "<< /Type /Pages /Kids [%d 0 R] /Count 1 >>", page_id);
    pdf_end_obj(&w);

    // We needed the Page object's Parent to point at Pages, but Pages
    // wasn't assigned until after. Patch via a second pass would be
    // ugly; instead we substitute the placeholder string we left in
    // the page's body. Simpler: just rewind, find "%PARENT%", and edit
    // — but we used stream output. Take the easy way: rewrite the
    // Page object now that pages_id is known.
    // (Streamed approach: rebuild and store using the right ID below.)
    // Patch: seek back to the page object's start and rewrite.
    long save_off = (long) w.off;
    if (fseek(w.fp, (long) w.obj_offsets[page_id], SEEK_SET) != 0) {
        fclose(fp); return -1;
    }
    // Calculate the byte count of the original page body so we can
    // overwrite it in place. The "%PARENT%" token is 8 bytes; the new
    // "<n> 0 R" is up to 7 bytes — we pad with spaces if shorter.
    char page_body[512];
    int n = snprintf(page_body, sizeof page_body,
        "%d 0 obj\n"
        "<< /Type /Page /Parent %d 0 R /MediaBox [0 0 %d %d] "
        "/Resources << /Font << /F1 %d 0 R >> "
        "/XObject << /Im1 %d 0 R /Im2 %d 0 R >> >> "
        "/Contents %d 0 R >>\nendobj\n",
        page_id, pages_id, W, H, font_id, spec_id, cb_id, cs_id);
    // The original write placed "%PARENT%" (8 chars) where pages_id will
    // print; the new body is shorter or equal — pad with spaces to keep
    // file layout intact for the xref.
    long page_len_old = (long) w.obj_offsets[pages_id]
                       - (long) w.obj_offsets[page_id];
    if (n > 0 && n < page_len_old) {
        memset(page_body + n, ' ', (size_t)(page_len_old - n - 1));
        page_body[page_len_old - 1] = '\n';
        fwrite(page_body, 1, (size_t) page_len_old, w.fp);
    } else if (n > 0) {
        fwrite(page_body, 1, (size_t) n, w.fp);
    }
    fseek(w.fp, save_off, SEEK_SET);
    w.off = (size_t) save_off;

    // Catalog.
    int catalog_id = pdf_begin_obj(&w);
    pdf_printf(&w,
        "<< /Type /Catalog /Pages %d 0 R >>", pages_id);
    pdf_end_obj(&w);

    // xref table.
    size_t xref_off = w.off;
    pdf_printf(&w, "xref\n0 %d\n", w.n_objs + 1);
    pdf_printf(&w, "0000000000 65535 f \n");
    for (int i = 1; i <= w.n_objs; ++i) {
        pdf_printf(&w, "%010zu 00000 n \n", w.obj_offsets[i]);
    }
    pdf_printf(&w,
        "trailer\n<< /Size %d /Root %d 0 R >>\nstartxref\n%zu\n%%%%EOF\n",
        w.n_objs + 1, catalog_id, xref_off);

    fclose(fp);
    return 0;
}

// --------------------------------------------------------------------
// Spectrogram pipeline
// --------------------------------------------------------------------

// (wf_opts_t is defined above, where the PDF writer also needs it.)

// Parse "YYYYMMDDTHHMMSS" out of `s` (a longer suffix like ".698.iq" is
// fine, sscanf ignores trailing bytes). Returns 0 on parse failure.
static time_t parse_ut_string(const char *s)
{
    if (s == NULL) return 0;
    int Y, M, D, h, m, sec;
    char T;
    if (sscanf(s, "%4d%2d%2d%c%2d%2d%2d",
               &Y, &M, &D, &T, &h, &m, &sec) != 7) return 0;
    if (T != 'T') return 0;
    struct tm tm = {0};
    tm.tm_year = Y - 1900;
    tm.tm_mon  = M - 1;
    tm.tm_mday = D;
    tm.tm_hour = h;
    tm.tm_min  = m;
    tm.tm_sec  = sec;
    return timegm(&tm);
}

// simple_sat_ops names IQ files <prefix>_UT=YYYYMMDDTHHMMSS.fff.iq. Pull
// the UT timestamp out of the filename. Returns 0 if there is no "UT=".
static time_t parse_ut_from_path(const char *path)
{
    if (path == NULL) return 0;
    const char *p = strstr(path, "UT=");
    if (p == NULL) return 0;
    return parse_ut_string(p + 3);
}

static float median_inplace(float *buf, int n)
{
    // Quickselect-style nth_element. Mutates buf. Good enough for our
    // per-bin background subtraction; n is at most a few thousand.
    if (n <= 0) return 0.0f;
    int lo = 0, hi = n - 1, k = n / 2;
    while (lo < hi) {
        float pivot = buf[(lo + hi) / 2];
        int i = lo, j = hi;
        while (i <= j) {
            while (buf[i] < pivot) ++i;
            while (buf[j] > pivot) --j;
            if (i <= j) {
                float t = buf[i]; buf[i] = buf[j]; buf[j] = t;
                ++i; --j;
            }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else return buf[k];
    }
    return buf[k];
}

static int build_waterfall(const int16_t *iq, size_t n_pairs,
                            const wf_opts_t *opt,
                            uint8_t **out_rgb, int *out_w, int *out_h)
{
    int N = opt->fft_size;
    int H = opt->hop;
    if (!is_pow2((unsigned) N) || N < 16 || H <= 0 || H > N) {
        fprintf(stderr, "gen_waterfall: invalid fft/hop\n");
        return -1;
    }
    if (n_pairs < (size_t) N) {
        fprintf(stderr, "gen_waterfall: only %zu IQ pairs, need >= %d\n",
                n_pairs, N);
        return -1;
    }

    // Hann window scaled to int16 input.
    float *win = (float *) malloc((size_t) N * sizeof(float));
    if (!win) return -1;
    for (int i = 0; i < N; ++i) {
        win[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * i / (N - 1)));
    }

    // Number of input FFT frames (raw, before time-binning).
    size_t n_frames = (n_pairs - (size_t) N) / (size_t) H + 1;
    if (n_frames < 1) { free(win); return -1; }

    // Allocate the high-resolution spectrogram: n_frames × N linear
    // magnitude² values. Keeping the per-frame data in the LINEAR
    // domain matters for the time-binning step below — averaging in
    // dB-space pulls the average toward the geometric mean rather
    // than the arithmetic mean, which on a sparse-burst capture (a
    // single bright cell mixed with mostly noise) buries the burst.
    // SatNOGS-style waterfalls keep linear sums until the final log.
    float *spec = (float *) malloc(n_frames * (size_t) N * sizeof(float));
    if (!spec) { free(win); return -1; }

    float *re = (float *) malloc((size_t) N * sizeof(float));
    float *im = (float *) malloc((size_t) N * sizeof(float));
    if (!re || !im) { free(win); free(spec); free(re); free(im); return -1; }

    for (size_t fi = 0; fi < n_frames; ++fi) {
        size_t base = fi * (size_t) H;
        for (int i = 0; i < N; ++i) {
            float w = win[i];
            float I = (float) iq[(base + (size_t) i) * 2 + 0] / 32768.0f;
            float Q = (float) iq[(base + (size_t) i) * 2 + 1] / 32768.0f;
            re[i] = I * w;
            im[i] = Q * w;
        }
        fft_forward(re, im, (unsigned) N);
        // |Z(f)|², with fftshift: bin 0 of the OUTPUT is the most
        // negative frequency (-Fs/2), bin N/2 is DC, bin N-1 is +Fs/2 - bin.
        for (int k = 0; k < N; ++k) {
            int src = (k + N / 2) % N;
            float mag2 = re[src] * re[src] + im[src] * im[src];
            spec[fi * (size_t) N + (size_t) k] = mag2;
        }
    }
    free(win); free(re); free(im);

    // Time-bin frames into output rows by summing the linear power and
    // converting to dB at the very end (single log per output cell,
    // not per input frame).
    int out_rows = opt->out_rows;
    if (out_rows <= 0) out_rows = 1080;
    if ((size_t) out_rows > n_frames) out_rows = (int) n_frames;
    float *binned = (float *) malloc((size_t) out_rows * (size_t) N * sizeof(float));
    if (!binned) { free(spec); return -1; }
    for (int r = 0; r < out_rows; ++r) {
        size_t a = (size_t) r * n_frames / (size_t) out_rows;
        size_t b = (size_t)(r + 1) * n_frames / (size_t) out_rows;
        if (b <= a) b = a + 1;
        if (b > n_frames) b = n_frames;
        size_t span = b - a;
        double inv = 1.0 / (double) span;
        for (int k = 0; k < N; ++k) {
            double sum = 0.0;
            for (size_t fi = a; fi < b; ++fi) {
                sum += spec[fi * (size_t) N + (size_t) k];
            }
            binned[(size_t) r * (size_t) N + (size_t) k] =
                (float)(10.0 * log10(sum * inv + 1e-20));
        }
    }
    free(spec);

    // Per-bin median subtraction so the noise floor flattens out. The
    // per-bin medians also feed an absolute-dBFS calibration for the
    // colorbar labels — without that step the bar reads "delta above
    // bin median", which is mathematically defensible but looks like
    // "0..10" with no units in the operator's eye.
    float *col = (float *) malloc((size_t) out_rows * sizeof(float));
    float *bin_medians = (float *) malloc((size_t) N * sizeof(float));
    if (!col || !bin_medians) {
        free(col); free(bin_medians); free(binned); return -1;
    }
    for (int k = 0; k < N; ++k) {
        for (int r = 0; r < out_rows; ++r) {
            col[r] = binned[(size_t) r * (size_t) N + (size_t) k];
        }
        float med = median_inplace(col, out_rows);
        bin_medians[k] = med;
        for (int r = 0; r < out_rows; ++r) {
            binned[(size_t) r * (size_t) N + (size_t) k] -= med;
        }
    }
    // Global noise floor estimate: median of the per-bin medians. The
    // raw FFT-power values are referenced to peak |Z|^2 = (N/2)^2 for
    // a full-scale complex tone (Hann coherent gain = 0.5), so subtract
    // 20*log10(N/2) to land on dBFS. With N=1024 that's ~54 dB.
    float floor_raw_db = median_inplace(bin_medians, N);
    float fft_scale_db = 20.0f * (float) log10((double) opt->fft_size / 2.0);
    ((wf_opts_t *) opt)->display_db_floor = floor_raw_db - fft_scale_db;
    free(bin_medians);
    free(col);

    // B210 direct-conversion LO bleed lands on the DC bin and a couple
    // of bins either side, even after median subtraction (the leakage
    // isn't quite constant — slow AGC + LO drift). Replace those bins
    // with the value of the first clean bin outside the notch so the
    // spike doesn't dominate the colormap. SatNOGS waterfalls don't
    // show this artefact because typical SatNOGS stations (RTL-SDR /
    // Airspy) deliberately tune a few kHz off-channel and shift in
    // software so DC never sits inside the signal lobe.
    int dc_k = N / 2;
    int notch = opt->dc_notch_bins > 0 ? opt->dc_notch_bins : 2;
    if (opt->dc_notch && dc_k - notch - 1 >= 0 && dc_k + notch + 1 < N) {
        for (int r = 0; r < out_rows; ++r) {
            float left  = binned[(size_t) r * (size_t) N + (size_t)(dc_k - notch - 1)];
            float right = binned[(size_t) r * (size_t) N + (size_t)(dc_k + notch + 1)];
            float fill  = 0.5f * (left + right);
            for (int off = -notch; off <= notch; ++off) {
                binned[(size_t) r * (size_t) N + (size_t)(dc_k + off)] = fill;
            }
        }
    }

    // Zoom: keep only the columns inside ±zoom_hz/2 around DC. Default
    // for this stack (48 kHz IQ, 9600 baud MSK, ~±10 kHz Doppler) is
    // ±15 kHz — fits the data lobe + Doppler tracks with a couple of
    // dB of margin and ditches the corners that are nothing but noise
    // floor. Pass --full-width to disable.
    int k_lo = 0;
    int k_hi = N - 1;
    double display_bw = (double) opt->sample_rate;
    if (opt->zoom_hz > 0.0 && opt->zoom_hz < (double) opt->sample_rate) {
        double half = opt->zoom_hz / 2.0;
        int half_bins = (int)(half / (double) opt->sample_rate * (double) N);
        if (half_bins < 1) half_bins = 1;
        if (half_bins > N / 2 - 1) half_bins = N / 2 - 1;
        k_lo = dc_k - half_bins;
        k_hi = dc_k + half_bins;
        display_bw = 2.0 * half_bins * (double) opt->sample_rate / (double) N;
    }
    ((wf_opts_t *) opt)->display_bw_hz = display_bw;
    int zoom_N = k_hi - k_lo + 1;
    if (zoom_N != N) {
        float *cropped = (float *) malloc((size_t) out_rows * (size_t) zoom_N
                                          * sizeof(float));
        if (!cropped) { free(binned); return -1; }
        for (int r = 0; r < out_rows; ++r) {
            memcpy(cropped + (size_t) r * (size_t) zoom_N,
                   binned + (size_t) r * (size_t) N + (size_t) k_lo,
                   (size_t) zoom_N * sizeof(float));
        }
        free(binned);
        binned = cropped;
        N = zoom_N;
    }

    // Auto-clip the dB range to data percentiles unless the caller
    // pinned both endpoints. Most real captures have a noise floor a
    // few dB wide and bright peaks 20-40 dB above it; the old fixed
    // -3..+20 dB range left the image purple-dim whenever the burst
    // SNR wasn't already in that band. Walk the post-median-subtraction
    // values, sample a 5th- and 99th-percentile pair, and use those as
    // the visible range. The user's --db-min / --db-max still override.
    float lo = opt->db_min, hi = opt->db_max;
    if (opt->auto_clip) {
        size_t n_cells = (size_t) out_rows * (size_t) N;
        // Sample-and-sort a subset to keep the percentile estimate fast.
        size_t sample_n = n_cells / 64;
        if (sample_n < 4096)   sample_n = 4096;
        if (sample_n > 200000) sample_n = 200000;
        if (sample_n > n_cells) sample_n = n_cells;
        float *sample = (float *) malloc(sample_n * sizeof(float));
        if (sample) {
            uint32_t state = 0xC0FFEEu;
            for (size_t i = 0; i < sample_n; ++i) {
                // xorshift32 — deterministic per-run, no need for proper RNG.
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                sample[i] = binned[(size_t) state % n_cells];
            }
            // Partial sort via stdlib qsort would be fine for sample_n
            // up to 200k; reuse the inplace selector for the two
            // percentiles instead — half the work.
            float p05 = median_inplace(sample, (int) sample_n);
            // median_inplace mutates and returns the median; redo with
            // a fresh copy for each percentile.
            for (size_t i = 0; i < sample_n; ++i) {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                sample[i] = binned[(size_t) state % n_cells];
            }
            // 99th percentile via quickselect: shuffle then take element
            // at index 0.99 * n.
            size_t k99 = (size_t)((double) sample_n * 0.99);
            if (k99 >= sample_n) k99 = sample_n - 1;
            {
                int lo_i = 0, hi_i = (int) sample_n - 1;
                while (lo_i < hi_i) {
                    float pivot = sample[(lo_i + hi_i) / 2];
                    int i = lo_i, j = hi_i;
                    while (i <= j) {
                        while (sample[i] < pivot) ++i;
                        while (sample[j] > pivot) --j;
                        if (i <= j) {
                            float t = sample[i]; sample[i] = sample[j]; sample[j] = t;
                            ++i; --j;
                        }
                    }
                    if ((int) k99 <= j) hi_i = j;
                    else if ((int) k99 >= i) lo_i = i;
                    else break;
                }
            }
            float p99 = sample[k99];
            free(sample);
            // Floor at p05 minus 1 dB so the noise floor maps to the
            // darkest cell, ceiling at max(p99, lo + 12) so we always
            // get at least 12 dB of visible range even on a quiet
            // capture (otherwise viridis compresses too hard).
            lo = p05 - 1.0f;
            hi = p99;
            if (hi < lo + 12.0f) hi = lo + 12.0f;
        }
    }

    // Record the dB range we actually used so render_with_axes can light
    // a matching colorbar in the right margin.
    ((wf_opts_t *) opt)->display_db_lo = lo;
    ((wf_opts_t *) opt)->display_db_hi = hi;

    // Map to viridis. Width = N (already fftshift'd). Height = out_rows.
    // The PNG's row 0 is the TOP of the image, so iterate as-is — the
    // first output row corresponds to the earliest IQ samples, which
    // matches the SatNOGS convention of "earliest at top".
    uint8_t *rgb = (uint8_t *) malloc((size_t) N * (size_t) out_rows * 3);
    if (!rgb) { free(binned); return -1; }
    float scale = (hi > lo) ? (255.0f / (hi - lo)) : 1.0f;
    for (int r = 0; r < out_rows; ++r) {
        for (int k = 0; k < N; ++k) {
            float v = binned[(size_t) r * (size_t) N + (size_t) k];
            float norm = (v - lo) * scale;
            int idx = (int) norm;
            if (idx < 0) idx = 0;
            if (idx > 255) idx = 255;
            uint8_t *p = rgb + ((size_t) r * (size_t) N + (size_t) k) * 3;
            p[0] = VIRIDIS[idx][0];
            p[1] = VIRIDIS[idx][1];
            p[2] = VIRIDIS[idx][2];
        }
    }
    free(binned);

    *out_rgb = rgb;
    *out_w = N;
    *out_h = out_rows;
    return 0;
}

// --------------------------------------------------------------------
// Bitmap font + axis renderer
// --------------------------------------------------------------------
//
// Hand-rolled 5x7 bitmap glyphs for just the characters we need on the
// axes ("0-9 . + - : k M H z s t f T space"). Each row of a glyph is
// the low 5 bits of one byte, MSB-of-the-5-bits is the leftmost pixel.
// Drawn at 2× scale so labels stay legible on 1080-row spectrograms.

static const uint8_t G_0[7]   = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E};
static const uint8_t G_1[7]   = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E};
static const uint8_t G_2[7]   = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F};
static const uint8_t G_3[7]   = {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E};
static const uint8_t G_4[7]   = {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02};
static const uint8_t G_5[7]   = {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E};
static const uint8_t G_6[7]   = {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E};
static const uint8_t G_7[7]   = {0x1F,0x01,0x02,0x04,0x08,0x08,0x08};
static const uint8_t G_8[7]   = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E};
static const uint8_t G_9[7]   = {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C};
static const uint8_t G_dot[7] = {0x00,0x00,0x00,0x00,0x00,0x06,0x06};
static const uint8_t G_plu[7] = {0x00,0x04,0x04,0x1F,0x04,0x04,0x00};
static const uint8_t G_min[7] = {0x00,0x00,0x00,0x1F,0x00,0x00,0x00};
static const uint8_t G_col[7] = {0x00,0x06,0x06,0x00,0x06,0x06,0x00};
static const uint8_t G_k[7]   = {0x10,0x10,0x12,0x14,0x18,0x14,0x12};
static const uint8_t G_M[7]   = {0x11,0x1B,0x15,0x11,0x11,0x11,0x11};
static const uint8_t G_H[7]   = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11};
static const uint8_t G_z[7]   = {0x00,0x00,0x1F,0x02,0x04,0x08,0x1F};
static const uint8_t G_s[7]   = {0x00,0x00,0x0F,0x10,0x0E,0x01,0x1E};
static const uint8_t G_t[7]   = {0x08,0x08,0x1E,0x08,0x08,0x09,0x06};
static const uint8_t G_f[7]   = {0x06,0x09,0x08,0x1E,0x08,0x08,0x08};
static const uint8_t G_T[7]   = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04};
static const uint8_t G_sp[7]  = {0,0,0,0,0,0,0};
// Lowercase d, uppercase B/F/S, lowercase m — needed to render the
// colorbar unit label ("dBFS" / "dBm") with the same bitmap font.
static const uint8_t G_d[7]   = {0x01,0x01,0x01,0x0F,0x11,0x11,0x0F};
static const uint8_t G_B[7]   = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E};
static const uint8_t G_F[7]   = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10};
static const uint8_t G_S[7]   = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E};
static const uint8_t G_m[7]   = {0x00,0x00,0x1A,0x15,0x15,0x15,0x15};

static const uint8_t *glyph_for(char c)
{
    switch (c) {
        case '0': return G_0;  case '1': return G_1;
        case '2': return G_2;  case '3': return G_3;
        case '4': return G_4;  case '5': return G_5;
        case '6': return G_6;  case '7': return G_7;
        case '8': return G_8;  case '9': return G_9;
        case '.': return G_dot; case '+': return G_plu;
        case '-': return G_min; case ':': return G_col;
        case 'k': return G_k;   case 'M': return G_M;
        case 'H': return G_H;   case 'z': return G_z;
        case 's': return G_s;   case 't': return G_t;
        case 'f': return G_f;   case 'T': return G_T;
        case 'd': return G_d;   case 'B': return G_B;
        case 'F': return G_F;   case 'S': return G_S;
        case 'm': return G_m;
        default:  return G_sp;
    }
}

static void px_set(uint8_t *rgb, int W, int H, int x, int y,
                   uint8_t r, uint8_t g, uint8_t b)
{
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    uint8_t *p = rgb + ((size_t) y * (size_t) W + (size_t) x) * 3;
    p[0] = r; p[1] = g; p[2] = b;
}

static void draw_text(uint8_t *rgb, int W, int H, int x, int y,
                      const char *s, int scale,
                      uint8_t r, uint8_t g, uint8_t b)
{
    int cx = x;
    for (; *s; ++s) {
        const uint8_t *gl = glyph_for(*s);
        for (int gy = 0; gy < 7; ++gy) {
            uint8_t row = gl[gy];
            for (int gx = 0; gx < 5; ++gx) {
                if (row & (1 << (4 - gx))) {
                    for (int sy = 0; sy < scale; ++sy)
                        for (int sx = 0; sx < scale; ++sx)
                            px_set(rgb, W, H,
                                   cx + gx * scale + sx,
                                   y  + gy * scale + sy,
                                   r, g, b);
                }
            }
        }
        cx += (5 + 1) * scale;
    }
}

static int text_width(const char *s, int scale) {
    int n = 0;
    for (const char *p = s; *p; ++p) ++n;
    return n * 6 * scale;
}

// Pick a "nice" tick step (1·10ⁿ, 2·10ⁿ, or 5·10ⁿ) close to range/target.
static double pick_tick_step(double range, int target_ticks)
{
    if (range <= 0.0 || target_ticks < 1) return 1.0;
    double raw = range / (double) target_ticks;
    double p = pow(10.0, floor(log10(raw)));
    double frac = raw / p;
    double mul;
    if      (frac < 1.5) mul = 1.0;
    else if (frac < 3.5) mul = 2.0;
    else if (frac < 7.0) mul = 5.0;
    else                 mul = 10.0;
    return p * mul;
}

// Format a frequency value as "<n>kHz" or "<n.n>MHz" (no centre-Hz
// baseline added — pass the absolute or relative Hz the caller wants).
// Returns bytes written.
static int fmt_freq(double hz, char *out, size_t out_cap)
{
    double abs_hz = (hz < 0.0) ? -hz : hz;
    const char *sgn = (hz < 0.0) ? "-" : "+";
    if (abs_hz >= 1.0e6 - 1.0) {
        return snprintf(out, out_cap, "%s%.2fMHz", sgn, abs_hz / 1.0e6);
    } else {
        return snprintf(out, out_cap, "%s%.1fkHz", sgn, abs_hz / 1.0e3);
    }
}

static int fmt_time(time_t base_utc, double sec, char *out, size_t out_cap)
{
    if (sec < 0.0) sec = 0.0;
    if (base_utc != 0) {
        time_t t = base_utc + (time_t)(sec + 0.5);
        struct tm lt;
        localtime_r(&t, &lt);
        return snprintf(out, out_cap, "%02d:%02d:%02d",
                        lt.tm_hour, lt.tm_min, lt.tm_sec);
    }
    int total = (int)(sec + 0.5);
    int h = total / 3600;
    int m = (total / 60) % 60;
    int s = total % 60;
    return snprintf(out, out_cap, "%02d:%02d:%02d", h, m, s);
}

// Pick a "human" time step from a fixed table (1, 2, 5, 10, 15, 30 s,
// 1, 2, 5, 10, 15, 30 min, 1, 2, 4 h) so y-axis ticks land on round
// HH:MM:SS values like 00:01:00 / 00:05:00 rather than 100s / 200s.
static double pick_time_step(double range_s, int target_ticks)
{
    static const double steps[] = {
        1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600, 7200, 14400
    };
    int n = (int)(sizeof steps / sizeof steps[0]);
    if (range_s <= 0.0 || target_ticks < 1) return 1.0;
    double target = range_s / (double) target_ticks;
    double best = steps[0];
    double best_d = fabs(steps[0] - target);
    for (int i = 1; i < n; ++i) {
        double d = fabs(steps[i] - target);
        if (d < best_d) { best_d = d; best = steps[i]; }
    }
    return best;
}

// Compose the final PNG: spectrogram pixels in the centre, axis ticks
// + labels in the surrounding margins. Caller owns spec_rgb; the
// returned buffer is freshly malloc'd (caller frees).
static int render_with_axes(const uint8_t *spec_rgb, int spec_w, int spec_h,
                            const wf_opts_t *opt, double duration_s,
                            uint8_t **out_rgb, int *out_w, int *out_h)
{
    const int LM = 80;
    // RM widened from 12 to 76 to host the dB colorbar: 12 px gap +
    // 16 px strip + 6 px ticks + ~36 px of label space + a small
    // breathing-room margin. The colorbar height matches spec_h.
    const int RM = 76;
    const int TM = 12;
    const int BM = 28;
    int W = spec_w + LM + RM;
    int H = spec_h + TM + BM;
    uint8_t *rgb = (uint8_t *) malloc((size_t) W * (size_t) H * 3);
    if (!rgb) return -1;
    // Fill margins with near-black; the spectrogram fills the centre.
    for (size_t i = 0; i < (size_t) W * (size_t) H; ++i) {
        rgb[i*3+0] = 0; rgb[i*3+1] = 0; rgb[i*3+2] = 0;
    }
    // Flip vertically so the earliest samples land at the BOTTOM of
    // the image and time progresses upward — matches how an operator
    // reads a real-time waterfall (newest sample crawling up from the
    // bottom edge). spec_rgb[0..spec_h) is in capture order (oldest
    // first); we paint spec row r into image row (spec_h - 1 - r).
    for (int r = 0; r < spec_h; ++r) {
        int dst_row = TM + (spec_h - 1 - r);
        memcpy(rgb + ((size_t) dst_row * (size_t) W + (size_t) LM) * 3,
               spec_rgb + (size_t) r * (size_t) spec_w * 3,
               (size_t) spec_w * 3);
    }

    const uint8_t LBL_R = 220, LBL_G = 220, LBL_B = 220;
    const uint8_t TIC_R = 180, TIC_G = 180, TIC_B = 180;

    // Frequency axis: ±BW/2 across spec_w pixels, where BW is the
    // displayed bandwidth (either the full sample rate or the zoom
    // width set by --zoom-khz). centre_hz adds to the labels so a
    // tuned carrier reads as the absolute RF frequency around the
    // nominal carrier.
    double fs = (opt->display_bw_hz > 0.0)
                ? opt->display_bw_hz : (double) opt->sample_rate;
    double f_lo = -fs / 2.0;
    double f_hi = +fs / 2.0;
    double f_step = pick_tick_step(fs, 8);
    // Start at the smallest multiple of f_step that's >= f_lo.
    double f0 = ceil(f_lo / f_step) * f_step;
    for (double f = f0; f <= f_hi + 0.5 * f_step; f += f_step) {
        double frac = (f - f_lo) / fs;
        int x = LM + (int)(frac * (double) spec_w);
        if (x < LM || x >= LM + spec_w) continue;
        for (int t = 0; t < 6; ++t) {
            px_set(rgb, W, H, x, TM + spec_h + t, TIC_R, TIC_G, TIC_B);
        }
        char buf[24];
        double f_abs = (opt->center_hz != 0.0) ? (opt->center_hz + f) : f;
        if (opt->center_hz != 0.0) fmt_freq(f_abs, buf, sizeof buf);
        else                       fmt_freq(f,     buf, sizeof buf);
        int lw = text_width(buf, 1);
        draw_text(rgb, W, H, x - lw / 2, TM + spec_h + 8, buf, 1,
                  LBL_R, LBL_G, LBL_B);
    }

    // Time axis: 0 at the BOTTOM, increasing upward (matches the flipped
    // spectrogram above). Minor ticks every 20 s without labels; major
    // ticks at pick_time_step(duration_s, 10) intervals get labels.
    double t_step = pick_time_step(duration_s, 10);
    for (double t = 0.0; t <= duration_s + 0.5; t += 20.0) {
        // Skip the minor tick that coincides with a major-tick position;
        // the major loop below will draw a longer tick there.
        double mod = fmod(t, t_step);
        if (fabs(mod) < 1.0 || fabs(mod - t_step) < 1.0) continue;
        double frac = 1.0 - t / duration_s;
        int y = TM + (int)(frac * (double) spec_h);
        if (y < TM || y >= TM + spec_h) continue;
        for (int dx = 0; dx < 3; ++dx) {
            px_set(rgb, W, H, LM - 1 - dx, y, TIC_R, TIC_G, TIC_B);
        }
    }
    for (double t = 0.0; t <= duration_s + 0.5 * t_step; t += t_step) {
        double frac = 1.0 - t / duration_s;
        int y = TM + (int)(frac * (double) spec_h);
        if (y < TM || y >= TM + spec_h) continue;
        for (int dx = 0; dx < 6; ++dx) {
            px_set(rgb, W, H, LM - 1 - dx, y, TIC_R, TIC_G, TIC_B);
        }
        char buf[24];
        fmt_time(opt->start_utc, t, buf, sizeof buf);
        int lw = text_width(buf, 1);
        draw_text(rgb, W, H, LM - 8 - lw, y - 3, buf, 1,
                  LBL_R, LBL_G, LBL_B);
    }

    // Axis names along the outer edge.
    draw_text(rgb, W, H, LM, TM + spec_h + 18, "frequency", 1,
              LBL_R, LBL_G, LBL_B);
    draw_text(rgb, W, H, 4, TM + 4, "time", 1,
              LBL_R, LBL_G, LBL_B);

    // dB colorbar in the right margin. Top of the strip is the maximum
    // displayed dB, bottom is the minimum — matches the convention of
    // "more = up". Same viridis table as the spectrogram.
    int cb_x_lo = LM + spec_w + 12;
    int cb_x_hi = cb_x_lo + 16;
    float db_lo = opt->display_db_lo;
    float db_hi = opt->display_db_hi;
    float db_range = db_hi - db_lo;
    if (db_range <= 0.0f) db_range = 1.0f;
    for (int y = 0; y < spec_h; ++y) {
        float t = 1.0f - (float) y / (float)(spec_h > 1 ? spec_h - 1 : 1);
        int idx = (int)(t * 255.0f + 0.5f);
        if (idx < 0)   idx = 0;
        if (idx > 255) idx = 255;
        for (int x = cb_x_lo; x < cb_x_hi; ++x) {
            px_set(rgb, W, H, x, TM + y,
                   VIRIDIS[idx][0], VIRIDIS[idx][1], VIRIDIS[idx][2]);
        }
    }
    // Thin gray frame around the strip so it reads as a discrete plot
    // element rather than blurring into the spectrogram.
    for (int y = -1; y <= spec_h; ++y) {
        px_set(rgb, W, H, cb_x_lo - 1, TM + y, 120, 120, 120);
        px_set(rgb, W, H, cb_x_hi,     TM + y, 120, 120, 120);
    }
    for (int x = cb_x_lo - 1; x <= cb_x_hi; ++x) {
        px_set(rgb, W, H, x, TM - 1,         120, 120, 120);
        px_set(rgb, W, H, x, TM + spec_h,    120, 120, 120);
    }

    // dB ticks + labels on the right side of the strip. Tick POSITIONS
    // are in the median-subtracted ("delta above floor") space so they
    // line up with the colormap mapping; tick LABELS shift by
    // display_db_floor (+ optional --power-offset) so the numbers read
    // as absolute dBFS (or dBm if the user calibrated).
    double db_step = pick_tick_step(db_range, 6);
    double db0 = ceil(db_lo / db_step) * db_step;
    double label_offset = (double) opt->display_db_floor
                        + (double) opt->power_offset_db;
    for (double v = db0; v <= db_hi + 0.5 * db_step; v += db_step) {
        float frac = (float)((v - db_lo) / db_range);
        int y = TM + (int)((1.0f - frac) * (float)(spec_h - 1) + 0.5f);
        if (y < TM || y >= TM + spec_h) continue;
        for (int dx = 0; dx < 5; ++dx) {
            px_set(rgb, W, H, cb_x_hi + 1 + dx, y, TIC_R, TIC_G, TIC_B);
        }
        char buf[16];
        snprintf(buf, sizeof buf, "%.0f", v + label_offset);
        draw_text(rgb, W, H, cb_x_hi + 8, y - 3, buf, 1,
                  LBL_R, LBL_G, LBL_B);
    }
    // Unit label above the colorbar so "0 to 10" doesn't read as
    // dimensionless any more. Uses the small bitmap font; only chars
    // present in glyph_for() render — extended chars fall back to a
    // blank glyph, which is fine for the typical "dBFS"/"dBm" strings.
    {
        int unit_x = cb_x_lo - 2;
        int unit_y = TM - 9;
        draw_text(rgb, W, H, unit_x, unit_y, opt->power_unit, 1,
                  LBL_R, LBL_G, LBL_B);
    }

    *out_rgb = rgb;
    *out_w   = W;
    *out_h   = H;
    return 0;
}

// --------------------------------------------------------------------
// CLI
// --------------------------------------------------------------------

static void usage(void)
{
    fprintf(stderr,
        "usage: gen_waterfall <iq_path> <sample_rate_hz> [<out_png>]\n"
        "                     [--pdf=<out_pdf>]\n"
        "                     [--fft=N] [--rows=N] [--db-min=X]\n"
        "                     [--db-max=X] [--center-hz=F]\n"
        "                     [--zoom-khz=W] [--full-width]\n"
        "                     [--dc-notch] [--dc-notch-bins=N]\n"
        "                     [--power-offset=X] [--power-unit=U]\n"
        "\n"
        "  Reads raw interleaved int16 I,Q samples (no header) from\n"
        "  <iq_path> at <sample_rate_hz>, builds a SatNOGS-style\n"
        "  waterfall with viridis colormap, and writes one or both of:\n"
        "    <out_png>            full-raster PNG (positional arg 3)\n"
        "    --pdf=<out_pdf>      vector-text PDF (sharp labels at any zoom)\n"
        "  Either output is fine on its own; supply both to emit both.\n"
        "\n"
        "  Options:\n"
        "    --fft=N              FFT length per frame, power of two.\n"
        "                         Default 1024 (≈47 Hz / bin at 48 kHz).\n"
        "                         Bigger = finer frequency resolution,\n"
        "                         coarser time resolution.\n"
        "    --rows=N             Output image height in pixels = number\n"
        "                         of time bins. Each row is the linear\n"
        "                         average of (n_frames / rows) FFT frames.\n"
        "                         Default 1080. Fewer rows = longer per-\n"
        "                         row integration, which dilutes burst\n"
        "                         peaks but cleans the noise floor.\n"
        "    --zoom-khz=W         Visible bandwidth around DC, in kHz.\n"
        "                         Default 30 (±15 kHz). --full-width to\n"
        "                         disable.\n"
        "    --dc-notch           Notch out DC and ±dc-notch-bins. Off by\n"
        "                         default — the B210 in this stack has a\n"
        "                         working DC blocker so DC sits BELOW the\n"
        "                         noise floor, and the bright vertical line\n"
        "                         one sometimes sees at DC is the satellite\n"
        "                         carrier passing through 0 Hz baseband at\n"
        "                         TCA. Enable for SDRs that leak DC.\n"
        "    --power-offset=X     Constant (dB) added to every colorbar\n"
        "                         label. Default 0 → labels are absolute\n"
        "                         dBFS per FFT bin. Pass the gain-chain\n"
        "                         offset (e.g. -60 for a B210 at 50 dB LNA\n"
        "                         gain with full-scale ≈ -10 dBm) to read\n"
        "                         dBm at the antenna directly off the bar.\n"
        "    --power-unit=U       Unit string drawn above the colorbar.\n"
        "                         Default 'dBFS'; auto-switches to 'dBm'\n"
        "                         when --power-offset is non-zero unless\n"
        "                         overridden here. Any short ASCII works.\n"
        "    --start-utc=YYYYMMDDTHHMMSS\n"
        "                         Override the capture-start UTC time used\n"
        "                         for the time-axis labels. Default: parse\n"
        "                         the UT= field out of <iq_path>.\n"
        "    --elapsed-time       Force the time axis to show HH:MM:SS\n"
        "                         elapsed-since-start instead of local\n"
        "                         clock time (the default when the UT=\n"
        "                         field is present in the filename).\n"
        "    dB range auto-clipped to the 5th/99th percentile of the\n"
        "    post-median-subtraction values unless --db-min and --db-max\n"
        "    are both set.\n");
}

static int parse_double_opt(const char *arg, const char *prefix, double *out)
{
    size_t plen = strlen(prefix);
    if (strncmp(arg, prefix, plen) != 0) return 0;
    char *endp = NULL;
    double v = strtod(arg + plen, &endp);
    if (endp == arg + plen) return -1;
    *out = v;
    return 1;
}

static int parse_int_opt(const char *arg, const char *prefix, int *out)
{
    size_t plen = strlen(prefix);
    if (strncmp(arg, prefix, plen) != 0) return 0;
    char *endp = NULL;
    long v = strtol(arg + plen, &endp, 10);
    if (endp == arg + plen) return -1;
    *out = (int) v;
    return 1;
}

int main(int argc, char **argv)
{
    if (argc < 3) { usage(); return 2; }
    const char *iq_path  = argv[1];
    int sample_rate      = atoi(argv[2]);
    // PNG path is positional but now optional — if argv[3] starts with
    // "--" it's already an option and the user is asking for PDF-only
    // (which they must supply via --pdf=...). We validate below.
    const char *out_png  = NULL;
    const char *out_pdf  = NULL;  // --pdf=<path>: emit a vector-text PDF too
    int first_opt = 3;
    if (argc >= 4 && strncmp(argv[3], "--", 2) != 0) {
        out_png = argv[3];
        first_opt = 4;
    }

    wf_opts_t opt;
    opt.fft_size      = 1024;
    opt.hop           = 0;            // 0 = N/2 default
    opt.out_rows      = 1080;
    opt.db_min        = 0.0f;
    opt.db_max        = 0.0f;
    opt.auto_clip     = 1;            // default: percentile-based dB clip
    opt.sample_rate   = sample_rate;
    opt.center_hz     = 0.0;
    opt.zoom_hz       = 30000.0;      // default ±15 kHz; --full-width disables
    // DC notch is OFF by default. The B210 in this stack ships with
    // its DC blocker active so the DC bin sits BELOW the noise floor,
    // and the bright vertical line one sees in the middle of a pass
    // is the satellite carrier passing through 0 Hz baseband at TCA
    // (Doppler crossing zero) — real signal, not LO bleed. Pass
    // --dc-notch when capturing from an SDR that leaks DC.
    opt.dc_notch      = 0;
    opt.dc_notch_bins = 4;            // ±4 bins ≈ ±188 Hz at fft=1024 fs=48k
    opt.display_bw_hz   = 0.0;        // build_waterfall fills this in
    opt.display_db_lo   = 0.0f;       // (build_waterfall fills these too)
    opt.display_db_hi   = 0.0f;
    opt.display_db_floor = 0.0f;
    opt.power_offset_db  = 0.0f;
    snprintf(opt.power_unit, sizeof opt.power_unit, "%s", "dBFS");
    // simple_sat_ops names IQ files <prefix>_UT=YYYYMMDDTHHMMSS.fff.iq;
    // parse that out so the y-axis labels show local clock time. CLI
    // --start-utc=YYYYMMDDTHHMMSS overrides.
    opt.start_utc     = parse_ut_from_path(iq_path);

    for (int i = first_opt; i < argc; ++i) {
        int rc = 0;
        double d = 0.0;
        int    v = 0;
        if ((rc = parse_int_opt(argv[i], "--fft=", &v)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.fft_size = v;
        } else if ((rc = parse_int_opt(argv[i], "--rows=", &v)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.out_rows = v;
        } else if ((rc = parse_double_opt(argv[i], "--db-min=", &d)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.db_min = (float) d;
            opt.auto_clip = 0;
        } else if ((rc = parse_double_opt(argv[i], "--db-max=", &d)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.db_max = (float) d;
            opt.auto_clip = 0;
        } else if ((rc = parse_double_opt(argv[i], "--center-hz=", &d)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.center_hz = d;
        } else if ((rc = parse_double_opt(argv[i], "--zoom-khz=", &d)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.zoom_hz = d * 1000.0;
        } else if (strcmp(argv[i], "--full-width") == 0) {
            opt.zoom_hz = 0.0;
        } else if (strcmp(argv[i], "--no-dc-notch") == 0) {
            opt.dc_notch = 0;
        } else if (strcmp(argv[i], "--dc-notch") == 0) {
            opt.dc_notch = 1;
        } else if ((rc = parse_int_opt(argv[i], "--dc-notch-bins=", &v)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.dc_notch_bins = v;
        } else if (strncmp(argv[i], "--start-utc=", 12) == 0) {
            time_t t = parse_ut_string(argv[i] + 12);
            if (t == 0) {
                fprintf(stderr,
                    "gen_waterfall: --start-utc must be YYYYMMDDTHHMMSS\n");
                return 2;
            }
            opt.start_utc = t;
        } else if (strcmp(argv[i], "--elapsed-time") == 0) {
            opt.start_utc = 0;
        } else if (strncmp(argv[i], "--pdf=", 6) == 0) {
            out_pdf = argv[i] + 6;
        } else if ((rc = parse_double_opt(argv[i], "--power-offset=", &d)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.power_offset_db = (float) d;
            // If the user gave us an offset, assume they want dBm
            // labels unless they also pass --power-unit explicitly.
            if (strcmp(opt.power_unit, "dBFS") == 0) {
                snprintf(opt.power_unit, sizeof opt.power_unit, "%s", "dBm");
            }
        } else if (strncmp(argv[i], "--power-unit=", 13) == 0) {
            snprintf(opt.power_unit, sizeof opt.power_unit, "%s", argv[i] + 13);
        } else {
            fprintf(stderr, "gen_waterfall: unknown option '%s'\n", argv[i]);
            usage();
            return 2;
        }
    }
    if (opt.hop == 0) opt.hop = opt.fft_size / 2;

    if (sample_rate <= 0) {
        fprintf(stderr, "gen_waterfall: invalid sample rate %d\n", sample_rate);
        return 2;
    }
    if (out_png == NULL && out_pdf == NULL) {
        fprintf(stderr,
            "gen_waterfall: need either <out_png> or --pdf=<path>\n");
        usage();
        return 2;
    }

    FILE *f = fopen(iq_path, "rb");
    if (!f) {
        fprintf(stderr, "gen_waterfall: open %s: %s\n",
                iq_path, strerror(errno));
        return 1;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "gen_waterfall: seek failed\n");
        fclose(f); return 1;
    }
    long fsize = ftell(f);
    if (fsize < 0) {
        fprintf(stderr, "gen_waterfall: ftell failed\n");
        fclose(f); return 1;
    }
    fseek(f, 0, SEEK_SET);
    size_t n_pairs = (size_t) fsize / 4;  // 2 bytes per int16 × 2 (I,Q)
    if (n_pairs == 0) {
        fprintf(stderr, "gen_waterfall: %s is empty\n", iq_path);
        fclose(f); return 1;
    }
    int16_t *iq = (int16_t *) malloc((size_t) fsize);
    if (!iq) { fprintf(stderr, "gen_waterfall: oom\n"); fclose(f); return 1; }
    if (fread(iq, 1, (size_t) fsize, f) != (size_t) fsize) {
        fprintf(stderr, "gen_waterfall: short read\n");
        free(iq); fclose(f); return 1;
    }
    fclose(f);

    uint8_t *spec_rgb = NULL;
    int spec_w = 0, spec_h = 0;
    int rc = build_waterfall(iq, n_pairs, &opt, &spec_rgb, &spec_w, &spec_h);
    free(iq);
    if (rc != 0) return 1;

    double duration_s = (double) n_pairs / (double) sample_rate;

    // The PNG path runs render_with_axes (composes the full image with
    // bitmap-rendered axes/labels); the PDF path skips that and emits
    // its own vector axes. We always need build_waterfall's spec_rgb
    // until both paths have consumed it.
    if (out_png) {
        uint8_t *rgb = NULL;
        int W = 0, H = 0;
        rc = render_with_axes(spec_rgb, spec_w, spec_h, &opt, duration_s,
                              &rgb, &W, &H);
        if (rc != 0) {
            free(spec_rgb);
            fprintf(stderr, "gen_waterfall: render_with_axes failed\n");
            return 1;
        }
        rc = write_png_rgb(out_png, rgb, W, H);
        free(rgb);
        if (rc != 0) {
            free(spec_rgb);
            fprintf(stderr, "gen_waterfall: write %s: %s\n",
                    out_png, strerror(errno));
            return 1;
        }
        fprintf(stderr,
            "gen_waterfall: %s -> %s (%dx%d, %.1fs, fft=%d)\n",
            iq_path, out_png, W, H, duration_s, opt.fft_size);
    } else {
        // PDF-only run: build_waterfall sets display_db_floor as a
        // side-effect, but the auto-clip step (which populates
        // display_db_lo / display_db_hi) lives inside that same call,
        // so the PDF writer has the dB range it needs without going
        // through render_with_axes.
    }

    if (out_pdf) {
        int pdf_rc = write_pdf_with_axes(out_pdf, spec_rgb, spec_w, spec_h,
                                         &opt, duration_s);
        if (pdf_rc != 0) {
            fprintf(stderr, "gen_waterfall: write %s: %s\n",
                    out_pdf, strerror(errno));
        } else {
            fprintf(stderr, "gen_waterfall: %s -> %s (PDF, %.1fs, fft=%d)\n",
                    iq_path, out_pdf, duration_s, opt.fft_size);
        }
    }
    free(spec_rgb);
    return 0;
}
