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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
// Spectrogram pipeline
// --------------------------------------------------------------------

typedef struct {
    int    fft_size;
    int    hop;
    int    out_rows;
    float  db_min;
    float  db_max;
    double center_hz;   // for informational labels; not baked into pixels
} wf_opts_t;

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

    // Allocate the high-resolution spectrogram: n_frames × N dB values.
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
        // dB magnitude², with fftshift: bin 0 of the OUTPUT is the most
        // negative frequency (-Fs/2), bin N/2 is DC, bin N-1 is +Fs/2 - bin.
        for (int k = 0; k < N; ++k) {
            int src = (k + N / 2) % N;
            float mag2 = re[src] * re[src] + im[src] * im[src];
            spec[fi * (size_t) N + (size_t) k] =
                10.0f * log10f(mag2 + 1e-20f);
        }
    }
    free(win); free(re); free(im);

    // Bin input frames into output rows (averaging in dB-space — close
    // enough for visualisation, much cheaper than averaging linearly).
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
        for (int k = 0; k < N; ++k) {
            double sum = 0.0;
            for (size_t fi = a; fi < b; ++fi) {
                sum += spec[fi * (size_t) N + (size_t) k];
            }
            binned[(size_t) r * (size_t) N + (size_t) k] = (float)(sum / (double) span);
        }
    }
    free(spec);

    // Per-bin median subtraction so the noise floor flattens out.
    float *col = (float *) malloc((size_t) out_rows * sizeof(float));
    if (!col) { free(binned); return -1; }
    for (int k = 0; k < N; ++k) {
        for (int r = 0; r < out_rows; ++r) {
            col[r] = binned[(size_t) r * (size_t) N + (size_t) k];
        }
        float med = median_inplace(col, out_rows);
        for (int r = 0; r < out_rows; ++r) {
            binned[(size_t) r * (size_t) N + (size_t) k] -= med;
        }
    }
    free(col);

    // Map to viridis. Width = N (already fftshift'd). Height = out_rows.
    // The PNG's row 0 is the TOP of the image, so iterate as-is — the
    // first output row corresponds to the earliest IQ samples, which
    // matches the SatNOGS convention of "earliest at top".
    uint8_t *rgb = (uint8_t *) malloc((size_t) N * (size_t) out_rows * 3);
    if (!rgb) { free(binned); return -1; }
    float lo = opt->db_min, hi = opt->db_max;
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
// CLI
// --------------------------------------------------------------------

static void usage(void)
{
    fprintf(stderr,
        "usage: gen_waterfall <iq_path> <sample_rate_hz> <out_png>\n"
        "                     [--fft=N] [--rows=N] [--db-min=X]\n"
        "                     [--db-max=X] [--center-hz=F]\n"
        "\n"
        "  Reads raw interleaved int16 I,Q samples (no header) from\n"
        "  <iq_path> at <sample_rate_hz>, builds a SatNOGS-style\n"
        "  waterfall PNG with viridis colormap, and writes it to\n"
        "  <out_png>. Defaults: --fft=1024, --rows=1080, --db-min=-3,\n"
        "  --db-max=20 (post-median-subtraction).\n");
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
    if (argc < 4) { usage(); return 2; }
    const char *iq_path  = argv[1];
    int sample_rate      = atoi(argv[2]);
    const char *out_png  = argv[3];

    wf_opts_t opt;
    opt.fft_size  = 1024;
    opt.hop       = 0;     // 0 = N/2 default
    opt.out_rows  = 1080;
    opt.db_min    = -3.0f;
    opt.db_max    = 20.0f;
    opt.center_hz = 0.0;

    for (int i = 4; i < argc; ++i) {
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
        } else if ((rc = parse_double_opt(argv[i], "--db-max=", &d)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.db_max = (float) d;
        } else if ((rc = parse_double_opt(argv[i], "--center-hz=", &d)) != 0) {
            if (rc < 0) { usage(); return 2; }
            opt.center_hz = d;
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

    uint8_t *rgb = NULL;
    int W = 0, H = 0;
    int rc = build_waterfall(iq, n_pairs, &opt, &rgb, &W, &H);
    free(iq);
    if (rc != 0) return 1;

    rc = write_png_rgb(out_png, rgb, W, H);
    free(rgb);
    if (rc != 0) {
        fprintf(stderr, "gen_waterfall: write %s: %s\n",
                out_png, strerror(errno));
        return 1;
    }
    double duration_s = (double) n_pairs / (double) sample_rate;
    fprintf(stderr,
        "gen_waterfall: %s -> %s (%dx%d, %.1fs, fft=%d)\n",
        iq_path, out_png, W, H, duration_s, opt.fft_size);
    (void) opt.center_hz;  // reserved for future axis labels
    return 0;
}
