/*

    Simple Satellite Operations  utils/cam_jpeg.c

    Decode the JPEG carried by a boom-camera file, and filter a decoded
    picture for display (histogram equalization, colours stretched to full
    scale). See cam_jpeg.h for the camera-file (START_CAM sentence) format.

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

#include "cam_jpeg.h"

#include <stdlib.h>
#include <string.h>

// Camera sentence geometry, from the firmware's CAM_SENTENCE_LEN = 67:
//   '@'(1) + index(4 hex) + total(4 hex) + data(56 hex = 28 bytes) + "\r\n"(2).
#define SENTENCE_HEADER_CHARS   9      // '@' + 4 index + 4 total
#define SENTENCE_DATA_HEX       56     // hex chars of image data per sentence
#define END_SENTINEL_INDEX      0xFACE // index of the trailing end-marker sentence

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int is_hex(int c)
{
    return hexval(c) >= 0;
}

// Parse `n` hex chars at p into a value. Returns -1 if any char isn't hex.
static long parse_hex(const uint8_t *p, int n)
{
    long v = 0;
    for (int i = 0; i < n; i++) {
        int d = hexval(p[i]);
        if (d < 0) return -1;
        v = v * 16 + d;
    }
    return v;
}

// Most common value in vals[0..n) (the declared total-sentence field, which is
// constant across the file -- voting shrugs off the odd corrupted header).
static long mode_value(const long *vals, int n)
{
    long best = -1;
    int best_count = 0;
    for (int i = 0; i < n; i++) {
        int c = 0;
        for (int j = 0; j < n; j++)
            if (vals[j] == vals[i]) c++;
        if (c > best_count) { best_count = c; best = vals[i]; }
    }
    return best;
}

int cam_jpeg_is_camera_file(const uint8_t *buf, long len)
{
    return buf != NULL && len >= 10 && memcmp(buf, "START_CAM:", 10) == 0;
}

uint8_t *cam_jpeg_decode(const uint8_t *file, long len, uint8_t fill,
                         cam_jpeg_stats_t *stats, long *out_len)
{
    if (stats != NULL) memset(stats, 0, sizeof *stats);
    if (file == NULL || len <= 0) return NULL;

    // Pass 1: vote on the declared total-sentence count and the highest index
    // seen, so we know how big the image is even from a partly damaged file.
    // '@' (0x40) never appears inside a hex payload or as the '?' gap fill, so
    // every '@' in the stream is a genuine sentence start.
    long *totals = malloc(sizeof(long) * (size_t) (len / SENTENCE_HEADER_CHARS + 1));
    if (totals == NULL) return NULL;
    int n_totals = 0;
    long max_index = -1;
    for (long p = 0; p + SENTENCE_HEADER_CHARS <= len; p++) {
        if (file[p] != '@') continue;
        long index = parse_hex(file + p + 1, 4);
        long total = parse_hex(file + p + 5, 4);
        if (index < 0) continue;                 // header eaten by a gap; skip
        if (index == END_SENTINEL_INDEX) continue;
        if (index > max_index) max_index = index;
        if (total >= 0 && total != END_SENTINEL_INDEX)
            totals[n_totals++] = total;
    }

    long declared_total = (n_totals > 0) ? mode_value(totals, n_totals) : -1;
    free(totals);

    long n_sentences = declared_total;
    if (n_sentences <= 0 || max_index >= n_sentences)
        n_sentences = max_index + 1;             // fall back to / extend past the largest index
    if (n_sentences <= 0) return NULL;           // no sentences at all

    long image_size = n_sentences * CAM_JPEG_SENTENCE_DATA_BYTES;
    uint8_t *img = malloc((size_t) image_size);
    uint8_t *present = calloc((size_t) image_size, 1);
    if (img == NULL || present == NULL) { free(img); free(present); return NULL; }
    memset(img, fill, (size_t) image_size);

    // Pass 2: place each surviving sentence's payload. For a sentence partly
    // eaten by a gap we still keep the intact leading hex bytes. A byte is only
    // written if still missing, so a clean retransmit can complete a partial
    // copy and good bytes are never clobbered by a worse one.
    int saw_end_sentinel = 0;
    for (long p = 0; p + SENTENCE_HEADER_CHARS <= len; p++) {
        if (file[p] != '@') continue;
        long index = parse_hex(file + p + 1, 4);
        if (index < 0) continue;
        if (index == END_SENTINEL_INDEX) { saw_end_sentinel = 1; continue; }
        if (index >= n_sentences) continue;      // corrupt/implausible index

        // Consume up to 56 hex chars of payload, stopping at the first non-hex
        // (the trailing "\r", or a '?' where the gap begins).
        const uint8_t *q = file + p + SENTENCE_HEADER_CHARS;
        long avail = len - (p + SENTENCE_HEADER_CHARS);
        int nhex = 0;
        while (nhex < SENTENCE_DATA_HEX && nhex < avail && is_hex(q[nhex]))
            nhex++;
        int nbytes = nhex / 2;                    // a lone trailing nibble is unusable
        long base = index * CAM_JPEG_SENTENCE_DATA_BYTES;
        for (int b = 0; b < nbytes; b++) {
            long off = base + b;
            if (off >= image_size) break;
            if (present[off]) continue;
            int hi = hexval(q[2 * b]);
            int lo = hexval(q[2 * b + 1]);
            img[off] = (uint8_t) ((hi << 4) | lo);
            present[off] = 1;
        }
    }

    // Tally per sentence (not per record), so a retransmitted sentence can't
    // push "present" past the total: a sentence counts as present if any of its
    // bytes arrived, and partial if some -- but not all -- of them did.
    long recovered = 0, sentences_present = 0, partial_sentences = 0;
    for (long s = 0; s < n_sentences; s++) {
        long here = 0;
        for (int b = 0; b < CAM_JPEG_SENTENCE_DATA_BYTES; b++)
            if (present[s * CAM_JPEG_SENTENCE_DATA_BYTES + b]) here++;
        recovered += here;
        if (here > 0) sentences_present++;
        if (here > 0 && here < CAM_JPEG_SENTENCE_DATA_BYTES) partial_sentences++;
    }
    free(present);

    if (stats != NULL) {
        stats->n_sentences = n_sentences;
        stats->sentences_present = sentences_present;
        stats->partial_sentences = partial_sentences;
        stats->image_bytes = image_size;
        stats->recovered_bytes = recovered;
        stats->saw_end_sentinel = saw_end_sentinel;
    }
    if (out_len != NULL) *out_len = image_size;
    return img;
}

// BT.601 luma in integers. The weights 77, 150 and 29 add up to 256, so a grey
// pixel's luma is its own level.
static int luma601(const uint8_t *p)
{
    return (77 * p[0] + 150 * p[1] + 29 * p[2] + 128) >> 8;
}

void cam_jpeg_equalize_rgb8(uint8_t *rgb, long npix)
{
    long hist[256] = {0};
    for (long i = 0; i < npix; i++) hist[luma601(rgb + 3 * i)]++;

    // The textbook mapping: level v goes to 255 (cdf(v) - cdf_min) /
    // (npix - cdf_min), rounded, where cdf(v) counts the pixels at or below v
    // and cdf_min is cdf at the darkest level present.
    long cdf[256], run = 0, cdf_min = -1;
    for (int v = 0; v < 256; v++) {
        run += hist[v];
        cdf[v] = run;
        if (cdf_min < 0 && hist[v] > 0) cdf_min = run;
    }
    long span = npix - cdf_min;
    if (span <= 0) return;
    int map[256];
    for (int v = 0; v < 256; v++) {
        long above = cdf[v] - cdf_min;
        if (above < 0) above = 0;
        map[v] = (int) ((255 * above + span / 2) / span);
    }

    for (long i = 0; i < npix; i++) {
        uint8_t *p = rgb + 3 * i;
        int y = luma601(p);
        int d = map[y] - y;
        for (int c = 0; c < 3; c++) {
            int v = p[c] + d;
            p[c] = (uint8_t) (v < 0 ? 0 : v > 255 ? 255 : v);
        }
    }
}

void cam_jpeg_stretch_rgb8(uint8_t *rgb, long npix)
{
    for (int c = 0; c < 3; c++) {
        int lo = 255, hi = 0;
        for (long i = 0; i < npix; i++) {
            int v = rgb[3 * i + c];
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (hi <= lo) continue;
        // Level v goes to 255 (v - lo) / (hi - lo), rounded.
        int span = hi - lo;
        uint8_t map[256] = {0};
        for (int v = lo; v <= hi; v++) map[v] = (uint8_t) ((255 * (v - lo) + span / 2) / span);
        for (long i = 0; i < npix; i++) rgb[3 * i + c] = map[rgb[3 * i + c]];
    }
}
