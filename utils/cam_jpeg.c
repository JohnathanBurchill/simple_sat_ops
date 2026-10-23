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

// The JPEG header every boom-camera picture opens with: start of image, the
// quantization and Huffman tables, the 640 x 480 frame header and the scan
// header, ending where the image data begins at byte 589 (inside sentence
// 21). All 19 pictures downloaded with their header whole, 2026-06-01 to
// 2026-10-22 in all three camera modes (d, m, n), carry exactly these bytes.
// Copied from camera/2026-10-22_150759Z.img.
#define STANDARD_HEADER_BYTES 589
static const uint8_t standard_header[STANDARD_HEADER_BYTES] = {
    0xff, 0xd8, 0xff, 0xdb, 0x00, 0x84, 0x00, 0x0d, 0x09, 0x09, 0x0b, 0x0a,
    0x08, 0x0d, 0x0b, 0x0a, 0x0b, 0x0e, 0x0e, 0x0d, 0x0f, 0x13, 0x20, 0x15,
    0x13, 0x12, 0x12, 0x13, 0x27, 0x1c, 0x1e, 0x17, 0x20, 0x2e, 0x29, 0x31,
    0x30, 0x2e, 0x29, 0x2d, 0x2c, 0x33, 0x3a, 0x4a, 0x3e, 0x33, 0x36, 0x46,
    0x37, 0x2c, 0x2d, 0x40, 0x57, 0x41, 0x46, 0x4c, 0x4e, 0x52, 0x53, 0x52,
    0x32, 0x3e, 0x5a, 0x61, 0x5a, 0x50, 0x60, 0x4a, 0x51, 0x52, 0x4f, 0x01,
    0x0e, 0x0e, 0x0e, 0x13, 0x11, 0x13, 0x26, 0x15, 0x15, 0x26, 0x4f, 0x35,
    0x2d, 0x35, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f,
    0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f,
    0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f,
    0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f, 0x4f,
    0x4f, 0x4f, 0x4f, 0x4f, 0xff, 0xc4, 0x01, 0xa2, 0x00, 0x00, 0x01, 0x05,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
    0x0b, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05,
    0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11,
    0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71,
    0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52,
    0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18,
    0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67,
    0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
    0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6,
    0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8,
    0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
    0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x09, 0x0a, 0x0b, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04,
    0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01,
    0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07,
    0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1,
    0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16,
    0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46,
    0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a,
    0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76,
    0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9,
    0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6,
    0xf7, 0xf8, 0xf9, 0xfa, 0xff, 0xc0, 0x00, 0x11, 0x08, 0x01, 0xe0, 0x02,
    0x80, 0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xff,
    0xda, 0x00, 0x0c, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00, 0x3f,
    0x00
};

// A received header still counts as the standard one when no more than this
// fraction of its bytes disagree with it. The disagreements come from packets
// that failed their checksum but still filled a gap -- the same mismatch the
// camera viewer allows between two downloads of one picture.
#define STANDARD_HEADER_MAX_MISMATCH 0.35

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

    // A picture whose first sentences were lost has no JPEG header, and no
    // decoder will open it. Every picture so far has had the same header, so
    // when what did arrive of this one agrees with it, the standard header
    // goes in whole: over the missing bytes, and over any a damaged packet
    // got wrong. A header that disagrees more than that is some other one
    // and is left as it came.
    long header_supplied = 0;
    {
        long hdr = image_size < STANDARD_HEADER_BYTES ? image_size : STANDARD_HEADER_BYTES;
        long have = 0, differ = 0;
        for (long b = 0; b < hdr; b++) {
            if (!present[b]) continue;
            have++;
            if (img[b] != standard_header[b]) differ++;
        }
        if (differ <= STANDARD_HEADER_MAX_MISMATCH * (double) have) {
            for (long b = 0; b < hdr; b++) {
                if (present[b] && img[b] == standard_header[b]) continue;
                img[b] = standard_header[b];
                header_supplied++;
            }
        }
    }
    free(present);

    if (stats != NULL) {
        stats->n_sentences = n_sentences;
        stats->sentences_present = sentences_present;
        stats->partial_sentences = partial_sentences;
        stats->image_bytes = image_size;
        stats->recovered_bytes = recovered;
        stats->saw_end_sentinel = saw_end_sentinel;
        stats->header_supplied = header_supplied;
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
