/*

    Simple Satellite Operations  utils/cam_reconstruct.c

    Rebuild a boom-camera JPEG from a downloaded camera file, recovering as
    much as possible when some packets were lost.

    The flight firmware (CTS-SAT-1, tag sat-1-rc3) captures a JPEG from the
    boom camera and writes it to a file on the satellite's flash. The file is
    NOT raw JPEG bytes: it starts with the text header "START_CAM:\n", then a
    run of fixed 67-byte "sentences" the camera streams over its serial line.
    Each sentence looks like

        @IIIITTTThhhh...hh\r\n

    where
        @           one literal '@' (0x40) marking a sentence start,
        IIII        four hex digits: this sentence's index (0, 1, 2, ...),
        TTTT        four hex digits: the total number of data sentences in
                    the image (constant across the file, e.g. 0174 = 372),
        hhhh...hh   56 hex digits = 28 bytes of JPEG data,
        \r\n        carriage return + newline.

    So sentence N carries JPEG bytes [N*28 .. N*28+28). A final sentinel
    sentence with index 0xFACE marks the end and carries no image data. A
    complete 372-sentence image is 372*28 = 10416 JPEG bytes; the camera pads
    the last sentence past the real end-of-image, which a JPEG decoder ignores.

    This file is downloaded with comms_bulk_file_downlink, which streams the
    flash file in fixed-size chunks (195 data bytes per RF packet) tagged with
    a byte offset. simple_sat_ops reassembles the chunks by offset and, where a
    packet never arrived, fills the missing bytes with '?' (0x3F). Those '?'
    runs are the lost data. Because the 195-byte download chunks do not line up
    with the 67-byte camera sentences, one lost chunk corrupts about three
    sentences, and the '?' runs also merge neighbouring sentences into long
    lines -- which is why the raw file can't simply be fed to a JPEG decoder.

    What this tool does:
      1. Scans for every surviving sentence (even ones partly eaten by a '?'
         run -- it keeps whatever leading hex bytes are intact), hex-decodes
         the payload, and places it at index*28 in the output image.
      2. Writes a best-effort .jpg with any still-missing bytes left as a fill
         value (0x00 by default) so the image stays byte-aligned and a tolerant
         decoder can render everything up to the first gap.
      3. Reports which file byte ranges are still missing (the '?' runs),
         snapped out to the 195-byte download-chunk boundaries, and prints
         ready-to-use comms_bulk_file_downlink_start commands to re-fetch them.

    Usage:
      cam_reconstruct <camera-file> [--jpg=<out.jpg>] [--fill=00|ff]
                      [--chunk=<bytes>] [--file-path=<sat-path>] [--quiet]

    With no --jpg, the output is the input path with a trailing ".bin" removed
    (if present) and ".jpg" appended. --file-path sets the on-satellite path
    printed in the re-download commands (default a "<file_path>" placeholder,
    since the downloaded file is usually named by content hash on the ground).

    Standalone: depends only on the C library. Buildable on the Mac dev host.

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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "cam_jpeg.h"

// Bulk-downlink chunk size: the firmware sends this many file bytes per RF
// packet (BULK_FILE_DOWNLINK_PACKET_MAX_DATA_BYTES_PER_PACKET), so file
// offsets that were lost are whole multiples of it. Re-download requests are
// snapped to this so we re-fetch the same chunks the firmware would resend.
#define DEFAULT_CHUNK_BYTES     195

#define GAP_CHAR                '?'    // simple_sat_ops fills lost bytes with this

// Parse two hex chars at p into a byte value, or -1 if either isn't hex.
static long parse_hex2(const char *p)
{
    int hi = -1, lo = -1;
    char c;
    c = p[0];
    if (c >= '0' && c <= '9') hi = c - '0';
    else if (c >= 'a' && c <= 'f') hi = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') hi = c - 'A' + 10;
    c = p[1];
    if (c >= '0' && c <= '9') lo = c - '0';
    else if (c >= 'a' && c <= 'f') lo = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') lo = c - 'A' + 10;
    if (hi < 0 || lo < 0) return -1;
    return (hi << 4) | lo;
}

static uint8_t *read_file(const char *path, long *out_len)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) { perror(path); return NULL; }
    if (fseek(f, 0, SEEK_END) != 0) { perror("fseek"); fclose(f); return NULL; }
    long len = ftell(f);
    if (len < 0) { perror("ftell"); fclose(f); return NULL; }
    rewind(f);
    uint8_t *buf = malloc((size_t) len + 1);
    if (buf == NULL) { fprintf(stderr, "out of memory\n"); fclose(f); return NULL; }
    if (len > 0 && fread(buf, 1, (size_t) len, f) != (size_t) len) {
        fprintf(stderr, "short read on %s\n", path);
        free(buf); fclose(f); return NULL;
    }
    buf[len] = 0;
    fclose(f);
    *out_len = len;
    return buf;
}

int main(int argc, char **argv)
{
    const char *in_path = NULL;
    const char *out_path = NULL;
    const char *sat_file_path = "<file_path>";
    uint8_t fill = 0x00;
    long chunk = DEFAULT_CHUNK_BYTES;
    int quiet = 0;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strncmp(a, "--jpg=", 6) == 0) {
            out_path = a + 6;
        } else if (strncmp(a, "--fill=", 7) == 0) {
            long v = (strlen(a + 7) == 2) ? parse_hex2(a + 7) : -1;
            if (v < 0) { fprintf(stderr, "--fill must be two hex digits (e.g. 00 or ff)\n"); return 2; }
            fill = (uint8_t) v;
        } else if (strncmp(a, "--chunk=", 8) == 0) {
            chunk = strtol(a + 8, NULL, 0);
            if (chunk <= 0) { fprintf(stderr, "--chunk must be positive\n"); return 2; }
        } else if (strncmp(a, "--file-path=", 12) == 0) {
            sat_file_path = a + 12;
        } else if (strcmp(a, "--quiet") == 0) {
            quiet = 1;
        } else if (a[0] == '-') {
            fprintf(stderr, "unknown option: %s\n", a);
            return 2;
        } else if (in_path == NULL) {
            in_path = a;
        } else {
            fprintf(stderr, "unexpected argument: %s\n", a);
            return 2;
        }
    }

    if (in_path == NULL) {
        fprintf(stderr,
            "usage: cam_reconstruct <camera-file> [--jpg=<out.jpg>] [--fill=00|ff]\n"
            "                       [--chunk=<bytes>] [--file-path=<sat-path>] [--quiet]\n");
        return 2;
    }

    long len = 0;
    uint8_t *buf = read_file(in_path, &len);
    if (buf == NULL) return 1;

    if (!cam_jpeg_is_camera_file(buf, len)) {
        fprintf(stderr,
            "warning: %s does not start with \"START_CAM:\" -- not a camera file?\n"
            "         continuing anyway (scanning for '@' sentences).\n", in_path);
    }

    // Decode the camera sentences into the JPEG (shared with packet_browser).
    cam_jpeg_stats_t st = {0};
    long image_size = 0;
    uint8_t *img = cam_jpeg_decode(buf, len, fill, &st, &image_size);
    if (img == NULL) {
        fprintf(stderr, "no camera sentences found in %s\n", in_path);
        free(buf);
        return 1;
    }
    long present_bytes = st.recovered_bytes;
    long missing_bytes = image_size - present_bytes;

    // Write the reconstructed JPEG.
    char auto_out[4096];
    if (out_path == NULL) {
        size_t n = strlen(in_path);
        if (n > 4 && strcmp(in_path + n - 4, ".bin") == 0) n -= 4;  // strip ".bin"
        if (n > sizeof auto_out - 5) n = sizeof auto_out - 5;
        memcpy(auto_out, in_path, n);
        memcpy(auto_out + n, ".jpg", 5);
        out_path = auto_out;
    }
    FILE *of = fopen(out_path, "wb");
    if (of == NULL) { perror(out_path); free(buf); free(img); return 1; }
    if (fwrite(img, 1, (size_t) image_size, of) != (size_t) image_size) {
        perror("fwrite"); fclose(of); free(buf); free(img); return 1;
    }
    fclose(of);

    // Re-download list: runs of the '?' gap fill in the RAW file. Those byte
    // offsets are file offsets into the satellite file (the download starts at
    // file offset 0). Each run is snapped out to whole download chunks.
    if (!quiet) {
        printf("camera file : %s (%ld bytes)\n", in_path, len);
        printf("output jpeg : %s (%ld bytes)\n", out_path, image_size);
        printf("sentences   : %ld of %ld present", st.sentences_present, st.n_sentences);
        if (st.partial_sentences) printf(" (%ld partial)", st.partial_sentences);
        printf("%s\n", st.saw_end_sentinel ? ", end-marker seen" : "");
        printf("image bytes : %ld recovered, %ld missing (%.1f%% recovered)\n",
               present_bytes, missing_bytes,
               image_size ? 100.0 * (double) present_bytes / (double) image_size : 0.0);
        printf("\n");
    }

    long n_runs = 0, missing_file_bytes = 0, redl_bytes = 0;
    long run_start = -1;
    // Iterate one past the end so a gap that runs to EOF is closed.
    if (!quiet) printf("missing file ranges (re-download these):\n");
    for (long i = 0; i <= len; i++) {
        int gap = (i < len) && (buf[i] == GAP_CHAR);
        if (gap && run_start < 0) {
            run_start = i;
        } else if (!gap && run_start >= 0) {
            long a = run_start, b = i;            // raw gap [a, b)
            missing_file_bytes += b - a;
            long sa = (a / chunk) * chunk;        // snap out to chunk boundaries
            long sb = ((b + chunk - 1) / chunk) * chunk;
            if (sb > len) sb = len;
            redl_bytes += sb - sa;
            n_runs++;
            if (!quiet)
                printf("  offset %6ld  len %5ld   "
                       "comms_bulk_file_downlink_start(%s,%ld,%ld)\n",
                       sa, sb - sa, sat_file_path, sa, sb - sa);
            run_start = -1;
        }
    }

    if (!quiet) {
        if (n_runs == 0)
            printf("  (none -- file is complete)\n");
        printf("\n");
        printf("gap fill    : 0x%02X (%ld raw '?' bytes across %ld runs)\n",
               fill, missing_file_bytes, n_runs);
        printf("re-download : %ld bytes (%ld-byte chunks)\n", redl_bytes, chunk);
    }

    free(buf);
    free(img);
    return 0;   // gaps are reported, not a failure
}
