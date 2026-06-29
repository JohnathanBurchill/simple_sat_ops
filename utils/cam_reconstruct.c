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

    When those re-fetched chunks come back they land in the packet DB as new
    bulk_file packets, one per missing offset. Rather than reassemble the file
    by hand in packet_browser, pass the new packets' DB ids with --patch and the
    store with --db: each bulk_file payload is [type:1][file_offset:4 LE][data..],
    and this tool lays that data over the matching '?' run in the raw file before
    decoding -- closing the gaps the first download missed.

    Usage:
      cam_reconstruct <camera-file> [--jpg=<out.jpg>] [--fill=00|ff]
                      [--chunk=<bytes>] [--file-path=<sat-path>] [--quiet]
                      [--db=<packet_db.sqlite>] [--patch=<id,id,...>]

    With no --jpg, the output is the input path with a trailing ".bin" removed
    (if present) and ".jpg" appended. --file-path sets the on-satellite path
    printed in the re-download commands (default a "<file_path>" placeholder,
    since the downloaded file is usually named by content hash on the ground).
    --patch takes one or more bulk_file packet ids (the flag may repeat, and
    each value may be a comma-separated list) and needs --db to point at the
    packet store the ids live in.

    Standalone except for the optional --patch path, which reads the packet DB
    and needs sqlite3 (compiled in when sqlite3 is found, as for packet_browser).
    Everything else depends only on the C library and builds on the Mac host.

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

#ifdef WITH_SQLITE3
#include <sqlite3.h>
#endif

#include "cam_jpeg.h"

// Bulk-downlink chunk size: the firmware sends this many file bytes per RF
// packet (BULK_FILE_DOWNLINK_PACKET_MAX_DATA_BYTES_PER_PACKET), so file
// offsets that were lost are whole multiples of it. Re-download requests are
// snapped to this so we re-fetch the same chunks the firmware would resend.
#define DEFAULT_CHUNK_BYTES     195

#define GAP_CHAR                '?'    // simple_sat_ops fills lost bytes with this

// A bulk_file packet payload is [packet_type:1][file_offset:4 LE][data...].
// Same geometry packet_browser uses to place download chunks by file offset.
#define BULK_FILE_PACKET_TYPE   16
#define BULK_FILE_HEADER_SIZE   5

// Most --patch ids a single image can need: a 25 KB capture is ~129 chunks, so
// this is comfortably above any real run and bounds the fixed id array.
#define MAX_PATCH_IDS           1024

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

#ifdef WITH_SQLITE3

// Little-endian uint32 file_offset out of a bulk_file payload.
static long bulk_file_offset(const uint8_t *pl)
{
    return (long) pl[1] | ((long) pl[2] << 8)
         | ((long) pl[3] << 16) | ((long) pl[4] << 24);
}

// Lay the re-downloaded bulk_file packets named by `ids` over the raw camera
// file in `buf`, overwriting the '?' gap bytes the first download missed. Each
// packet's data goes at its own file_offset -- exactly where simple_sat_ops
// left a gap. A missing id, a non-bulk_file id, or an offset outside the file
// is warned and skipped, not fatal. Returns the number of gap bytes filled, or
// -1 if the DB itself could not be opened.
static long apply_patches(const char *db_path, uint8_t *buf, long len,
                          const long *ids, int n_ids, int quiet)
{
    if (db_path == NULL) {
        fprintf(stderr, "--patch needs --db=<packet_db.sqlite> to read the packets from.\n");
        return -1;
    }
    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "cannot open %s: %s\n", db_path,
                db ? sqlite3_errmsg(db) : "out of memory");
        sqlite3_close(db);
        return -1;
    }
    sqlite3_stmt *stmt = NULL;
    if (sqlite3_prepare_v2(db,
            "SELECT packet_type, payload FROM packet WHERE id = ?1",
            -1, &stmt, NULL) != SQLITE_OK) {
        fprintf(stderr, "query failed on %s: %s\n", db_path, sqlite3_errmsg(db));
        sqlite3_close(db);
        return -1;
    }

    if (!quiet) printf("patching from %s:\n", db_path);
    long total_filled = 0;
    for (int i = 0; i < n_ids; i++) {
        sqlite3_reset(stmt);
        sqlite3_bind_int64(stmt, 1, (sqlite3_int64) ids[i]);
        if (sqlite3_step(stmt) != SQLITE_ROW) {
            fprintf(stderr, "  id %ld: not found -- skipped\n", ids[i]);
            continue;
        }
        int ptype = sqlite3_column_int(stmt, 0);
        if (ptype != BULK_FILE_PACKET_TYPE) {
            fprintf(stderr, "  id %ld: packet_type %d is not bulk_file (%d) -- skipped\n",
                    ids[i], ptype, BULK_FILE_PACKET_TYPE);
            continue;
        }
        const uint8_t *pl = (const uint8_t *) sqlite3_column_blob(stmt, 1);
        int pl_len = sqlite3_column_bytes(stmt, 1);
        if (pl == NULL || pl_len <= BULK_FILE_HEADER_SIZE) {
            fprintf(stderr, "  id %ld: payload too short (%d bytes) -- skipped\n",
                    ids[i], pl_len);
            continue;
        }
        long off = bulk_file_offset(pl);
        long dl = pl_len - BULK_FILE_HEADER_SIZE;
        const uint8_t *src = pl + BULK_FILE_HEADER_SIZE;
        if (off < 0 || off >= len) {
            fprintf(stderr, "  id %ld: file offset %ld is outside the file (0..%ld) -- skipped\n",
                    ids[i], off, len);
            continue;
        }
        int clipped = 0;
        if (off + dl > len) { dl = len - off; clipped = 1; }
        long filled = 0, changed = 0;
        for (long k = 0; k < dl; k++) {
            if (buf[off + k] == GAP_CHAR) filled++;
            else if (buf[off + k] != src[k]) changed++;
            buf[off + k] = src[k];
        }
        total_filled += filled;
        if (!quiet) {
            printf("  id %ld -> offset %ld, %ld bytes (%ld gap byte%s filled",
                   ids[i], off, dl, filled, filled == 1 ? "" : "s");
            if (changed) printf(", %ld changed", changed);
            if (clipped) printf(", clipped to file end");
            printf(")\n");
        }
    }
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    if (!quiet) printf("\n");
    return total_filled;
}

#else  // !WITH_SQLITE3

// Patch ids name DB rows, so the feature needs sqlite3; main only calls this
// when ids were given, so any call here means the build can't honour them.
static long apply_patches(const char *db_path, uint8_t *buf, long len,
                          const long *ids, int n_ids, int quiet)
{
    (void) db_path; (void) buf; (void) len; (void) ids; (void) n_ids; (void) quiet;
    fprintf(stderr,
        "--patch needs sqlite3 support, which this build lacks.\n"
        "Rebuild with sqlite3 installed (brew install sqlite), or reassemble the\n"
        "missing packets by hand in packet_browser.\n");
    return -1;
}

#endif // WITH_SQLITE3

int main(int argc, char **argv)
{
    const char *in_path = NULL;
    const char *out_path = NULL;
    const char *sat_file_path = "<file_path>";
    const char *db_path = NULL;
    uint8_t fill = 0x00;
    long chunk = DEFAULT_CHUNK_BYTES;
    int quiet = 0;
    long patch_ids[MAX_PATCH_IDS];
    int  n_patch_ids = 0;

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
        } else if (strncmp(a, "--db=", 5) == 0) {
            db_path = a + 5;
        } else if (strncmp(a, "--patch=", 8) == 0) {
            // One or more packet ids, comma- or space-separated; the flag may
            // also repeat. strtol stops at each separator and reports where.
            const char *s = a + 8;
            while (*s) {
                char *end = NULL;
                long v = strtol(s, &end, 0);
                if (end == s) {
                    fprintf(stderr, "--patch expects packet ids, e.g. --patch=30088,30090\n");
                    return 2;
                }
                if (n_patch_ids >= MAX_PATCH_IDS) {
                    fprintf(stderr, "--patch: too many ids (max %d)\n", MAX_PATCH_IDS);
                    return 2;
                }
                patch_ids[n_patch_ids++] = v;
                s = end;
                while (*s == ',' || *s == ' ') s++;
            }
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
            "                       [--chunk=<bytes>] [--file-path=<sat-path>] [--quiet]\n"
            "                       [--db=<packet_db.sqlite>] [--patch=<id,id,...>]\n");
        return 2;
    }

    long len = 0;
    uint8_t *buf = read_file(in_path, &len);
    if (buf == NULL) return 1;

    // Lay any re-downloaded packets over the raw file before decoding, so the
    // filled bytes flow into both the JPEG and the still-missing report below.
    if (n_patch_ids > 0
        && apply_patches(db_path, buf, len, patch_ids, n_patch_ids, quiet) < 0) {
        free(buf);
        return 1;
    }

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
