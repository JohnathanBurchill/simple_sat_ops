/*

    Simple Satellite Operations  utils/cam_jpeg.h

    Decode the JPEG carried by a boom-camera file.

    The flight firmware (CTS-SAT-1, tag sat-1-rc3) writes a captured JPEG to a
    file that begins with the text header "START_CAM:\n" and then a run of fixed
    67-byte "sentences" the camera streams over its serial port:

        @IIIITTTThhhh...hh\r\n

    where IIII is the sentence's hex index, TTTT the total data-sentence count,
    and hhhh...hh is 56 hex digits = 28 bytes of JPEG data. Sentence N carries
    image bytes [N*28, N*28+28). Index 0xFACE marks the end and carries no data.

    Shared by utils/cam_reconstruct.c (the standalone tool) and
    utils/packet_browser.c (the "save camera JPEG" option in the bulk-file
    reconstruction view).

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

#ifndef CAM_JPEG_H
#define CAM_JPEG_H

#include <stdint.h>

// Image data carried per camera sentence (56 hex chars = 28 bytes).
#define CAM_JPEG_SENTENCE_DATA_BYTES   28

typedef struct {
    long n_sentences;        // data sentences the image spans (declared total)
    long sentences_present;  // sentences with at least one byte recovered
    long partial_sentences;  // present sentences still missing some bytes
    long image_bytes;        // n_sentences * 28 (size of the returned buffer)
    long recovered_bytes;    // image bytes actually filled from sentences
    int  saw_end_sentinel;   // the 0xFACE end-marker sentence was seen
} cam_jpeg_stats_t;

// True if `buf` looks like a boom-camera file (starts with "START_CAM:").
int cam_jpeg_is_camera_file(const uint8_t *buf, long len);

// Decode the JPEG carried by a START_CAM camera file. `file`/`len` is the raw
// downloaded file; bytes lost in downlink can appear as a gap char ('?') or any
// other non-hex byte, and are simply skipped (a sentence partly eaten by a gap
// still contributes its intact leading bytes). Still-missing image bytes are
// set to `fill` so the JPEG stays byte-aligned for a tolerant decoder.
//
// Returns a malloc'd buffer of the reconstructed image (caller frees), with its
// length in *out_len, and fills *stats when non-NULL. Returns NULL on error (no
// sentences found, or out of memory).
uint8_t *cam_jpeg_decode(const uint8_t *file, long len, uint8_t fill,
                         cam_jpeg_stats_t *stats, long *out_len);

#endif
