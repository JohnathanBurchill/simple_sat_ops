/*

    Simple Satellite Operations  unit_tests/cam_jpeg_selftest.c

    Coverage for utils/cam_jpeg.c -- decoding the JPEG a boom-camera file
    carries as hex "@IIIITTTThhh...\r\n" sentences (28 image bytes each).

    The oracle is a hand-built camera file with known, distinct byte patterns
    per sentence, so every placement, gap, dedup, and order case maps to an
    exact expected output byte -- a misplacement, an off-by-one in the 9-char
    header skip, or a swapped nibble lands on the wrong value and fails. The
    cases exercised, each of which has bitten a real decoder:

      - a clean sentence placed at index*28,
      - a sentence partly eaten by a '?' gap (keep the intact leading bytes),
      - the SAME sentence retransmitted complete (the partial copy is filled
        in, not clobbered; the per-sentence tally doesn't exceed the total),
      - a sentence that never arrived (stays at the fill byte),
      - a sentence that arrives AFTER the 0xFACE end-marker (out of order, and
        the sentinel must not stop the scan or be placed as image data).

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

#include "cam_jpeg.h"
#include "tap.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define DB CAM_JPEG_SENTENCE_DATA_BYTES   // 28

// Append a sentence header "@IIIITTTT" then `nbytes` of value `val` as hex.
// If `gap` is non-zero, the hex stops short with a run of '?' (a lost tail)
// instead of "\r\n"; otherwise it closes with "\r\n".
static char *emit(char *w, int index, int total, uint8_t val, int nbytes, int gap)
{
    static const char H[] = "0123456789ABCDEF";
    w += sprintf(w, "@%04X%04X", index, total);
    for (int i = 0; i < nbytes; i++) { *w++ = H[val >> 4]; *w++ = H[val & 0xF]; }
    if (gap) { for (int i = 0; i < 6; i++) *w++ = '?'; }
    else     { *w++ = '\r'; *w++ = '\n'; }
    *w = '\0';
    return w;
}

int main(void)
{
    char buf[4096];
    char *w = buf;
    w += sprintf(w, "START_CAM:\n");
    w = emit(w, 0, 4, 0xA0, DB, 0);          // sentence 0: clean, full 28 bytes
    w = emit(w, 1, 4, 0xA1, 10, 1);          // sentence 1: partial -- only 10 bytes, then a gap
    w = emit(w, 1, 4, 0xB1, DB, 0);          // sentence 1 retransmit: complete (fills bytes 10..27)
    // sentence 2 never arrives -- represent the hole as a run of '?'
    for (int i = 0; i < 10; i++) *w++ = '?';
    w = emit(w, 0xFACE, 4, 0x00, DB, 0);     // end-marker sentence (no image data)
    w = emit(w, 3, 4, 0xA3, DB, 0);          // sentence 3: AFTER the sentinel (out of order)
    *w = '\0';
    long flen = (long) (w - buf);

    // --- is-camera-file detector -------------------------------------
    tap_ok(cam_jpeg_is_camera_file((const uint8_t *) buf, flen),
           "START_CAM file is recognised as a camera file");
    tap_ok(!cam_jpeg_is_camera_file((const uint8_t *) "GARBAGE___", 10),
           "non-START_CAM bytes are not a camera file");
    tap_ok(!cam_jpeg_is_camera_file((const uint8_t *) "START", 5),
           "too-short buffer is not a camera file");

    // --- decode ------------------------------------------------------
    cam_jpeg_stats_t st;
    long jlen = 0;
    uint8_t *img = cam_jpeg_decode((const uint8_t *) buf, flen, 0x00, &st, &jlen);
    tap_ok(img != NULL, "decode returns an image");
    if (img == NULL) { tap_diag("decode failed; cannot continue"); return tap_done(); }

    // Size comes from the declared total (TTTT=4), 28 bytes each.
    tap_okf(jlen == 4 * DB, "image length == total*28 == %d (got %ld)", 4 * DB, jlen);
    tap_okf(st.n_sentences == 4, "n_sentences == 4 (got %ld)", st.n_sentences);

    // Byte placement: each present sentence's block holds its own pattern, the
    // missing one holds the fill, and the retransmit completed sentence 1's tail.
    int s0 = 1, s1 = 1, s2 = 1, s3 = 1;
    for (int b = 0; b < DB; b++) {
        if (img[0 * DB + b] != 0xA0) s0 = 0;
        // sentence 1: first 10 bytes from the partial (0xA1), the rest from the
        // retransmit (0xB1) -- fill-if-absent must NOT overwrite the first 10.
        uint8_t want1 = (b < 10) ? 0xA1 : 0xB1;
        if (img[1 * DB + b] != want1) s1 = 0;
        if (img[2 * DB + b] != 0x00) s2 = 0;   // never arrived -> fill
        if (img[3 * DB + b] != 0xA3) s3 = 0;
    }
    tap_ok(s0, "sentence 0 placed at offset 0");
    tap_ok(s1, "sentence 1: partial 10 bytes kept, tail completed by retransmit");
    tap_ok(s2, "sentence 2 (lost) left at the 0x00 fill");
    tap_ok(s3, "sentence 3 placed despite arriving after the end-marker");

    // Stats: 3 of 4 sentences present, exactly one of them partial; recovered
    // byte count = 28+28+0+28; the sentinel was seen but never placed.
    tap_okf(st.sentences_present == 3, "sentences_present == 3 (got %ld)", st.sentences_present);
    tap_okf(st.partial_sentences == 0,
            "partial_sentences == 0 after retransmit completes it (got %ld)", st.partial_sentences);
    tap_okf(st.recovered_bytes == 3 * DB,
            "recovered_bytes == %d (got %ld)", 3 * DB, st.recovered_bytes);
    tap_ok(st.saw_end_sentinel, "0xFACE end-marker sentence was seen");
    free(img);

    // --- a still-partial sentence is reported as partial -------------
    // Same file minus the retransmit: sentence 1 stays 10/28 -> one partial.
    {
        char b2[4096]; char *p = b2;
        p += sprintf(p, "START_CAM:\n");
        p = emit(p, 0, 4, 0xA0, DB, 0);
        p = emit(p, 1, 4, 0xA1, 10, 1);        // partial, never completed
        p = emit(p, 3, 4, 0xA3, DB, 0);
        *p = '\0';
        cam_jpeg_stats_t s2st; long l2 = 0;
        uint8_t *i2 = cam_jpeg_decode((const uint8_t *) b2, (long)(p - b2), 0x00, &s2st, &l2);
        tap_ok(i2 != NULL, "second file decodes");
        tap_okf(s2st.sentences_present == 3 && s2st.partial_sentences == 1,
                "uncompleted 10/28 sentence counts as 1 partial of 3 present "
                "(got %ld present, %ld partial)", s2st.sentences_present, s2st.partial_sentences);
        tap_okf(s2st.recovered_bytes == 2 * DB + 10,
                "recovered_bytes == %d (got %ld)", 2 * DB + 10, s2st.recovered_bytes);
        free(i2);
    }

    // --- fill byte is honoured ---------------------------------------
    {
        uint8_t *iff = cam_jpeg_decode((const uint8_t *) buf, flen, 0xFF, NULL, NULL);
        tap_ok(iff != NULL && iff[2 * DB] == 0xFF,
               "lost sentence honours a non-zero fill (0xFF)");
        free(iff);
    }

    // --- empty / non-camera input doesn't crash or fabricate ---------
    tap_ok(cam_jpeg_decode((const uint8_t *) "", 0, 0, NULL, NULL) == NULL,
           "empty input returns NULL");
    tap_ok(cam_jpeg_decode((const uint8_t *) "no sentences here", 17, 0, NULL, NULL) == NULL,
           "input with no '@' sentences returns NULL");

    return tap_done();
}
