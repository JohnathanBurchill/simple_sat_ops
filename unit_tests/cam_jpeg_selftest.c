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

    The standard JPEG header the decoder supplies when a picture's own is lost
    or damaged is checked against a real picture's header (the first 22
    sentences of the 2026-06-01 picture, below): restored byte for byte when
    sentences 0-20 are lost, a wrong byte corrected, never counted as received,
    and a header mostly unlike it left as it came.

    It also covers cam_jpeg_equalize_rgb8, the viewer's histogram
    equalization, against values worked by hand from the textbook mapping:
    the darkest-level offset, rounding, brightness-only shifting that keeps a
    pixel's colour, clamping at 255, and a picture of one brightness. And
    cam_jpeg_stretch_rgb8, colours stretched to full scale: each colour on
    its own range, rounding, a colour already at full range, and a colour
    holding one value.

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

// The first 22 sentences of camera/2026-06-01_capture.img, the boom camera's
// first picture, as downloaded: the JPEG header fills sentences
// 0 to 20 and the first byte of sentence 21, where the image data begins. A
// different picture from the one cam_jpeg.c copied its standard header from,
// so the header tests below also check that the two pictures' headers agree.
static const char real_head[] =
    "START_CAM:\n"
    "@00000174FFD8FFDB0084000D09090B0A080D0B0A0B0E0E0D0F13201513121213\r\n"
    "@00010174271C1E17202E2931302E292D2C333A4A3E333646372C2D405741464C\r\n"
    "@000201744E525352323E5A615A50604A51524F010E0E0E131113261515264F35\r\n"
    "@000301742D354F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F\r\n"
    "@000401744F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4F4FFFC401A2\r\n"
    "@000501740000010501010101010100000000000000000102030405060708090A\r\n"
    "@000601740B100002010303020403050504040000017D01020300041105122131\r\n"
    "@00070174410613516107227114328191A1082342B1C11552D1F0243362728209\r\n"
    "@000801740A161718191A25262728292A3435363738393A434445464748494A53\r\n"
    "@000901745455565758595A636465666768696A737475767778797A8384858687\r\n"
    "@000A017488898A92939495969798999AA2A3A4A5A6A7A8A9AAB2B3B4B5B6B7B8\r\n"
    "@000B0174B9BAC2C3C4C5C6C7C8C9CAD2D3D4D5D6D7D8D9DAE1E2E3E4E5E6E7E8\r\n"
    "@000C0174E9EAF1F2F3F4F5F6F7F8F9FA01000301010101010101010100000000\r\n"
    "@000D017400000102030405060708090A0B110002010204040304070504040001\r\n"
    "@000E01740277000102031104052131061241510761711322328108144291A1B1\r\n"
    "@000F0174C109233352F0156272D10A162434E125F11718191A262728292A3536\r\n"
    "@001001743738393A434445464748494A535455565758595A636465666768696A\r\n"
    "@00110174737475767778797A82838485868788898A92939495969798999AA2A3\r\n"
    "@00120174A4A5A6A7A8A9AAB2B3B4B5B6B7B8B9BAC2C3C4C5C6C7C8C9CAD2D3D4\r\n"
    "@00130174D5D6D7D8D9DAE2E3E4E5E6E7E8E9EAF2F3F4F5F6F7F8F9FAFFC00011\r\n"
    "@001401740801E0028003012200021101031101FFDA000C03010002110311003F\r\n"
    "@0015017400E7DC5446A66435091CD7421094D634A4D464D520109A61A76690D3\r\n";
#define REAL_HEAD_HEADER_BYTES 589
#define REAL_SENTENCE_AT(k) (11 + 67 * (k))

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

    // --- the standard header ------------------------------------------
    // The reference is the 2026-06-01 picture's own header, decoded from the
    // sentences it arrived in. It must open with the JPEG start-of-image
    // marker and carry the start-of-scan marker at byte 575, or the fixture
    // is not the header it claims to be.
    {
        long rlen = (long) strlen(real_head);
        cam_jpeg_stats_t rst;
        long rjlen = 0;
        uint8_t *ref = cam_jpeg_decode((const uint8_t *) real_head, rlen, 0x00, &rst, &rjlen);
        tap_ok(ref != NULL && rjlen > REAL_HEAD_HEADER_BYTES
               && ref[0] == 0xFF && ref[1] == 0xD8 && ref[575] == 0xFF && ref[576] == 0xDA,
               "real header decodes: start-of-image at 0, start-of-scan at 575");
        tap_okf(rst.header_supplied == 0,
                "a header that arrived whole is left alone (supplied %ld)", rst.header_supplied);

        char f[sizeof real_head];

        // Sentences 0 to 20 lost: the header comes back byte for byte, and
        // the counts still say only sentence 21 arrived.
        memcpy(f, real_head, sizeof real_head);
        memset(f + REAL_SENTENCE_AT(0), '?', (size_t) (REAL_SENTENCE_AT(21) - REAL_SENTENCE_AT(0)));
        cam_jpeg_stats_t st1;
        long l1 = 0;
        uint8_t *o1 = cam_jpeg_decode((const uint8_t *) f, rlen, 0x00, &st1, &l1);
        tap_ok(o1 != NULL && ref != NULL && memcmp(o1, ref, REAL_HEAD_HEADER_BYTES) == 0,
               "header lost with sentences 0-20 is restored byte for byte");
        tap_okf(st1.header_supplied == REAL_HEAD_HEADER_BYTES - 1,
                "588 header bytes supplied, byte 588 arrived in sentence 21 (got %ld)",
                st1.header_supplied);
        tap_okf(st1.recovered_bytes == DB && st1.sentences_present == 1,
                "supplied bytes are not counted as received (got %ld bytes, %ld sentences)",
                st1.recovered_bytes, st1.sentences_present);
        free(o1);

        // Only sentence 1 arrived, with one hex digit wrong, as a packet
        // that failed its checksum can leave it: the wrong byte is replaced
        // along with the missing ones.
        memcpy(f, real_head, sizeof real_head);
        memset(f + REAL_SENTENCE_AT(0), '?', (size_t) (REAL_SENTENCE_AT(1) - REAL_SENTENCE_AT(0)));
        memset(f + REAL_SENTENCE_AT(2), '?', (size_t) (REAL_SENTENCE_AT(21) - REAL_SENTENCE_AT(2)));
        char *digit = f + REAL_SENTENCE_AT(1) + 9;
        *digit = (*digit == '0') ? '1' : '0';
        cam_jpeg_stats_t st2;
        uint8_t *o2 = cam_jpeg_decode((const uint8_t *) f, rlen, 0x00, &st2, NULL);
        tap_ok(o2 != NULL && ref != NULL && memcmp(o2, ref, REAL_HEAD_HEADER_BYTES) == 0,
               "a damaged byte in a partial header is corrected too");
        tap_okf(st2.header_supplied == 561,
                "561 bytes supplied: 560 missing plus 1 wrong (got %ld)", st2.header_supplied);
        free(o2);

        // Sentences 0 to 9 all 0x11 and the rest intact: 280 of the 589
        // header bytes disagree, so this is not the standard header and it
        // must come through as it arrived.
        memcpy(f, real_head, sizeof real_head);
        for (int k = 0; k < 10; k++)
            memset(f + REAL_SENTENCE_AT(k) + 9, '1', 2 * DB);
        cam_jpeg_stats_t st3;
        uint8_t *o3 = cam_jpeg_decode((const uint8_t *) f, rlen, 0x00, &st3, NULL);
        int kept = o3 != NULL;
        for (int b = 0; kept && b < 10 * DB; b++) if (o3[b] != 0x11) kept = 0;
        tap_okf(kept && st3.header_supplied == 0,
                "a header mostly unlike the standard one is not replaced (supplied %ld)",
                st3.header_supplied);
        free(o3);
        free(ref);
    }

    // --- histogram equalization ---------------------------------------
    // Expected values are worked by hand from the textbook mapping,
    // round(255 (cdf(v) - cdf_min) / (npix - cdf_min)), with BT.601 luma.

    // Grey levels 10, 10, 20, 30: cdf 2, 3, 4 and cdf_min 2, so they go to
    // 0, 0, 128 (127.5 rounded up) and 255. Two pixels share the darkest
    // level, which catches a cdf_min taken as 1.
    {
        uint8_t g[] = { 10, 10, 10,  10, 10, 10,  20, 20, 20,  30, 30, 30 };
        const uint8_t want[] = { 0, 0, 0,  0, 0, 0,  128, 128, 128,  255, 255, 255 };
        cam_jpeg_equalize_rgb8(g, 4);
        tap_ok(memcmp(g, want, sizeof want) == 0,
               "grey 10,10,20,30 equalize to 0,0,128,255");
    }

    // A coloured pixel moves by its brightness and keeps its colour.
    // (60,20,10) has luma (77*60 + 150*20 + 29*10 + 128) >> 8 = 31; with grey
    // 10 below it and grey 240 above, the levels go to 0, 128 and 255, so all
    // three channels rise by 97. Equalizing each channel on its own would
    // put red at 128, and an equal-weight brightness (30) would move them 98.
    {
        uint8_t px[] = { 10, 10, 10,  60, 20, 10,  240, 240, 240 };
        const uint8_t want[] = { 0, 0, 0,  157, 117, 107,  255, 255, 255 };
        cam_jpeg_equalize_rgb8(px, 3);
        tap_okf(memcmp(px, want, sizeof want) == 0,
                "(60,20,10) between grey 10 and 240 becomes (157,117,107) (got %d,%d,%d)",
                px[3], px[4], px[5]);
    }

    // A channel pushed past 255 stops there rather than wrapping. (250,200,200)
    // has luma 215 and is the brightest level, so it goes to 255 and every
    // channel rises by 40: red would be 290, which wraps to 34 in 8 bits.
    {
        uint8_t px[] = { 0, 0, 0,  250, 200, 200 };
        const uint8_t want[] = { 0, 0, 0,  255, 240, 240 };
        cam_jpeg_equalize_rgb8(px, 2);
        tap_okf(memcmp(px, want, sizeof want) == 0,
                "(250,200,200) beside black becomes (255,240,240) (got %d,%d,%d)",
                px[3], px[4], px[5]);
    }

    // One brightness throughout has nothing to spread (npix - cdf_min is 0)
    // and comes back untouched.
    {
        uint8_t px[] = { 90, 40, 30,  90, 40, 30,  90, 40, 30 };
        const uint8_t want[] = { 90, 40, 30,  90, 40, 30,  90, 40, 30 };
        cam_jpeg_equalize_rgb8(px, 3);
        tap_ok(memcmp(px, want, sizeof want) == 0,
               "a picture of one brightness is left as it is");
    }

    // --- colours stretched to full scale --------------------------------
    // Expected values are round(255 (v - lo) / (hi - lo)) per colour.

    // Each colour on its own range: red 50..150 and green 10..30 put their
    // middle values at 127.5, rounded up to 128; blue 200..210 has nothing
    // in the middle. One range shared by the three (10..210) would put the
    // first pixel's red at 51, not 0.
    {
        uint8_t px[] = { 50, 10, 200,  100, 20, 200,  150, 30, 210 };
        const uint8_t want[] = { 0, 0, 0,  128, 128, 0,  255, 255, 255 };
        cam_jpeg_stretch_rgb8(px, 3);
        tap_okf(memcmp(px, want, sizeof want) == 0,
                "colours stretched separately (got %d,%d,%d / %d,%d,%d / %d,%d,%d)",
                px[0], px[1], px[2], px[3], px[4], px[5], px[6], px[7], px[8]);
    }

    // A colour already spanning 0..255 comes back unchanged, and a colour
    // holding one value (green 90, blue 7) has no range to stretch and is
    // left as it is.
    {
        uint8_t px[] = { 0, 90, 7,  77, 90, 7,  255, 90, 7 };
        const uint8_t want[] = { 0, 90, 7,  77, 90, 7,  255, 90, 7 };
        cam_jpeg_stretch_rgb8(px, 3);
        tap_okf(memcmp(px, want, sizeof want) == 0,
                "full-range red and one-valued green, blue unchanged (got %d,%d,%d)",
                px[3], px[4], px[5]);
    }

    return tap_done();
}
