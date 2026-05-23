/*

   Simple Satellite Operations  utils/pdf_writer.h

   Minimal single-page vector PDF writer.  Standard-14 fonts
   (Helvetica + Courier), stroked / filled lines + rectangles,
   ASCII-only text.  Coordinates are pixel-style (Y down, origin
   top-left); the writer flips internally to PDF page space.

   Originally inlined in utils/iq_annotator.c; pulled out so the
   bench tools (b210_gain_sweep, …) can share the same code path
   and produce visually consistent reports.

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

#ifndef PDF_WRITER_H
#define PDF_WRITER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// RGBA byte color.  Layout-compatible with raylib's Color so callers
// already using raylib can pass-through with a single explicit cast or
// field-by-field copy.
typedef struct pdfw_rgb {
    unsigned char r, g, b, a;
} pdfw_rgb_t;

#define PDFW_BLACK   ((pdfw_rgb_t){  0,   0,   0, 255})
#define PDFW_WHITE   ((pdfw_rgb_t){255, 255, 255, 255})
#define PDFW_GREY    ((pdfw_rgb_t){120, 120, 120, 255})
#define PDFW_LGREY   ((pdfw_rgb_t){200, 200, 210, 255})
#define PDFW_DGREY   ((pdfw_rgb_t){ 80,  80,  80, 255})

// Opaque writer handle.  Begin opens the file and emits the header.
// End writes the trailer, closes the file, and frees the handle.
typedef struct pdfw pdfw_t;

pdfw_t *pdfw_begin(const char *path, float page_w, float page_h);
int     pdfw_end(pdfw_t *w);

// Graphics state.
void pdfw_set_stroke(pdfw_t *w, pdfw_rgb_t c);
void pdfw_set_fill  (pdfw_t *w, pdfw_rgb_t c);
void pdfw_lw        (pdfw_t *w, float lw);

// Primitives.  All coordinates use top-left-origin pixel space; the
// writer flips to PDF's bottom-left-origin internally.
void pdfw_line       (pdfw_t *w, float x1, float y1, float x2, float y2);
void pdfw_rect_stroke(pdfw_t *w, float x,  float y,  float ww, float hh);
void pdfw_rect_fill  (pdfw_t *w, float x,  float y,  float ww, float hh);

// Text.  y_top = y coordinate of the cap line.  mono != 0 selects
// Courier; otherwise Helvetica.  Characters outside printable ASCII
// are replaced with '?'.
void  pdfw_text     (pdfw_t *w, float x, float y_top,
                      const char *s, float fsz, int mono);
float pdfw_str_width(const char *s, float fsz, int mono);

#ifdef __cplusplus
}
#endif

#endif // PDF_WRITER_H
