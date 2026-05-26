/*

   Simple Satellite Operations  utils/pdf_writer.c

   Minimal vector PDF writer — see utils/pdf_writer.h for the API.
   Pattern originally lifted from ~/src/lorentz_tracer's pdf_export.c,
   pared down to one page, Standard-14 fonts, ASCII-only text.

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

#include "pdf_writer.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PDFMAX_OBJ 64

struct pdfw {
    FILE  *fp;
    long   bytes;
    long   obj_off[PDFMAX_OBJ];
    int    n_objects;
    char  *cs;
    size_t cs_len, cs_cap;
    float  page_w, page_h;
    int    catalog_obj, pages_obj, page_obj, content_obj;
    int    font_helv, font_cour;
};

static void pdfw_cs_append(pdfw_t *w, const char *s, size_t n)
{
    if (w->cs_len + n + 1 > w->cs_cap) {
        size_t nc = w->cs_cap ? w->cs_cap * 2 : 8192;
        while (nc < w->cs_len + n + 1) nc *= 2;
        char *p = (char *) realloc(w->cs, nc);
        if (p == NULL) return;
        w->cs = p; w->cs_cap = nc;
    }
    memcpy(w->cs + w->cs_len, s, n);
    w->cs_len += n;
    w->cs[w->cs_len] = '\0';
}

__attribute__((format(printf, 2, 3)))
static void pdfw_csf(pdfw_t *w, const char *fmt, ...)
{
    char buf[1024];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0 && n < (int) sizeof buf) pdfw_cs_append(w, buf, (size_t) n);
}

__attribute__((format(printf, 2, 3)))
static void pdfw_writef(pdfw_t *w, const char *fmt, ...)
{
    char buf[1024];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) { fwrite(buf, 1, (size_t) n, w->fp); w->bytes += n; }
}

#define PDFY(W, Y) ((W)->page_h - (float)(Y))

pdfw_t *pdfw_begin(const char *path, float page_w, float page_h)
{
    pdfw_t *w = (pdfw_t *) calloc(1, sizeof(*w));
    if (w == NULL) return NULL;
    w->fp = fopen(path, "wb");
    if (w->fp == NULL) { free(w); return NULL; }
    w->page_w = page_w; w->page_h = page_h;
    static const char hdr[] = "%PDF-1.4\n%\xe2\xe3\xcf\xd3\n";
    fwrite(hdr, 1, sizeof hdr - 1, w->fp);
    w->bytes = (long)(sizeof hdr - 1);
    return w;
}

static int pdfw_next_obj(pdfw_t *w)
{
    if (w->n_objects + 1 >= PDFMAX_OBJ) return -1;
    return ++w->n_objects;
}

void pdfw_set_stroke(pdfw_t *w, pdfw_rgb_t c)
{
    pdfw_csf(w, "%.4f %.4f %.4f RG\n", c.r/255.0, c.g/255.0, c.b/255.0);
}
void pdfw_set_fill(pdfw_t *w, pdfw_rgb_t c)
{
    pdfw_csf(w, "%.4f %.4f %.4f rg\n", c.r/255.0, c.g/255.0, c.b/255.0);
}
void pdfw_lw(pdfw_t *w, float lw) { pdfw_csf(w, "%.3f w\n", lw); }

void pdfw_line(pdfw_t *w, float x1, float y1, float x2, float y2)
{
    pdfw_csf(w, "%.4f %.4f m %.4f %.4f l S\n",
             x1, PDFY(w, y1), x2, PDFY(w, y2));
}
void pdfw_rect_stroke(pdfw_t *w, float x, float y, float ww, float hh)
{
    pdfw_csf(w, "%.4f %.4f %.4f %.4f re S\n",
             x, PDFY(w, y + hh), ww, hh);
}
void pdfw_rect_fill(pdfw_t *w, float x, float y, float ww, float hh)
{
    pdfw_csf(w, "%.4f %.4f %.4f %.4f re f\n",
             x, PDFY(w, y + hh), ww, hh);
}
void pdfw_clip_begin(pdfw_t *w, float x, float y, float ww, float hh)
{
    pdfw_csf(w, "q\n%.4f %.4f %.4f %.4f re W n\n",
             x, PDFY(w, y + hh), ww, hh);
}
void pdfw_clip_end(pdfw_t *w)
{
    pdfw_csf(w, "Q\n");
}
void pdfw_text(pdfw_t *w, float x, float y_top,
               const char *s, float fsz, int mono)
{
    if (s == NULL || !*s) return;
    float baseline_y = y_top + fsz * 0.8f;
    pdfw_csf(w, "BT /F%d %.2f Tf %.4f %.4f Td (", mono ? 1 : 0,
             fsz, x, PDFY(w, baseline_y));
    for (const char *p = s; *p; ++p) {
        unsigned char c = (unsigned char) *p;
        if (c == '(' || c == ')' || c == '\\') {
            char b[2] = {'\\', (char) c}; pdfw_cs_append(w, b, 2);
        } else if (c < 0x20 || c > 0x7E) {
            pdfw_cs_append(w, "?", 1);
        } else {
            pdfw_cs_append(w, (const char *) &c, 1);
        }
    }
    pdfw_cs_append(w, ") Tj ET\n", 8);
}
float pdfw_str_width(const char *s, float fsz, int mono)
{
    if (s == NULL) return 0.0f;
    int n = (int) strlen(s);
    return n * (mono ? 0.60f : 0.55f) * fsz;
}

int pdfw_end(pdfw_t *w)
{
    w->catalog_obj = pdfw_next_obj(w);
    w->pages_obj   = pdfw_next_obj(w);
    w->page_obj    = pdfw_next_obj(w);
    w->content_obj = pdfw_next_obj(w);
    w->font_helv   = pdfw_next_obj(w);
    w->font_cour   = pdfw_next_obj(w);
    if (w->font_cour < 0) { fclose(w->fp); free(w->cs); free(w); return -1; }

    w->obj_off[w->catalog_obj] = w->bytes;
    pdfw_writef(w, "%d 0 obj\n<< /Type /Catalog /Pages %d 0 R >>\nendobj\n",
                w->catalog_obj, w->pages_obj);
    w->obj_off[w->pages_obj] = w->bytes;
    pdfw_writef(w,
        "%d 0 obj\n<< /Type /Pages /Count 1 /Kids [%d 0 R] >>\nendobj\n",
        w->pages_obj, w->page_obj);
    w->obj_off[w->page_obj] = w->bytes;
    pdfw_writef(w,
        "%d 0 obj\n<< /Type /Page /Parent %d 0 R "
        "/MediaBox [0 0 %.4f %.4f] /Contents %d 0 R "
        "/Resources << /Font << /F0 %d 0 R /F1 %d 0 R >> >> "
        ">>\nendobj\n",
        w->page_obj, w->pages_obj, w->page_w, w->page_h,
        w->content_obj, w->font_helv, w->font_cour);
    w->obj_off[w->content_obj] = w->bytes;
    pdfw_writef(w, "%d 0 obj\n<< /Length %zu >>\nstream\n",
                w->content_obj, w->cs_len);
    if (w->cs_len > 0) {
        fwrite(w->cs, 1, w->cs_len, w->fp);
        w->bytes += (long) w->cs_len;
    }
    pdfw_writef(w, "\nendstream\nendobj\n");
    w->obj_off[w->font_helv] = w->bytes;
    pdfw_writef(w,
        "%d 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica "
        "/Encoding /WinAnsiEncoding >>\nendobj\n", w->font_helv);
    w->obj_off[w->font_cour] = w->bytes;
    pdfw_writef(w,
        "%d 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Courier "
        "/Encoding /WinAnsiEncoding >>\nendobj\n", w->font_cour);

    long xref_off = w->bytes;
    pdfw_writef(w, "xref\n0 %d\n0000000000 65535 f \n",
                w->n_objects + 1);
    for (int i = 1; i <= w->n_objects; ++i)
        pdfw_writef(w, "%010ld 00000 n \n", w->obj_off[i]);
    pdfw_writef(w,
        "trailer\n<< /Size %d /Root %d 0 R >>\nstartxref\n%ld\n%%%%EOF\n",
        w->n_objects + 1, w->catalog_obj, xref_off);
    fclose(w->fp);
    free(w->cs);
    free(w);
    return 0;
}
