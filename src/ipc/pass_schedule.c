/*

    Simple Satellite Operations  pass_schedule.c

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

// See pass_schedule.h. Self-contained codec (it does not share
// sso_ipc_codec's static helpers). The per-pass values ride as one flat JSON
// number array under "p" — five numbers per pass [aos,los,peak,el,az] — which
// keeps both the encoder and the decoder trivial (no nested-array walk): the
// decoder just reads count*5 doubles between the brackets.

#include "pass_schedule.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Locate "key" at the top level and return a pointer just past its colon
// (whitespace skipped), or NULL. The leading quote in the search pattern means
// a bare substring inside another key/value can't match.
static const char *find_key(const char *json, const char *key)
{
    char pat[64];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    size_t plen = strlen(pat);
    const char *p = json;
    while ((p = strstr(p, pat)) != NULL) {
        const char *q = p + plen;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == ':') {
            q++;
            while (*q == ' ' || *q == '\t') q++;
            return q;
        }
        p += plen;
    }
    return NULL;
}

// Copy a string value (starting at the opening quote) into out, stopping at the
// closing quote. Minimal unescaping (the producer never puts quotes/backslashes
// in sat names or designators). Returns 1 if a string was read, 0 otherwise.
static int get_str(const char *json, const char *key, char *out, size_t out_n)
{
    if (out_n) out[0] = '\0';
    const char *q = find_key(json, key);
    if (!q || *q != '"') return 0;
    q++;
    size_t i = 0;
    while (*q && *q != '"' && i + 1 < out_n) {
        if (*q == '\\' && q[1]) q++;   // pass the escaped char through verbatim
        out[i++] = *q++;
    }
    if (out_n) out[i] = '\0';
    return 1;
}

static int get_num(const char *json, const char *key, double *out)
{
    const char *q = find_key(json, key);
    if (!q) return 0;
    char *endp;
    double v = strtod(q, &endp);
    if (endp == q) return 0;
    *out = v;
    return 1;
}

int pass_schedule_encode(const pass_schedule_t *s, char *out, size_t out_size)
{
    if (!s || !out || out_size < 16) return -1;
    int n = s->count;
    if (n < 0) n = 0;
    if (n > PASS_SCHED_MAX) n = PASS_SCHED_MAX;

    int off = snprintf(out, out_size,
                       "{\"t\":\"passes\",\"sat\":\"%s\",\"idesg\":\"%s\","
                       "\"gen\":%.0f,\"ep_min\":%.6g,\"n\":%d,\"p\":[",
                       s->satellite, s->idesg, s->generated_unix,
                       s->tle_epoch_min, n);
    if (off < 0 || (size_t) off >= out_size) return -1;

    for (int i = 0; i < n; i++) {
        const pass_t_wire *p = &s->passes[i];
        int w = snprintf(out + off, out_size - off,
                         "%s%.0f,%.0f,%.0f,%.3f,%.3f",
                         i ? "," : "",
                         p->aos_unix, p->los_unix, p->peak_unix,
                         p->peak_el_deg, p->peak_az_deg);
        if (w < 0 || (size_t) (off + w) >= out_size) return -1;
        off += w;
    }

    int w = snprintf(out + off, out_size - off, "]}\n");
    if (w < 0 || (size_t) (off + w) >= out_size) return -1;
    return 0;
}

int pass_schedule_decode(const char *line, pass_schedule_t *out)
{
    if (!line || !out) return -1;
    memset(out, 0, sizeof *out);

    // Must be a passes event.
    char t[16];
    if (!get_str(line, "t", t, sizeof t) || strcmp(t, "passes") != 0) return -1;

    get_str(line, "sat", out->satellite, sizeof out->satellite);
    get_str(line, "idesg", out->idesg, sizeof out->idesg);
    get_num(line, "gen", &out->generated_unix);
    get_num(line, "ep_min", &out->tle_epoch_min);
    double nd = 0;
    get_num(line, "n", &nd);
    int n = (int) nd;
    if (n < 0) n = 0;
    if (n > PASS_SCHED_MAX) n = PASS_SCHED_MAX;

    const char *q = find_key(line, "p");
    if (q && *q == '[') {
        q++;
        int read = 0;
        int want = n * 5;
        double vals[PASS_SCHED_MAX * 5];
        while (read < want && *q && *q != ']') {
            char *endp;
            double v = strtod(q, &endp);
            if (endp == q) break;
            vals[read++] = v;
            q = endp;
            while (*q == ' ' || *q == ',') q++;
        }
        // Only keep whole passes that actually arrived.
        int have = read / 5;
        if (have < n) n = have;
        for (int i = 0; i < n; i++) {
            pass_t_wire *p = &out->passes[i];
            p->aos_unix    = vals[i * 5 + 0];
            p->los_unix    = vals[i * 5 + 1];
            p->peak_unix   = vals[i * 5 + 2];
            p->peak_el_deg = vals[i * 5 + 3];
            p->peak_az_deg = vals[i * 5 + 4];
        }
    } else {
        n = 0;
    }
    out->count = n;
    return 0;
}
