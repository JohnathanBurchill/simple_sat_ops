/*

    Simple Satellite Operations  oem.c

    Copyright (C) 2025  Johnathan K Burchill

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

#include "oem.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Two-body extrapolation constants. Used beyond the OEM window so
// ground-segment planners can schedule passes further ahead than the
// propagated ephemeris covers. Pure Kepler (no J2, no drag) — degrades
// to minutes/day for LEO; unsuitable for space-safety / conjunction
// screening, fine for pass scheduling.
#define MU_EARTH_KM3_S2     398600.4418
#define OMEGA_EARTH_RAD_S   7.2921150e-5

// Convert Gregorian UTC Y/M/D/h/m/s to Julian Date (fractional days).
// Uses Fliegel & Van Flandern's formula; leap seconds are ignored
// (UTC day is assumed 86400 s, same convention SSM uses).
static double gregorian_to_julian_utc(int y, int mo, int d,
                                      int h, int mi, double sec)
{
    int a = (14 - mo) / 12;
    int yy = y + 4800 - a;
    int mm = mo + 12 * a - 3;
    long jdn = (long)d + (153L * mm + 2) / 5 + 365L * yy
             + yy / 4 - yy / 100 + yy / 400 - 32045L;
    double jul = (double)jdn
               + ((double)h - 12.0) / 24.0
               + (double)mi / 1440.0
               + sec / 86400.0;
    return jul;
}

// Parse an ISO-8601 UTC timestamp ("YYYY-MM-DDThh:mm:ss[.fff...]") into
// a Julian Date. Tolerates trailing "Z" or timezone-less input.
// Returns 0 on success, -1 on malformed.
static int parse_iso8601_utc(const char *s, double *jul_utc_out)
{
    int y = 0, mo = 0, d = 0, h = 0, mi = 0;
    double sec = 0.0;
    int n = sscanf(s, "%d-%d-%dT%d:%d:%lf", &y, &mo, &d, &h, &mi, &sec);
    if (n != 6) {
        return -1;
    }
    *jul_utc_out = gregorian_to_julian_utc(y, mo, d, h, mi, sec);
    return 0;
}

// Strip leading whitespace.
static const char *skip_ws(const char *s)
{
    while (*s == ' ' || *s == '\t') ++s;
    return s;
}

// Strip trailing whitespace in-place.
static void rstrip(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && (s[n-1] == ' ' || s[n-1] == '\t' ||
                     s[n-1] == '\r' || s[n-1] == '\n')) {
        s[--n] = '\0';
    }
}

// Returns 1 if `line` starts with `prefix` (after leading whitespace).
// Sets *value_out to the first char after the '=' (or NULL if no '=').
static int match_kv(const char *line, const char *prefix, const char **value_out)
{
    const char *p = skip_ws(line);
    size_t pn = strlen(prefix);
    if (strncmp(p, prefix, pn) != 0) return 0;
    // Require either end-of-key or whitespace/= after the prefix
    char c = p[pn];
    if (c != '\0' && c != ' ' && c != '\t' && c != '=') return 0;
    const char *eq = strchr(p + pn, '=');
    if (eq == NULL) {
        if (value_out) *value_out = NULL;
        return 1;
    }
    if (value_out) {
        *value_out = skip_ws(eq + 1);
    }
    return 1;
}

static int ensure_capacity(oem_table_t *t, size_t need)
{
    if (need <= t->capacity) return 0;
    size_t cap = t->capacity == 0 ? 256 : t->capacity;
    while (cap < need) cap *= 2;
    oem_sample_t *grown = (oem_sample_t *)realloc(t->samples, cap * sizeof(*grown));
    if (grown == NULL) {
        fprintf(stderr, "oem: out of memory (need %zu samples)\n", need);
        return -1;
    }
    t->samples = grown;
    t->capacity = cap;
    return 0;
}

// State-machine OEM parser. Accepts the subset SSM emits (CCSDS OEM 3.0
// with META, ephemeris, optional COVARIANCE). Unknown keys are ignored.
int oem_parse(const char *text, oem_table_t *out)
{
    if (text == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));

    enum { PRE_META, IN_META, EPHEMERIS, IN_COV, POST } mode = PRE_META;
    const char *p = text;
    int line_no = 0;
    char line[1024];

    while (*p) {
        // Copy one line
        const char *eol = strchr(p, '\n');
        size_t len = eol ? (size_t)(eol - p) : strlen(p);
        if (len >= sizeof(line)) len = sizeof(line) - 1;
        memcpy(line, p, len);
        line[len] = '\0';
        rstrip(line);
        p = eol ? (eol + 1) : (p + strlen(p));
        ++line_no;

        // Blank lines allowed anywhere
        const char *t = skip_ws(line);
        if (*t == '\0') continue;

        if (mode == PRE_META) {
            if (match_kv(line, "META_START", NULL)) {
                mode = IN_META;
                continue;
            }
            // Header keys (CCSDS_OEM_VERS, CREATION_DATE, ORIGINATOR) just
            // get skipped; we don't need them.
            continue;
        }

        if (mode == IN_META) {
            const char *val = NULL;
            if (match_kv(line, "META_STOP", NULL)) {
                mode = EPHEMERIS;
                continue;
            }
            if (match_kv(line, "OBJECT_NAME", &val) && val) {
                strncpy(out->object_name, val, sizeof(out->object_name) - 1);
                continue;
            }
            if (match_kv(line, "OBJECT_ID", &val) && val) {
                strncpy(out->object_id, val, sizeof(out->object_id) - 1);
                continue;
            }
            if (match_kv(line, "REF_FRAME", &val) && val) {
                strncpy(out->ref_frame, val, sizeof(out->ref_frame) - 1);
                continue;
            }
            if (match_kv(line, "TIME_SYSTEM", &val) && val) {
                strncpy(out->time_system, val, sizeof(out->time_system) - 1);
                continue;
            }
            if (match_kv(line, "START_TIME", &val) && val) {
                if (parse_iso8601_utc(val, &out->start_jul_utc) != 0) {
                    fprintf(stderr, "oem: bad START_TIME at line %d: %s\n", line_no, val);
                    oem_free(out);
                    return -1;
                }
                continue;
            }
            if (match_kv(line, "STOP_TIME", &val) && val) {
                if (parse_iso8601_utc(val, &out->stop_jul_utc) != 0) {
                    fprintf(stderr, "oem: bad STOP_TIME at line %d: %s\n", line_no, val);
                    oem_free(out);
                    return -1;
                }
                continue;
            }
            // Other meta keys — ignore.
            continue;
        }

        if (mode == EPHEMERIS) {
            if (match_kv(line, "COVARIANCE_START", NULL)) {
                mode = IN_COV;
                continue;
            }
            if (match_kv(line, "META_START", NULL)) {
                // OEM allows multiple META blocks; we don't support that.
                fprintf(stderr, "oem: multiple META blocks not supported (line %d)\n", line_no);
                oem_free(out);
                return -1;
            }
            // Ephemeris row: <ISO8601> <X> <Y> <Z> <VX> <VY> <VZ>
            char tstr[64];
            double x, y, z, vx, vy, vz;
            int n = sscanf(line, "%63s %lf %lf %lf %lf %lf %lf",
                           tstr, &x, &y, &z, &vx, &vy, &vz);
            if (n != 7) {
                fprintf(stderr, "oem: skipping unrecognized ephemeris line %d: %s\n",
                        line_no, line);
                continue;
            }
            double jul;
            if (parse_iso8601_utc(tstr, &jul) != 0) {
                fprintf(stderr, "oem: bad time at line %d: %s\n", line_no, tstr);
                oem_free(out);
                return -1;
            }
            if (ensure_capacity(out, out->n_samples + 1) != 0) {
                oem_free(out);
                return -1;
            }
            oem_sample_t *s = &out->samples[out->n_samples++];
            s->jul_utc = jul;
            s->r_ecef[0] = x;
            s->r_ecef[1] = y;
            s->r_ecef[2] = z;
            s->v_ecef[0] = vx;
            s->v_ecef[1] = vy;
            s->v_ecef[2] = vz;
            continue;
        }

        if (mode == IN_COV) {
            if (match_kv(line, "COVARIANCE_STOP", NULL)) {
                mode = POST;
                continue;
            }
            // Covariance rows — skip; we don't use them for pass planning.
            continue;
        }

        // POST: ignore further content.
    }

    if (out->n_samples < 2) {
        fprintf(stderr, "oem: need at least 2 ephemeris points, got %zu\n",
                out->n_samples);
        oem_free(out);
        return -1;
    }
    if (out->start_jul_utc == 0.0) {
        out->start_jul_utc = out->samples[0].jul_utc;
    }
    if (out->stop_jul_utc == 0.0) {
        out->stop_jul_utc = out->samples[out->n_samples - 1].jul_utc;
    }
    return 0;
}

// Slurp stdout of `ssm trajectory <id>` into a buffer, then parse.
int oem_load_from_ssm(const char *trajectory_id, oem_table_t *out)
{
    if (trajectory_id == NULL || out == NULL) return -1;
    // Shell-quote the id paranoidly (single quotes + escape any ' in id).
    // IDs from `ssm trajectories` are UUIDs so in practice no escaping is
    // needed, but treat it as untrusted.
    size_t cmd_cap = strlen(trajectory_id) * 2 + 64;
    char *cmd = (char *)malloc(cmd_cap);
    if (cmd == NULL) return -1;
    size_t off = 0;
    off += (size_t)snprintf(cmd + off, cmd_cap - off, "ssm trajectory '");
    for (const char *c = trajectory_id; *c; ++c) {
        if (*c == '\'') {
            if (off + 4 >= cmd_cap) { free(cmd); return -1; }
            memcpy(cmd + off, "'\\''", 4);
            off += 4;
        } else {
            if (off + 1 >= cmd_cap) { free(cmd); return -1; }
            cmd[off++] = *c;
        }
    }
    if (off + 2 >= cmd_cap) { free(cmd); return -1; }
    cmd[off++] = '\'';
    cmd[off] = '\0';

    FILE *pipe = popen(cmd, "r");
    if (pipe == NULL) {
        fprintf(stderr, "oem: popen(%s) failed: %s\n", cmd, strerror(errno));
        free(cmd);
        return -1;
    }
    free(cmd);

    // Slurp stdout into a growing buffer.
    size_t buf_cap = 65536;
    size_t buf_len = 0;
    char *buf = (char *)malloc(buf_cap);
    if (buf == NULL) {
        pclose(pipe);
        return -1;
    }
    for (;;) {
        if (buf_cap - buf_len < 4096) {
            size_t nc = buf_cap * 2;
            char *grown = (char *)realloc(buf, nc);
            if (grown == NULL) {
                free(buf);
                pclose(pipe);
                return -1;
            }
            buf = grown;
            buf_cap = nc;
        }
        size_t got = fread(buf + buf_len, 1, buf_cap - buf_len - 1, pipe);
        buf_len += got;
        if (got == 0) break;
    }
    buf[buf_len] = '\0';
    int rc = pclose(pipe);
    if (rc != 0) {
        fprintf(stderr, "oem: `ssm trajectory %s` exited non-zero (%d). "
                "Check that ssm is on PATH and the id is valid "
                "(run `ssm trajectories` to list).\n", trajectory_id, rc);
        free(buf);
        return -1;
    }

    int rv = oem_parse(buf, out);
    free(buf);
    return rv;
}

// Greenwich Mean Sidereal Time in radians, IAU 1982 simplified form.
// Good to arcseconds over decades — far better than needed for ground-
// segment pointing accuracy.
static double gmst_rad(double jul_utc)
{
    double d = jul_utc - 2451545.0;  // days since J2000
    double g = 2.0 * M_PI * (0.7790572732640 + 1.00273781191135448 * d);
    g = fmod(g, 2.0 * M_PI);
    if (g < 0.0) g += 2.0 * M_PI;
    return g;
}

// ITRF (rotating) -> ECI (inertial) state transform at `jul_utc`.
static void itrf_to_eci(double jul_utc,
                        const double r_itrf[3], const double v_itrf[3],
                        double r_eci[3], double v_eci[3])
{
    double th = gmst_rad(jul_utc);
    double c = cos(th), s = sin(th);
    r_eci[0] = c * r_itrf[0] - s * r_itrf[1];
    r_eci[1] = s * r_itrf[0] + c * r_itrf[1];
    r_eci[2] = r_itrf[2];
    // v_eci = ω × r_eci + R_z(θ) · v_itrf
    double v_rot[3] = {
        c * v_itrf[0] - s * v_itrf[1],
        s * v_itrf[0] + c * v_itrf[1],
        v_itrf[2]
    };
    v_eci[0] = v_rot[0] - OMEGA_EARTH_RAD_S * r_eci[1];
    v_eci[1] = v_rot[1] + OMEGA_EARTH_RAD_S * r_eci[0];
    v_eci[2] = v_rot[2];
}

// ECI (inertial) -> ITRF (rotating) state transform at `jul_utc`.
static void eci_to_itrf(double jul_utc,
                        const double r_eci[3], const double v_eci[3],
                        double r_itrf[3], double v_itrf[3])
{
    double th = gmst_rad(jul_utc);
    double c = cos(th), s = sin(th);
    r_itrf[0] =  c * r_eci[0] + s * r_eci[1];
    r_itrf[1] = -s * r_eci[0] + c * r_eci[1];
    r_itrf[2] = r_eci[2];
    // v_itrf = R_z(-θ) · (v_eci - ω × r_eci)
    double vm[3] = {
        v_eci[0] + OMEGA_EARTH_RAD_S * r_eci[1],
        v_eci[1] - OMEGA_EARTH_RAD_S * r_eci[0],
        v_eci[2]
    };
    v_itrf[0] =  c * vm[0] + s * vm[1];
    v_itrf[1] = -s * vm[0] + c * vm[1];
    v_itrf[2] = vm[2];
}

// Two-body Keplerian propagator using Lagrange f/g coefficients.
// Returns 0 on success, -1 if the orbit is parabolic/hyperbolic (we
// don't handle escape trajectories here).
static int kepler_propagate_eci(const double r0[3], const double v0[3],
                                double dt_sec,
                                double r_out[3], double v_out[3])
{
    double mu = MU_EARTH_KM3_S2;
    double r0m = sqrt(r0[0]*r0[0] + r0[1]*r0[1] + r0[2]*r0[2]);
    if (r0m <= 0.0) return -1;
    double v0sq = v0[0]*v0[0] + v0[1]*v0[1] + v0[2]*v0[2];
    double energy = 0.5 * v0sq - mu / r0m;
    if (energy >= 0.0) return -1;
    double a = -mu / (2.0 * energy);

    // h = r × v, |h|
    double h[3] = {
        r0[1]*v0[2] - r0[2]*v0[1],
        r0[2]*v0[0] - r0[0]*v0[2],
        r0[0]*v0[1] - r0[1]*v0[0]
    };
    (void)h;

    // Eccentricity vector e_vec = (v × h)/μ - r/|r|
    double vxh[3] = {
        v0[1]*h[2] - v0[2]*h[1],
        v0[2]*h[0] - v0[0]*h[2],
        v0[0]*h[1] - v0[1]*h[0]
    };
    double e_vec[3] = {
        vxh[0]/mu - r0[0]/r0m,
        vxh[1]/mu - r0[1]/r0m,
        vxh[2]/mu - r0[2]/r0m
    };
    double e = sqrt(e_vec[0]*e_vec[0] + e_vec[1]*e_vec[1] + e_vec[2]*e_vec[2]);

    // Eccentric anomaly at t0
    double cosE0 = (1.0 - r0m / a) / (e > 1e-20 ? e : 1e-20);
    if (cosE0 >  1.0) cosE0 =  1.0;
    if (cosE0 < -1.0) cosE0 = -1.0;
    double E0 = acos(cosE0);
    double rdv = r0[0]*v0[0] + r0[1]*v0[1] + r0[2]*v0[2];
    if (rdv < 0.0) E0 = -E0;

    double n = sqrt(mu / (a*a*a));
    double M0 = E0 - e * sin(E0);
    double M  = M0 + n * dt_sec;

    // Newton solve M = E - e sin E. Initial guess = M; converges in <10
    // iterations for any eccentricity we'd see on a LEO satellite.
    double E = M;
    for (int i = 0; i < 30; ++i) {
        double f  = E - e * sin(E) - M;
        double fp = 1.0 - e * cos(E);
        double dE = f / fp;
        E -= dE;
        if (fabs(dE) < 1e-12) break;
    }

    double dE_step = E - E0;
    double sinDE = sin(dE_step), cosDE = cos(dE_step);
    double rm = a * (1.0 - e * cos(E));
    double fc = 1.0 - (a / r0m) * (1.0 - cosDE);
    double gc = dt_sec - sqrt(a*a*a / mu) * (dE_step - sinDE);
    double fcd = -sqrt(mu * a) / (rm * r0m) * sinDE;
    double gcd = 1.0 - (a / rm) * (1.0 - cosDE);

    for (int k = 0; k < 3; ++k) {
        r_out[k] = fc  * r0[k] + gc  * v0[k];
        v_out[k] = fcd * r0[k] + gcd * v0[k];
    }
    return 0;
}

// Binary search for the largest index i such that samples[i].jul_utc <= t.
// Returns -1 if t is before samples[0], or n_samples-1 if after the last.
static ssize_t bracket_left(const oem_table_t *t, double jul_utc)
{
    if (t->n_samples == 0) return -1;
    if (jul_utc < t->samples[0].jul_utc) return -1;
    if (jul_utc >= t->samples[t->n_samples - 1].jul_utc) {
        return (ssize_t)(t->n_samples - 1);
    }
    size_t lo = 0, hi = t->n_samples - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (t->samples[mid].jul_utc <= jul_utc) lo = mid;
        else hi = mid;
    }
    return (ssize_t)lo;
}

int oem_sample_at(const oem_table_t *t, double jul_utc,
                  double r_ecef[3], double v_ecef[3])
{
    if (t == NULL || t->n_samples < 2) return -1;

    // Beyond-window: two-body Keplerian extrapolation from the nearest
    // endpoint sample. Accuracy for LEO is ~minutes/day of AOS drift
    // (no J2, no drag); adequate for ground-segment scheduling, not
    // space-safety work.
    if (jul_utc < t->start_jul_utc - 1e-9 ||
        jul_utc > t->stop_jul_utc  + 1e-9) {
        const oem_sample_t *anchor =
            (jul_utc > t->stop_jul_utc)
                ? &t->samples[t->n_samples - 1]
                : &t->samples[0];
        double r_eci[3], v_eci[3];
        itrf_to_eci(anchor->jul_utc, anchor->r_ecef, anchor->v_ecef, r_eci, v_eci);
        double dt_sec = (jul_utc - anchor->jul_utc) * 86400.0;
        double r_eci_new[3], v_eci_new[3];
        if (kepler_propagate_eci(r_eci, v_eci, dt_sec, r_eci_new, v_eci_new) != 0) {
            return -1;
        }
        eci_to_itrf(jul_utc, r_eci_new, v_eci_new, r_ecef, v_ecef);
        return 0;
    }

    ssize_t i = bracket_left(t, jul_utc);
    if (i < 0 || (size_t)i >= t->n_samples - 1) {
        // At or past the last knot — clamp to the final pair.
        i = (ssize_t)t->n_samples - 2;
    }
    const oem_sample_t *a = &t->samples[i];
    const oem_sample_t *b = &t->samples[i + 1];

    double dt_days = b->jul_utc - a->jul_utc;
    if (dt_days <= 0.0) return -1;
    double dt_sec = dt_days * 86400.0;
    double u = (jul_utc - a->jul_utc) / dt_days;
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;

    // Cubic Hermite basis
    double u2 = u * u;
    double u3 = u2 * u;
    double h00 =  2.0 * u3 - 3.0 * u2 + 1.0;
    double h10 =        u3 - 2.0 * u2 + u;
    double h01 = -2.0 * u3 + 3.0 * u2;
    double h11 =        u3 -       u2;

    // Derivative basis (per unit u; divide by dt_sec to convert to per sec)
    double dh00 =  6.0 * u2 - 6.0 * u;
    double dh10 =  3.0 * u2 - 4.0 * u + 1.0;
    double dh01 = -6.0 * u2 + 6.0 * u;
    double dh11 =  3.0 * u2 - 2.0 * u;

    for (int k = 0; k < 3; ++k) {
        r_ecef[k] = h00 * a->r_ecef[k]
                  + h10 * dt_sec * a->v_ecef[k]
                  + h01 * b->r_ecef[k]
                  + h11 * dt_sec * b->v_ecef[k];
        v_ecef[k] = (dh00 * a->r_ecef[k]
                  +  dh10 * dt_sec * a->v_ecef[k]
                  +  dh01 * b->r_ecef[k]
                  +  dh11 * dt_sec * b->v_ecef[k]) / dt_sec;
    }
    return 0;
}

void oem_free(oem_table_t *t)
{
    if (t == NULL) return;
    free(t->samples);
    t->samples = NULL;
    t->n_samples = 0;
    t->capacity = 0;
}
