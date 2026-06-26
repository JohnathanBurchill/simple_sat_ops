/*

    Simple Satellite Operations  src/proto/adcs_mag.c

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

#include "adcs_mag.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Find the integer value of a JSON number that follows `key` (which must
// include its surrounding quotes, e.g. "\"x_nT\""). Returns 1 and sets *out
// on success. The quoted key keeps "x_nT" from matching inside the longer
// "magnetic_field_x_nT" -- the embedded run there is preceded by '_', not '"'.
static int find_long(const char *text, const char *key, long *out)
{
    const char *p = strstr(text, key);
    if (p == NULL) return 0;
    p += strlen(key);
    // Skip to the ':' that separates the key from its value, then any spaces.
    const char *colon = strchr(p, ':');
    if (colon == NULL) return 0;
    p = colon + 1;
    while (*p == ' ' || *p == '\t') p++;
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) return 0;   // no digits where a number was expected
    *out = v;
    return 1;
}

int adcs_mag_parse(const char *text, adcs_mag_vec_t *out)
{
    out->kind = ADCS_MAG_NONE;
    out->x_nT = out->y_nT = out->z_nT = 0;
    if (text == NULL) return 0;

    // adcs_measurements carries the measured field under magnetic_field_*_nT
    // (alongside much else). Recognize it first; its keys are unambiguous.
    long x, y, z;
    if (find_long(text, "\"magnetic_field_x_nT\"", &x)
        && find_long(text, "\"magnetic_field_y_nT\"", &y)
        && find_long(text, "\"magnetic_field_z_nT\"", &z)) {
        out->kind = ADCS_MAG_MEASUREMENTS;
        out->x_nT = x; out->y_nT = y; out->z_nT = z;
        return 1;
    }

    // The bare {"x_nT":..,"y_nT":..,"z_nT":..} object, shared by the measured
    // and IGRF-model field commands.
    if (find_long(text, "\"x_nT\"", &x)
        && find_long(text, "\"y_nT\"", &y)
        && find_long(text, "\"z_nT\"", &z)) {
        out->kind = ADCS_MAG_NT;
        out->x_nT = x; out->y_nT = y; out->z_nT = z;
        return 1;
    }

    // The raw magnetometer object {"x":..,"y":..,"z":..}: uncalibrated ADC
    // counts. The quoted keys "x"/"y"/"z" can't match inside "x_nT" or
    // "x_micro" (those have '_' after the x, not a closing quote), so this is
    // checked last and only fires on the bare form. Values are counts, not nT.
    if (find_long(text, "\"x\"", &x)
        && find_long(text, "\"y\"", &y)
        && find_long(text, "\"z\"", &z)) {
        out->kind = ADCS_MAG_RAW;
        out->x_nT = x; out->y_nT = y; out->z_nT = z;
        return 1;
    }

    return 0;
}

double adcs_mag_magnitude(const adcs_mag_vec_t *v)
{
    double x = (double)v->x_nT, y = (double)v->y_nT, z = (double)v->z_nT;
    return sqrt(x * x + y * y + z * z);
}

// Hex digit value, or -1 if c isn't a hex digit.
static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int adcs_mag_parse_telem_hex(const char *body, int nt_per_lsb, adcs_mag_vec_t *out)
{
    out->kind = ADCS_MAG_NONE;
    out->x_nT = out->y_nT = out->z_nT = 0;
    if (body == NULL) return 0;

    unsigned char b[6];
    int nb = 0;
    const char *p = body;
    while (nb < 6) {
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        int hi = hexval((unsigned char)*p);
        if (hi < 0) break;   // not a hex token: the dump has ended
        int val = hi; p++;
        int lo = hexval((unsigned char)*p);
        if (lo >= 0) { val = val * 16 + lo; p++; }
        // A byte is one or two hex digits; a third means this isn't a byte dump.
        if (hexval((unsigned char)*p) >= 0) return 0;
        b[nb++] = (unsigned char)val;
    }
    if (nb < 6) return 0;

    // Little-endian int16 per axis (low byte first), as the firmware packs it.
    out->kind = ADCS_MAG_NT;
    out->x_nT = (long)(int16_t)((b[1] << 8) | b[0]) * nt_per_lsb;
    out->y_nT = (long)(int16_t)((b[3] << 8) | b[2]) * nt_per_lsb;
    out->z_nT = (long)(int16_t)((b[5] << 8) | b[4]) * nt_per_lsb;
    return 1;
}

int adcs_generic_telem_frame(const char *command)
{
    if (command == NULL) return -1;
    const char *p = strstr(command, "adcs_generic_telemetry_request");
    if (p == NULL) return -1;
    const char *lp = strchr(p, '(');
    if (lp == NULL) return -1;   // bare command name (no argument list)
    char *end = NULL;
    long id = strtol(lp + 1, &end, 10);   // strtol skips leading spaces
    if (end == lp + 1) return -1;
    return (int)id;
}

int adcs_tle_epoch_unix_ms(int epoch_year, double epoch_day, int64_t *out_ms)
{
    // TLEs use a 2-digit year; the line parser may already have widened it.
    // Anything below 57 is 21st century (NORAD's own pivot), 57..99 is 1957+.
    if (epoch_year < 57)        epoch_year += 2000;
    else if (epoch_year < 100)  epoch_year += 1900;
    if (epoch_year < 1957 || epoch_year > 2200) return -1;
    if (epoch_day < 1.0 || epoch_day >= 367.0)  return -1;

    // Seconds from the Unix epoch to Jan 1 00:00:00Z of epoch_year, then add
    // the fractional day-of-year (day 1.0 == Jan 1 00:00:00Z, hence -1.0).
    struct tm jan1 = {0};
    jan1.tm_year = epoch_year - 1900;
    jan1.tm_mon  = 0;
    jan1.tm_mday = 1;
    time_t base = timegm(&jan1);
    if (base == (time_t)-1) return -1;

    double secs = (double)base + (epoch_day - 1.0) * 86400.0;
    *out_ms = (int64_t)llround(secs * 1000.0);
    return 0;
}

int adcs_closest_index(const int64_t *epochs_ms, int n, int64_t target_ms)
{
    if (n <= 0) return -1;
    int best = 0;
    int64_t best_gap = llabs(epochs_ms[0] - target_ms);
    for (int i = 1; i < n; ++i) {
        int64_t gap = llabs(epochs_ms[i] - target_ms);
        if (gap < best_gap) { best_gap = gap; best = i; }
    }
    return best;
}
