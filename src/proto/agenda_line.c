/*

    Simple Satellite Operations  agenda_line.c

    Inline-comment splitting for telecommand agenda lines. See
    agenda_line.h for the contract.

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

#include "agenda_line.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

const char *agenda_find_inline_comment(const char *s, size_t *cmd_len)
{
    for (size_t i = 0; s[i] != '\0'; ++i) {
        if (s[i] == '#' && (i == 0 || s[i - 1] == ' ' || s[i - 1] == '\t')) {
            // Back up over the whitespace run that precedes the '#' so the
            // command length excludes it.
            size_t j = i;
            while (j > 0 && (s[j - 1] == ' ' || s[j - 1] == '\t')) --j;
            if (cmd_len) *cmd_len = j;
            return s + i;
        }
    }
    if (cmd_len) *cmd_len = strlen(s);
    return NULL;
}

int agenda_parse_directive_ms(const char *line, const char *key,
                              long long *out)
{
    if (line == NULL || key == NULL || out == NULL) return 0;
    size_t klen = strlen(key);
    for (const char *p = line; *p != '\0'; ++p) {
        if (strncmp(p, key, klen) != 0) continue;
        const char *digits = p + klen;
        const char *d = digits;
        if (*d == '+' || *d == '-') ++d;
        const char *digit_start = d;
        while (isdigit((unsigned char) *d)) ++d;
        if (d == digit_start) continue;
        char numbuf[32];
        size_t n = (size_t)(d - digits);
        if (n >= sizeof numbuf) continue;
        memcpy(numbuf, digits, n);
        numbuf[n] = '\0';
        *out = strtoll(numbuf, NULL, 10);
        return 1;
    }
    return 0;
}
