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
