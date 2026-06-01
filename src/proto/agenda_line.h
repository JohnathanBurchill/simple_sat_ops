/*

    Simple Satellite Operations  agenda_line.h

    Shared parsing for a single line of a telecommand agenda file. An
    agenda is one telecommand per line; '#' begins a comment. A whole-line
    comment (the first non-blank character is '#') is each caller's own
    concern, but the rule for an INLINE trailing comment -- "CTS1+foo()!  #
    note" -- lives here so simple_sat_ops (which transmits) and agenda_check
    (which audits) agree on exactly what is command and what is comment.

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

#ifndef AGENDA_LINE_H
#define AGENDA_LINE_H

#include <stddef.h>

// Find an inline trailing comment in an agenda command line.
//
// To keep a wrong telecommand from ever going on the air, a '#' is treated
// as a comment delimiter ONLY when it is preceded by whitespace (space or
// tab) or sits at index 0 -- a '#' embedded in the command text (no space
// before it) is left intact. Telecommands always end with '!', so a real
// inline comment is always separated from the command by whitespace.
//
// Returns a pointer to the '#' that starts the comment, or NULL if the
// line carries no inline comment. *cmd_len (when non-NULL) receives the
// length of the command with the run of whitespace before the '#' trimmed
// off, so the command is s[0 .. *cmd_len) and the comment is the returned
// pointer onward. When no comment is found, *cmd_len is set to strlen(s).
const char *agenda_find_inline_comment(const char *s, size_t *cmd_len);

// Find the first @<key>=<digits> directive in `line` and return its
// integer value. `key` includes the trailing '=' (e.g. "@tssent=" or
// "@tsexec="). A leading +/- is accepted. These directives carry unix-ms
// timestamps; @tssent in particular is the value the satellite echoes
// back in its tcmd_response, so this is how a transmitted command is tied
// to its response. Returns 1 and writes *out on success, 0 if the key is
// absent or not followed by digits (*out untouched).
int agenda_parse_directive_ms(const char *line, const char *key,
                              long long *out);

#endif
