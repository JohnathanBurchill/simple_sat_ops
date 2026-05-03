/*

    Simple Satellite Operations  hmac_keyfile.h

    Strict loader for the FrontierSat HMAC key file. Expected format:
    a single line of plain uppercase hexadecimal characters, no spaces,
    no separators, optionally followed by a single newline. The file must
    be chmod 0600 (owner read/write only).

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

#ifndef HMAC_KEYFILE_H
#define HMAC_KEYFILE_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

// Default location under $HOME.
#define HMAC_KEYFILE_DEFAULT_RELPATH ".local/state/simple_sat_ops/frontiersat_hmac"

// Loads the HMAC key from `path`. On success writes the decoded bytes
// to `out` and returns the number of bytes written. On error prints a
// diagnostic to stderr and returns -1.
//
// Enforces: regular file, mode 0600 exactly (no group/other bits),
// non-empty, even-length, uppercase-hex-only.
ssize_t hmac_keyfile_load(const char *path, uint8_t *out, size_t out_cap);

// Fills out_path with "$HOME/" + HMAC_KEYFILE_DEFAULT_RELPATH. Returns 0
// on success, -1 if $HOME is unset or the buffer is too small.
int hmac_keyfile_default_path(char *out_path, size_t out_cap);

#endif // HMAC_KEYFILE_H
