/*

    Simple Satellite Operations  hmac_keyfile.h

    Strict loader for the FrontierSat HMAC key file. Expected format:
    a single line of plain uppercase hexadecimal characters, no spaces,
    no separators, optionally followed by a single newline. The file
    must be chmod 0600 (personal) or 0640 (shared with the sso-ops
    group); any world bits are rejected.

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

// Shared keyfile path on the ground machine. Sysadmin chowns it
// root:sso-ops and chmods 0640 so every operator account can read but
// nobody else can. Per-user fallback (HMAC_KEYFILE_USER_RELPATH under
// $HOME) is used when the shared file is absent — useful on dev hosts.
#define HMAC_KEYFILE_SHARED_PATH      "/FrontierSat/HMAC/frontiersat_hmac"
#define HMAC_KEYFILE_USER_RELPATH     ".local/state/simple_sat_ops/frontiersat_hmac"

// Back-compat alias: existing tool help text (uplink_test, rx_decode,
// rx_replay, tx_frame_sdr) prints this. Now resolves to the per-user
// fallback path, since the primary default is HMAC_KEYFILE_SHARED_PATH.
#define HMAC_KEYFILE_DEFAULT_RELPATH  HMAC_KEYFILE_USER_RELPATH

// Loads the HMAC key from `path`. On success writes the decoded bytes
// to `out` and returns the number of bytes written. On error prints a
// diagnostic to stderr and returns -1.
//
// Enforces: regular file, mode 0600 or 0640 (any world bits → reject),
// non-empty, even-length, uppercase-hex-only.
ssize_t hmac_keyfile_load(const char *path, uint8_t *out, size_t out_cap);

// Resolves the keyfile path to use when the operator hasn't passed
// --keyfile=. Prefers the shared HMAC_KEYFILE_SHARED_PATH; falls back
// to "$HOME/" + HMAC_KEYFILE_USER_RELPATH if the shared file isn't
// readable. Returns 0 on success, -1 if neither location resolves.
int hmac_keyfile_default_path(char *out_path, size_t out_cap);

// Repairs `path` so hmac_keyfile_load will accept it. First strips any
// POSIX ACL entries with `setfacl -b` (an ACL can grant access the
// plain mode bits don't reveal, so removing it is part of the canonical
// keyfile setup), then chmods the file to a valid mode: 0640 for the
// shared keyfile (group sso-ops reads it), 0600 for a personal one.
// Reports what it did to stderr. Returns 0 if the file ends up at a
// valid mode, -1 on error. setfacl is Linux-only; on a host without it
// the ACL strip is skipped (with a warning) and the chmod still runs.
int hmac_keyfile_fix_permissions(const char *path);

#endif // HMAC_KEYFILE_H
