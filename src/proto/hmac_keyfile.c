/*

    Simple Satellite Operations  hmac_keyfile.c

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

#include "hmac_keyfile.h"

#include <errno.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>

extern char **environ;

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int hmac_keyfile_default_path(char *out_path, size_t out_cap)
{
    // Prefer the shared keyfile under /FrontierSat/HMAC. If it doesn't
    // exist or isn't a regular file, fall back to the per-user path
    // under $HOME. The actual mode/group-readability check happens in
    // hmac_keyfile_load — this resolver is purely "which path?".
    struct stat st;
    if (stat(HMAC_KEYFILE_SHARED_PATH, &st) == 0 && S_ISREG(st.st_mode)) {
        int n = snprintf(out_path, out_cap, "%s", HMAC_KEYFILE_SHARED_PATH);
        if (n < 0 || (size_t) n >= out_cap) return -1;
        return 0;
    }
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') {
        fprintf(stderr,
            "hmac_keyfile: %s not found and $HOME unset; pass --keyfile=\n",
            HMAC_KEYFILE_SHARED_PATH);
        return -1;
    }
    int n = snprintf(out_path, out_cap, "%s/%s", home,
                     HMAC_KEYFILE_USER_RELPATH);
    if (n < 0 || (size_t) n >= out_cap) return -1;
    return 0;
}

ssize_t hmac_keyfile_load(const char *path, uint8_t *out, size_t out_cap)
{
    if (path == NULL || out == NULL || out_cap == 0) {
        return -1;
    }

    struct stat st;
    if (stat(path, &st) != 0) {
        fprintf(stderr, "hmac_keyfile: stat(%s): %s\n", path, strerror(errno));
        return -1;
    }
    if (!S_ISREG(st.st_mode)) {
        fprintf(stderr, "hmac_keyfile: %s is not a regular file\n", path);
        return -1;
    }
    // 0600 (personal) or 0640 (shared, group-readable). Reject any
    // world bits — even world-read is a no-go for an HMAC key.
    unsigned mode = st.st_mode & 0777;
    if (mode != 0600 && mode != 0640) {
        fprintf(stderr,
                "hmac_keyfile: %s must be chmod 0600 or 0640 (got 0%03o); "
                "run: chmod 640 %s   (or 600 for personal)\n",
                path, mode, path);
        return -1;
    }

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "hmac_keyfile: open(%s): %s\n", path, strerror(errno));
        return -1;
    }

    // Read into a fixed local buffer. HMAC keys are small; 1 KiB of hex
    // is overkill but harmless.
    char buf[1024];
    size_t n_read = fread(buf, 1, sizeof(buf) - 1, f);
    int read_err = ferror(f);
    int at_eof = feof(f);
    fclose(f);

    if (read_err) {
        fprintf(stderr, "hmac_keyfile: read error on %s\n", path);
        return -1;
    }
    if (!at_eof) {
        fprintf(stderr, "hmac_keyfile: %s exceeds %zu bytes (not a key)\n",
                path, sizeof(buf) - 1);
        return -1;
    }
    buf[n_read] = '\0';

    if (n_read > 0 && buf[n_read - 1] == '\n') {
        n_read--;
        buf[n_read] = '\0';
    }

    if (n_read == 0) {
        fprintf(stderr, "hmac_keyfile: %s is empty\n", path);
        return -1;
    }
    if ((n_read & 1u) != 0) {
        fprintf(stderr,
                "hmac_keyfile: %s has %zu hex chars (must be even)\n",
                path, n_read);
        return -1;
    }

    size_t n_bytes = n_read / 2;
    if (n_bytes > out_cap) {
        fprintf(stderr,
                "hmac_keyfile: %s decodes to %zu bytes (buffer holds %zu)\n",
                path, n_bytes, out_cap);
        return -1;
    }

    for (size_t i = 0; i < n_read; ++i) {
        int v = hex_value(buf[i]);
        if (v < 0) {
            fprintf(stderr,
                    "hmac_keyfile: %s has a non-hex or lowercase char 0x%02X "
                    "at offset %zu (the key must be uppercase 0-9 A-F)\n",
                    path, (unsigned char)buf[i], i);
            return -1;
        }
    }

    for (size_t i = 0; i < n_bytes; ++i) {
        int hi = hex_value(buf[2 * i]);
        int lo = hex_value(buf[2 * i + 1]);
        out[i] = (uint8_t)((hi << 4) | lo);
    }

    return (ssize_t)n_bytes;
}

// Runs `setfacl -b <path>` to strip every extended ACL entry. Spawned
// directly (no shell) so a path with spaces or shell metacharacters is
// safe. Returns 0 if setfacl ran and exited 0; -1 if it couldn't be
// spawned (e.g. macOS, which has no setfacl) or exited non-zero.
static int run_setfacl_b(const char *path)
{
    char *const argv[] = { "setfacl", "-b", (char *)path, NULL };
    pid_t pid;
    if (posix_spawnp(&pid, "setfacl", NULL, NULL, argv, environ) != 0) {
        return -1;
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        return -1;
    }
    return (WIFEXITED(status) && WEXITSTATUS(status) == 0) ? 0 : -1;
}

int hmac_keyfile_fix_permissions(const char *path)
{
    if (path == NULL) {
        return -1;
    }

    struct stat st;
    if (stat(path, &st) != 0) {
        fprintf(stderr, "hmac_keyfile: stat(%s): %s\n", path, strerror(errno));
        return -1;
    }
    if (!S_ISREG(st.st_mode)) {
        fprintf(stderr,
                "hmac_keyfile: %s is not a regular file; refusing to fix\n",
                path);
        return -1;
    }

    // The shared keyfile is group-readable (sso-ops); a personal one is
    // owner-only. Either is a valid target for hmac_keyfile_load.
    unsigned target = (strcmp(path, HMAC_KEYFILE_SHARED_PATH) == 0) ? 0640u
                                                                    : 0600u;

    // Strip ACLs first: with an extended ACL present the group mode bits
    // show the ACL mask, not the real group entry, so the mode check can
    // be fooled. Removing the ACL makes the plain mode bits govern again.
    // Run it unconditionally (idempotent) even if the mode already looks
    // valid. setfacl is Linux-only; warn and carry on where it's absent.
    if (run_setfacl_b(path) != 0) {
        fprintf(stderr,
                "hmac_keyfile: warning: `setfacl -b %s` did not run "
                "(setfacl not installed?); skipping ACL strip, "
                "continuing to chmod\n",
                path);
    }

    if (chmod(path, target) != 0) {
        fprintf(stderr, "hmac_keyfile: chmod 0%03o %s: %s\n",
                target, path, strerror(errno));
        return -1;
    }

    // Confirm the file now passes the same gate hmac_keyfile_load applies.
    if (stat(path, &st) != 0) {
        fprintf(stderr, "hmac_keyfile: re-stat(%s): %s\n", path, strerror(errno));
        return -1;
    }
    unsigned mode = st.st_mode & 0777;
    if (mode != 0600 && mode != 0640) {
        fprintf(stderr,
                "hmac_keyfile: %s still at 0%03o after fix (an ACL or "
                "mount option may be overriding the mode)\n",
                path, mode);
        return -1;
    }

    fprintf(stderr, "hmac_keyfile: %s permissions set to 0%03o\n", path, mode);
    return 0;
}
