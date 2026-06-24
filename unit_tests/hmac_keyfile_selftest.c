/*

    Simple Satellite Operations  unit_tests/hmac_keyfile_selftest.c

    Coverage for src/proto/hmac_keyfile.c. This is the loader that reads
    the FrontierSat HMAC key off disk and hands the bytes to every
    uplink burst (cli_args.c -> state->tx.hmac_key -> tx_burst -> ax100).
    Its whole reason for existing is to refuse a key the operating
    system would let the wrong people read, so the headline coverage
    here is the permission-mode matrix: exactly 0600 or 0640 is
    accepted, everything else (any world bit, any group-write bit, even
    an over-restrictive 0400) is rejected. A regression that loosened
    that check would silently weaken uplink authentication.

    What's covered:
      Permission gate (hmac_keyfile_load):
        - mode 0600 (personal) accepted, bytes decoded correctly.
        - mode 0640 (shared, group-readable) accepted.
        - mode 0644 / 0604 (any world bit) rejected.
        - mode 0660 / 0666 (group-write / world-write) rejected.
        - mode 0700 (owner-exec) and 0400 (read-only) rejected: the
          contract is exactly 0600 or 0640, not "no worse than".
        - a directory (not a regular file) rejected.
        - a path that doesn't exist rejected.
      Parser (hmac_keyfile_load):
        - valid uppercase hex decodes to the expected bytes.
        - a single trailing newline is stripped.
        - empty file / newline-only file rejected.
        - odd hex-char count rejected.
        - lowercase hex rejected (the key is uppercase-only by spec).
        - a non-hex char and an embedded space rejected.
        - a key that decodes to more bytes than out_cap rejected.
        - a file larger than the internal read buffer rejected.
        - a long (32-byte) key round-trips.
        - NULL path, NULL out, and zero out_cap rejected.
      Default path resolver (hmac_keyfile_default_path):
        - with the shared path absent, falls back to
          "$HOME/" HMAC_KEYFILE_USER_RELPATH.
        - with the shared path absent and $HOME unset, fails.

    Exit status: 0 = all tests passed, non-zero = failure.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include "hmac_keyfile.h"
#include "tap.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// Create a fresh, unique temp file and return its path in `out`. The
// file is left existing with mkstemp's default 0600; callers chmod it
// to the mode under test via write_keyfile.
static int make_tmp_path(char *out, size_t cap)
{
    snprintf(out, cap, "/tmp/hmac_keyfile_selftest_XXXXXX");
    int fd = mkstemp(out);
    if (fd < 0) return -1;
    close(fd);
    return 0;
}

// Write `text` to `path` (truncating) and chmod it to `mode` exactly.
// chmod is absolute, so the resulting mode bits don't depend on umask.
static int write_keyfile(const char *path, const char *text, mode_t mode)
{
    FILE *f = fopen(path, "wb");
    if (f == NULL) return -1;
    size_t len = strlen(text);
    if (len > 0 && fwrite(text, 1, len, f) != len) {
        fclose(f);
        return -1;
    }
    if (fclose(f) != 0) return -1;
    return chmod(path, mode);
}

// Each permission case writes the same valid 4-byte key, then asserts
// load() accepts (returns 4) or rejects (returns -1) purely on the mode.
static void test_permission_gate(void)
{
    char path[64];
    if (make_tmp_path(path, sizeof path) != 0) {
        tap_bail("could not create temp keyfile");
        return;
    }
    uint8_t out[64];
    const char *key = "DEADBEEF";  // -> DE AD BE EF
    const uint8_t want[4] = { 0xDE, 0xAD, 0xBE, 0xEF };

    write_keyfile(path, key, 0600);
    ssize_t n = hmac_keyfile_load(path, out, sizeof out);
    tap_okf(n == 4, "mode 0600 accepted (got %zd, want 4)", n);
    tap_ok(n == 4 && memcmp(out, want, 4) == 0, "mode 0600 decodes correct bytes");

    write_keyfile(path, key, 0640);
    n = hmac_keyfile_load(path, out, sizeof out);
    tap_okf(n == 4, "mode 0640 accepted (got %zd, want 4)", n);

    write_keyfile(path, key, 0644);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "mode 0644 rejected (world-read)");

    write_keyfile(path, key, 0604);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "mode 0604 rejected (world-read, no group)");

    write_keyfile(path, key, 0660);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "mode 0660 rejected (group-write)");

    write_keyfile(path, key, 0666);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "mode 0666 rejected (world-write)");

    write_keyfile(path, key, 0700);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "mode 0700 rejected (owner-exec, not exactly 0600/0640)");

    write_keyfile(path, key, 0400);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "mode 0400 rejected (over-restrictive, not exactly 0600/0640)");

    unlink(path);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "non-existent path rejected");

    char dir[64];
    snprintf(dir, sizeof dir, "/tmp/hmac_keyfile_selftest_dir_XXXXXX");
    if (mkdtemp(dir) != NULL) {
        tap_ok(hmac_keyfile_load(dir, out, sizeof out) == -1,
               "directory rejected (not a regular file)");
        rmdir(dir);
    } else {
        tap_diag("mkdtemp failed; skipping not-a-regular-file case");
    }
}

static void test_parser(void)
{
    char path[64];
    if (make_tmp_path(path, sizeof path) != 0) {
        tap_bail("could not create temp keyfile");
        return;
    }
    uint8_t out[64];
    const uint8_t want[4] = { 0xDE, 0xAD, 0xBE, 0xEF };

    write_keyfile(path, "DEADBEEF", 0600);
    ssize_t n = hmac_keyfile_load(path, out, sizeof out);
    tap_ok(n == 4 && memcmp(out, want, 4) == 0, "valid uppercase hex decodes");

    write_keyfile(path, "DEADBEEF\n", 0600);
    n = hmac_keyfile_load(path, out, sizeof out);
    tap_ok(n == 4 && memcmp(out, want, 4) == 0, "trailing newline stripped");

    write_keyfile(path, "", 0600);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1, "empty file rejected");

    write_keyfile(path, "\n", 0600);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "newline-only file rejected (empty after strip)");

    write_keyfile(path, "ABC", 0600);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "odd hex-char count rejected");

    write_keyfile(path, "deadbeef", 0600);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "lowercase hex rejected (key is uppercase-only)");

    write_keyfile(path, "DEADBEEG", 0600);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "non-hex char rejected");

    write_keyfile(path, "DE AD", 0600);
    tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
           "embedded space rejected");

    // 20 hex chars -> 10 bytes, but the output buffer holds only 4.
    write_keyfile(path, "00112233445566778899", 0600);
    tap_ok(hmac_keyfile_load(path, out, 4) == -1,
           "decoded length over out_cap rejected");

    // A file larger than the loader's internal 1023-byte read window.
    {
        char big[2048];
        memset(big, 'A', sizeof big - 1);  // valid uppercase hex, but huge
        big[sizeof big - 1] = '\0';
        write_keyfile(path, big, 0600);
        tap_ok(hmac_keyfile_load(path, out, sizeof out) == -1,
               "file larger than read buffer rejected");
    }

    // A full 32-byte key round-trips (64 uppercase hex chars).
    write_keyfile(path,
        "000102030405060708090A0B0C0D0E0F"
        "101112131415161718191A1B1C1D1E1F", 0600);
    n = hmac_keyfile_load(path, out, sizeof out);
    tap_okf(n == 32, "32-byte key returns 32 (got %zd)", n);
    tap_ok(n == 32 && out[0] == 0x00 && out[31] == 0x1F,
           "32-byte key decodes first/last byte");

    unlink(path);
}

static void test_null_guards(void)
{
    uint8_t out[64];
    tap_ok(hmac_keyfile_load(NULL, out, sizeof out) == -1, "NULL path rejected");
    tap_ok(hmac_keyfile_load("/tmp/whatever", NULL, sizeof out) == -1,
           "NULL out rejected");
    tap_ok(hmac_keyfile_load("/tmp/whatever", out, 0) == -1, "zero out_cap rejected");
}

static void test_default_path(void)
{
    // The resolver prefers HMAC_KEYFILE_SHARED_PATH and only falls back
    // to $HOME when that absolute path isn't a regular file. On a normal
    // dev/test host the shared path is absent, so we can exercise the
    // fallback; if it happens to exist, just assert it wins.
    struct stat st;
    int shared_present =
        (stat(HMAC_KEYFILE_SHARED_PATH, &st) == 0 && S_ISREG(st.st_mode));

    // Save HOME so the rest of the process (and any restart) is unaffected.
    char saved_home[1024];
    int had_home = 0;
    const char *h = getenv("HOME");
    if (h != NULL) {
        snprintf(saved_home, sizeof saved_home, "%s", h);
        had_home = 1;
    }

    char got[1024];
    setenv("HOME", "/tmp/hmac_home_probe", 1);
    int rc = hmac_keyfile_default_path(got, sizeof got);
    if (shared_present) {
        tap_ok(rc == 0 && strcmp(got, HMAC_KEYFILE_SHARED_PATH) == 0,
               "default path = shared path when present");
    } else {
        char want[1024];
        snprintf(want, sizeof want, "/tmp/hmac_home_probe/%s",
                 HMAC_KEYFILE_USER_RELPATH);
        tap_ok(rc == 0 && strcmp(got, want) == 0,
               "default path falls back to $HOME/" HMAC_KEYFILE_USER_RELPATH);
    }

    unsetenv("HOME");
    rc = hmac_keyfile_default_path(got, sizeof got);
    if (shared_present) {
        tap_ok(rc == 0, "shared path resolves even with $HOME unset");
    } else {
        tap_ok(rc == -1, "no shared path + $HOME unset fails");
    }

    if (had_home) setenv("HOME", saved_home, 1);
}

int main(void)
{
    // The loader prints a diagnostic to stderr on every rejection, and
    // most cases here are deliberate rejections. Send that expected
    // noise to /dev/null so the TAP stream on stdout stays clean.
    freopen("/dev/null", "w", stderr);

    test_permission_gate();
    test_parser();
    test_null_guards();
    test_default_path();
    return tap_done();
}
