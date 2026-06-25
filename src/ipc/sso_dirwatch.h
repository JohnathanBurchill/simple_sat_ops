/*

    Simple Satellite Operations  src/ipc/sso_dirwatch.h

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

// sso_dirwatch.h — watch a directory for entries appearing / disappearing.
//
// Used by --viewer-stream to notice the instant a --control operator binds
// its socket in the runtime dir (sso_ipc_runtime_dir()), so the stream can
// connect and relay without waiting out its periodic probe. Backed by
// inotify on Linux and kqueue on macOS/BSD; on any other host
// sso_dirwatch_open() returns NULL and the caller falls back to polling.

#ifndef SSO_DIRWATCH_H
#define SSO_DIRWATCH_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sso_dirwatch sso_dirwatch_t;

// Begin watching `dir` for directory-entry changes (a file created,
// deleted, or renamed). Returns a handle, or NULL if `dir` can't be watched
// or the host has no supported backend — the caller should treat NULL as
// "no watch" and keep polling. The directory must already exist.
sso_dirwatch_t *sso_dirwatch_open(const char *dir);

// Block until the watched directory changes or `timeout_ms` elapses (a
// negative timeout blocks indefinitely). Returns 1 if a change was
// observed, 0 on timeout, -1 on error. Spurious wakeups are allowed, so a 1
// only means "re-check" — the caller confirms by trying to connect.
int sso_dirwatch_wait(sso_dirwatch_t *w, int timeout_ms);

// Release the watch. Safe on NULL.
void sso_dirwatch_close(sso_dirwatch_t *w);

#ifdef __cplusplus
}
#endif

#endif
