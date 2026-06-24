/*

    Simple Satellite Operations  src/ipc/sso_ipc_paths.c

    Filesystem paths for the sso_ipc runtime: the per-user runtime directory
    and the Unix-domain socket path a tool listens on or connects to.

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

#include "sso_ipc_paths.h"

#include "sso_paths.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define SSO_IPC_BUF 256

static char g_runtime_dir[SSO_IPC_BUF];
static int g_runtime_resolved = 0;

static int is_dir(const char *p) {
    struct stat st;
    if (stat(p, &st) != 0) return 0;
    return S_ISDIR(st.st_mode);
}

const char *sso_ipc_runtime_dir(void) {
    if (g_runtime_resolved) return g_runtime_dir;
    const char *env = getenv("SSO_RUNTIME_DIR");
    if (env && env[0]) {
        snprintf(g_runtime_dir, sizeof(g_runtime_dir), "%s", env);
        g_runtime_resolved = 1;
        return g_runtime_dir;
    }
    if (is_dir("/run/sso")) {
        snprintf(g_runtime_dir, sizeof(g_runtime_dir), "/run/sso");
        g_runtime_resolved = 1;
        return g_runtime_dir;
    }
    const char *xdg = getenv("XDG_RUNTIME_DIR");
    if (xdg && xdg[0]) {
        snprintf(g_runtime_dir, sizeof(g_runtime_dir), "%s/sso", xdg);
    } else {
        snprintf(g_runtime_dir, sizeof(g_runtime_dir), "/tmp/sso-%d",
                 (int) geteuid());
    }
    g_runtime_resolved = 1;
    return g_runtime_dir;
}

int sso_ipc_socket_path(char *out, size_t out_size, const char *tool) {
    if (!out || out_size == 0 || !tool || !tool[0]) return -1;
    int n = snprintf(out, out_size, "%s/%s.sock",
                     sso_ipc_runtime_dir(), tool);
    return (n < 0 || (size_t) n >= out_size) ? -1 : 0;
}

int sso_ipc_pid_path(char *out, size_t out_size, const char *tool) {
    if (!out || out_size == 0 || !tool || !tool[0]) return -1;
    int n = snprintf(out, out_size, "%s/%s.pid",
                     sso_ipc_runtime_dir(), tool);
    return (n < 0 || (size_t) n >= out_size) ? -1 : 0;
}

int sso_ipc_ensure_runtime_dir(void) {
    const char *dir = sso_ipc_runtime_dir();
    return sso_mkdir_p(dir);
}
