/*

    Simple Satellite Operations  src/ipc/sso_paths.c

    Resolves the FrontierSat data root ($FRONTIERSAT_ROOT or /FrontierSat) and
    builds the standard subdirectory paths the tools write into.

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

#include "sso_paths.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define SSO_PATHS_BUF 512

static char g_root[SSO_PATHS_BUF];
static int g_root_resolved = 0;

const char *sso_frontiersat_root(void) {
    if (g_root_resolved) return g_root;
    const char *env = getenv("FRONTIERSAT_ROOT");
    if (env && env[0]) {
        snprintf(g_root, sizeof(g_root), "%s", env);
    } else {
        // No override: always the well-known shared tree. (We used to
        // probe for /FrontierSat and otherwise fall back to
        // $HOME/FrontierSat; now every host without FRONTIERSAT_ROOT
        // resolves to the same /FrontierSat, matching how the ground
        // machine is set up. Point a dev host elsewhere with
        // FRONTIERSAT_ROOT.)
        snprintf(g_root, sizeof(g_root), "/FrontierSat");
        // If that default root is absent, say so once with the options,
        // so files don't quietly fail to land where the user expects.
        struct stat st;
        if (stat(g_root, &st) != 0 || !S_ISDIR(st.st_mode)) {
            fprintf(stderr,
                "simple_sat_ops: data root %s does not exist. Options:\n"
                "       - create it:  sudo mkdir -p %s && sudo chown \"$(whoami)\" %s\n"
                "       - or point elsewhere:  export FRONTIERSAT_ROOT=/path/to/your/tree\n",
                g_root, g_root, g_root);
        }
    }
    g_root_resolved = 1;
    return g_root;
}

int sso_frontiersat_subpath(char *out, size_t out_size, const char *subdir) {
    if (!out || out_size == 0) return -1;
    const char *root = sso_frontiersat_root();
    int n;
    if (!subdir || !subdir[0]) {
        n = snprintf(out, out_size, "%s", root);
    } else {
        n = snprintf(out, out_size, "%s/%s", root, subdir);
    }
    return (n < 0 || (size_t) n >= out_size) ? -1 : 0;
}

static const char *static_subpath(const char *subdir) {
    static char buf[SSO_PATHS_BUF];
    if (sso_frontiersat_subpath(buf, sizeof(buf), subdir) != 0) {
        // Overflow shouldn't happen for these short fixed names; return
        // the root alone as a safe fallback so callers don't crash.
        snprintf(buf, sizeof(buf), "%s", sso_frontiersat_root());
    }
    return buf;
}

const char *sso_tles_dir(void)               { return static_subpath("TLEs"); }
const char *sso_operations_dir(void)         { return static_subpath("Operations"); }
const char *sso_operations_current_symlink(void) {
    return static_subpath("Operations/current");
}
const char *sso_satnogs_archive_dir(void)    { return static_subpath("satnogs_archive"); }
const char *sso_captures_dir(void)           { return static_subpath("captures"); }
const char *sso_packet_db_path(void)         { return static_subpath("packet_db.sqlite"); }
const char *sso_testing_dir(void)            { return static_subpath("Testing"); }

int sso_mkdir_p(const char *path) {
    if (!path || !path[0]) {
        errno = EINVAL;
        return -1;
    }
    char tmp[SSO_PATHS_BUF];
    size_t len = strlen(path);
    if (len >= sizeof(tmp)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    memcpy(tmp, path, len + 1);
    if (tmp[len - 1] == '/') tmp[len - 1] = '\0';
    for (char *p = tmp + 1; *p; ++p) {
        if (*p != '/') continue;
        *p = '\0';
        if (mkdir(tmp, 0775) != 0 && errno != EEXIST) return -1;
        *p = '/';
    }
    if (mkdir(tmp, 0775) != 0 && errno != EEXIST) return -1;
    return 0;
}

int sso_mkdir_p_for_file(const char *path) {
    if (!path || !path[0]) {
        errno = EINVAL;
        return -1;
    }
    char tmp[SSO_PATHS_BUF];
    size_t len = strlen(path);
    if (len >= sizeof(tmp)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    memcpy(tmp, path, len + 1);
    char *slash = strrchr(tmp, '/');
    if (!slash || slash == tmp) return 0;
    *slash = '\0';
    return sso_mkdir_p(tmp);
}
