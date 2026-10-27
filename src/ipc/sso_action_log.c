/*

    Simple Satellite Operations  src/ipc/sso_action_log.c

    Per-pass plain-text action log: one timestamped line per operator
    action, rotator command, or tracking state change, written to
    <pass_folder>/actions.log.

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

#include "sso_action_log.h"

#include "sso_time.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define ACTION_LOG_LINE   512
// Startup lines held until the pass folder exists. Bring-up logs a
// handful; anything past this is dropped oldest-first and noted.
#define ACTION_LOG_PENDING 64

static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;
static int  g_fd = -1;
static char g_dir[512];

static char g_pending[ACTION_LOG_PENDING][ACTION_LOG_LINE];
static int  g_pending_head  = 0;   // oldest held line
static int  g_pending_count = 0;
static int  g_pending_dropped = 0;

// Write the whole line or give up; a short write on a local file only
// happens on a full disk, where there is nothing better to do.
static void write_line(int fd, const char *line)
{
    size_t len = strlen(line);
    while (len > 0) {
        ssize_t n = write(fd, line, len);
        if (n <= 0) return;
        line += n;
        len  -= (size_t) n;
    }
}

static void pending_push(const char *line)
{
    int slot;
    if (g_pending_count == ACTION_LOG_PENDING) {
        slot = g_pending_head;
        g_pending_head = (g_pending_head + 1) % ACTION_LOG_PENDING;
        g_pending_dropped++;
    } else {
        slot = (g_pending_head + g_pending_count) % ACTION_LOG_PENDING;
        g_pending_count++;
    }
    snprintf(g_pending[slot], ACTION_LOG_LINE, "%s", line);
}

static void pending_flush_locked(void)
{
    if (g_pending_dropped > 0) {
        char note[96];
        snprintf(note, sizeof note,
                 "(%d earlier startup lines dropped)\n", g_pending_dropped);
        write_line(g_fd, note);
    }
    for (int i = 0; i < g_pending_count; ++i) {
        write_line(g_fd, g_pending[(g_pending_head + i) % ACTION_LOG_PENDING]);
    }
    g_pending_head = g_pending_count = g_pending_dropped = 0;
}

int sso_action_log_set_dir(const char *dir)
{
    if (dir == NULL || dir[0] == '\0') return -1;
    pthread_mutex_lock(&g_mu);
    if (g_fd >= 0 && strcmp(dir, g_dir) == 0) {
        pthread_mutex_unlock(&g_mu);
        return 0;
    }
    char path[600];
    snprintf(path, sizeof path, "%.500s/actions.log", dir);
    int fd = open(path, O_WRONLY | O_APPEND | O_CREAT | O_CLOEXEC, 0644);
    if (fd < 0) {
        pthread_mutex_unlock(&g_mu);
        return -1;
    }
    if (g_fd >= 0) close(g_fd);
    g_fd = fd;
    snprintf(g_dir, sizeof g_dir, "%s", dir);
    pending_flush_locked();
    pthread_mutex_unlock(&g_mu);
    return 0;
}

void sso_action_log(const char *cat, const char *fmt, ...)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    char iso[40];
    sso_iso_utc_from_ts(&ts, iso, sizeof iso);

    char msg[ACTION_LOG_LINE];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    for (char *p = msg; *p; ++p) {
        if (*p == '\n' || *p == '\r') *p = ' ';
    }

    char tag[16];
    snprintf(tag, sizeof tag, "[%.10s]", cat ? cat : "");
    char line[ACTION_LOG_LINE];
    int n = snprintf(line, sizeof line, "%s %-7s %s\n", iso, tag, msg);
    if (n >= (int) sizeof line) {
        // Truncated: keep the line terminated.
        line[sizeof line - 2] = '\n';
    }

    pthread_mutex_lock(&g_mu);
    if (g_fd >= 0) {
        write_line(g_fd, line);
    } else {
        pending_push(line);
    }
    pthread_mutex_unlock(&g_mu);
}

void sso_action_log_close(void)
{
    pthread_mutex_lock(&g_mu);
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    g_dir[0] = '\0';
    pthread_mutex_unlock(&g_mu);
}
