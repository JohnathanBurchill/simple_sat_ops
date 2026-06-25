/*

    Simple Satellite Operations  src/ipc/sso_dirwatch.c

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

// sso_dirwatch.c — directory-change watch (inotify on Linux, kqueue on
// macOS/BSD). See sso_dirwatch.h for the contract. Each backend defines its
// own struct sso_dirwatch; a third stub branch keeps the build linking on
// hosts with neither facility (open returns NULL -> caller polls).

#include "sso_dirwatch.h"

#include <errno.h>
#include <stdlib.h>

#if defined(__linux__)

#include <poll.h>
#include <sys/inotify.h>
#include <unistd.h>

struct sso_dirwatch {
    int fd;   // inotify instance
    int wd;   // watch descriptor on the directory
};

sso_dirwatch_t *sso_dirwatch_open(const char *dir)
{
    if (!dir || !dir[0]) return NULL;
    int fd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (fd < 0) return NULL;
    // bind() creates the .sock node (IN_CREATE); a stale-socket unlink
    // before it shows as IN_DELETE. Either way we just want to re-check.
    int wd = inotify_add_watch(fd, dir,
                               IN_CREATE | IN_MOVED_TO |
                               IN_DELETE | IN_MOVED_FROM);
    if (wd < 0) {
        close(fd);
        return NULL;
    }
    sso_dirwatch_t *w = calloc(1, sizeof *w);
    if (!w) {
        close(fd);
        return NULL;
    }
    w->fd = fd;
    w->wd = wd;
    return w;
}

int sso_dirwatch_wait(sso_dirwatch_t *w, int timeout_ms)
{
    if (!w) return -1;
    struct pollfd pfd = { .fd = w->fd, .events = POLLIN, .revents = 0 };
    int rc = poll(&pfd, 1, timeout_ms);
    if (rc < 0) return (errno == EINTR) ? 0 : -1;
    if (rc == 0) return 0;
    // Drain the queued events so the next poll() blocks afresh instead of
    // returning immediately on the same backlog. Their contents don't
    // matter: any change means "re-check the socket".
    char buf[4096]
        __attribute__((aligned(__alignof__(struct inotify_event))));
    while (read(w->fd, buf, sizeof buf) > 0) {
        // keep draining
    }
    return 1;
}

void sso_dirwatch_close(sso_dirwatch_t *w)
{
    if (!w) return;
    if (w->wd >= 0) inotify_rm_watch(w->fd, w->wd);
    if (w->fd >= 0) close(w->fd);
    free(w);
}

#elif defined(__APPLE__) || defined(__FreeBSD__) || defined(__NetBSD__) || \
      defined(__OpenBSD__) || defined(__DragonFly__)

#include <fcntl.h>
#include <sys/event.h>
#include <sys/time.h>
#include <unistd.h>

#ifndef O_EVTONLY
#define O_EVTONLY O_RDONLY
#endif

struct sso_dirwatch {
    int kq;
    int dirfd;   // the directory we registered for vnode events
};

sso_dirwatch_t *sso_dirwatch_open(const char *dir)
{
    if (!dir || !dir[0]) return NULL;
    int dirfd = open(dir, O_EVTONLY);
    if (dirfd < 0) return NULL;
    int kq = kqueue();
    if (kq < 0) {
        close(dirfd);
        return NULL;
    }
    struct kevent ev;
    // EV_CLEAR makes the note edge-triggered: each kevent() reports a change
    // once and re-arms, so we don't spin on a single past event.
    EV_SET(&ev, dirfd, EVFILT_VNODE, EV_ADD | EV_CLEAR,
           NOTE_WRITE | NOTE_DELETE | NOTE_RENAME, 0, NULL);
    if (kevent(kq, &ev, 1, NULL, 0, NULL) < 0) {
        close(kq);
        close(dirfd);
        return NULL;
    }
    sso_dirwatch_t *w = calloc(1, sizeof *w);
    if (!w) {
        close(kq);
        close(dirfd);
        return NULL;
    }
    w->kq = kq;
    w->dirfd = dirfd;
    return w;
}

int sso_dirwatch_wait(sso_dirwatch_t *w, int timeout_ms)
{
    if (!w) return -1;
    struct timespec ts;
    struct timespec *tsp = NULL;
    if (timeout_ms >= 0) {
        ts.tv_sec = timeout_ms / 1000;
        ts.tv_nsec = (long) (timeout_ms % 1000) * 1000000L;
        tsp = &ts;
    }
    struct kevent ev;
    int rc = kevent(w->kq, NULL, 0, &ev, 1, tsp);
    if (rc < 0) return (errno == EINTR) ? 0 : -1;
    return (rc == 0) ? 0 : 1;
}

void sso_dirwatch_close(sso_dirwatch_t *w)
{
    if (!w) return;
    if (w->kq >= 0) close(w->kq);
    if (w->dirfd >= 0) close(w->dirfd);
    free(w);
}

#else

// No directory-watch facility on this host. open() returns NULL and the
// caller keeps polling; wait()/close() never see a live handle.
sso_dirwatch_t *sso_dirwatch_open(const char *dir)
{
    (void) dir;
    return NULL;
}

int sso_dirwatch_wait(sso_dirwatch_t *w, int timeout_ms)
{
    (void) w;
    (void) timeout_ms;
    return -1;
}

void sso_dirwatch_close(sso_dirwatch_t *w)
{
    (void) w;
}

#endif
