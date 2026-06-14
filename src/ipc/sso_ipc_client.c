// sso_ipc_client.c — the sso_ipc client transport: connect to a tool's
// socket, drain reads and dispatch parsed events (via the codec's
// sso_event_decode) to the registered callback, flush queued writes. Shares
// the low-level socket helpers in sso_ipc_codec.c.
//
// Split out of the former monolithic sso_ipc.c.

#define _GNU_SOURCE
#include "sso_ipc.h"
#include "sso_ipc_internal.h"

#include "sso_ipc_paths.h"
#include "sso_audit.h"   // sso_unix_user (audit link anchor at end of file)

#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

// =============================================================
// Client

struct sso_ipc_client {
    int fd;
    char tool[64];
    char read_buf[SSO_IPC_READ_BUF];
    size_t read_len;
    char write_buf[SSO_IPC_WRITE_BUF];
    size_t write_len;
    sso_ipc_client_on_event_fn on_event;
    void *on_event_user;
    int connected;
};

sso_ipc_client_t *sso_ipc_client_connect(const char *tool) {
    if (!tool || !tool[0]) return NULL;
    sso_ipc_sigpipe_ignore_once();
    char path[256];
    if (sso_ipc_socket_path(path, sizeof(path), tool) != 0) return NULL;
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return NULL;
    if (sso_ipc_set_nonblock(fd) < 0) {
        close(fd);
        return NULL;
    }
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    size_t path_len = strlen(path);
    if (path_len >= sizeof(addr.sun_path)) {
        // Linux's sun_path is 108 bytes; our path buffer is 256. Reject
        // overlong paths up front rather than letting snprintf silently
        // truncate (and rather than letting GCC -Wformat-truncation
        // complain about it).
        errno = ENAMETOOLONG;
        close(fd);
        return NULL;
    }
    memcpy(addr.sun_path, path, path_len + 1);
    if (connect(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        if (errno != EINPROGRESS) {
            close(fd);
            return NULL;
        }
    }
    sso_ipc_client_t *cli = calloc(1, sizeof(*cli));
    if (!cli) {
        close(fd);
        return NULL;
    }
    cli->fd = fd;
    snprintf(cli->tool, sizeof(cli->tool), "%s", tool);
    cli->connected = 1;
    return cli;
}

void sso_ipc_client_close(sso_ipc_client_t *cli) {
    if (!cli) return;
    if (cli->fd >= 0) close(cli->fd);
    free(cli);
}

void sso_ipc_client_on_event(sso_ipc_client_t *cli,
                              sso_ipc_client_on_event_fn fn, void *user) {
    if (!cli) return;
    cli->on_event = fn;
    cli->on_event_user = user;
}

static int client_drain_read(sso_ipc_client_t *cli) {
    for (;;) {
        if (cli->read_len >= sizeof(cli->read_buf) - 1) {
            cli->connected = 0;
            return 1;
        }
        ssize_t n = read(cli->fd, cli->read_buf + cli->read_len,
                         sizeof(cli->read_buf) - cli->read_len - 1);
        if (n == 0) {
            cli->connected = 0;
            return 1;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            cli->connected = 0;
            return 1;
        }
        cli->read_len += (size_t) n;
        cli->read_buf[cli->read_len] = '\0';
        for (;;) {
            char *nl = memchr(cli->read_buf, '\n', cli->read_len);
            if (!nl) break;
            *nl = '\0';
            if (cli->on_event) {
                sso_event_t evt;
                if (sso_event_decode(cli->read_buf, &evt) == 0) {
                    cli->on_event(cli, &evt, cli->on_event_user);
                }
            }
            size_t consumed = (size_t) (nl - cli->read_buf) + 1;
            size_t remain = cli->read_len - consumed;
            memmove(cli->read_buf, nl + 1, remain);
            cli->read_len = remain;
            cli->read_buf[cli->read_len] = '\0';
        }
    }
    return 0;
}

static void client_drain_write(sso_ipc_client_t *cli) {
    while (cli->write_len > 0) {
        ssize_t n = write(cli->fd, cli->write_buf, cli->write_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;
            cli->connected = 0;
            return;
        }
        if (n == 0) {
            cli->connected = 0;
            return;
        }
        size_t remain = cli->write_len - (size_t) n;
        memmove(cli->write_buf, cli->write_buf + n, remain);
        cli->write_len = remain;
    }
}

int sso_ipc_client_step(sso_ipc_client_t *cli, int timeout_ms) {
    if (!cli || cli->fd < 0 || !cli->connected) return 1;
    struct pollfd pfd;
    pfd.fd = cli->fd;
    pfd.events = POLLIN | (cli->write_len > 0 ? POLLOUT : 0);
    int r = poll(&pfd, 1, timeout_ms);
    if (r < 0) {
        if (errno == EINTR) return 0;
        return -1;
    }
    if (r == 0) return 0;
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        cli->connected = 0;
        return 1;
    }
    if (pfd.revents & POLLIN) {
        if (client_drain_read(cli)) return 1;
    }
    if (pfd.revents & POLLOUT) client_drain_write(cli);
    return 0;
}

int sso_ipc_client_send(sso_ipc_client_t *cli, const char *line) {
    if (!cli || !cli->connected || !line) return -1;
    size_t n = strlen(line);
    if (cli->write_len + n > sizeof(cli->write_buf)) {
        cli->connected = 0;
        return -1;
    }
    memcpy(cli->write_buf + cli->write_len, line, n);
    cli->write_len += n;
    client_drain_write(cli);
    return 0;
}

int sso_ipc_client_is_connected(const sso_ipc_client_t *cli) {
    return cli && cli->connected;
}

// (void) sso_audit symbol referenced to ensure the audit lib is part
// of the link graph of consumers that pull sso_ipc.
static __attribute__((unused)) const void *sso_ipc_audit_anchor = (const void *) sso_unix_user;
