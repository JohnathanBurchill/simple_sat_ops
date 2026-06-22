// sso_ipc_server.c — the sso_ipc server transport: the listen socket, the
// fixed pool of client slots, accept / read / dispatch / write / prune, and
// the broadcast + targeted-send + iterate + peer-uid API. Parses inbound
// lines with the codec (sso_event_decode) and shares the low-level socket
// helpers in sso_ipc_codec.c.
//
// Split out of the former monolithic sso_ipc.c.

#define _GNU_SOURCE  // SO_PEERCRED / struct ucred on glibc
#include "sso_ipc.h"
#include "sso_ipc_internal.h"

#include "sso_ipc_paths.h"

#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#define SSO_IPC_MAX_CLIENTS 32

// =============================================================
// Server

typedef struct {
    int fd;
    sso_client_id_t id;
    char read_buf[SSO_IPC_READ_BUF];
    size_t read_len;
    char write_buf[SSO_IPC_WRITE_BUF];
    size_t write_len;
    int dead;
    char user[64];
    char role[16];
    char since[40];
    uid_t peer_uid;
    int   peer_uid_valid;
} sso_ipc_client_slot_t;

struct sso_ipc_server {
    int listen_fd;
    char sock_path[256];
    char pid_path[256];
    sso_ipc_client_slot_t clients[SSO_IPC_MAX_CLIENTS];
    sso_client_id_t next_id;
    sso_ipc_on_event_fn on_event;
    void *on_event_user;
};

static void slot_close(sso_ipc_client_slot_t *slot) {
    if (slot->fd >= 0) close(slot->fd);
    slot->fd = -1;
    slot->dead = 0;
    slot->read_len = 0;
    slot->write_len = 0;
    slot->user[0] = '\0';
    slot->role[0] = '\0';
    slot->since[0] = '\0';
    slot->peer_uid = 0;
    slot->peer_uid_valid = 0;
}

static void slot_capture_peer_uid(sso_ipc_client_slot_t *slot) {
    slot->peer_uid_valid = 0;
#if defined(__linux__) && defined(SO_PEERCRED)
    struct ucred cred;
    socklen_t len = sizeof(cred);
    if (getsockopt(slot->fd, SOL_SOCKET, SO_PEERCRED, &cred, &len) == 0) {
        slot->peer_uid = cred.uid;
        slot->peer_uid_valid = 1;
    }
#else
    uid_t uid;
    gid_t gid;
    if (getpeereid(slot->fd, &uid, &gid) == 0) {
        slot->peer_uid = uid;
        slot->peer_uid_valid = 1;
    }
#endif
}

sso_ipc_server_t *sso_ipc_server_open(const char *tool) {
    if (!tool || !tool[0]) return NULL;
    sso_ipc_sigpipe_ignore_once();
    if (sso_ipc_ensure_runtime_dir() != 0) return NULL;

    sso_ipc_server_t *srv = calloc(1, sizeof(*srv));
    if (!srv) return NULL;
    srv->listen_fd = -1;
    srv->next_id = 1;
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) srv->clients[i].fd = -1;

    if (sso_ipc_socket_path(srv->sock_path, sizeof(srv->sock_path), tool) != 0
        || sso_ipc_pid_path(srv->pid_path, sizeof(srv->pid_path), tool) != 0) {
        free(srv);
        return NULL;
    }

    // Remove stale socket (best-effort).
    unlink(srv->sock_path);

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        free(srv);
        return NULL;
    }
    if (sso_ipc_set_nonblock(fd) < 0) {
        close(fd);
        free(srv);
        return NULL;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    // sun_path is 108 bytes on Linux. Our sock_path buffer is wider, so
    // bound-check before copying — GCC's -Wformat-truncation flags an
    // snprintf here otherwise, and the underlying ENAMETOOLONG is a
    // real failure mode worth surfacing.
    size_t sp_len = strlen(srv->sock_path);
    if (sp_len >= sizeof(addr.sun_path)) {
        errno = ENAMETOOLONG;
        close(fd);
        free(srv);
        return NULL;
    }
    memcpy(addr.sun_path, srv->sock_path, sp_len + 1);

    if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        close(fd);
        free(srv);
        return NULL;
    }

    chmod(srv->sock_path, 0660);

    if (listen(fd, SSO_IPC_MAX_CLIENTS) < 0) {
        close(fd);
        unlink(srv->sock_path);
        free(srv);
        return NULL;
    }

    // Write pid file.
    FILE *pf = fopen(srv->pid_path, "w");
    if (pf) {
        fprintf(pf, "%d\n", (int) getpid());
        fclose(pf);
        chmod(srv->pid_path, 0660);
    }

    srv->listen_fd = fd;
    return srv;
}

void sso_ipc_server_close(sso_ipc_server_t *srv) {
    if (!srv) return;
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) slot_close(&srv->clients[i]);
    if (srv->listen_fd >= 0) close(srv->listen_fd);
    if (srv->sock_path[0]) unlink(srv->sock_path);
    if (srv->pid_path[0]) unlink(srv->pid_path);
    free(srv);
}

void sso_ipc_server_on_event(sso_ipc_server_t *srv,
                              sso_ipc_on_event_fn fn, void *user) {
    if (!srv) return;
    srv->on_event = fn;
    srv->on_event_user = user;
}

static sso_ipc_client_slot_t *server_alloc_slot(sso_ipc_server_t *srv) {
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) {
        if (srv->clients[i].fd < 0) return &srv->clients[i];
    }
    return NULL;
}

static void server_accept(sso_ipc_server_t *srv) {
    for (;;) {
        struct sockaddr_un peer;
        socklen_t peerlen = sizeof(peer);
        int cfd = accept(srv->listen_fd, (struct sockaddr *) &peer, &peerlen);
        if (cfd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;
            return;
        }
        sso_ipc_client_slot_t *slot = server_alloc_slot(srv);
        if (!slot) {
            close(cfd);
            continue;
        }
        sso_ipc_set_nonblock(cfd);
        slot->fd = cfd;
        slot->id = srv->next_id++;
        slot->read_len = 0;
        slot->write_len = 0;
        slot->dead = 0;
        slot->user[0] = '\0';
        slot->role[0] = '\0';
        sso_ipc_iso_utc_now(slot->since, sizeof(slot->since));
        slot_capture_peer_uid(slot);
    }
}

static int slot_dispatch_line(sso_ipc_server_t *srv,
                              sso_ipc_client_slot_t *slot,
                              char *line) {
    sso_event_t evt;
    if (sso_event_decode(line, &evt) != 0) return 0;
    // Special handling: capture hello's identity onto the slot so the
    // server can build the roster + route handoff offers without the
    // caller having to track it themselves.
    if (evt.type == SSO_EVT_HELLO) {
        if (evt.user[0]) snprintf(slot->user, sizeof(slot->user), "%s", evt.user);
        if (evt.role[0]) snprintf(slot->role, sizeof(slot->role), "%s", evt.role);
    }
    if (srv->on_event) {
        srv->on_event(srv, slot->id, &evt, srv->on_event_user);
    }
    return 0;
}

static void slot_drain_read(sso_ipc_server_t *srv,
                             sso_ipc_client_slot_t *slot) {
    for (;;) {
        if (slot->read_len >= sizeof(slot->read_buf) - 1) {
            // Line longer than buffer: drop the client.
            slot->dead = 1;
            return;
        }
        ssize_t n = read(slot->fd, slot->read_buf + slot->read_len,
                         sizeof(slot->read_buf) - slot->read_len - 1);
        if (n == 0) {
            slot->dead = 1;
            return;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            slot->dead = 1;
            return;
        }
        slot->read_len += (size_t) n;
        slot->read_buf[slot->read_len] = '\0';
        for (;;) {
            char *nl = memchr(slot->read_buf, '\n', slot->read_len);
            if (!nl) break;
            *nl = '\0';
            slot_dispatch_line(srv, slot, slot->read_buf);
            size_t consumed = (size_t) (nl - slot->read_buf) + 1;
            size_t remain = slot->read_len - consumed;
            memmove(slot->read_buf, nl + 1, remain);
            slot->read_len = remain;
            slot->read_buf[slot->read_len] = '\0';
        }
    }
}

static void slot_drain_write(sso_ipc_client_slot_t *slot) {
    while (slot->write_len > 0) {
        ssize_t n = write(slot->fd, slot->write_buf, slot->write_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;
            slot->dead = 1;
            return;
        }
        if (n == 0) {
            slot->dead = 1;
            return;
        }
        size_t remain = slot->write_len - (size_t) n;
        memmove(slot->write_buf, slot->write_buf + n, remain);
        slot->write_len = remain;
    }
}

static int slot_queue(sso_ipc_client_slot_t *slot, const char *line) {
    size_t n = strlen(line);
    if (slot->write_len + n > sizeof(slot->write_buf)) {
        // Slow consumer overflow: drop the client (we'd lose synchrony).
        slot->dead = 1;
        return -1;
    }
    memcpy(slot->write_buf + slot->write_len, line, n);
    slot->write_len += n;
    return 0;
}

// Best-effort enqueue: on overflow DROP THE LINE and keep the client alive,
// rather than killing it. For loss-tolerant high-rate traffic (live audio)
// where a missed frame is fine — the Ogg/Vorbis decoder resyncs at the next
// page — but a dropped connection would stop the stream and force a reconnect.
// Returns 0 queued, 1 dropped (buffer full).
static int slot_queue_lossy(sso_ipc_client_slot_t *slot, const char *line) {
    size_t n = strlen(line);
    if (slot->write_len + n > sizeof(slot->write_buf)) {
        return 1;
    }
    memcpy(slot->write_buf + slot->write_len, line, n);
    slot->write_len += n;
    return 0;
}

int sso_ipc_server_step(sso_ipc_server_t *srv, int timeout_ms) {
    if (!srv || srv->listen_fd < 0) return -1;
    struct pollfd pfds[1 + SSO_IPC_MAX_CLIENTS];
    nfds_t nfds = 0;
    pfds[nfds].fd = srv->listen_fd;
    pfds[nfds].events = POLLIN;
    nfds++;
    int slot_indices[SSO_IPC_MAX_CLIENTS];
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) {
        if (srv->clients[i].fd < 0) continue;
        pfds[nfds].fd = srv->clients[i].fd;
        pfds[nfds].events = POLLIN | (srv->clients[i].write_len > 0 ? POLLOUT : 0);
        slot_indices[nfds - 1] = (int) i;
        nfds++;
    }
    int r = poll(pfds, nfds, timeout_ms);
    if (r < 0) {
        if (errno == EINTR) return 0;
        return -1;
    }
    if (r == 0) return 0;
    if (pfds[0].revents & POLLIN) server_accept(srv);
    for (nfds_t i = 1; i < nfds; ++i) {
        int idx = slot_indices[i - 1];
        sso_ipc_client_slot_t *slot = &srv->clients[idx];
        if (pfds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            slot->dead = 1;
        }
        if (!slot->dead && (pfds[i].revents & POLLIN)) slot_drain_read(srv, slot);
        if (!slot->dead && (pfds[i].revents & POLLOUT)) slot_drain_write(slot);
    }
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) {
        if (srv->clients[i].fd >= 0 && srv->clients[i].dead) {
            slot_close(&srv->clients[i]);
        }
    }
    return 0;
}

int sso_ipc_server_broadcast(sso_ipc_server_t *srv, const char *line) {
    if (!srv || !line) return -1;
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) {
        if (srv->clients[i].fd >= 0 && !srv->clients[i].dead) {
            slot_queue(&srv->clients[i], line);
            slot_drain_write(&srv->clients[i]);
        }
    }
    return 0;
}

int sso_ipc_server_send(sso_ipc_server_t *srv, sso_client_id_t id,
                         const char *line) {
    if (!srv || !line) return -1;
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) {
        if (srv->clients[i].fd >= 0 && srv->clients[i].id == id) {
            slot_queue(&srv->clients[i], line);
            slot_drain_write(&srv->clients[i]);
            return 0;
        }
    }
    return -1;
}

int sso_ipc_server_send_lossy(sso_ipc_server_t *srv, sso_client_id_t id,
                              const char *line) {
    if (!srv || !line) return -1;
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) {
        if (srv->clients[i].fd >= 0 && srv->clients[i].id == id) {
            // Drain first (the socket may have caught up since last call) to
            // free room, then best-effort enqueue, then push it out.
            slot_drain_write(&srv->clients[i]);
            int rc = slot_queue_lossy(&srv->clients[i], line);
            slot_drain_write(&srv->clients[i]);
            return rc;   // 0 sent, 1 dropped (full); client kept alive
        }
    }
    return -1;
}

size_t sso_ipc_server_client_count(const sso_ipc_server_t *srv) {
    if (!srv) return 0;
    size_t n = 0;
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) {
        if (srv->clients[i].fd >= 0 && !srv->clients[i].dead) n++;
    }
    return n;
}

int sso_ipc_server_next_client(const sso_ipc_server_t *srv,
                                sso_ipc_iter_t *iter,
                                sso_client_id_t *out_id,
                                char *out_user, size_t out_user_size,
                                char *out_role, size_t out_role_size,
                                char *out_since, size_t out_since_size) {
    if (!srv || !iter) return -1;
    for (; iter->cursor < SSO_IPC_MAX_CLIENTS; ++iter->cursor) {
        const sso_ipc_client_slot_t *s = &srv->clients[iter->cursor];
        if (s->fd < 0 || s->dead) continue;
        if (out_id) *out_id = s->id;
        if (out_user && out_user_size) snprintf(out_user, out_user_size, "%s", s->user);
        if (out_role && out_role_size) snprintf(out_role, out_role_size, "%s", s->role);
        if (out_since && out_since_size) snprintf(out_since, out_since_size, "%s", s->since);
        iter->cursor++;
        return 0;
    }
    return -1;
}

int sso_ipc_server_peer_uid(const sso_ipc_server_t *srv,
                             sso_client_id_t id, uid_t *out) {
    if (!srv || !out) return -1;
    for (size_t i = 0; i < SSO_IPC_MAX_CLIENTS; ++i) {
        const sso_ipc_client_slot_t *s = &srv->clients[i];
        if (s->fd < 0 || s->dead) continue;
        if (s->id != id) continue;
        if (!s->peer_uid_valid) return -1;
        *out = s->peer_uid;
        return 0;
    }
    return -1;
}
