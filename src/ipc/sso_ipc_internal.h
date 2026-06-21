// sso_ipc_internal.h — private declarations shared between the sso_ipc
// codec, server, and client translation units. NOT part of the public API;
// callers include sso_ipc.h. The split keeps the wire codec independent of
// the socket transport so the codec can be unit-tested on its own.

#ifndef SSO_IPC_INTERNAL_H
#define SSO_IPC_INTERNAL_H

#include <stddef.h>

#include "sso_ipc.h"   // SSO_IPC_LINE_MAX (the shared encode / per-line cap)

#ifdef __cplusplus
extern "C" {
#endif

// Per-connection buffer sizes, shared by the server's client slots and the
// client handle. SSO_IPC_LINE_MAX (the per-line cap) is public, in sso_ipc.h.
#define SSO_IPC_READ_BUF    8192
#define SSO_IPC_WRITE_BUF   32768

// Shared low-level helpers, defined in sso_ipc_codec.c (the common TU both
// the server and the client already depend on for event encode/decode).

// Format the current UTC time as "YYYY-MM-DDThh:mm:ss.mmmZ" into out.
void sso_ipc_iso_utc_now(char *out, size_t out_size);

// Put a socket into non-blocking mode. Returns 0 on success, -1 on error.
int sso_ipc_set_nonblock(int fd);

// Ignore SIGPIPE process-wide the first time any IPC handle is opened, so a
// write to a peer that just closed returns EPIPE instead of killing us.
void sso_ipc_sigpipe_ignore_once(void);

#ifdef __cplusplus
}
#endif

#endif // SSO_IPC_INTERNAL_H
