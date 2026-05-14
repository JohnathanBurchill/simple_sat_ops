// sso_ipc_paths.h — resolve socket / pid-file locations for sso IPC.
//
// On the ground machine: /run/sso/<tool>.sock + /run/sso/<tool>.pid.
// tmpfiles.d ships a config that creates /run/sso 2770 root:sso-ops at
// boot. Dev hosts (no /run/sso) fall back to
// ${XDG_RUNTIME_DIR:-/tmp}/sso/<tool>.sock.

#ifndef SSO_IPC_PATHS_H
#define SSO_IPC_PATHS_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Absolute path of the runtime directory (/run/sso or fallback).
const char *sso_ipc_runtime_dir(void);

// Compose path of the operator socket for `tool` ("simple_sat_ops",
// "b210_rx_tx"). Writes <runtime_dir>/<tool>.sock into out. Returns
// 0 on success, -1 on overflow.
int sso_ipc_socket_path(char *out, size_t out_size, const char *tool);

// Compose path of the operator PID file for `tool`. Writes
// <runtime_dir>/<tool>.pid. Returns 0 / -1.
int sso_ipc_pid_path(char *out, size_t out_size, const char *tool);

// Ensure the runtime directory exists. Returns 0 on success / already
// present, -1 on error.
int sso_ipc_ensure_runtime_dir(void);

#ifdef __cplusplus
}
#endif

#endif
