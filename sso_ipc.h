// sso_ipc.h — Unix domain socket server/client for the in-band IPC.
//
// Each operator-mode tool (simple_sat_ops, b210_rx_live) opens an
// sso_ipc_server bound to its tool socket; viewers and external
// producers connect via sso_ipc_client.
//
// Wire format: newline-delimited JSON. Each line is one event with a
// "t" field naming its type plus event-specific fields. The encoder
// only emits a fixed set of fields (the union of every event's
// fields); the decoder fills whatever fields it sees, tolerating
// unknown ones for forward compat. No nested objects (roster is
// transmitted as a JSON-encoded string for now).

#ifndef SSO_IPC_H
#define SSO_IPC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------- Events ----------

typedef enum {
    SSO_EVT_UNKNOWN = 0,
    SSO_EVT_HELLO,
    SSO_EVT_WELCOME,
    SSO_EVT_STATE,
    SSO_EVT_OPERATOR_CHANGED,
    SSO_EVT_HANDOFF_OFFER,
    SSO_EVT_HANDOFF_ACCEPTED,
    SSO_EVT_HANDOFF_DECLINED,
    SSO_EVT_RX_STATS,
    SSO_EVT_TX_COMMAND_SENT,
    SSO_EVT_BYE,
    SSO_EVT_YIELD_REQUEST,  // sent by force-claim path; recipient yields
} sso_event_type_t;

typedef struct {
    sso_event_type_t type;
    char ts[40];
    char from[64];

    // hello / welcome / operator / handoff
    char role[16];       // "viewer" | "external"
    char user[64];
    char operator_user[64];
    char prev[64];
    char new_user[64];
    char by[64];
    char to[64];
    int forced;          // 0 / 1
    char reason[32];
    char pass_folder[256];

    // state snapshot (also embedded in welcome)
    int has_state;
    char satellite[64];
    double az, el;
    long freq_hz;
    double doppler_hz;
    char rx_status[160];
    char tx_status[160];
    char roster_json[1024];  // serialized array of {user,role,since}

    // rx-stats
    double snr_db;
    long packets;
    char last_packet_ts[40];
    char last_packet_summary[160];

    // tx-command-sent
    char ascii[160];
} sso_event_t;

void sso_event_init(sso_event_t *evt, sso_event_type_t type);
const char *sso_event_type_name(sso_event_type_t type);
sso_event_type_t sso_event_type_from_name(const char *name);

// Encode an event to one newline-terminated JSON line. Returns 0 on
// success, -1 on overflow.
int sso_event_encode(const sso_event_t *evt, char *out, size_t out_size);

// Decode one JSON line (with or without trailing newline) into evt.
// Returns 0 on success, -1 on parse error.
int sso_event_decode(const char *line, sso_event_t *evt);

// Helper to embed a roster JSON array into evt->roster_json. Use this
// rather than manual concatenation. Returns 0 / -1 on overflow.
typedef struct {
    char user[64];
    char role[16];   // "operator" | "viewer"
    char since[40];
} sso_roster_entry_t;

int sso_event_set_roster(sso_event_t *evt,
                         const sso_roster_entry_t *entries,
                         size_t count);

// ---------- Server ----------

struct sso_ipc_server;
typedef struct sso_ipc_server sso_ipc_server_t;

typedef int sso_client_id_t;

// Bind the socket for `tool` at sso_ipc_socket_path(). Removes any
// stale socket file first; writes the pid file alongside. Returns the
// server handle, or NULL on error (errno set).
sso_ipc_server_t *sso_ipc_server_open(const char *tool);

// Close + cleanup (removes socket + pid file). Safe with NULL.
void sso_ipc_server_close(sso_ipc_server_t *srv);

// Non-blocking step: accept new clients, read incoming events, queue
// outgoing bytes, prune dead clients. Caller passes a poll timeout in
// ms (0 = non-blocking). Returns 0 on success, -1 on fatal error.
int sso_ipc_server_step(sso_ipc_server_t *srv, int timeout_ms);

// Fan-out: send the (already-encoded) line to every connected client.
// Caller-supplied line must include the trailing '\n'. Returns 0 if
// queued for everyone (slow consumers buffered up to a cap), -1 on
// fatal error.
int sso_ipc_server_broadcast(sso_ipc_server_t *srv, const char *line);

// Targeted send to one client.
int sso_ipc_server_send(sso_ipc_server_t *srv, sso_client_id_t id,
                         const char *line);

// Callback to receive parsed events from clients. Set once via
// sso_ipc_server_on_event. NULL clears.
typedef void (*sso_ipc_on_event_fn)(sso_ipc_server_t *srv,
                                     sso_client_id_t id,
                                     const sso_event_t *evt,
                                     void *user);
void sso_ipc_server_on_event(sso_ipc_server_t *srv,
                              sso_ipc_on_event_fn fn, void *user);

// Number of currently connected clients.
size_t sso_ipc_server_client_count(const sso_ipc_server_t *srv);

// Iterate clients. Returns 0 on each found client (with id + user set),
// -1 when no more. Caller starts with cursor=0.
typedef struct {
    size_t cursor;
} sso_ipc_iter_t;
int sso_ipc_server_next_client(const sso_ipc_server_t *srv,
                                sso_ipc_iter_t *iter,
                                sso_client_id_t *out_id,
                                char *out_user, size_t out_user_size,
                                char *out_role, size_t out_role_size,
                                char *out_since, size_t out_since_size);

// ---------- Client ----------

struct sso_ipc_client;
typedef struct sso_ipc_client sso_ipc_client_t;

// Connect to the server socket for `tool`. Returns a client handle, or
// NULL if the socket isn't bound / connection refused (errno set).
// `tool` is e.g. "simple_sat_ops" or "b210_rx_live".
sso_ipc_client_t *sso_ipc_client_connect(const char *tool);

// Close. Safe with NULL.
void sso_ipc_client_close(sso_ipc_client_t *cli);

// Non-blocking I/O step: drain read buffer, dispatch parsed events to
// the registered callback, flush queued writes. Returns 0 on success,
// 1 on disconnect (server closed the socket — caller should reconnect
// or render STALE), -1 on fatal error.
int sso_ipc_client_step(sso_ipc_client_t *cli, int timeout_ms);

// Send a line (must include '\n'). Returns 0 queued, -1 fatal.
int sso_ipc_client_send(sso_ipc_client_t *cli, const char *line);

typedef void (*sso_ipc_client_on_event_fn)(sso_ipc_client_t *cli,
                                             const sso_event_t *evt,
                                             void *user);
void sso_ipc_client_on_event(sso_ipc_client_t *cli,
                              sso_ipc_client_on_event_fn fn, void *user);

// True if the underlying socket is still open. Use this to drive STALE
// UX in viewer-mode UIs.
int sso_ipc_client_is_connected(const sso_ipc_client_t *cli);

#ifdef __cplusplus
}
#endif

#endif
