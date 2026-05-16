// sso_ipc.h — Unix domain socket server/client for the in-band IPC.
//
// Each operator-mode tool (simple_sat_ops, b210_rx_tx) opens an
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
#include <sys/types.h>

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
    SSO_EVT_TX_COMMAND_PREVIEW, // operator -> all viewers (debounced draft)
    SSO_EVT_TX_REQUEST,         // operator -> b210_rx_tx (commit)
    SSO_EVT_TX_ACK,             // b210_rx_tx -> operator (queued/ok/rejected)
    SSO_EVT_CMD_PREVIEW,        // operator -> viewers: live ":" prompt buffer
    SSO_EVT_CMD_EXECUTED,       // operator -> viewers: dispatched cmd + result
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
    double az, el;         // current rotator az/el (hardware-reported)
    long freq_hz;
    double doppler_hz;
    char rx_status[160];
    char tx_status[160];
    char roster_json[1024];  // serialized array of {user,role,since}
    char tle_path[256];      // operator's active TLE file (informational;
                             // viewer no longer loads it)
    double target_az;        // commanded rotator target
    double target_el;
    int flip;                // flip_mode_pass
    int in_pass;
    int tracking;
    int has_rotator;         // operator's have_antenna_rotator
    double jul_utc;          // operator's SGP4 epoch for this tick

    // Pre-computed prediction snapshot — the viewer renders these
    // verbatim, no local SGP4. Every field is the value the operator
    // is showing on its own screen right now.
    char idesg[9];           // matches TLE struct exactly
    double epoch_min;        // minutes_since_epoch
    double min_visible;      // predicted_minutes_until_visible
    double min_above_0;
    double min_above_30;
    double max_el;
    double pred_az;          // satellite_ephem.azimuth (SGP4, not rotator)
    double pred_el;
    double alt_km;
    double lat_deg;
    double lon_deg;
    double speed_kms;
    double range_km;
    double range_rate_kms;

    // rx-stats
    double snr_db;
    long packets;
    char last_packet_ts[40];
    char last_packet_summary[160];

    // tx-command-sent / tx-preview / tx-request / tx-ack
    char ascii[160];

    // Carried by tx-preview, tx-request, tx-command-sent (raw payload
    // the operator typed; viewers re-render this verbatim). tx_status is
    // only filled on tx-ack.
    char    tx_payload_kind[8];   // "hex" | "ascii"
    char    tx_payload[160];
    uint8_t tx_csp_src;
    uint8_t tx_csp_dst;
    uint8_t tx_csp_dport;
    uint8_t tx_csp_sport;
    uint8_t tx_csp_prio;
    long    tx_freq_hz;
    double  tx_gain_db;
    int     tx_allow_tx;
    int     tx_allow_high_power;
    int     tx_allow_hf_tx;
    int     tx_repeat;
    int     tx_gap_ms;
    char    tx_ack_status[24];    // tx-ack only: "ok" | "rejected: <reason>"

    // cmd-preview / cmd-executed: mirror the operator's ":" prompt to
    // viewers. cmd_text is the live buffer being typed (preview) or the
    // dispatched command line (executed). cmd_status is the post-dispatch
    // status string the operator sees in g_cmd_status; empty on preview.
    char    cmd_text[160];
    char    cmd_status[160];

    // RX panel mirror — viewer renders these directly. Carried inside
    // STATE events alongside the existing state fields. Six fixed slots
    // correspond to RX_PT_* in rx_session.h; both sides use the same
    // label table (rx_panel_pt_label in main.c). Payload preview is
    // raw bytes, hex-encoded on the wire.
#define SSO_RX_PT_SLOTS       6
#define SSO_RX_PT_PAYLOAD_MAX 64
#define SSO_RX_PT_SUMMARY_MAX 160
#define SSO_RIBBON_MAX        60
    int     rx_have_session;
    int     rx_rec_active;
    double  rx_freq_hz;
    double  rx_peak_dbfs;
    double  rx_rms_dbfs;
    long    rx_frames_total;
    long    rx_frames_iq;       // shadow IQ-demod count for A/B
    long    rx_frames_vit;      // shadow Viterbi-MLSE count for A/B
    char    rx_last_frame_summary[80];
    double  rx_age_s;                   // <0 means "no frame yet"
    long    rx_pt_count[SSO_RX_PT_SLOTS];
    int     rx_pt_payload_len[SSO_RX_PT_SLOTS];
    uint8_t rx_pt_payload[SSO_RX_PT_SLOTS][SSO_RX_PT_PAYLOAD_MAX];
    char    rx_pt_summary[SSO_RX_PT_SLOTS][SSO_RX_PT_SUMMARY_MAX];
    int     rx_ribbon_n;
    char    rx_ribbon[SSO_RIBBON_MAX + 1];  // '.' / '-' chars per second + nul
    int8_t  rx_ribbon_peak[SSO_RIBBON_MAX]; // peak dBFS per second (parallel)
    char    rx_warning[80];                  // optional rx-panel warning row
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

// Peer credential captured at accept(). Returns 0 with *out filled on
// success, -1 if the slot is unknown / dead / peer-cred unavailable.
int sso_ipc_server_peer_uid(const sso_ipc_server_t *srv,
                             sso_client_id_t id, uid_t *out);

// ---------- Client ----------

struct sso_ipc_client;
typedef struct sso_ipc_client sso_ipc_client_t;

// Connect to the server socket for `tool`. Returns a client handle, or
// NULL if the socket isn't bound / connection refused (errno set).
// `tool` is e.g. "simple_sat_ops" or "b210_rx_tx".
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
