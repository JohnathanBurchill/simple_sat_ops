#include "sso_ipc.h"

#include "sso_ipc_paths.h"
#include "sso_paths.h"
#include "sso_audit.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define SSO_IPC_MAX_CLIENTS 32
#define SSO_IPC_READ_BUF    8192
#define SSO_IPC_WRITE_BUF   32768
#define SSO_IPC_LINE_MAX    8000

// =============================================================
// JSON helpers (tiny, flat-object subset).
//
// We never produce nested objects at the top level except via the
// pre-serialised roster_json field, which is emitted verbatim. The
// decoder scans for "key":value pairs at the top level and tolerates
// extra/unknown fields for forward compatibility.

static void iso_utc_now(char *out, size_t out_size) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    gmtime_r(&ts.tv_sec, &tm);
    snprintf(out, out_size,
             "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             (long) (ts.tv_nsec / 1000000));
}

static int json_append(char **p, char *end, const char *s) {
    size_t n = strlen(s);
    if (*p + n > end) return -1;
    memcpy(*p, s, n);
    *p += n;
    return 0;
}

static int json_append_escaped(char **p, char *end, const char *s) {
    if (!s) s = "";
    if (*p + 1 > end) return -1;
    *(*p)++ = '"';
    for (; *s; ++s) {
        unsigned char c = (unsigned char) *s;
        char esc[8];
        const char *out = NULL;
        size_t outlen = 0;
        switch (c) {
            case '"':  out = "\\\""; outlen = 2; break;
            case '\\': out = "\\\\"; outlen = 2; break;
            case '\n': out = "\\n";  outlen = 2; break;
            case '\r': out = "\\r";  outlen = 2; break;
            case '\t': out = "\\t";  outlen = 2; break;
            default:
                if (c < 0x20) {
                    snprintf(esc, sizeof(esc), "\\u%04x", c);
                    out = esc;
                    outlen = 6;
                } else {
                    esc[0] = (char) c;
                    out = esc;
                    outlen = 1;
                }
                break;
        }
        if (*p + outlen > end) return -1;
        memcpy(*p, out, outlen);
        *p += outlen;
    }
    if (*p + 1 > end) return -1;
    *(*p)++ = '"';
    return 0;
}

static int json_field_str(char **p, char *end, int *first,
                           const char *key, const char *val) {
    if (!val || !val[0]) return 0;
    if (!*first && json_append(p, end, ",") < 0) return -1;
    if (json_append(p, end, "\"") < 0) return -1;
    if (json_append(p, end, key) < 0) return -1;
    if (json_append(p, end, "\":") < 0) return -1;
    if (json_append_escaped(p, end, val) < 0) return -1;
    *first = 0;
    return 0;
}

static int json_field_int(char **p, char *end, int *first,
                           const char *key, long val) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%ld", val);
    if (!*first && json_append(p, end, ",") < 0) return -1;
    if (json_append(p, end, "\"") < 0) return -1;
    if (json_append(p, end, key) < 0) return -1;
    if (json_append(p, end, "\":") < 0) return -1;
    if (json_append(p, end, buf) < 0) return -1;
    *first = 0;
    return 0;
}

static int json_field_double(char **p, char *end, int *first,
                              const char *key, double val) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%.6g", val);
    if (!*first && json_append(p, end, ",") < 0) return -1;
    if (json_append(p, end, "\"") < 0) return -1;
    if (json_append(p, end, key) < 0) return -1;
    if (json_append(p, end, "\":") < 0) return -1;
    if (json_append(p, end, buf) < 0) return -1;
    *first = 0;
    return 0;
}

static int json_field_bool(char **p, char *end, int *first,
                            const char *key, int val) {
    if (!*first && json_append(p, end, ",") < 0) return -1;
    if (json_append(p, end, "\"") < 0) return -1;
    if (json_append(p, end, key) < 0) return -1;
    if (json_append(p, end, "\":") < 0) return -1;
    if (json_append(p, end, val ? "true" : "false") < 0) return -1;
    *first = 0;
    return 0;
}

static int json_field_raw(char **p, char *end, int *first,
                           const char *key, const char *raw) {
    if (!raw || !raw[0]) return 0;
    if (!*first && json_append(p, end, ",") < 0) return -1;
    if (json_append(p, end, "\"") < 0) return -1;
    if (json_append(p, end, key) < 0) return -1;
    if (json_append(p, end, "\":") < 0) return -1;
    if (json_append(p, end, raw) < 0) return -1;
    *first = 0;
    return 0;
}

// Parser: locate "key" at the top level and return the unescaped value
// (if string) into outbuf, or write the textual value into outbuf for
// numbers / booleans. Returns 1 if found, 0 if not, -1 on parse error.
//
// We don't run a full JSON state machine; we look for the exact byte
// sequence "key": at brace-depth 1 (ignoring nested objects/arrays
// inside the value). Sufficient for our flat schemas.
static int json_skip_value(const char **p, const char *end);

static int json_skip_ws(const char **p, const char *end) {
    while (*p < end && (**p == ' ' || **p == '\t' || **p == '\n' || **p == '\r')) (*p)++;
    return 0;
}

static int json_skip_string(const char **p, const char *end) {
    if (*p >= end || **p != '"') return -1;
    (*p)++;
    while (*p < end) {
        if (**p == '\\') {
            (*p)++;
            if (*p < end) (*p)++;
            continue;
        }
        if (**p == '"') {
            (*p)++;
            return 0;
        }
        (*p)++;
    }
    return -1;
}

static int json_skip_value(const char **p, const char *end) {
    json_skip_ws(p, end);
    if (*p >= end) return -1;
    char c = **p;
    if (c == '"') return json_skip_string(p, end);
    if (c == '{' || c == '[') {
        char open = c, close = (c == '{') ? '}' : ']';
        int depth = 0;
        while (*p < end) {
            if (**p == '"') {
                if (json_skip_string(p, end) < 0) return -1;
                continue;
            }
            if (**p == open) depth++;
            else if (**p == close) {
                depth--;
                if (depth == 0) {
                    (*p)++;
                    return 0;
                }
            }
            (*p)++;
        }
        return -1;
    }
    // number / true / false / null — read until comma or close brace
    while (*p < end && **p != ',' && **p != '}' && **p != ']'
           && **p != ' ' && **p != '\t' && **p != '\n' && **p != '\r') (*p)++;
    return 0;
}

static int json_unescape_into(const char *src, size_t srclen,
                                char *out, size_t out_size) {
    if (out_size == 0) return -1;
    size_t oi = 0;
    for (size_t i = 0; i < srclen; ++i) {
        char c = src[i];
        if (c == '\\' && i + 1 < srclen) {
            ++i;
            char e = src[i];
            char m;
            switch (e) {
                case '"':  m = '"';  break;
                case '\\': m = '\\'; break;
                case '/':  m = '/';  break;
                case 'n':  m = '\n'; break;
                case 'r':  m = '\r'; break;
                case 't':  m = '\t'; break;
                case 'u':
                    // Hex escape — represent as '?' for our purposes.
                    if (i + 4 < srclen) i += 4;
                    m = '?';
                    break;
                default:   m = e; break;
            }
            if (oi + 1 >= out_size) return -1;
            out[oi++] = m;
        } else {
            if (oi + 1 >= out_size) return -1;
            out[oi++] = c;
        }
    }
    out[oi] = '\0';
    return 0;
}

static int json_get_field(const char *json, const char *key,
                            int *was_string, const char **val_start,
                            const char **val_end) {
    size_t json_len = strlen(json);
    const char *p = json;
    const char *end = json + json_len;
    json_skip_ws(&p, end);
    if (p >= end || *p != '{') return 0;
    p++;
    while (p < end) {
        json_skip_ws(&p, end);
        if (p >= end) return 0;
        if (*p == '}') return 0;
        if (*p != '"') return -1;
        const char *kstart = p + 1;
        const char *kp = p;
        if (json_skip_string(&kp, end) < 0) return -1;
        const char *kend = kp - 1;  // points at closing "
        size_t klen = (size_t) (kend - kstart);
        json_skip_ws(&kp, end);
        if (kp >= end || *kp != ':') return -1;
        kp++;
        json_skip_ws(&kp, end);
        const char *vstart = kp;
        int is_str = (kp < end && *kp == '"');
        if (json_skip_value(&kp, end) < 0) return -1;
        const char *vend = kp;
        // String values: trim the wrapping quotes.
        if (strlen(key) == klen && memcmp(key, kstart, klen) == 0) {
            *was_string = is_str;
            if (is_str) {
                *val_start = vstart + 1;
                *val_end = vend - 1;
            } else {
                *val_start = vstart;
                *val_end = vend;
            }
            return 1;
        }
        p = kp;
        json_skip_ws(&p, end);
        if (p < end && *p == ',') p++;
    }
    return 0;
}

static int json_get_string(const char *json, const char *key,
                            char *out, size_t out_size) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, key, &is_str, &vs, &ve);
    if (r <= 0) return r;
    if (!is_str) return -1;
    if (json_unescape_into(vs, (size_t) (ve - vs), out, out_size) < 0)
        return -1;
    return 1;
}

static int json_get_int(const char *json, const char *key, long *out) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, key, &is_str, &vs, &ve);
    if (r <= 0) return r;
    char tmp[40];
    size_t n = (size_t) (ve - vs);
    if (n >= sizeof(tmp)) return -1;
    memcpy(tmp, vs, n);
    tmp[n] = '\0';
    char *endp;
    long v = strtol(tmp, &endp, 10);
    if (endp == tmp) return -1;
    *out = v;
    return 1;
}

static int json_get_double(const char *json, const char *key, double *out) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, key, &is_str, &vs, &ve);
    if (r <= 0) return r;
    char tmp[40];
    size_t n = (size_t) (ve - vs);
    if (n >= sizeof(tmp)) return -1;
    memcpy(tmp, vs, n);
    tmp[n] = '\0';
    char *endp;
    double v = strtod(tmp, &endp);
    if (endp == tmp) return -1;
    *out = v;
    return 1;
}

static int json_get_bool(const char *json, const char *key, int *out) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, key, &is_str, &vs, &ve);
    if (r <= 0) return r;
    size_t n = (size_t) (ve - vs);
    if (n == 4 && memcmp(vs, "true", 4) == 0) { *out = 1; return 1; }
    if (n == 5 && memcmp(vs, "false", 5) == 0) { *out = 0; return 1; }
    return -1;
}

// Copy the raw substring of the value (including quotes / braces /
// brackets) for fields that are arrays or pre-serialised objects.
static int json_get_raw(const char *json, const char *key,
                         char *out, size_t out_size) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, key, &is_str, &vs, &ve);
    if (r <= 0) return r;
    // For raw extraction we want quotes/brackets included as found in
    // the original; json_get_field strips string quotes, so put them
    // back here for the string case.
    size_t n = (size_t) (ve - vs);
    if (is_str) {
        if (n + 2 >= out_size) return -1;
        out[0] = '"';
        memcpy(out + 1, vs, n);
        out[n + 1] = '"';
        out[n + 2] = '\0';
    } else {
        if (n >= out_size) return -1;
        memcpy(out, vs, n);
        out[n] = '\0';
    }
    return 1;
}

// =============================================================
// Event type table

static const struct {
    sso_event_type_t type;
    const char *name;
} g_event_types[] = {
    { SSO_EVT_HELLO,             "hello" },
    { SSO_EVT_WELCOME,           "welcome" },
    { SSO_EVT_STATE,             "state" },
    { SSO_EVT_OPERATOR_CHANGED,  "operator-changed" },
    { SSO_EVT_HANDOFF_OFFER,     "handoff-offer" },
    { SSO_EVT_HANDOFF_ACCEPTED,  "handoff-accepted" },
    { SSO_EVT_HANDOFF_DECLINED,  "handoff-declined" },
    { SSO_EVT_RX_STATS,          "rx-stats" },
    { SSO_EVT_TX_COMMAND_SENT,   "tx-command-sent" },
    { SSO_EVT_BYE,               "bye" },
    { SSO_EVT_YIELD_REQUEST,     "yield-request" },
    { SSO_EVT_UNKNOWN,           NULL },
};

const char *sso_event_type_name(sso_event_type_t type) {
    for (size_t i = 0; g_event_types[i].name; ++i) {
        if (g_event_types[i].type == type) return g_event_types[i].name;
    }
    return "?";
}

sso_event_type_t sso_event_type_from_name(const char *name) {
    if (!name) return SSO_EVT_UNKNOWN;
    for (size_t i = 0; g_event_types[i].name; ++i) {
        if (strcmp(g_event_types[i].name, name) == 0) return g_event_types[i].type;
    }
    return SSO_EVT_UNKNOWN;
}

void sso_event_init(sso_event_t *evt, sso_event_type_t type) {
    memset(evt, 0, sizeof(*evt));
    evt->type = type;
    iso_utc_now(evt->ts, sizeof(evt->ts));
}

int sso_event_encode(const sso_event_t *evt, char *out, size_t out_size) {
    if (!evt || !out || out_size < 8) return -1;
    char *p = out;
    char *end = out + out_size - 2;  // leave room for "}\n"
    if (json_append(&p, end, "{") < 0) return -1;
    int first = 1;
    if (json_field_str(&p, end, &first, "t", sso_event_type_name(evt->type)) < 0) return -1;
    if (json_field_str(&p, end, &first, "ts", evt->ts) < 0) return -1;
    if (json_field_str(&p, end, &first, "from", evt->from) < 0) return -1;
    if (json_field_str(&p, end, &first, "role", evt->role) < 0) return -1;
    if (json_field_str(&p, end, &first, "user", evt->user) < 0) return -1;
    if (json_field_str(&p, end, &first, "operator", evt->operator_user) < 0) return -1;
    if (json_field_str(&p, end, &first, "prev", evt->prev) < 0) return -1;
    if (json_field_str(&p, end, &first, "new", evt->new_user) < 0) return -1;
    if (json_field_str(&p, end, &first, "by", evt->by) < 0) return -1;
    if (json_field_str(&p, end, &first, "to", evt->to) < 0) return -1;
    if (evt->forced) {
        if (json_field_bool(&p, end, &first, "forced", 1) < 0) return -1;
    }
    if (json_field_str(&p, end, &first, "reason", evt->reason) < 0) return -1;
    if (json_field_str(&p, end, &first, "pass_folder", evt->pass_folder) < 0) return -1;
    if (evt->has_state) {
        if (json_field_str(&p, end, &first, "sat", evt->satellite) < 0) return -1;
        if (json_field_double(&p, end, &first, "az", evt->az) < 0) return -1;
        if (json_field_double(&p, end, &first, "el", evt->el) < 0) return -1;
        if (evt->freq_hz) {
            if (json_field_int(&p, end, &first, "freq", evt->freq_hz) < 0) return -1;
        }
        if (evt->doppler_hz != 0.0) {
            if (json_field_double(&p, end, &first, "doppler", evt->doppler_hz) < 0) return -1;
        }
        if (json_field_str(&p, end, &first, "rx_status", evt->rx_status) < 0) return -1;
        if (json_field_str(&p, end, &first, "tx_status", evt->tx_status) < 0) return -1;
        if (json_field_str(&p, end, &first, "tle_path", evt->tle_path) < 0) return -1;
        if (json_field_double(&p, end, &first, "target_az", evt->target_az) < 0) return -1;
        if (json_field_double(&p, end, &first, "target_el", evt->target_el) < 0) return -1;
        if (evt->flip) {
            if (json_field_bool(&p, end, &first, "flip", 1) < 0) return -1;
        }
        if (evt->in_pass) {
            if (json_field_bool(&p, end, &first, "in_pass", 1) < 0) return -1;
        }
        if (evt->tracking) {
            if (json_field_bool(&p, end, &first, "tracking", 1) < 0) return -1;
        }
        if (evt->roster_json[0]) {
            if (json_field_raw(&p, end, &first, "roster", evt->roster_json) < 0) return -1;
        }
    }
    // rx-stats fields
    if (evt->type == SSO_EVT_RX_STATS) {
        if (json_field_double(&p, end, &first, "snr_db", evt->snr_db) < 0) return -1;
        if (json_field_int(&p, end, &first, "packets", evt->packets) < 0) return -1;
        if (json_field_str(&p, end, &first, "last_packet_ts", evt->last_packet_ts) < 0) return -1;
        if (json_field_str(&p, end, &first, "last_packet_summary", evt->last_packet_summary) < 0) return -1;
    }
    if (evt->type == SSO_EVT_TX_COMMAND_SENT) {
        if (json_field_str(&p, end, &first, "ascii", evt->ascii) < 0) return -1;
    }
    if (json_append(&p, end + 2, "}\n") < 0) return -1;
    *p = '\0';
    return 0;
}

int sso_event_decode(const char *line, sso_event_t *evt) {
    if (!line || !evt) return -1;
    memset(evt, 0, sizeof(*evt));
    char t[32];
    if (json_get_string(line, "t", t, sizeof(t)) <= 0) return -1;
    evt->type = sso_event_type_from_name(t);
    json_get_string(line, "ts", evt->ts, sizeof(evt->ts));
    json_get_string(line, "from", evt->from, sizeof(evt->from));
    json_get_string(line, "role", evt->role, sizeof(evt->role));
    json_get_string(line, "user", evt->user, sizeof(evt->user));
    json_get_string(line, "operator", evt->operator_user, sizeof(evt->operator_user));
    json_get_string(line, "prev", evt->prev, sizeof(evt->prev));
    json_get_string(line, "new", evt->new_user, sizeof(evt->new_user));
    json_get_string(line, "by", evt->by, sizeof(evt->by));
    json_get_string(line, "to", evt->to, sizeof(evt->to));
    int forced = 0;
    if (json_get_bool(line, "forced", &forced) > 0) evt->forced = forced;
    json_get_string(line, "reason", evt->reason, sizeof(evt->reason));
    json_get_string(line, "pass_folder", evt->pass_folder, sizeof(evt->pass_folder));

    if (json_get_string(line, "sat", evt->satellite, sizeof(evt->satellite)) > 0) {
        evt->has_state = 1;
    }
    if (json_get_double(line, "az", &evt->az) > 0) evt->has_state = 1;
    if (json_get_double(line, "el", &evt->el) > 0) evt->has_state = 1;
    long freq;
    if (json_get_int(line, "freq", &freq) > 0) {
        evt->freq_hz = freq;
        evt->has_state = 1;
    }
    if (json_get_double(line, "doppler", &evt->doppler_hz) > 0) evt->has_state = 1;
    if (json_get_string(line, "rx_status", evt->rx_status, sizeof(evt->rx_status)) > 0) evt->has_state = 1;
    if (json_get_string(line, "tx_status", evt->tx_status, sizeof(evt->tx_status)) > 0) evt->has_state = 1;
    if (json_get_string(line, "tle_path", evt->tle_path, sizeof(evt->tle_path)) > 0) evt->has_state = 1;
    if (json_get_double(line, "target_az", &evt->target_az) > 0) evt->has_state = 1;
    if (json_get_double(line, "target_el", &evt->target_el) > 0) evt->has_state = 1;
    int flag = 0;
    if (json_get_bool(line, "flip",     &flag) > 0) evt->flip = flag;
    if (json_get_bool(line, "in_pass",  &flag) > 0) evt->in_pass = flag;
    if (json_get_bool(line, "tracking", &flag) > 0) evt->tracking = flag;
    if (json_get_raw(line, "roster", evt->roster_json, sizeof(evt->roster_json)) > 0) {
        evt->has_state = 1;
    }
    json_get_double(line, "snr_db", &evt->snr_db);
    json_get_int(line, "packets", &evt->packets);
    json_get_string(line, "last_packet_ts", evt->last_packet_ts, sizeof(evt->last_packet_ts));
    json_get_string(line, "last_packet_summary", evt->last_packet_summary, sizeof(evt->last_packet_summary));
    json_get_string(line, "ascii", evt->ascii, sizeof(evt->ascii));
    return 0;
}

int sso_event_set_roster(sso_event_t *evt,
                         const sso_roster_entry_t *entries,
                         size_t count) {
    if (!evt) return -1;
    char *p = evt->roster_json;
    char *end = evt->roster_json + sizeof(evt->roster_json) - 1;
    if (json_append(&p, end, "[") < 0) return -1;
    for (size_t i = 0; i < count; ++i) {
        if (i > 0 && json_append(&p, end, ",") < 0) return -1;
        if (json_append(&p, end, "{\"user\":") < 0) return -1;
        if (json_append_escaped(&p, end, entries[i].user) < 0) return -1;
        if (json_append(&p, end, ",\"role\":") < 0) return -1;
        if (json_append_escaped(&p, end, entries[i].role) < 0) return -1;
        if (entries[i].since[0]) {
            if (json_append(&p, end, ",\"since\":") < 0) return -1;
            if (json_append_escaped(&p, end, entries[i].since) < 0) return -1;
        }
        if (json_append(&p, end, "}") < 0) return -1;
    }
    if (json_append(&p, end, "]") < 0) return -1;
    *p = '\0';
    return 0;
}

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

static int set_nonblock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

// Ignore SIGPIPE process-wide the first time any IPC handle is opened.
// Without this, writing to a viewer socket that has just been closed by
// the peer kills the operator with the default SIGPIPE action — and
// since simple_sat_ops is in raw curses mode, that leaves the terminal
// in an unusable state. EPIPE is already handled by the write paths.
static void sigpipe_ignore_once(void) {
    static int done = 0;
    if (done) return;
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGPIPE, &sa, NULL);
    done = 1;
}

static void slot_close(sso_ipc_client_slot_t *slot) {
    if (slot->fd >= 0) close(slot->fd);
    slot->fd = -1;
    slot->dead = 0;
    slot->read_len = 0;
    slot->write_len = 0;
    slot->user[0] = '\0';
    slot->role[0] = '\0';
    slot->since[0] = '\0';
}

sso_ipc_server_t *sso_ipc_server_open(const char *tool) {
    if (!tool || !tool[0]) return NULL;
    sigpipe_ignore_once();
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
    if (set_nonblock(fd) < 0) {
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
        set_nonblock(cfd);
        slot->fd = cfd;
        slot->id = srv->next_id++;
        slot->read_len = 0;
        slot->write_len = 0;
        slot->dead = 0;
        slot->user[0] = '\0';
        slot->role[0] = '\0';
        iso_utc_now(slot->since, sizeof(slot->since));
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
    sigpipe_ignore_once();
    char path[256];
    if (sso_ipc_socket_path(path, sizeof(path), tool) != 0) return NULL;
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return NULL;
    if (set_nonblock(fd) < 0) {
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
