// sso_ipc_codec.c — the sso_ipc wire codec: the tiny flat-object JSON
// reader/writer and the event-type table + per-event encode/decode. Plus
// the handful of low-level helpers shared with the server and client
// transports (timestamp, socket non-block, one-time SIGPIPE ignore), kept
// here because both transports already depend on this TU for the codec.
//
// Split out of the former monolithic sso_ipc.c. Wire format and the public
// API are documented in sso_ipc.h.

#define _GNU_SOURCE  // gmtime_r and friends on glibc
#include "sso_ipc.h"
#include "sso_ipc_internal.h"
#include "sso_time.h"

#include <fcntl.h>      // O_NONBLOCK (sso_ipc_set_nonblock)
#include <signal.h>     // sigaction (sso_ipc_sigpipe_ignore_once)
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>       // clock_gettime (sso_ipc_iso_utc_now)

// =============================================================
// Shared low-level helpers (used by the server and client transports).
// sso_ipc_iso_utc_now lives below with the JSON helpers (it formats the
// event timestamp).

int sso_ipc_set_nonblock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

// Ignore SIGPIPE process-wide the first time any IPC handle is opened.
// Without this, writing to a viewer socket that has just been closed by
// the peer kills the operator with the default SIGPIPE action — and
// since simple_sat_ops is in raw curses mode, that leaves the terminal
// in an unusable state. EPIPE is already handled by the write paths.
void sso_ipc_sigpipe_ignore_once(void) {
    static int done = 0;
    if (done) return;
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGPIPE, &sa, NULL);
    done = 1;
}

// =============================================================
// JSON helpers (tiny, flat-object subset).
//
// We never produce nested objects at the top level except via the
// pre-serialised roster_json field, which is emitted verbatim. The
// decoder scans for "key":value pairs at the top level and tolerates
// extra/unknown fields for forward compatibility.

void sso_ipc_iso_utc_now(char *out, size_t out_size) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    sso_iso_utc_from_ts(&ts, out, out_size);
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
                    // Hex escape — represent as '?' for our purposes. i points
                    // at the 'u'; the four hex digits are at i+1..i+4, so they
                    // are all in range exactly when i+4 < srclen. A truncated
                    // \u at the very end (< 4 digits) falls through and its
                    // stray chars are copied literally rather than over-skipped.
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

static int json_get_field(const char *json, size_t json_len, const char *key,
                            int *was_string, const char **val_start,
                            const char **val_end) {
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

static int json_get_string(const char *json, size_t json_len, const char *key,
                            char *out, size_t out_size) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, json_len, key, &is_str, &vs, &ve);
    if (r <= 0) return r;
    if (!is_str) return -1;
    if (json_unescape_into(vs, (size_t) (ve - vs), out, out_size) < 0)
        return -1;
    return 1;
}

static int json_get_int(const char *json, size_t json_len, const char *key, long *out) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, json_len, key, &is_str, &vs, &ve);
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

static int json_get_double(const char *json, size_t json_len, const char *key, double *out) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, json_len, key, &is_str, &vs, &ve);
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

static int json_get_bool(const char *json, size_t json_len, const char *key, int *out) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, json_len, key, &is_str, &vs, &ve);
    if (r <= 0) return r;
    size_t n = (size_t) (ve - vs);
    if (n == 4 && memcmp(vs, "true", 4) == 0) { *out = 1; return 1; }
    if (n == 5 && memcmp(vs, "false", 5) == 0) { *out = 0; return 1; }
    return -1;
}

// Copy the raw substring of the value (including quotes / braces /
// brackets) for fields that are arrays or pre-serialised objects.
static int json_get_raw(const char *json, size_t json_len, const char *key,
                         char *out, size_t out_size) {
    int is_str;
    const char *vs, *ve;
    int r = json_get_field(json, json_len, key, &is_str, &vs, &ve);
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
    { SSO_EVT_TX_COMMAND_PREVIEW,"tx-preview" },
    { SSO_EVT_TX_REQUEST,        "tx-request" },
    { SSO_EVT_TX_NOT_SENT,       "tx-not-sent" },
    { SSO_EVT_CMD_PREVIEW,       "cmd-preview" },
    { SSO_EVT_CMD_EXECUTED,      "cmd-executed" },
    { SSO_EVT_AUDIO_CTL,         "audio-ctl" },
    { SSO_EVT_AUDIO_STATUS,      "audio-status" },
    { SSO_EVT_AUDIO,             "audio" },
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
    sso_ipc_iso_utc_now(evt->ts, sizeof(evt->ts));
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
        if (evt->source[0]) {
            if (json_field_str(&p, end, &first, "source", evt->source) < 0) return -1;
        }
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
        if (evt->jul_utc != 0.0) {
            if (json_field_double(&p, end, &first, "jul", evt->jul_utc) < 0) return -1;
        }
        if (evt->has_rotator) {
            if (json_field_bool(&p, end, &first, "has_rot", 1) < 0) return -1;
        }
        if (json_field_str(&p, end, &first, "idesg", evt->idesg) < 0) return -1;
        if (json_field_double(&p, end, &first, "ep_min",  evt->epoch_min) < 0) return -1;
        if (json_field_double(&p, end, &first, "mv",      evt->min_visible) < 0) return -1;
        if (json_field_double(&p, end, &first, "ma0",     evt->min_above_0) < 0) return -1;
        if (json_field_double(&p, end, &first, "ma30",    evt->min_above_30) < 0) return -1;
        if (json_field_double(&p, end, &first, "max_el",  evt->max_el) < 0) return -1;
        if (json_field_double(&p, end, &first, "p_az",    evt->pred_az) < 0) return -1;
        if (json_field_double(&p, end, &first, "p_el",    evt->pred_el) < 0) return -1;
        if (json_field_double(&p, end, &first, "alt",     evt->alt_km) < 0) return -1;
        if (json_field_double(&p, end, &first, "lat",     evt->lat_deg) < 0) return -1;
        if (json_field_double(&p, end, &first, "lon",     evt->lon_deg) < 0) return -1;
        if (json_field_double(&p, end, &first, "spd",     evt->speed_kms) < 0) return -1;
        if (json_field_double(&p, end, &first, "rng",     evt->range_km) < 0) return -1;
        if (json_field_double(&p, end, &first, "rrate",   evt->range_rate_kms) < 0) return -1;
        if (evt->roster_json[0]) {
            if (json_field_raw(&p, end, &first, "roster", evt->roster_json) < 0) return -1;
        }
        // Auto-TCMD progress. Only on the wire while the operator has a
        // run to report; viewers render "<sent>/<total> (<state>)".
        if (evt->auto_tcmd_on) {
            if (json_field_bool(&p, end, &first, "at_on", 1) < 0) return -1;
            if (json_field_int (&p, end, &first, "at_sent", evt->auto_tcmd_sent) < 0) return -1;
            if (json_field_int (&p, end, &first, "at_tot",  evt->auto_tcmd_total) < 0) return -1;
            if (json_field_str (&p, end, &first, "at_st",   evt->auto_tcmd_state) < 0) return -1;
        }
        // Operator-level warning (e.g., low disk). Emitted outside the
        // rx_have_session block so viewers see it even on builds without
        // an SDR — disk pressure matters either way.
        if (evt->rx_warning[0]) {
            if (json_field_str(&p, end, &first, "rx_warn",
                               evt->rx_warning) < 0) return -1;
        }
        // RX panel mirror. Only worth shipping when the operator has an
        // active rx_session; non-B210 broadcasts omit the whole block.
        if (evt->rx_have_session) {
            if (json_field_bool  (&p, end, &first, "rx_has", 1) < 0) return -1;
            if (evt->rx_rec_active) {
                if (json_field_bool(&p, end, &first, "rx_rec", 1) < 0) return -1;
            }
            if (json_field_double(&p, end, &first, "rx_fhz", evt->rx_freq_hz) < 0) return -1;
            if (json_field_double(&p, end, &first, "rx_pk",  evt->rx_peak_dbfs) < 0) return -1;
            if (json_field_double(&p, end, &first, "rx_rm",  evt->rx_rms_dbfs) < 0) return -1;
            if (json_field_int   (&p, end, &first, "rx_fr",  evt->rx_frames_total) < 0) return -1;
            if (evt->rx_frames_pcm) {
                if (json_field_int(&p, end, &first, "rx_fr_pcm",
                                   evt->rx_frames_pcm) < 0) return -1;
            }
            if (evt->rx_frames_vit) {
                if (json_field_int(&p, end, &first, "rx_fr_vt",
                                   evt->rx_frames_vit) < 0) return -1;
            }
            if (evt->rx_last_frame_summary[0]) {
                if (json_field_str(&p, end, &first, "rx_lf",
                                   evt->rx_last_frame_summary) < 0) return -1;
            }
            if (evt->rx_age_s >= 0.0) {
                if (json_field_double(&p, end, &first, "rx_age",
                                      evt->rx_age_s) < 0) return -1;
            }
            for (int s = 0; s < SSO_RX_PT_SLOTS; ++s) {
                if (evt->rx_pt_count[s] == 0) continue;
                char key[16];
                snprintf(key, sizeof key, "rx_pt%d_c", s);
                if (json_field_int(&p, end, &first, key,
                                   evt->rx_pt_count[s]) < 0) return -1;
                int n = evt->rx_pt_payload_len[s];
                if (n > 0) {
                    snprintf(key, sizeof key, "rx_pt%d_l", s);
                    if (json_field_int(&p, end, &first, key, n) < 0) return -1;
                    // Hex-encode the payload preview. Cap at the wire-
                    // declared max so a misbehaving sender can't blow
                    // through the receive buffer.
                    int max_b = (n < SSO_RX_PT_PAYLOAD_MAX)
                                ? n : SSO_RX_PT_PAYLOAD_MAX;
                    char hex[SSO_RX_PT_PAYLOAD_MAX * 2 + 1] = {0};
                    static const char hd[] = "0123456789ABCDEF";
                    for (int b = 0; b < max_b; ++b) {
                        hex[b * 2 + 0] = hd[(evt->rx_pt_payload[s][b] >> 4) & 0xF];
                        hex[b * 2 + 1] = hd[ evt->rx_pt_payload[s][b]       & 0xF];
                    }
                    hex[max_b * 2] = '\0';
                    snprintf(key, sizeof key, "rx_pt%d_p", s);
                    if (json_field_str(&p, end, &first, key, hex) < 0) return -1;
                }
                if (evt->rx_pt_summary[s][0]) {
                    snprintf(key, sizeof key, "rx_pt%d_s", s);
                    if (json_field_str(&p, end, &first, key,
                                       evt->rx_pt_summary[s]) < 0) return -1;
                }
            }
            if (evt->rx_ribbon_n > 0) {
                if (json_field_str(&p, end, &first, "rx_rb", evt->rx_ribbon) < 0)
                    return -1;
                // Parallel peak-dBFS array hex-encoded: each int8 → 2 hex
                // chars (two's complement so negative numbers round-trip).
                char hex[SSO_RIBBON_MAX * 2 + 1] = {0};
                static const char hd[] = "0123456789ABCDEF";
                int hn = evt->rx_ribbon_n;
                for (int i = 0; i < hn; ++i) {
                    uint8_t b = (uint8_t) evt->rx_ribbon_peak[i];
                    hex[i * 2 + 0] = hd[(b >> 4) & 0xF];
                    hex[i * 2 + 1] = hd[ b       & 0xF];
                }
                hex[hn * 2] = '\0';
                if (json_field_str(&p, end, &first, "rx_rb_p", hex) < 0)
                    return -1;
            }
        }
    }
    // rx-stats fields
    if (evt->type == SSO_EVT_RX_STATS) {
        if (json_field_double(&p, end, &first, "snr_db", evt->snr_db) < 0) return -1;
        if (json_field_int(&p, end, &first, "packets", evt->packets) < 0) return -1;
        if (json_field_str(&p, end, &first, "last_packet_ts", evt->last_packet_ts) < 0) return -1;
        if (json_field_str(&p, end, &first, "last_packet_summary", evt->last_packet_summary) < 0) return -1;
    }
    if (evt->type == SSO_EVT_TX_COMMAND_SENT
     || evt->type == SSO_EVT_TX_COMMAND_PREVIEW
     || evt->type == SSO_EVT_TX_REQUEST
     || evt->type == SSO_EVT_TX_NOT_SENT) {
        if (json_field_str(&p, end, &first, "ascii", evt->ascii) < 0) return -1;
        if (json_field_str(&p, end, &first, "tx_kind", evt->tx_payload_kind) < 0) return -1;
        if (json_field_str(&p, end, &first, "tx_pl",   evt->tx_payload) < 0) return -1;
        if (evt->tx_csp_src) {
            if (json_field_int(&p, end, &first, "tx_src", evt->tx_csp_src) < 0) return -1;
        }
        if (evt->tx_csp_dst) {
            if (json_field_int(&p, end, &first, "tx_dst", evt->tx_csp_dst) < 0) return -1;
        }
        if (evt->tx_csp_dport) {
            if (json_field_int(&p, end, &first, "tx_dp", evt->tx_csp_dport) < 0) return -1;
        }
        if (evt->tx_csp_sport) {
            if (json_field_int(&p, end, &first, "tx_sp", evt->tx_csp_sport) < 0) return -1;
        }
        if (evt->tx_csp_prio) {
            if (json_field_int(&p, end, &first, "tx_prio", evt->tx_csp_prio) < 0) return -1;
        }
        if (evt->tx_freq_hz) {
            if (json_field_int(&p, end, &first, "tx_freq", evt->tx_freq_hz) < 0) return -1;
        }
        if (evt->tx_gain_db != 0.0) {
            if (json_field_double(&p, end, &first, "tx_gain", evt->tx_gain_db) < 0) return -1;
        }
        if (evt->tx_allow_tx) {
            if (json_field_bool(&p, end, &first, "tx_allow", 1) < 0) return -1;
        }
        if (evt->tx_allow_high_power) {
            if (json_field_bool(&p, end, &first, "tx_hp", 1) < 0) return -1;
        }
        if (evt->tx_allow_hf_tx) {
            if (json_field_bool(&p, end, &first, "tx_hf", 1) < 0) return -1;
        }
        if (evt->tx_repeat) {
            if (json_field_int(&p, end, &first, "tx_rep", evt->tx_repeat) < 0) return -1;
        }
        if (evt->tx_gap_ms) {
            if (json_field_int(&p, end, &first, "tx_gap", evt->tx_gap_ms) < 0) return -1;
        }
        if (json_field_str(&p, end, &first, "tx_st", evt->tx_not_sent_reason) < 0) return -1;
        if (evt->tx_origin[0]) {
            if (json_field_str(&p, end, &first, "tx_org", evt->tx_origin) < 0) return -1;
        }
    }
    if (evt->type == SSO_EVT_CMD_PREVIEW || evt->type == SSO_EVT_CMD_EXECUTED) {
        if (json_field_str(&p, end, &first, "cmd_text",   evt->cmd_text)   < 0) return -1;
        if (json_field_str(&p, end, &first, "cmd_status", evt->cmd_status) < 0) return -1;
    }
    // Live-audio relay events (reason, if set, is already emitted above).
    if (evt->type == SSO_EVT_AUDIO_CTL) {
        if (json_field_bool(&p, end, &first, "enable", evt->audio_enable ? 1 : 0) < 0) return -1;
        if (evt->audio_quality > 0.0) {
            if (json_field_double(&p, end, &first, "q", evt->audio_quality) < 0) return -1;
        }
    }
    if (evt->type == SSO_EVT_AUDIO_STATUS) {
        if (json_field_str(&p, end, &first, "state", evt->audio_state) < 0) return -1;
        if (evt->audio_sr) {
            if (json_field_int(&p, end, &first, "sr", evt->audio_sr) < 0) return -1;
        }
        if (evt->audio_ch) {
            if (json_field_int(&p, end, &first, "ch", evt->audio_ch) < 0) return -1;
        }
    }
    if (evt->type == SSO_EVT_AUDIO) {
        // seq is always emitted (0 is valid — the first frame).
        if (json_field_int(&p, end, &first, "seq", evt->audio_seq) < 0) return -1;
        if (evt->audio_start) {
            if (json_field_bool(&p, end, &first, "start", 1) < 0) return -1;
            if (evt->audio_sr) {
                if (json_field_int(&p, end, &first, "sr", evt->audio_sr) < 0) return -1;
            }
            if (evt->audio_ch) {
                if (json_field_int(&p, end, &first, "ch", evt->audio_ch) < 0) return -1;
            }
        }
        if (json_field_str(&p, end, &first, "data", evt->audio_b64) < 0) return -1;
    }
    if (json_append(&p, end + 2, "}\n") < 0) return -1;
    *p = '\0';
    return 0;
}

int sso_event_decode(const char *line, sso_event_t *evt) {
    if (!line || !evt) return -1;
    memset(evt, 0, sizeof(*evt));
    // Measure the line once and thread it through every json_get_* below.
    // Each used to strlen the whole line, making decode O(fields x length).
    size_t line_len = strlen(line);
    char t[32];
    if (json_get_string(line, line_len, "t", t, sizeof(t)) <= 0) return -1;
    evt->type = sso_event_type_from_name(t);
    json_get_string(line, line_len, "ts", evt->ts, sizeof(evt->ts));
    json_get_string(line, line_len, "from", evt->from, sizeof(evt->from));
    json_get_string(line, line_len, "role", evt->role, sizeof(evt->role));
    json_get_string(line, line_len, "user", evt->user, sizeof(evt->user));
    json_get_string(line, line_len, "operator", evt->operator_user, sizeof(evt->operator_user));
    json_get_string(line, line_len, "prev", evt->prev, sizeof(evt->prev));
    json_get_string(line, line_len, "new", evt->new_user, sizeof(evt->new_user));
    json_get_string(line, line_len, "by", evt->by, sizeof(evt->by));
    json_get_string(line, line_len, "to", evt->to, sizeof(evt->to));
    int forced = 0;
    if (json_get_bool(line, line_len, "forced", &forced) > 0) evt->forced = forced;
    json_get_string(line, line_len, "reason", evt->reason, sizeof(evt->reason));
    json_get_string(line, line_len, "pass_folder", evt->pass_folder, sizeof(evt->pass_folder));

    if (json_get_string(line, line_len, "sat", evt->satellite, sizeof(evt->satellite)) > 0) {
        evt->has_state = 1;
    }
    if (json_get_string(line, line_len, "source", evt->source, sizeof(evt->source)) > 0) {
        evt->has_state = 1;
    }
    if (json_get_double(line, line_len, "az", &evt->az) > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "el", &evt->el) > 0) evt->has_state = 1;
    long freq;
    if (json_get_int(line, line_len, "freq", &freq) > 0) {
        evt->freq_hz = freq;
        evt->has_state = 1;
    }
    if (json_get_double(line, line_len, "doppler", &evt->doppler_hz) > 0) evt->has_state = 1;
    if (json_get_string(line, line_len, "rx_status", evt->rx_status, sizeof(evt->rx_status)) > 0) evt->has_state = 1;
    if (json_get_string(line, line_len, "tx_status", evt->tx_status, sizeof(evt->tx_status)) > 0) evt->has_state = 1;
    if (json_get_string(line, line_len, "tle_path", evt->tle_path, sizeof(evt->tle_path)) > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "target_az", &evt->target_az) > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "target_el", &evt->target_el) > 0) evt->has_state = 1;
    int flag = 0;
    if (json_get_bool(line, line_len, "flip",     &flag) > 0) evt->flip = flag;
    if (json_get_bool(line, line_len, "in_pass",  &flag) > 0) evt->in_pass = flag;
    if (json_get_bool(line, line_len, "tracking", &flag) > 0) evt->tracking = flag;
    if (json_get_double(line, line_len, "jul", &evt->jul_utc) > 0) evt->has_state = 1;
    int rotflag = 0;
    if (json_get_bool(line, line_len, "has_rot", &rotflag) > 0) evt->has_rotator = rotflag;
    if (json_get_string(line, line_len, "idesg", evt->idesg, sizeof evt->idesg) > 0)
        evt->has_state = 1;
    if (json_get_double(line, line_len, "ep_min", &evt->epoch_min)    > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "mv",     &evt->min_visible)  > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "ma0",    &evt->min_above_0)  > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "ma30",   &evt->min_above_30) > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "max_el", &evt->max_el)       > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "p_az",   &evt->pred_az)      > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "p_el",   &evt->pred_el)      > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "alt",    &evt->alt_km)       > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "lat",    &evt->lat_deg)      > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "lon",    &evt->lon_deg)      > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "spd",    &evt->speed_kms)    > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "rng",    &evt->range_km)     > 0) evt->has_state = 1;
    if (json_get_double(line, line_len, "rrate",  &evt->range_rate_kms) > 0) evt->has_state = 1;
    if (json_get_raw(line, line_len, "roster", evt->roster_json, sizeof(evt->roster_json)) > 0) {
        evt->has_state = 1;
    }
    // Auto-TCMD progress mirror. Absent fields stay zeroed (memset at
    // top of decode), which is what tells the viewer "no run to show".
    int at_flag = 0;
    if (json_get_bool(line, line_len, "at_on", &at_flag) > 0) evt->auto_tcmd_on = at_flag;
    long at_int = 0;
    if (json_get_int(line, line_len, "at_sent", &at_int) > 0) evt->auto_tcmd_sent  = (int) at_int;
    if (json_get_int(line, line_len, "at_tot",  &at_int) > 0) evt->auto_tcmd_total = (int) at_int;
    json_get_string(line, line_len, "at_st", evt->auto_tcmd_state, sizeof(evt->auto_tcmd_state));

    json_get_double(line, line_len, "snr_db", &evt->snr_db);
    json_get_int(line, line_len, "packets", &evt->packets);
    json_get_string(line, line_len, "last_packet_ts", evt->last_packet_ts, sizeof(evt->last_packet_ts));
    json_get_string(line, line_len, "last_packet_summary", evt->last_packet_summary, sizeof(evt->last_packet_summary));
    json_get_string(line, line_len, "ascii", evt->ascii, sizeof(evt->ascii));
    json_get_string(line, line_len, "tx_kind", evt->tx_payload_kind, sizeof(evt->tx_payload_kind));
    json_get_string(line, line_len, "tx_pl", evt->tx_payload, sizeof(evt->tx_payload));
    long tx_int = 0;
    if (json_get_int(line, line_len, "tx_src",  &tx_int) > 0) evt->tx_csp_src   = (uint8_t) tx_int;
    if (json_get_int(line, line_len, "tx_dst",  &tx_int) > 0) evt->tx_csp_dst   = (uint8_t) tx_int;
    if (json_get_int(line, line_len, "tx_dp",   &tx_int) > 0) evt->tx_csp_dport = (uint8_t) tx_int;
    if (json_get_int(line, line_len, "tx_sp",   &tx_int) > 0) evt->tx_csp_sport = (uint8_t) tx_int;
    if (json_get_int(line, line_len, "tx_prio", &tx_int) > 0) evt->tx_csp_prio  = (uint8_t) tx_int;
    if (json_get_int(line, line_len, "tx_freq", &tx_int) > 0) evt->tx_freq_hz   = tx_int;
    json_get_double(line, line_len, "tx_gain", &evt->tx_gain_db);
    int tx_flag = 0;
    if (json_get_bool(line, line_len, "tx_allow", &tx_flag) > 0) evt->tx_allow_tx         = tx_flag;
    if (json_get_bool(line, line_len, "tx_hp",    &tx_flag) > 0) evt->tx_allow_high_power = tx_flag;
    if (json_get_bool(line, line_len, "tx_hf",    &tx_flag) > 0) evt->tx_allow_hf_tx      = tx_flag;
    if (json_get_int(line, line_len, "tx_rep", &tx_int) > 0) evt->tx_repeat = (int) tx_int;
    if (json_get_int(line, line_len, "tx_gap", &tx_int) > 0) evt->tx_gap_ms = (int) tx_int;
    json_get_string(line, line_len, "tx_st", evt->tx_not_sent_reason, sizeof(evt->tx_not_sent_reason));
    json_get_string(line, line_len, "tx_org", evt->tx_origin, sizeof(evt->tx_origin));
    json_get_string(line, line_len, "cmd_text",   evt->cmd_text,   sizeof(evt->cmd_text));
    json_get_string(line, line_len, "cmd_status", evt->cmd_status, sizeof(evt->cmd_status));

    // Live-audio relay fields (absent on other events; stay zeroed).
    int au_flag = 0;
    if (json_get_bool(line, line_len, "enable", &au_flag) > 0) evt->audio_enable = au_flag;
    json_get_double(line, line_len, "q", &evt->audio_quality);
    json_get_string(line, line_len, "state", evt->audio_state, sizeof(evt->audio_state));
    long au_int = 0;
    if (json_get_int(line, line_len, "seq", &au_int) > 0) evt->audio_seq = (int) au_int;
    if (json_get_bool(line, line_len, "start", &au_flag) > 0) evt->audio_start = au_flag;
    if (json_get_int(line, line_len, "sr", &au_int) > 0) evt->audio_sr = (int) au_int;
    if (json_get_int(line, line_len, "ch", &au_int) > 0) evt->audio_ch = (int) au_int;
    json_get_string(line, line_len, "data", evt->audio_b64, sizeof(evt->audio_b64));

    // RX panel mirror. Absent fields stay zeroed (memset at top of decode).
    int rx_flag = 0;
    if (json_get_bool(line, line_len, "rx_has", &rx_flag) > 0) evt->rx_have_session = rx_flag;
    if (json_get_bool(line, line_len, "rx_rec", &rx_flag) > 0) evt->rx_rec_active   = rx_flag;
    json_get_double(line, line_len, "rx_fhz", &evt->rx_freq_hz);
    json_get_double(line, line_len, "rx_pk",  &evt->rx_peak_dbfs);
    json_get_double(line, line_len, "rx_rm",  &evt->rx_rms_dbfs);
    long rx_long = 0;
    if (json_get_int(line, line_len, "rx_fr", &rx_long) > 0) evt->rx_frames_total = rx_long;
    if (json_get_int(line, line_len, "rx_fr_pcm", &rx_long) > 0) evt->rx_frames_pcm = rx_long;
    if (json_get_int(line, line_len, "rx_fr_vt", &rx_long) > 0) evt->rx_frames_vit = rx_long;
    json_get_string(line, line_len, "rx_lf",
                    evt->rx_last_frame_summary,
                    sizeof(evt->rx_last_frame_summary));
    // rx_age may be absent (no frame yet) — leave -1 sentinel via the
    // memset, unless the field is present.
    if (json_get_double(line, line_len, "rx_age", &evt->rx_age_s) <= 0) {
        evt->rx_age_s = -1.0;
    }
    for (int s = 0; s < SSO_RX_PT_SLOTS; ++s) {
        char key[16];
        snprintf(key, sizeof key, "rx_pt%d_c", s);
        long c = 0;
        if (json_get_int(line, line_len, key, &c) > 0) evt->rx_pt_count[s] = c;
        snprintf(key, sizeof key, "rx_pt%d_l", s);
        long n = 0;
        if (json_get_int(line, line_len, key, &n) > 0) evt->rx_pt_payload_len[s] = (int) n;
        snprintf(key, sizeof key, "rx_pt%d_p", s);
        char hex[SSO_RX_PT_PAYLOAD_MAX * 2 + 1] = {0};
        if (json_get_string(line, line_len, key, hex, sizeof hex) > 0) {
            int hl = (int) strlen(hex);
            int bytes = hl / 2;
            if (bytes > SSO_RX_PT_PAYLOAD_MAX) bytes = SSO_RX_PT_PAYLOAD_MAX;
            for (int b = 0; b < bytes; ++b) {
                char hi = hex[b * 2], lo = hex[b * 2 + 1];
                int hv = (hi >= '0' && hi <= '9') ? (hi - '0')
                       : (hi >= 'A' && hi <= 'F') ? (hi - 'A' + 10)
                       : (hi >= 'a' && hi <= 'f') ? (hi - 'a' + 10) : 0;
                int lv = (lo >= '0' && lo <= '9') ? (lo - '0')
                       : (lo >= 'A' && lo <= 'F') ? (lo - 'A' + 10)
                       : (lo >= 'a' && lo <= 'f') ? (lo - 'a' + 10) : 0;
                evt->rx_pt_payload[s][b] = (uint8_t) ((hv << 4) | lv);
            }
        }
        snprintf(key, sizeof key, "rx_pt%d_s", s);
        json_get_string(line, line_len, key, evt->rx_pt_summary[s],
                        sizeof evt->rx_pt_summary[s]);
    }
    json_get_string(line, line_len, "rx_warn", evt->rx_warning, sizeof evt->rx_warning);
    if (json_get_string(line, line_len, "rx_rb",
                        evt->rx_ribbon, sizeof evt->rx_ribbon) > 0) {
        evt->rx_ribbon_n = (int) strlen(evt->rx_ribbon);
        if (evt->rx_ribbon_n > SSO_RIBBON_MAX) evt->rx_ribbon_n = SSO_RIBBON_MAX;
    }
    char rb_hex[SSO_RIBBON_MAX * 2 + 1] = {0};
    if (json_get_string(line, line_len, "rx_rb_p", rb_hex, sizeof rb_hex) > 0) {
        int hn = (int) strlen(rb_hex) / 2;
        if (hn > SSO_RIBBON_MAX) hn = SSO_RIBBON_MAX;
        for (int i = 0; i < hn; ++i) {
            char hi = rb_hex[i * 2], lo = rb_hex[i * 2 + 1];
            int hv = (hi >= '0' && hi <= '9') ? (hi - '0')
                   : (hi >= 'A' && hi <= 'F') ? (hi - 'A' + 10)
                   : (hi >= 'a' && hi <= 'f') ? (hi - 'a' + 10) : 0;
            int lv = (lo >= '0' && lo <= '9') ? (lo - '0')
                   : (lo >= 'A' && lo <= 'F') ? (lo - 'A' + 10)
                   : (lo >= 'a' && lo <= 'f') ? (lo - 'a' + 10) : 0;
            evt->rx_ribbon_peak[i] = (int8_t) ((hv << 4) | lv);
        }
    }
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
