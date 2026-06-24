/*

   Simple Satellite Operations  control/operator_audio.c  — see operator_audio.h.

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

#include "operator_audio.h"
#include "state.h"

#include "ogg_stream.h"
#include "sso_base64.h"
#include "rx_session.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// PCM samples drained from the RX ring per inner pump iteration. ~43 ms at
// the 96 kHz post-decim rate — small enough to keep frames flowing, large
// enough that the per-frame JSON overhead stays negligible.
#define AUDIO_PUMP_BATCH 4096

// Default VBR quality when a viewer doesn't specify one. Low: this is a
// monitoring path over SSH, not hi-fi.
#define AUDIO_DEFAULT_QUALITY 0.2

typedef struct {
    int               in_use;
    sso_client_id_t   id;
    ogg_stream_t     *enc;
    int               seq;        // next frame's seq (per-stream, from 0)
    int               started;    // 1 once the first (start) frame went out
    int               sr;
    int               ch;
    // Set right before each encode so the sink (called from inside
    // ogg_stream_write) knows where to target the frame.
    sso_ipc_server_t *srv;
} audio_sub_t;

static audio_sub_t g_subs[SSO_AUDIO_MAX_SUBS];
static int         g_n;   // subscribers currently in use

static int find_slot(sso_client_id_t id)
{
    for (int i = 0; i < SSO_AUDIO_MAX_SUBS; ++i)
        if (g_subs[i].in_use && g_subs[i].id == id) return i;
    return -1;
}

static int find_free_slot(void)
{
    for (int i = 0; i < SSO_AUDIO_MAX_SUBS; ++i)
        if (!g_subs[i].in_use) return i;
    return -1;
}

static void send_status(sso_ipc_server_t *srv, sso_client_id_t id,
                        const char *st, const char *reason, int sr, int ch)
{
    sso_event_t e;
    sso_event_init(&e, SSO_EVT_AUDIO_STATUS);
    snprintf(e.audio_state, sizeof e.audio_state, "%s", st);
    if (reason) snprintf(e.reason, sizeof e.reason, "%s", reason);
    e.audio_sr = sr;
    e.audio_ch = ch;
    char line[SSO_IPC_LINE_MAX];
    if (sso_event_encode(&e, line, sizeof line) == 0) {
        sso_ipc_server_send(srv, id, line);
    }
}

// libsndfile hands us encoded Ogg bytes here (from inside ogg_stream_write,
// or during the encoder open for the header pages). Slice into wire-sized
// chunks, base64 each, and targeted-send it to this subscriber. Best-effort:
// a failed send is dropped (the viewer sees a seq gap and the decoder
// resyncs) — the prune sweep removes a subscriber that has truly gone.
static long audio_sub_sink(const uint8_t *bytes, size_t n, void *user)
{
    audio_sub_t *s = (audio_sub_t *) user;
    size_t off = 0;
    do {
        size_t chunk = n - off;
        if (chunk > SSO_AUDIO_RAW_MAX) chunk = SSO_AUDIO_RAW_MAX;
        sso_event_t e;
        sso_event_init(&e, SSO_EVT_AUDIO);
        e.audio_seq = s->seq++;
        if (!s->started) {
            e.audio_start = 1;
            e.audio_sr    = s->sr;
            e.audio_ch    = s->ch;
            s->started    = 1;
        }
        int is_start = e.audio_start;
        sso_base64_encode(bytes + off, chunk, e.audio_b64, sizeof e.audio_b64);
        char line[SSO_IPC_LINE_MAX];
        if (sso_event_encode(&e, line, sizeof line) == 0) {
            // The start frame carries the Ogg/Vorbis headers — without them
            // the viewer can't decode anything, so send it reliably (the
            // buffer is empty at subscribe time, so it won't overflow).
            // Ongoing frames are best-effort: a stalled link drops a frame
            // (the decoder resyncs at the next page) instead of the whole
            // subscriber, so audio + telemetry both survive the hiccup.
            if (is_start) sso_ipc_server_send(s->srv, s->id, line);
            else          sso_ipc_server_send_lossy(s->srv, s->id, line);
        }
        off += chunk;
    } while (off < n);
    return (long) n;
}

static void drop_slot(int i)
{
    if (!g_subs[i].in_use) return;
    if (g_subs[i].enc) ogg_stream_close(g_subs[i].enc);
    memset(&g_subs[i], 0, sizeof g_subs[i]);
    if (g_n > 0) g_n--;
}

// Turn the RX session's PCM tap on/off to match whether anyone is listening.
static void update_tap(state_t *state)
{
#ifdef SSO_WITH_SDR
    if (state->sdr.rx_session) rx_session_set_audio_tap(state->sdr.rx_session, g_n > 0);
#else
    (void) state;
#endif
}

void operator_audio_handle_ctl(state_t *state, sso_client_id_t id,
                               const sso_event_t *evt)
{
    sso_ipc_server_t *srv = state->ipc;
    if (!srv) return;

    if (!evt->audio_enable) {
        int i = find_slot(id);
        if (i >= 0) drop_slot(i);
        update_tap(state);
        send_status(srv, id, "off", NULL, 0, 0);
        return;
    }

    if (state->sdr.no_audio) {
        send_status(srv, id, "unavailable", "audio disabled", 0, 0);
        return;
    }

#ifdef SSO_WITH_SDR
    if (state->sdr.rx_session == NULL) {
        send_status(srv, id, "unavailable", "no receiver", 0, 0);
        return;
    }
    int sr = (int) lround(rx_session_get_bandwidth_hz(state->sdr.rx_session));
    if (sr <= 0) sr = 96000;
    int    ch = 1;
    double q  = (evt->audio_quality > 0.0) ? evt->audio_quality
                                           : AUDIO_DEFAULT_QUALITY;

    int i = find_slot(id);
    int is_new = (i < 0);
    if (is_new) {
        i = find_free_slot();
        if (i < 0) {
            send_status(srv, id, "unavailable", "too many listeners", 0, 0);
            return;
        }
    }
    audio_sub_t *s = &g_subs[i];
    // Restart from clean state (a re-enable replaces any existing stream,
    // so the viewer always gets a fresh start frame with the headers).
    if (s->enc) ogg_stream_close(s->enc);
    s->id      = id;
    s->sr      = sr;
    s->ch      = ch;
    s->seq     = 0;
    s->started = 0;
    s->srv     = srv;
    s->enc     = ogg_stream_open(sr, ch, q, audio_sub_sink, s);
    if (!s->enc) {
        // No libsndfile in this build (or encoder init failed).
        if (!is_new) { s->in_use = 0; if (g_n > 0) g_n--; }
        update_tap(state);
        send_status(srv, id, "unavailable",
                    "no audio support in this build", 0, 0);
        return;
    }
    if (is_new) { s->in_use = 1; g_n++; }
    update_tap(state);
    send_status(srv, id, "on", NULL, sr, ch);
#else
    (void) evt;
    send_status(srv, id, "unavailable", "no receiver", 0, 0);
#endif
}

void operator_audio_pump(state_t *state)
{
#ifdef SSO_WITH_SDR
    if (g_n == 0 || state->sdr.rx_session == NULL) return;
    int16_t pcm[AUDIO_PUMP_BATCH];
    size_t got;
    while ((got = rx_session_read_audio(state->sdr.rx_session, pcm,
                                        AUDIO_PUMP_BATCH)) > 0) {
        for (int i = 0; i < SSO_AUDIO_MAX_SUBS; ++i) {
            if (!g_subs[i].in_use) continue;
            g_subs[i].srv = state->ipc;
            // The sink sends frames as bytes come out of the encoder.
            ogg_stream_write(g_subs[i].enc, pcm, got);
        }
        if (got < AUDIO_PUMP_BATCH) break;   // ring drained
    }
#else
    (void) state;
#endif
}

void operator_audio_prune(state_t *state)
{
    if (g_n == 0 || state->ipc == NULL) return;
    int alive[SSO_AUDIO_MAX_SUBS] = {0};
    sso_ipc_iter_t it = {0};
    sso_client_id_t cid;
    char u[64], r[16], sc[40];
    while (sso_ipc_server_next_client(state->ipc, &it, &cid,
                                      u, sizeof u, r, sizeof r,
                                      sc, sizeof sc) == 0) {
        int i = find_slot(cid);
        if (i >= 0) alive[i] = 1;
    }
    int dropped = 0;
    for (int i = 0; i < SSO_AUDIO_MAX_SUBS; ++i) {
        if (g_subs[i].in_use && !alive[i]) { drop_slot(i); dropped = 1; }
    }
    if (dropped) update_tap(state);
}

int operator_audio_active(void)
{
    return g_n > 0;
}

void operator_audio_shutdown(state_t *state)
{
    for (int i = 0; i < SSO_AUDIO_MAX_SUBS; ++i) drop_slot(i);
    g_n = 0;
    update_tap(state);
}
