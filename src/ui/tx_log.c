/*

   Simple Satellite Operations  ui/tx_log.c

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

#include "tx_log.h"
#include "state.h"

#include "sso_ipc.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

// Pull "HH:MM:SS" out of an event's ISO ts ("2026-05-14T13:22:01.450Z").
// Falls back to local clock if the event ts is empty/garbled.
static void tx_log_ts_from_event(const sso_event_t *evt,
                                 char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    if (evt && evt->ts[0]) {
        const char *t = strchr(evt->ts, 'T');
        if (t && strlen(t) >= 9) {
            size_t n = 8;
            if (n >= out_size) n = out_size - 1;
            memcpy(out, t + 1, n);
            out[n] = '\0';
            return;
        }
    }
    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    snprintf(out, out_size, "%02d:%02d:%02d",
             lt.tm_hour, lt.tm_min, lt.tm_sec);
}

// Append one event to <pass_folder>/tx.log as a JSON line. Opens the
// file lazily; no-op when state->op.pass_folder isn't set yet (so events that
// arrive before pass-folder bring-up land in the in-memory ring but
// aren't dropped silently — they just don't reach disk until the
// folder exists). fflush after every write so a SIGKILL mid-pass
// preserves the last command sent.
static void tx_log_file_append(state_t *state, const sso_event_t *evt)
{
    if (!evt) return;
    if (state->tx.tx_log_fp == NULL) {
        if (state->op.pass_folder[0] == '\0') return;
        char path[512];
        snprintf(path, sizeof path, "%.500s/tx.log", state->op.pass_folder);
        FILE *fp = fopen(path, "a");
        if (!fp) return;
        snprintf(state->tx.tx_log_path, sizeof state->tx.tx_log_path, "%s", path);
        state->tx.tx_log_fp = fp;
    }
    char buf[2048];
    if (sso_event_encode(evt, buf, sizeof buf) != 0) return;
    // sso_event_encode already terminates with "}\n" — don't add another.
    fputs(buf, state->tx.tx_log_fp);
    fflush(state->tx.tx_log_fp);
}

// Push an event into the ring. PREVIEW events overwrite a trailing
// PREVIEW entry (live cursor-style update). SENT promotes a trailing
// PREVIEW to SENT, or appends a fresh entry. NOT_SENT appends with the
// status string filled in for rendering.
void tx_log_push(state_t *state, const sso_event_t *evt)
{
    if (!evt) return;
    if (evt->type != SSO_EVT_TX_COMMAND_PREVIEW
     && evt->type != SSO_EVT_TX_COMMAND_SENT
     && evt->type != SSO_EVT_TX_NOT_SENT) return;

    // The on-disk tx.log is the record of what was actually keyed, so only
    // sent / not-sent commands go to the file. PREVIEW (draft) lines still
    // land in the in-memory ring below so the live TX panel shows the draft
    // as the operator types -- they just don't pollute the persistent log.
    if (evt->type != SSO_EVT_TX_COMMAND_PREVIEW) {
        tx_log_file_append(state, evt);
    }

    tx_log_entry_t entry;
    memset(&entry, 0, sizeof entry);
    entry.kind = evt->type;
    tx_log_ts_from_event(evt, entry.ts, sizeof entry.ts);
    snprintf(entry.ascii, sizeof entry.ascii, "%s", evt->ascii);
    snprintf(entry.tx_not_sent_reason, sizeof entry.tx_not_sent_reason, "%s",
             evt->tx_not_sent_reason);
    snprintf(entry.source, sizeof entry.source, "%s", evt->tx_origin);

    if (evt->type == SSO_EVT_TX_COMMAND_PREVIEW
        && state->tx.tx_log_count > 0
        && state->tx.tx_log[state->tx.tx_log_count - 1].kind == SSO_EVT_TX_COMMAND_PREVIEW) {
        state->tx.tx_log[state->tx.tx_log_count - 1] = entry;
        return;
    }
    if (evt->type == SSO_EVT_TX_COMMAND_SENT
        && state->tx.tx_log_count > 0
        && state->tx.tx_log[state->tx.tx_log_count - 1].kind == SSO_EVT_TX_COMMAND_PREVIEW) {
        // Promote the trailing draft in-place.
        state->tx.tx_log[state->tx.tx_log_count - 1] = entry;
        return;
    }
    if (state->tx.tx_log_count < TX_LOG_SIZE) {
        state->tx.tx_log[state->tx.tx_log_count++] = entry;
    } else {
        memmove(&state->tx.tx_log[0], &state->tx.tx_log[1],
                sizeof(state->tx.tx_log[0]) * (TX_LOG_SIZE - 1));
        state->tx.tx_log[TX_LOG_SIZE - 1] = entry;
    }
}
