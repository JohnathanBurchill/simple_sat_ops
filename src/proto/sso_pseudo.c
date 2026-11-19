/*

    Simple Satellite Operations  sso_pseudo.c

    See sso_pseudo.h. The shared parse mirrors the framing rules of a real
    telecommand (a name, a parenthesised argument list, a single terminating
    '!') so an SSO+ line is validated the same way the linter validates the
    CTS1+ command it expands into.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include "sso_pseudo.h"
#include "tcmd_lint.h"   // TCMD_RF_MAX_LEN

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 250 ms TX lag + 250 ms average on-board execution lag. Added to the
// queue-time clock so the satellite's clock lands close to true UTC by the
// time the command actually executes.
#define SSO_TIME_SYNC_BUDGET_MS 500

// One SSO+ command's expander: builds the concrete CTS1+ payload. The shared
// parser has already validated the prefix, framing, and arg count, so `args`
// (the raw text between the parens, possibly empty) is rarely needed. Returns
// SSO_PSEUDO_OK on success (writes a NUL-terminated CTS1+ string into `out`),
// or SSO_PSEUDO_TRUNCATED / SSO_PSEUDO_BAD_SYNTAX on failure.
// `route` arrives pre-filled with the OBC telecommand default; an expander
// aimed elsewhere overwrites it.
typedef sso_pseudo_status_t (*sso_pseudo_expand_fn)(
    const char *args, size_t args_len, const sso_pseudo_ctx_t *ctx,
    char *out, size_t out_cap, sso_pseudo_route_t *route,
    char *err, size_t err_cap);

typedef struct {
    const char           *name;      // exact, case-sensitive SSO+ command name
    int                   num_args;  // required count of comma-separated args
    sso_pseudo_expand_fn  expand;    // builds the CTS1+ replacement
    const char           *help;      // one-line description
} sso_pseudo_spec_t;

// Set the satellite clock to ground UTC, captured fresh at queue time:
//   arg     = ctx->now_ms + SSO_TIME_SYNC_BUDGET_MS  (full precision, per send)
//   @tssent = ctx->tssent_ms                         (pass start, minute-truncated)
// @tssent is deliberately the pass-start value, not the per-send clock: the
// firmware dedups on it, so a constant-per-session value runs the sync once per
// pass (and survives an hour/midnight crossing within the pass) while a later
// session re-runs it. See sso_pseudo.h.
static sso_pseudo_status_t expand_sync_sat_time_to_ground(
    const char *args, size_t args_len, const sso_pseudo_ctx_t *ctx,
    char *out, size_t out_cap, sso_pseudo_route_t *route,
    char *err, size_t err_cap)
{
    (void) args;
    (void) args_len;
    (void) route;
    (void) err;
    (void) err_cap;
    long long arg = ctx->now_ms + SSO_TIME_SYNC_BUDGET_MS;
    int n = snprintf(out, out_cap, "CTS1+set_system_time(%lld)@tssent=%lld!",
                     arg, ctx->tssent_ms);
    if (n < 0 || (size_t) n >= out_cap) return SSO_PSEUDO_TRUNCATED;
    return SSO_PSEUDO_OK;
}

static sso_pseudo_status_t fail(char *err, size_t err_cap,
                                sso_pseudo_status_t st, const char *text);

// Raw CSP to an arbitrary node/port: args are
// "<dst>,<dport>,<sport>,<hex payload>". The reply comes back to ground node
// 10 on <sport>. The hex is validated here so lint catches a bad line.
static sso_pseudo_status_t expand_raw_csp(
    const char *args, size_t args_len, const sso_pseudo_ctx_t *ctx,
    char *out, size_t out_cap, sso_pseudo_route_t *route,
    char *err, size_t err_cap)
{
    (void) ctx;
    char buf[512];
    if (args_len >= sizeof buf)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX, "SSO+raw_csp arguments too long");
    memcpy(buf, args, args_len);
    buf[args_len] = '\0';

    char *end;
    long dst = strtol(buf, &end, 10);
    if (end == buf || *end != ',' || dst < 0 || dst > 31)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "SSO+raw_csp dst must be a CSP node 0..31");
    char *p = end + 1;
    long dport = strtol(p, &end, 10);
    if (end == p || *end != ',' || dport < 0 || dport > 63)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "SSO+raw_csp dport must be a CSP port 0..63");
    p = end + 1;
    long sport = strtol(p, &end, 10);
    if (end == p || *end != ',' || sport < 0 || sport > 63)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "SSO+raw_csp sport must be a CSP port 0..63");
    const char *hex = end + 1;
    size_t hex_len = strlen(hex);
    if (hex_len == 0 || hex_len % 2 != 0)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "SSO+raw_csp payload must be a non-empty, even-length hex string");
    for (size_t k = 0; k < hex_len; ++k)
        if (!isxdigit((unsigned char) hex[k]))
            return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                        "SSO+raw_csp payload has a non-hex character");
    // Same RS-block capacity as an ASCII telecommand: one byte per char.
    if (hex_len / 2 > (size_t) TCMD_RF_MAX_LEN)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "SSO+raw_csp payload over the RF uplink limit");

    int n = snprintf(out, out_cap, "%s", hex);
    if (n < 0 || (size_t) n >= out_cap) return SSO_PSEUDO_TRUNCATED;
    if (route) {
        route->hdr.dst   = (uint8_t) dst;
        route->hdr.dport = (uint8_t) dport;
        route->hdr.sport = (uint8_t) sport;
        route->is_hex    = 1;
    }
    return SSO_PSEUDO_OK;
}

#define SSO_AX100_CSP_NODE 5   // FrontierSat's AX100 CSP node
#define CSP_PORT_PING      1   // libcsp service port: echoes the payload back
#define SSO_PING_SPORT     9   // not 16, so ping replies stand out from telecommand traffic

static sso_pseudo_status_t expand_ping_ax100(
    const char *args, size_t args_len, const sso_pseudo_ctx_t *ctx,
    char *out, size_t out_cap, sso_pseudo_route_t *route,
    char *err, size_t err_cap)
{
    (void) args;
    (void) args_len;
    char a[32];
    // Payload is ASCII "PING".
    snprintf(a, sizeof a, "%d,%d,%d,50494e47",
             SSO_AX100_CSP_NODE, CSP_PORT_PING, SSO_PING_SPORT);
    return expand_raw_csp(a, strlen(a), ctx, out, out_cap, route, err, err_cap);
}

// The registry. A future SSO+ command is one expander above + one row here.
static const sso_pseudo_spec_t SSO_PSEUDO_SPEC[] = {
    { "sync_sat_time_to_ground", 0, expand_sync_sat_time_to_ground,
      "set the satellite clock to ground UTC at TX time (+500 ms lag budget)" },
    { "ping_ax100", 0, expand_ping_ax100,
      "CSP ping (port 1, from port 9) to the AX100 radio" },
    { "raw_csp", 4, expand_raw_csp,
      "raw CSP packet: raw_csp(<dst node>,<dst port>,<src port>,<hex payload>)" },
};
static const size_t SSO_PSEUDO_SPEC_COUNT =
    sizeof SSO_PSEUDO_SPEC / sizeof SSO_PSEUDO_SPEC[0];

static int is_name_char(char c)
{
    return isalnum((unsigned char) c) || c == '_';
}

static const sso_pseudo_spec_t *spec_find(const char *name, size_t name_len)
{
    for (size_t i = 0; i < SSO_PSEUDO_SPEC_COUNT; ++i) {
        const char *cand = SSO_PSEUDO_SPEC[i].name;
        if (strlen(cand) == name_len && strncmp(cand, name, name_len) == 0) {
            return &SSO_PSEUDO_SPEC[i];
        }
    }
    return NULL;
}

static sso_pseudo_status_t fail(char *err, size_t err_cap,
                                sso_pseudo_status_t st, const char *text)
{
    if (err && err_cap) snprintf(err, err_cap, "%s", text);
    return st;
}

int sso_pseudo_is_directed(const char *cmd)
{
    return cmd != NULL
        && strncmp(cmd, SSO_PSEUDO_PREFIX, SSO_PSEUDO_PREFIX_LEN) == 0;
}

sso_pseudo_status_t sso_pseudo_expand(const char *cmd, const sso_pseudo_ctx_t *ctx,
                                      char *out, size_t out_cap,
                                      sso_pseudo_route_t *route,
                                      char *err, size_t err_cap)
{
    if (err && err_cap) err[0] = '\0';
    if (out_cap) out[0] = '\0';
    if (route) *route = (sso_pseudo_route_t){
        .hdr = { .prio = CSP_PRIO_NORM, .src = 10, .dst = 1,
                 .dport = 7, .sport = 16, .flags = 0 },
        .is_hex = 0,
    };
    if (cmd == NULL || ctx == NULL || out == NULL || out_cap == 0)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX, "null argument");

    // Not an SSO+ line: hand the caller a verbatim copy so it can use `out`
    // unconditionally.
    if (!sso_pseudo_is_directed(cmd)) {
        size_t L = strlen(cmd);
        if (L >= out_cap) L = out_cap - 1;
        memcpy(out, cmd, L);
        out[L] = '\0';
        return SSO_PSEUDO_NOT_PSEUDO;
    }

    size_t n = strlen(cmd);

    // Command name: from after the prefix up to the first non-name char.
    size_t i = SSO_PSEUDO_PREFIX_LEN;
    while (i < n && is_name_char(cmd[i])) i++;
    size_t name_len = i - SSO_PSEUDO_PREFIX_LEN;
    if (name_len == 0)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "missing command name after 'SSO+'");

    // Exactly one terminating '!' after the prefix (mirrors the firmware /
    // tcmd_lint rule: zero means unterminated, more than one means two
    // commands smashed onto one line).
    int bang_count = 0;
    for (size_t j = SSO_PSEUDO_PREFIX_LEN; j < n; ++j)
        if (cmd[j] == '!') bang_count++;
    if (bang_count == 0)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX, "missing terminating '!'");
    if (bang_count > 1)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "more than one '!' (two commands on one line?)");

    // Opening paren must immediately follow the name; a closing paren must
    // exist (the first ')' closes the list, as the firmware takes it).
    if (i >= n || cmd[i] != '(')
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "missing '(' after SSO+ command name");
    const char *close = strchr(cmd + i, ')');
    if (!close)
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX,
                    "missing ')' to close the SSO+ argument list");
    size_t open_idx  = i;
    size_t close_idx = (size_t) (close - cmd);
    const char *args = cmd + open_idx + 1;
    size_t args_len  = close_idx - open_idx - 1;

    // Argument count, exactly as the firmware computes it: 0 when the parens
    // are empty, otherwise (commas + 1).
    const sso_pseudo_spec_t *spec = spec_find(cmd + SSO_PSEUDO_PREFIX_LEN, name_len);
    if (!spec) {
        char m[96];
        snprintf(m, sizeof m, "unknown SSO+ command '%.*s'",
                 (int) name_len, cmd + SSO_PSEUDO_PREFIX_LEN);
        return fail(err, err_cap, SSO_PSEUDO_UNKNOWN_CMD, m);
    }
    int num_commas = 0;
    for (size_t j = open_idx + 1; j < close_idx; ++j)
        if (cmd[j] == ',') num_commas++;
    int provided = (args_len == 0) ? 0 : (num_commas + 1);
    if (provided != spec->num_args) {
        char m[96];
        snprintf(m, sizeof m, "SSO+ '%s' expects %d arg%s, got %d",
                 spec->name, spec->num_args,
                 spec->num_args == 1 ? "" : "s", provided);
        return fail(err, err_cap, SSO_PSEUDO_BAD_SYNTAX, m);
    }

    return spec->expand(args, args_len, ctx, out, out_cap, route, err, err_cap);
}

const char *sso_pseudo_status_label(sso_pseudo_status_t st)
{
    switch (st) {
        case SSO_PSEUDO_OK:          return "ok";
        case SSO_PSEUDO_NOT_PSEUDO:  return "not an SSO+ command";
        case SSO_PSEUDO_UNKNOWN_CMD: return "unknown SSO+ command";
        case SSO_PSEUDO_BAD_SYNTAX:  return "malformed SSO+ command";
        case SSO_PSEUDO_TRUNCATED:   return "expansion too long";
    }
    return "unknown";
}
