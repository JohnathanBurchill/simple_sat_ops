/*

    Simple Satellite Operations  unit_tests/keybindings_selftest.c

    Coverage for src/ui/keybindings.{c,h} -- the single source of truth for
    the operator keyboard. The table feeds three consumers (the dispatcher,
    the on-screen legend, and --help-full), and the whole point of the table
    is that they can't drift apart. Issue #35 was exactly that drift: the
    "Track" legend line got overpainted and the t / A help lines had gone
    missing. These tests pin the table's integrity and the dispatcher's lock
    gating so a future edit can't quietly reintroduce either.

    The real action bodies live in ui/input.c and call into the entire
    operator stack (tracking / scan / the TX modals / ncurses). We don't link
    that; instead we provide stub kb_act_* definitions that just record which
    action a key routed to. The linker resolves keybindings.c's table against
    these stubs, so the test stays light and exercises only the routing logic.

    What's covered:
      - No key is claimed by two rows (a real dispatch ambiguity).
      - Every row has keys + an action; every *visible* row has a combo and a
        description (an empty legend cell is the #35 failure mode).
      - keybindings_visible_count matches the non-hidden row count.
      - Dispatch routes a key to the matching action and passes the actual
        key through (the jog rows need it to pick a direction).
      - The keyboard lock gates ordinary keys but not KB_ALWAYS (K).
      - Unbound keys and ERR / NUL are ignored.
      - --help-full lists t (Compose TX) and A (Auto-TCMD) -- the lines the old
        hand-written help had dropped -- and omits the hidden ":" row.

    Exit status: 0 = all tests passed, non-zero = failure.

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

#include "keybindings.h"
#include "state.h"
#include "tap.h"

#include <stdio.h>
#include <string.h>

// Stub action bodies (see file header). Record which one fired and with
// what key, so dispatch routing can be checked without the real handlers.
static const char *g_last_action;
static int         g_last_key;
static int         g_call_count;

#define STUB_ACTION(name)                                   \
    void name(state_t *state, int key) {                    \
        (void) state;                                       \
        g_last_action = #name;                              \
        g_last_key    = key;                                \
        g_call_count++;                                     \
    }

STUB_ACTION(kb_act_track)
STUB_ACTION(kb_act_stop)
STUB_ACTION(kb_act_reset)
STUB_ACTION(kb_act_jog)
STUB_ACTION(kb_act_tx_compose)
STUB_ACTION(kb_act_auto_tcmd)
STUB_ACTION(kb_act_lock)
STUB_ACTION(kb_act_quit)
STUB_ACTION(kb_act_cmdline)

static void reset_calls(void)
{
    g_last_action = NULL;
    g_last_key    = 0;
    g_call_count  = 0;
}

// ------------------------------------------------------------------
// 1. No key is claimed by two rows.
// ------------------------------------------------------------------

static void test_no_duplicate_keys(void)
{
    size_t n = 0;
    const keybinding_t *t = keybindings_table(&n);
    int seen[256] = {0};
    int dup = 0;
    char dup_ch = '?';
    for (size_t i = 0; i < n; ++i) {
        for (const char *k = t[i].keys; k && *k; ++k) {
            unsigned char c = (unsigned char) *k;
            if (seen[c]) { dup = 1; dup_ch = *k; }
            seen[c] = 1;
        }
    }
    tap_okf(!dup, "table: no key claimed by two rows (dup='%c')",
            dup ? dup_ch : '-');
}

// ------------------------------------------------------------------
// 2. Rows are well-formed; visible rows carry a legend label.
// ------------------------------------------------------------------

static void test_rows_well_formed(void)
{
    size_t n = 0;
    const keybinding_t *t = keybindings_table(&n);
    int rows_bad = 0, labels_bad = 0;
    int nonhidden = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!(t[i].keys && t[i].keys[0]) || t[i].action == NULL) rows_bad = 1;
        if (!(t[i].flags & KB_HIDDEN)) {
            nonhidden++;
            if (!(t[i].combo && t[i].combo[0] && t[i].desc && t[i].desc[0]))
                labels_bad = 1;
        }
    }
    tap_ok(!rows_bad, "table: every row has keys and an action");
    tap_ok(!labels_bad, "table: every visible row has a combo and a desc");
    tap_okf(keybindings_visible_count() == nonhidden,
            "table: visible_count == non-hidden rows (%d)", nonhidden);

    // The Track row is the one issue #35 lost from the legend -- make sure
    // it is present and visible.
    int has_track = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!(t[i].flags & KB_HIDDEN) && t[i].desc
            && strstr(t[i].desc, "Track")) has_track = 1;
    }
    tap_ok(has_track, "table: a visible row tracks the satellite (issue #35)");
}

// ------------------------------------------------------------------
// 3. Dispatch routing + key pass-through.
// ------------------------------------------------------------------

static void test_dispatch_routing(void)
{
    state_t st;
    memset(&st, 0, sizeof st);
    st.ui.keyboard_unlocked = 1;

    reset_calls();
    keybindings_dispatch(&st, 'T');
    tap_ok(g_last_action && strcmp(g_last_action, "kb_act_track") == 0,
           "dispatch: 'T' routes to the track action");

    reset_calls();
    keybindings_dispatch(&st, 't');
    tap_ok(g_last_action && strcmp(g_last_action, "kb_act_tx_compose") == 0,
           "dispatch: 't' (case-sensitive) routes to TX compose");

    // The jog rows share one action and rely on the actual key for direction.
    reset_calls();
    keybindings_dispatch(&st, '[');
    tap_ok(g_last_action && strcmp(g_last_action, "kb_act_jog") == 0
           && g_last_key == '[',
           "dispatch: '[' routes to jog and passes the key through");
    reset_calls();
    keybindings_dispatch(&st, '>');
    tap_ok(g_last_action && strcmp(g_last_action, "kb_act_jog") == 0
           && g_last_key == '>',
           "dispatch: '>' routes to jog with key '>'");

    // Unbound key: no action.
    reset_calls();
    keybindings_dispatch(&st, 'Z');
    tap_ok(g_call_count == 0, "dispatch: unbound key 'Z' fires nothing");

    // ERR (-1) and NUL must not match a row's terminating '\0'.
    reset_calls();
    keybindings_dispatch(&st, -1);
    keybindings_dispatch(&st, 0);
    tap_ok(g_call_count == 0, "dispatch: ERR and NUL are ignored");
}

// ------------------------------------------------------------------
// 4. Keyboard lock gates ordinary keys but not KB_ALWAYS.
// ------------------------------------------------------------------

static void test_lock_gating(void)
{
    state_t st;
    memset(&st, 0, sizeof st);
    st.ui.keyboard_unlocked = 0;   // LOCKED

    reset_calls();
    keybindings_dispatch(&st, 'q');
    keybindings_dispatch(&st, 'T');
    keybindings_dispatch(&st, '[');
    tap_ok(g_call_count == 0, "lock: ordinary keys do nothing while locked");

    reset_calls();
    keybindings_dispatch(&st, 'K');
    tap_ok(g_last_action && strcmp(g_last_action, "kb_act_lock") == 0,
           "lock: 'K' fires while locked (KB_ALWAYS)");

    // Unlocked: the same ordinary key now fires.
    st.ui.keyboard_unlocked = 1;
    reset_calls();
    keybindings_dispatch(&st, 'q');
    tap_ok(g_last_action && strcmp(g_last_action, "kb_act_quit") == 0,
           "lock: 'q' fires once unlocked");
}

// ------------------------------------------------------------------
// 5. --help-full listing is complete and hides the ":" row.
// ------------------------------------------------------------------

static void test_print_help(void)
{
    FILE *fp = tmpfile();
    if (!fp) { tap_bail("tmpfile"); return; }
    keybindings_print_help(fp);
    long sz = ftell(fp);
    if (sz <= 0 || sz > 4096) { fclose(fp); tap_bail("ftell"); return; }
    rewind(fp);
    char buf[4096];
    size_t got = fread(buf, 1, sizeof buf - 1, fp);
    buf[got] = '\0';
    fclose(fp);

    tap_ok(strstr(buf, "Track satellite") != NULL,
           "help: lists T (Track) -- the legend entry issue #35 lost");
    tap_ok(strstr(buf, "Compose TX command") != NULL,
           "help: lists t (Compose TX) -- the old --help-full omission");
    tap_ok(strstr(buf, "Auto-TCMD") != NULL,
           "help: lists A (Auto-TCMD) -- the old --help-full omission");
    tap_ok(strstr(buf, "Lock/unlock keyboard") != NULL,
           "help: lists K (lock)");
    tap_ok(strstr(buf, "Command line") == NULL,
           "help: omits the hidden ':' row");
}

int main(void)
{
    test_no_duplicate_keys();
    test_rows_well_formed();
    test_dispatch_routing();
    test_lock_gating();
    test_print_help();
    return tap_done();
}
