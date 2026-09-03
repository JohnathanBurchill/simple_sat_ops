/*

   Simple Satellite Operations  src/hw/tr_switch_find.c

   See tr_switch_find.h for what this is for.

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

#include "tr_switch_find.h"
#include "sso_paths.h"
#include "sso_time.h"

#include <ctype.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Linux names its USB-CDC ports ttyACM*, macOS names them cu.usbmodem*.
#define LINUX_PORT_PREFIX "ttyACM"
#define MACOS_PORT_PREFIX "cu.usbmodem"

// How long a probe sleeps between reads while it waits for a heartbeat.
#define PROBE_POLL_NS (20L * 1000L * 1000L)

// ---- reading what the operating system says about a port -------------------

// Read the first line of a sysfs attribute into `out`, without the
// newline. Returns 0 on success, -1 when the file isn't there.
static int read_attr(const char *path, char *out, size_t cap)
{
    if (out == NULL || cap == 0) return -1;
    out[0] = '\0';
    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;
    char *got = fgets(out, (int) cap, f);
    fclose(f);
    if (got == NULL) return -1;
    size_t n = strlen(out);
    while (n > 0 && (out[n - 1] == '\n' || out[n - 1] == '\r' || out[n - 1] == ' ')) {
        out[--n] = '\0';
    }
    return 0;
}

// The USB descriptors sit on the device the port's interface hangs off,
// which sysfs reaches as <port>/device/.. — read one of them as hex.
static unsigned read_usb_id(const char *sysfs_root, const char *name,
                            const char *attr)
{
    char path[512];
    snprintf(path, sizeof path, "%s/class/tty/%s/device/../%s",
             sysfs_root, name, attr);
    char buf[32];
    if (read_attr(path, buf, sizeof buf) != 0) return 0;
    return (unsigned) strtoul(buf, NULL, 16);
}

static void read_usb_string(const char *sysfs_root, const char *name,
                            const char *attr, char *out, size_t cap)
{
    char path[512];
    snprintf(path, sizeof path, "%s/class/tty/%s/device/../%s",
             sysfs_root, name, attr);
    if (read_attr(path, out, cap) != 0) out[0] = '\0';
}

static void add_candidate(tr_switch_candidate_t *out, int cap, int *n,
                          const tr_switch_candidate_t *c)
{
    if (*n >= cap) return;
    out[*n] = *c;
    (*n)++;
}

// Linux: every ttyACM* under /sys/class/tty, with whatever the USB
// descriptors say about it. A port whose /dev node hasn't been made yet
// is skipped -- there is nothing to open.
static void scan_sysfs(const char *sysfs_root, const char *dev_root,
                       tr_switch_candidate_t *out, int cap, int *n)
{
    char dirpath[256];
    snprintf(dirpath, sizeof dirpath, "%s/class/tty", sysfs_root);
    DIR *d = opendir(dirpath);
    if (d == NULL) return;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        // Sysfs writes these as symlinks; go by the name rather than the
        // directory-entry type.
        if (strncmp(e->d_name, LINUX_PORT_PREFIX, strlen(LINUX_PORT_PREFIX)) != 0)
            continue;
        tr_switch_candidate_t c = {0};
        // A name too long to hold is a port we could not open anyway.
        int len = snprintf(c.path, sizeof c.path, "%s/%s", dev_root, e->d_name);
        if (len < 0 || (size_t) len >= sizeof c.path) continue;
        if (access(c.path, F_OK) != 0) continue;
        c.vid = read_usb_id(sysfs_root, e->d_name, "idVendor");
        c.pid = read_usb_id(sysfs_root, e->d_name, "idProduct");
        read_usb_string(sysfs_root, e->d_name, "product", c.product, sizeof c.product);
        read_usb_string(sysfs_root, e->d_name, "serial",  c.serial,  sizeof c.serial);
        add_candidate(out, cap, n, &c);
    }
    closedir(d);
}

// macOS: the cu.usbmodem* nodes in /dev, which carry no descriptors --
// these can only be told apart by listening to them.
static void scan_dev(const char *dev_root,
                     tr_switch_candidate_t *out, int cap, int *n)
{
    DIR *d = opendir(dev_root);
    if (d == NULL) return;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, MACOS_PORT_PREFIX, strlen(MACOS_PORT_PREFIX)) != 0)
            continue;
        tr_switch_candidate_t c = {0};
        int len = snprintf(c.path, sizeof c.path, "%s/%s", dev_root, e->d_name);
        if (len < 0 || (size_t) len >= sizeof c.path) continue;
        add_candidate(out, cap, n, &c);
    }
    closedir(d);
}

// Path order, so the order a machine offers its directory entries in
// doesn't change which port is tried first.
static int by_path(const void *va, const void *vb)
{
    const tr_switch_candidate_t *a = va, *b = vb;
    return strcmp(a->path, b->path);
}

int tr_switch_scan(const char *sysfs_root, const char *dev_root,
                   tr_switch_candidate_t *out, int cap)
{
    if (out == NULL || cap <= 0) return 0;
    if (sysfs_root == NULL) sysfs_root = "/sys";
    if (dev_root   == NULL) dev_root   = "/dev";
    int n = 0;
    scan_sysfs(sysfs_root, dev_root, out, cap, &n);
    scan_dev(dev_root, out, cap, &n);
    qsort(out, (size_t) n, sizeof out[0], by_path);
    return n;
}

// ---- remembering the board across runs -------------------------------------

int tr_switch_state_path(char *out, size_t cap)
{
    if (out == NULL || cap == 0) return -1;
    out[0] = '\0';
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') return -1;
    int n = snprintf(out, cap, "%s/%s", home, TR_SWITCH_STATE_RELPATH);
    return (n > 0 && (size_t) n < cap) ? 0 : -1;
}

// Whitespace off both ends of a line, in place, the newline with it.
static char *trim(char *s)
{
    while (*s != '\0' && isspace((unsigned char) *s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char) e[-1])) e--;
    *e = '\0';
    return s;
}

int tr_switch_memo_read(const char *path, tr_switch_memo_t *m)
{
    if (path == NULL || m == NULL) return -1;
    memset(m, 0, sizeof *m);
    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;
    char line[256];
    while (fgets(line, sizeof line, f) != NULL) {
        char *s = trim(line);
        if (s[0] == '\0' || s[0] == '#') continue;
        char *eq = strchr(s, '=');
        if (eq == NULL) continue;
        *eq = '\0';
        char *key = trim(s);
        char *val = trim(eq + 1);
        if      (strcmp(key, "serial")  == 0) snprintf(m->serial,  sizeof m->serial,  "%s", val);
        else if (strcmp(key, "product") == 0) snprintf(m->product, sizeof m->product, "%s", val);
        else if (strcmp(key, "path")    == 0) snprintf(m->path,    sizeof m->path,    "%s", val);
        else if (strcmp(key, "vid")     == 0) m->vid = (unsigned) strtoul(val, NULL, 16);
        else if (strcmp(key, "pid")     == 0) m->pid = (unsigned) strtoul(val, NULL, 16);
        // Any other key belongs to another build; leave it alone.
    }
    fclose(f);
    return 0;
}

int tr_switch_memo_write(const char *path, const tr_switch_memo_t *m)
{
    if (path == NULL || m == NULL) return -1;
    if (sso_mkdir_p_for_file(path) != 0) return -1;
    FILE *f = fopen(path, "w");
    if (f == NULL) return -1;
    fprintf(f,
            "# simple_sat_ops: the T/R antenna switch as it was last found.\n"
            "# Written by the startup search; delete this file to make the\n"
            "# next run search from scratch. serial identifies the board,\n"
            "# path is only where it was last plugged in.\n");
    if (m->serial [0] != '\0') fprintf(f, "serial = %s\n",  m->serial);
    if (m->product[0] != '\0') fprintf(f, "product = %s\n", m->product);
    if (m->vid != 0)           fprintf(f, "vid = %04x\n",   m->vid);
    if (m->pid != 0)           fprintf(f, "pid = %04x\n",   m->pid);
    if (m->path   [0] != '\0') fprintf(f, "path = %s\n",    m->path);
    return fclose(f) == 0 ? 0 : -1;
}

tr_switch_memo_hit_t tr_switch_memo_pick(const tr_switch_memo_t *m,
                                         const tr_switch_candidate_t *c, int n,
                                         int *index)
{
    if (m == NULL || c == NULL || index == NULL) return TR_SWITCH_MEMO_NONE;
    // The serial is the only field that names one board rather than a
    // model, so it is the only one worth opening a port on trust.
    if (m->serial[0] != '\0') {
        for (int i = 0; i < n; i++) {
            if (strcmp(c[i].serial, m->serial) == 0) { *index = i; return TR_SWITCH_MEMO_SERIAL; }
        }
    }
    if (m->path[0] != '\0') {
        for (int i = 0; i < n; i++) {
            if (strcmp(c[i].path, m->path) == 0) { *index = i; return TR_SWITCH_MEMO_PATH; }
        }
    }
    return TR_SWITCH_MEMO_NONE;
}

// ---- finding and opening ---------------------------------------------------

// Open a port and wait for a heartbeat. Returns 0 with the link still
// open, -1 with it closed again.
static int listen_for_heartbeat(tr_switch_t *s)
{
    double t0 = monotonic_seconds();
    for (;;) {
        double t = monotonic_seconds();
        tr_switch_pump(s, t);
        if (s->heartbeat_count > 0) return 0;
        // The pump drops the link itself on a read error.
        if (!s->connected) return -1;
        if (t - t0 > TR_SWITCH_PROBE_SECONDS) break;
        struct timespec nap = { 0, PROBE_POLL_NS };
        nanosleep(&nap, NULL);
    }
    tr_switch_disconnect(s);
    return -1;
}

// Remember what answered, so the next run looks there first. Best
// effort: a state file that can't be written costs nothing but a slower
// search next time.
static void remember(const char *path, const tr_switch_candidate_t *c)
{
    if (path == NULL) return;
    tr_switch_memo_t m = {0};
    snprintf(m.serial,  sizeof m.serial,  "%s", c->serial);
    snprintf(m.product, sizeof m.product, "%s", c->product);
    snprintf(m.path,    sizeof m.path,    "%s", c->path);
    m.vid = c->vid;
    m.pid = c->pid;
    tr_switch_memo_write(path, &m);
}

int tr_switch_open_detected(tr_switch_t *s, char *path_out, size_t path_cap,
                            tr_switch_candidate_t *found, int *remembered,
                            char *looked_at, size_t looked_cap)
{
    char state_file[512];
    const char *state = tr_switch_state_path(state_file, sizeof state_file) == 0
                      ? state_file : NULL;
    return tr_switch_open_found(s, NULL, NULL, state, path_out, path_cap,
                                found, remembered, looked_at, looked_cap);
}

int tr_switch_open_found(tr_switch_t *s,
                         const char *sysfs_root, const char *dev_root,
                         const char *state_file,
                         char *path_out, size_t path_cap,
                         tr_switch_candidate_t *found, int *remembered,
                         char *looked_at, size_t looked_cap)
{
    if (s == NULL || path_out == NULL || path_cap == 0) return -1;
    path_out[0] = '\0';
    if (found != NULL) memset(found, 0, sizeof *found);
    if (remembered != NULL) *remembered = 0;
    if (looked_at != NULL && looked_cap > 0) looked_at[0] = '\0';

    tr_switch_candidate_t cand[TR_SWITCH_MAX_CANDIDATES];
    int n = tr_switch_scan(sysfs_root, dev_root, cand, TR_SWITCH_MAX_CANDIDATES);

    // What a previous run found, if anything, and which port that is now.
    tr_switch_memo_t memo = {0};
    int memo_idx = -1;
    tr_switch_memo_hit_t hit = TR_SWITCH_MEMO_NONE;
    if (state_file != NULL && tr_switch_memo_read(state_file, &memo) == 0) {
        hit = tr_switch_memo_pick(&memo, cand, n, &memo_idx);
    }

    // The remembered port first, then the rest in path order.
    for (int k = 0; k < n; k++) {
        int i = k;
        if (memo_idx >= 0) {
            if (k == 0) i = memo_idx;
            else if (k <= memo_idx) i = k - 1;
        }
        if (looked_at != NULL && looked_cap > 0) {
            size_t at = strlen(looked_at);
            snprintf(looked_at + at, looked_cap - at, "%s%s",
                     at > 0 ? ", " : "", cand[i].path);
        }
        snprintf(path_out, path_cap, "%s", cand[i].path);
        s->device_filename = path_out;
        if (tr_switch_init(s) != 0) continue;
        // The remembered board, named by its serial, is taken as read.
        // Everything else has to say something first.
        int is_remembered_board = (i == memo_idx && hit == TR_SWITCH_MEMO_SERIAL);
        if (is_remembered_board || listen_for_heartbeat(s) == 0) {
            if (found != NULL) *found = cand[i];
            if (remembered != NULL) *remembered = (i == memo_idx);
            // Write it back even when it was remembered: the port it is
            // plugged into can move between runs, and the descriptors can
            // fill in on a machine that didn't have them before.
            remember(state_file, &cand[i]);
            return 0;
        }
    }

    s->device_filename = NULL;
    path_out[0] = '\0';
    return -1;
}
