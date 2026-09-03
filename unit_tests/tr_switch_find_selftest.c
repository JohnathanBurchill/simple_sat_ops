/*

    Simple Satellite Operations  unit_tests/tr_switch_find_selftest.c

    Coverage for src/hw/tr_switch_find.c: which of a machine's USB serial
    ports are candidates at all, how the board a previous run found is
    remembered and matched back to a port, and -- against a stand-in for
    the switch on a pseudo-terminal -- the whole search end to end.
    Getting any of it wrong is quiet and bad: miss the switch and the
    operator is told there isn't one, or point the search at the wrong
    port and startup opens somebody else's hardware and sends it bytes.

    Nothing here names a vendor or product id as "the switch": the
    firmware leaves the Pico SDK's stock descriptors in place, so no id
    in the source could identify this board. The ids and serials below
    are sample data standing in for whatever the station's board reports,
    and the code is expected to treat them as opaque.

    The scan takes its two directory roots as arguments, so each test
    builds a small tree under /tmp and points the scan at that. The tree
    is laid out the way Linux really lays sysfs out, because the code
    depends on that shape: the port directory holds a `device` symlink to
    the USB *interface*, and the vendor, product and serial attributes
    live one level above it, which is why the paths read here go through
    "device/..". A test tree that put the attributes next to the port
    would pass while the real thing found nothing.

    What's covered:
      Picking ports out of a tree (tr_switch_scan):
        - a ttyACM* port with a /dev node is a candidate; one whose node
          hasn't been made is not.
        - ttyS0 and ttyUSB0 are never candidates -- the rotator lives on
          ttyUSB and must not be reachable from here.
        - a macOS cu.usbmodem* node is a candidate; the matching tty.*
          node is not (opening that one blocks on carrier detect), and
          neither is cu.Bluetooth-Incoming-Port.
        - a port entry that is a symlink (as real sysfs writes it) and one
          that is a directory are both read, so nothing here can start
          filtering on the directory-entry type.
        - vendor and product ids are parsed as hex out of "device/..",
          and the product and serial strings come back with the trailing
          newline sysfs writes stripped.
        - a port with no attribute files reads as unknown (0, "") rather
          than failing the scan.
        - ports come back in path order whatever order the directory
          offers them in.
        - more ports than the caller's array holds returns exactly `cap`
          and writes nothing past it; a NULL array, a zero cap and an
          empty tree all return 0.
      Remembering the board (tr_switch_memo_read/write):
        - a memo round-trips through the file, ids included.
        - comments, blank lines, odd spacing and an unknown key are all
          tolerated; a line with no '=' is skipped.
        - a missing file fails rather than returning a half-filled memo.
        - the state path is under $HOME, and fails with no $HOME.
      Matching it back (tr_switch_memo_pick):
        - the remembered serial wins even when the board has moved to a
          different port -- that is the whole point of storing it.
        - a memo with no serial falls back to the remembered path, and
          reports the weaker match so the caller still probes.
        - a remembered serial that isn't attached falls back to the path;
          a memo matching nothing attached, and an empty memo, pick
          nothing -- so a stale file can misdirect the order of the
          search but never send it outside the scanned ports.

      The whole search, against a simulated switch (tr_switch_open_found,
      driving tr_switch.c's open / wake-up / parse as well):
        - with two ports answering, the one that beats is kept and the
          one that only talks is handed back.
        - the heartbeat is parsed out of the firmware's own line format,
          log_level= field and all, so state and mode come back right.
        - what answered is written to the state file, serial and port.
        - the next run opens the remembered board at once: no probe, no
          wait, and the other port is never opened at all.
        - a board moved to a different port is still found by its serial,
          and the remembered port is brought up to date.
        - when nothing beats, nothing is left open and nothing is
          written down.

    These four wait on real 2500 ms heartbeat periods and one full 4 s
    probe window, so this binary takes about eleven seconds.

    Not covered: tr_switch_open_detected() itself, which is the same
    search with the real /sys, /dev and $HOME resolved -- the seam these
    tests drive is one call inside it.

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

// glibc keeps posix_openpt / grantpt / unlockpt / ptsname behind a
// feature macro, so without this they arrive on the ground machine as
// implicit declarations -- and an implicit ptsname returns int, which
// truncates the pty's path to garbage. Apple's headers declare them
// regardless, which is why the Mac build and the gcc lint both stayed
// quiet. Must come before every include.
#define _GNU_SOURCE

#include "tr_switch_find.h"
#include "tap.h"

#include "sso_time.h"

#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

// ---- building a fake /sys + /dev tree --------------------------------------

typedef struct {
    char root[64];
    char sys [80];
    char dev [80];
} tree_t;

// Compose a path. A truncated one would build a tree the tests then
// quietly pass against, so it bails the run instead.
static void join(char *out, size_t cap, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out, cap, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t) n >= cap) tap_bail("a temp path outgrew the test's buffers");
}

static int make_dir(const char *path)
{
    if (mkdir(path, 0755) == 0) return 0;
    return -1;
}

static int make_dirs(const char *fmt, ...)
{
    char path[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(path, sizeof path, fmt, ap);
    va_end(ap);
    // Walk the path making each component in turn.
    for (char *p = path + 1; *p != '\0'; ++p) {
        if (*p != '/') continue;
        *p = '\0';
        mkdir(path, 0755);
        *p = '/';
    }
    mkdir(path, 0755);
    struct stat st;
    return (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) ? 0 : -1;
}

static int write_file(const char *path, const char *text)
{
    FILE *f = fopen(path, "wb");
    if (f == NULL) return -1;
    fputs(text, f);
    return fclose(f) == 0 ? 0 : -1;
}

static int write_attr(const char *dir, const char *name, const char *text)
{
    char path[512];
    join(path, sizeof path, "%s/%s", dir, name);
    return write_file(path, text);
}

static void rm_rf(const char *path)
{
    DIR *d = opendir(path);
    if (d != NULL) {
        struct dirent *e;
        while ((e = readdir(d)) != NULL) {
            if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) continue;
            char child[512];
            join(child, sizeof child, "%s/%s", path, e->d_name);
            // lstat, so a symlink to a directory is unlinked, not walked.
            struct stat st;
            if (lstat(child, &st) == 0 && S_ISDIR(st.st_mode)) rm_rf(child);
            else unlink(child);
        }
        closedir(d);
    }
    rmdir(path);
    unlink(path);
}

static int tree_open(tree_t *t)
{
    // Kept short and under /tmp on purpose: the scan builds device paths
    // into a TR_SWITCH_PATH_MAX buffer, and a long temp root would be
    // truncated rather than tested.
    snprintf(t->root, sizeof t->root, "/tmp/trsw_find_selftest_XXXXXX");
    if (mkdtemp(t->root) == NULL) return -1;
    join(t->sys, sizeof t->sys, "%s/sys", t->root);
    join(t->dev, sizeof t->dev, "%s/dev", t->root);
    if (make_dirs("%s/class/tty", t->sys) != 0) return -1;
    if (make_dir(t->dev) != 0) return -1;
    return 0;
}

static void tree_close(tree_t *t)
{
    rm_rf(t->root);
}

// Add one USB serial port to the tree, laid out as Linux lays it out:
// the USB device directory carries the descriptors, the interface
// directory sits inside it, and the port's `device` entry points at the
// interface. `vid`/`pid` are written as the four hex digits sysfs uses,
// and a NULL string means that attribute file is simply absent.
//
// `as_symlink` chooses how the port entry itself is made: real sysfs
// writes a symlink, so one port in these tests is a symlink and another
// a plain directory, and both have to be read.
static void tree_add_usb_port(tree_t *t, const char *port, int as_symlink,
                              unsigned vid, unsigned pid,
                              const char *product, const char *manufacturer,
                              const char *serial, int make_dev_node)
{
    char usbdev[512], iface[512], portdir[512];
    join(usbdev, sizeof usbdev, "%s/devices/usb_%s", t->sys, port);
    join(iface,  sizeof iface,  "%s/%s:1.0", usbdev, port);
    make_dirs("%s", iface);

    char buf[32];
    if (vid != 0) { snprintf(buf, sizeof buf, "%04x\n", vid); write_attr(usbdev, "idVendor", buf); }
    if (pid != 0) { snprintf(buf, sizeof buf, "%04x\n", pid); write_attr(usbdev, "idProduct", buf); }
    // sysfs strings come with a trailing newline; the reader has to strip it.
    if (product != NULL) {
        char line[128];
        snprintf(line, sizeof line, "%s\n", product);
        write_attr(usbdev, "product", line);
    }
    if (manufacturer != NULL) {
        char line[128];
        snprintf(line, sizeof line, "%s\n", manufacturer);
        write_attr(usbdev, "manufacturer", line);
    }
    if (serial != NULL) {
        char line[128];
        snprintf(line, sizeof line, "%s\n", serial);
        write_attr(usbdev, "serial", line);
    }

    join(portdir, sizeof portdir, "%s/class/tty/%s", t->sys, port);
    if (as_symlink) {
        // What real sysfs holds: the port entry is itself a symlink into
        // the devices tree, and `device` inside it points at the interface.
        char target[512];
        join(target, sizeof target, "../../devices/usb_%s/%s:1.0/tty/%s",
             port, port, port);
        char real[512];
        join(real, sizeof real, "%s/tty/%s", iface, port);
        make_dirs("%s", real);
        symlink(target, portdir);
        char devlink[512];
        join(devlink, sizeof devlink, "%s/device", real);
        symlink("../..", devlink);
    } else {
        make_dirs("%s", portdir);
        char devlink[512];
        join(devlink, sizeof devlink, "%s/device", portdir);
        // From the port directory, the interface is three levels up and
        // back down through devices/.
        char target[512];
        join(target, sizeof target, "../../../devices/usb_%s/%s:1.0",
             port, port);
        symlink(target, devlink);
    }

    if (make_dev_node) {
        char node[512];
        join(node, sizeof node, "%s/%s", t->dev, port);
        write_file(node, "");
    }
}

// A bare /dev node with no sysfs behind it -- what macOS offers.
static void tree_add_dev_node(tree_t *t, const char *name)
{
    char node[512];
    join(node, sizeof node, "%s/%s", t->dev, name);
    write_file(node, "");
}

static int ends_with(const char *s, const char *suffix)
{
    size_t sl = strlen(s), xl = strlen(suffix);
    return sl >= xl && strcmp(s + sl - xl, suffix) == 0;
}

static const tr_switch_candidate_t *find_by_name(const tr_switch_candidate_t *c,
                                                 int n, const char *suffix)
{
    for (int i = 0; i < n; i++) {
        if (ends_with(c[i].path, suffix)) return &c[i];
    }
    return NULL;
}

// Sample descriptors. Two RP2040 boards of the same build, which is the
// case the serial exists for: everything except the serial is identical.
#define SW_VID     0x2e8a
#define SW_PID     0x000a
#define SW_PRODUCT "Pico"
#define SW_MAKER   "Raspberry Pi"
#define SW_SERIAL  "E66178758B3F7C2B"
#define OTHER_SERIAL "E6614C775B392A21"

// ---- tests: finding the ports ----------------------------------------------

// A ground station with the switch and a dev board plugged in. ttyACM1 is
// written as a symlink and ttyACM0 as a directory, so both entry shapes
// are read, and everything the descriptors carry has to come back.
static void test_scan_reads_descriptors(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    tree_add_usb_port(&t, "ttyACM0", 0, 0x0483, 0x374b, "STM32 STLink",
                      "STMicroelectronics", "0672FF485550755187092345", 1);
    tree_add_usb_port(&t, "ttyACM1", 1, SW_VID, SW_PID, SW_PRODUCT,
                      SW_MAKER, SW_SERIAL, 1);

    tr_switch_candidate_t c[TR_SWITCH_MAX_CANDIDATES];
    int n = tr_switch_scan(t.sys, t.dev, c, TR_SWITCH_MAX_CANDIDATES);
    tap_okf(n == 2, "two ports found (got %d, want 2)", n);

    const tr_switch_candidate_t *sw = find_by_name(c, n, "/ttyACM1");
    const tr_switch_candidate_t *st = find_by_name(c, n, "/ttyACM0");
    tap_ok(sw != NULL, "the symlinked port entry is among the candidates");
    tap_ok(st != NULL, "the plain-directory port entry is among the candidates");
    if (sw == NULL || st == NULL) { tree_close(&t); return; }

    tap_okf(sw->vid == SW_VID && sw->pid == SW_PID,
            "ids read through device/.. (got %04x:%04x, want %04x:%04x)",
            sw->vid, sw->pid, SW_VID, SW_PID);
    tap_okf(strcmp(sw->product, SW_PRODUCT) == 0,
            "product string read without its newline (got \"%s\")", sw->product);
    tap_okf(strcmp(sw->serial, SW_SERIAL) == 0,
            "serial read without its newline (got \"%s\")", sw->serial);
    tap_okf(st->vid == 0x0483 && st->pid == 0x374b,
            "the other board's ids read too (got %04x:%04x, want 0483:374b)",
            st->vid, st->pid);
    tap_okf(ends_with(c[0].path, "/ttyACM0"),
            "ports come back in path order (first is %s)", c[0].path);
    tree_close(&t);
}

// A port can come with nothing to go on, which must not upset the scan.
static void test_scan_port_without_descriptors(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    tree_add_usb_port(&t, "ttyACM0", 0, 0, 0, NULL, NULL, NULL, 1);

    tr_switch_candidate_t c[TR_SWITCH_MAX_CANDIDATES];
    int n = tr_switch_scan(t.sys, t.dev, c, TR_SWITCH_MAX_CANDIDATES);
    tap_okf(n == 1, "a bare port is still a candidate (got %d, want 1)", n);
    if (n == 1) {
        tap_okf(c[0].vid == 0 && c[0].pid == 0
                && c[0].product[0] == '\0' && c[0].serial[0] == '\0',
                "its descriptors read as unknown (got %04x:%04x \"%s\" \"%s\")",
                c[0].vid, c[0].pid, c[0].product, c[0].serial);
    }
    tree_close(&t);
}

// What must not be picked up: ports of the wrong kind, and a port whose
// /dev node isn't there to open.
static void test_scan_rejects(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    // The rotator's own kind of port, wearing the same descriptors.
    tree_add_usb_port(&t, "ttyUSB0", 0, SW_VID, SW_PID, SW_PRODUCT, SW_MAKER,
                      SW_SERIAL, 1);
    tree_add_usb_port(&t, "ttyS0",   0, 0, 0, NULL, NULL, NULL, 1);
    // A real CDC port, but udev hasn't made the node.
    tree_add_usb_port(&t, "ttyACM4", 0, SW_VID, SW_PID, NULL, NULL, NULL, 0);
    // And one that is there, so the scan has something to return.
    tree_add_usb_port(&t, "ttyACM5", 0, SW_VID, SW_PID, NULL, NULL, NULL, 1);

    tr_switch_candidate_t c[TR_SWITCH_MAX_CANDIDATES];
    int n = tr_switch_scan(t.sys, t.dev, c, TR_SWITCH_MAX_CANDIDATES);
    tap_okf(n == 1, "only the openable CDC port is a candidate (got %d, want 1)", n);
    tap_ok(find_by_name(c, n, "/ttyACM5") != NULL, "the openable port is the one returned");
    tap_ok(find_by_name(c, n, "/ttyUSB0") == NULL,
           "a ttyUSB port is never a candidate, whatever it reports");
    tap_ok(find_by_name(c, n, "/ttyS0") == NULL, "a plain serial port is not a candidate");
    tap_ok(find_by_name(c, n, "/ttyACM4") == NULL, "a port with no /dev node is not a candidate");
    tree_close(&t);
}

// macOS: bare /dev nodes, no descriptors, and only the callout device.
static void test_scan_macos_nodes(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    tree_add_dev_node(&t, "cu.usbmodem14301");
    tree_add_dev_node(&t, "cu.usbmodem14201");
    tree_add_dev_node(&t, "tty.usbmodem14201");
    tree_add_dev_node(&t, "cu.Bluetooth-Incoming-Port");
    tree_add_dev_node(&t, "cu.wlan-debug");

    tr_switch_candidate_t c[TR_SWITCH_MAX_CANDIDATES];
    int n = tr_switch_scan(t.sys, t.dev, c, TR_SWITCH_MAX_CANDIDATES);
    tap_okf(n == 2, "both usbmodem callout nodes found (got %d, want 2)", n);
    tap_ok(find_by_name(c, n, "/tty.usbmodem14201") == NULL,
           "the tty.* twin is left alone (it blocks on carrier detect)");
    tap_ok(find_by_name(c, n, "/cu.Bluetooth-Incoming-Port") == NULL,
           "the Bluetooth serial port is not a candidate");
    if (n == 2) {
        tap_okf(ends_with(c[0].path, "/cu.usbmodem14201"),
                "they come in path order (first is %s)", c[0].path);
        tap_ok(c[0].serial[0] == '\0' && c[0].vid == 0,
               "a /dev-only port carries no descriptors");
    }
    tree_close(&t);
}

// The caller's array is a fixed size and the scan must respect it.
static void test_scan_bounds_and_guards(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    tr_switch_candidate_t empty[TR_SWITCH_MAX_CANDIDATES];
    tap_ok(tr_switch_scan(t.sys, t.dev, empty, TR_SWITCH_MAX_CANDIDATES) == 0,
           "a machine with no USB serial ports finds none");
    tap_ok(tr_switch_scan(t.sys, t.dev, NULL, TR_SWITCH_MAX_CANDIDATES) == 0,
           "a NULL array finds none");
    tap_ok(tr_switch_scan(t.sys, t.dev, empty, 0) == 0, "a zero cap finds none");
    tap_ok(tr_switch_scan("/nonexistent", "/nonexistent", empty,
                          TR_SWITCH_MAX_CANDIDATES) == 0,
           "roots that aren't there find none");

    for (int i = 0; i < 6; i++) {
        char port[16];
        snprintf(port, sizeof port, "ttyACM%d", i);
        tree_add_usb_port(&t, port, 0, 0, 0, NULL, NULL, NULL, 1);
    }
    // One slot past the cap, holding a sentinel the scan must not touch.
    tr_switch_candidate_t c[4];
    memset(c, 0, sizeof c);
    snprintf(c[3].path, sizeof c[3].path, "untouched");
    int n = tr_switch_scan(t.sys, t.dev, c, 3);
    tap_okf(n == 3, "six ports into an array of three returns three (got %d)", n);
    tap_okf(strcmp(c[3].path, "untouched") == 0,
            "nothing is written past the cap (slot 3 holds \"%s\")", c[3].path);
    tree_close(&t);
}

// ---- tests: remembering the board ------------------------------------------

static void memo_tmp_path(char *out, size_t cap)
{
    join(out, cap, "/tmp/trsw_memo_selftest_XXXXXX");
    int fd = mkstemp(out);
    if (fd >= 0) close(fd);
}

static void test_memo_round_trip(void)
{
    char path[80];
    memo_tmp_path(path, sizeof path);

    tr_switch_memo_t w = {0};
    snprintf(w.serial,  sizeof w.serial,  "%s", SW_SERIAL);
    snprintf(w.product, sizeof w.product, "%s", SW_PRODUCT);
    snprintf(w.path,    sizeof w.path,    "/dev/ttyACM1");
    w.vid = SW_VID;
    w.pid = SW_PID;
    tap_ok(tr_switch_memo_write(path, &w) == 0, "the memo writes");

    tr_switch_memo_t r = {0};
    tap_ok(tr_switch_memo_read(path, &r) == 0, "the memo reads back");
    tap_okf(strcmp(r.serial, SW_SERIAL) == 0, "serial round-trips (got \"%s\")", r.serial);
    tap_okf(strcmp(r.product, SW_PRODUCT) == 0, "product round-trips (got \"%s\")", r.product);
    tap_okf(strcmp(r.path, "/dev/ttyACM1") == 0, "path round-trips (got \"%s\")", r.path);
    tap_okf(r.vid == SW_VID && r.pid == SW_PID,
            "ids round-trip as hex (got %04x:%04x)", r.vid, r.pid);
    unlink(path);
}

static void test_memo_parser(void)
{
    char path[80];
    memo_tmp_path(path, sizeof path);

    // Comments, blank lines, odd spacing, a key from some other build,
    // and a line with no '=' at all.
    write_file(path,
               "# written by hand\n"
               "\n"
               "   serial =   " SW_SERIAL "   \n"
               "vid=2E8A\n"
               "path = /dev/ttyACM3\n"
               "future_key = 42\n"
               "this line has no equals sign\n");
    tr_switch_memo_t m = {0};
    tap_ok(tr_switch_memo_read(path, &m) == 0, "a hand-edited memo reads");
    tap_okf(strcmp(m.serial, SW_SERIAL) == 0,
            "spaces around the value are trimmed (got \"%s\")", m.serial);
    tap_okf(m.vid == 0x2e8a, "uppercase hex is read (got %04x)", m.vid);
    tap_okf(strcmp(m.path, "/dev/ttyACM3") == 0, "path is read (got \"%s\")", m.path);
    tap_ok(m.pid == 0 && m.product[0] == '\0',
           "a key the file omits stays empty");
    unlink(path);

    tr_switch_memo_t none = {0};
    snprintf(none.serial, sizeof none.serial, "stale");
    tap_ok(tr_switch_memo_read("/tmp/trsw_memo_selftest_does_not_exist", &none) == -1,
           "a missing memo file fails");
    tap_ok(none.serial[0] == '\0',
           "and clears the caller's memo rather than leaving it half-filled");
}

static void test_state_path(void)
{
    char *saved = getenv("HOME");
    char home[256] = {0};
    int had_home = saved != NULL;
    if (had_home) snprintf(home, sizeof home, "%s", saved);

    setenv("HOME", "/tmp/trsw_home", 1);
    char path[256];
    tap_ok(tr_switch_state_path(path, sizeof path) == 0, "the state path resolves");
    tap_okf(strncmp(path, "/tmp/trsw_home/", 15) == 0
            && ends_with(path, "tr_switch.state"),
            "it sits under $HOME (got %s)", path);

    unsetenv("HOME");
    tap_ok(tr_switch_state_path(path, sizeof path) == -1, "with no $HOME it fails");
    char tiny[8];
    setenv("HOME", "/tmp/trsw_home", 1);
    tap_ok(tr_switch_state_path(tiny, sizeof tiny) == -1,
           "a buffer too small to hold it fails rather than truncating");

    if (had_home) setenv("HOME", home, 1);
    else unsetenv("HOME");
}

// The board is remembered by serial precisely so it can be found again
// after the ports have shuffled.
static void test_memo_pick(void)
{
    tr_switch_candidate_t c[3] = {0};
    snprintf(c[0].path, sizeof c[0].path, "/dev/ttyACM0");
    snprintf(c[0].serial, sizeof c[0].serial, "%s", OTHER_SERIAL);
    c[0].vid = SW_VID; c[0].pid = SW_PID;
    snprintf(c[1].path, sizeof c[1].path, "/dev/ttyACM1");
    snprintf(c[1].serial, sizeof c[1].serial, "%s", SW_SERIAL);
    c[1].vid = SW_VID; c[1].pid = SW_PID;
    snprintf(c[2].path, sizeof c[2].path, "/dev/cu.usbmodem14201");

    // Remembered on ttyACM2 last time, but the board is on ttyACM1 now.
    tr_switch_memo_t m = {0};
    snprintf(m.serial, sizeof m.serial, "%s", SW_SERIAL);
    snprintf(m.path, sizeof m.path, "/dev/ttyACM2");
    int idx = -1;
    tr_switch_memo_hit_t hit = tr_switch_memo_pick(&m, c, 3, &idx);
    tap_ok(hit == TR_SWITCH_MEMO_SERIAL, "a remembered serial is a certain match");
    tap_okf(idx == 1, "and finds the board where it is now, not where it was (idx %d)", idx);

    // A memo from a machine with no descriptors: path only.
    tr_switch_memo_t p = {0};
    snprintf(p.path, sizeof p.path, "/dev/cu.usbmodem14201");
    idx = -1;
    hit = tr_switch_memo_pick(&p, c, 3, &idx);
    tap_ok(hit == TR_SWITCH_MEMO_PATH,
           "a path-only memo is the weaker match, so the caller still probes");
    tap_okf(idx == 2, "and points at the remembered port (idx %d)", idx);

    // The remembered board is unplugged, but its old port is in use by
    // something else: fall back to the port, weakly.
    tr_switch_memo_t gone = {0};
    snprintf(gone.serial, sizeof gone.serial, "NOT-ATTACHED");
    snprintf(gone.path, sizeof gone.path, "/dev/ttyACM0");
    idx = -1;
    hit = tr_switch_memo_pick(&gone, c, 3, &idx);
    tap_ok(hit == TR_SWITCH_MEMO_PATH && idx == 0,
           "an absent serial falls back to the remembered port");

    // Nothing the memo names is attached, and an empty memo.
    tr_switch_memo_t nowhere = {0};
    snprintf(nowhere.serial, sizeof nowhere.serial, "NOT-ATTACHED");
    snprintf(nowhere.path, sizeof nowhere.path, "/dev/ttyACM9");
    tap_ok(tr_switch_memo_pick(&nowhere, c, 3, &idx) == TR_SWITCH_MEMO_NONE,
           "a memo naming nothing attached picks nothing");
    tr_switch_memo_t blank = {0};
    tap_ok(tr_switch_memo_pick(&blank, c, 3, &idx) == TR_SWITCH_MEMO_NONE,
           "an empty memo picks nothing");
    tap_ok(tr_switch_memo_pick(&m, c, 0, &idx) == TR_SWITCH_MEMO_NONE,
           "with no ports attached, nothing is picked");
}

// ---- a stand-in for the switch, on a pseudo-terminal -----------------------
//
// The search can only be tested end to end against something that behaves
// like the board, so this is the firmware's serial side reimplemented from
// CalgaryToSpace/CTS-Ground-Station-UHF-Antenna-Switch's firmware/main.c:
//
//   - every line is printf("[<ms since boot>] [<level>] " fmt "\n"), where
//     the level names are "ALWAYS", " INFO ", "DEBUG ", "TRACE " (the
//     spacing is the firmware's, and the Pico's stdio turns the \n into
//     \r\n on the wire).
//   - it boots at log level 0, so it says nothing periodic until it is
//     told to. '0'..'3' set the level and are acknowledged at ALWAYS;
//     't'/'r'/'a'/'s' force TX / RX / AUTO / back to the slide switch,
//     also acknowledged at ALWAYS.
//   - at level 1 and above it prints, every 2500 ms,
//       "Heartbeat: log_level=%d state=%s mode=%s last_tx_ago_s=never"
//     with state "RX"/"TX" and mode "AUTO"/"FORCE_TX"/"FORCE_RX".
//
// That last line is the one the driver has to parse, and it carries a
// log_level= field before state= that our parser has to step over -- which
// is worth having a test drive rather than assume.
//
// A decoy sim talks the same way but never reaches level 1, so it is a
// port that answers with plausible traffic and no heartbeat.

typedef struct {
    pid_t pid;
    char  slave[64];
} sim_t;

// Every sim this process started, so a bail-out doesn't leave one running.
static sim_t g_sims[4];
static int   g_nsims = 0;

static void sims_stop_all(void)
{
    for (int i = 0; i < g_nsims; i++) {
        if (g_sims[i].pid > 0) {
            kill(g_sims[i].pid, SIGTERM);
            waitpid(g_sims[i].pid, NULL, 0);
            g_sims[i].pid = 0;
        }
    }
    g_nsims = 0;
}

static void sim_say(int fd, const char *level, unsigned long ms, const char *text)
{
    char line[256];
    int n = snprintf(line, sizeof line, "[%lu] [%s] %s\r\n", ms, level, text);
    if (n > 0) (void) !write(fd, line, (size_t) n);
}

// Runs in the child, holding the master side of the pty, until killed.
static void sim_run(int master, int can_reach_info_level)
{
    int level = 0;                 // the firmware boots at LOG_ALWAYS
    const char *mode = "AUTO";
    unsigned long ms = 0;
    unsigned long last_beat = 0;
    sim_say(master, "ALWAYS", ms, "===============================================");
    sim_say(master, "ALWAYS", ms, "CTS UHF RX/TX Switch connected to USB...");
    for (;;) {
        char c;
        ssize_t got = read(master, &c, 1);
        if (got == 1) {
            char ack[64];
            if (c >= '0' && c <= '3') {
                int want = c - '0';
                // A decoy accepts the command and stays quiet anyway.
                level = can_reach_info_level ? want : 0;
                snprintf(ack, sizeof ack, "Log level set to %d", want);
                sim_say(master, "ALWAYS", ms, ack);
            } else if (c == 't') { mode = "FORCE_TX"; sim_say(master, "ALWAYS", ms, "Serial override -> FORCE_TX"); }
            else if (c == 'r') { mode = "FORCE_RX"; sim_say(master, "ALWAYS", ms, "Serial override -> FORCE_RX"); }
            else if (c == 'a') { mode = "AUTO";     sim_say(master, "ALWAYS", ms, "Serial override -> AUTO"); }
            else if (c == 's') { mode = "AUTO";     sim_say(master, "ALWAYS", ms, "Serial override cleared, using slide switch"); }
        }
        if (level >= 1 && ms - last_beat >= 2500) {
            char beat[160];
            snprintf(beat, sizeof beat,
                     "Heartbeat: log_level=%d state=RX mode=%s last_tx_ago_s=never",
                     level, mode);
            sim_say(master, " INFO ", ms, beat);
            last_beat = ms;
        } else if (level == 0 && ms - last_beat >= 500) {
            // The decoy's traffic. Deliberately the hardest kind to
            // reject: it is in the same house style and carries the same
            // state= and mode= fields a heartbeat does -- the firmware's
            // own TRACE line does exactly this -- so the only thing that
            // tells the two apart is the word "Heartbeat:". A driver that
            // parsed any line with those fields would take this port for
            // the switch.
            sim_say(master, "ALWAYS", ms,
                    "ADC: now_ms=1234 raw=41 mV=33 state=RX mode=AUTO");
            last_beat = ms;
        }
        struct timespec nap = { 0, 10L * 1000L * 1000L };
        nanosleep(&nap, NULL);
        ms += 10;
    }
}

// Start a sim and return the pty slave path the ground software should open.
static int sim_start(sim_t *out, int can_reach_info_level)
{
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0) return -1;
    if (grantpt(master) != 0 || unlockpt(master) != 0) { close(master); return -1; }
    const char *slave = ptsname(master);
    if (slave == NULL) { close(master); return -1; }
    snprintf(out->slave, sizeof out->slave, "%s", slave);
    // Hold the slave open in the parent too, so the pty survives the
    // driver closing it between probes.
    int keep = open(slave, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(master, F_SETFL, O_NONBLOCK);
    pid_t pid = fork();
    if (pid < 0) { close(master); if (keep >= 0) close(keep); return -1; }
    if (pid == 0) {
        sim_run(master, can_reach_info_level);
        _exit(0);
    }
    close(master);
    out->pid = pid;
    if (g_nsims < (int) (sizeof g_sims / sizeof g_sims[0])) g_sims[g_nsims++] = *out;
    return 0;
}

// Point a port in the fake tree at a sim's pty, descriptors and all.
static void tree_add_sim_port(tree_t *t, const char *port, const sim_t *sim,
                              unsigned vid, unsigned pid,
                              const char *product, const char *serial)
{
    tree_add_usb_port(t, port, 0, vid, pid, product, SW_MAKER, serial, 0);
    char node[512];
    join(node, sizeof node, "%s/%s", t->dev, port);
    symlink(sim->slave, node);
}

// The first time: two ports answer, one of them with heartbeats. The
// quiet one has to be rejected, the switch kept and written down.
static void test_sim_first_find(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    sim_t decoy = {0}, sw = {0};
    if (sim_start(&decoy, 0) != 0 || sim_start(&sw, 1) != 0) {
        tap_bail("could not start a simulated switch on a pty");
        tree_close(&t);
        return;
    }
    tree_add_sim_port(&t, "ttyACM0", &decoy, 0x0483, 0x374b, "STM32 STLink", "0672FF48");
    tree_add_sim_port(&t, "ttyACM1", &sw, SW_VID, SW_PID, SW_PRODUCT, SW_SERIAL);

    char state[80];
    join(state, sizeof state, "%s/tr_switch.state", t.root);

    tr_switch_t s = {0};
    s.serial_speed = B115200;
    char path[TR_SWITCH_PATH_MAX] = "";
    tr_switch_candidate_t found = {0};
    int remembered = -1;
    char looked[256] = "";
    int rc = tr_switch_open_found(&s, t.sys, t.dev, state, path, sizeof path,
                                  &found, &remembered, looked, sizeof looked);
    tap_okf(rc == 0, "the switch is found among two answering ports (rc %d)", rc);
    tap_okf(ends_with(path, "/ttyACM1"), "and it is the one that beats (got %s)", path);
    tap_ok(remembered == 0, "nothing was remembered yet, so it had to be found");
    tap_okf(s.connected && s.heartbeat_count > 0,
            "the link is open with a parsed heartbeat (count %lu)", s.heartbeat_count);
    // Parsed out of the firmware's own line, log_level= field and all.
    tap_okf(strcmp(s.state_str, "RX") == 0 && strcmp(s.mode_str, "AUTO") == 0,
            "state and mode are read off it (got state=%s mode=%s)",
            s.state_str, s.mode_str);
    tap_ok(strstr(looked, "ttyACM0") != NULL && strstr(looked, "ttyACM1") != NULL,
           "both ports were tried, the quiet one first");
    tr_switch_disconnect(&s);

    tr_switch_memo_t m = {0};
    tap_ok(tr_switch_memo_read(state, &m) == 0, "the answer was written down");
    tap_okf(strcmp(m.serial, SW_SERIAL) == 0 && ends_with(m.path, "/ttyACM1"),
            "with the board's serial and port (got \"%s\" on %s)", m.serial, m.path);
    sims_stop_all();
    tree_close(&t);
}

// The next time: the remembered board is opened straight away, without
// waiting on the quiet port again.
static void test_sim_second_run_is_quick(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    sim_t decoy = {0}, sw = {0};
    if (sim_start(&decoy, 0) != 0 || sim_start(&sw, 1) != 0) {
        tap_bail("could not start a simulated switch on a pty");
        tree_close(&t);
        return;
    }
    tree_add_sim_port(&t, "ttyACM0", &decoy, 0x0483, 0x374b, "STM32 STLink", "0672FF48");
    tree_add_sim_port(&t, "ttyACM1", &sw, SW_VID, SW_PID, SW_PRODUCT, SW_SERIAL);

    char state[80];
    join(state, sizeof state, "%s/tr_switch.state", t.root);
    tr_switch_memo_t m = {0};
    snprintf(m.serial, sizeof m.serial, "%s", SW_SERIAL);
    join(m.path, sizeof m.path, "%s/ttyACM1", t.dev);
    m.vid = SW_VID; m.pid = SW_PID;
    tr_switch_memo_write(state, &m);

    tr_switch_t s = {0};
    s.serial_speed = B115200;
    char path[TR_SWITCH_PATH_MAX] = "";
    int remembered = -1;
    char looked[256] = "";
    double t0 = monotonic_seconds();
    int rc = tr_switch_open_found(&s, t.sys, t.dev, state, path, sizeof path,
                                  NULL, &remembered, looked, sizeof looked);
    double took = monotonic_seconds() - t0;
    tap_okf(rc == 0 && ends_with(path, "/ttyACM1"),
            "the remembered board is opened (rc %d, %s)", rc, path);
    tap_ok(remembered == 1, "and is reported as the one from last time");
    // A probe would cost a heartbeat period or the whole 4 s window; a
    // remembered serial costs an open.
    tap_okf(took < 1.0, "without waiting to be convinced (%.2f s)", took);
    tap_ok(strstr(looked, "ttyACM0") == NULL,
           "and the quiet port is never opened at all");
    tr_switch_disconnect(&s);
    sims_stop_all();
    tree_close(&t);
}

// The board was moved to another port between runs: the serial finds it
// there and the remembered port is brought up to date.
static void test_sim_board_moved(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    sim_t sw = {0};
    if (sim_start(&sw, 1) != 0) {
        tap_bail("could not start a simulated switch on a pty");
        tree_close(&t);
        return;
    }
    tree_add_sim_port(&t, "ttyACM2", &sw, SW_VID, SW_PID, SW_PRODUCT, SW_SERIAL);

    char state[80];
    join(state, sizeof state, "%s/tr_switch.state", t.root);
    tr_switch_memo_t m = {0};
    snprintf(m.serial, sizeof m.serial, "%s", SW_SERIAL);
    join(m.path, sizeof m.path, "%s/ttyACM1", t.dev);   // where it used to be
    tr_switch_memo_write(state, &m);

    tr_switch_t s = {0};
    s.serial_speed = B115200;
    char path[TR_SWITCH_PATH_MAX] = "";
    int remembered = -1;
    int rc = tr_switch_open_found(&s, t.sys, t.dev, state, path, sizeof path,
                                  NULL, &remembered, NULL, 0);
    tap_okf(rc == 0 && ends_with(path, "/ttyACM2"),
            "the board is found on its new port (rc %d, %s)", rc, path);
    tap_ok(remembered == 1, "by its serial, so it still counts as remembered");
    tr_switch_disconnect(&s);

    tr_switch_memo_t after = {0};
    tr_switch_memo_read(state, &after);
    tap_okf(ends_with(after.path, "/ttyACM2"),
            "and the remembered port is brought up to date (now %s)", after.path);
    sims_stop_all();
    tree_close(&t);
}

// A port that talks but never beats is not the switch, and nothing is
// written down when nothing is found.
static void test_sim_nothing_answers(void)
{
    tree_t t;
    if (tree_open(&t) != 0) { tap_bail("could not make a temp tree"); return; }
    sim_t decoy = {0};
    if (sim_start(&decoy, 0) != 0) {
        tap_bail("could not start a simulated port on a pty");
        tree_close(&t);
        return;
    }
    tree_add_sim_port(&t, "ttyACM0", &decoy, 0x0483, 0x374b, "STM32 STLink", "0672FF48");

    char state[80];
    join(state, sizeof state, "%s/tr_switch.state", t.root);

    tr_switch_t s = {0};
    s.serial_speed = B115200;
    char path[TR_SWITCH_PATH_MAX] = "";
    char looked[256] = "";
    int rc = tr_switch_open_found(&s, t.sys, t.dev, state, path, sizeof path,
                                  NULL, NULL, looked, sizeof looked);
    tap_okf(rc == -1, "a port that talks without beating is not the switch (rc %d)", rc);
    tap_ok(!s.connected && path[0] == '\0', "nothing is left open");
    tap_ok(strstr(looked, "ttyACM0") != NULL, "and the warning can name what was tried");
    tr_switch_memo_t m = {0};
    tap_ok(tr_switch_memo_read(state, &m) == -1,
           "nothing is written down when nothing was found");
    sims_stop_all();
    tree_close(&t);
}

int main(void)
{
    atexit(sims_stop_all);
    test_scan_reads_descriptors();
    test_scan_port_without_descriptors();
    test_scan_rejects();
    test_scan_macos_nodes();
    test_scan_bounds_and_guards();
    test_memo_round_trip();
    test_memo_parser();
    test_state_path();
    test_memo_pick();
    // These four drive the whole search against a simulated switch, so
    // they wait on real heartbeat periods and take a few seconds each.
    test_sim_first_find();
    test_sim_second_run_is_quick();
    test_sim_board_moved();
    test_sim_nothing_answers();
    return tap_done();
}
