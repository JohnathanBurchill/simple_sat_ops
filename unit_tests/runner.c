/*

    Simple Satellite Operations  unit_tests/runner.c

    Spawn the project's *_selftest binaries one at a time, parse their
    TAP output, and present the aggregate result in an ncurses TUI:

        Tests                              Pass  Fail
        ------------------------------------------------
        [+] rs_selftest                      12     0  OK
        [-] modem_iq_selftest                 8     1  FAIL
              sps=10, awgn=10dB ............ OK
              sps=10, awgn=5dB ............. FAIL  (BER 0.034 > 0.01)
              sps=20, awgn=10dB ............ OK
        [+] ax100_selftest                   24     0  OK
        ...
        ------------------------------------------------
        50/51 passed, 1 failed across 6 groups.

    Keys:
        Up/Down/PgUp/PgDn/Home/End    navigate
        Enter / Space                 toggle expand on focused group
        d                             expand all (open everything)
        a                             collapse all
        r                             rerun all
        q / Esc                       quit

    Failed groups auto-expand on first display. Exit status = number of
    groups that failed.

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

#define _GNU_SOURCE

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <locale.h>
#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

// One TAP "ok"/"not ok" line.
typedef struct {
    int    seq;
    int    ok;
    char   desc[200];   // includes " # reason" tail for not ok lines
} tap_line_t;

// One selftest binary's worth of results.
typedef struct {
    char        name[64];
    char        exec_path[512];
    tap_line_t *lines;
    int         n_lines;
    int         cap_lines;
    int         pass;
    int         fail;
    int         exit_code;     // child wait status, 0 if not yet run
    int         bailed;        // 1 if test emitted "Bail out!"
    char        bail_reason[200];
    int         expanded;      // 1 if user has clicked open
    char        stderr_tail[4096]; // captured stderr for the details view
} group_t;

static void group_push(group_t *g, const tap_line_t *l)
{
    if (g->n_lines >= g->cap_lines) {
        int n = g->cap_lines ? g->cap_lines * 2 : 32;
        tap_line_t *nb = (tap_line_t *) realloc(g->lines,
                                                (size_t) n * sizeof *nb);
        if (!nb) return;
        g->lines = nb;
        g->cap_lines = n;
    }
    g->lines[g->n_lines++] = *l;
}

static void group_reset(group_t *g)
{
    g->n_lines = 0;
    g->pass = 0;
    g->fail = 0;
    g->exit_code = 0;
    g->bailed = 0;
    g->bail_reason[0] = '\0';
    g->stderr_tail[0] = '\0';
}

// Parse one line of TAP. Recognised forms:
//   ok N - description
//   not ok N - description
//   not ok N - description # reason
//   Bail out! reason
//   1..N
//   # comment
// Returns 1 if the line produced an assertion (ok/not ok), 0 otherwise.
static int parse_tap_line(group_t *g, const char *line)
{
    while (*line == ' ') ++line;

    if (strncmp(line, "Bail out!", 9) == 0) {
        g->bailed = 1;
        const char *r = line + 9;
        while (*r == ' ') ++r;
        snprintf(g->bail_reason, sizeof g->bail_reason, "%s", r);
        return 0;
    }
    if (line[0] == '#' || line[0] == '\0') return 0;
    if (line[0] == '1' && line[1] == '.' && line[2] == '.') return 0;

    tap_line_t out = {0};
    int neg = 0;
    if (strncmp(line, "not ok", 6) == 0) {
        neg = 1;
        line += 6;
    } else if (strncmp(line, "ok", 2) == 0) {
        line += 2;
    } else {
        return 0;
    }
    while (*line == ' ') ++line;
    out.seq = atoi(line);
    while (*line && *line != ' ' && *line != '-') ++line;
    while (*line == ' ' || *line == '-') ++line;
    snprintf(out.desc, sizeof out.desc, "%s", line);
    // Trim trailing newline.
    size_t n = strlen(out.desc);
    while (n > 0 && (out.desc[n-1] == '\n' || out.desc[n-1] == '\r')) {
        out.desc[--n] = '\0';
    }
    out.ok = neg ? 0 : 1;
    group_push(g, &out);
    if (out.ok) ++g->pass; else ++g->fail;
    return 1;
}

// Fork + exec one selftest, pipe its stdout into the TAP parser, capture
// stderr into a small tail buffer for the details view. Returns the wait
// status of the child.
static int run_one(group_t *g)
{
    group_reset(g);

    int out_p[2], err_p[2];
    if (pipe(out_p) != 0) return -1;
    if (pipe(err_p) != 0) { close(out_p[0]); close(out_p[1]); return -1; }

    pid_t pid = fork();
    if (pid < 0) {
        close(out_p[0]); close(out_p[1]);
        close(err_p[0]); close(err_p[1]);
        return -1;
    }
    if (pid == 0) {
        dup2(out_p[1], STDOUT_FILENO);
        dup2(err_p[1], STDERR_FILENO);
        close(out_p[0]); close(out_p[1]);
        close(err_p[0]); close(err_p[1]);
        execl(g->exec_path, g->exec_path, (char *) NULL);
        fprintf(stderr, "runner: exec %s: %s\n", g->exec_path, strerror(errno));
        _exit(127);
    }
    close(out_p[1]);
    close(err_p[1]);

    // Read stdout line by line and parse TAP. Stderr drains into the tail.
    FILE *fp = fdopen(out_p[0], "r");
    if (fp == NULL) {
        close(out_p[0]);
        close(err_p[0]);
        waitpid(pid, NULL, 0);
        return -1;
    }
    char buf[2048];
    while (fgets(buf, sizeof buf, fp)) {
        parse_tap_line(g, buf);
    }
    fclose(fp);

    // Slurp stderr into a 4 KB tail (oldest bytes get rolled off).
    size_t cap = sizeof g->stderr_tail;
    size_t held = 0;
    char chunk[1024];
    ssize_t nread;
    while ((nread = read(err_p[0], chunk, sizeof chunk)) > 0) {
        if (held + (size_t) nread <= cap - 1) {
            memcpy(g->stderr_tail + held, chunk, (size_t) nread);
            held += (size_t) nread;
        } else {
            size_t keep = cap - 1 - (size_t) nread;
            memmove(g->stderr_tail, g->stderr_tail + (held - keep), keep);
            memcpy(g->stderr_tail + keep, chunk, (size_t) nread);
            held = cap - 1;
        }
    }
    g->stderr_tail[held] = '\0';
    close(err_p[0]);

    int status = 0;
    waitpid(pid, &status, 0);
    g->exit_code = status;
    return 0;
}

// Discover selftest binaries. They live in the same directory as the
// runner binary (typically build/), which is exactly what CMake produces
// for add_executable. We scan that directory for "*_selftest" executables
// rather than keeping a hand-maintained list — a curated list silently
// fell behind the suite (skipped 13 of 37 tests at one point).
static int dir_of(const char *argv0, char *out, size_t out_cap)
{
    if (argv0[0] == '/') {
        snprintf(out, out_cap, "%s", argv0);
    } else {
        char cwd[1024];
        if (!getcwd(cwd, sizeof cwd)) return -1;
        snprintf(out, out_cap, "%s/%s", cwd, argv0);
    }
    char *slash = strrchr(out, '/');
    if (slash) *slash = '\0';
    return 0;
}

static int add_group_if_exists(group_t *groups, int *n, int cap,
                               const char *dir, const char *name)
{
    if (*n >= cap) return 0;
    char path[512];
    snprintf(path, sizeof path, "%s/%s", dir, name);
    if (access(path, X_OK) != 0) return 0;
    group_t *g = &groups[(*n)++];
    memset(g, 0, sizeof *g);
    snprintf(g->name, sizeof g->name, "%s", name);
    snprintf(g->exec_path, sizeof g->exec_path, "%s", path);
    return 1;
}

static int ends_with(const char *s, const char *suffix)
{
    size_t ls = strlen(s), lf = strlen(suffix);
    return ls >= lf && strcmp(s + ls - lf, suffix) == 0;
}

static int cmp_names(const void *a, const void *b)
{
    return strcmp(*(const char *const *) a, *(const char *const *) b);
}

// Scan `dir` for executable regular files named "*_selftest" and build one
// group per match, sorted by name for a stable display order. Returns a
// malloc'd array (caller frees) via *out_groups, count via *out_n. Returns
// 0 on success (including the no-tests-found case, n == 0) and -1 if the
// directory can't be read or memory runs out.
static int discover_selftests(const char *dir, group_t **out_groups, int *out_n)
{
    *out_groups = NULL;
    *out_n = 0;

    DIR *d = opendir(dir);
    if (!d) return -1;

    char **names = NULL;
    int n = 0, cap = 0;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (!ends_with(de->d_name, "_selftest")) continue;
        char path[512];
        snprintf(path, sizeof path, "%s/%s", dir, de->d_name);
        struct stat st;
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) continue;
        if (access(path, X_OK) != 0) continue;
        if (n >= cap) {
            int nc = cap ? cap * 2 : 32;
            char **nb = (char **) realloc(names, (size_t) nc * sizeof *nb);
            if (!nb) { for (int i = 0; i < n; ++i) free(names[i]);
                       free(names); closedir(d); return -1; }
            names = nb;
            cap = nc;
        }
        names[n] = strdup(de->d_name);
        if (!names[n]) { for (int i = 0; i < n; ++i) free(names[i]);
                         free(names); closedir(d); return -1; }
        ++n;
    }
    closedir(d);

    if (n == 0) { free(names); return 0; }

    qsort(names, (size_t) n, sizeof *names, cmp_names);

    group_t *groups = (group_t *) calloc((size_t) n, sizeof *groups);
    if (!groups) { for (int i = 0; i < n; ++i) free(names[i]);
                   free(names); return -1; }
    int ng = 0;
    for (int i = 0; i < n; ++i) {
        add_group_if_exists(groups, &ng, n, dir, names[i]);
        free(names[i]);
    }
    free(names);

    *out_groups = groups;
    *out_n = ng;
    return 0;
}

// Rendering ----------------------------------------------------------

// One visible row in the scrollable list. Either a group header or
// an indented subtest line under an expanded group.
typedef enum { ROW_GROUP, ROW_SUB } row_kind_t;
typedef struct {
    row_kind_t kind;
    int        group_idx;
    int        sub_idx;    // only when kind == ROW_SUB
} row_t;

// Build the flat list of visible rows — one per group header, plus the
// subtest lines of any expanded group — into a heap buffer that grows to
// fit exactly, so the list can never silently truncate however large the
// suite gets. *rows / *cap persist across calls (the caller frees *rows
// once at exit). Returns the row count.
static int build_rows(group_t *groups, int n_groups, row_t **rows, int *cap)
{
    int need = 0;
    for (int gi = 0; gi < n_groups; ++gi) {
        ++need;
        if (groups[gi].expanded) need += groups[gi].n_lines;
    }
    if (need > *cap) {
        int nc = *cap ? *cap : 64;
        while (nc < need) nc *= 2;
        row_t *nb = (row_t *) realloc(*rows, (size_t) nc * sizeof *nb);
        if (nb) { *rows = nb; *cap = nc; }
        // On the (vanishingly unlikely) realloc failure, keep the old
        // smaller buffer; the n < *cap guards below truncate the display
        // rather than overflow.
    }
    int n = 0;
    for (int gi = 0; gi < n_groups && n < *cap; ++gi) {
        (*rows)[n++] = (row_t){ ROW_GROUP, gi, 0 };
        if (groups[gi].expanded) {
            for (int si = 0; si < groups[gi].n_lines && n < *cap; ++si) {
                (*rows)[n++] = (row_t){ ROW_SUB, gi, si };
            }
        }
    }
    return n;
}

static void draw_row(int y, int width, const row_t *r, const group_t *groups,
                     int focused, int color_ok, int color_fail)
{
    if (focused) attron(A_REVERSE);
    if (r->kind == ROW_GROUP) {
        const group_t *g = &groups[r->group_idx];
        const char *marker = g->expanded ? "[-]" : "[+]";
        int passed = g->pass;
        int failed = g->fail;
        int ok = (failed == 0 && !g->bailed
                  && WIFEXITED(g->exit_code)
                  && WEXITSTATUS(g->exit_code) == 0);
        if (ok) attron(COLOR_PAIR(color_ok));
        else    attron(COLOR_PAIR(color_fail));
        char status[16];
        if (g->bailed)          snprintf(status, sizeof status, "BAIL");
        else if (failed > 0)    snprintf(status, sizeof status, "FAIL");
        else if (!WIFEXITED(g->exit_code)
                 || WEXITSTATUS(g->exit_code) != 0)
                                snprintf(status, sizeof status, "CRASH");
        else                    snprintf(status, sizeof status, "OK");
        mvprintw(y, 1, "%s %-32.32s  %4d  %4d  %-5s",
                 marker, g->name, passed, failed, status);
        if (ok) attroff(COLOR_PAIR(color_ok));
        else    attroff(COLOR_PAIR(color_fail));
    } else {
        const group_t *g = &groups[r->group_idx];
        const tap_line_t *l = &g->lines[r->sub_idx];
        const char *tag = l->ok ? "OK  " : "FAIL";
        if (l->ok) attron(COLOR_PAIR(color_ok));
        else       attron(COLOR_PAIR(color_fail));
        // Indent under the parent group; truncate description to fit.
        int dw = width - 16;
        if (dw < 8) dw = 8;
        if (dw > 200) dw = 200;
        mvprintw(y, 8, "%-*.*s %s", dw, dw, l->desc, tag);
        if (l->ok) attroff(COLOR_PAIR(color_ok));
        else       attroff(COLOR_PAIR(color_fail));
    }
    clrtoeol();
    if (focused) attroff(A_REVERSE);
}

static void render(group_t *groups, int n_groups, const row_t *rows, int n_rows,
                   int focus, int scroll_top, int color_ok, int color_fail)
{
    erase();
    int H = LINES, W = COLS;

    // Title bar.
    attron(A_BOLD);
    mvprintw(0, 1, "Simple Sat Ops — unit tests");
    attroff(A_BOLD);
    mvprintw(0, W - 30, "[Enter] expand   [r] rerun   [q] quit");
    mvhline(1, 0, ACS_HLINE, W);

    // Header row.
    mvprintw(2, 1, "%-32s  %4s  %4s  %-5s",
             "Tests", "Pass", "Fail", "Status");
    mvhline(3, 0, ACS_HLINE, W);

    // Body — scrollable list of rows, built by the caller (see build_rows).
    int body_top = 4;
    int body_bot = H - 3;
    int body_h = body_bot - body_top + 1;
    if (focus < 0) focus = 0;
    if (focus >= n_rows && n_rows > 0) focus = n_rows - 1;
    if (focus < scroll_top) scroll_top = focus;
    if (focus >= scroll_top + body_h) scroll_top = focus - body_h + 1;
    if (scroll_top < 0) scroll_top = 0;

    for (int i = 0; i < body_h; ++i) {
        int ri = scroll_top + i;
        if (ri >= n_rows) {
            move(body_top + i, 0);
            clrtoeol();
            continue;
        }
        draw_row(body_top + i, W, &rows[ri], groups, ri == focus,
                 color_ok, color_fail);
    }

    // Footer with totals.
    mvhline(H - 2, 0, ACS_HLINE, W);
    int total_pass = 0, total_fail = 0, groups_failed = 0;
    for (int gi = 0; gi < n_groups; ++gi) {
        total_pass += groups[gi].pass;
        total_fail += groups[gi].fail;
        int ok = (groups[gi].fail == 0 && !groups[gi].bailed
                  && WIFEXITED(groups[gi].exit_code)
                  && WEXITSTATUS(groups[gi].exit_code) == 0);
        if (!ok) ++groups_failed;
    }
    if (groups_failed == 0) attron(COLOR_PAIR(color_ok));
    else                    attron(COLOR_PAIR(color_fail));
    mvprintw(H - 1, 1,
             "%d/%d passed, %d failed across %d groups.",
             total_pass, total_pass + total_fail, total_fail, n_groups);
    if (groups_failed == 0) attroff(COLOR_PAIR(color_ok));
    else                    attroff(COLOR_PAIR(color_fail));

    refresh();
}

int main(int argc, char **argv)
{
    char rdir[1024];
    if (dir_of(argc > 0 ? argv[0] : "./runner", rdir, sizeof rdir) != 0) {
        fprintf(stderr, "runner: cannot resolve own directory\n");
        return 2;
    }

    // Discover every "*_selftest" executable next to the runner. Anything
    // not present (missing libs, deliberately disabled in the build) simply
    // isn't there to find, so we never crash on missing optional pieces —
    // and a newly added test shows up with no edit here.
    group_t *groups = NULL;
    int n_groups = 0;
    if (discover_selftests(rdir, &groups, &n_groups) != 0) {
        fprintf(stderr, "runner: cannot scan %s for selftest binaries\n", rdir);
        return 2;
    }
    if (n_groups == 0) {
        fprintf(stderr, "runner: no *_selftest binaries found next to %s\n",
                argv[0]);
        return 2;
    }

    // Run everything once up front so the screen has content when it
    // appears; the user can rerun with 'r' any time after.
    for (int i = 0; i < n_groups; ++i) {
        run_one(&groups[i]);
        // Auto-expand failing groups on first run so the user lands on
        // the actionable detail without having to click.
        if (groups[i].fail > 0 || groups[i].bailed
            || (WIFEXITED(groups[i].exit_code)
                && WEXITSTATUS(groups[i].exit_code) != 0)
            || !WIFEXITED(groups[i].exit_code)) {
            groups[i].expanded = 1;
        }
    }

    // ncurses setup. setlocale before initscr so the terminal's UTF-8
    // capability is detected and ACS_HLINE renders as a clean rule.
    setlocale(LC_ALL, "");
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    int color_ok = 2, color_fail = 1;
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(color_fail, COLOR_RED,   -1);
        init_pair(color_ok,   COLOR_GREEN, -1);
    }

    int focus = 0;
    int scroll_top = 0;
    int running = 1;
    row_t *rows = NULL;
    int rows_cap = 0;
    while (running) {
        int n_rows = build_rows(groups, n_groups, &rows, &rows_cap);
        render(groups, n_groups, rows, n_rows, focus, scroll_top,
               color_ok, color_fail);

        int key = getch();
        switch (key) {
            case 'q': case 'Q': case 27 /* Esc */:
                running = 0;
                break;
            case KEY_UP:
                if (focus > 0) --focus;
                break;
            case KEY_DOWN:
                if (focus + 1 < n_rows) ++focus;
                break;
            case KEY_PPAGE:
                focus -= LINES / 2;
                if (focus < 0) focus = 0;
                break;
            case KEY_NPAGE:
                focus += LINES / 2;
                if (focus >= n_rows) focus = n_rows - 1;
                break;
            case KEY_HOME:
                focus = 0;
                break;
            case KEY_END:
                if (n_rows > 0) focus = n_rows - 1;
                break;
            case '\n': case ' ':
                if (focus < n_rows && rows[focus].kind == ROW_GROUP) {
                    int gi = rows[focus].group_idx;
                    groups[gi].expanded = !groups[gi].expanded;
                }
                break;
            case 'd':
                for (int i = 0; i < n_groups; ++i) groups[i].expanded = 1;
                break;
            case 'a':
                for (int i = 0; i < n_groups; ++i) groups[i].expanded = 0;
                break;
            case 'r': {
                // Briefly show a "running…" footer while each test executes.
                for (int i = 0; i < n_groups; ++i) {
                    mvprintw(LINES - 1, 1, "running %-50.50s ...",
                             groups[i].name);
                    clrtoeol();
                    refresh();
                    run_one(&groups[i]);
                    if (groups[i].fail > 0 || groups[i].bailed
                        || !WIFEXITED(groups[i].exit_code)
                        || WEXITSTATUS(groups[i].exit_code) != 0) {
                        groups[i].expanded = 1;
                    } else {
                        groups[i].expanded = 0;
                    }
                }
                break;
            }
            default:
                break;
        }
    }

    endwin();

    // Exit code = number of failing groups so CI / make targets can
    // gate on a clean run.
    int failed = 0;
    for (int i = 0; i < n_groups; ++i) {
        int ok = (groups[i].fail == 0 && !groups[i].bailed
                  && WIFEXITED(groups[i].exit_code)
                  && WEXITSTATUS(groups[i].exit_code) == 0);
        if (!ok) ++failed;
        free(groups[i].lines);
    }
    free(rows);
    free(groups);
    return failed;
}
