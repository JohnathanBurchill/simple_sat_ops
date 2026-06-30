/*

   Simple Satellite Operations  ui/tui.c

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

#include "tui.h"
#include "state.h"

#include <fcntl.h>
#include <locale.h>
#include <ncurses.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>   // TIOCGWINSZ, struct winsize
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// --- ncurses ------------------------------------------------------

// While the ncurses TUI owns the screen, any stray write to stderr (a
// UHD/libusb error, a backend diagnostic, a library warning) lands on
// top of the panels and corrupts the display until the next full
// redraw. We therefore point fd 2 at a log file for the lifetime of
// the TUI: nothing reaches the terminal, but every message is still
// captured for post-pass debugging (the LIBUSB_TRANSFER_OVERFLOW flood
// during TX, for one). Restored on teardown so final console messages
// print normally. dup2 on the fd (not the FILE*) catches direct fd-2
// writes from C libraries too, not just our fprintf(stderr, ...).
static int   g_saved_stderr_fd = -1;
// Path of the redirected log and its size at grab time, so on quit we
// can tell the operator whether anything was logged THIS run (the file
// is opened append, so we compare against the starting size, not zero).
// Empty path => no real log file (e.g. a viewer with no pass folder).
static char  g_stderr_log_path[320] = "";
static off_t g_stderr_log_start_size = 0;

static void tui_grab_stderr(state_t *state)
{
    if (g_saved_stderr_fd != -1) return;   // already redirected

    char path[320];
    if (state->op.pass_folder[0]) {
        snprintf(path, sizeof path, "%.300s/sso_stderr.log", state->op.pass_folder);
    } else {
        // No pass folder (e.g. a viewer): we still must not corrupt the
        // screen, so swallow stderr rather than leave it on the tty.
        snprintf(path, sizeof path, "/dev/null");
    }

    int log_fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (log_fd < 0) return;                // leave stderr as-is on failure

    // Remember the log path + its current size so tui_report_errors can
    // tell whether this run appended anything. Only for a real file; a
    // /dev/null sink leaves the path empty and is never reported on.
    g_stderr_log_path[0]    = '\0';
    g_stderr_log_start_size = 0;
    if (state->op.pass_folder[0]) {
        struct stat st;
        if (fstat(log_fd, &st) == 0) g_stderr_log_start_size = st.st_size;
        snprintf(g_stderr_log_path, sizeof g_stderr_log_path, "%s", path);
    }

    fflush(stderr);
    g_saved_stderr_fd = dup(STDERR_FILENO);
    if (g_saved_stderr_fd < 0) { close(log_fd); g_saved_stderr_fd = -1; return; }
    dup2(log_fd, STDERR_FILENO);
    close(log_fd);
    // Unbuffered so log lines land promptly even on an abnormal exit.
    setvbuf(stderr, NULL, _IONBF, 0);
}

void tui_release_stderr(void)
{
    if (g_saved_stderr_fd == -1) return;
    fflush(stderr);
    dup2(g_saved_stderr_fd, STDERR_FILENO);
    close(g_saved_stderr_fd);
    g_saved_stderr_fd = -1;
}

// One-line closing status: did anything hit the redirected stderr log
// this run? Call once, last, after the TUI has been torn down and stderr
// restored. Silent when there was no real log file (e.g. a viewer).
void tui_report_errors(void)
{
    if (g_stderr_log_path[0] == '\0') return;
    struct stat st;
    if (stat(g_stderr_log_path, &st) == 0
        && st.st_size > g_stderr_log_start_size) {
        printf("Errors logged in %s\n", g_stderr_log_path);
    } else {
        printf("No errors reported\n");
    }
    fflush(stdout);
}

// --- Crash / quit signal safety net -------------------------------
//
// A streaming SDR yanked off USB makes UHD throw from a C++ destructor
// deep inside recv (on our worker thread), which the C API can't turn
// into an error return — it goes std::terminate -> abort -> SIGABRT. We
// can't recover from that, but we can refuse to leave the operator with
// a cryptic "Abort trap: 6" and a terminal stuck in ncurses raw mode.
// The handler restores the screen, prints one clear line to the real
// terminal, and re-raises so the process still dies with the original
// signal (and a core dump if enabled). SIGINT/SIGTERM instead ask the
// main loop to quit cleanly via its normal teardown.
static volatile sig_atomic_t g_signal_quit = 0;
// SIGUSR1 sets this — used by the force-claim takeover path to nudge the
// operator-mode loop into a graceful exit. (Full in-place demotion is a
// follow-up; for now SIGUSR1 = quit.)
static volatile sig_atomic_t g_yield_requested = 0;
// Claimed (test-and-set) by the first thread to enter the crash handler.
// On device loss TWO threads abort at once (our RX worker and a UHD
// internal thread); only one may run the terminal-restore + message.
static volatile char g_crash_claimed = 0;
// Terminal modes captured before ncurses took over, so the crash handler
// can restore the tty without relying on a (thread-racy) endwin().
static struct termios g_saved_termios;
static int            g_have_saved_termios = 0;
// Live-waterfall child pid, registered by the owner (main) so the crash
// handler can SIGKILL it before re-raising. Read through the pointer so
// the owner keeps full control of the pid's lifetime.
static pid_t *g_waterfall_pid_ptr = NULL;

// write() a string literal — async-signal-safe (sizeof-1 drops the NUL).
#define CRASH_WRITE(fd, s) do { ssize_t w_ = write((fd), (s), sizeof(s) - 1); (void) w_; } while (0)

static void graceful_quit_handler(int sig)
{
    (void) sig;
    g_signal_quit = 1;   // main loop notices and runs its normal teardown
}

static void on_sigusr1(int sig)
{
    (void) sig;
    g_yield_requested = 1;
}

static void crash_handler(int sig)
{
    // On device loss two threads abort almost simultaneously. The first
    // claims the handler and does the cleanup; any other thread must NOT
    // _exit here — that single fast syscall would terminate the process
    // before the first thread finished restoring the terminal and
    // printing the message (the bug: "waterfall down, terminal garbled,
    // no message"). Instead, wait: the claiming thread re-raises and
    // brings the whole process down within microseconds. Bounded so a
    // wedged cleanup can't hang forever.
    if (__atomic_test_and_set(&g_crash_claimed, __ATOMIC_SEQ_CST)) {
        for (int i = 0; i < 50; i++) {
            struct timespec ts = { 0, 10 * 1000 * 1000 };   // 10 ms
            nanosleep(&ts, NULL);
        }
        _exit(128 + sig);
    }

    // Kill the spawned live-waterfall window so it doesn't orphan when we
    // die — the normal teardown that usually does this won't run. kill()
    // is async-signal-safe.
    if (g_waterfall_pid_ptr != NULL && *g_waterfall_pid_ptr > 0) {
        kill(*g_waterfall_pid_ptr, SIGKILL);
    }

    // Restore the terminal DETERMINISTICALLY. The fault is usually on the
    // RX worker thread, where ncurses endwin() races the main thread's
    // drawing and can leave the tty half-restored ("needs reset"). So we
    // skip endwin() and instead restore the saved termios + emit the
    // raw escapes to leave the alt-screen, show the cursor, and reset
    // attributes. tcsetattr + write are plain syscalls, safe from here.
    if (g_have_saved_termios) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_termios);
    }
    CRASH_WRITE(STDOUT_FILENO, "\033[?1049l\033[?25h\033[0m\r\n");

    // stderr is redirected to the pass-folder log during the TUI; aim the
    // message at the real terminal so the operator actually sees it.
    int fd = (g_saved_stderr_fd >= 0) ? g_saved_stderr_fd : STDERR_FILENO;
    CRASH_WRITE(fd, "\n*** simple_sat_ops: fatal error");
    switch (sig) {
        case SIGABRT: CRASH_WRITE(fd, " (SIGABRT - a USB/SDR device was likely disconnected)"); break;
        case SIGSEGV: CRASH_WRITE(fd, " (SIGSEGV)"); break;
        case SIGBUS:  CRASH_WRITE(fd, " (SIGBUS)");  break;
        default:      CRASH_WRITE(fd, "");           break;
    }
    CRASH_WRITE(fd,
        ".\nThe terminal has been restored. Any detail is in the pass-folder\n"
        "log (sso_stderr.log). Reconnect the device and restart.\n");

    // Re-raise with the default disposition so the process terminates
    // with the original signal (preserving core-dump behaviour).
    signal(sig, SIG_DFL);
    raise(sig);
}

void install_signal_handlers(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = crash_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGABRT, &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGBUS,  &sa, NULL);

    struct sigaction sq;
    memset(&sq, 0, sizeof sq);
    sq.sa_handler = graceful_quit_handler;
    sigemptyset(&sq.sa_mask);
    sigaction(SIGINT,  &sq, NULL);
    sigaction(SIGTERM, &sq, NULL);
}

// Arm the SIGUSR1 (force-takeover) handler. Installed separately from the
// crash/quit handlers because only the operator (after a successful IPC
// bind) listens for the yield signal.
void tui_install_yield_handler(void)
{
    struct sigaction su;
    memset(&su, 0, sizeof su);
    su.sa_handler = on_sigusr1;
    sigemptyset(&su.sa_mask);
    sigaction(SIGUSR1, &su, NULL);
}

void tui_save_termios(void)
{
    if (tcgetattr(STDIN_FILENO, &g_saved_termios) == 0) {
        g_have_saved_termios = 1;
    }
}

int tui_should_quit(void)
{
    return g_signal_quit;
}

int tui_yield_requested(void)
{
    return g_yield_requested;
}

void tui_register_waterfall_pid(pid_t *pidp)
{
    g_waterfall_pid_ptr = pidp;
}

void tui_handle_resize(void)
{
    // ncurses *usually* re-queries the size and updates LINES/COLS itself when
    // it delivers KEY_RESIZE, but not on every build/platform. Do it explicitly
    // so the panels -- whose truncation widths track COLS -- always follow the
    // window, instead of staying stuck at the pre-resize size.
    //
    // Use resize_term, NOT resizeterm: when ncurses supplies its own SIGWINCH
    // handler (the default), resizeterm ungetch's a fresh KEY_RESIZE. Calling
    // it from inside our own KEY_RESIZE handler would re-arm that event on
    // every getch, an endless loop that starves real keypresses. resize_term
    // does the same window/LINES/COLS resize without that bookkeeping.
    struct winsize ws = {0};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0
        && ws.ws_row > 0 && ws.ws_col > 0) {
        resize_term(ws.ws_row, ws.ws_col);
    }
    // clear() erases stdscr *and* arms clearok, so the next refresh wipes the
    // physical screen before repainting -- no leftover characters from the
    // old, differently sized layout (e.g. the vertical ribbon's old column).
    clear();
}

void init_window(state_t *state)
{
    // setlocale BEFORE initscr so ncurses knows the terminal can render
    // its alternate-character-set line glyphs (and UTF-8 elsewhere).
    // Without this, box() and friends emit the ACS fallback letters
    // (q for horizontal, x for vertical, lkjm for corners) instead of
    // line-drawing characters.
    setlocale(LC_ALL, "");

    initscr(); cbreak(); noecho();
    nonl();
    timeout(0);
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);
    // ncurses defaults ESCDELAY to 1000 ms — fine for distinguishing
    // bare Esc from the leading byte of a function-key sequence, but
    // makes Esc-to-cancel and arrow-key composition feel sluggish.
    // 25 ms is the conventional snappy value; any real escape sequence
    // arrives in a few ms so this isn't tight.
    set_escdelay(25);
    start_color();
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    curs_set(0);

    // Now that ncurses owns the screen, divert stderr to the pass-folder
    // log so backend/library errors never paint over the panels.
    tui_grab_stderr(state);
}
