#include "sso_audit.h"

#include "sso_paths.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define SSO_AUDIT_BUF 512

static char g_tool[64];
static int g_exit_code = 0;
static int g_atexit_armed = 0;
static char g_log_path[SSO_AUDIT_BUF];
static int g_log_path_resolved = 0;

// --- Async writer state ---------------------------------------------
//
// Producer/consumer ring drained by a single worker pthread. The
// producer (sso_audit_event / sso_audit_start) snapshots the timestamp
// and copies the event into the ring under g_audit_mu, then broadcasts;
// the worker takes the lock briefly to drain the ring and writes each
// event off-lock with per-event flock (preserves cross-process serial-
// ization on /var/log/sso/runs.log). Drop-oldest on overflow keeps the
// producer non-blocking; the worker emits a synthesized
// "audit-overflow dropped=N" line whenever drops have happened.

#define SSO_AUDIT_RING_SIZE 256

typedef struct {
    struct timespec ts;
    char event[64];
    char detail[480];
} audit_evt_t;

static audit_evt_t      g_audit_ring[SSO_AUDIT_RING_SIZE];
static int              g_audit_head     = 0;   // next slot to write
static int              g_audit_tail     = 0;   // next slot to read
static int              g_audit_count    = 0;
static int              g_audit_dropped  = 0;

static pthread_mutex_t  g_audit_mu       = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t   g_audit_cv       = PTHREAD_COND_INITIALIZER;
static pthread_t        g_audit_thread;
static int              g_audit_thread_started = 0;
static int              g_audit_stop     = 0;

static const char *resolve_log_path(void) {
    if (g_log_path_resolved) return g_log_path;
    const char *env = getenv("SSO_AUDIT_LOG");
    if (env && env[0]) {
        snprintf(g_log_path, sizeof(g_log_path), "%s", env);
        g_log_path_resolved = 1;
        return g_log_path;
    }
    // Try /var/log/sso/runs.log first. The tmpfiles.d config creates
    // that directory at boot with sso-ops group ownership. If we can't
    // even stat the dir, fall back to the dev-host location.
    struct stat st;
    if (stat("/var/log/sso", &st) == 0 && S_ISDIR(st.st_mode)) {
        snprintf(g_log_path, sizeof(g_log_path), "/var/log/sso/runs.log");
        g_log_path_resolved = 1;
        return g_log_path;
    }
    const char *home = getenv("HOME");
    if (!home || !home[0]) home = "/tmp";
    snprintf(g_log_path, sizeof(g_log_path),
             "%s/.local/share/simple_sat_ops/runs.log", home);
    g_log_path_resolved = 1;
    return g_log_path;
}

const char *sso_unix_user(void) {
    static char buf[64];
    static int resolved = 0;
    if (resolved) return buf;
    const char *u = getenv("SUDO_USER");
    if (!u || !u[0]) u = getenv("USER");
    if (!u || !u[0]) u = getenv("LOGNAME");
    if (!u || !u[0]) {
        struct passwd *pw = getpwuid(getuid());
        u = (pw && pw->pw_name) ? pw->pw_name : "unknown";
    }
    snprintf(buf, sizeof(buf), "%s", u);
    resolved = 1;
    return buf;
}

static void iso_utc_from_ts(const struct timespec *ts,
                            char *out, size_t out_size) {
    struct tm tm;
    gmtime_r(&ts->tv_sec, &tm);
    snprintf(out, out_size,
             "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             (long) (ts->tv_nsec / 1000000));
}

// Sanitize one free-form field: replace tabs and newlines with spaces
// so they don't break the tab-separated record format.
static void sanitize(char *s) {
    if (!s) return;
    for (; *s; ++s) {
        if (*s == '\t' || *s == '\n' || *s == '\r') *s = ' ';
    }
}

// Write one audit line to runs.log, using the supplied timestamp (so
// events stay stamped at producer time even when the worker writes them
// later). Per-event flock preserves serialization with other processes
// that also append to runs.log. Returns 0 on success, -1 on I/O error.
static int append_line_at(const char *event, const char *detail,
                          const struct timespec *ts) {
    const char *path = resolve_log_path();
    sso_mkdir_p_for_file(path);
    int fd = open(path, O_WRONLY | O_APPEND | O_CREAT, 0664);
    if (fd < 0) return -1;
    char tsbuf[64];
    iso_utc_from_ts(ts, tsbuf, sizeof(tsbuf));
    char user_buf[64];
    snprintf(user_buf, sizeof(user_buf), "%s", sso_unix_user());
    sanitize(user_buf);
    char tool_buf[64];
    snprintf(tool_buf, sizeof(tool_buf), "%s",
             g_tool[0] ? g_tool : "?");
    sanitize(tool_buf);
    char event_buf[64];
    snprintf(event_buf, sizeof(event_buf), "%s",
             (event && event[0]) ? event : "-");
    sanitize(event_buf);
    char detail_buf[480];
    snprintf(detail_buf, sizeof(detail_buf), "%s",
             (detail && detail[0]) ? detail : "");
    sanitize(detail_buf);
    char line[SSO_AUDIT_BUF];
    int n = snprintf(line, sizeof(line),
                     "%s\t%s\t%s\t%d\t%s\t%s\n",
                     tsbuf, user_buf, tool_buf, (int) getpid(),
                     event_buf, detail_buf);
    if (n <= 0) {
        close(fd);
        return -1;
    }
    if ((size_t) n >= sizeof(line)) n = (int) sizeof(line) - 1;
    flock(fd, LOCK_EX);
    ssize_t w = write(fd, line, (size_t) n);
    flock(fd, LOCK_UN);
    close(fd);
    return (w == n) ? 0 : -1;
}

// Synchronous fallback — used by sso_audit_event before the worker is
// up, and by the worker itself.
static int append_line(const char *event, const char *detail) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return append_line_at(event, detail, &ts);
}

// --- Worker thread --------------------------------------------------

static void *audit_worker_fn(void *arg) {
    (void) arg;
    for (;;) {
        pthread_mutex_lock(&g_audit_mu);
        while (g_audit_count == 0 && !g_audit_stop) {
            pthread_cond_wait(&g_audit_cv, &g_audit_mu);
        }
        if (g_audit_count == 0 && g_audit_stop) {
            pthread_mutex_unlock(&g_audit_mu);
            break;
        }
        // Drain the ring into a stack-local snapshot so we can write
        // off-lock (the per-event flock can stall for tens to hundreds
        // of ms under contention with other tools).
        audit_evt_t local[SSO_AUDIT_RING_SIZE];
        int n = g_audit_count;
        for (int i = 0; i < n; ++i) {
            local[i] = g_audit_ring[g_audit_tail];
            g_audit_tail = (g_audit_tail + 1) % SSO_AUDIT_RING_SIZE;
        }
        g_audit_count = 0;
        int dropped_local = g_audit_dropped;
        g_audit_dropped = 0;
        pthread_mutex_unlock(&g_audit_mu);

        for (int i = 0; i < n; ++i) {
            (void) append_line_at(local[i].event, local[i].detail,
                                  &local[i].ts);
        }
        if (dropped_local > 0) {
            char det[64];
            snprintf(det, sizeof det, "dropped=%d", dropped_local);
            (void) append_line("audit-overflow", det);
        }
    }
    return NULL;
}

// Spawn the worker once. Subsequent calls are no-ops. Returns 0 on
// success, -1 if pthread_create fails (the producer then falls back to
// synchronous writes).
static int audit_worker_init(void) {
    pthread_mutex_lock(&g_audit_mu);
    if (g_audit_thread_started) {
        pthread_mutex_unlock(&g_audit_mu);
        return 0;
    }
    if (pthread_create(&g_audit_thread, NULL, audit_worker_fn, NULL) != 0) {
        pthread_mutex_unlock(&g_audit_mu);
        return -1;
    }
    g_audit_thread_started = 1;
    pthread_mutex_unlock(&g_audit_mu);
    return 0;
}

// Stop the worker, broadcast, join. Idempotent — safe to call from
// multiple atexit paths.
static void audit_worker_close(void) {
    pthread_t  th_local;
    int        joinable = 0;
    pthread_mutex_lock(&g_audit_mu);
    if (g_audit_thread_started && !g_audit_stop) {
        g_audit_stop = 1;
        th_local = g_audit_thread;
        joinable = 1;
        pthread_cond_broadcast(&g_audit_cv);
    }
    pthread_mutex_unlock(&g_audit_mu);
    if (joinable) {
        pthread_join(th_local, NULL);
        // Mark thread_started = 0 so a late atexit chain doesn't try
        // to re-join. Take the lock for the write so any concurrent
        // producer sees a consistent state.
        pthread_mutex_lock(&g_audit_mu);
        g_audit_thread_started = 0;
        pthread_mutex_unlock(&g_audit_mu);
    }
}

// Enqueue an event into the ring. Drop-oldest on overflow; the worker
// will emit "audit-overflow dropped=N" on its next drain. Always
// succeeds (no -1 path).
static void audit_enqueue(const char *event, const char *detail) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    pthread_mutex_lock(&g_audit_mu);
    if (g_audit_count == SSO_AUDIT_RING_SIZE) {
        // Drop oldest.
        g_audit_tail = (g_audit_tail + 1) % SSO_AUDIT_RING_SIZE;
        g_audit_count--;
        g_audit_dropped++;
    }
    audit_evt_t *slot = &g_audit_ring[g_audit_head];
    slot->ts = ts;
    snprintf(slot->event,  sizeof slot->event,  "%s",
             (event && event[0]) ? event : "-");
    snprintf(slot->detail, sizeof slot->detail, "%s",
             (detail && detail[0]) ? detail : "");
    g_audit_head = (g_audit_head + 1) % SSO_AUDIT_RING_SIZE;
    g_audit_count++;
    pthread_cond_broadcast(&g_audit_cv);
    pthread_mutex_unlock(&g_audit_mu);
}

// --- Atexit ---------------------------------------------------------
//
// Registered once by sso_audit_start. Runs on normal exit: enqueues
// the matching "end" line so it goes through the writer in order, then
// drains + joins the worker. Joining on the exit path is bounded (the
// worker can be stuck in at most one per-event flock, ~100 ms typical).

static void atexit_writer(void) {
    char detail[32];
    snprintf(detail, sizeof(detail), "exit=%d", g_exit_code);
    // sso_audit_event picks between enqueue (worker up) and synchronous
    // append (worker failed to spawn earlier), so the "end" line always
    // makes it to disk before we join.
    (void) sso_audit_event("end", detail);
    audit_worker_close();
}

// --- Public API -----------------------------------------------------

int sso_audit_start(const char *tool, const char *detail) {
    if (tool && tool[0]) {
        snprintf(g_tool, sizeof(g_tool), "%s", tool);
    } else {
        g_tool[0] = '\0';
    }
    if (!g_atexit_armed) {
        atexit(atexit_writer);
        g_atexit_armed = 1;
    }
    int worker_ok = (audit_worker_init() == 0);
    if (worker_ok) {
        audit_enqueue("start", detail);
        return 0;
    }
    // Fall back to a synchronous write if the worker thread couldn't be
    // spawned. The atexit path stays armed; it'll just no-op the join.
    return append_line("start", detail);
}

int sso_audit_event(const char *event, const char *detail) {
    // If sso_audit_start has run and the worker is up, go through the
    // queue (non-blocking). Otherwise, fall back to a direct synchronous
    // write so events emitted before init aren't lost.
    pthread_mutex_lock(&g_audit_mu);
    int worker_up = g_audit_thread_started;
    pthread_mutex_unlock(&g_audit_mu);
    if (worker_up) {
        audit_enqueue(event, detail);
        return 0;
    }
    return append_line(event, detail);
}

void sso_audit_set_exit_code(int code) {
    g_exit_code = code;
}
