#include "sso_audit.h"

#include "sso_paths.h"

#include <errno.h>
#include <fcntl.h>
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

static void iso_utc(char *out, size_t out_size) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    gmtime_r(&ts.tv_sec, &tm);
    snprintf(out, out_size,
             "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             (long) (ts.tv_nsec / 1000000));
}

// Sanitize one free-form field: replace tabs and newlines with spaces
// so they don't break the tab-separated record format.
static void sanitize(char *s) {
    if (!s) return;
    for (; *s; ++s) {
        if (*s == '\t' || *s == '\n' || *s == '\r') *s = ' ';
    }
}

static int append_line(const char *event, const char *detail) {
    const char *path = resolve_log_path();
    sso_mkdir_p_for_file(path);
    int fd = open(path, O_WRONLY | O_APPEND | O_CREAT, 0664);
    if (fd < 0) return -1;
    char ts[64];
    iso_utc(ts, sizeof(ts));
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
    char detail_buf[256];
    snprintf(detail_buf, sizeof(detail_buf), "%s",
             (detail && detail[0]) ? detail : "");
    sanitize(detail_buf);
    char line[SSO_AUDIT_BUF];
    int n = snprintf(line, sizeof(line),
                     "%s\t%s\t%s\t%d\t%s\t%s\n",
                     ts, user_buf, tool_buf, (int) getpid(),
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

static void atexit_writer(void) {
    char detail[32];
    snprintf(detail, sizeof(detail), "exit=%d", g_exit_code);
    append_line("end", detail);
}

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
    return append_line("start", detail);
}

int sso_audit_event(const char *event, const char *detail) {
    return append_line(event, detail);
}

void sso_audit_set_exit_code(int code) {
    g_exit_code = code;
}
