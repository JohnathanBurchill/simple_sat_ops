/*

    Simple Satellite Operations  utils/gnss_opm.c

    Build an OPM (orbit parameter message) from the satellite's own GNSS fix
    so the trajectory can be propagated and uploaded to the SpaceX Space
    Safety conjunction-screening API.

    FrontierSat's NovAtel receiver returns a BESTXYZA log (an Earth-fixed
    position/velocity solution with per-axis 1-sigma uncertainties) in a
    gnss_send_cmd_ascii telecommand response. This tool finds those responses
    in the packet DB, reassembles the multi-packet ones, checks the NovAtel
    CRC, picks the best valid fix, and writes the state vector -- with its
    uncertainties -- in the OPM form the space_safety_manager (`ssm`) tool
    reads. `ssm propagate` / `ssm upload-opm` then turns it into a CCSDS OEM
    (growing the uncertainty into an RTN covariance over the window) and
    uploads it. Read-only on the DB.

    Examples:
      gnss_opm                          # newest CRC-ok SOL_COMPUTED fix
      gnss_opm --since=7d --list        # list candidate fixes, newest first
      gnss_opm --id=17769 > frontiersat.opm
      gnss_opm --name="FrontierSat (CTS-Sat-1)" --hard-body-radius=0.71

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

#include "argparse.h"
#include "beacon_cts1.h"
#include "bestxyz.h"
#include "gnss_frag.h"
#include "packet_db.h"
#include "shortarc.h"
#include "sso_version.h"

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef WITH_SQLITE3
int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "gnss_opm")) return 0;
    (void)argc; (void)argv;
    fprintf(stderr,
            "gnss_opm: built without sqlite3 support. Install\n"
            "libsqlite3-dev (or `brew install sqlite`) and rebuild.\n");
    return 1;
}
#else

#include <sqlite3.h>

#define TCMD_TYPE     0x04
#define TCMD_HDR      COMMS_TCMD_RESPONSE_HEADER_SIZE
#define TCMD_MAXDATA  COMMS_TCMD_RESPONSE_PACKET_MAX_DATA_BYTES_PER_PACKET

// Parse a duration spec (90s | 30m | 2h | 1d) into seconds. Returns <0 on error.
static double parse_duration_s(const char *spec)
{
    if (spec == NULL || spec[0] == '\0') return -1.0;
    size_t len = strlen(spec);
    char unit = spec[len - 1];
    char *endp = NULL;
    long n = strtol(spec, &endp, 10);
    if (endp == spec || endp != spec + len - 1 || n <= 0) return -1.0;
    switch (unit) {
        case 's': return (double)n;
        case 'm': return (double)n * 60.0;
        case 'h': return (double)n * 3600.0;
        case 'd': return (double)n * 86400.0;
        default:  return -1.0;
    }
}

// Max fragments accumulated for one reception (also sizes fix_t.ids, so the
// per-fix id list can hold every contributing fragment -- a smaller cap made
// an --id= belonging to a later fragment silently fail to match).
#define GNSS_FRAG_MAX 260

typedef struct {
    const char *db_path;
    const char *since;
    const char *until;
    const char *name;
    const char *object_id;
    double      hard_body_radius;
    long long   want_id;        // 0 = pick best
    const char *fit_window;     // e.g. "30m": short-arc LS over fixes in window
    int         list;
    int         allow_insufficient;
} args_t;

#define OPTW 26

static int parse_args(args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;
        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (gnss_starts_with(arg, "--db=") || help) {
            if (help) parse_help_line(OPTW, "--db=<path>", "override default DB path ($SSO_PACKET_DB or the default)");
            else a->db_path = arg + 5;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--since=") || help) {
            if (help) parse_help_line(OPTW, "--since=<spec>", "24h | 7d | ISO-8601 (default: all)");
            else a->since = arg + 8;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--until=") || help) {
            if (help) parse_help_line(OPTW, "--until=<spec>", "same syntax as --since (default: now)");
            else a->until = arg + 8;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--name=") || help) {
            if (help) parse_help_line(OPTW, "--name=<s>", "OPM object name (default: FrontierSat)");
            else a->name = arg + 7;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--object-id=") || help) {
            if (help) parse_help_line(OPTW, "--object-id=<s>", "OPM object id (optional)");
            else a->object_id = arg + 12;
            matched = 1;
        }
        if (gnss_starts_with(arg, "--hard-body-radius=") || help) {
            if (help) parse_help_line(OPTW, "--hard-body-radius=<m>", "hard-body radius in metres (default: 0.71)");
            else a->hard_body_radius = atof(arg + 19);
            matched = 1;
        }
        if (gnss_starts_with(arg, "--id=") || help) {
            if (help) parse_help_line(OPTW, "--id=<n>", "use the GNSS fix containing packet id n");
            else a->want_id = atoll(arg + 5);
            matched = 1;
        }
        if (gnss_starts_with(arg, "--fit-window=") || help) {
            if (help) parse_help_line(OPTW, "--fit-window=<dur>", "short-arc least-squares fit over fixes within <dur> (e.g. 30m) before the newest");
            else a->fit_window = arg + 13;
            matched = 1;
        }
        if (strcmp(arg, "--list") == 0 || help) {
            if (help) parse_help_line(OPTW, "--list", "list candidate fixes (newest first), write no OPM");
            else a->list = 1;
            matched = 1;
        }
        if (strcmp(arg, "--allow-insufficient") == 0 || help) {
            if (help) parse_help_line(OPTW, "--allow-insufficient", "accept a fix that is not SOL_COMPUTED (NOT for upload)");
            else a->allow_insufficient = 1;
            matched = 1;
        }
        if ((strcmp(arg, "-V") == 0 || strcmp(arg, "--version") == 0) || help) {
            if (help) parse_help_line(OPTW, "-V, --version", "print version and exit");
            matched = 1;
        }
        if (!help && !matched) {
            fprintf(stderr, "gnss_opm: unknown option '%s' (try --help)\n", arg);
            return PARSE_ERROR;
        }
    }
    return help ? PARSE_HELP : PARSE_OK;
}

// One accepted GNSS fix.
typedef struct {
    bestxyz_t b;
    char      ts_received[40];
    long long ids[GNSS_FRAG_MAX];
    int       n_ids;
    int       crc_ok;
    double    epoch_s;     // GNSS solution epoch, GPS seconds (week*604800 + sow)
} fix_t;

// If the reassembled reception is a CRC-ok BESTXYZA, fill *out and return 1.
static int parse_fix(const gnss_frag_t *frags, int n, fix_t *out)
{
    static unsigned char buf[65536];
    int len = gnss_reassemble(frags, n, buf, sizeof buf);
    char *msg = (char *)buf;
    char *marker = strstr(msg, "GNSS Response (");
    if (marker == NULL) return 0;

    // Trim to the firmware-declared receiver length so trailer/padding can't
    // confuse the parser.
    int decl = 0;
    char *colon = strstr(marker, "): ");
    if (sscanf(marker, "GNSS Response (%d chars)", &decl) == 1 && colon && decl > 0) {
        int end = (int)(colon + 3 - msg) + decl;
        if (end < len) { len = end; msg[len] = '\0'; }
    }
    if (strstr(msg, "BESTXYZA") == NULL) return 0;

    char err[96];
    if (bestxyz_parse(msg, &out->b, err, sizeof err) != 0) return 0;
    out->crc_ok = out->b.crc_present && out->b.crc_ok;
    out->epoch_s = (double)out->b.gps_week * 604800.0 + out->b.gps_sow;
    snprintf(out->ts_received, sizeof out->ts_received, "%s", frags[0].ts_received);
    out->n_ids = 0;
    for (int i = 0; i < n && i < GNSS_FRAG_MAX; ++i) out->ids[out->n_ids++] = frags[i].id;
    return 1;
}

static int fix_is_usable(const fix_t *f, const args_t *a)
{
    if (!f->crc_ok) return 0;
    if (!a->allow_insufficient && strcmp(f->b.pos_sol_status, "SOL_COMPUTED") != 0) return 0;
    return 1;
}

static void print_candidate(const fix_t *f)
{
    fprintf(stderr, "  %s  id %lld  %s/%s  %d/%d SV  CRC %s  clk %s\n",
            f->ts_received, f->ids[0], f->b.pos_sol_status, f->b.pos_type,
            f->b.num_sol_sv, f->b.num_sv, f->crc_ok ? "ok" : "BAD",
            f->b.time_status);
}

// Age of the fix in hours (now - GNSS epoch), or -1 if the epoch won't convert.
static double fix_age_hours(const bestxyz_t *b)
{
    int y, mo, d, h, mi; double s;
    bestxyz_gps_to_utc(b->gps_week, b->gps_sow, BESTXYZ_DEFAULT_LEAP_SECONDS,
                       &y, &mo, &d, &h, &mi, &s);
    struct tm tm = {0};
    tm.tm_year = y - 1900; tm.tm_mon = mo - 1; tm.tm_mday = d;
    tm.tm_hour = h; tm.tm_min = mi; tm.tm_sec = (int)s;
    time_t epoch = timegm(&tm);
    if (epoch == (time_t)-1) return -1.0;
    return difftime(time(NULL), epoch) / 3600.0;
}

// Score a usable fix against the upload rules of thumb (see USER_MANUAL.md,
// "When a fix is good enough to upload"). Writes the shortfalls into reasons.
// Returns 2 = recommended, 1 = usable with caution, 0 = not recommended.
static int fix_recommendation(const fix_t *f, char *reasons, size_t rsz)
{
    reasons[0] = '\0';
    int hard = 0, caution = 0;
    #define NOTE(...) do { \
        size_t _l = strlen(reasons); \
        if (_l) snprintf(reasons + _l, rsz - _l, "; "); \
        _l = strlen(reasons); \
        snprintf(reasons + _l, rsz - _l, __VA_ARGS__); \
    } while (0)

    if (strcmp(f->b.pos_type, "SINGLE") != 0) { caution = 1; NOTE("type %s", f->b.pos_type); }

    if (f->b.num_sol_sv < 4)      { hard = 1;    NOTE("only %d SV", f->b.num_sol_sv); }
    else if (f->b.num_sol_sv < 6) { caution = 1; NOTE("%d SV (<6)", f->b.num_sol_sv); }

    double smax = f->b.pos_sigma[0];
    if (f->b.pos_sigma[1] > smax) smax = f->b.pos_sigma[1];
    if (f->b.pos_sigma[2] > smax) smax = f->b.pos_sigma[2];
    if (smax > 50.0)      { hard = 1;    NOTE("sigma %.0f m (>50)", smax); }
    else if (smax > 25.0) { caution = 1; NOTE("sigma %.0f m (>25)", smax); }

    // A fine-precision clock (FINESTEERING / FINE / FINEADJUSTING /
    // FINEBACKUPSTEERING -- all the "FINE*" states) gives a trustworthy epoch.
    // FREEWHEELING / COARSE* / SATTIME mean the time reference is coasting or
    // only coarse, so the few-metre sigma understates the true along-track
    // uncertainty (epoch error maps to ~7.5 km/s of along-track position).
    if (strncmp(f->b.time_status, "FINE", 4) != 0) {
        caution = 1; NOTE("%s clock", f->b.time_status);
    }

    double age = fix_age_hours(&f->b);
    if (age >= 0.0) {
        if (age > 504.0)      { hard = 1;    NOTE("%.0f h old (>504 h API window)", age); }
        else if (age > 24.0)  { caution = 1; NOTE("%.0f h old (>24 h)", age); }
        else if (age > 6.0)   { caution = 1; NOTE("%.1f h old (>6 h ideal)", age); }
    }
    #undef NOTE

    if (hard) return 0;
    if (caution) return 1;
    return 2;
}

// Candidates collected for the --list view so they can be sorted newest-first.
static fix_t *g_cands = NULL;
static int    g_ncands = 0, g_capcands = 0;

static void collect_candidate(const fix_t *f)
{
    if (g_ncands == g_capcands) {
        int cap = g_capcands ? g_capcands * 2 : 64;
        fix_t *t = realloc(g_cands, (size_t)cap * sizeof *t);
        if (t == NULL) return;
        g_cands = t; g_capcands = cap;
    }
    g_cands[g_ncands++] = *f;
}

static int cmp_cand_desc(const void *a, const void *b)
{
    return strcmp(((const fix_t *)b)->ts_received, ((const fix_t *)a)->ts_received);
}

static void write_opm(FILE *fp, const fix_t *f, const args_t *a,
                      const shortarc_fit_t *fit, const char *window)
{
    int y, mo, d, h, mi; double s;
    bestxyz_gps_to_utc(f->b.gps_week, f->b.gps_sow, BESTXYZ_DEFAULT_LEAP_SECONDS,
                       &y, &mo, &d, &h, &mi, &s);

    fprintf(fp, "# OPM generated by simple_sat_ops gnss_opm\n");
    if (fit != NULL) {
        fprintf(fp, "# short-arc least-squares fit: %d fixes within %s, epoch = newest fix (id %lld)\n",
                fit->n_obs, window ? window : "?", f->ids[0]);
        fprintf(fp, "# position residual RMS %.2f m, %d Gauss-Newton iterations\n",
                fit->pos_rms, fit->iterations);
        fprintf(fp, "# sigmas below are the fit covariance diagonal (the arc tightens velocity)\n");
    } else {
        fprintf(fp, "# GNSS fix: packet id %lld, BESTXYZA %s/%s, %d/%d SV, CRC %08x\n",
                f->ids[0], f->b.pos_sol_status, f->b.pos_type,
                f->b.num_sol_sv, f->b.num_sv, f->b.crc_read);
        fprintf(fp, "# r_ecef_sigma_m / v_ecef_sigma_m_per_s are the receiver 1-sigma\n");
    }
    fprintf(fp, "# epoch from GPS week %d sow %.3f, leap=%d s; %s clock\n",
            f->b.gps_week, f->b.gps_sow, BESTXYZ_DEFAULT_LEAP_SECONDS, f->b.time_status);
    fprintf(fp, "# ssm rotates the sigmas ECEF->RTN and grows them over the propagation window.\n");
    fprintf(fp, "- name: %s\n", a->name);
    if (a->object_id && a->object_id[0])
        fprintf(fp, "  object_id: %s\n", a->object_id);
    fprintf(fp, "  date: %04d-%02d-%02dT%02d:%02d:%09.6fZ\n", y, mo, d, h, mi, s);
    fprintf(fp, "  r_ecef_m: [%.4f, %.4f, %.4f]\n",
            f->b.pos[0], f->b.pos[1], f->b.pos[2]);
    fprintf(fp, "  v_ecef_m_per_s: [%.4f, %.4f, %.4f]\n",
            f->b.vel[0], f->b.vel[1], f->b.vel[2]);
    fprintf(fp, "  r_ecef_sigma_m: [%.4f, %.4f, %.4f]\n",
            f->b.pos_sigma[0], f->b.pos_sigma[1], f->b.pos_sigma[2]);
    fprintf(fp, "  v_ecef_sigma_m_per_s: [%.6f, %.6f, %.6f]\n",
            f->b.vel_sigma[0], f->b.vel_sigma[1], f->b.vel_sigma[2]);
    fprintf(fp, "  hard_body_radius_m: %.3f\n", a->hard_body_radius);
}

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "gnss_opm")) return 0;

    args_t cfg = {0};
    cfg.name = "FrontierSat";
    cfg.hard_body_radius = 0.71;
    switch (parse_args(&cfg, argc, argv, 0)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
        default: break;
    }

    char db_default[1024];
    const char *db_path = cfg.db_path;
    if (db_path == NULL) {
        if (packet_db_default_path(db_default, sizeof db_default) != 0) {
            fprintf(stderr, "gnss_opm: no DB path (set $SSO_PACKET_DB or pass --db=<path>)\n");
            return 1;
        }
        db_path = db_default;
    }

    char since_iso[40] = {0}, until_iso[40] = {0};
    if (cfg.since && gnss_parse_time_spec(cfg.since, since_iso, sizeof since_iso) != 0) {
        fprintf(stderr, "gnss_opm: bad --since=%s\n", cfg.since); return 1;
    }
    if (cfg.until && gnss_parse_time_spec(cfg.until, until_iso, sizeof until_iso) != 0) {
        fprintf(stderr, "gnss_opm: bad --until=%s\n", cfg.until); return 1;
    }

    sqlite3 *db = NULL;
    if (sqlite3_open_v2(db_path, &db, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
        fprintf(stderr, "gnss_opm: cannot open %s: %s\n",
                db_path, db ? sqlite3_errmsg(db) : "open failed");
        if (db) sqlite3_close(db);
        return 1;
    }

    const char *sql =
        "SELECT id, ts_received, payload FROM packet "
        "WHERE packet_type=?1 AND length(payload) >= ?2 "
        "ORDER BY substr(payload,2,8), ts_received, id";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) {
        fprintf(stderr, "gnss_opm: query failed: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
        return 1;
    }
    sqlite3_bind_int(st, 1, TCMD_TYPE);
    sqlite3_bind_int(st, 2, TCMD_HDR + 1);

    gnss_frag_t recv[GNSS_FRAG_MAX];
    int nrecv = 0;
    unsigned char curkey[8];
    int have_key = 0, last_seq = 0;

    #define CONSIDER() do { \
        if (nrecv > 0) { \
            fix_t f = {0}; \
            if (parse_fix(recv, nrecv, &f)) { \
                int in_range = (!since_iso[0] || strcmp(f.ts_received, since_iso) >= 0) \
                            && (!until_iso[0] || strcmp(f.ts_received, until_iso) <= 0); \
                int id_match = 1; \
                if (cfg.want_id) { id_match = 0; \
                    for (int k = 0; k < f.n_ids; ++k) if (f.ids[k] == cfg.want_id) id_match = 1; } \
                if (in_range && id_match && fix_is_usable(&f, &cfg)) \
                    collect_candidate(&f); \
            } \
            for (int i = 0; i < nrecv; ++i) { free(recv[i].payload); } \
            nrecv = 0; \
        } \
    } while (0)

    while (sqlite3_step(st) == SQLITE_ROW) {
        const unsigned char *pl = sqlite3_column_blob(st, 2);
        int pl_len = sqlite3_column_bytes(st, 2);
        if (pl == NULL || pl_len < TCMD_HDR + 1) continue;
        int seq = pl[12];
        if (seq < 1) continue;
        unsigned char key[8];
        memcpy(key, pl + 1, 8);

        int new_group = !have_key || memcmp(key, curkey, 8) != 0;
        if (new_group) { CONSIDER(); memcpy(curkey, key, 8); have_key = 1; last_seq = 0; }
        else if (seq <= last_seq) { CONSIDER(); }
        if (nrecv >= (int)(sizeof recv / sizeof recv[0])) CONSIDER();

        gnss_frag_t *fr = &recv[nrecv++];
        fr->id = sqlite3_column_int64(st, 0);
        snprintf(fr->ts_received, sizeof fr->ts_received, "%s",
                 (const char *)sqlite3_column_text(st, 1));
        memcpy(fr->tskey, key, 8);
        fr->seq = seq;
        fr->payload_len = pl_len;
        fr->payload = malloc((size_t)pl_len);
        if (fr->payload) memcpy(fr->payload, pl, (size_t)pl_len);
        last_seq = seq;
    }
    CONSIDER();
    sqlite3_finalize(st);
    sqlite3_close(db);

    qsort(g_cands, (size_t)g_ncands, sizeof *g_cands, cmp_cand_desc);   // newest first

    if (cfg.list) {
        int n_rec = 0, n_caution = 0;
        for (int i = 0; i < g_ncands; ++i) {
            print_candidate(&g_cands[i]);
            char reasons[160];
            int v = fix_recommendation(&g_cands[i], reasons, sizeof reasons);
            if (v == 2)      { fprintf(stderr, "      -> RECOMMENDED for upload\n"); n_rec++; }
            else if (v == 1) { fprintf(stderr, "      -> usable with caution: %s\n", reasons); n_caution++; }
            else             { fprintf(stderr, "      -> NOT recommended: %s\n", reasons); }
        }
        fprintf(stderr,
                "\n%d usable fix%s: %d recommended, %d usable with caution.\n",
                g_ncands, g_ncands == 1 ? "" : "es", n_rec, n_caution);
        if (n_rec == 0 && g_ncands > 0)
            fprintf(stderr,
                    "No fix clears every rule of thumb; the newest "
                    "\"usable with caution\" one is the best available.\n");
        free(g_cands);
        return g_ncands ? 0 : 1;
    }

    if (g_ncands == 0) {
        fprintf(stderr,
                "gnss_opm: no usable GNSS fix found%s.\n"
                "Need a CRC-ok BESTXYZA%s. Try --list, a wider --since, or "
                "--allow-insufficient (not for upload).\n",
                cfg.want_id ? " for that --id" : "",
                cfg.allow_insufficient ? "" : " with SOL_COMPUTED");
        free(g_cands);
        return 1;
    }

    // The newest usable fix is the OPM epoch (and the fit's a priori).
    fix_t chosen = g_cands[0];
    shortarc_fit_t fit;
    int did_fit = 0;

    if (cfg.fit_window) {
        double win_s = parse_duration_s(cfg.fit_window);
        if (win_s <= 0.0) {
            fprintf(stderr, "gnss_opm: bad --fit-window=%s\n", cfg.fit_window);
            free(g_cands);
            return 1;
        }
        // Gather fixes within [epoch - window, epoch] (epoch = newest fix).
        shortarc_obs_t obs[64];
        int nobs = 0;
        for (int i = 0; i < g_ncands && nobs < 64; ++i) {
            double dt = g_cands[i].epoch_s - chosen.epoch_s;   // <= 0
            if (dt > 1e-6 || dt < -win_s - 1e-6) continue;
            shortarc_obs_t *o = &obs[nobs++];
            o->dt = dt;
            for (int k = 0; k < 3; ++k) {
                o->pos[k]       = g_cands[i].b.pos[k];
                o->vel[k]       = g_cands[i].b.vel[k];
                o->pos_sigma[k] = g_cands[i].b.pos_sigma[k];
                o->vel_sigma[k] = g_cands[i].b.vel_sigma[k];
            }
        }
        if (nobs >= 2 && shortarc_fit(obs, nobs, &fit) == 0 && fit.ok) {
            for (int k = 0; k < 3; ++k) {
                chosen.b.pos[k]       = fit.pos[k];
                chosen.b.vel[k]       = fit.vel[k];
                chosen.b.pos_sigma[k] = sqrt(fit.cov[k][k]);
                chosen.b.vel_sigma[k] = sqrt(fit.cov[k + 3][k + 3]);
            }
            did_fit = 1;
            fprintf(stderr,
                    "gnss_opm: short-arc LS over %d fixes in %s; residual RMS %.2f m, %d iter; "
                    "velocity sigma %.4f m/s (was %.4f from one fix).\n",
                    fit.n_obs, cfg.fit_window, fit.pos_rms, fit.iterations,
                    sqrt(fit.cov[3][3]), obs[nobs - 1].vel_sigma[0]);
        } else if (nobs < 2) {
            fprintf(stderr,
                    "gnss_opm: only %d fix in the %s window -- not enough to fit; "
                    "using the single newest fix.\n", nobs, cfg.fit_window);
        } else {
            fprintf(stderr,
                    "gnss_opm: short-arc fit over %d fixes did not converge "
                    "(window too long for the dynamics model, or inconsistent fixes?); "
                    "using the single newest fix.\n", nobs);
        }
    }

    if (!did_fit)
        fprintf(stderr,
                "gnss_opm: using fix from %s (packet id %lld), %s/%s, %d/%d SV.\n",
                chosen.ts_received, chosen.ids[0], chosen.b.pos_sol_status, chosen.b.pos_type,
                chosen.b.num_sol_sv, chosen.b.num_sv);

    write_opm(stdout, &chosen, &cfg, did_fit ? &fit : NULL, cfg.fit_window);
    free(g_cands);
    return 0;
}

#endif
