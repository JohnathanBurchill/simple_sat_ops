/*

    Simple Satellite Operations  next_in_queue.c

    Copyright (C) 2025  Johnathan K Burchill

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
#include "state.h"
#include "prediction.h"
#include "oem.h"
#include "satellite_status.h"
#include "tle_csv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sgp4sdp4.h>

// Pull an exactly-8-digit run (a YYYYMMDD date) out of a filename and
// return it as a long, or -1 if there isn't one. "tle-20260529.tle"
// yields 20260529; a 14-digit timestamp's leading 8 digits also match.
static long extract_yyyymmdd(const char *name)
{
    for (const char *p = name; *p != '\0'; p++) {
        if (!isdigit((unsigned char) *p)) continue;
        int k = 0;
        while (isdigit((unsigned char) p[k])) k++;
        if (k == 8) {
            char buf[9];
            memcpy(buf, p, 8);
            buf[8] = '\0';
            return atol(buf);
        }
        p += k > 0 ? k - 1 : 0;  // skip the run; loop's p++ steps past it
    }
    return -1;
}

// Recursively scan `dir` for *.tle files whose name carries a YYYYMMDD
// date (the tle-YYYYMMDD.tle convention) and return the path of the one
// with the largest date via out. *best carries the running max across
// the recursion; start it at -1. Returns 0 if at least one dated .tle
// was found in the tree, -1 otherwise (including an unreadable dir).
static int find_latest_dated_tle(const char *dir, char *out, size_t cap,
                                 long *best)
{
    DIR *d = opendir(dir);
    if (d == NULL) return -1;
    int found = 0;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char child[1024];
        int n = snprintf(child, sizeof child, "%s/%s", dir, de->d_name);
        if (n < 0 || (size_t) n >= sizeof child) continue;
        struct stat st;
        if (stat(child, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            if (find_latest_dated_tle(child, out, cap, best) == 0) found = 1;
        } else if (S_ISREG(st.st_mode)) {
            size_t nlen = strlen(de->d_name);
            if (nlen < 4 || strcmp(de->d_name + nlen - 4, ".tle") != 0) continue;
            long date = extract_yyyymmdd(de->d_name);
            if (date < 0) continue;
            found = 1;
            if (date > *best) {
                *best = date;
                snprintf(out, cap, "%s", child);
            }
        }
    }
    closedir(d);
    return found ? 0 : -1;
}

// Resolve the TLE file to use when only a satellite name was given.
// Order: the newest dated tle-YYYYMMDD.tle under /<satname>/TLEs, then
// the shared default active.tle (the caller filters it by name). Returns
// 0 and fills out on success, -1 if neither source exists.
static int resolve_named_tle(const char *satname, char *out, size_t cap)
{
    char dir[1024];
    int n = snprintf(dir, sizeof dir, "/%s/TLEs", satname);
    if (n > 0 && (size_t) n < sizeof dir) {
        long best = -1;
        if (find_latest_dated_tle(dir, out, cap, &best) == 0) {
            fprintf(stderr, "next_in_queue: using TLE %s\n", out);
            return 0;
        }
    }
    char active[1024];
    if (tle_default_path(active, sizeof active) == 0
        && access(active, R_OK) == 0) {
        snprintf(out, cap, "%s", active);
        fprintf(stderr,
                "next_in_queue: no dated TLE under /%s/TLEs; "
                "using %s (looking for '%s')\n",
                satname, out, satname);
        return 0;
    }
    return -1;
}

// Print the multi-line usage synopsis (the four invocation forms + the
// one-paragraph summary). Shared by the --help / --help-full header and
// the post-parse "0 positionals is ambiguous" error, which writes it to
// stderr. The per-option / per-positional detail is printed by parse_args
// in help mode; the long EXAMPLES + NOTES live in parse_args's
// HELP_FULL epilog.
static void print_synopsis(FILE *dest, const char *name)
{
    fprintf(dest,
        "usage:\n"
        "  %s <satellite_name> [options]\n"
        "  %s <min_alt_km> <max_alt_km> [<satellite_name>] [options]\n"
        "  %s --tle <path> [<satellite_name>] [options]\n"
        "  %s --trajectory-id=<id> [options]\n"
        "\n"
        "Scan a TLE file (or a propagated SSM trajectory) for upcoming passes\n"
        "over the ground station and report the soonest (or every) match.\n"
        "Read-only; no hardware commands.\n",
        name, name, name, name);
}

// Parsed command-line configuration. parse_args() fills this; main() copies
// the fields out into working locals so the (large) prediction body is
// unchanged. Positionals are POLYMORPHIC -- their meaning depends on count
// and on whether --tle / --trajectory-id is present -- so they are collected
// generically here and interpreted in main() after parse_args returns.
typedef struct {
    double site_latitude;
    double site_longitude;
    double site_altitude;
    int list_all;
    int max_passes;
    int show_radio_info;
    double min_minutes_away;
    double max_minutes_away;
    int max_minutes_user_set;
    double min_elevation;
    double max_elevation;
    const char *regex;
    int regex_ignore_case;
    int with_constellations;
    int reverse;
    int tle_explicit;
    char *tles_filename;
    const char *trajectory_id;
    const char *t0_str;
    double min_altitude_km;
    double max_altitude_km;
    int minutes_offset;
    const char *positionals[8];
    int n_positionals;
} niq_args_t;

// Option column width: the widest label below ("--t0=<yyyy-mm-ddThh:mm:ss>") +
// a small margin. See src/cli/argparse.h for the parse_args convention.
#define OPTW 28

// Parse argv into *a (help == 0), or print the synopsis + one right-aligned
// help line per option/positional and return (help != 0). Each block's test
// carries "|| help", so help mode falls through and prints them all. The
// polymorphic positionals are merely COLLECTED here (count/context is
// interpreted in main); a token consumed by the --tle space form is taken
// inside that block so the collector never sees it.
static int parse_args(niq_args_t *a, int argc, char **argv, int help)
{
    if (help) print_synopsis(stdout, "next_in_queue");
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        // Polymorphic positionals: any non-option token (a lone "-" counts).
        // Collected generically; main() interprets them by count/context.
        // Declared first so the <...> forms list above the --options in help.
        if ((arg[0] != '-' || strcmp(arg, "-") == 0) || help) {
            if (help) {
                parse_help_line(OPTW, "<satellite_name>", "named form: one object; TLE auto-discovered, passes for next week");
                parse_help_line(OPTW, "<min_alt_km> <max_alt_km>", "catalog-scan form: scan default TLE in this altitude band (optional 3rd arg = name prefix)");
            } else if (a->n_positionals < (int)(sizeof a->positionals / sizeof a->positionals[0])) {
                a->positionals[a->n_positionals++] = arg;
            } else {
                fprintf(stderr, "too many positional arguments\n");
                return PARSE_ERROR;
            }
            matched = 1;
        }
        if (strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help", "short help (this message)");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "--help-full") == 0 || help) {
            if (help) parse_help_line(OPTW, "--help-full", "detailed help with examples");
            else { parse_args(a, argc, argv, HELP_FULL); return PARSE_HELP; }
            matched = 1;
        }
        if (strcmp(arg, "--tle") == 0 || help) {
            if (help) parse_help_line(OPTW, "--tle <path>", "TLE file (2/3-line); space form is tab-completable");
            else {
                // arg is argv[t+1]; its value is the next token argv[t+2].
                // Consume it (++t) so the positional collector never sees it.
                if (t + 2 >= argc) {
                    fprintf(stderr, "%s: --tle requires a file path\n", argv[0]);
                    return PARSE_ERROR;
                }
                a->tles_filename = tle_path_resolve(argv[t + 2]);
                a->tle_explicit = 1;
                ++t;
            }
            matched = 1;
        }
        if (strncmp("--tle=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--tle=<path>", "legacy TLE-path form (default: active.tle)");
            else {
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->tles_filename = tle_path_resolve(arg + 6);
                a->tle_explicit = 1;
            }
            matched = 1;
        }
        if (strncmp("--trajectory-id=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--trajectory-id=<id>", "plan against `ssm trajectory <id>` (mutually exclusive with --tle)");
            else {
                if (strlen(arg) < 17) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->trajectory_id = arg + 16;
            }
            matched = 1;
        }
        if (strcmp("--list", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--list", "print all matching passes (default: one)");
            else a->list_all = 1;
            matched = 1;
        }
        if (strcmp("--reverse", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--reverse", "sort latest-first");
            else a->reverse = 1;
            matched = 1;
        }
        if (strncmp("--max-passes=", arg, 13) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-passes=<n>", "limit output to n passes");
            else {
                if (strlen(arg) < 14) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->max_passes = atoi(arg + 13);
            }
            matched = 1;
        }
        if (strcmp("--show-radio-info", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--show-radio-info", "annotate with amateur-radio info from active_radios.txt next to the TLE");
            else a->show_radio_info = 1;
            matched = 1;
        }
        if (strncmp("--min-altitude-km=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-altitude-km=<km>", "minimum orbital altitude (default 0)");
            else {
                if (strlen(arg) < 19) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->min_altitude_km = atof(arg + 18);
            }
            matched = 1;
        }
        if (strncmp("--max-altitude-km=", arg, 18) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-altitude-km=<km>", "maximum orbital altitude (default unlimited)");
            else {
                if (strlen(arg) < 19) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->max_altitude_km = atof(arg + 18);
            }
            matched = 1;
        }
        if (strncmp("--min-minutes=", arg, 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-minutes=<n>", "minimum minutes until AOS (default 0)");
            else {
                if (strlen(arg) < 15) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->min_minutes_away = atof(arg + 14);
            }
            matched = 1;
        }
        if (strncmp("--max-minutes=", arg, 14) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-minutes=<n>", "maximum minutes until AOS (default 1440)");
            else {
                if (strlen(arg) < 15) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->max_minutes_away = atof(arg + 14);
                a->max_minutes_user_set = 1;
            }
            matched = 1;
        }
        if (strncmp("--min-elevation=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--min-elevation=<deg>", "minimum peak elevation (default 0)");
            else {
                if (strlen(arg) < 17) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->min_elevation = atof(arg + 16);
            }
            matched = 1;
        }
        if (strncmp("--max-elevation=", arg, 16) == 0 || help) {
            if (help) parse_help_line(OPTW, "--max-elevation=<deg>", "maximum peak elevation (default 90)");
            else {
                if (strlen(arg) < 17) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->max_elevation = atof(arg + 16);
            }
            matched = 1;
        }
        if (strncmp("--minutes-offset=", arg, 17) == 0 || help) {
            if (help) parse_help_line(OPTW, "--minutes-offset=<n>", "advance `now` by n minutes for planning");
            else {
                if (strlen(arg) < 18) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->minutes_offset = atoi(arg + 17);
            }
            matched = 1;
        }
        if (strncmp("--t0=", arg, 5) == 0 || help) {
            if (help) parse_help_line(OPTW, "--t0=<yyyy-mm-ddThh:mm:ss>", "absolute UTC start time (default now; --minutes-offset on top)");
            else {
                if (strlen(arg) < 6) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->t0_str = arg + 5;
            }
            matched = 1;
        }
        if (strcmp("--all-satellites", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--all-satellites", "include Starlink/OneWeb-style swarms (default: excluded)");
            else a->with_constellations = 1;
            matched = 1;
        }
        if (strncmp("--regex=", arg, 8) == 0 || help) {
            if (help) parse_help_line(OPTW, "--regex=<pattern>", "only satellites whose names match");
            else {
                if (strlen(arg) < 9) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->regex = arg + 8;
            }
            matched = 1;
        }
        if (strcmp("--ignore-case", arg) == 0 || help) {
            if (help) parse_help_line(OPTW, "--ignore-case", "case-insensitive regex match");
            else a->regex_ignore_case = 1;
            matched = 1;
        }
        if (strncmp("--lat=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lat=<deg>", "observer geodetic latitude (default RAO Priddis)");
            else {
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->site_latitude = atof(arg + 6);
            }
            matched = 1;
        }
        if (strncmp("--lon=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--lon=<deg>", "observer geodetic longitude, east positive (default RAO Priddis)");
            else {
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->site_longitude = atof(arg + 6);
            }
            matched = 1;
        }
        if (strncmp("--alt=", arg, 6) == 0 || help) {
            if (help) parse_help_line(OPTW, "--alt=<m>", "observer altitude above ellipsoid, metres (default RAO Priddis)");
            else {
                if (strlen(arg) < 7) {
                    fprintf(stderr, "Unable to parse %s\n", arg);
                    return PARSE_ERROR;
                }
                a->site_altitude = atof(arg + 6);
            }
            matched = 1;
        }

        if (!matched && !help) {
            // Original behaviour: only tokens starting with "--" are
            // rejected as options here. A lone "-" / non-dash token is
            // claimed by the positional block above, so the sole way to
            // land here is an unrecognized "--option".
            fprintf(stderr, "Unable to parse option '%s'\n", arg);
            return PARSE_ERROR;
        }
    }
    if (help >= HELP_FULL) {
        printf(
            "\n"
            "TLE discovery (named form):\n"
            "  1. newest /<satellite_name>/TLEs/<date>/tle-<date>.tle (by date)\n"
            "  2. else $HOME/.local/state/simple_sat_ops/active.tle (filtered by name)\n"
            "  3. else error. Override either form with --tle=<path>.\n"
            "\n"
            "EXAMPLES\n"
            "\n"
            "  # Next week of passes for one satellite (auto-discovers its TLE)\n"
            "  next_in_queue FrontierSat\n"
            "\n"
            "  # Next satellite pass (uses default TLE at ~/.local/state/simple_sat_ops/active.tle)\n"
            "  next_in_queue 0 2000\n"
            "\n"
            "  # All passes in the next 3 hours above 30 deg elevation\n"
            "  next_in_queue 0 2000 --list \\\n"
            "      --max-minutes=180 --min-elevation=30\n"
            "\n"
            "  # ISS-class only with a specific TLE file, with amateur-radio info annotation\n"
            "  next_in_queue 300 500 --list \\\n"
            "      --tle=TLEs/amateur.tle \\\n"
            "      --regex='ISS|ZARYA' --ignore-case --show-radio-info\n"
            "\n"
            "  # Passes for a specific satellite over the next week\n"
            "  next_in_queue 0 2000 'ISS (ZARYA)' --list\n"
            "\n"
            "  # Passes against a SSM-propagated trajectory (FrontierSat-style)\n"
            "  next_in_queue --trajectory-id=$(ssm trajectories | jq -r '.[0].id') --list\n"
            "\n"
            "NOTES\n"
            "  - The tool never opens the radio or rotator; safe to run on any host.\n"
            "  - `active_radios.txt` (looked for in the same directory as the TLE)\n"
            "    is a community-maintained CSV used by --show-radio-info;\n"
            "    missing satellites are simply omitted.\n"
            "  - `--trajectory-id` needs `ssm` on PATH. The trajectory's window\n"
            "    (from `ssm trajectory-meta <id>`) caps how far ahead passes can\n"
            "    be predicted; `--max-minutes` is clamped if it exceeds the window.\n");
    }
    return PARSE_OK;
}

void print_radio_info(const char *name, satellite_status_t *sat_info, int n_entries);

// Format a Julian Date (UTC) as Calgary local time. Relies on TZ having
// been set to "America/Edmonton" at startup so DST is handled.
static void format_local_aos(double jul_utc, char *buf, size_t bufsize)
{
    time_t t = (time_t)((jul_utc - 2440587.5) * 86400.0);
    struct tm lt;
    if (localtime_r(&t, &lt) == NULL) {
        snprintf(buf, bufsize, "          ?");
        return;
    }
    strftime(buf, bufsize, "%m-%d %H:%M", &lt);
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "next_in_queue")) return 0;
    // AOS-local rendering follows the operator's TZ (system default or
    // shell-set). Set TZ=America/Edmonton in the env if you want
    // Calgary time regardless of where you're ssh'd in from.
    tzset();

    state_t state = {0};
    niq_args_t cfg = {
        .site_latitude = RAO_LATITUDE,
        .site_longitude = RAO_LONGITUDE,
        .site_altitude = RAO_ALTITUDE,
        .max_passes = -1,
        .min_minutes_away = 0.0,
        .max_minutes_away = 1440.0,
        .min_elevation = 0.0,
        .max_elevation = 90.0,
        .min_altitude_km = 0.0,
        .max_altitude_km = 100000.0,
    };
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }

    // Copy parsed config into the working locals the prediction body uses.
    double site_latitude = cfg.site_latitude;
    double site_longitude = cfg.site_longitude;
    double site_altitude = cfg.site_altitude;
    int list_all = cfg.list_all;
    int max_passes = cfg.max_passes;
    int show_radio_info = cfg.show_radio_info;
    double min_minutes_away = cfg.min_minutes_away;
    double max_minutes_away = cfg.max_minutes_away;
    int max_minutes_user_set = cfg.max_minutes_user_set;
    double min_elevation = cfg.min_elevation;
    double max_elevation = cfg.max_elevation;
    const char *regex = cfg.regex;
    int regex_ignore_case = cfg.regex_ignore_case;
    int with_constellations = cfg.with_constellations;
    int reverse = cfg.reverse;
    int tle_explicit = cfg.tle_explicit;
    const char *trajectory_id = cfg.trajectory_id;
    const char *t0_str = cfg.t0_str;
    double min_altitude_km = cfg.min_altitude_km;
    double max_altitude_km = cfg.max_altitude_km;
    int minutes_offset = cfg.minutes_offset;
    // --tle / --tle= wrote here in the old loop; carry it across so the
    // catalog-scan / explicit-TLE branches see it (the named form below
    // overwrites it when no --tle was given).
    state.track.prediction.tles_filename = cfg.tles_filename;

    if (trajectory_id != NULL && tle_explicit) {
        fprintf(stderr, "--tle= and --trajectory-id= are mutually exclusive\n");
        return EXIT_FAILURE;
    }

    // Interpret the polymorphic positionals collected by parse_args. Numeric
    // values like '0' / '2000' and string names like 'ISS (ZARYA)' both land
    // in cfg.positionals; their meaning depends on count and on whether
    // --tle / --trajectory-id is present. (The --tle space-form value was
    // consumed inside parse_args, so it never reached this collector.)
    const char **positionals = cfg.positionals;
    int n_positional = cfg.n_positionals;
    const char *satellite_name = NULL;

    if (trajectory_id != NULL) {
        // OEM path: no positionals (trajectory is implicit).
        if (n_positional != 0) {
            fprintf(stderr,
                    "--trajectory-id= does not accept positional arguments; "
                    "use --min-altitude-km= / --max-altitude-km= for altitude filtering.\n");
            return EXIT_FAILURE;
        }
    } else if (tle_explicit) {
        // Explicit --tle=: 0 or 1 positional (optional <satellite_name>).
        if (n_positional > 1) {
            fprintf(stderr,
                    "with --tle=, pass at most one positional (<satellite_name>). "
                    "Use --min-altitude-km= / --max-altitude-km= for altitude filtering.\n");
            return EXIT_FAILURE;
        }
        if (n_positional == 1) satellite_name = positionals[0];
    } else if (n_positional == 1) {
        // Named form: a single positional is the satellite name. We know
        // the object, so no altitude limits are needed; auto-discover its
        // TLE from /<satname>/TLEs, then the default active.tle.
        satellite_name = positionals[0];
        static char named_tle[1024];
        if (resolve_named_tle(satellite_name, named_tle, sizeof named_tle) != 0) {
            fprintf(stderr,
                "next_in_queue: no TLE for '%s'. Drop a tle-YYYYMMDD.tle "
                "under /%s/TLEs/<date>/, stage one at the default "
                "active.tle, or pass --tle=<path>.\n",
                satellite_name, satellite_name);
            return EXIT_FAILURE;
        }
        state.track.prediction.tles_filename = tle_path_resolve(named_tle);
    } else if (n_positional == 2 || n_positional == 3) {
        // Catalog-scan form: <min_alt_km> <max_alt_km> [<satellite_name>].
        min_altitude_km = atof(positionals[0]);
        max_altitude_km = atof(positionals[1]);
        if (n_positional == 3) satellite_name = positionals[2];
    } else {
        // 0 positionals (and no --tle / --trajectory-id) is ambiguous.
        print_synopsis(stderr, argv[0]);
        return EXIT_FAILURE;
    }
    if (satellite_name != NULL && !max_minutes_user_set) {
        max_minutes_away = 1440 * 7;  // one week when filtering to a specific sat
    }
    if (trajectory_id != NULL && !max_minutes_user_set) {
        max_minutes_away = 1440 * 14;  // two weeks for trajectory planning
    }

    if (trajectory_id == NULL && state.track.prediction.tles_filename == NULL) {
        static char default_tle[1024];
        if (tle_default_path(default_tle, sizeof(default_tle)) != 0) {
            fprintf(stderr, "HOME unset or path too long; pass --tle=<path>\n");
            return EXIT_FAILURE;
        }
        state.track.prediction.tles_filename = tle_path_resolve(default_tle);
    }

    // Load OEM if --trajectory-id was given. Table lives on the stack and
    // is attached to prediction; freed at exit.
    oem_table_t oem = {0};
    if (trajectory_id != NULL) {
        if (oem_load_from_ssm(trajectory_id, &oem) != 0) {
            return EXIT_FAILURE;
        }
        state.track.prediction.oem = &oem;
        state.track.prediction.satellite_ephem.name = oem.object_name[0]
                                              ? oem.object_name : (char *)"UNKNOWN";
        double window_min = (oem.stop_jul_utc - oem.start_jul_utc) * 1440.0;
        if (max_minutes_away > window_min) {
            // Beyond the propagated window we fall back to two-body
            // Kepler extrapolation inside oem_sample_at(). Good enough
            // for ground-segment scheduling; not for conjunction work.
            fprintf(stderr,
                    "note: --max-minutes=%.0f exceeds OEM window (%.1f min / %.2f h); "
                    "passes beyond the window use two-body Kepler extrapolation "
                    "(no J2, no drag — drift of minutes/day for LEO).\n",
                    max_minutes_away, window_min, window_min / 60.0);
        }
    }

    /* Set up observer location */
    state.track.prediction.observer_ephem.position_geodetic.lat = site_latitude * M_PI / 180.0;
    state.track.prediction.observer_ephem.position_geodetic.lon = site_longitude * M_PI / 180.0;
    state.track.prediction.observer_ephem.position_geodetic.alt = site_altitude / 1000.0;

    double jul_utc;
    if (t0_str != NULL) {
        int y = 0, mo = 0, d = 0, h = 0, mi = 0;
        double sec = 0.0;
        int n = sscanf(t0_str, "%d-%d-%dT%d:%d:%lf", &y, &mo, &d, &h, &mi, &sec);
        if (n != 6) {
            fprintf(stderr, "--t0: expected yyyy-mm-ddThh:mm:ss, got '%s'\n", t0_str);
            return EXIT_FAILURE;
        }
        int a = (14 - mo) / 12;
        int yy = y + 4800 - a;
        int mm = mo + 12 * a - 3;
        long jdn = (long)d + (153L * mm + 2) / 5 + 365L * yy
                 + yy / 4 - yy / 100 + yy / 400 - 32045L;
        jul_utc = (double)jdn
                + ((double)h - 12.0) / 24.0
                + (double)mi / 1440.0
                + sec / 86400.0;
    } else {
        struct tm utc;
        struct timeval tv;
        UTC_Calendar_Now(&utc, &tv);
        jul_utc = Julian_Date(&utc, &tv);
    }
    jul_utc += minutes_offset / 1440.0;
    struct tm utc_ref;
    Date_Time(jul_utc, &utc_ref);

    if (trajectory_id != NULL) {
        printf("Checking SSM trajectory %s (%s) for upcoming ",
               trajectory_id,
               oem.object_name[0] ? oem.object_name : "UNKNOWN");
    } else {
        printf("Checking %s for upcoming ", state.track.prediction.tles_filename);
    }
    if (satellite_name != NULL) {
        printf("%s", satellite_name);
    } else {
        printf("satellite");
    }
    printf(" passes within ");
    if (max_minutes_away < 120) {
        printf("%.0f minutes", max_minutes_away);
    } else if (max_minutes_away < 2880) {
        printf("%.1f hours", max_minutes_away / 60.0);
    } else {
        printf("%.1f days", max_minutes_away / 1440.0);
    }
    printf(" from %04d-%02d-%02d %02d:%02d:%02d UTC\n", utc_ref.tm_year + 1900, utc_ref.tm_mon + 1, utc_ref.tm_mday, utc_ref.tm_hour, utc_ref.tm_min, utc_ref.tm_sec);
    criteria_t criteria = {
        .min_altitude_km = min_altitude_km,
        .max_altitude_km = max_altitude_km,
        .min_minutes = min_minutes_away,
        .max_minutes = max_minutes_away,
        .min_elevation = min_elevation,
        .max_elevation = max_elevation,
        .regex = (char *)regex,
        .regex_ignore_case = regex_ignore_case,
        .with_constellations = with_constellations,
        .name_prefix = satellite_name,
    };

    int count = 0;
    int number_checked = 0;
    // --list says "show every matching pass" — without it, find_passes
    // breaks after the first pass per satellite. Otherwise a single-sat
    // TLE plus --list returns just one pass even though we have 7 days
    // of orbits to find passes in.
    int find_all = (list_all || satellite_name != NULL || trajectory_id != NULL) ? 1 : 0;
    (void) find_passes(&state.track.prediction, jul_utc, 1.0, &criteria, &count, &number_checked, reverse, find_all);
    const size_t n_passes = number_of_passes();

    // Satellite radio-info annotation. Only loaded if the user asked for
    // it (--show-radio-info) and we have a TLE directory to derive the
    // file path from. Not applicable to the OEM / trajectory path.
    satellite_status_t *sat_info = NULL;
    int n_entries = 0;
    if (show_radio_info && state.track.prediction.tles_filename != NULL) {
        char radios_file[FILENAME_MAX];
        const char *slash = strrchr(state.track.prediction.tles_filename, '/');
        if (slash != NULL) {
            int dir_len = (int)(slash - state.track.prediction.tles_filename);
            snprintf(radios_file, sizeof(radios_file),
                     "%.*s/active_radios.txt", dir_len, state.track.prediction.tles_filename);
        } else {
            snprintf(radios_file, sizeof(radios_file), "active_radios.txt");
        }
        (void) parse_satellite_status_file(radios_file, &sat_info, &n_entries);
    }


    if (n_passes > 0) {
        const pass_t *p = NULL;
        // Default (-1) shows all; explicit user value is clamped down to
        // n_passes so the print loop's get_pass(i) never returns NULL.
        if (max_passes < 0 || (size_t)max_passes > n_passes) {
            max_passes = (int)n_passes;
        }
        if (list_all || satellite_name != NULL) {
            if (!reverse) {
                if (satellite_name == NULL) {
                    printf("Found %lu upcoming passes from a total of %d satellites\n", n_passes, count);
                }
                printf("%26s  %11s %9s %9s %9s %9s %9s %10s %10s %10s %25s %9s\n", "Name", "AOS local", "in", "dur (min)", "alt (km)", "azi (deg)", "ele (deg)", "up (MHz)", "down (MHz)", "bcn (MHz)", "mode", "status");
            }
            for (int i = 0; i < max_passes; i++) {
                p = get_pass(i);
                char aos_local[32];
                format_local_aos(p->ascension_jul_utc, aos_local, sizeof(aos_local));
                char t_until[16];
                if (p->minutes_away < 120.0) {
                    snprintf(t_until, sizeof t_until, "%.1f m", p->minutes_away);
                } else if (p->minutes_away < 2880.0) {
                    snprintf(t_until, sizeof t_until, "%.1f h", p->minutes_away / 60.0);
                } else {
                    snprintf(t_until, sizeof t_until, "%.1f d", p->minutes_away / 1440.0);
                }
                printf("%26s  %11s %9s %9.1f %9.1f %9.1f %9.1f",
                       p->name, aos_local, t_until,
                       p->pass_duration, p->max_altitude,
                       p->ascension_azimuth, p->max_elevation);
                if (satellite_name == NULL && show_radio_info == 1) {
                    print_radio_info(p->name, sat_info, n_entries);
                }
                printf("\n");
            }
            if (reverse) {
                printf("%26s  %11s %9s %9s %9s %9s %9s %10s %10s %10s %25s %9s\n", "Name", "AOS local", "in", "dur (min)", "alt (km)", "azi (deg)", "ele (deg)", "up (MHz)", "down (MHz)", "bcn (MHz)", "mode", "status");
                if (satellite_name != NULL) {
                    printf("Found %lu upcoming passes for %s\n", n_passes, satellite_name);
                } else {
                    printf("Found %lu upcoming passes from a total of %d satellites\n", n_passes, count);
                }
            }
        } else {
            printf("Searched %d satellites\n", count);
            p = get_pass(0);
            printf("%26s in %.1f minutes from %04d-%02d-%02d %02d:%02d:%02d UTC at azimuth %.1f deg. for %.1f minutes reaching elevation %.1f deg.\n", p->name, p->minutes_away, utc_ref.tm_year + 1900, utc_ref.tm_mon + 1, utc_ref.tm_mday, utc_ref.tm_hour, utc_ref.tm_min, utc_ref.tm_sec, p->ascension_azimuth, p->pass_duration, p->max_elevation);
        }
    } else {
        printf("No passes found\n");
    }

    free_passes();
    oem_free(&oem);

    return 0;
}

void print_radio_info(const char *name, satellite_status_t *sat_info, int n_entries)
{
    for (int s = 0; s < n_entries; ++s) {
        if (strcmp(name, sat_info[s].name) == 0) {
            if (strlen(sat_info[s].f_uplink_mhz) > 0) {
                printf(" %10s", sat_info[s].f_uplink_mhz);
            } else {
                printf(" %10s", "-");
            }
            if (strlen(sat_info[s].f_downlink_mhz) > 0) {
                printf(" %10s", sat_info[s].f_downlink_mhz);
            } else {
                printf(" %10s", "-");
            }
            if (strlen(sat_info[s].f_beacon_mhz) > 0) {
                printf(" %10s", sat_info[s].f_beacon_mhz);
            } else {
                printf(" %10s", "-");
            }
            if (strlen(sat_info[s].mode) > 0) {
                printf(" %25s", sat_info[s].mode);
            } else {
                printf(" %25s", "-");
            }
            if (strlen(sat_info[s].status) > 0) {
                printf(" %9s", sat_info[s].status);
            } else {
                printf(" %9s", "-");
            }
            break;
        }
    }
}
