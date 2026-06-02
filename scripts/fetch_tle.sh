#!/usr/bin/env bash
# scripts/fetch_tle.sh — fetch the latest TLE for one catalogued object
# from CelesTrak and drop it into the FrontierSat TLE tree, dated by day.
#
# Meant to run once a day from cron. It writes:
#
#     <root>/TLEs/YYYYMMDD/tle-YYYYMMDD.tle
#
# where <root> is $FRONTIERSAT_ROOT if set, else /FrontierSat (the same
# resolution the simple_sat_ops tools use). The file is the standard
# 3-line TLE (name + line 1 + line 2) that simple_sat_ops / next_in_queue
# accept as argv[1].
#
# Cron, 9 am server-local each day (`crontab -e`):
#
#     0 9 * * * /path/to/fetch_tle.sh >> /var/log/sso/fetch_tle.log 2>&1
#
# cron runs with a bare environment, so FRONTIERSAT_ROOT is normally
# unset there and the /FrontierSat default applies — which is what the
# server wants. To target a different tree (a dev host), export
# FRONTIERSAT_ROOT in the crontab line or the script's environment.
#
# Knobs (environment overrides):
#   CATNR=<n>     NORAD catalog number to fetch (default 69015, FrontierSat)
#   USE_UTC=1     date the path/file in UTC instead of server-local time
#
# Safe to run more than once a day: it just refreshes that day's file.
# A failed or malformed download never overwrites an existing good file —
# it's downloaded to a temp file, validated as a TLE for the requested
# object, and only then moved into place.

set -euo pipefail

CATNR="${CATNR:-69015}"
ROOT="${FRONTIERSAT_ROOT:-/FrontierSat}"
URL="https://celestrak.org/NORAD/elements/gp.php?CATNR=${CATNR}&FORMAT=TLE"

if [ "${USE_UTC:-0}" = "1" ]; then
    DATE="$(date -u +%Y%m%d)"
else
    DATE="$(date +%Y%m%d)"
fi
DEST_DIR="${ROOT}/TLEs/${DATE}"
DEST="${DEST_DIR}/tle-${DATE}.tle"

log() { printf '%s fetch_tle: %s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$*"; }

# Make the day's directory up front so the temp file can live beside the
# destination — then publishing is a rename within one filesystem, which
# is atomic, so a reader never sees a half-written TLE.
mkdir -p "$DEST_DIR"
work="$(mktemp "${DEST_DIR}/.tle-${CATNR}.XXXXXX")"
trap 'rm -f "$work" "$work".lf' EXIT

log "fetching catalog ${CATNR} from CelesTrak"
if ! curl --fail --silent --show-error --location \
          --retry 3 --retry-delay 5 --max-time 60 \
          --user-agent "FrontierSat-ground-station/fetch_tle (catnr ${CATNR})" \
          -o "$work" "$URL"; then
    log "ERROR: download failed; left any existing ${DEST} untouched"
    exit 1
fi

# CelesTrak serves CRLF line endings; normalise to LF.
tr -d '\r' < "$work" > "$work".lf && mv -f "$work".lf "$work"

# Validate before publishing: the body must be the TLE for the object we
# asked for, not "No GP data found", an HTML error, or a different sat.
# TLE line 1 begins "1 <catnr>", line 2 begins "2 <catnr>".
if ! grep -q "^1 ${CATNR}" "$work" || ! grep -q "^2 ${CATNR}" "$work"; then
    log "ERROR: response is not a valid TLE for ${CATNR}; not publishing. Got:"
    head -n 3 "$work" | sed 's/^/    /'
    exit 1
fi

mv -f "$work" "$DEST"
chmod 0644 "$DEST"
log "wrote ${DEST}"
sed 's/^/    /' "$DEST"
