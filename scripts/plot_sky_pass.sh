#!/usr/bin/env bash
# plot_sky_pass.sh — gnuplot polar sky plot of decoded-packet az/el
# from the AX100 packet DB. Sibling of plot_sky_pass.py; same data
# source, fewer dependencies (just gnuplot — no Python / matplotlib).
#
# Reads packet_query --format=csv, peels out (type, az, el, ts) into
# a temp file, hands it to gnuplot in polar mode with `set theta top
# clockwise` so north is up and east is on the right (the conventional
# ground-station sky view). Renders to PNG by default; pass
# --terminal qt to open an interactive window instead.
#
# Usage:
#   plot_sky_pass.sh [--db PATH] [--since 24h] [--until ...]
#                    [--type beacon|tcmd_response|log|bulk_file]
#                    [--satellite CTS1] [--source-tool ...]
#                    [--out sky_pass.png]
#                    [--terminal pngcairo|qt|svg]
#                    [--connect]
#                    [--packet-query PATH]
#
# Rows with NULL az_deg or el_deg are dropped silently — those are
# typically packets decoded by audio-path receivers without SGP4. Run
# rx_replay --update with the pass's TLE to backfill.

set -euo pipefail

usage() {
    sed -n '2,/^$/{ s/^# \{0,1\}//; /^[A-Z]/q; p; }' "$0" >&2
    exit 1
}

DB=""
SINCE=""
UNTIL=""
TYPE=""
SATELLITE=""
SOURCE_TOOL=""
LIKE=""
LIMIT=10000
OUT="sky_pass.png"
TERMINAL="pngcairo"
CONNECT=0
PACKET_QUERY=""
TITLE="Decoded-packet sky positions (RAO frame)"

# Tolerant arg parser: accepts both `--flag value` and `--flag=value`.
take() {
    if [ "${1#--*=}" != "$1" ]; then
        printf '%s' "${1#*=}"
    else
        printf '%s' "$2"
    fi
}
shift_for() {
    if [ "${1#--*=}" != "$1" ]; then echo 1
    else echo 2
    fi
}

while [ $# -gt 0 ]; do
    case "$1" in
        --db|--db=*)              DB=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --since|--since=*)        SINCE=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --until|--until=*)        UNTIL=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --type|--type=*)          TYPE=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --satellite|--satellite=*) SATELLITE=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --source-tool|--source-tool=*) SOURCE_TOOL=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --like|--like=*)          LIKE=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --limit|--limit=*)        LIMIT=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --out|--out=*)            OUT=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --terminal|--terminal=*)  TERMINAL=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --packet-query|--packet-query=*) PACKET_QUERY=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --title|--title=*)        TITLE=$(take "$1" "${2:-}"); shift "$(shift_for "$1")" ;;
        --connect)                CONNECT=1; shift ;;
        -h|--help)                usage ;;
        *)                        echo "unknown option: $1" >&2; usage ;;
    esac
done

# Locate packet_query.
if [ -z "$PACKET_QUERY" ]; then
    here="$(cd "$(dirname "$0")" && pwd)"
    if [ -x "$here/../build/packet_query" ]; then
        PACKET_QUERY="$here/../build/packet_query"
    elif command -v packet_query >/dev/null 2>&1; then
        PACKET_QUERY="$(command -v packet_query)"
    else
        echo "plot_sky_pass.sh: cannot find packet_query — pass --packet-query=<path>" >&2
        exit 1
    fi
fi
command -v gnuplot >/dev/null || {
    echo "plot_sky_pass.sh: gnuplot not on PATH (apt: gnuplot, brew: gnuplot)" >&2
    exit 1
}

# packet_query argv.
PQ_ARGS=(--format=csv --limit="$LIMIT")
[ -n "$DB" ]          && PQ_ARGS+=("--db=$DB")
[ -n "$SINCE" ]       && PQ_ARGS+=("--since=$SINCE")
[ -n "$UNTIL" ]       && PQ_ARGS+=("--until=$UNTIL")
[ -n "$TYPE" ]        && PQ_ARGS+=("--type=$TYPE")
[ -n "$SATELLITE" ]   && PQ_ARGS+=("--satellite=$SATELLITE")
[ -n "$SOURCE_TOOL" ] && PQ_ARGS+=("--source-tool=$SOURCE_TOOL")
[ -n "$LIKE" ]        && PQ_ARGS+=("--like=$LIKE")

TMPDATA="$(mktemp -t sky_pass_XXXXXX)"
TMPGP="$(mktemp -t sky_pass_XXXXXX).gp"
trap 'rm -f "$TMPDATA" "$TMPGP"' EXIT

# Extract type / az / el / ts. Drop rows with missing az or el. Sort
# by ts so a --connect line draws a sensible arc through the pass.
"$PACKET_QUERY" "${PQ_ARGS[@]}" | awk -F',' '
    BEGIN { OFS = "\t" }
    NR == 1 {
        for (i = 1; i <= NF; i++) col[$i] = i
        next
    }
    {
        ty = $col["packet_type_name"]
        az = $col["az_deg"]
        el = $col["el_deg"]
        ts = $col["ts_received"]
        if (az == "" || el == "") next
        print ty, az, el, ts
    }' | sort -k4 > "$TMPDATA"

POINT_COUNT=$(wc -l < "$TMPDATA" | tr -d ' ')
if [ "$POINT_COUNT" = "0" ]; then
    echo "plot_sky_pass.sh: no rows with az/el in this query (run rx_replay --update on the WAVs to backfill)" >&2
    exit 1
fi

if [ "$CONNECT" = "1" ]; then
    STYLE="with linespoints pt 7 ps 0.7 lw 1"
else
    STYLE="with points pt 7 ps 0.7"
fi

# Escape single quotes in TITLE for the gnuplot string literal.
SAFE_TITLE=$(printf '%s' "$TITLE" | sed "s/'/''/g")

cat > "$TMPGP" <<GP
set terminal $TERMINAL size 900,900
GP
case "$TERMINAL" in
    pngcairo|png|svg|pdf|pdfcairo)
        printf "set output '%s'\n" "$OUT" >> "$TMPGP"
        ;;
esac

cat >> "$TMPGP" <<GP
set polar
set angles degrees
# North up, east clockwise (gnuplot 5.4+). Older gnuplot just gets the
# default east-right ccw layout — still polar, still readable.
set theta top clockwise
set size square
set grid polar 30
unset border
unset xtics
unset ytics
set rrange [0:90]
set rtics 15
set rlabel "el (deg below horizon=outer ring)"
set key outside right top
set title '$SAFE_TITLE' noenhanced
plot \\
  '$TMPDATA' using (strcol(1) eq 'beacon'        ? \$2 : NaN):(90-\$3) \
      $STYLE lc rgb '#1f77b4' title 'beacon', \\
  '$TMPDATA' using (strcol(1) eq 'tcmd_response' ? \$2 : NaN):(90-\$3) \
      $STYLE lc rgb '#ff7f0e' title 'tcmd_response', \\
  '$TMPDATA' using (strcol(1) eq 'log'           ? \$2 : NaN):(90-\$3) \
      $STYLE lc rgb '#2ca02c' title 'log', \\
  '$TMPDATA' using (strcol(1) eq 'bulk_file'     ? \$2 : NaN):(90-\$3) \
      $STYLE lc rgb '#9467bd' title 'bulk_file'
GP

gnuplot "$TMPGP"

case "$TERMINAL" in
    pngcairo|png|svg|pdf|pdfcairo)
        echo "plot_sky_pass.sh: wrote $OUT ($POINT_COUNT points)" >&2
        ;;
esac
