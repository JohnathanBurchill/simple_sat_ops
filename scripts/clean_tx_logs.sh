#!/usr/bin/env bash
# scripts/clean_tx_logs.sh — scrub the misleading "ack" lines out of
# existing tx.log files (it edits the logs in place; it does NOT delete
# them).
#
# Why: older operator passes wrote a "tx-ack" line for every command,
# carrying tx_st:"ok". Those acks were synthesised locally the instant
# the burst left the radio -- they are NOT acknowledgements from the
# satellite, so they read as if the spacecraft replied when it never
# did. The transmit itself is already recorded by the paired
# "tx-command-sent" line, so the ok-ack line is redundant and
# misleading. The current code no longer writes acks at all; instead a
# command that was NOT put on the air gets a "tx-not-sent" line carrying
# the reason. This script brings old logs into that newer shape:
#
#   * tx-ack with tx_st:"ok"      -> removed (the tx-command-sent line
#                                    remains as the record of the send).
#   * tx-ack with any other tx_st -> rewritten as tx-not-sent, keeping
#                                    the reason and the command text
#                                    (i.e. the note that follows the
#                                    command that was not sent).
#   * tx-command-sent / tx-preview / anything else -> left untouched.
#
# Safe by default: with no flags it only REPORTS what it would change in
# each file (a dry run). Pass -f / --force to rewrite the files in
# place. Pass a directory to override the search root.
#
# Usage:
#   scripts/clean_tx_logs.sh                 # dry run: report changes
#   scripts/clean_tx_logs.sh -f              # rewrite the logs in place
#   scripts/clean_tx_logs.sh -f /some/dir    # operate under a chosen root
#
# Root resolution (same order as the C code): $FRONTIERSAT_ROOT, else
# /FrontierSat, else $HOME/FrontierSat. The Operations subtree under that
# root is searched recursively for files named exactly "tx.log".
#
# Exit status: 0 on success (including "nothing to do"); non-zero on a
# bad argument or an unreadable root.
#
# Copyright (C) 2026  Johnathan K Burchill — GPLv3 or later.

set -euo pipefail

force=0
root=""

while [ $# -gt 0 ]; do
    case "$1" in
        -f|--force) force=1 ;;
        -h|--help)
            sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        -*)
            echo "clean_tx_logs: unknown option '$1'" >&2
            exit 2 ;;
        *)
            if [ -n "$root" ]; then
                echo "clean_tx_logs: only one root directory may be given" >&2
                exit 2
            fi
            root="$1" ;;
    esac
    shift
done

# Resolve the search root.
if [ -z "$root" ]; then
    if [ -n "${FRONTIERSAT_ROOT:-}" ]; then
        root="$FRONTIERSAT_ROOT/Operations"
    elif [ -d /FrontierSat ]; then
        root="/FrontierSat/Operations"
    else
        root="$HOME/FrontierSat/Operations"
    fi
fi

if [ ! -d "$root" ]; then
    echo "clean_tx_logs: no such directory: $root" >&2
    exit 1
fi

# Collect tx.log files (NUL-delimited so paths with spaces survive).
mapfile -d '' -t files < <(find "$root" -type f -name tx.log -print0)

if [ "${#files[@]}" -eq 0 ]; then
    echo "clean_tx_logs: no tx.log files under $root"
    exit 0
fi

# The per-line transform, shared by the dry-run count and the in-place
# rewrite. When `out` is set, transformed lines are written there;
# either way the final line on stdout is "<removed> <converted>".
read_and_count() {
    # $1 = input file, $2 = output file ("" -> count only, don't write)
    awk -v out="$2" '
        # Successful local ack: redundant now -> drop it.
        /"t":"tx-ack"/ && /"tx_st":"ok"/ { removed++; next }
        # Any other ack carried a reason the command was not sent ->
        # rewrite to the current tx-not-sent shape, keep the reason.
        /"t":"tx-ack"/ {
            sub(/"t":"tx-ack"/, "\"t\":\"tx-not-sent\"")
            converted++
            if (out != "") print > out
            next
        }
        { if (out != "") print > out }
        END { printf "%d %d\n", removed + 0, converted + 0 }
    ' "$1"
}

total_removed=0
total_converted=0
changed_files=0

for f in "${files[@]}"; do
    if [ "$force" -eq 0 ]; then
        counts=$(read_and_count "$f" "")
        removed=${counts% *}
        converted=${counts#* }
        if [ "$removed" -ne 0 ] || [ "$converted" -ne 0 ]; then
            echo "$f: would remove $removed ack line(s), convert $converted to tx-not-sent"
            changed_files=$((changed_files + 1))
        fi
        total_removed=$((total_removed + removed))
        total_converted=$((total_converted + converted))
    else
        tmp="$f.clean.$$"
        : > "$tmp"
        counts=$(read_and_count "$f" "$tmp")
        removed=${counts% *}
        converted=${counts#* }
        if [ "$removed" -ne 0 ] || [ "$converted" -ne 0 ]; then
            # Swap in the cleaned copy (same directory, so mv is atomic).
            mv -f -- "$tmp" "$f"
            echo "$f: removed $removed ack line(s), converted $converted to tx-not-sent"
            changed_files=$((changed_files + 1))
            total_removed=$((total_removed + removed))
            total_converted=$((total_converted + converted))
        else
            rm -f -- "$tmp"
        fi
    fi
done

if [ "$changed_files" -eq 0 ]; then
    echo "clean_tx_logs: nothing to change in ${#files[@]} tx.log file(s) under $root"
    exit 0
fi

if [ "$force" -eq 0 ]; then
    echo "clean_tx_logs: $changed_files file(s) would change " \
         "($total_removed ack line(s) removed, $total_converted converted)."
    echo "Re-run with -f / --force to rewrite them in place."
else
    echo "clean_tx_logs: rewrote $changed_files file(s) " \
         "($total_removed ack line(s) removed, $total_converted converted)."
fi
