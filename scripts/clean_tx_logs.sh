#!/usr/bin/env bash
# scripts/clean_tx_logs.sh — remove previous tx.log files from pass folders.
#
# Why: every operator pass writes a tx.log (the JSON record of what was
# transmitted) into its pass folder under <root>/Operations/<date>/<time>/.
# These accumulate across passes; this script clears the old ones when you
# want a clean slate (e.g. before a fresh bring-up run).
#
# Safe by default: with no flags it only LISTS the tx.log files it would
# remove (a dry run). Pass -f / --force to actually delete them. Pass a
# directory to override the search root.
#
# Usage:
#   scripts/clean_tx_logs.sh                 # dry run: list tx.log files
#   scripts/clean_tx_logs.sh -f              # delete them
#   scripts/clean_tx_logs.sh -f /some/dir    # delete under a chosen root
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
            sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
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

n=${#files[@]}
if [ "$n" -eq 0 ]; then
    echo "clean_tx_logs: no tx.log files under $root"
    exit 0
fi

if [ "$force" -eq 0 ]; then
    echo "clean_tx_logs: would remove $n tx.log file(s) under $root:"
    printf '  %s\n' "${files[@]}"
    echo "Re-run with -f / --force to delete them."
    exit 0
fi

for f in "${files[@]}"; do
    rm -f -- "$f"
    echo "removed $f"
done
echo "clean_tx_logs: removed $n tx.log file(s)."
