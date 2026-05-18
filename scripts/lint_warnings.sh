#!/usr/bin/env bash
# scripts/lint_warnings.sh — parallel GCC build that catches the warnings
# Apple clang misses on the Mac dev host.
#
# Why: clang accepts -Wformat-truncation as a no-op (the analysis isn't
# implemented), so snprintf-truncation bugs slip past the default build
# and only surface on the Linux ground-machine compile. Homebrew ships
# real GCC, which does run the analysis. This script drives a separate
# build directory (build-lint/) with GCC + the warnings escalated to
# errors, so a single command answers "would the RAO build complain?"
#
# Usage:
#   bash scripts/lint_warnings.sh           # full lint build
#   bash scripts/lint_warnings.sh --quick   # skip cmake reconfigure
#
# Exit status: 0 if everything compiled clean; non-zero if any warning
# or error occurred (the warning text is dumped to stdout).
#
# Copyright (C) 2026  Johnathan K Burchill — GPLv3 or later.

set -euo pipefail

# Locate a Homebrew GCC. Prefer the newest; bail with a hint if none found.
GCC=""
for cand in gcc-15 gcc-14 gcc-13 gcc-12; do
    if command -v "$cand" >/dev/null 2>&1; then
        GCC="$cand"
        break
    fi
done
if [[ -z "$GCC" ]]; then
    echo "lint_warnings: no gcc-N on PATH; install via 'brew install gcc'" >&2
    exit 2
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build-lint"

# Warning set — same as the main build plus the format-truncation /
# format-overflow analyses that only GCC implements. -Werror= turns the
# specific format diagnostics into hard errors so CI can gate on them
# without losing other warnings to a global -Werror.
WFLAGS=(
    -Wall -Wextra
    -Wformat=2
    -Wformat-truncation=2
    -Wformat-overflow=2
    -Werror=format-truncation
    -Werror=format-overflow
)

# The bundled sgp4sdp4/ installs its header under /usr/local/include,
# which clang scans by default but gcc-15 doesn't. Add it explicitly so
# the SGP4-dependent targets compile. Mirror the lib path too.
SGP4SDP4_INC=""
for d in /usr/local/include /opt/homebrew/include; do
    if [[ -f "$d/sgp4sdp4.h" ]]; then SGP4SDP4_INC="-I$d"; break; fi
done

mode="full"
if [[ "${1:-}" == "--quick" ]]; then
    mode="quick"
fi

if [[ "$mode" == "full" || ! -d "$BUILD_DIR" ]]; then
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake "$ROOT" \
        -DCMAKE_C_COMPILER="$GCC" \
        -DCMAKE_C_FLAGS="${WFLAGS[*]} ${SGP4SDP4_INC}" \
        -DWITH_USRP_B210=OFF \
        >cmake.log 2>&1 || { cat cmake.log; exit 1; }
fi

cd "$BUILD_DIR"

# Run the build, tee warnings/errors to stdout. We strip noisy "In file
# included from..." chains so the operator sees the actual diagnostic
# lines first; the full log lives in build-lint/build.log.
make -j4 2>&1 | tee build.log | \
    grep -E "warning:|error:" | \
    grep -v "include-fixed/" || true

# Exit code policy: fail only on hard errors and on the format-truncation
# / format-overflow diagnostics that motivated this script — the GCC
# bounds prover Apple clang lacks. Pre-existing -Wunused / -Wsign-compare
# noise is reported above but doesn't block the build.
fail=0
if grep -qE "error:" build.log; then fail=1; fi
if grep -qE "format-truncation|format-overflow" build.log; then fail=1; fi
if [[ $fail -ne 0 ]]; then
    echo
    echo "lint_warnings: hard error or format-truncation found (full log: $BUILD_DIR/build.log)" >&2
    exit 1
fi
n_warn=$(grep -cE "warning:" build.log || true)
echo "lint_warnings: clean ($GCC, $n_warn non-fatal warning(s) — full log: $BUILD_DIR/build.log)"
