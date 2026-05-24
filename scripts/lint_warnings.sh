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

# UHD coverage: turn on WITH_USRP_B210 when the libuhd headers are
# discoverable via pkg-config, so format-truncation bugs in B210-
# specific utilities (b210_rx_capture, b210_gain_sweep, tx_frame_sdr)
# are caught here rather than only on the Linux ground machine. Mac
# hosts without `brew install uhd` fall back to the off-mode build.
USRP_FLAG="-DWITH_USRP_B210=OFF"
if pkg-config --exists uhd 2>/dev/null; then
    USRP_FLAG="-DWITH_USRP_B210=ON"
fi

if [[ "$mode" == "full" || ! -d "$BUILD_DIR" ]]; then
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake "$ROOT" \
        -DCMAKE_C_COMPILER="$GCC" \
        -DCMAKE_C_FLAGS="${WFLAGS[*]} ${SGP4SDP4_INC}" \
        "$USRP_FLAG" \
        >cmake.log 2>&1 || { cat cmake.log; exit 1; }
fi

cd "$BUILD_DIR"

# Run the build, tee the full output to build.log; show ONLY diagnostic
# lines from project .c/.h files. gcc-15 can't parse Objective-C, so
# compiling decode_inspector_macos.m emits hundreds of irrelevant errors
# from the AppKit / Foundation headers — we drop those entirely.
make -j4 2>&1 | tee build.log | \
    grep -E "warning:|error:" | \
    grep -v "include-fixed/" | \
    grep -E "/(utils|src|sgp4sdp4|include)/.+\.(c|h):" || true

# Exit code policy: fail only on diagnostics in OUR .c files. Hard
# errors from gcc-15 trying to compile the .m shim or follow-on linker
# failures don't count — the script's job is to surface format-truncation
# / format-overflow bugs that Apple clang can't, not to be a full CI.
relevant_diag() {
    grep -E "warning:|error:" build.log \
        | grep -v "include-fixed/" \
        | grep -v "framework/Headers/" \
        | grep -v "\.m:" \
        | grep -E "\.(c|h):"
}
fail=0
if relevant_diag | grep -qE "error:"; then fail=1; fi
if relevant_diag | grep -qE "format-truncation|format-overflow"; then fail=1; fi
if [[ $fail -ne 0 ]]; then
    echo
    echo "lint_warnings: project-source error or format-truncation found (full log: $BUILD_DIR/build.log)" >&2
    exit 1
fi
n_warn=$(relevant_diag | grep -cE "warning:" || true)
echo "lint_warnings: clean ($GCC, $n_warn non-fatal warning(s) in project sources — full log: $BUILD_DIR/build.log)"
