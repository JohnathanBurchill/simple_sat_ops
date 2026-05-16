#!/bin/bash
# Build and run the figure-data generator, then render every .gp into a PNG.
# Outputs land alongside this script. Re-run any time the underlying C code
# (csp.c / ax100.c / golay24.c / rs.c) changes — the figures use those
# encoders to produce the actual on-wire bytes.
set -euo pipefail
cd "$(dirname "$0")"

# Locate OpenSSL headers/libs (macOS / Homebrew default).
OPENSSL_PREFIX="${OPENSSL_PREFIX:-/opt/homebrew/opt/openssl@3}"
if [ ! -d "$OPENSSL_PREFIX/include/openssl" ]; then
    OPENSSL_PREFIX="${OPENSSL_PREFIX:-/usr/local/opt/openssl@3}"
fi

echo "==> compiling gen_figdata"
cc -O2 -Wall -I../.. -I"$OPENSSL_PREFIX/include" \
    gen_figdata.c ../../csp.c ../../ax100.c ../../golay24.c ../../rs.c \
    -L"$OPENSSL_PREFIX/lib" -lcrypto -lm -o gen_figdata

echo "==> generating .dat files"
./gen_figdata

echo "==> rendering figures"
for gp in fig_*.gp; do
    echo "    $gp"
    gnuplot "$gp"
done

echo "done. PNGs:"
ls -la fig_*.png
