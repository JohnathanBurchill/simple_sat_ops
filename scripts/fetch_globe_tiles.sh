#!/usr/bin/env bash
# fetch_globe_tiles.sh
#
# Fetch the detail map the raylib viewers' globe (utils/sat_globe.c) draws from
# when it is zoomed in: NASA's "Blue Marble: Land Surface, Shallow Water, and
# Shaded Topography" -- the same map as the bundled
# assets/nasa_blue_marble_2048.png, so the colours do not jump as the view
# closes in -- at 21600 x 10800, about 1.9 km to the pixel at the equator.
#
# The one 182 MB TIFF is downloaded to a temporary directory and cut into
# square 675-pixel JPEG tiles at four sizes, 21600, 10800, 5400 and 2700
# pixels across, each in a directory named for its width and numbered row by
# row from the top left (90 N 180 W). About 50 MB in all. The TIFF is deleted
# afterwards. Public domain (NASA imagery carries no copyright).
#
# Usage:
#   scripts/fetch_globe_tiles.sh [--out=<dir>] [--from=<land_shallow_topo_21600.tif>]
#
#   --out   where the tiles go. Default
#           ${XDG_DATA_HOME:-~/.local/share}/simple_sat_ops/globe_tiles,
#           which is where the viewers look. Replaces a set already there.
#   --from  cut a TIFF already on disk instead of downloading it.
#
# Needs curl and ImageMagick (magick, or convert for version 6). The tiles are
# read through raylib's JPEG support, which frontiersat_camera_viewer already
# depends on for the pictures themselves.

set -euo pipefail

SRC_URL="https://eoimages.gsfc.nasa.gov/images/imagerecords/57000/57752/land_shallow_topo_21600.tif"
TILE=675
TOP_W=21600

OUT="${XDG_DATA_HOME:-$HOME/.local/share}/simple_sat_ops/globe_tiles"
FROM=""
for arg in "$@"; do
    case "$arg" in
        --out=*)  OUT="${arg#--out=}" ;;
        --from=*) FROM="${arg#--from=}" ;;
        -h|--help) sed -n '2,/^$/s/^# \{0,1\}//p' "$0"; exit 0 ;;
        *) echo "fetch_globe_tiles: unknown option '$arg' (try --help)" >&2; exit 2 ;;
    esac
done

if command -v magick >/dev/null 2>&1; then
    IM=(magick)
elif command -v convert >/dev/null 2>&1; then
    IM=(convert)
else
    echo "fetch_globe_tiles: needs ImageMagick (brew install imagemagick, or apt install imagemagick)" >&2
    exit 2
fi

WORK="$(mktemp -d "${TMPDIR:-/tmp}/globe_tiles.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

if [[ -z "$FROM" ]]; then
    echo "fetch_globe_tiles: downloading $SRC_URL (182 MB)"
    curl -fL --progress-bar -o "$WORK/src.tif" "$SRC_URL"
    FROM="$WORK/src.tif"
fi

# The tiles are made beside the destination and swapped in at the end, so a
# run that stops part way leaves the old set, or none, rather than half of one.
mkdir -p "$(dirname "$OUT")"
STAGE="$OUT.new"
rm -rf "$STAGE"
mkdir -p "$STAGE"

w=$TOP_W
while (( w >= TOP_W / 8 )); do
    h=$(( w / 2 ))
    echo "fetch_globe_tiles: cutting the ${w} x ${h} level"
    mkdir -p "$STAGE/$w"
    "${IM[@]}" "$FROM" -strip -resize "${w}x${h}!" \
        -crop "${TILE}x${TILE}" +repage -quality 90 "$STAGE/$w/%d.jpg"
    want=$(( (w / TILE) * (h / TILE) ))
    got=$(find "$STAGE/$w" -name '*.jpg' | wc -l | tr -d ' ')
    if [[ "$got" != "$want" ]]; then
        echo "fetch_globe_tiles: the ${w} level came out as $got tiles, not $want" >&2
        exit 1
    fi
    w=$(( w / 2 ))
done

rm -rf "$OUT"
mv "$STAGE" "$OUT"
echo "fetch_globe_tiles: $(du -sh "$OUT" | awk '{print $1}') of tiles in $OUT"
