#!/usr/bin/env bash
b=$(basename $1 .mov)
ffmpeg -i $1 -filter:v scale=640:-1 -pix_fmt rgb8 -r 10 tmp.gif
#magick tmp.gif -verbose -coalesce -layers OptimizeFrame tmp1.gif
gifsicle -O2 tmp.gif -o $b.gif
rm tmp.gif

