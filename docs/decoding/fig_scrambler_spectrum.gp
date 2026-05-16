set terminal pngcairo size 900,500 enhanced font 'Helvetica,11'
set output 'fig_scrambler_spectrum.png'

set grid
set title "Why scramble: PSD of a structured payload (alternating 0x00 / 0xFF) before and after CCSDS scrambler" font ',12'
set xlabel "normalized frequency (cycles/bit)"
set ylabel "PSD (dB, peak-normalized)"
set xrange [0:0.5]
set yrange [-50:5]
set key bottom right

set label "fundamental\n(period 16 bits)" at 0.0625,3 left tc rgb '#a02020' font ',9'
set arrow from 0.0625,2 to 0.0625,-2 lc rgb '#a02020' lw 1
set label "3rd harmonic" at 0.1875,-7 center tc rgb '#a02020' font ',9'
set label "5th" at 0.3125,-14 center tc rgb '#a02020' font ',9'

plot 'scrambler_spectrum.dat' using 1:2 with lines lw 2 lc rgb '#a02020' title 'unscrambled — square-wave harmonics at 1/16, 3/16, 5/16, ...', \
     '' using 1:3 with lines lw 2 lc rgb '#206020' title 'scrambled — flat (≈white) across the band'
