set terminal pngcairo size 1000,500 enhanced font 'Helvetica,11'
set output 'fig_matched_filter.png'

set grid
set title "Boxcar matched filter (length sps=10) recovers the ±π/20 rad/sample symbol estimates at SNR 5 dB" font ',12'
set xlabel "time (ms)"
set ylabel "Δφ (rad/sample)"
set xrange [0:32.0/9.6]
set yrange [-1.2:1.2]
set ytics (-1, "-π/20" -0.157, 0, "+π/20" 0.157, 1)

# Decision sample markers: every SPS samples → every 1/9.6 ms
# We'll mark x = 0.5/9.6, 1.5/9.6, ... mid-symbol; eye-center is at integer-symbol boundary.

set arrow from 0,-0.157 to 32.0/9.6,-0.157 nohead lc rgb '#888888' dt 3 lw 1
set arrow from 0,+0.157 to 32.0/9.6,+0.157 nohead lc rgb '#888888' dt 3 lw 1

plot 'matched_filter.dat' using 1:2 with lines lw 1.0 lc rgb '#c08080' title 'discriminator Δφ (jagged)', \
     'matched_filter.dat' using 1:3 with lines lw 2.4 lc rgb '#202060' title 'after matched filter (clean ±π/20)'
