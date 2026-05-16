set terminal pngcairo size 1000,500 enhanced font 'Helvetica,11'
set output 'fig_asm_correlation.png'

set grid
set title "Sliding 32-bit window: Hamming distance to ASM = 0x930B51DE" font ',12'
set xlabel "bit offset into the wire stream"
set ylabel "Hamming distance (bits)"
set yrange [-1:32]
set ytics (0, 4, 8, 16, 24, 32)
set xrange [0:*]

set arrow from 32,32 to 32,0 nohead lc rgb '#cc6060' dt 2 lw 1.5
set label "exact match\n(after 32-bit preamble)" at 38,30 left tc rgb '#cc2020' font ',10'

set arrow from 0,4 to 463,4 nohead lc rgb '#888888' dt 3 lw 1
set label "sync\\_max\\_ham = 4 (accept threshold)" at 230,7 center tc rgb '#444444' font ',9'

plot 'asm_correlation.dat' using 1:2 with lines lw 1.2 lc rgb '#2060a0' notitle, \
     '' using 1:2 with points pt 7 ps 0.25 lc rgb '#2060a0' notitle
