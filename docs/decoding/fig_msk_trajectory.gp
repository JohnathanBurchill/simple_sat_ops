set terminal pngcairo size 1000,720 enhanced font 'Helvetica,11'
set output 'fig_msk_trajectory.png'

# MSK at h = 0.5, baud = 9600, dev = 2400 Hz, fs = 96 kHz (sps = 10)
# First 32 bits of the real wire stream:
#   preamble (4 bytes 0xAA = bits 10101010...)  → first 32 bits
# so the first symbol period is +deviation, then alternating ±, until
# the ASM at bit 32.

set multiplot layout 3,1 margins 0.08,0.97,0.10,0.94 spacing 0,0.04
set grid

set ylabel "bit value (NRZ)"
set yrange [-1.5:1.5]
set ytics ("-1" -1, "0" 0, "+1" 1)
set xrange [0:32.0/9.6]
unset xtics
set title "MSK on the wire — first 32 bits of an actual AX100 frame (preamble 0xAA × 4)" font ',12'
plot 'msk_trajectory.dat' using 1:2 with steps lw 2 lc rgb '#2060a0' notitle

set ylabel "f_{inst} (Hz)"
set yrange [-3200:3200]
set ytics (-2400, 0, 2400)
set title ""
plot 'msk_trajectory.dat' using 1:3 with steps lw 2 lc rgb '#208040' title 'instantaneous frequency = ±h·baud/2'

set ylabel "phase φ(t) (rad)"
set xlabel "time (ms)"
set xtics auto
set autoscale y
set ytics auto
plot 'msk_trajectory.dat' using 1:4 with lines lw 2 lc rgb '#a02020' title 'continuous-phase trajectory (slope = 2πf_{inst})'

unset multiplot
