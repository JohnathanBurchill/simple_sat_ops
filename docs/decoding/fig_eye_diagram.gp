set terminal pngcairo size 800,520 enhanced font 'Helvetica,11'
set output 'fig_eye_diagram.png'

set grid
set title "Eye diagram of the matched-filter output (120 overlaid 2-symbol windows, SNR 8 dB)" font ',12'
set xlabel "time (symbols)"
set ylabel "MF output (rad/sample)"
set xrange [0:2]
set yrange [-0.3:0.3]
set xtics (0, 0.5, 1, 1.5, 2)
set ytics ("-π/20" -0.157, 0, "+π/20" +0.157)

# Decision instants are at x = 0, 1, 2 (symbol boundaries in this parameterization).
set arrow from 0,-0.3 to 0,0.3 nohead lc rgb '#cc6060' dt 2 lw 1
set arrow from 1,-0.3 to 1,0.3 nohead lc rgb '#cc6060' dt 2 lw 1
set arrow from 2,-0.3 to 2,0.3 nohead lc rgb '#cc6060' dt 2 lw 1
set label "decision" at 1.02,0.27 left tc rgb '#cc2020' font ',9'

plot 'eye_diagram.dat' using 1:2 with lines lw 0.6 lc rgb '#406090' notitle
