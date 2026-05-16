set terminal pngcairo size 800,520 enhanced font 'Helvetica,11'
set output 'fig_rs_cliff.png'

set grid
set title "Reed-Solomon(255,223) decode probability vs i.i.d. bit-error rate" font ',12'
set xlabel "bit error rate p_b  (i.i.d.)"
set ylabel "P(decode success)  =  P(byte errors ≤ 16 in 255)"
set logscale x
set xrange [1e-5:5e-2]
set yrange [-0.05:1.05]
set ytics 0,0.2,1.0
set format x "10^{%T}"

set arrow from 7.9e-3,0 to 7.9e-3,1 nohead lc rgb '#cc6060' dt 2 lw 1.5
set label "cliff near p_b ≈ 8×10^{-3}\n(byte-error rate ≈ 6 %, matches 16/255)" \
    at 1.2e-2,0.5 left tc rgb '#cc2020' font ',10'

plot 'rs_cliff.dat' using 1:2 with lines lw 2.4 lc rgb '#2060a0' notitle
