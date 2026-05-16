set terminal pngcairo size 800,500 enhanced font 'Helvetica,11'
set output 'fig_gardner_scurve.png'

set grid
set title "Gardner TED — average detector output vs sample-timing offset" font ',12'
set xlabel "timing offset τ (samples; symbol period = 10)"
set ylabel "mean TED  ⟨y_{mid} · (y_k − y_{k−1})⟩"
set xrange [-5:5]
set yrange [-7e-3:7e-3]
set xzeroaxis lt -1 lc rgb '#888888'
set yzeroaxis lt -1 lc rgb '#888888'

set arrow from 0,-7e-3 to 0,7e-3 nohead lc rgb '#cc6060' dt 2 lw 1.5
set label "stable lock\n(τ = 0)" at 0.2,5e-3 left tc rgb '#cc2020' font ',10'

# Annotate restoring force: positive TED → reduce τ (negative correction)
set arrow from 3,4.2e-3 to 1.2,1.5e-3 lw 1 lc rgb '#444444' filled
set label "late timing →\n+TED → −Kp·TED\npushes τ back to 0" at 3.1,4.2e-3 left tc rgb '#444444' font ',9'

plot 'gardner_scurve.dat' using 1:2 with lines lw 2.4 lc rgb '#2060a0' notitle
