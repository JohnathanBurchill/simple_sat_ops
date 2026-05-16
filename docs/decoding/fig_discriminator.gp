set terminal pngcairo size 1000,720 enhanced font 'Helvetica,11'
set output 'fig_discriminator.png'

# Δφ[n] = arg(s[n]·s*[n-1]) at three SNRs.
# Signal is ±2π·dev/fs = ±2π·2400/96000 ≈ ±0.157 rad/sample.

set multiplot layout 3,1 margins 0.10,0.97,0.10,0.94 spacing 0,0.04
set grid
set xrange [0:32.0/9.6]
set yrange [-1.0:1.0]
set ytics (-0.5, "-π/20" -0.157, 0, "+π/20" 0.157, 0.5)
set ylabel "Δφ (rad/sample)"

set title "FM discriminator output Δφ[n] = arg(s[n]·s*[n-1])  —  same 32 bits, three SNR levels" font ',12'
unset xtics
plot 'discriminator.dat' using 1:2 with lines lw 1.5 lc rgb '#2060a0' title 'noise-free'

set title ""
plot 'discriminator.dat' using 1:3 with lines lw 1.2 lc rgb '#208040' title 'SNR 10 dB'

set xtics auto
set xlabel "time (ms)"
set yrange [-3.5:3.5]
set ytics (-3, "-π/20" -0.157, 0, "+π/20" 0.157, 3)
plot 'discriminator.dat' using 1:4 with lines lw 1.0 lc rgb '#a02020' title 'SNR 0 dB  (atan2 click spikes ≈ ±π)'

unset multiplot
