set xrange [0:4]
set yrange [8:12]
set autoscale x
set datafile separator ","
set xlabel "Time (s)"
set ylabel "Latency (ms)"
set grid	
set xtics 5

plot "latency.csv" using 1:2 with lines  lw 3

