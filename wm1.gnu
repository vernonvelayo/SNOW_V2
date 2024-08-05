
set yrange [-8:8]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	


set title "WM1 Arm Position"

set multiplot layout 2,1

set ylabel "Position (radians)"
set ytics 0.2
set xrange[5:9]

plot 	"joint_state.csv" using 1:11 with lines title "pWheelActual[1]" lw 3, "target.csv" using 1:3 with lines title "vWheelTarget[1]" lw 3

set title "WM1 Wheel Velocity"
set ylabel "Velocity (radians/second)"
set autoscale xy
set xtics 0.5 
set ytics 0.5
set xrange[5:9]

plot 	"joint_state.csv" using 1:15 with lines title "vArmActual[1]" lw 3, "target.csv" using 1:7 with lines title "vArmTarget[1]" lw 3




