
set yrange [-8:8]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	


set title "WM2 Arm Position"

set multiplot layout 2,1

set ylabel "Position (radians)"
set ytics 0.2
set xrange[2:6]

plot 	"joint_state.csv" using 1:12 with lines title "pWheelActual[2]" lw 3, "target.csv" using 1:4 with lines title "vWheelTarget[2]" lw 3

set title "WM2 Wheel Velocity"
set ylabel "Velocity (radians/second)"
set autoscale xy
set xtics 0.5 
set ytics 0.5
set xrange[2:6]

plot 	"joint_state.csv" using 1:16 with lines title "vArmActual[2]" lw 3, "target.csv" using 1:8 with lines title "vArmTarget[2]" lw 3




