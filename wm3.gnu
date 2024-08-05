
set yrange [-8:8]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	


set title "WM3 Arm Position"

set multiplot layout 2,1

set ylabel "Position (radians)"
set ytics 0.2

plot 	"joint_state_1.csv" using 1:13 with lines title "pWheelActual[3]" lw 3, "target_1.csv" using 1:5 with lines title "vWheelTarget[3]" lw 3

set title "WM3 Wheel Velocity"
set ylabel "Velocity (radians/second)"
set autoscale xy
set xtics 0.5 
set ytics 0.5

plot 	"joint_state_1.csv" using 1:17 with lines title "vArmActual[3]" lw 3, "target_1.csv" using 1:9 with lines title "vArmTarget[3]" lw 3




