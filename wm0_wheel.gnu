
set yrange [-8:8]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	


set title "WM0 Wheel Position"

set multiplot layout 2,1

set ylabel "Position (radians)"
set xrange[5:30]

plot 	"joint_state.csv" using 1:2 with lines title "pWheelActual[0]" lw 3, "target.csv" using 1:14 with lines title "pWheelTarget[0]" lw 3, "joint_state.csv" using 1:18 with lines title "pState"  lw 3

set title "WM0 Wheel Velocity"
set ylabel "Velocity (radians/second)"
set autoscale xy
set xtics 0.5 
set xrange[5:30]

plot 	"joint_state.csv" using 1:6 with lines title "vWheelActual[0]" lw 3, "target.csv" using 1:6 with lines title "vWheelTarget[0]" lw 3, "commands.csv" using 1:2 with lines title "uWheel[0]" lw 3, "commands.csv" using 1:6 with lines title "uArm[0]" lw 3




