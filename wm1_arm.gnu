
set yrange [-8:8]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	


set title "WM0 Arm Position"

set multiplot layout 2,1

set ylabel "Position (radians)"
set ytics 0.5

plot 	"joint_state.csv" using 1:11 with lines title "pArmActual[0]" lw 3, "target.csv" using 1:3 with lines title "pArmTarget[0]" lw 3, "joint_state.csv" using 1:18 with lines title "pState"  lw 3

set title "WM0 Wheel Velocity"
set ylabel "Velocity (radians/second)"
set autoscale xy
set xtics 0.5 
set ytics 0.5

plot 	"joint_state.csv" using 1:15 with lines title "vArmActual[0]" lw 3, "target.csv" using 1:7 with lines title "vArmTarget[0]" lw 3, "commands.csv" using 1:3 with lines title "uWheel[0]" lw 3, "commands.csv" using 1:7 with lines title "uArm[0]" lw 3




