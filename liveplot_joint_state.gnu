
set yrange [-8:8]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	
set xtics 1
set ytics 1

set title "Wheel Position"


set ylabel "Position (meters/wheel radius)"
set ytics 10 

set terminal qt 0
plot 	"joint_state.csv" using 1:2 with lines title "pWheel[0]" lw 3, "joint_state.csv" using 1:3 with lines title "pWheel[1]" lw 3, "joint_state.csv" using 1:4 with lines title "pWheel[2]" lw 3, "joint_state.csv" using 1:5 with lines title "pWheel[3]" lw 3, "joint_state.csv" using 1:18 with lines title "pState"  lw 3

set title "Wheel Velocity"
set ylabel "Velocity (radians/second)"
set ytics 1




set terminal qt 1
plot 	"joint_state.csv" using 1:6 with lines title "vWheel[0]" lw 3, "joint_state.csv" using 1:7 with lines title "vWheel[1]" lw 3, "joint_state.csv" using 1:8 with lines title "vWheel[2]" lw 3, "joint_state.csv" using 1:9 with lines title "vWheel[3]"  lw 3, "joint_state.csv" using 1:18 with lines title "pState"  lw 3




