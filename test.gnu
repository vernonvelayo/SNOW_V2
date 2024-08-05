set xrange [0:5]
set yrange [0:20]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	
set xtics 5

set title "Wheel Position"

set multiplot layout 2,1

set ylabel "Position (meters/wheel radius)"
set ytics 10 

plot 	"joint_state.csv" using 1:2 with lines title "pWheel[0]", "joint_state.csv" using 1:3 with lines title "pWheel[1]", "joint_state.csv" using 1:4 with lines title "pWheel[2]", "joint_state.csv" using 1:5 with lines title "pWheel[3]", "joint_state.csv" using 1:18 with lines title "pState"

set title "Wheel Velocity"
set ylabel "Velocity (radians/second)"
set ytics 1

plot 	"joint_state.csv" using 1:6 with lines title "vWheel[0]", "joint_state.csv" using 1:7 with lines title "vWheel[1]", "joint_state.csv" using 1:8 with lines title "vWheel[2]", "joint_state.csv" using 1:9 with lines title "vWheel[3]" , "joint_state.csv" using 1:18 with lines title "pState" 



set xrange [0:5]
set yrange [0:20]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	
set xtics 5

set title "Arm Position"

set multiplot layout 2,1

set ylabel "Position (meters/Arm radius)"
set ytics 0.7853981634 

plot 	"joint_state.csv" using 1:10 with lines title "pArm[0]" lw 3, "joint_state.csv" using 1:11 with lines title "pArm[1]" lw 3, "joint_state.csv" using 1:12 with lines title "pArm[2]" lw 3, "joint_state.csv" using 1:13 with lines title "pArm[3]" lw 3, "joint_state.csv" using 1:18 with lines title "pState" lw 3

set title "Arm Velocity"
set ylabel "Velocity (radians/second)"
set ytics 1

plot 	"joint_state.csv" using 1:14 with lines title "vArm[0]" lw 3, "joint_state.csv" using 1:15 with lines title "vArm[1]" lw 3, "joint_state.csv" using 1:16 with lines title "vArm[2]" lw 3, "joint_state.csv" using 1:17 with lines title "vArm[3]"  lw 3, "joint_state.csv" using 1:18 with lines title "pState"  lw 3

