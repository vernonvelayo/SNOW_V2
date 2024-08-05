set xrange [0:5]
set yrange [0:20]
set autoscale xy
set datafile separator ","
set xlabel "Time (s)"
set grid	

set title "Arm Position"

set multiplot layout 3,1

set ylabel "Position (meters/Arm radius)"
set ytics 0.1

plot 	"joint_state.csv" using 1:10 with lines title "pArm[0]" lw 3, "joint_state.csv" using 1:11 with lines title "pArm[1]" lw 3, "joint_state.csv" using 1:12 with lines title "pArm[2]" lw 3, "joint_state.csv" using 1:13 with lines title "pArm[3]" lw 3

set title "Arm Velocity"
set ylabel "Velocity (radians/second)"
set ytics 0.5

plot 	"joint_state.csv" using 1:14 with lines title "vArm[0]" lw 3, "joint_state.csv" using 1:15 with lines title "vArm[1]" lw 3, "joint_state.csv" using 1:16 with lines title "vArm[2]" lw 3, "joint_state.csv" using 1:17 with lines title "vArm[3]"  lw 3

set title "Arm Effort"
set ylabel "Torque (nm)"
set ytics 0.1

plot 	"commands.csv" using 1:6 with lines title "uArm[0]" lw 3, "commands.csv" using 1:7 with lines title "uArm[1]" lw 3, "commands.csv" using 1:8 with lines title "uArm[2]" lw 3, "commands.csv" using 1:9 with lines title "uArm[3]"  lw 3






