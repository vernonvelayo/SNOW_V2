
# Set the font for the title
set title "Plot Title" font "Times New Roman,26"

# Set the font for the x-axis tick labels
set xtics font "Times New Roman,17"

# Set the font for the y-axis tick labels
set ytics font "Times New Roman,17"

# Set the font for the key (legend)
set key font "Times New Roman,26"

# Set the margins around each subplot
set bmargin 5

set key left

set autoscale xy
set datafile separator ","
set xtics 0.5
set ytics 0.5
set xrange[9.5:13]


set grid	

set xlabel "Time (s)" font "Times New Roman,20"
set ylabel "Position (radians)" font "Times New Roman,20"
set title "WM2 Arm Position" font "Times New Roman,26"

set multiplot layout 2,1


plot 	"joint_state.csv" using 1:12 with lines title "pArmActual[2]" lw 3, "target.csv" using 1:4 with lines title "pArmTarget[2]" lw 4

set title "WM2 Wheel Velocity" font "Times New Roman,26"
set ylabel "Velocity (radians/second)"  font "Times New Roman,20"

set autoscale xy
set xrange[9.5:13]
set xtics 0.5
set ytics 0.5

plot 	"joint_state.csv" using 1:16 with lines title "vArmActual[2]" lw 3, "target.csv" using 1:8 with lines title "vArmTarget[2]" lw 3


