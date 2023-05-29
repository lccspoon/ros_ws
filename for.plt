set multiplot

set datafile separator ' '
set origin 0.0,0.66
set size 1,0.33
plot "robot_force.txt" using 0:1 w l lt 1 lw 1

set origin 0.0,0.33
set size 1,0.33
plot  "robot_force.txt" using 0:2 w l lt 2 lw 1

set origin 0.0,0.0
set size 1,0.33
plot  "robot_force.txt" using 0:3 w l lt 3 lw 1



