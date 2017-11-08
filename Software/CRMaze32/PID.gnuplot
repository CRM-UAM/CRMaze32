set multiplot layout 4,1 rowsfirst
plot 'data.txt' u ($1/1000):2 w l t "rEncoder", '' u ($1/1000):3 w l t "lEncoder",
plot 'data.txt' u ($1/1000):($2-$3) w l, '' u ($1/1000):4 w l t "xSpeed"
plot 'data.txt' u ($1/1000):4 w l t "WSpeed",  '' u ($1/1000):6 w l t "gyroError",  '' u ($1/1000):9 w l t "encFeed", '' u ($1/1000):($6+$9)/2 w l t "WError"
plot 'data.txt' u ($1/1000):7 w l t "leftSpeed", '' u ($1/1000):8 w l t "rSpeed"


unset multiplot
pause -1 "Hit any key to continue"
