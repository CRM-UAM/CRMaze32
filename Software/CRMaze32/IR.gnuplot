set multiplot layout 5,1 rowsfirst
plot 'IRdata.txt' u ($1/1000):2 w l t "distanciaLeft",
plot 'IRdata.txt' u ($1/1000):3 w l t "DL", '' u ($1/1000):4 w l t "DR"
plot 'IRdata.txt' u ($1/1000):5 w l t "FL", '' u ($1/1000):6 w l t "FR"
plot 'IRdata.txt' u ($1/1000):8 w l t "SL", '' u ($1/1000):7 w l t "SR"
plot 'IRdata.txt' u ($1/1000):9 w l t "Error"

unset multiplot
pause -1 "Hit any key to continue"
