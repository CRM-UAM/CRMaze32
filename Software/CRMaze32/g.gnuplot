plot 'data.txt' u ($1/1000):4 w l t "WSpeed",  '' u ($1/1000):6 w l t "gyroError",  '' u ($1/1000):9 w l t "encFeed", '' u ($1/1000):($7/10) w l t "WallError", '' u ($1/1000):8 w l t "TotalWError"
pause -1 "Hit any key to continue"
