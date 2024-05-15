#set term postscript eps color blacktext "Helvetica" 24

#set output "default_path_prokladka_position.eps"

#set size 3, 3
#set multiplot layout 3,2

#set title "Vx"
#plot "C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\velocity.dat" \
#			using 1:2 title "" with lines

#set title "Vz"
#plot "C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\velocity.dat" \
#			using 1:3 title "" with lines

#set title "acceleration"
#plot "C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\acceleration.dat" \
			 title "" with lines

#set title "wishVel"
#plot "C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\wishVel.dat" \
#			 title "" with lines

set title "path AND position"
set xrange[0:15]
set yrange[-10:5]
set size ratio -1
plot "C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\path.dat" \
			 title "path" with lines, \
	 "C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\wishPos.dat" \
	         using 1:2 title "turn path XZ"  with lines, \
	 "C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\position.dat" \
			 title "position" with points pt 5 ps 0.2

#set title "wishPos XY"
#plot	 "C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\wishPos.dat" \
#	         using 1:3 title "turn path XY" with lines

#set title "wishPos YZ"
#plot	"C:\\Users\\14\\source\\repos\\FVLib\\FVLib\\wishPos.dat" \
#	         using 2:3 title "turn path YZ" with lines

 set output
 quit