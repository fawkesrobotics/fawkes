set xlabel "Time (sec)" font "Times-Roman,10"
set ylabel "Run-Time per Loop (sec)" font "Times-Roman,10"
set xtics out nomirror font "Times-Roman,8"
set ytics out nomirror font "Times-Roman,8"
set object 1 rect from graph 0, graph 0 to graph 1, graph 1 back
set object 1 fc rgb "#fafafa"
# To disable legend:
#set key off
# For PDF output (requires patched gnuplot):
#set term pdf color enhanced font "Times-Roman" size 14.2cm,6cm
#set output "runtime-chart.pdf"
plot "time.log" u (2*$1):2:9 t "Sensor" with filledcu lt rgb "#e6e6ff", "" u (2*$1):9:14 t "World State" with filledcu lt rgb "blue", "" u (2*$1):14:19 t "Think" with filledcu lt rgb "#40ff40", "" u (2*$1):19:24 t "Skill" with filledcu lt rgb "orange", "" u (2*$1):24:29 t "Act" with filledcu lt rgb "red", "" u (2*$1):47 t "Total" with lines lt rgb "#8000ff"
pause -1
