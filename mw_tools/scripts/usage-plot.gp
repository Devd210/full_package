#!/usr/bin/gnuplot --persist

infile=ARG1
outfile=ARG3
process=ARG2


set term x11
set terminal svg enhanced background rgb 'white'
set title 'Memory, CPU usage of ' . process
set xdata time
set timefmt "%s"
set xlabel "Time [[hh:]mm:ss]"
set ylabel "Memory usage"
set format y '%.1s%cB'
set y2label 'CPU usage'
set format y2 '%.0s%%'
set y2tics nomirror
set tics out
set autoscale y
set autoscale y2


# number of points in moving average
n = 50

# initialize the variables
do for [i=1:n] {
    eval(sprintf("back%d=0", i))
}

# build shift function (back_n = back_n-1, ..., back1=x)
shift = "("
do for [i=n:2:-1] {
    shift = sprintf("%sback%d = back%d, ", shift, i, i-1)
} 
shift = shift."back1 = x)"
# uncomment the next line for a check
# print shift

# build sum function (back1 + ... + backn)
sum = "(back1"
do for [i=2:n] {
    sum = sprintf("%s+back%d", sum, i)
}
sum = sum.")"
# uncomment the next line for a check
# print sum

# define the functions like in the gnuplot demo
# use macro expansion for turning the strings into real functions
samples(x) = $0 > (n-1) ? n : ($0+1)
avg_n(x) = (shift_n(x), @sum/samples($0))
shift_n(x) = @shift

resolveUnit(u,mul,x)=(pos=strstrt(x,u), pos > 0 ? sprintf("%f",real(substr(x,1,pos - 1))*mul) : x)
resolveUnits(x)=(resolveUnit("g",1024*1024*1024,resolveUnit("m",1024*1024,x)))
check(x)=(real(resolveUnits(x)))


plot infile u 1:3 with lp axes x1y2 title "cpu" linestyle 2, \
    infile using 1:(check(stringcolumn(2))) with linespoints title "memory" linestyle 1

MAX=GPVAL_Y_MAX
MIN=GPVAL_Y_MIN
set yrange [MIN-(MAX-MIN)*0.1:MAX+(MAX-MIN)*0.1]
MAX2=GPVAL_Y2_MAX
MIN2=GPVAL_Y2_MIN
set y2range [MIN2-(MAX2-MIN2)*0.1:MAX2+(MAX2-MIN2)*0.1]


if (exists("outfile") && strlen(outfile) > 0) {
    print "Outputting to the file ", outfile
    set term svg # 640,480
    set output outfile
}

# Styling
set style line 1 linewidth 2 linecolor 'blue'
set style line 2 linecolor 'light-green'
#set xtics font ", 10"
#set tics font ", 10"
set xtics rotate 60 # put label every 60s, make vertical so they don't clash in .png if too many
plot infile u 1:(avg_n($3)) w l lc rgb "light-green" axes x1y2 title "avg cpu", \
    infile using 1:(check(stringcolumn(2))) w l lc rgb "blue" title "memory", \
