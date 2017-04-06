if test $# = 0
    then
	echo ""
	echo "USAGE: prepare_trace.sh [-f <sfx>] [-c <col>] [-p <pre>] [-w <n>] [-i <n>] <input files>"
  	echo ""
	echo "  <sfx> is the suffix of the files on which the statistics are computed"
	echo "  -w <n> set the size <n> of the moving average"
	echo "  -i <n> output a point every <n> steps"
	echo "  <input files> the statistics files where data have been saved"
	echo ""
  	echo ""
	exit
fi

# set confsys suffix
if test "$1" = "-f"
	then 
		label=$2
		shift
		shift
	else # imposta finestra di 50 dati
		label=none
fi

# which column? 
if test "$1" = "-c"
	then # column to be analyzed given by -c option
		column=$2
		shift
		shift
	else # default column is 1
		window=100
fi

# what prefix for the output files? 
if test "$1" = "-p"
	then # prefix given by -p option
		prefix=$2
		shift
		shift
	else # default prefix is stat
		prefix="stat"
fi

# set window size for the moving average
if test "$1" = "-w"
	then 
		window=$2
		shift
		shift
	else # imposta finestra di 50 dati
		window=100
fi

# set window size for the moving average
if test "$1" = "-i"
	then 
		interval=$2
		shift
		shift
	else # imposta finestra di 50 dati
		interval=$window
fi

nofiles=`ls -l statistics.$label-*.gz | wc -l | gawk '{print $1;}'`

xcs_ma.sh -c $column -p $prefix -w $window -i $interval trace.$label-*.gz
mm2 2 AVERAGE.$prefix.$label-$nofiles $prefix.$label-*
