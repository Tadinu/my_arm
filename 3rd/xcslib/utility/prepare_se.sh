if test $# = 0
    then
	echo ""
	echo "USAGE: prepare_se.sh -f <label> -w <window> -i <interval>"
  	echo ""
	echo "  <label> is the suffix of the files on which the statistics are computed"
	echo "  -w <n> set the size <n> of the moving average"
	echo "  -i <n> output a point every <n> steps"
  	echo ""
	exit
fi

# set confsys suffix
if test "$1" = "-f"
	then 
		label=$2
		shift
		shift
	else # 
		label=none
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

size=`grep "Population Size" confsys.$label | gawk '{print $4}'`
nofiles=`ls -l statistics.$label-*.gz | wc -l | gawk '{print $1;}'`

xcs_se.sh -w $window -i $interval statistics.$label-*.gz
mm2 2 AVERAGE.system_error.$label-$nofiles system_error.$label-*
