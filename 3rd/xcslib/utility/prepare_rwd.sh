if test $# = 0
    then
	echo ""
	echo "USAGE: prepare_rwd.sh [-f <sfx>] [-w <n>] [-i <n>] <input files>"
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
	else # 
		label=none
fi

# set window size for the moving average
if test "$1" = "-w"
	then 
		window=$2
		shift
		shift
	else # default window is on 100 results
		window=100
fi

# set window size for the moving average
if test "$1" = "-i"
	then 
		interval=$2
		shift
		shift
	else 
		interval=$window
fi


size=`grep "Population Size" confsys.$label | gawk '{print $4}'`
nofiles=`ls -l statistics.$label-*.gz | wc -l | gawk '{print $1;}'`
# max reward

xcs_rwd.sh -w $window -i $interval statistics.$label-*.gz
mm2 2 AVERAGE.perf.$label-$nofiles perf.$label-*

echo "REWARD MAX " $maxreward
