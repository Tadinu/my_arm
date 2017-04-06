if test $# = 0
    then
	echo ""
	echo "USAGE: prepare_steps.sh [-f <sfx>] [-w <n>] [-i <n>] <input file>"
  	echo ""
	echo "  <input prefix> is the prefix of the files on which the statistics are computed"
	echo "  -w <n> set the size <n> of the moving average"
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
		echo ""
		echo "USAGE: prepare_steps.sh [-f <sfx>] [-w <n>] [-i <n>]"
  		echo ""
		echo "  <input prefix> is the prefix of the files on which the statistics are computed"
		echo "  -w <n> set the size <n> of the moving average"
		echo "  -i <n> set the size <n> of the plotting interval"
		echo ""
  		echo ""
		exit
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
# max reward

xcs_steps.sh -w $window -i $interval statistics.$label-*.gz
mm2 2 AVERAGE.steps.$label-$nofiles steps.$label-*

echo "REWARD MAX " $maxreward
