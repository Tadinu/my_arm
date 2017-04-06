if test $# = 0
    then
	echo ""
	echo "USAGE: prepare_pop.sh [-f <sfx>] [-w <n>] [-i <n>] <input files>"
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


size=`grep -i "Population Size" confsys.$label | gawk '{print $4}'`
nofiles=`ls -l statistics.$label-*.gz | wc -l | gawk '{print $1;}'`
# max reward

xcs_pop.sh -w $window -i $interval statistics.$label-*.gz
mm2 2 average.popul.$label-$nofiles popul.$label-*

echo "INTERVAL = " $interval
echo "N = " $size

gawk --assign INTERVAL=$interval --assign N=$size \
         '{printf("%d\t%9.5f\n", $1, 100*($2/(N)));}' \
		 average.popul.$label-$nofiles > AVERAGE.popul.$label-$nofiles
rm -f average.popul.$label-$nofiles 
