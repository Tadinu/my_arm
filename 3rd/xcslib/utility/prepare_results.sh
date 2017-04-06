
if test $# -ge 4
  then 
  # at least three parameters are needed

  # define the window for the moving average
  window=$1

  # define the interval between the plotted points
  points=$2

  # label for the result files
  label=$3

  # number of experiments
  exps=$4

  # population size
  size=$5

  # max reward
  maxreward=$6

  xcs_reward -s -f $window perf statistics.$label-*.gz
  xcs_steps  -s -f $window steps statistics.$label-*.gz
  xcs_population -s -f $window popul statistics.$label-*.gz
  mm perf.$label- $exps
  mm steps.$label- $exps
  mm popul.$label- $exps

  gawk --assign INTERVAL=$points --assign MAX=$maxreward \
       '{if ( (NR==1) || (NR%INTERVAL==0) ) printf("%d\t%5.2f\n", NR, ($1/MAX)*100);}' average.perf.$label-$exps > AVERAGE.perf.$label-$exps
  gawk --assign INTERVAL=$points --assign N=$size \
       '{if ( (NR==1) || (NR%INTERVAL==0) ) printf("%d\t%5.2f\n", NR, 100*($1/(N)));}' average.popul.$label-$exps > AVERAGE.popul.$label-$exps
  gawk --assign INTERVAL=$points '{if ( (NR==1) || (NR%INTERVAL==0) ) print NR "\t" $0;}' average.steps.$label-$exps > AVERAGE.steps.$label-$exps
else 
	echo "xcs_prepare_results <window> <interval> <label> <number of experiments> <N> <max reward>"
fi;
