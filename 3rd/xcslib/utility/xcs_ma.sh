# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.


# work silent
silent=1

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
	then # size of the moving window is given by -w option
		window=$2
		shift
		shift
	else # default window is 100
		window=100
fi

# set columns
if test "$1" = "-i"
	then # printing interval given by -i
		interval=$2
		shift
		shift
	else # default interval is 100
		interval=1
fi


if test $# -ge 1
  then # there are at least three parameters
    outBaseName=$prefix
    for inFileName in $*
    do
      outFileName="$outBaseName.`echo $inFileName | gawk -F '.' '{print $2}'`"
      if test "$interval" = 1
         then 
           zcat -f $inFileName | gawk --assign COLUMN=$column '{if ($NF=="Testing") print $COLUMN;}' | moving_average $window > $outFileName
	 else   
	   zcat -f $inFileName | grep Testing\$ | gawk --assign COLUMN=$column '{print $COLUMN}' | moving_average $window | \
		   gawk --assign WINDOW=$window --assign INTERVAL=$interval \
		  '{last=$0; if ((NR==1)||((NR+WINDOW-1)%INTERVAL)==0) printf("%-7d\t %12.4f\n", NR+WINDOW-1,$1);}END{printf("%-7d\t%7.4f\n",NR+WINDOW,last);}'> $outFileName

	  fi
	  echo "$inFileName > $outFileName ..."
    done
  else # parametri specificati insufficienti: spiega utilizzo
  	echo ""
	echo "USAGE: xcs_ma.sh [-c <col>] [-p <pre>] [-w <window>] [-i <interval>] <input files>"
  	echo ""
	echo "  <input prefix> prefix of the files on which the statistics are computed"
	echo ""
	echo "  <window>       set the size <n> of the moving average"
	echo ""
	echo "  <interval>     report only every <inteval> points"
	echo ""
	echo "  <input files>  statistics files used to generate the population average"
	echo ""
	echo "  computes the moving average for the data in column <col> and saves"
	echo "  the data in a set of files with prefix <pre>"
	echo ""
fi;
