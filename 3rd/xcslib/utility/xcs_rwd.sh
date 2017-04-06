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


# generate the plot for the reward

# set window size for the moving average
if test "$1" = "-w"
	then # larghezza finestra specificata sulla linea di comando
		window=$2
		shift
		shift
	else # imposta finestra di 50 dati
		window=1
fi

# set columns
if test "$1" = "-i"
	then # interval for plotting
		interval=$2
		shift
		shift
	else # 
		interval=1
fi

# set range
if test "$1" = "-r"
	then # reward range for normalization 
		range=$2
		shift
		shift
	else # by default the range is 1000
		range=1000
fi

if test $# -ge 1
  then # specificati almeno due parametri: procede ...
    outBaseName=perf
    #shift
    for inFileName in $*
    do
      outFileName="$outBaseName.`echo $inFileName | gawk -F '.' '{print $2}'`"
      if test "$interval" = 1
         then 
		   zcat -f $inFileName | gawk --assign RANGE=$range '{if ($NF=="Testing") printf("%12.4f", $4/RANGE;}' | moving_average $window > $outFileName
		 else   
		   zcat -f $inFileName | grep Testing\$ | gawk '{print $4}' | moving_average $window | \
		   gawk --assign WINDOW=$window --assign INTERVAL=$interval --assign RANGE=$range \
			  '{last=$0; if ((NR==1)||((NR+WINDOW-1)%INTERVAL)==0) printf("%-7d\t %12.4f\n", NR+WINDOW-1, ($1/RANGE)*100);}END{printf("%-7d\t %7.4f\n",NR+WINDOW,(last/RANGE)*100);}'> $outFileName
	  fi   
	  echo "$inFileName > $outFileName ..."
    done
  else # parametri specificati insufficienti: spiega utilizzo
  	echo ""
	echo "USAGE: xcs_rwd.sh [-w <window>] [-i <interval>] [-r <range>] <input files>"
  	echo ""
	echo "  <window> window for the moving average"
	echo "  <interval> report only every <inteval> points"
	echo "  <range> range of the incoming reward: it is used to normalize the output between 0 and 100"
	echo "  <input files> statistics files"
	echo ""
  	echo ""
fi;
