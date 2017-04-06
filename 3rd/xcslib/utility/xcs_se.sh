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
	then # larghezza finestra specificata sulla linea di comando
		interval=$2
		shift
		shift
	else # imposta finestra di 50 dati
		interval=1
fi


if test $# -ge 1
then # specificati almeno due parametri: procede ...
	outBaseName=system_error

	for inFileName in $*
	do
		outFileName="$outBaseName.`echo $inFileName | gawk -F '.' '{print $2}'`"
		zcat -f $inFileName | gawk '{if ($NF=="Testing") print $6;}' | moving_average $window |
		gawk --assign WINDOW=$window --assign INTERVAL=$interval \
			'{last=$0; if ((NR==1)||((NR+WINDOW-1)%INTERVAL)==0) printf("%-7d\t %7.4f\n", NR+WINDOW-1, $1);}END{printf("%-7d\t %7.4f\n",NR+WINDOW,last);}'> $outFileName
		echo "$inFileName > $outFileName ..."
	done
else # parametri specificati insufficienti: spiega utilizzo
	echo ""
	echo "USAGE: xcs_se [-w <n>] <input file>"
	echo ""
	echo "  <input prefix> is the prefix of the files on which the statistics are computed"
	echo "  -w <n> set the size <n> of the moving average"
	echo ""
	echo ""

fi;
