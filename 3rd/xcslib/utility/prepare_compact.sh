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

if test $# = 0
    then
	echo ""
	echo "USAGE: prepare_rwd.sh [-f <sfx>] <input files>"
  	echo ""
	echo "  <sfx> is the suffix of the files on which the statistics are computed"
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

size=`grep -i "Population Size" confsys.$label | gawk '{print $4}'`
nofiles=$#

echo "no files " $nofiles
echo "size " $size

PERFLIST=""
POPLIST=""
STEPLIST=""

for inFile in $*
do
  outperf="perf.`echo $inFile | gawk -F '.' '{print $2}'`"
  PERFLIST="$PERFLIST $outperf"
  zcat -f $inFile | gawk '{print $2 "\t" $4;}' > $outperf
  outpop="popul.`echo $inFile | gawk -F '.' '{print $2}'`"
  POPLIST="$POPLIST $outpop"
  zcat  -f $inFile | gawk --assign SIZE=$size '{print $2 "\t" 100*($5/SIZE);}' > $outpop
  outsteps="steps.`echo $inFile | gawk -F '.' '{print $2}'`"
  STEPLIST="$STEPLIST $outsteps"
  zcat -f $inFile | gawk '{print $2 "\t" $3;}' > $outsteps
  echo "done ... " $inFile
done
echo "LISTA " $PERFLIST

mm2 2 AVERAGE.perf.$label-$nofiles $PERFLIST
mm2 2 AVERAGE.popul.$label-$nofiles $POPLIST
mm2 2 AVERAGE.steps.$label-$nofiles $STEPLIST
