import json
from itertools import chain

# Script to create Q-Value JSON file, initilazing with zeros

qval = {}
# Z -> [-40,-30...120] U [140, 210 ... 490]
for x in chain(list(range(-4,14,1)), list(range(14,42,7))):
    for y in chain(list(range(-30,18,1)), list(range(18,42,6))):
        for z in chain(list(range(-30,18,1)), list(range(18,42,6))):
            for v in range(-10,11):
                key_str = ''
                for i in range(5):
                    key_str += str(str(x)+'_'+str(y)+'_'+str(z)+'_'+str(v)+'_')
                qval[key_str] = [0,0,0]


fd = open('qvalues.json', 'w')
json.dump(qval, fd)
fd.close()
