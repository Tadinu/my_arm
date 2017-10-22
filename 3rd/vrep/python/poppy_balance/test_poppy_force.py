# http://nbviewer.jupyter.org/github/jjehl/poppy_balance/blob/master/test_VREP_force.ipynb
from poppy.creatures import PoppyHumanoid

poppy = PoppyHumanoid(simulator='vrep',scene='poppy_humanoid_add_force.ttt')

%pylab inline

import time

"""
Populating the interactive namespace from numpy and matplotlib
"""

t0 = time.time()
t_simu = poppy.current_simulation_time
while time.time()-t0 <10:
    if poppy.current_simulation_time - t_simu < 1 :
        time.sleep(0.01)
    else :
        poppy.set_VREP_force([7,0,0],'l_forearm_respondable')
        t_simu = poppy.current_simulation_time
        while poppy.current_simulation_time - t_simu < 0.025:
            time.sleep(0.01)
        poppy.set_VREP_force([0,0,0],'l_forearm_respondable')   