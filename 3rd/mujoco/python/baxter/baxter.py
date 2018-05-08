#!/usr/bin/env python3
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print('CURRENT DIR:', currentdir)
parentdir = os.path.dirname(currentdir)
print('PARENT DIR:', parentdir)
os.sys.path.insert(0, parentdir)

import math
import os
import copy
import numpy as np

from robot import Robot # robot.py in parentdir
import rotations, utils # in currentdir
from utils import seeding

try:
    import mujoco_py
except ImportError as e:
    raise error.DependencyNotInstalled("{}. (HINT: you need to install mujoco_py, and also perform the setup instructions here: https://github.com/openai/mujoco-py/.)".format(e))

CMODEL_PATH   = "/home/brhm/DUC/RobotArm/src/my_arm/3rd/mujoco/python/baxter/assets/baxter.xml"
CINITIAL_QPOS = {
                    "right_s0" : 0.4049,
                    "right_s1" : 0.48,
                    "right_e0" : 0.0,
                    "right_e1" : 1.05,
                    "right_w0" : 0.4,
                    "right_w1" : 0.0,
                    "right_w2" : 0.0,

                    "left_s0"  : 0.1,
                    "left_s1"  : 0.2,
                    "left_e0"  : 0.3,
                    "left_e1"  : 0.4,
                    "left_w0"  : 0.5,
                    "left_w1"  : 0.4,
                    "left_w2"  : 0.4,

                    #'table0:slide0': 1.05,
                    #'table0:slide1': 0.4,
                    #'table0:slide2': 0.0,
                    'box': [1.25, 0.53, 0.4, 1., 0., 0., 0.],
                }

class Baxter(Robot):
    def __init__(self, model_path, n_substeps):
        super(Manipulator, self).__init__(
            model_path=model_path, initial_qpos=initial_qpos, n_substeps=n_substeps)

    # ==========================================================================
    # SIMULATOR METHODS ========================================================
    #

    # ==========================================================================
    # ROBOT METHODS ============================================================
    #
    def act(self, action):
        assert action.shape == (14,)
        action = action.copy()  # ensure that we don't change the action outside of this scope
        left_ctrl, right_ctrl = action[:7], action[7:]
        # Pos Control
        print('ACTION :', action)
        self.sim.data.ctrl[7:14] = right_ctrl
        self.sim.data.ctrl[:7]   = left_ctrl

        self.count += 1
        self.sim.step()
        self.viewer.render()
        if self.count > 100 and os.getenv('TESTING') is not None:
            break

    def get_jacobian(self,site=0):
        '''
        void mj_jac(const mjModel* m,const mjData* d, mjtNum* jacp, mjtNum* jacr, const mjtNum* point, int body);

        jacp and jacr point to 3-by-nv matrices you allocated that will hold the result.
        If either is NULL, that Jacobian is not computed. The point is in 3D global coordinates.
        body is the body index.

        There are also helper functions mj_jacBody, mj_jacSite etc that compute the Jacobian of the frame
        of the specified body, site etc.

        Note that the position-dependent stages of the computation must have been executed for the current
        state in order for these functions to return correct results. So to be safe, do mj_forward and then
        mj_jac. If you do mj_step and then call mj_jac, the Jacobians will correspond to the state before
        the integration of positions and velocities took place.
        '''
        # site : 0 for left hand gripper
        ## return translation jacobian of the first site

        jac = np.zeros((3,7))
        idx = site * 3
        temp  = np.zeros((3,25))
        temp2 = np.ndarray((3,25))
        temp2 = np.reshape(temp2, (75))
        #temp3 = np.ndarray(shape=(75), dtype=float, order='F')
        #print(temp3)
        cymj._mj_jacSite(env.sim.model, env.sim.data, temp2, None, site)

        #jac = temp[:,10:17]
        print('jac')
        print(jac)
        print("----")
        return jac

    def set_params(self, x):
        # print("new state")
        # print(x)
        # print("----")

        i = 0
        for name, value in initial_qpos.items():
            self.sim.data.set_joint_qpos(name, x[i])
            i+=1
            if(i==7):
                break

        #curr_qpos = self.sim.data.qpos.copy()
        #print('SHAPE', x[:, np.newaxis].shape,  curr_qpos[12:19].shape)
        #curr_qpos[12:19] = x
        #self.sim.data.qpos = curr_qpos
        # env._set_action(action={"right": [], "left": x})
        for _ in range(1000):   self.sim.step()


    def f(self, x):
        self.set_params(x)

        lhs = env.sim.data.site_xpos[0]
        rhs = self.target
        cost = 0.5 * np.linalg.norm(lhs - rhs) ** 2
        print("cost:%.4f"%cost)
        # set_trace()
        return cost


    def g(self, x):
        self.set_params(x)

        lhs = env.sim.data.site_xpos[0]
        rhs = self.target

        J = self.get_jacobian(site=0)
        g = (lhs - rhs)[np.newaxis, :].dot(J).flatten()

        print("g")
        print(g)

        return g

    def do_ik(self, ee_target, jt_pos):
        # print("doing ik using pydart")
        # self.pydart_model.target = ee_target            ## 3d ee pose
        # self.pydart_model.set_params(jt_pos.flatten())            ## 14d current jt pose
        # return self.pydart_model.solve()

        self.target = ee_target
        print(jt_pos)
        res = minimize(self.f,
                       x0=jt_pos,
                       jac=self.g,
                       method="SLSQP")
        print(res)
        return res.x

    def set_state(self, qpos, qvel):
        assert qpos.shape == (self.sim.model.nq,) and qvel.shape == (self.sim.model.nv,)
        self.sim.data.qpos = qpos
        self.sim.data.qvel = qvel
        self.sim.model._compute_subtree() #pylint: disable=W0212
        self.sim.model.forward()

    def get_body_com(self, body_name):
        idx = self.sim.model.body_names.index(six.b(body_name))
        return self.sim.data.com_subtree[idx]

    def get_idx_by_name(self, body_name):
        return self.sim.model.body_names.index(six.b(body_name))


if __name__ == "__main__":
    robot = Baxter(CMODEL_PATH, initial_qpos=CINITIAL_QPOS, n_substeps=20)

    while True:
        robot.act()
