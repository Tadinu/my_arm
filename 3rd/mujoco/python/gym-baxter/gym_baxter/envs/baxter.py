import numpy as np
import six
import mujoco_py
from mujoco_py import load_model_from_path, MjSim, MjViewer
#from mujoco_py.mjlib import mjlib
#from mujoco_py.mjtypes import *

#from pydart_ik import MyWorld
#import pydart2 as pydart

from scipy.optimize import minimize

#from ipdb import set_trace

class BaxterEnv():
    def __init__(self, fullpath="/home/brhm/OPENAI/baxter/baxter.xml", visible= True):                
        self.model = load_model_from_path(fullpath)
        self.sim = MjSim(self.model)
        self.data = self.sim.data
        self.idx = {
                        "right" : [1,7],
                        "left" : [10,16]
                    }

        self.gripper_idx = {
                        "right" : [8,9],
                        "left" : [17,18]
                    }

        self.tuck_pose = {
                       'left':  [[-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50]],
                       'right':  [[0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]]
                       }

        ctrl = self.data.ctrl.copy()
        self.data.ctrl[-1] = 10
        # self.apply_action(action={"left": np.array(self.tuck_pose["left"] ), "right"=[]})
                
        self.viewer = mujoco_py.MjViewer(self.sim)
        self.viewer.render()
        #self.viewer.set_model(self.model)

        # cam_pos = np.array([0.1, 0.0, 0.7, 0.01, -45., 0.])
        cam_pos = np.array([1.0, 0.0, 0.7, 0.5, -45, 180])
        self.set_cam_position(self.viewer, cam_pos)

        # ik model
        # pydart.init()
        # self.pydart_model = MyWorld()
        
    def get_jacobian(self,site=0):
        # site : 0 for left hand gripper
        ## return translation jacobian of the first site

        jac = np.zeros((3,7))
        idx = site * 3
        temp = np.zeros((3, 26))
        mj_jacSite(env.model.ptr, env.data.ptr, temp.ctypes.data_as(POINTER(c_double)), None, site)
        jac = temp[:,10:17]
        print('jac')
        print(jac)
        print("----")
        return jac

    def apply_action(self, 
                        action = {"left": None, "right" : None}
                        ):
        
        ctrl = self.data.ctrl.copy()
        if len(action["left"]) >0:
            ctrl[7:14,0] = np.array(action["left"])
        if len(action["right"])>0:
            ctrl[:7,0] = np.array(action["right"] )

        self.data.ctrl = ctrl


    def set_params(self, x):
        # print("new state")
        # print(x)
        # print("----")
        curr_qpos = self.data.qpos.copy()
        curr_qpos[12:19] = x[:, np.newaxis]
        self.data.qpos = curr_qpos
        # env.apply_action(action={"right": [], "left": x})
        for _ in range(1000):   self.step()


    def f(self, x):
        self.set_params(x)

        lhs = env.data.site_xpos[0]
        rhs = self.target
        cost = 0.5 * np.linalg.norm(lhs - rhs) ** 2
        print("cost:%.4f"%cost)
        # set_trace()
        return cost


    def g(self, x):
        self.set_params(x)

        lhs = env.data.site_xpos[0]
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
        res = minimize(self.f,
                       x0=jt_pos,
                       jac=self.g,
                       method="SLSQP")
        print(res)
        return res.x
        
    def set_state(self, qpos, qvel):
        assert qpos.shape == (self.model.nq,) and qvel.shape == (self.model.nv,)
        self.data.qpos = qpos
        self.data.qvel = qvel
        self.model._compute_subtree() #pylint: disable=W0212
        self.model.forward()

    def close_gripper(self, left_gap=0, right_gap=0):
        pass

    def get_body_com(self, body_name):
        idx = self.model.body_names.index(six.b(body_name))
        return self.data.com_subtree[idx]

    def get_idx_by_name(self, body_name):
        return self.model.body_names.index(six.b(body_name))

    def set_cam_position(self, viewer, cam_pos):
        for i in range(3):
            viewer.cam.lookat[i] = cam_pos[i]
        viewer.cam.distance = cam_pos[3]
        viewer.cam.elevation = cam_pos[4]
        viewer.cam.azimuth = cam_pos[5]
        viewer.cam.trackbodyid = -1

    def get_obs(self):
        return np.concatenate([
            self.data.qpos.flat[2:],
            self.data.qvel.flat,
            np.clip(self.data.cfrc_ext, -1, 1).flat,
        ])

    def reset_model(self):
        qpos = self.init_qpos #+ np.random.uniform(size=self.model.nq,low=-.1,high=.1)
        qvel = self.init_qvel #+ np.random.randn(self.model.nv) * .1
        self.set_state(qpos, qvel)
        
        return self.get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5

    def step(self):
        self.sim.step()
        #self.viewer.loop_once()
        #self.model.step()

    def close_gripper(self):
        pass



if __name__ == "__main__":
    env = BaxterEnv()
    
    
    print(env.data.site_xpos[3])
    
    i = 0
    while(True):
       
        env.step()
        i += 1
        if(i==1000):
            ## left hand only
            target = env.data.site_xpos[3].copy() + np.array([0.0, 0.0, 0.05])            
            new_pose = env.do_ik(ee_target= target, jt_pos = env.data.qpos[12:19])
            print(new_pose.T)
            # set_trace()
            env.apply_action(action={"right": [], "left": new_pose})

            print(env.data.ctrl)
            # set_trace()

        if(i==80000):
            print("stable pose")
            print(env.data.qpos[7:14].T)
            print(env.data.site_xpos[0])
            # set_trace()
        env.viewer.render()

    