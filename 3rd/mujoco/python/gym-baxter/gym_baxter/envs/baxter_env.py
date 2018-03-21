import numpy as np
import ctypes
from gym.envs.robotics import rotations, robot_env, utils

from scipy.optimize import minimize


try:
    import mujoco_py
except ImportError as e:
    raise error.DependencyNotInstalled("{}. (HINT: you need to install mujoco_py, and also perform the setup instructions here: https://github.com/openai/mujoco-py/.)".format(e))

from mujoco_py import cymj

def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


class BaxterEnv(robot_env.RobotEnv):
    """Superclass for all Fetch environments.
    """

    def __init__(
        self, model_path, n_substeps,
        distance_threshold, initial_qpos, reward_type
    ):
        """Initializes a new Fetch environment.

        Args:
            model_path (string): path to the environments XML file
            n_substeps (int): number of substeps the simulation runs on every call to step
            gripper_extra_height (float): additional height above the table when positioning the gripper
            block_gripper (boolean): whether or not the gripper is blocked (i.e. not movable) or not
            has_object (boolean): whether or not the environment has an object
            target_in_the_air (boolean): whether or not the target should be in the air above the table or on the table surface
            target_offset (float or array with 3 elements): offset of the target
            obj_range (float): range of a uniform distribution for sampling initial object positions
            target_range (float): range of a uniform distribution for sampling a target
            distance_threshold (float): the threshold after which a goal is considered achieved
            initial_qpos (dict): a dictionary of joint names and values that define the initial configuration
            reward_type ('sparse' or 'dense'): the reward type, i.e. sparse or dense
        """
        self.distance_threshold = distance_threshold
        self.reward_type = reward_type

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

        super(BaxterEnv, self).__init__(
            model_path=model_path, n_substeps=n_substeps, n_actions=4,
            initial_qpos=initial_qpos)

        self.sim.data.ctrl[-1] = 10

        # Camera Viewer
        self.viewer = self._get_viewer()
        # cam_pos = np.array([0.1, 0.0, 0.7, 0.01, -45., 0.])
        cam_pos = np.array([1.0, 0.0, 0.7, 0.5, -45, 180])
        self.set_cam_position(cam_pos)

    # GoalEnv methods
    # ----------------------------

    def compute_reward(self, achieved_goal, goal, info):
        # Compute distance between goal and the achieved goal.
        d = goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            return -d

    # RobotEnv methods
    # ----------------------------

    #def _step_callback(self):
    #    if self.block_gripper:
    #        self.sim.data.set_joint_qpos('robot0:l_gripper_finger_joint', 0.)
    #        self.sim.data.set_joint_qpos('robot0:r_gripper_finger_joint', 0.)
    #        self.sim.forward()

    def _set_action(self, action):
        assert action.shape == (4,)
        action = action.copy()  # ensure that we don't change the action outside of this scope

        # Pos Control
        if len(action["left"]) >0:
            self.sim.data.ctrl[7:14,0] = np.array(action["left"])
        if len(action["right"])>0:
            self.sim.data.ctrl[:7,0] = np.array(action["right"] )

        # Gripper Control

        # Apply action to simulation.
        #utils.ctrl_set_action(self.sim, action)
        #utils.mocap_set_action(self.sim, action)

    def _get_obs(self):
        # positions
        #grip_pos = self.sim.data.get_site_xpos('robot0:grip')
        #dt = self.sim.nsubsteps * self.sim.model.opt.timestep
        #grip_velp = self.sim.data.get_site_xvelp('robot0:grip') * dt
        #robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
        #if self.has_object:
        #    object_pos = self.sim.data.get_site_xpos('object0')
        #    # rotations
        #    object_rot = rotations.mat2euler(self.sim.data.get_site_xmat('object0'))
        #    # velocities
        #    object_velp = self.sim.data.get_site_xvelp('object0') * dt
        #    object_velr = self.sim.data.get_site_xvelr('object0') * dt
        #    # gripper state
        #    object_rel_pos = object_pos - grip_pos
        #    object_velp -= grip_velp
        #else:
        #    object_pos = object_rot = object_velp = object_velr = object_rel_pos = np.zeros(0)
        #gripper_state = robot_qpos[-2:]
        #gripper_vel = robot_qvel[-2:] * dt  # change to a scalar if the gripper is made symmetric
        #
        #if not self.has_object:
        #    achieved_goal = grip_pos.copy()
        #else:
        #    achieved_goal = np.squeeze(object_pos.copy())
        object_pos = self.sim.data.get_site_xpos('box')
        achieved_goal = np.squeeze(object_pos.copy())
        obs = np.concatenate([self.sim.data.qpos.flat[2:],
                              self.sim.data.qvel.flat,
                              np.clip(self.sim.data.cfrc_ext, -1, 1).flat,
                             ])

        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy(),
        }

    def _viewer_setup(self):
        self.viewer.cam.distance = self.sim.model.stat.extent * 0.5
        self.viewer.cam.distance = 2.5
        self.viewer.cam.azimuth = 132.
        self.viewer.cam.elevation = -14.

    def _render_callback(self):
        # Visualize target.
        sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
        site_id = self.sim.model.site_name2id('target0')
        self.sim.model.site_pos[site_id] = self.goal - sites_offset[0]
        self.sim.forward()

    def _reset_sim(self):
        self.sim.set_state(self.initial_state)

        # Randomize start position of object.
        self.sim.forward()
        return True

    def _sample_goal(self):
        goal = self.np_random.uniform(-0.15, 0.15, size=3)
        return goal.copy()

    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)

    def _env_setup(self, initial_qpos):
        for name, value in initial_qpos.items():
            self.sim.data.set_joint_qpos(name, value)
        #utils.reset_mocap_welds(self.sim)
        self.sim.forward()

        # Move end effector into position.
        for _ in range(10):
            self.sim.step()

        # Extract information for sampling goals.

    # BatchEnv methods
    # ----------------------------
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

    def close_gripper(self, left_gap=0, right_gap=0):
        pass

    def get_body_com(self, body_name):
        idx = self.sim.model.body_names.index(six.b(body_name))
        return self.sim.data.com_subtree[idx]

    def get_idx_by_name(self, body_name):
        return self.sim.model.body_names.index(six.b(body_name))

    def set_cam_position(self, cam_pos):
        for i in range(3):
            self.viewer.cam.lookat[i] = cam_pos[i]
        self.viewer.cam.distance = cam_pos[3]
        self.viewer.cam.elevation = cam_pos[4]
        self.viewer.cam.azimuth = cam_pos[5]
        self.viewer.cam.trackbodyid = -1


# TESTING THE ENV LOADING
if __name__ == "__main__":
    initial_qpos = {"right_s0" : 0.4049,
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
                    "left_w2"  : 0.4
    }
    env = BaxterEnv("/home/brhm/DUC/RobotArm/src/my_arm/3rd/mujoco/python/gym-baxter/gym_baxter/envs/assets/baxter.xml", 20,0.05,
                    initial_qpos, 'sparse')

    print(env.sim.data.site_xpos[3])
    i=0
    while True:
        env.sim.step()
        i += 1

        ## left hand only
        target = env.sim.data.site_xpos[3].copy() + np.array([0.0, 0.0, 0.05])
        new_pose = env.do_ik(ee_target= target, jt_pos = env.sim.data.qpos[12:19])
        print(new_pose.T)
        # set_trace()
        env._set_action(action={"right": [], "left": new_pose})

        print(env.sim.data.ctrl)
        env.reset()
        # set_trace()

        env.viewer.render()
