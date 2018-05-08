#!/usr/bin/env python3

import math
import os
import copy
import numpy as np

import rotations, utils
from gym.utils import seeding

try:
    import mujoco_py
except ImportError as e:
    raise error.DependencyNotInstalled("{}. (HINT: you need to install mujoco_py, and also perform the setup instructions here: https://github.com/openai/mujoco-py/.)".format(e))

class Robot(object):
    """Superclass for all mujoco robots.
    """
    def __init__(self, model_path, initial_qpos, n_substeps):
        if model_path.startswith('/'):
            fullpath = model_path
        else:
            fullpath = os.path.join(os.path.dirname(__file__), 'xmls', model_path)
        if not os.path.exists(fullpath):
            raise IOError('File {} does not exist'.format(fullpath))

        self.model = mujoco_py.load_model_from_path(fullpath)
        # ------------------------------------------------------------------------
        #self.model = load_model_from_xml(MODEL_XML)
        self.sim    = mujoco_py.MjSim(self.model, nsubsteps=n_substeps)
        self.viewer = mujoco_py.MjViewer(self.sim)
        self.count = 0
        self.initial_qpos = initial_qpos

        self.metadata = {
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second': int(np.round(1.0 / self.dt))
        }
        self.seed()
        self.initial_state = copy.deepcopy(self.sim.get_state())

        # ------------------------------------------------------------------------
        self.setup_viewer()
        self.initialize()

    @property
    def dt(self):
        return self.sim.model.opt.timestep * self.sim.nsubsteps

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # ==========================================================================
    # SIMULATOR METHODS ========================================================
    #
    def setup_viewer(self):
        pass

    def set_cam_position(self, cam_pos):
        for i in range(3):
            self.viewer.cam.lookat[i] = cam_pos[i]
        self.viewer.cam.distance = cam_pos[3]
        self.viewer.cam.elevation = cam_pos[4]
        self.viewer.cam.azimuth = cam_pos[5]
        self.viewer.cam.trackbodyid = -1

    def get_site_id(self,site_name):
        self.sim.model.site_name2id(site_name)
        #self.sim.model.site_pos[site_id] =
        return self.sim.model.site_name2id(site_name)

    def reset_sim(self):
        self.sim.set_state(self.initial_state)
        self.sim.forward()
        return True

    # ==========================================================================
    # ROBOT METHODS ============================================================
    #
    def initialize(self):
        pass

    def set_object_joint(self, joint_name, q_pos):
        self.sim.data.set_joint_qpos(joint_name, q_pos)

    def act(self, action):
        """ Order robot act
        """
        pass
