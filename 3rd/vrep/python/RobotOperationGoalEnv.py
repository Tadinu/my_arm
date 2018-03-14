import os
import copy
import math
import numpy as np
import random
import time
from time import sleep
import collections as col
import threading
from threading import Timer
## threading.Timer(1, self.timerTask).start()

import gym
from gym import error, spaces
from gym.utils import seeding

from robot import Robot
from robotBot import RobotBot
from menace_object import MenaceObject
from repeated_timer import RepeatedTimer

from RobotOperationEnv import RobotOperationEnvironment
import robotCommon as RC

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

# Refer to from gym.envs.robotics import fetch_env
def goal_distance(goal_a, goal_b):
    #assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=0)

# Refer to from gym.envs.robotics import robot_env
class RobotOperationGoalEnvironment(RobotOperationEnvironment):

    def __init__(self, clientID, robotId, robotHandle):
        self.seed()
        super(RobotOperationGoalEnvironment, self).__init__(clientID, robotId, robotHandle)

        #self.initial_state = copy.deepcopy(self.sim.get_state())

        self.goal = self._sample_goal()
        obs = self._get_obs()
        self.action_space = spaces.Box(-1., 1., shape=(RC.GB_ACTION_DIM,), dtype='float32')

        self.observation_space = spaces.Dict(dict(
            desired_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            achieved_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            observation=spaces.Box(-np.inf, np.inf, shape=obs['observation'].shape, dtype='float32'),
        ))

        self._max_episode_steps = 50000
        self.distance_threshold = 0.05
        self.reward_type='sparse'

    #@property
    #def dt(self):
    #    return self.sim.model.opt.timestep * self.sim.nsubsteps

    # Env methods
    # ----------------------------

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        #action = np.clip(action, self.action_space.low, self.action_space.high)
        obs, reward, done, note = super().step(action)

        #self.sim.step()
        self._step_callback()

        obs = self._get_obs()
        info = {
            'is_success': self._is_success(obs['achieved_goal'], self.goal),
        }
        reward = self.compute_reward(obs['achieved_goal'], self.goal, info)
        return obs, reward, done, info

    def reset(self):
        super().reset()

        self.goal = self._sample_goal().copy()
        obs = self._get_obs()
        return obs

    def close(self):
        return None

    def _get_viewer(self):
        return None

    # Extension methods
    # ----------------------------

    def _reset_sim(self):
        return True

    def _get_obs(self):
        # positions
        '''
        grip_pos = self.sim.data.get_site_xpos('robot0:grip')
        dt = self.sim.nsubsteps * self.sim.model.opt.timestep
        grip_velp = self.sim.data.get_site_xvelp('robot0:grip') * dt
        robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
        if self.has_object:
            object_pos = self.sim.data.get_site_xpos('object0')
            # rotations
            object_rot = rotations.mat2euler(self.sim.data.get_site_xmat('object0'))
            # velocities
            object_velp = self.sim.data.get_site_xvelp('object0') * dt
            object_velr = self.sim.data.get_site_xvelr('object0') * dt
            # gripper state
            object_rel_pos = object_pos - grip_pos
            object_velp -= grip_velp
        else:
            object_pos = object_rot = object_velp = object_velr = object_rel_pos = np.zeros(0)
        gripper_state = robot_qpos[-2:]
        gripper_vel = robot_qvel[-2:] * dt  # change to a scalar if the gripper is made symmetric

        if not self.has_object:
            achieved_goal = grip_pos.copy()
        else:
            achieved_goal = np.squeeze(object_pos.copy())
        obs = np.concatenate([
            grip_pos, object_pos.ravel(), object_rel_pos.ravel(), gripper_state, object_rot.ravel(),
            object_velp.ravel(), object_velr.ravel(), grip_velp, gripper_vel,
        ])
        '''

        #currentJointPos = self._robot.getConcernedJointsCurrentPos()
        basePlateNormalVector     = self.getBasePlateNormalVector()
        maxBasePlateSlantingAngle = self.getMaxBasePlateSlantingAngle()
        basePlateSlantingAngle = abs(RC.angle_between(np.array([0,0,1]), np.array(self.getBasePlateNormalVector())))
        if(self.isObjAwayFromBasePlate()):
            achieved_goal = np.array(maxBasePlateSlantingAngle, dtype=np.float32)
        else:
            achieved_goal = np.array(basePlateSlantingAngle, dtype=np.float32)

        ob = super().getObservation([0]*RC.GB_ACTION_DIM)
        obs = np.concatenate([ob[0], ob[1] , ob[2], ob[3], ob[4]])
        return {
            'observation'  : obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal' : self.goal.copy(),
        }

    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)

    def _sample_goal(self):
        if(self.isObjAwayFromBasePlate()):
            goal = np.array([0.05] + self.np_random.uniform(0, math.pi/3, size=1), dtype=np.float32)
        else:
            goal = np.array([0.05], dtype=np.float32)

        return goal.copy()

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.
        """
        pass

    def _viewer_setup(self):
        """Initial configuration of the viewer. Can be used to set the camera position,
        for example.
        """
        pass

    def _render_callback(self):
        """A custom callback that is called before rendering. Can be used
        to implement custom visualizations.
        """
        pass

    def _step_callback(self):
        """A custom callback that is called after stepping the simulation. Can be used
        to enforce additional constraints on the simulation state.
        """
        pass

    # GoalEnv methods
    # ----------------------------

    def compute_reward(self, achieved_goal, goal, info):
        # Compute distance between goal and the achieved goal.
        d = goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            return -d
