import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
from gazebo_env import GazeboEnv
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding

from shadow_hand_agent import ShadowHandAgent

class HandBalanceEnv(GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        launchFileName = "sr_right_ur10arm_hand.launch"
        rosPackName = "sr_robot_launch"
        rosNodeName = "hand_balance_env"
        GazeboEnv.__init__(self, rosPackName, launchFileName, rosNodeName)

        self._timeStep = 0.01
        self._actionRepeat = 1
        self._isEnableSelfCollision = True
        self._observation = []
        self._envStepCounter = 0

        self._hand = ShadowHandAgent()
        action_dim = self._hand.getActionDimension()
        self._action_bound = 1
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Box(-action_high, action_high)
        # observation_high = np.array([np.finfo(np.float32).max] * observationDim)
        observationDim = action_dim + 6 #len(self.getObservation())
        observation_high = np.ones(observationDim) * 1000 #np.inf
        self.observation_space = spaces.Box(-observation_high, observation_high)
        #self.viewer = None
        self.reward_range = (-np.inf, np.inf)

        self._seed()
        print('Env inited!!!')

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _reset(self):

        # WE DO NOT NEED TO RESET THE AGENT, WHICH WILL RESET THE WHOLE ROS and GAZEBO SERVICE
        # ONLY THE PLATE MODEL IS RESET
        #self._hand = ShadowHandAgent()
        self._hand.reset()
        self._envStepCounter = 0
        self._observation = self.getObservation()

        return self._observation

    def getObservation(self):
        self._observation = self._hand.getObservation()
        return self._observation

    def _step(self, action):
        print('ACTION:', action)
        self._hand.applyAction(action)
        for i in range(self._actionRepeat):
            time.sleep(self._timeStep)
            self._observation = self.getObservation()

            if self._termination():
                print('Step Counter:', self._envStepCounter)
                break
            self._envStepCounter += 1
        reward = self._reward()
        done = self._termination()
        #print("len=%r" % len(self._observation))

        return self._observation, reward, done, {}

        """
        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.3
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = 0.3
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = -0.3
            self.vel_pub.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except rospy.ServiceException as e:
            print ("/gazebo/pause_physics service call failed")

        state,done = self.discretize_observation(data,5)

        if not done:
            if action == 0:
                reward = 5
            else:
                reward = 1
        else:
            reward = -200

        return state, reward, done, {}
        """

    def _termination(self):
        return self._hand.isPlateOnGround()

    def _reward(self):
        return self._hand.getReward()
