#Reference: https://github.com/openai/baselines/tree/master/baselines/ppo2 (GPU-enabled PPO, compared to PPO1)

#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print('CURRENT DIR:', currentdir)
parentdir = os.path.dirname(currentdir)
print('PARENT DIR:', parentdir)
os.sys.path.insert(0, parentdir)

import gym
from RobotOperationObjSupportEnv import RobotOperationEnvironment
import robotCommon as RC

from gym import utils, spaces
from gym.utils import seeding
from std_srvs.srv import Empty

#from baselines import deepq

import time
import datetime
import numpy as np
import random
import argparse
import tensorflow as tf
#from keras.engine.training import collect_trainable_weights
import json

# PPO2
from baselines.common import set_global_seeds
from baselines.common.vec_env.vec_normalize import VecNormalize
from ppo.ppo2 import ppo2
from ppo.ppo2.policies import MlpPolicy
from baselines import bench, logger

from baselines.common.vec_env.dummy_vec_env import DummyVecEnv
import timeit

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

##############################################################################################################################################################
##############################################################################################################################################################
def startTrainingPPO2(num_timesteps, seed):
    # Create the environment
    print('START ENV', RC.GB_CLIENT_ID(), RC.gbRobotHandle())
    env = RobotOperationEnvironment(RC.GB_CLIENT_ID(), RC.GB_CSERVER_ROBOT_ID, RC.gbRobotHandle())
    #env = bench.Monitor(env, logger.get_dir())

    ncpu = 1
    config = tf.ConfigProto(allow_soft_placement=True,
                            intra_op_parallelism_threads=ncpu,
                            inter_op_parallelism_threads=ncpu)
    tf.Session(config=config).__enter__()
    #env = DummyVecEnv(env)
    #env = VecNormalize(env)

    set_global_seeds(seed)
    policy = MlpPolicy
    ppo2.learn(policy=policy, env=env, nsteps=128, nminibatches=32,
        lam=0.95, gamma=0.99, noptepochs=10, log_interval=1,
        ent_coef=0.0,
        lr=3e-4,
        cliprange=0.2,
        total_timesteps=num_timesteps, save_interval=1)

if __name__ == '__main__':
    RC.initialize_vrep()
    logger.configure()
    startTrainingPPO2(10000, np.random.seed(1337))
    RC.finalize_vrep()
