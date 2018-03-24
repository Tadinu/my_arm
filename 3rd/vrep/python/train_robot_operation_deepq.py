#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print('CURRENT DIR:', currentdir)
parentdir = os.path.dirname(currentdir)
print('PARENT DIR:', parentdir)
os.sys.path.insert(0, parentdir)

import gym
from RobotOperationEnv import RobotOperationEnvironment
import robotCommon as RC
import time
import datetime

import itertools
import numpy as np
import tensorflow as tf
import tensorflow.contrib.layers as layers

import baselines.common.tf_util as U

from baselines import logger
from baselines import deepq
from baselines.deepq.replay_buffer import ReplayBuffer
from baselines.deepq.utils import BatchInput
from baselines.common.schedules import LinearSchedule

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

def model(inpt, num_actions, scope, reuse=False):
    """This model takes as input an observation and returns values of all actions."""
    with tf.variable_scope(scope, reuse=reuse):
        out = inpt
        out = layers.fully_connected(out, num_outputs=64, activation_fn=tf.nn.tanh)
        out = layers.fully_connected(out, num_outputs=num_actions, activation_fn=None)
        return out

def startTraining():
    # Create the environment
    print('START ENV', RC.GB_CLIENT_ID(), RC.gbRobotHandle())
    env = RobotOperationEnvironment(RC.GB_CLIENT_ID(), RC.GB_CSERVER_ROBOT_ID, RC.gbRobotHandle())
    #print('ACTION_SPACE', env.action_space.shape)
    # Create all the functions necessary to train the model
    act, train, update_target, debug = deepq.build_train(
        make_obs_ph=lambda name: BatchInput(env.observation_space.shape, name=name),
        q_func=model,
        num_actions=env.action_space.n,
        optimizer=tf.train.AdamOptimizer(learning_rate=5e-4),
    )
    # Create the replay buffer
    replay_buffer = ReplayBuffer(50000)
    # Create the schedule for exploration starting from 1 (every action is random) down to
    # 0.02 (98% of actions are selected according to values predicted by the model).
    exploration = LinearSchedule(schedule_timesteps=10000, initial_p=1.0, final_p=0.02)

    # Initialize the parameters and copy them to the target network.
    U.initialize()
    update_target()

    episode_rewards = [0.0]
    obs = env.reset()
    print("Manipulator DEEPQ Training Experiment Start.")
    for t in itertools.count():
        print('Episode ', len(episode_rewards), 'Step ', t,'--------------')
        print('Start waiting for the next action', env._robot.getOperationState())
        while(env._robot.getOperationState() != RC.CROBOT_STATE_READY):
            time.sleep(0.01)

        # Take action and update exploration to the newest value
        action = act(obs[None], update_eps=exploration.value(t))[0]
        print('Generated action:', action)
        new_obs, rew, done, _ = env.step(action)
        # Store transition in the replay buffer.
        replay_buffer.add(obs, action, rew, new_obs, float(done))
        obs = new_obs

        episode_rewards[-1] += rew
        if done:
            obs = env.reset()
            episode_rewards.append(0)

        is_solved = t > 100 and np.mean(episode_rewards[-101:-1]) >= 200
        if is_solved:
            # Show off the result
            #env.render()
            pass
        else:
            # Minimize the error in Bellman's equation on a batch sampled from replay buffer.
            if t > 1000:
                obses_t, actions, rewards, obses_tp1, dones = replay_buffer.sample(32)
                print('Generated actions:', actions)
                train(obses_t, actions, rewards, obses_tp1, dones, np.ones_like(rewards))
            # Update target network periodically.
            if t % 1000 == 0:
                update_target()

        if done and len(episode_rewards) % 10 == 0:
            logger.record_tabular("steps", t)
            logger.record_tabular("episodes", len(episode_rewards))
            logger.record_tabular("mean episode reward", round(np.mean(episode_rewards[-101:-1]), 1))
            logger.record_tabular("% time spent exploring", int(100 * exploration.value(t)))
            logger.dump_tabular()

if __name__ == '__main__':
    RC.initialize_vrep()
    with U.make_session(8):
        startTraining()
    RC.finalize_vrep()
