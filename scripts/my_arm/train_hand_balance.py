#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print('CURRENT DIR:', currentdir)
parentdir = os.path.dirname(currentdir)
print('PARENT DIR:', parentdir)
os.sys.path.insert(0, parentdir)

import gym
from hand_balance_env import HandBalanceEnv

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding

import rospy
import roslaunch
import time
import datetime

import numpy as np
import random
import argparse
from keras.models import model_from_json, Model
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.optimizers import Adam
import tensorflow as tf
#from keras.engine.training import collect_trainable_weights
import json

# DDPG
from ddpg.ReplayBuffer import ReplayBuffer
from ddpg.ActorNetwork import ActorNetwork
from ddpg.CriticNetwork import CriticNetwork
from ddpg.OU import OU
import timeit

from shadow_hand_agent import GB_HAND_JOINT_NAMES

OU = OU()       #Ornstein-Uhlenbeck Process

def callback(lcl, glb):
    # stop training if reward exceeds 199
    total = sum(lcl['episode_rewards'][-101:-1]) / 100
    totalt = lcl['t']
    is_solved = totalt > 2000 and total >= -50
    return is_solved

def startTraining(train_indicator=0):    #1 means Train, 0 means simply Run
    BUFFER_SIZE = 100000
    BATCH_SIZE = 32
    GAMMA = 0.99
    TAU = 0.001     #Target Network HyperParameters
    LRA = 0.0001    #Learning rate for Actor
    LRC = 0.001     #Lerning rate for Critic

    action_dim = 6  # 6 hand eigentgrasps
    # Each contacting scenario consists of hand and plate state (well enough to be used as Environment Observation)
    state_dim  = 13  # Hand_Info(6 hand eigentgrasps) + Plate_info(plate position + orientation)

    np.random.seed(1337)

    vision = False

    EXPLORE = 100000.
    episode_count = 2000
    max_steps = 100000
    reward = 0
    done = False
    step = 0
    epsilon = 1
    indicator = 0

    #Tensorflow GPU optimization
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    sess = tf.Session(config=config)
    from keras import backend as K
    K.set_session(sess)

    actor = ActorNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRA)
    critic = CriticNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRC)
    buff = ReplayBuffer(BUFFER_SIZE)    #Create replay buffer

    env = HandBalanceEnv()

    ## ---------------------------------------------------------------

    #Now load the weight
    print("Now we load the weight")
    try:
        actor.model.load_weights("actormodel.h5")
        critic.model.load_weights("criticmodel.h5")
        actor.target_model.load_weights("actormodel.h5")
        critic.target_model.load_weights("criticmodel.h5")
        print("Weight load successfully")
    except:
        print("Cannot find the weight")

    print("Hand Object Balancing Experiment Start.")
    for episode in range(episode_count):

        print("Episode : " + str(episode) + " Replay Buffer " + str(buff.count()))

        ob = env.reset()

        #s_t = np.reshape(ob, (-1, action_dim))
        s_t = np.hstack((ob.amps0, ob.amps1, ob.amps2, ob.amps3, ob.amps4, ob.amps5,
                         ob.plate_posX, ob.plate_posY, ob.plate_posZ,
                         ob.plate_ornX, ob.plate_ornY, ob.plate_ornZ, ob.plate_ornW))
        #print('OB', s_t)

        total_reward = 0.
        for j in range(max_steps):
            loss = 0
            epsilon -= 1.0 / EXPLORE
            a_t = np.zeros([1,action_dim])
            noise_t = np.zeros([1,action_dim])

            #print("ST RESHAPE", s_t.reshape(1, s_t.shape[0]), s_t.shape[0])
            a_t_original = actor.model.predict(np.reshape(s_t, (1, s_t.shape[0])))

            #print("a_t", a_t)
            #print("noise_t", noise_t)
            #print("a_t_original", a_t_original)
            for i in range(action_dim):
                noise_t[0][i] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][i],  0.0 , 0.60, 0.30)

            #The following code do the stochastic brake
            #if random.random() <= 0.1:
            #    print("********Now we apply the brake***********")
            #    noise_t[0][2] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][2],  0.2 , 1.00, 0.10)

            for i in range(action_dim):
                a_t[0][i] = a_t_original[0][i] + noise_t[0][i]
            ob, r_t, done, info = env.step(a_t[0])

            s_t1 = np.hstack((ob.amps0, ob.amps1, ob.amps2, ob.amps3, ob.amps4, ob.amps5,
                              ob.plate_posX, ob.plate_posY, ob.plate_posZ,
                              ob.plate_ornX, ob.plate_ornY, ob.plate_ornZ, ob.plate_ornW))
            #print('OB reshape', s_t1)

            buff.add(s_t, a_t[0], r_t, s_t1, done)      #Add replay buffer

            #Do the batch update
            batch = buff.getBatch(BATCH_SIZE)

            states = np.asarray([e[0] for e in batch])
            actions = np.asarray([e[1] for e in batch])
            rewards = np.asarray([e[2] for e in batch])
            new_states = np.asarray([e[3] for e in batch])
            #print('New State:', new_states)
            dones = np.asarray([e[4] for e in batch])
            y_t = np.asarray([e[1] for e in batch])

            target_q_values = critic.target_model.predict([new_states, actor.target_model.predict(new_states)])
            #print('target_q_values:', target_q_values)
            #print('batch:', len(batch))
            for k in range(len(batch)):
                if dones[k]:
                    y_t[k] = rewards[k]
                else:
                    y_t[k] = rewards[k] + GAMMA*target_q_values[k]

            if (train_indicator):
                loss += critic.model.train_on_batch([states,actions], y_t)
                a_for_grad = actor.model.predict(states)
                grads = critic.gradients(states, a_for_grad)
                actor.train(states, grads)
                actor.target_train()
                critic.target_train()

            total_reward += r_t
            s_t = s_t1

            print("Episode", episode, "Step", step, "Action", a_t, "Reward", r_t, "Loss", loss)

            step += 1
            if done:
                break

        print ('A:',episode, np.mod(episode, 3))
        if np.mod(episode, 3) == 0:
            if (train_indicator):
                print("Now we save the model and its weights")
                actor.model.save_weights("actormodel.h5", overwrite=True)
                with open("actormodel.json", "w") as outfile:
                    json.dump(actor.model.to_json(), outfile)

                critic.model.save_weights("criticmodel.h5", overwrite=True)
                with open("criticmodel.json", "w") as outfile:
                    json.dump(critic.model.to_json(), outfile)

        print("TOTAL REWARD @ " + str(episode) +"-th Episode  : Reward " + str(total_reward))
        print("Total Step: " + str(step))
        print("")

    #env.end()  # This is for shutting down GAZEBO
    #rospy.spin()
    print("Finish.")

if __name__ == "__main__":
    startTraining(1)

# http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
# rate = rospy.Rate(10) # 10hz
# while not rospy.is_shutdown():
#     hello_str = "hello world %s" % rospy.get_time()
#     rospy.loginfo(hello_str)
#     pub.publish(hello_str)
#     rate.sleep()

# The most common usage patterns for testing for shutdown in rospy are:
#
# while not rospy.is_shutdown():
#    do some work
#
# and
#
# ... setup callbacks
# rospy.spin()
#
# The spin() code simply sleeps until the is_shutdown() flag is True. It is mainly used to prevent your Python Main thread from exiting.
#
# There are multiple ways in which a node can receive a shutdown request, so it is important that you use one of the two methods above for ensuring your program terminates properly.

