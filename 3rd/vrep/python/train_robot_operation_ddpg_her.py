#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print('CURRENT DIR:', currentdir)
parentdir = os.path.dirname(currentdir)
print('PARENT DIR:', parentdir)
os.sys.path.insert(0, parentdir)

import sys
import gym
from RobotOperationGoalEnv import RobotOperationGoalEnvironment
import robotCommon as RC

from gym import utils, spaces
from gym.utils import seeding
from std_srvs.srv import Empty

#from baselines import deepq

import time
import timeit
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

# DDPG + HER
#from ddpg.ReplayBuffer import ReplayBuffer
#from ddpg.ActorNetwork import ActorNetwork
#from ddpg.CriticNetwork import CriticNetwork
#from ddpg.OU import OU
import click
from mpi4py import MPI

from baselines import logger
from baselines.common import set_global_seeds
from baselines.common.mpi_moments import mpi_moments
import her.experiment.config as config
from her.rollout import RolloutWorker
from her.util import mpi_fork


# MATPLOT
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

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

CSERVER_PORT = 19999
##############################################################################################################################################################
##############################################################################################################################################################

def initialize_vrep():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections

    global gbClientID
    gbClientID=vrep.simxStart('127.0.0.1', CSERVER_PORT,True,True,5000,5) # Connect to V-REP
    if gbClientID!=-1:
        print ('Connected to remote API server',gbClientID)

        # Init Robot Common
        RC.init(gbClientID)

        # Start the simulation:
        RC.startSimulation(gbClientID)

        # Load a robot instance:    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'loadRobot',[],[0,0,0,0],['d:/v_rep/qrelease/release/test.ttm'],emptyBuff,vrep.simx_opmode_oneshot_wait)
        #    robotHandle=retInts[0]

        # Get scene objects data
        res, objHandles, intData, floatData, objNames = vrep.simxGetObjectGroupData(gbClientID,vrep.sim_appobj_object_type, 0, vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('Number of objects in the scene: ',len(objHandles), len(objNames))
            for i in range(len(objHandles)):
                print('Obj:', objHandles[i], objNames[i])
        else:
            print ('Remote API function call returned with error code: ',res)

        # Retrieve some handles:
        global gbRobotHandle
        res, gbRobotHandle = vrep.simxGetObjectHandle(gbClientID, RC.GB_CSERVER_ROBOT_NAME, vrep.simx_opmode_oneshot_wait)

def mpi_average(value):
    if value == []:
        value = [0.]
    if not isinstance(value, list):
        value = [value]
    return mpi_moments(np.array(value))[0]


def train(policy, rollout_worker, evaluator,
          n_epochs, n_test_rollouts, n_cycles, n_batches, policy_save_interval,
          save_policies, **kwargs):
    rank = MPI.COMM_WORLD.Get_rank()

    latest_policy_path = os.path.join(logger.get_dir(), 'policy_latest.pkl')
    best_policy_path = os.path.join(logger.get_dir(), 'policy_best.pkl')
    periodic_policy_path = os.path.join(logger.get_dir(), 'policy_{}.pkl')

    logger.info("Training...: ", n_epochs, ' epochs -',  n_cycles, ' cycles -', n_batches, ' batches')
    best_success_rate = -1
    for epoch in range(n_epochs):
        # train
        rollout_worker.clear_history()
        for _ in range(n_cycles):
            episode = rollout_worker.generate_rollouts() # Run steps here!
            policy.store_episode(episode)
            for _ in range(n_batches):
                policy.train()
            policy.update_target_net()

        # test
        evaluator.clear_history()
        for _ in range(n_test_rollouts):
            evaluator.generate_rollouts()

        # record logs
        logger.record_tabular('epoch', epoch)
        for key, val in evaluator.logs('test'):
            logger.record_tabular(key, mpi_average(val))
        for key, val in rollout_worker.logs('train'):
            logger.record_tabular(key, mpi_average(val))
        for key, val in policy.logs():
            logger.record_tabular(key, mpi_average(val))

        if rank == 0:
            logger.dump_tabular()

        # save the policy if it's better than the previous ones
        success_rate = mpi_average(evaluator.current_success_rate())
        if rank == 0 and success_rate >= best_success_rate and save_policies:
            best_success_rate = success_rate
            logger.info('New best success rate: {}. Saving policy to {} ...'.format(best_success_rate, best_policy_path))
            evaluator.save_policy(best_policy_path)
            evaluator.save_policy(latest_policy_path)
        if rank == 0 and policy_save_interval > 0 and epoch % policy_save_interval == 0 and save_policies:
            policy_path = periodic_policy_path.format(epoch)
            logger.info('Saving periodic policy to {} ...'.format(policy_path))
            evaluator.save_policy(policy_path)

        # make sure that different threads have different seeds
        local_uniform = np.random.uniform(size=(1,))
        root_uniform = local_uniform.copy()
        MPI.COMM_WORLD.Bcast(root_uniform, root=0)
        if rank != 0:
            assert local_uniform[0] != root_uniform[0]

def startTraining(env_name, logdir, n_epochs, num_cpu, seed, replay_strategy, policy_save_interval, clip_return,
                  override_params={}, save_policies=True):

    print('START ENV', gbClientID, gbRobotHandle)
    env = RobotOperationGoalEnvironment(gbClientID, RC.GB_CSERVER_ROBOT_ID, gbRobotHandle)
    config.set_cached_env(env)
    #assert hasattr(config.CACHED_ENV, '_max_episode_steps')

    # ----------------------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------------------
    # Fork for multi-CPU MPI implementation.
    #
    if num_cpu > 1:
        whoami = mpi_fork(num_cpu)
        if whoami == 'parent':
            sys.exit(0)
        import baselines.common.tf_util as U
        U.single_threaded_session().__enter__()
    rank = MPI.COMM_WORLD.Get_rank()

    # Configure logging
    if rank == 0:
        if logdir or logger.get_dir() is None:
            logger.configure(dir=logdir)
    else:
        logger.configure()
    logdir = logger.get_dir()
    assert logdir is not None
    os.makedirs(logdir, exist_ok=True)

    # Seed everything.
    rank_seed = seed + 1000000 * rank
    set_global_seeds(rank_seed)

    # Prepare params.

    params = config.DEFAULT_PARAMS
    params['env_name'] = env_name
    params['replay_strategy'] = replay_strategy
    if env_name in config.DEFAULT_ENV_PARAMS:
        params.update(config.DEFAULT_ENV_PARAMS[env_name])  # merge env-specific parameters in
    params.update(**override_params)  # makes it possible to override any parameter
    with open(os.path.join(logger.get_dir(), 'params.json'), 'w') as f:
        json.dump(params, f)
    params = config.prepare_params(params)
    config.log_params(params, logger=logger)

    if num_cpu == 1:
        logger.warn()
        logger.warn('*** Warning ***')
        logger.warn(
            'You are running HER with just a single MPI worker. This will work, but the ' +
            'experiments that we report in Plappert et al. (2018, https://arxiv.org/abs/1802.09464) ' +
            'were obtained with --num_cpu 19. This makes a significant difference and if you ' +
            'are looking to reproduce those results, be aware of this. Please also refer to ' +
            'https://github.com/openai/baselines/issues/314 for further details.')
        logger.warn('****************')
        logger.warn()

    # Run step here!
    dims = config.configure_dims(params)
    # Create the DDPG with dims
    policy = config.configure_ddpg(dims=dims, params=params, clip_return=clip_return)

    rollout_params = {
        'exploit': False,
        'use_target_net': False,
        'use_demo_states': True,
        'compute_Q': False,
        'T': params['T'],
    }

    eval_params = {
        'exploit': True,
        'use_target_net': params['test_with_polyak'],
        'use_demo_states': False,
        'compute_Q': True,
        'T': params['T'],
    }

    for name in ['T', 'rollout_batch_size', 'gamma', 'noise_eps', 'random_eps']:
        rollout_params[name] = params[name]
        eval_params[name] = params[name]

    rollout_worker = RolloutWorker(env, policy, dims, logger, **rollout_params)
    rollout_worker.seed(rank_seed)

    evaluator = RolloutWorker(env, policy, dims, logger, **eval_params)
    evaluator.seed(rank_seed)

    train(
        logdir=logdir, policy=policy, rollout_worker=rollout_worker,
        evaluator=evaluator, n_epochs=n_epochs, n_test_rollouts=params['n_test_rollouts'],
        n_cycles=params['n_cycles'], n_batches=params['n_batches'],
        policy_save_interval=policy_save_interval, save_policies=save_policies)

if __name__ == '__main__':
    initialize_vrep()
    parser = argparse.ArgumentParser(description='provide arguments for DDPG+HER agent')

    parser.add_argument('--env_name', type=str, default='FetchReach-v0', help='the name of the OpenAI Gym environment that you want to train on')
    parser.add_argument('--logdir', type=str, default=None, help='the path to where logs and policy pickles should go. If not specified, creates a folder in /tmp/')
    parser.add_argument('--n_epochs', type=int, default=50, help='the number of training epochs to run')
    parser.add_argument('--num_cpu', type=int, default=1, help='the number of CPU cores to use (using MPI)')
    parser.add_argument('--seed', type=int, default=0, help='the random seed used to seed both the environment and the training code')
    parser.add_argument('--policy_save_interval', type=int, default=5, help='the interval with which policy pickles are saved. If set to 0, only the best and latest policy will be pickled.')
    parser.add_argument('--replay_strategy', type=click.Choice(['future', 'none']), default='future', help='the HER replay strategy to be used. "future" uses HER, "none" disables HER.')
    parser.add_argument('--clip_return', type=int, default=1, help='whether or not returns should be clipped')

    parser.set_defaults(render_env=False)
    parser.set_defaults(use_gym_monitor=True)

    args = vars(parser.parse_args())
    #pp.pprint(args)

    startTraining(args['env_name'],
                  args['logdir'],
                  args['n_epochs'],
                  args['num_cpu'],
                  args['seed'],
                  args['replay_strategy'],
                  args['policy_save_interval'],
                  args['clip_return'])
    finalize_vrep()
