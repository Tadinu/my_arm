import gym
from envs.kukaFallingObjsGymEnv import KukaFallingObjsGymEnv

from baselines import deepq

import datetime

import pybullet as p

def callback(lcl, glb):
    # stop training if reward exceeds 199
    total = sum(lcl['episode_rewards'][-101:-1]) / 100
    totalt = lcl['t']
    #print("totalt")
    #print(totalt)
    is_solved = totalt > 2000 and total >= 10
    return is_solved

def main():

    env = KukaFallingObjsGymEnv(renders=True)
    env.mainRobotTraining()
    #model = deepq.models.mlp([64])
    #act = deepq.learn(
    #    env,
    #    q_func=model,
    #    lr=1e-3,
    #    max_timesteps=10000000,
    #    buffer_size=50000,
    #    exploration_fraction=0.1,
    #    exploration_final_eps=0.02,
    #    print_freq=10,
    #    callback=callback
    #)
    ##print("Saving model to kuka_model.pkl")
    act.save("kuka_model.pkl")


if __name__ == '__main__':
    main()
