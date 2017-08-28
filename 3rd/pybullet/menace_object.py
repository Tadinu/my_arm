import json

import math
import pybullet as p
import numpy as np
import time
import random

class MenaceObject(object):
    '''
    The Object represents an object flying to the robot
    '''
    def __init__(self, env):
        self._env = env
        self.__id = -1
        self.__robotHit = False

    def id(self):
        return self.__id

    def load(self, urdfPath, posX, posY, posZ, ornX, ornY, ornZ, ornW):
        self.__id = p.loadURDF(urdfPath, posX, posY, posZ, ornX, ornY, ornZ, ornW)
        return self.__id

    def getBasePosAndOrn(self):
        return p.getBasePositionAndOrientation(self.__id)

    def isGroundHit(self):
        basePos, baseOrn = p.getBasePositionAndOrientation(self.__id)
        return basePos[2] < 0
        #closestPoints  = p.getClosestPoints(self.__id, self._env._groundUId, 1)
        #closestPoints2 = p.getClosestPoints(self.__id, self._env._tableUId, 1)
        #print( len(closestPoints) , len(closestPoints2))
        #if(len(closestPoints) > 0):
        #    print(closestPoints[0][8])
        #return len(closestPoints) > 0 or len(closestPoints2) > 0

    def robotHit(self):
        self.__robotHit = True

    def isRobotHit(self):
        return self.__robotHit

    def isReward(self):
        return (not self.isRobotHit()) and (self.isGroundHit())

    # Called from the env's timer to avoid create a timer for each object --> too many timers!
    #def timerTask(self):
        #print('TIMER TASK')
