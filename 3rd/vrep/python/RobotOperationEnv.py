import time
import math
import numpy as np
import random
import time
import collections as col
from threading import Timer
## threading.Timer(1, self.timerTask).start()

import gym
from gym import spaces
from gym.utils import seeding

from robot import Robot
from robotBot import RobotBot
from menace_object import MenaceObject
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

CSERVER_REMOTE_FUNC_SHOW_MESSAGE          = 'displayMessage'
CSERVER_REMOTE_FUNC_RELOAD_FALL_OBJS      = 'reloadFallingObjects'
CSERVER_REMOTE_FUNC_DETECT_OBJ_GROUND     = 'detectObjectsOnGround'

#CFALL_OBJS_NAMES = ['Obj1', 'Obj2', 'Obj3', 'Obj4', 'Obj5']
CFALL_OBJS_NAMES = ['Obj1']

class RobotOperationEnvironment(gym.Env):

    def __init__(self, clientID, robotId, robotHandle):
        self._clientID = clientID
        self._robotId  = robotId
        self._robotHandle = robotHandle

        self._timeStep = 1./240.
        self._observation = []
        self._envStepCounter = 0

        # INITIALIZE ENVIRONMENT
        self.loadEnvironmentObjects()

        # Reset
        self.reset() ## --> Call self._reset() defined  below!

        # After loading the robot!
        observationDim = len(self.getObservation())
        #print("observationDim")
        #print(observationDim)

        observation_high = np.array([np.finfo(np.float32).max] * observationDim)
        self.action_space = spaces.Discrete(7)
        self.observation_space = spaces.Box(-observation_high, observation_high)
        self.viewer = None

        ## DEBUG STUFFS --
        #self.initializeDebugShapes()

        #def initializeDebugShapes(self):
        #  self._sphere = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
        #  self._cube   = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1,0.1,0.1])
        #
        #def drawDebugShape(self, shapeId, pos):
        #  return p.createMultiBody(1, shapeId, -1, pos, [0,0,0,1])

    def loadEnvironmentObjects(self):
        ## FALLING OBJECTS --
        self._CNUM_OBJS = 1
        self._objs = []
        self._sphereUid = self._cubeUId = -1
        self.__reward = 0
        self._CKUKA_MOVE_INTERVAL = math.pi/6
        #self._timer = Timer(1, self.timerTask)

        ## SETUP ENVIRONMENT --
        ##
        ## LOAD MANIPULATOR KUKA --
        self._robot = Robot(self._clientID, self._robotId, self._robotHandle)

        # Initialize the bot
        self._robotBot = RobotBot(self._robotId)

    def _reset(self):
        print("_reset")
        self._terminated = 0

        ## LOAD FALLING OBJS --
        self.reloadFallingObjects()

        ## RESET ROBOT POS --
        #self._robot.resetRobotPos()

        ## STEP SIMULATION --
        self._envStepCounter = 0

        ## OBSERVATION --
        self._observation = self.getObservation()

        ## ducta--
        #self._timer.start()
        return self._observation

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def getObservation(self):
        self._observation = self._robot.getObservation()

        pos       = self.getObjectWorldPosition(CFALL_OBJS_NAMES[0])
        linearVel = self.getObjectVelocity(CFALL_OBJS_NAMES[0])

        print('OBJECT OBSERVED:', pos, linearVel)

        self._observation.append(np.array(pos[0], dtype=np.float32))
        self._observation.append(np.array(pos[1], dtype=np.float32))
        self._observation.append(np.array(pos[2], dtype=np.float32))

        self._observation.append(np.array(linearVel[0], dtype=np.float32))
        self._observation.append(np.array(linearVel[1], dtype=np.float32))
        self._observation.append(np.array(linearVel[2], dtype=np.float32))

        return self._observation

    def reloadFallingObjects(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = [53.21,17.39]
        inputStrings = CFALL_OBJS_NAMES
        inputBuffer  = bytearray()
        inputBuffer.append(78)
        inputBuffer.append(42)
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME,    \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_RELOAD_FALL_OBJS,              \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)

    #def printMotorNames(self):
        #motorsIds=[]
        #for i in range (len(self._kuka.motorNames)):
        #    motor = self._kuka.motorNames[i]
        #    motorJointIndex = self._kuka.motorIndices[i]
        #    motorsIds.append(self._p.addUserDebugParameter(motor,-3,3,self._kuka.jointPositions[i]))
        #    print(motor, '-', motorJointIndex)
        #    #print(motor, self._kuka.getJointInfo(motorJointIndex))
        #return 0

    #def __del__(self):

    def _step(self, action):
        ## ----------------------------------------------------------------------------------------
        print('ACTION:', action)
        self._robot.applyAction(action)
        time.sleep(self._timeStep)
        self._observation = self.getObservation()
        self._envStepCounter += 1
        #print("self._envStepCounter")
        #print(self._envStepCounter)

        done = self._termination()
        reward = self._reward()
        #print("len=%r" % len(self._observation))

        if(done):
            self._observation = self.getObservation()

        return self._observation, reward, done, {}

    def _termination(self):
        #print("self._envStepCounter")
        #print(self._envStepCounter)
        if (self._terminated or self._envStepCounter>1000):
            return True

        # Hit the object or object hit the ground
        if(self._robot.detectCollisionWith(CFALL_OBJS_NAMES[0]) or \
           self.detectObjectsReachGround()):
            return True

        return False

    def _reward(self):
        #print("reward")
        #print(__reward)

        objPos   = self.getObjectWorldPosition(CFALL_OBJS_NAMES[0])
        distance = self._robot.distanceFromEndTipToPos(objPos)
        print('DISTANCE:', distance)
        self.__reward = 1000 - distance

        return self.__reward

    def objectAvoidanceTraining(self):
        # self._objs
        #
        # 3.1 Objects Pos & Velocity
        # Compose Environment Info (State input)
        baseCurPos = self._robot.getCurrentBaseJointPos()
        print('BASE CUR POS:', baseCurPos)
        envInfo = [baseCurPos]
        objsPos = []
        for i in range(self._CNUM_OBJS):
            objInfo = []
            #print("Obj Ids: ", i, self._objs[i].id())
            objPos = self.getObjectWorldPosition(CFALL_OBJS_NAMES[i])
            objsPos.append(objPos)
            # This returns a list of two vector3 values (3 floats in a list) representing the linear velocity [x,y,z]
            # and angular velocity [wx,wy,wz] in Cartesian worldspace coordinates.
            objLinearVel = self.getObjectVelocity(CFALL_OBJS_NAMES[i])
            #print("OBJPS:",i, objPos)
            objInfo+= objPos
            #objInfo.append(objLinearVel[2]) # zVel only
            envInfo.append(objInfo)
            #print("OBJINFO:", objInfo)

        # 5. ACTION -------------------------------------------------------------------------
        #
        actionId = self._robotBot.act(envInfo)
        print("Robot Act: ", actionId)
        self._robot.actRobot(actionId)

        # 6. UPDATE OBJECTS ROBOT-HIT & GROUND-HIT STATUS
        for i in range(self._CNUM_OBJS):
            #print("CHECK KUKA COLLISION WITH BALL", CFALL_OBJS_NAMES[i])
            if(self._robot.detectCollisionWith(CFALL_OBJS_NAMES[i]) or \
               self.detectObjectsReachGround()):
                self._terminated = 1
                print('Terminate')
                break

    def objectCatchTraining(self):
        # self._objs
        #
        # 3.1 Objects Pos & Velocity
        # Compose Environment Info (State input)
        baseCurPos = self._robot.getCurrentBaseJointPos()
        print('BASE CUR POS:', baseCurPos)
        envInfo = [baseCurPos]
        objsPos = []
        for i in range(self._CNUM_OBJS):
            objInfo = []
            #print("Obj Ids: ", i, self._objs[i].id())
            objPos = self.getObjectWorldPosition(CFALL_OBJS_NAMES[i])
            objsPos.append(objPos)
            # This returns a list of two vector3 values (3 floats in a list) representing the linear velocity [x,y,z]
            # and angular velocity [wx,wy,wz] in Cartesian worldspace coordinates.
            objLinearVel = self.getObjectVelocity(CFALL_OBJS_NAMES[i])
            #print("OBJPS:",i, objPos)
            objInfo+= objPos
            #objInfo.append(objLinearVel[2]) # zVel only
            envInfo.append(objInfo)
            #print("OBJINFO:", objInfo)

        # 5. ACTION -------------------------------------------------------------------------
        #
        actionId = self._robotBot.act(envInfo)
        print("Robot Act: ", actionId)
        self._robot.actRobot(actionId)

        # 6. UPDATE OBJECTS ROBOT-HIT & GROUND-HIT STATUS
        for i in range(self._CNUM_OBJS):
            #print("CHECK KUKA COLLISION WITH BALL", CFALL_OBJS_NAMES[i])
            if(self._robot.detectCollisionWith(CFALL_OBJS_NAMES[i]) or \
               self.detectObjectsReachGround()):
                self._terminated = 1
                print('Terminate')

    def mainRobotTraining(self):
        score = 0

        #robotBaseJointLimit = self._kuka.getJointLimit(0)

        ## DEBUG ++
        #self.drawDebugShape(self._cubeUId, [1,1,1])
        #self.drawDebugShape(self._cubeUId, [0,0,0])
        ## DEBUG --

        run_count = 1
        print('RUN # 1 ---------------------------------')

        while True:
            if(self._robotId == RC.CYOUBOT):
                # OBJECTS AVOIDANCE ---------------------------------------------------------------
                self.objectAvoidanceTraining()
            else:
                # OBJECTS CATCH -------------------------------------------------------------------
                self.objectCatchTraining()

            if self._terminated: # Terminal state
                run_count += 1
                print('RUN #', run_count, '---------------------------------')
                # Update the q_values
                self._robotBot.update_qvalues()

                ## 0. RELOAD FALLING OBJS --
                self.reset()
                time.sleep(0.5)
                continue

    def getObjectWorldPosition(self, objectName):
        res, objectHandle = vrep.simxGetObjectHandle(self._clientID, objectName, vrep.simx_opmode_oneshot_wait)
        res, objectPos    =  vrep.simxGetObjectPosition(self._clientID, objectHandle, -1, vrep.simx_opmode_oneshot)
        return objectPos

    def getObjectVelocity(self, objectName):
        res, objectHandle = vrep.simxGetObjectHandle(self._clientID, objectName, vrep.simx_opmode_oneshot_wait)
        res, objectLinearVel, objectAngVel = vrep.simxGetObjectVelocity(self._clientID, objectHandle, vrep.simx_opmode_buffer)
        return objectLinearVel

    def showMessage(self, message):
        # Display a message:
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                                     vrep.sim_scripttype_childscript,          \
                                                                                     CSERVER_REMOTE_FUNC_SHOW_MESSAGE,         \
                                                                                     [], [], [message], inputBuffer,           \
                                                                                     vrep.simx_opmode_oneshot_wait)

    def detectObjectsReachGround(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = [53.21,17.39]
        inputStrings = []
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME,    \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_DETECT_OBJ_GROUND,             \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        return retInts[0]

    def showStatusBarMessage(self, message):
        vrep.simxAddStatusbarMessage(self._clientID, message, vrep.simx_opmode_oneshot)
