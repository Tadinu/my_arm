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
from gym import spaces
from gym.utils import seeding

from robot import Robot
from robotBot import RobotBot
from menace_object import MenaceObject
from repeated_timer import RepeatedTimer
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

class RobotOperationEnvironment(gym.Env):

    def __init__(self, clientID, robotId, robotHandle):
        self._clientID = clientID
        self._robotId  = robotId
        self._robotHandle = robotHandle

        self._timeStep = 1./240.
        self._observation = []
        self._envStepCounter = 0
        self._mutex = threading.Lock()

        # INITIALIZE ENVIRONMENT
        self.loadEnvironmentObjects()

        # Reset
        self.reset() ## --> Call self._reset() defined  below!

        # After loading the robot!
        observationDim = len(self.getObservation()) ## !!NOTE: Don't use GB_STATE_DIM here due to the need of verification!
        print("observationDim", observationDim)

        observation_high = np.array([np.finfo(np.float32).max] * observationDim)
        self.action_space = spaces.Discrete(RC.GB_ACTION_DIM)
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

    def __del__(self):
        self._rt.stop()

    def loadEnvironmentObjects(self):
        ## FALLING OBJECTS --
        self._CNUM_OBJS = 1
        self._objs = []
        self._sphereUid = self._cubeUId = -1
        self.__reward = 0
        self._CKUKA_MOVE_INTERVAL = math.pi/6

        ## SETUP ENVIRONMENT --
        ##
        ## LOAD MANIPULATOR KUKA --
        self._robot = Robot(self._clientID, self._robotId, self._robotHandle)

        # Initialize the bot
        self._robotBot = RobotBot(self._robotId)

        # TIMER TASK
        self._timerInterval = 1
        self._rt = RepeatedTimer(self._timerInterval, self.doTimerTask) # it auto-starts, no need of rt.start()
        self._rt.start()

    def setTerminated(self, terminated):
        self._mutex.acquire()
        self._terminated = terminated
        self._mutex.release()

    def isTerminated(self):
        return self._terminated

    def _reset(self):
        if(RC.GB_TRACE):
            print("_reset")
        self.setTerminated(0)
        #print("Start back simulation")
        #RC.startSimulation(self._clientID)

        ## RESET ROBOT POS --
        self._robot.resetRobot()

        time.sleep(4)
        ## LOAD FALLING OBJS --
        self.reloadFallingObjects()

        ## STEP SIMULATION --
        self._envStepCounter = 0

        ## OBSERVATION --
        self._observation = self.getObservation()

        self._robot.commandJointVibration(1)

        ## ducta--
        #self._timer.start()
        return self._observation

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def doTimerTask(self):
        if(not self.isTerminated()):
            self._robot.randomJoints()

    def getObservation(self):
        self._observation = self._robot.getObservation()

        #robotId = self._robot.id()
        if(RC.isTaskObjBalance()):
            ##############################################################################################
            # PLATE INFO
            #
            # Plate Object
            # Angle rotation away from horizontal plane
            plateOrient = RC.getObjectOrientation(RC.CPLATE_OBJ_NAME) # Must be the same name set in V-Rep
            self._observation.append(np.array(abs(plateOrient[0]), dtype=np.float32))

            # Distance from plate to the center of end-tip
            platePos = RC.getObjectWorldPosition(RC.CPLATE_OBJ_NAME)
            palmPos  = self._robot.getEndTipWorldPosition()
            d = math.sqrt((platePos[0] - palmPos[0])**2 +
                          (platePos[1] - palmPos[1])**2)
            self._observation.append(np.array(d, dtype=np.float32))

        elif(RC.isTaskObjHold()):
            ##############################################################################################
            # TUBE INFO
            #
            # Tube Object
            # Angle rotation away from horizontal plane
            tubeOrient = RC.getObjectOrientation(RC.CTUBE_OBJ_NAME) # Must be the same name set in V-Rep
            self._observation.append(np.array(abs(tubeOrient[1]), dtype=np.float32))

            # Distance from plate to the center of end-tip
            tubePos = RC.getObjectWorldPosition(RC.CTUBE_OBJ_NAME)
            palmPos = self._robot.getEndTipWorldPosition()
            d = math.sqrt((tubePos[0] - palmPos[0])**2 +
                          (tubePos[1] - palmPos[1])**2)
            self._observation.append(np.array(d, dtype=np.float32))

        return self._observation

    def reloadFallingObjects(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = [53.21,17.39]
        inputStrings = RC.CFALL_OBJS_NAMES
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
        if(RC.GB_TRACE):
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
        if (self._terminated or self._envStepCounter>1000): # Reset as step counter exceeds certain value to avoid underfitting
            RC.stopSimulation(self._clientID)
            self._robot.commandJointVibration(0)
            return True

        # Hit the object or object hit the ground
        #self._robot.detectCollisionWith(RC.CFALL_OBJS_NAMES[0])

        if(RC.isTaskObjBalance()):
            pos    = RC.getObjectWorldPosition(RC.CPLATE_OBJ_NAME)
        elif(RC.isTaskObjHold()):
            pos    = RC.getObjectWorldPosition(RC.CTUBE_OBJ_NAME)
        #print('Plate Pos:', platePos)
        if (pos[2] < 1.4):
            RC.stopSimulation(self._clientID)
            self._robot.commandJointVibration(0)
            return True

        res = self.detectObjectsReachGround()
        if(res):
            RC.stopSimulation(self._clientID)
            self._robot.commandJointVibration(0)
            return res

        res = self.detectHandOnGround()
        if(res):
            RC.stopSimulation(self._clientID)
            self._robot.commandJointVibration(0)
        return res

    def _reward(self):
        #print("reward")
        #print(__reward)
        self.__reward = 1000

        robotId = self._robot.id()
        #
        if(robotId == RC.CJACO_ARM_HAND or robotId == RC.CKUKA_ARM_BARRETT_HAND \
                                        or robotId == RC.CUR5_ARM_BARRETT_HAND):
            if(RC.isTaskObjBalance()):
                # Distance of plate away from hand palm center -----------------------------------------------
                platePos    = RC.getObjectWorldPosition(RC.CPLATE_OBJ_NAME)
                endTipPos   = self._robot.getEndTipWorldPosition()
                #print('Plate Pos:', platePos)
                #print('Endtip Pos:', endTipPos)
                d = math.sqrt((platePos[0] - endTipPos[0])**2 +
                              (platePos[1] - endTipPos[1])**2)
                #print('DDDD:', d)
                self.__reward -= d

                # Orientation of the plate -------------------------------------------------------------------
                plateOrient = RC.getObjectOrientation(RC.CPLATE_OBJ_NAME) # Must be the same name set in V-Rep
                alpha = plateOrient[0]
                beta  = plateOrient[1]
                gamma = plateOrient[2]
                #print('Plate Orient', plateOrient)
                self.__reward -= (abs(alpha) + abs(beta))

                #endTipOrient = self._robot.getEndTipOrientation()
                #print('Endtip Orient', endTipOrient)
                #
                #endTipVelocity = self._robot.getEndTipVelocity()
                #print('Endtip Vel', endTipVelocity)

            #handPos = self._robot.getHandWorldPosition()
            #handOrient = self._robot.getHandOrientation()
            ##print('Hand Orient', handOrient)
            #handVelocity = self._robot.getHandVelocity()
            ##print('Hand Vel', handVelocity)

        if(RC.GB_TRACE):
            print('self.__reward:', self.__reward)
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
            objPos = RC.getObjectWorldPosition(RC.CFALL_OBJS_NAMES[i])
            objsPos.append(objPos)
            # This returns a list of two vector3 values (3 floats in a list) representing the linear velocity [x,y,z]
            # and angular velocity [wx,wy,wz] in Cartesian worldspace coordinates.
            objLinearVel = RC.getObjectVelocity(RC.CFALL_OBJS_NAMES[i])
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
            #print("CHECK KUKA COLLISION WITH BALL", RC.CFALL_OBJS_NAMES[i])
            if(self._robot.detectCollisionWith(RC.CFALL_OBJS_NAMES[i]) or \
               self.detectObjectsReachGround()):
                self.setTerminated(1)
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
            objPos = RC.getObjectWorldPosition(RC.CFALL_OBJS_NAMES[i])
            objsPos.append(objPos)
            # This returns a list of two vector3 values (3 floats in a list) representing the linear velocity [x,y,z]
            # and angular velocity [wx,wy,wz] in Cartesian worldspace coordinates.
            objLinearVel = RC.getObjectVelocity(RC.CFALL_OBJS_NAMES[i])
            #print("OBJPS:",i, objPos)
            objInfo+= objPos
            #objInfo.append(objLinearVel[2]) # zVel only
            envInfo.append(objInfo)
            #print("OBJINFO:", objInfo)

        # 5. ACTION -------------------------------------------------------------------------
        #
        actionId = self._robotBot.act(envInfo)
        #print("Robot Act: ", actionId)
        self._robot.actRobot(actionId)

        # 6. UPDATE OBJECTS ROBOT-HIT & GROUND-HIT STATUS
        for i in range(self._CNUM_OBJS):
            #print("CHECK KUKA COLLISION WITH BALL", RC.CFALL_OBJS_NAMES[i])
            if(self._robot.detectCollisionWith(RC.CFALL_OBJS_NAMES[i]) or \
               self.detectObjectsReachGround()):
                self.setTerminated(1)
                print('Terminate')

    '''
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

            if self.isTerminated(): # Terminal state
                run_count += 1
                print('RUN #', run_count, '---------------------------------')
                # Update the q_values
                self._robotBot.update_qvalues()

                ## 0. RELOAD FALLING OBJS --
                self.reset()
                time.sleep(0.5)
                continue
    '''

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


    def detectHandOnGround(self):
        handPos = self._robot.getHandWorldPosition()
        #print('HAND POS', handPos)
        return handPos[2] < 0.1
