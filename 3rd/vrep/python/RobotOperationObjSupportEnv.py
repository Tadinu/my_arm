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

CSERVER_REMOTE_FUNC_SHOW_MESSAGE      = 'displayMessage'
CSERVER_REMOTE_FUNC_RELOAD_FALL_OBJS  = 'reloadFallingObjectsFromClient'
CSERVER_REMOTE_FUNC_DETECT_OBJ_GROUND = 'detectObjectOnGroundFromClient'
CSERVER_REMOTE_FUNC_OPERATION_TIME    = 'getRobotOperationTimeToTheMomentFromClient'
CSERVER_REMOTE_FUNC_OBJ_AWAY_BASE_PLATE      = 'isObjAwayFromBasePlate'

class RobotOperationEnvironment(gym.Env):

    def __init__(self, clientID, robotId, robotHandle):
        self._clientID = clientID
        self._robotId  = robotId
        self._robotHandle = robotHandle

        self._timeStep = 0.1 # This is self-defined time interval for each reset!
        self._observation = []
        self._envStepCounter = 0
        self._mutex = threading.Lock()
        self._objAwayFromBasePlate = False
        self._robotOperationTime = 0

        # INITIALIZE ENVIRONMENT
        self.loadEnvironmentObjects()

        # Reset
        self.reset() ## --> Call self._reset() defined below!

        # After loading the robot!
        observationDim = RC.GB_STATE_DIM
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
        return

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
        #self._rt = RepeatedTimer(self._timerInterval, self.doTimerTask) # it auto-starts, no need of rt.start()
        #self._rt.start()

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
        self._objAwayFromBasePlate = False
        #print("Start back simulation")
        #RC.startSimulation(self._clientID)

        ##1- OBSERVATION --
        self._observation = self.getObservation([0]*RC.GB_ACTION_DIM)

        ##2- RESET ROBOT POS --
        self._robot.resetRobot() # Time sleep inside

        ## LOAD FALLING OBJS --  (HAD BETTER LET THIS JOB TO V-REP SERVER, SINCE IT KNOWS WHEN THE RESET PROCESS FINISHED!
        ##self.reloadFallingObjects()

        ## STEP SIMULATION --
        #self._envStepCounter = 0 # This count is set in train_robot_operation.py

        if(not RC.doJointVibration()):
            self._robot.commandJointVibration(1)

        ## ducta--
        #self._timer.start()
        time.sleep(self._timeStep)
        return self._observation

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def doTimerTask(self):
        return 0

    def getObservation(self, action):
        self._observation = self._robot.getObservation()
        #if(RC.isTaskObjSuctionBalancePlate()):
            # Vel-trained joint vel, action[1] --> Refer to the V-Rep Server Robot script to confirm this!
            #self._observation.append(np.array(action[1], dtype=np.float32))

        #robotId = self._robot.id()
        if(RC.isTaskObjSuctionObjectSupport()):
            # Cuboid Pos ------------------------------------------------------------------------------------------
            cuboidPos = RC.getObjectWorldPosition(RC.CCUBOID_OBJ_NAME)
            self._observation.append(np.array(cuboidPos[0], dtype=np.float32))
            self._observation.append(np.array(cuboidPos[1], dtype=np.float32))
            self._observation.append(np.array(cuboidPos[2], dtype=np.float32))

            # Distance from base plate pos and cuboid center ------------------------------------------------------
            basePlatePos    = RC.getObjectWorldPosition(RC.CBASE_PLATE_OBJ_NAME)
            d = math.sqrt((cuboidPos[0] - basePlatePos[0])**2 +
                          (cuboidPos[1] - basePlatePos[1])**2)
            self._observation.append(np.array(d, dtype=np.float32))

            # Orientation of the base plate -----------------------------------------------------------------------
            basePlateOrient = RC.getObjectOrientation(RC.CBASE_PLATE_OBJ_NAME) # Must be the same name set in V-Rep
            alpha1 = basePlateOrient[0] # Around x
            beta1  = basePlateOrient[1] # Around y
            gamma1 = basePlateOrient[2] # Around z
            #print('Plate Orient', plateOrient)
            self._observation.append(np.array(alpha1, dtype=np.float32))
            self._observation.append(np.array(beta1, dtype=np.float32))

        return self._observation

    def reloadFallingObjects(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = [53.21,17.39]
        inputStrings = RC.CFALL_OBJS_NAMES
        inputBuffer  = bytearray()
        ##inputBuffer.append(78)
        ##inputBuffer.append(42)
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

    def isObjAwayFromBasePlate(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = []
        inputStrings = []
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_OBJ_AWAY_BASE_PLATE,           \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        return retInts[0] == 0

    def _step(self, action):
        ## ----------------------------------------------------------------------------------------
        if(RC.GB_TRACE):
            print('ACTION:', action)
        ## ----------------------------------------------------------------------------------------
        ## APPLY ACTION START
        ##
        self._objAwayFromBasePlate = False
        self._robotOperationTime = 0

        ## WAIT FOR V-REP SERVER SIMULATION STATE TO BE READY (ACTUALLY, THIS IS ALREADY DONE IN THE OUTER LOOP CALLING _step())
        ##
        ##while(self._robot.getOperationState() != RC.CROBOT_STATE_READY):
            ##time.sleep(0.01)
        self._robot.applyAction(action)

        #time.sleep(self._timeStep)
        #!!!Wait for some time since starting the action then observe:
        reward = 0
        i = 0
        while(self._robot.getOperationState() == RC.CROBOT_STATE_MOVING):
            time.sleep(0.05)
            self._objAwayFromBasePlate = self.isObjAwayFromBasePlate()
            if(self._objAwayFromBasePlate):
                #print('GROUND HIT',i+1)
                reward -= 100
            i=i+1
            #print('Still moving',i)

        self._observation = self.getObservation(action)
        reward += self._reward() # Reward Addition after observation 2nd!
        done    = self._termination()
        if(self._objAwayFromBasePlate):
            reward -= 100
        print('Env observed 2nd!', reward, done) # self._robot.getOperationState()
        #print("len=%r" % len(self._observation))

        return self._observation, reward, False, {}
        ##return self._observation, reward, done, {}

    def _termination(self):
        if (self._terminated):
            ##RC.stopSimulation(self._clientID)
            ##self._robot.commandJointVibration(0)
            return True

        self._objAwayFromBasePlate = self.isObjAwayFromBasePlate()
        return (self._objAwayFromBasePlate)
        #return false

    def _reward(self):
        #print(__reward)
        self.__reward = 1000

        robotId = self._robot.id()
        #
        if(robotId == RC.CJACO_ARM_HAND or robotId == RC.CKUKA_ARM_BARRETT_HAND
                                        or robotId == RC.CKUKA_ARM_SUCTION_PAD
                                        or robotId == RC.CUR5_ARM_BARRETT_HAND
                                        or robotId == RC.CHEXAPOD):
            # ----------------------------------------------------------------------------------------------------------
            # BALANCE TASK ---------------------------------------------------------------------------------------------
            #
            #if(not self.isObjAwayFromBasePlate()):
            cuboidPos    = RC.getObjectWorldPosition(RC.CCUBOID_OBJ_NAME)
            basePlatePos = RC.getObjectWorldPosition(RC.CBASE_PLATE_OBJ_NAME)
            d = math.sqrt((cuboidPos[0] - basePlatePos[0])**2 +
                          (cuboidPos[1] - basePlatePos[1])**2)
            #print('DDDD:', d)
            self.__reward -= d

            # Orientation of the base plate -----------------------------------------------------------------------
            basePlateOrient = RC.getObjectOrientation(RC.CBASE_PLATE_OBJ_NAME) # Must be the same name set in V-Rep
            alpha1 = basePlateOrient[0] # Around x
            beta1  = basePlateOrient[1] # Around y
            gamma1 = basePlateOrient[2] # Around z
            #print('Plate Orient', plateOrient)
            self.__reward -= (abs(alpha1) + abs(beta1))
            #else:
                #self.__reward -= 100

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

    def getRobotOperationTime(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = []
        inputStrings = []
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME,    \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_OPERATION_TIME,             \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        return retInts[0]

    def showStatusBarMessage(self, message):
        vrep.simxAddStatusbarMessage(self._clientID, message, vrep.simx_opmode_oneshot)


    def detectHandOnGround(self):
        handPos = self._robot.getHandWorldPosition()
        #print('HAND POS', handPos)
        return handPos[2] < 0.1
