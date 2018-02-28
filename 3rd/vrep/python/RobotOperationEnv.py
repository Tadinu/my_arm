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

CSERVER_REMOTE_FUNC_SHOW_MESSAGE                 = 'displayMessage'
CSERVER_REMOTE_FUNC_RELOAD_FALL_OBJS             = 'reloadFallingObjectsFromClient'
CSERVER_REMOTE_FUNC_DETECT_OBJ_GROUND            = 'detectObjectOnGroundFromClient'
CSERVER_REMOTE_FUNC_OPERATION_TIME               = 'getRobotOperationTimeToTheMomentFromClient'
CSERVER_REMOTE_FUNC_OBJ_AWAY_BASE_PLATE          = 'isObjAwayFromBasePlate'
CSERVER_REMOTE_FUNC_GET_BASE_PLATE_NORMAL_VECTOR = 'getBasePlateNormalVectorFromClient'
CSERVER_REMOTE_FUNC_GET_PLATE_NORMAL_VECTOR      = 'getPlateNormalVectorFromClient'

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

        ##1- RESET ROBOT POS --
        self._robot.resetRobot() # Time sleep inside

        ##2- OBSERVATION -- (The observation after robot reset)
        self._observation = self.getObservation([0]*RC.GB_ACTION_DIM)

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
        #if(RC.isTaskObjSuctionBalance()):
            # Vel-trained joint vel, action[1] --> Refer to the V-Rep Server Robot script to confirm this!
            #self._observation.append(np.array(action[1], dtype=np.float32))

        #robotId = self._robot.id()
        if(RC.isTaskObjHandBalance() or RC.isTaskObjSuctionBalance() or RC.isTaskObjHexapodBalance()):
            ##############################################################################################
            # PLATE INFO
            #
            # Plate Object
            # Distance from plate to the center of end-tip
            platePos  = RC.getObjectWorldPosition(RC.CPLATE_OBJ_NAME)
            endTipPos = self._robot.getEndTipWorldPosition()
            #basePlatePos = RC.getObjectWorldPosition(RC.CBASE_PLATE_OBJ_NAME)
            d = math.sqrt((platePos[0] - endTipPos[0])**2 +
                          (platePos[1] - endTipPos[1])**2)

            # Base Plate Normal Vector -----------------------------------------------------------------------------
            normalVector = self.getBasePlateNormalVector()
            if(len(normalVector) == 3):
                slantingDegree = abs(RC.angle_between(np.array([0,0,1]), np.array(normalVector)))
            else:
                slantingDegree = 0
            #print('SLANT', slantingDegree)
            self._observation.append(np.array(d, dtype=np.float32))
            self._observation.append(np.array(slantingDegree, dtype=np.float32))
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

        elif(RC.isTaskObjCatch()):
            ##############################################################################################
            # BALL INFO
            #
            # Ball Object
            # Ball z position only
            ballPos    = RC.getObjectWorldPosition(RC.CBALL_OBJ_NAME)
            ballLinVel = RC.getObjectVelocity(RC.CBALL_OBJ_NAME)
            ##self._observation.append(np.array(ballPos[0], dtype=np.float32))
            ##self._observation.append(np.array(ballPos[1], dtype=np.float32))
            self._observation.append(np.array(ballPos[2], dtype=np.float32))
            self._observation.append(np.array(ballLinVel[2], dtype=np.float32))

        elif(RC.isTaskObjTimelyPick()):
            ##############################################################################################
            # OBJ INFO
            #
            # Object position on conveyor belt
            objPos = RC.getObjectWorldPosition(RC.CCUBOID_OBJ_NAME)
            self._observation.append(np.array(objPos[1], dtype=np.float32))

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
        return (retInts[0] == 0)

    def _step(self, action):
        ## ----------------------------------------------------------------------------------------
        if(RC.GB_TRACE):
            print('ACTION:', action)
        ## ----------------------------------------------------------------------------------------
        ## APPLY ACTION START
        ##
        self._objAwayFromBasePlate = False

        ## WAIT FOR V-REP SERVER SIMULATION STATE TO BE READY (ACTUALLY, THIS IS ALREADY DONE IN THE OUTER LOOP CALLING _step())
        ##
        ##while(self._robot.getOperationState() != RC.CROBOT_STATE_READY):
            ##time.sleep(0.01)
        self._robot.applyAction(action)

        if(RC.isTaskObjTimelyPick()):
            objName = RC.CCUBOID_OBJ_NAME
            pos = RC.getObjectWorldPosition(objName)
            while(pos[1] <=0.28):
                time.sleep(2)
            ##print('Obj Pos:', pos)

            self._observation = self.getObservation(action)
            reward = self._reward()
            done   = self._termination()
        else:
            reward = 0
            overlong = False
            while(self._robot.getOperationState() == RC.CROBOT_STATE_MOVING):
                #time.sleep(0.05)
                self._objAwayFromBasePlate = self.isObjAwayFromBasePlate()
                if(self._objAwayFromBasePlate):
                    #print('GROUND HIT',i+1)
                    reward -= 10

                #print('Still moving', operationTime)
                if(self.getRobotOperationTime() > 7000):
                    overlong = True
                    break

            # Wait for some time to check again the on-base-plate state of the object:
            if(not self._objAwayFromBasePlate):
                print('Obj still on base!')
            self._observation = self.getObservation(action)
            reward += self._reward() # Reward Addition after observation 2nd!
            done    = self._termination() or overlong
            if(self._objAwayFromBasePlate):
                reward -= 100
            print('Env observed 2nd!', reward, done) # self._robot.getOperationState()
            #print("len=%r" % len(self._observation))

        return self._observation, reward, done, {}

    def _termination(self):
        if (self._terminated):
            ##RC.stopSimulation(self._clientID)
            ##self._robot.commandJointVibration(0)
            return True

        # Hit the object or object hit the ground
        #self._robot.detectCollisionWith(RC.CFALL_OBJS_NAMES[0])

        if(RC.isTaskObjCatch()):
            if (self._robot.checkObjectCaught()):
                return True

        if(RC.isTaskObjTimelyPick()):
            pos = RC.getObjectWorldPosition(RC.CCUBOID_OBJ_NAME)
            if (pos[1] >= 0.27):
                #self._objGroundHit = True
                return True
        # Simple Ground Hit Check
        else:
            self._objAwayFromBasePlate = self.isObjAwayFromBasePlate()
            if(self._objAwayFromBasePlate):
                return True

        if(not RC.isTaskObjCatch()):
            res = self.detectHandOnGround()
            ##if(res):
                ##RC.stopSimulation(self._clientID)
                ##self._robot.commandJointVibration(0)

        return res

    def _reward(self):
        #print("reward")
        #print(__reward)
        self.__reward = 1000

        robotId = self._robot.id()
        #
        if(robotId == RC.CJACO_ARM_HAND or robotId == RC.CKUKA_ARM_BARRETT_HAND
                                        or robotId == RC.CKUKA_ARM_SUCTION_PAD
                                        or robotId == RC.CUR5_ARM_BARRETT_HAND
                                        or robotId == RC.CHEXAPOD):
            # ------------------------------------------------------------------------------------------------------------
            # BALANCE TASK -----------------------------------------------------------------------------------------------
            #
            if(RC.isTaskObjHandBalance() or RC.isTaskObjSuctionBalance() or RC.isTaskObjHexapodBalance()):
                # Distance of plate away from hand palm center -----------------------------------------------------------
                platePos  = RC.getObjectWorldPosition(RC.CPLATE_OBJ_NAME)
                endTipPos = self._robot.getEndTipWorldPosition()
                #print('Plate Pos:', platePos)
                #print('Endtip Pos:', endTipPos)
                d = math.sqrt((platePos[0] - endTipPos[0])**2 +
                              (platePos[1] - endTipPos[1])**2)
                #print('DDDD:', d)

                # Slanting Degree of the plate ---------------------------------------------------------------------------
                #suctionPadOrient = RC.getObjectOrientation(RC.CSUCTION_PAD_NAME)
                slantingDegree   = abs(RC.angle_between(np.array([0,0,1]), np.array(self.getBasePlateNormalVector())))
                #print('SUC', slantingDegree)
                # Actually d/cos(slantingDegree) = Plate Center <-> Base Plate Center distance & slantingDegree is already included into the penalty/reward
                self.__reward -= d
                self.__reward -= 50*slantingDegree

                #endTipOrient = self._robot.getEndTipOrientation()
                #
                #endTipVelocity = self._robot.getEndTipVelocity()
                #print('Endtip Vel', endTipVelocity)
                #if(RC.isTaskObjSuctionBalance()):
                #    suctionPadOrient = RC.getObjectOrientation(RC.CSUCTION_PAD_NAME)
                #    alpha2 = suctionPadOrient[0] # Around x
                #    beta2  = suctionPadOrient[1] # Around y
                #    gamma2 = suctionPadOrient[2] # Around z
                #    self.__reward -= (abs(alpha2+3.14) + abs(beta2))
                #    #print('SuctionPad Orient', suctionPadOrient)

                #handPos = self._robot.getHandWorldPosition()
                #handOrient = self._robot.getHandOrientation()
                ##print('Hand Orient', handOrient)
                #handVelocity = self._robot.getHandVelocity()
                ##print('Hand Vel', handVelocity)

            # ----------------------------------------------------------------------------------------------------------
            # CATCH TASK -----------------------------------------------------------------------------------------------
            #
            elif(RC.isTaskObjCatch()):
                # Distance of base joint pos to the ball
                ballPos = RC.getObjectWorldPosition(RC.CBALL_OBJ_NAME)
                basePos = self._robot.getCurrentBaseJointPos()
                angle   = math.atan2(ballPos[1], ballPos[0])
                self.__reward -= abs(angle - basePos)

                # Z Distance from ball to the suction pad tip
                suctionPadPosZ = 9.5846e-01 + 1.0000e-01
                dist = ballPos[2] - suctionPadPosZ
                if(dist > 0):
                    self.__reward -= dist
                else:
                    self.__reward -= 100

                # Wrist joint orientation directed to the ball
                #suctionPadOrient = RC.getObjectOrientation(RC.CSUCTION_PAD_NAME)
                #print('Suction Pad Orient', suctionPadOrient)

                # 3D Distance from ball to the suction pad tip
                padTip = self._robot.getSuctionPadTipInfo()
                endTip = RC.getObjectWorldPosition('RobotTip')
                self.__reward -= RC.getDistanceFromPointToLine(ballPos, padTip, endTip)
                ##suctionPadLine = padTip - endTip

        #
        if(robotId == RC.CUR5_ARM_GRIPPER):
            cuboidPos   = RC.getObjectWorldPosition(RC.CCUBOID_OBJ_NAME)
            grippingPos = [-0.3766 + 0.105, -0.0001, 1.0013]
            self.__reward -= abs(cuboidPos[1]-grippingPos[1])

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

    def getBasePlateNormalVector(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = []
        inputStrings = []
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_GET_BASE_PLATE_NORMAL_VECTOR,  \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        return retFloats

    def getPlateNormalVector(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = []
        inputStrings = []
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_GET_PLATE_NORMAL_VECTOR,       \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        return retFloats

    def showStatusBarMessage(self, message):
        vrep.simxAddStatusbarMessage(self._clientID, message, vrep.simx_opmode_oneshot)


    def detectHandOnGround(self):
        handPos = self._robot.getHandWorldPosition()
        #print('HAND POS', handPos)
        return handPos[2] < 0.1
