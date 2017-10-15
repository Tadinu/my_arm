import numpy as np
import copy
import time
import math
import numpy as np
from numpy.linalg import norm
import random
import collections as col
from threading import Timer
## threading.Timer(1, self.timerTask).start()
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

CSERVER_REMOTE_FUNC_ROBOT_ACT           = 'actRobot'
CSERVER_REMOTE_FUNC_DETECT_COLLISION    = 'detectCollisionWith'
CSERVER_REMOTE_FUNC_RESET_ROBOT_POS     = 'resetRobotPos'
CSERVER_REMOTE_FUNC_GET_OPERATION_STATE = 'getOperationState'


# KUKA ROBOTS ------------------------------------------------------------
#
LBR_iiwa_7_R800_JOINT_NAMES = ['LBR_iiwa_7_R800_joint1',
                               'LBR_iiwa_7_R800_joint2',
                               'LBR_iiwa_7_R800_joint3',
                               'LBR_iiwa_7_R800_joint4',
                               'LBR_iiwa_7_R800_joint5',
                               'LBR_iiwa_7_R800_joint6',
                               'LBR_iiwa_7_R800_joint7']

CBASE_JOINT_NAME = LBR_iiwa_7_R800_JOINT_NAMES[0] # "LBR4p_joint1"
CEND_TIP_NAME = 'LBR_iiwa_7_R800_connection'

class Robot:

    def __init__(self, clientID, robotId, robotHandle):
        self._name = ''
        self._clientID = clientID
        self._id = robotId
        self._robotHandle = robotHandle
        self._jointNames = LBR_iiwa_7_R800_JOINT_NAMES

        self._jointHandles = [vrep.simxGetObjectHandle(clientID,
                                                      jointName,
                                                      vrep.simx_opmode_blocking)[1]

                               for jointName in self._jointNames]

    def getRobotWorldPosition(self):
        res, robotPos =  vrep.simxGetObjectPosition(self._clientID, self._robotHandle, -1, vrep.simx_opmode_oneshot)
        return robotPos

    # action containing joint poses of LBR_iiwa_7_R800_JOINT_NAMES respectively
    def applyAction(self, action):
        inputInts    = []
        inputFloats  = action
        inputStrings = []
        inputBuffer  = bytearray()
        if(self._id == RC.CYOUBOT):
            remoteObjectName = 'youBot'
        else:
            remoteObjectName = RC.CSERVER_REMOTE_API_OBJECT_NAME

        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, remoteObjectName,                  \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_ROBOT_ACT,                     \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)
        return res

    def applyActionForces(self, actionForces):

        # joint target velocities discussed below
        joint_target_velocities = np.ones(len(self._jointNames)) * 10000.0

        for i,jointHandle in enumerate(self._jointHandles):
            # the way we're going to do force control is by setting
            # the target velocities of each joint super high and then
            # controlling the max torque allowed (yeah, i know)

            # get the current joint torque
            res, torque = vrep.simxGetJointForce(self._clientID,
                                                 jointHandle,
                                                 vrep.simx_opmode_blocking)
            if res !=0 : raise Exception()

            vrep.simxSetJointTargetVelocity(self._clientID,
                                            jointHandle,
                                            joint_target_velocities[i], # target velocity
                                            vrep.simx_opmode_blocking)

            # and now modulate the force
            res = vrep.simxSetJointForce(self._clientID,
                                         jointHandle,
                                         abs(actionForces[i]*100), # force to apply
                                         vrep.simx_opmode_blocking)
            if res!=0 : raise Exception()

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)

    def actRobot(self, actionId):
        inputInts    = [actionId] #[objHandles[16]]
        inputFloats  = []
        inputStrings = []
        inputBuffer  = bytearray()
        if(self._id == RC.CYOUBOT):
            remoteObjectName = 'youBot'
        else:
            remoteObjectName = RC.CSERVER_REMOTE_API_OBJECT_NAME

        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, remoteObjectName,                  \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_ROBOT_ACT,                     \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)

    def getOperationState(self):
        inputInts    = [] #[objHandles[16]]
        inputFloats  = []
        inputStrings = []
        inputBuffer  = bytearray()
        if(self._id == RC.CYOUBOT):
            remoteObjectName = 'youBot'
        else:
            remoteObjectName = RC.CSERVER_REMOTE_API_OBJECT_NAME

        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, remoteObjectName,                  \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_GET_OPERATION_STATE,           \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        return retInts[0]

    def getObservation(self):
        observation = []
        for i in range(len(LBR_iiwa_7_R800_JOINT_NAMES)):
            res, jointHandle = vrep.simxGetObjectHandle(self._clientID, LBR_iiwa_7_R800_JOINT_NAMES[i], vrep.simx_opmode_oneshot_wait)
            res, pos         = vrep.simxGetJointPosition(self._clientID, jointHandle, vrep.simx_opmode_streaming)
            #res, vel         = vrep.simxGetJointMatrix(self._clientID, jointHandle, vrep.simx_opmode_streaming)
            res, force       = vrep.simxGetJointForce(self._clientID, jointHandle, vrep.simx_opmode_streaming)
            observation.append(np.array(pos, dtype=np.float32)) # Pos
            observation.append(np.array(force, dtype=np.float32)) # Force

        return observation

    def getCurrentBaseJointPos(self):
        res, jointHandle = vrep.simxGetObjectHandle(self._clientID, CBASE_JOINT_NAME, vrep.simx_opmode_oneshot_wait)
        res, pos         = vrep.simxGetJointPosition(self._clientID, jointHandle, vrep.simx_opmode_streaming)
        return pos

    def resetRobotPos(self):
        inputInts    = [] #[objHandles[16]]
        inputFloats  = []
        inputStrings = []
        inputBuffer  = bytearray()
        if(self._id == RC.CYOUBOT):
            remoteObjectName = 'youBot'
        else:
            remoteObjectName = RC.CSERVER_REMOTE_API_OBJECT_NAME

        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, remoteObjectName,                  \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_RESET_ROBOT_POS,               \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)

    def detectCollisionWith(self, objectName):
        #res, objectCollisionHandle = vrep.simxGetCollisionHandle(self._clientID, objectName, vrep.simx_opmode_oneshot_wait)
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = [53.21,17.39]
        inputStrings = [objectName]
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_DETECT_COLLISION,              \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        if(retInts[0]):
            print('COLLIDED WITH: ', objectName)
        return retInts[0]

    def distanceFromEndTipToPos(self, pos):
        res, endTipHandle = vrep.simxGetObjectHandle(self._clientID, CEND_TIP_NAME, vrep.simx_opmode_oneshot_wait)
        res, endTipPos    = vrep.simxGetObjectPosition(self._clientID, endTipHandle, -1, vrep.simx_opmode_oneshot)
        print('POS:', pos)
        p1 = np.array([pos[0], pos[1], pos[2]])
        p2 = np.array([pos[0], pos[1], pos[2]+1])

        result = norm(np.cross(p2-p1, p1-endTipPos))/norm(p2-p1)
        return result

        dist = math.sqrt((pos[0] - endTipPos[0])**2 +
                         (pos[1] - endTipPos[1])**2 +
                         (pos[2] - endTipPos[2])**2
                         )
        #print('DIST', dist)
        return dist
