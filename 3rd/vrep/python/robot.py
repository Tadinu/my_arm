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

# EigenGrasp
from eigen_grasp import EigenGrasp, EigenGraspInterface

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
CSERVER_REMOTE_FUNC_RESET_ROBOT         = 'resetRobot'
CSERVER_REMOTE_FUNC_GET_OPERATION_STATE = 'getOperationState'

# KUKA ROBOTS --------------------------------------------------------------------------
#
KUKA_ARM_JOINT_NAMES = [RC.CKUKA_ARM_NAME.append('_joint1'),
                        RC.CKUKA_ARM_NAME.append('_joint2'),
                        RC.CKUKA_ARM_NAME.append('_joint3'),
                        RC.CKUKA_ARM_NAME.append('_joint4'),
                        RC.CKUKA_ARM_NAME.append('_joint5'),
                        RC.CKUKA_ARM_NAME.append('_joint6'),
                        RC.CKUKA_ARM_NAME.append('_joint7')]

ARM_JOINT_SIGNAL_NAME_PREFIXES = ['armJoint1',
                                  'armJoint2',
                                  'armJoint3',
                                  'armJoint4',
                                  'armJoint5',
                                  'armJoint6',
                                  'armJoint7']
# JACO ARM -----------------------------------------------------------------------------
#

# From GRASPIT/graspit/models/robots/HumanHand/eigen/human_eigen_cgdb_refined.xml

GB_JACO_HAND_EIGENGRASP_VALUES = [[-0.52275 ,-0.52275 ,-0.52275]]
GB_JACO_HAND_EIGENGRASP_ORI = [-0.52275 ,-0.52275 ,-0.52275]

# BARRETT HAND ---------------------------------------------------------------------------
#
BARRETT_HAND_JOINT_NAMES = ['BarrettHand_jointA_0', # V-Rep Joint Handles: j[1][1]
                            'BarrettHand_jointB_0', # j[1][2]
                            'BarrettHand_jointC_0', # j[1][3]

                            'BarrettHand_jointB_1', # j[2][2] (j[2][1] is dummy)
                            'BarrettHand_jointC_1', # j[2][3]

                            'BarrettHand_jointA_2', # j[3][1]
                            'BarrettHand_jointB_2', # j[3][2]
                            'BarrettHand_jointC_2'] # j[3][3]

KUKA_ARM_BARRETT_HAND_JOINT_NAMES=[]
for i in range(len(KUKA_ARM_JOINT_NAMES)):
    KUKA_ARM_BARRETT_HAND_JOINT_NAMES.append(KUKA_ARM_JOINT_NAMES[i])

for i in range(len(BARRETT_HAND_JOINT_NAMES)):
    KUKA_ARM_BARRETT_HAND_JOINT_NAMES.append(BARRETT_HAND_JOINT_NAMES[i])

BARRETT_HAND_JOINT_DOF_RATIOS = {BARRETT_HAND_JOINT_NAMES[0]: 1,
                                 BARRETT_HAND_JOINT_NAMES[1]: 1.2,
                                 BARRETT_HAND_JOINT_NAMES[2]: 1,
                                 BARRETT_HAND_JOINT_NAMES[3]: -1.8,
                                 BARRETT_HAND_JOINT_NAMES[4]: 1.2,
                                 BARRETT_HAND_JOINT_NAMES[5]: 1,
                                 BARRETT_HAND_JOINT_NAMES[6]:-1.8,
                                 BARRETT_HAND_JOINT_NAMES[7]: 1.2}

# From GRASPIT/graspit/models/robots/HumanHand/eigen/human_eigen_cgdb_refined.xml
GB_BARRETT_HAND_EIGENGRASP_VALUES = [[1.0, 0.0 ,0.0 ,0.0],
                                     [0.0, 1.0 ,1.0 ,1.0]]
GB_BARRETT_HAND_EIGENGRASP_ORI = [1.13, 0.79, 0.79, 0.79]

# ---------------------------------------------------------------------------------
# EIGENGRASP INFO
#
GB_HAND_EIGENGRASP_VALUES = GB_BARRETT_HAND_EIGENGRASP_VALUES
GB_CEIGENGRASP_NO         = len(GB_HAND_EIGENGRASP_VALUES)

GB_HAND_EIGENGRASP_ORI = GB_BARRETT_HAND_EIGENGRASP_ORI

# ROBOT COMMON INFO
#
GB_CROBOT_JOINT_NAMES                = []
GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES = []
GB_CROBOT_HAND_JOINT_NAMES = BARRETT_HAND_JOINT_NAMES
if(RC.GB_CSERVER_ROBOT_ID == RC.CKUKA_ARM_BARRETT_HAND):
    GB_CROBOT_JOINT_NAMES = KUKA_ARM_JOINT_NAMES
    GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES = ARM_JOINT_SIGNAL_NAME_PREFIXES
#else ...

GB_CBASE_JOINT_NAME = GB_CROBOT_JOINT_NAMES[0] # "LBR4p_joint1"
GB_CEND_TIP_NAME = RC.CKUKA_ARM_NAME.append('_connection')


class Robot:

    def __init__(self, clientID, robotId, robotHandle):
        self._name = ''
        self._clientID = clientID
        self._id = robotId
        self._robotHandle = robotHandle
        self._jointNames     = GB_CROBOT_JOINT_NAMES
        self._handJointNames = GB_CROBOT_HAND_JOINT_NAMES

        self._jointHandles = [vrep.simxGetObjectHandle(clientID,
                                                      jointName,
                                                      vrep.simx_opmode_blocking)[1]
                              for jointName in self._jointNames]

        self._handJointHandles = [vrep.simxGetObjectHandle(clientID,
                                              handJointName,
                                              vrep.simx_opmode_blocking)[1]
                                  for handJointName in self._handJointNames]
        # ---------------------------------------------------------------------------------------------------
        # Initialize EigenGrasps Info (after initializing the hand specifics)
        if(self._id == RC.CJACO_ARM_HAND or self._id == RC.CKUKA_ARM_BARRETT_HAND):
            self.eigen_grasp_initialize()

    # ########################################################################################################
    # INIT EIGEN GRASPS
    #
    def eigen_grasp_initialize(self):

        # Eigen Graps List
        #
        global gb_hand_eigengrasps
        gb_hand_eigengrasps = [EigenGrasp(self.getDofsCount(), 0.0) for i in range(GB_CEIGENGRASP_NO)]
        for i in range(len(gb_hand_eigengrasps)):
            gb_hand_eigengrasps[i].setVals(GB_HAND_EIGENGRASP_VALUES[i])

        # Eigen Graps Interface
        #
        global gb_hand_eigengrasp_interface
        gb_hand_eigengrasp_interface = EigenGraspInterface(self, gb_hand_eigengrasps, GB_HAND_EIGENGRASP_ORI)

    def accumulateMove(self, dofs):
        handJointPoses = self.getCurrentHandJointPoses()
        for dof in dofs:
            for i in range(self.getHandJointsCount()):
                handJointPoses[i] = dof * BARRETT_HAND_JOINT_DOF_RATIOS[self._handJointNames[i]]
        return handJointPoses

    def id(self):
        return self._id

    def getRobotWorldPosition(self):
        res, robotPos =  vrep.simxGetObjectPosition(self._clientID, self._robotHandle, -1, vrep.simx_opmode_oneshot)
        return robotPos

    def getEndTipWorldPosition(self):
        return RC.getObjectWorldPosition(GB_CEND_TIP_NAME)

    def getEndTipOrientation(self):
        return RC.getObjectOrientation(GB_CEND_TIP_NAME)

    def getEndTipVelocity(self):
        return RC.getObjectVelocity(GB_CEND_TIP_NAME)

    def getDOFRange(self, i):
        return -3.14, 3.14

    # This name is immutable, required by eigen_grasp
    def getDofsCount(self):
        return len(GB_BARRETT_HAND_EIGENGRASP_VALUES[0])

    def getJointsCount(self):
        return len(self._jointHandles)

    def getHandJointsCount(self):
        return len(self._handJointHandles)

    def getCurrentHandJointPoses(self):
        handJointPoses = []
        for i in range(len(self._handJointNames)):
            res, handJointHandle = vrep.simxGetObjectHandle(self._clientID, self._handJointNames[i], vrep.simx_opmode_oneshot_wait)
            pos = RC.getJointPosition(handJointHandle)
            handJointPoses.append(pos)

        return handJointPoses

    def getJointCurrentPos(self, jointName):
        res, jointHandle = vrep.simxGetObjectHandle(self._clientID, jointName, vrep.simx_opmode_oneshot_wait)
        return RC.getJointPosition(jointHandle)

    # This name is immutable, required by eigen_grasp
    def getCurrentDofs(self):
        dofs = [1] * self.getDofsCount()
        handJointPoses = self.getCurrentHandJointPoses()

        for i in range(len(dofs)):
            for j in range(len(handJointPoses)):
                dofs[i] = handJointPoses[j]/BARRETT_HAND_JOINT_DOF_RATIOS[self._handJointNames[j]]

        return dofs

    def setJointVelocity(self, jointIndex, vel):
        # http://www.coppeliarobotics.com/helpFiles/en/jointDescription.htm
        # Signal to the joint control callback script
        jointSignalName = GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES[jointIndex] + "TargetVel"
        res = vrep.simxSetFloatSignal(self._clientID, jointSignalName, vel, \
                                      vrep.simx_opmode_oneshot) # set the signal value

        #res = vrep.simxSetJointTargetVelocity(self._clientID,
        #                                      self._jointHandles[jointIndex],
        #                                      vel,  # target velocity
        #                                      vrep.simx_opmode_blocking)
        return res

    def setJointForce(self, jointIndex, force):
        # Signal to the joint control callback script
        jointSignalName = GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES[jointIndex] + "ForceTorque"
        res = vrep.simxSetFloatSignal(self._clientID, jointSignalName, force, \
                                      vrep.simx_opmode_oneshot) # set the signal value

        # OR CALL DIRECTLY
        #res = vrep.simxSetJointForce(self._clientID,
        #                             self._jointHandles[jointIndex],
        #                             force, # force to apply
        #                             vrep.simx_opmode_blocking)
        return res

    def getCurrentJointForces(self):
        forces = []
        for i in range(len(self._jointHandles)):
            forces.append(RC.getJointForce(self._jointHandles[i]))

        return forces

    # action containing eigen graps ams of KUKA_ARM_JOINT_NAMES respectively
    def applyAction(self, action):
        if(self._id == RC.CYOUBOT):
            remoteObjectName = 'youBot'
        else:
            remoteObjectName = RC.CSERVER_REMOTE_API_OBJECT_NAME

        scale = 2
        if(self._id == RC.CKUKA_ARM or self._id == RC.CKUKA_ARM_BARRETT_HAND):
            scale = 2
        elif(self._id == RC.CJACO_ARM_HAND):
            hand_eigengrasp_amps = action
            dofs = gb_hand_eigengrasp_interface.toDOF(hand_eigengrasp_amps)
            action = self.accumulateMove(dofs) # dofs -> jointPoses

        action = [action[i]*scale for i in range(len(action))]

        # -----------------------------------------------------------------------
        #
        inputInts    = []
        inputFloats  = action
        inputStrings = []
        inputBuffer  = bytearray()
        #print('ApplyAction:', action)
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, remoteObjectName,                  \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_ROBOT_ACT,                     \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)
        return self.applyActionForces(action)

    def applyActionForces(self, actionForces):

        # joint target velocities discussed below
        # !Note: len(self._jointNames) == len(self._jointHandles)
        joint_target_velocities = np.ones(len(self._jointNames)) * 10000.0

        for i,jointHandle in enumerate(self._jointHandles):
            # http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=497
            # The way joints operate in V-REP is directly linked to the ODE or Bullet physics engines.
            # When in velocity control (i.e. motor is enabled, position control is disabled (Uncheck it in joint's dynamic properties dialog):
            #
            #     You specify a target velocity
            #     You specify a max. torque
            #     If current velocity is below the target velocity, the max. torque is applied
            #     If current velocity is above the target velocity, a negative torque is applied

            # This means that if you want to control your joint in force/torque,
            # just specify a target velocity very high (e.g. that will never be reached).
            # To modulate the torque/force, use the simSetJointForce function.
            #
            # 1) the engine will apply the specified torque/force to the joint
            # 2) if the load on the joint is very small, the maximum velocity will be reached in one
            # simulation step. And the joint will stay at that maximum velocity
            # 3) if the load on the joint is high, the joint will accelerate, until the maximum velocity is reached

            # get the current joint torque
            res, torque = vrep.simxGetJointForce(self._clientID,
                                                 jointHandle,
                                                 vrep.simx_opmode_blocking)
            if res !=0 : raise Exception()
            #print('TORQUE', i, torque)
            # if force has changed signs,
            # we need to change the target velocity sign
            if(np.sign(torque) * np.sign(actionForces[i]) < 0):
                joint_target_velocities[i] = joint_target_velocities[i] * (-1)

            res = self.setJointVelocity(i, joint_target_velocities[i])
            #if res!=0: raise Exception()

            # and now modulate the force
            res = self.setJointForce(i, abs(actionForces[i]*10))  # force to apply
            #if res!=0 : raise Exception()
        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)

        return res

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

        for i in range(len(self._jointHandles)):
            pos   = RC.getJointPosition(self._jointHandles[i])
            #linearVel, angVel  = RC.getObjectVelocity(self._jointHandles[i])
            force = RC.getJointForce(self._jointHandles[i])
            #observation.append(np.array(pos, dtype=np.float32)) # Pos
            observation.append(np.array(force, dtype=np.float32)) # Force

        if(self._id == RC.CKUKA_ARM):
            endTipPos = self.getEndTipWorldPosition()
            observation.append(np.array(endTipPos[0], dtype=np.float32))
            observation.append(np.array(endTipPos[1], dtype=np.float32))
            observation.append(np.array(endTipPos[2], dtype=np.float32))

        elif(self._id == RC.CJACO_ARM_HAND or self._id == RC.CKUKA_ARM_BARRETT_HAND):
            x = 1

            '''
            # HAND INFO
            # Hand Joint Pos
            handJointPoses = self.getCurrentHandJointPoses()
            observation.append(np.array(handJointPoses[0], dtype=np.float32))
            observation.append(np.array(handJointPoses[5], dtype=np.float32))

            ##############################################################################################
            # PLATE INFO
            #
            # Plate Object
            # Angle rotation away from horizontal plane
            plateOrient = RC.getObjectOrientation(RC.CPLATE_OBJ_NAME) # Must be the same name set in V-Rep
            observation.append(np.array(abs(plateOrient[0]), dtype=np.float32))

            # Distance from plate to the center of end-tip
            platePos    = RC.getObjectWorldPosition(RC.CPLATE_OBJ_NAME)
            endTipPos   = self.getEndTipWorldPosition()
            d = math.sqrt((platePos[0] - endTipPos[0])**2 +
                          (platePos[1] - endTipPos[1])**2)
            observation.append(np.array(d, dtype=np.float32))
            '''

        return observation

    def getCurrentBaseJointPos(self):
        res, jointHandle = vrep.simxGetObjectHandle(self._clientID, GB_CBASE_JOINT_NAME, vrep.simx_opmode_oneshot_wait)
        res, pos         = vrep.simxGetJointPosition(self._clientID, jointHandle, vrep.simx_opmode_streaming)
        return pos

    def resetRobot(self):
        # !NOTE: V-REP NOT ALLOW ALL-VOID PARAMETERS, SO WE HAVE TO ADD A DUMMY VALUE INTO inputFloats[]
        inputInts    = []
        inputFloats  = [1]
        inputStrings = []
        inputBuffer  = bytearray()
        if(self._id == RC.CYOUBOT):
            remoteObjectName = 'youBot'
        else:
            remoteObjectName = RC.CSERVER_REMOTE_API_OBJECT_NAME

        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, remoteObjectName,                  \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_RESET_ROBOT,                   \
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
        result = 0

        endTipPos = self.getEndTipWorldPosition()
        #print('POS:', pos, endTipPos)

        #distPosLine = norm(np.cross(p2-p1, p1-endTipPos))/norm(p2-p1)

        distToPos = math.sqrt((pos[0] - endTipPos[0])**2 +
                              (pos[1] - endTipPos[1])**2 +
                              (pos[2] - endTipPos[2])**2
                              )
        #print('DIST', distToPos)

        result += distToPos
        return result

    def getHandOrientation(self):
        eulerAngles = RC.getObjectOrientation(RC.CBARRETT_HAND_NAME)
        return eulerAngles

    def getHandVelocity(self):
        vel = RC.getObjectVelocity(RC.CBARRETT_HAND_NAME)
        return vel
