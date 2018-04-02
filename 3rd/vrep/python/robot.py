import numpy as np
import copy
import time
import math
import numpy as np
from numpy.linalg import norm
import random
import collections as col
import threading
from threading import Timer
from time import sleep

## threading.Timer(1, self.timerTask).start()
from repeated_timer import RepeatedTimer
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

# http://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
#
# by default the remote API client and V-REP work asynchronously together. And as you mention it, if the client sends a same command more than once for a same simulation step, then only the last command will be retained for processing. This is most of the time not a problem, e.g. when you set an object position several times, then the server will simply set the last specified position for that object. It is important to mention that this command dropping or overwriting happens on a command ID base (and nor for special commands). Two command ids are the same if the command AND the main arguments are the same.
#
# To avoid dropping information you have several possibilities:
#
#     In asynchronous mode, you can use simxWriteStringStream/simxReadStringStream, or simxCallScriptFunction.
#     In synchronous mode, since you trigger each simulation step manually, the client is fully in control when the server has to execute next simulation step. You can read here more about the synchronous mode.

# When the real-time mode is not enabled, then your simulation will update and run as fast as possible.
#
# When the real-time mode is enabled, then your simulation will update and run in a way as to keep not running faster than real-time. If your simulation content is heavy, then it will not keep up with the real-time.
#
# So in your case, if you have not enabled the real-time mode, then each time you send one trigger, the simulation will advance by 50ms. 50ms in simulation time!


CSERVER_REMOTE_FUNC_ROBOT_ACT           = 'actRobot'
CSERVER_REMOTE_FUNC_DETECT_COLLISION    = 'detectCollisionWith'
CSERVER_REMOTE_FUNC_RESET_ROBOT         = 'resetRobot'
CSERVER_REMOTE_FUNC_GET_OPERATION_STATE = 'getRobotOperationStateFromClient'
CSERVER_REMOTE_FUNC_ROBOT_CAUGHT_OBJ    = 'checkRobotCaughtObj'
CSERVER_REMOTE_FUNC_GET_SUCTION_PAD_TIP_INFO = 'getSuctionPadTipInfo'

# KUKA ROBOTS --------------------------------------------------------------------------
#
KUKA_ARM_JOINT_NAMES = [RC.CKUKA_ARM_NAME + ('_joint1'),
                        RC.CKUKA_ARM_NAME + ('_joint2'),
                        RC.CKUKA_ARM_NAME + ('_joint3'),
                        RC.CKUKA_ARM_NAME + ('_joint4'),
                        RC.CKUKA_ARM_NAME + ('_joint5'),
                        RC.CKUKA_ARM_NAME + ('_joint6'),
                        RC.CKUKA_ARM_NAME + ('_joint7')]

KUKA_ARM_JOINT_SIGNAL_NAME_PREFIXES = ['armJoint1',
                                       'armJoint2',
                                       'armJoint3',
                                       'armJoint4',
                                       'armJoint5',
                                       'armJoint6',
                                       'armJoint7']
# UR5 ROBOTS --------------------------------------------------------------------------
#
UR5_ARM_JOINT_NAMES = [RC.CUR5_ARM_NAME + ('_joint1'),
                       RC.CUR5_ARM_NAME + ('_joint2'),
                       RC.CUR5_ARM_NAME + ('_joint3'),
                       RC.CUR5_ARM_NAME + ('_joint4'),
                       RC.CUR5_ARM_NAME + ('_joint5'),
                       RC.CUR5_ARM_NAME + ('_joint6')]

UR5_ARM_JOINT_SIGNAL_NAME_PREFIXES = ['armJoint1',
                                      'armJoint2',
                                      'armJoint3',
                                      'armJoint4',
                                      'armJoint5',
                                      'armJoint6']

# JACO ARM -----------------------------------------------------------------------------
#

# From GRASPIT/graspit/models/robots/HumanHand/eigen/human_eigen_cgdb_refined.xml

GB_JACO_HAND_EIGENGRASP_VALUES = [[-0.52275 ,-0.52275 ,-0.52275]]
GB_JACO_HAND_EIGENGRASP_ORI = [-0.52275 ,-0.52275 ,-0.52275]

# BARRETT HAND ---------------------------------------------------------------------------
#
BARRETT_HAND_JOINT_NAMES = ['Barrett_openCloseJoint',
                            'Barrett_openCloseJoint0',

                            'BarrettHand_jointA_0', # V-Rep Joint Handles: j[1][1]
                            'BarrettHand_jointB_0', # j[1][2]
                            'BarrettHand_jointC_0', # j[1][3]

                            'BarrettHand_jointB_1', # j[2][2] (j[2][1] is dummy)
                            'BarrettHand_jointC_1', # j[2][3]

                            'BarrettHand_jointA_2', # j[3][1]
                            'BarrettHand_jointB_2', # j[3][2]
                            'BarrettHand_jointC_2'] # j[3][3]

BARRETT_HAND_JOINT_SIGNAL_NAME_PREFIXES = ['handOpenCloseJoint',
                                           'handOpenCloseJoint0',
                                           'handJointA0',
                                           'handJointB0',
                                           'handJointC0',

                                           'handJointB1',
                                           'handJointC1',

                                           'handJointA2',
                                           'handJointB2',
                                           'handJointC2']

BARRETT_HAND_JOINT_DOF_RATIOS = {BARRETT_HAND_JOINT_NAMES[0]: 1,
                                 BARRETT_HAND_JOINT_NAMES[1]: 1,
                                 BARRETT_HAND_JOINT_NAMES[2]: 1,
                                 BARRETT_HAND_JOINT_NAMES[3]: 1.2,
                                 BARRETT_HAND_JOINT_NAMES[4]: 1,
                                 BARRETT_HAND_JOINT_NAMES[5]: -1.8,
                                 BARRETT_HAND_JOINT_NAMES[6]: 1.2,
                                 BARRETT_HAND_JOINT_NAMES[7]: 1,
                                 BARRETT_HAND_JOINT_NAMES[8]:-1.8,
                                 BARRETT_HAND_JOINT_NAMES[9]: 1.2}

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

# HEXAPOD ROBOT -------------------------------------------------------------------
#
HEXAPOD_JOINT_NAMES = ['hexa_joint1_0',
                       'hexa_joint2_0',
                       'hexa_joint3_0',

                       'hexa_joint1_1',
                       'hexa_joint2_1',
                       'hexa_joint3_1',

                       'hexa_joint1_2',
                       'hexa_joint2_2',
                       'hexa_joint3_2',

                       'hexa_joint1_3',
                       'hexa_joint2_3',
                       'hexa_joint3_3',

                       'hexa_joint1_4',
                       'hexa_joint2_4',
                       'hexa_joint3_4',

                       'hexa_joint1_5',
                       'hexa_joint2_5',
                       'hexa_joint3_5'
                       ]

HEXAPOD_JOINT_SIGNAL_NAME_PREFIXES = ['hexaJoint10',
                                      'hexaJoint20',
                                      'hexaJoint30',

                                      'hexaJoint11',
                                      'hexaJoint21',
                                      'hexaJoint31',

                                      'hexaJoint12',
                                      'hexaJoint22',
                                      'hexaJoint32',

                                      'hexaJoint13',
                                      'hexaJoint23',
                                      'hexaJoint33',

                                      'hexaJoint14',
                                      'hexaJoint24',
                                      'hexaJoint34',

                                      'hexaJoint15',
                                      'hexaJoint25',
                                      'hexaJoint35'
                                      ]

# ROBOT COMMON INFO ---------------------------------------------------------------
#
GB_CROBOT_JOINT_NAMES                = []
GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES = []
GB_CROBOT_HAND_JOINT_SIGNAL_NAME_PREFIXES = []
GB_CROBOT_HAND_JOINT_NAMES = BARRETT_HAND_JOINT_NAMES

if(RC.GB_CSERVER_ROBOT_ID == RC.CKUKA_ARM_BARRETT_HAND):
    GB_CROBOT_JOINT_NAMES                     = KUKA_ARM_JOINT_NAMES
    GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES      = KUKA_ARM_JOINT_SIGNAL_NAME_PREFIXES
    GB_CROBOT_HAND_JOINT_SIGNAL_NAME_PREFIXES = BARRETT_HAND_JOINT_SIGNAL_NAME_PREFIXES

elif(RC.GB_CSERVER_ROBOT_ID == RC.CUR5_ARM_BARRETT_HAND):
    GB_CROBOT_JOINT_NAMES                     = UR5_ARM_JOINT_NAMES
    GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES      = UR5_ARM_JOINT_SIGNAL_NAME_PREFIXES
    GB_CROBOT_HAND_JOINT_SIGNAL_NAME_PREFIXES = BARRETT_HAND_JOINT_SIGNAL_NAME_PREFIXES

elif(RC.GB_CSERVER_ROBOT_ID == RC.CUR5_ARM_GRIPPER):
    GB_CROBOT_JOINT_NAMES                     = UR5_ARM_JOINT_NAMES
    GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES      = UR5_ARM_JOINT_SIGNAL_NAME_PREFIXES

elif(RC.GB_CSERVER_ROBOT_ID == RC.CHEXAPOD):
    GB_CROBOT_JOINT_NAMES                     = HEXAPOD_JOINT_NAMES
    GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES      = HEXAPOD_JOINT_SIGNAL_NAME_PREFIXES
#else ...

GB_CBASE_JOINT_NAME = GB_CROBOT_JOINT_NAMES[0]
GB_CEND_TIP_NAME = RC.CKUKA_ARM_NAME + ('_connection')


class Robot:

    def __init__(self, clientID, robotId, robotHandle):
        self._name = ''
        self._clientID = clientID
        self._id = robotId
        self._robotHandle = robotHandle
        self._jointNames     = GB_CROBOT_JOINT_NAMES
        self._handJointNames = GB_CROBOT_HAND_JOINT_NAMES
        self._x = 1

        print(self._jointNames)
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
        if(self._id == RC.CJACO_ARM_HAND or self._id == RC.CKUKA_ARM_BARRETT_HAND \
                                         or self._id == RC.CUR5_ARM_BARRETT_HAND):
            self.eigen_grasp_initialize()


        # ---------------------------------------------------------------------------------------------------
        # Initialize Timer Task
        #self.initializeTimerTask()

    #def __del__(self):
        #self.finalizeTimerTask()

    # ########################################################################################################
    # INIT TIMER TASK
    #
    def initializeTimerTask(self):
        # 1 - Timer essential properties
        self._timerInterval = 1

        # 2 - Start Timer
        self._rt = RepeatedTimer(self._timerInterval, self.doTimerTask) # it auto-starts, no need of rt.start()

    def finalizeTimerTask(self):
        self._rt.stop()

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

    # !NOTE: useSignalToJoint means that the joint Control Loop is enabled & in Custom Control Mode (Signal to Joint Control Callback Script)
    def setJointVelocity(self, jointIndex, vel, isHandJoint = False, useSignalToJoint = 0):
        if(isHandJoint):
            jointSignalPrefixes = GB_CROBOT_HAND_JOINT_SIGNAL_NAME_PREFIXES
        else:
            jointSignalPrefixes = GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES

        # http://www.coppeliarobotics.com/helpFiles/en/jointDescription.htm
        # Signal to the joint control callback script
        if(useSignalToJoint):
            jointSignalName = jointSignalPrefixes[jointIndex] + "TargetVel"
            res = vrep.simxSetFloatSignal(self._clientID, jointSignalName, vel,
                                          vrep.simx_opmode_oneshot_wait) # set the signal value: !!simx_opmode_oneshot_wait
        else:
            res = vrep.simxSetJointTargetVelocity(self._clientID,
                                                  self._jointHandles[jointIndex],
                                                  vel, # target velocity
                                                  vrep.simx_opmode_blocking)
        if(res != 0):
            print('Set Joint Velocity Error', res)

        return res

    # !NOTE: useSignalToJoint means that the joint Control Loop is enabled & in Custom Control Mode (Signal to Joint Control Callback Script)
    def setJointForce(self, jointIndex, force, isHandJoint = False, useSignalToJoint = 0):
        if(isHandJoint):
            jointSignalPrefixes = GB_CROBOT_HAND_JOINT_SIGNAL_NAME_PREFIXES
        else:
            jointSignalPrefixes = GB_CROBOT_JOINT_SIGNAL_NAME_PREFIXES

        # Signal to the joint control callback script
        jointSignalName = jointSignalPrefixes[jointIndex] + "Force"
        if(useSignalToJoint):
            res = vrep.simxSetFloatSignal(self._clientID, jointSignalName, force,
                                          vrep.simx_opmode_oneshot_wait) # set the signal value: !!simx_opmode_oneshot_wait
        else:
            res = vrep.simxSetJointForce(self._clientID,
                                         self._jointHandles[jointIndex],
                                         force, # force to apply
                                         vrep.simx_opmode_blocking)
        if(res != 0):
            print('Set Joint Force Error', res)
        return res

    def getCurrentJointForces(self):
        forces = []
        for i in range(len(self._jointHandles)):
            forces.append(RC.getJointForce(self._jointHandles[i]))

        return forces

    # action containing eigen graps ams of KUKA_ARM_JOINT_NAMES respectively
    # !NOTE: Current condition: Target is velocity command to the joint

    # Oct 14: Since working with custom control is not yet cleared yet, set force/torque and velocity does not work!
    # -> Tentative solution is disabling Control loop for joints targeted, then set the target velocity directly using
    # simSetJointTargetVelocity, as specified in this link: http://www.coppeliarobotics.com/helpFiles/en/jointDescription.htm
    #
    # When the joint motor is enabled and the control loop is disabled, then the joint will try to reach the desired
    # target velocity given the maximum torque/force it is capable to deliver. When that maximum torque/force is very high,
    # the target velocity is instantaneously reached and the joint operates in velocity control, otherwise it operates at
    # the specified torque/force until the desired target velocity is reached (torque/force control).
    #
    def applyAction(self, action, requestRemoteServer = True):
        #
        if(requestRemoteServer):
            if(self._id == RC.CYOUBOT):
                remoteObjectName = 'youBot'
            else:
                remoteObjectName = RC.CSERVER_REMOTE_API_OBJECT_NAME

            scale = 1
            if(self._id == RC.CKUKA_ARM_BARRETT_HAND):
                scale = 1
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
            print('ApplyAction:', action)
            res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, remoteObjectName,                  \
                                                                                         vrep.sim_scripttype_childscript,                   \
                                                                                         CSERVER_REMOTE_FUNC_ROBOT_ACT,                     \
                                                                                         inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                         vrep.simx_opmode_oneshot_wait)
        # ===================================================================================================================================
        #
        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)
        return 1

    def getSuctionPadTipInfo(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = [1]
        inputStrings = []
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_GET_SUCTION_PAD_TIP_INFO,      \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        return retFloats
    # ========================================================================================================================================
    # JOIN CONTROL ---------------------------------------------------------------------------------------------------------------------------
    #
    #
    # http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=497
    # The way joints operate in V-REP is directly linked to the ODE or Bullet physics engines.
    #
    # VELOCITY CONTROL:
    # !NOTE: THIS REQUIRES CONTROL LOOP BE DISABLED (OF COURSE, MOTOR ENABLED)
    #
    # When in velocity control (i.e. motor is enabled, position control is disabled (Uncheck it in joint's dynamic properties dialog):
    #
    #     You specify a target velocity
    #     You specify a max. torque
    #     If current velocity is below the target velocity, the max. torque is applied
    #     If current velocity is above the target velocity, a negative torque is applied
    #
    # http://www.coppeliarobotics.com/helpFiles/en/jointDescription.htm
    # When the joint motor is enabled and the control loop is disabled, then the joint will try
    # to reach the desired target velocity given the maximum torque/force it is capable to deliver.
    #
    # When that maximum torque/force is very high, the target velocity is instantaneously reached
    # and the joint operates in velocity control,

    # FORCE/TORQUE CONTROL:
    # Otherwise it operates at the specified torque/force until the desired target velocity is reached (torque/force control).
    #
    # This means that if you want to control your joint in force/torque,
    # just specify a target velocity very high (e.g. that will never be reached).
    # To modulate the torque/force, use the simSetJointForce function.
    #
    # 1) the engine will apply the specified torque/force to the joint
    # 2) if the load on the joint is very small, the maximum velocity will be reached in one
    #    simulation step. And the joint will stay at that maximum velocity
    # 3) if the load on the joint is high, the joint will accelerate, until the maximum velocity is reached

    # SIDE NOTES by Coppelia:
    # http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=497
    # Well,
    #
    # when you measure the force/torque that is applied to a joint, it always depends what is acting upon the joint and how.
    # Let's take an example with a revolute joint:
    #
    #     1) the motor of the joint is disabled
    #     2) the joint has no free movement because its lower limit is 0, and its higher limit is 0 too
    #     3) the rotation axis of the joint lies in the x/y plane
    #
    # If a mass is attached to the joint on one side, you will measure a torque of T=D*M*G (Distance*Mass*Gravity).
    # If the mass is attached to the joint on the other side, you will measure a torque of -T (the opposite).
    #
    # In case your joint exerts a torque/force, you will also be able to measure a positive or negative torque/force, depending on the situation.

    # ========================================================================================================================================
    # JOINT FORCE CONTROL --------------------------------------------------------------------------------------------------------------------
    #
    def doJointForceControl(self, jointIndex, inputForce, handJoint = False):
        # get the current joint force
        if(handJoint):
            jointHandle = self._handJointHandles[jointIndex]
        else:
            jointHandle = self._jointHandles[jointIndex]

        res, curForce = vrep.simxGetJointForce(self._clientID,
                                               jointHandle,
                                               vrep.simx_opmode_blocking)
        if res !=0 : raise Exception()

        curVel = RC.getJointVelocity(jointHandle)

        # To make the joint balance around the position
        # if force has changed signs,
        # we need to change the target velocity sign
        joint_target_vel = 10000
        if(np.sign(curForce) < 0):
            joint_target_vel *= (-1)
            #print('SWITCH FORCE', jointIndex, curForce, curVel)

        res = self.setJointVelocity(jointIndex, joint_target_vel, handJoint)
        if res!=0: raise Exception()

        # and now modulate the force
        res = self.setJointForce(jointIndex, inputForce, handJoint)  # force to apply
        if res!=0 : raise Exception()

        return res

    def applyAllJointActionForces(self, actionForces, handJoint = False):

        # joint target velocities discussed below
        # !Note: len(self._jointNames) == len(self._jointHandles)
        jointCount = len(self._handJointNames) if handJoint else len(self._jointNames)
        for i in range(jointCount):
            self.applyJointSingleActionForce(i, actionForces[i], handJoint)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)

        return 1

    def applyJointSpecificActionForces(self, actionForces, jointIds, handJoint = False):
        for i in range(len(jointIds)):
            self.doJointForceControl(jointIds[i], actionForces[i], handJoint)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)

        return 1

    # ========================================================================================================================================
    # JOINT VEL CONTROL ----------------------------------------------------------------------------------------------------------------------
    #
    # !NOTE: THIS REQUIRES CONTROL LOOP BE DISABLED (OF COURSE, MOTOR ENABLED)
    def doJointVelocityControl(self, jointIndex, inputVel, handJoint = False):
        # get the current joint force
        if(handJoint):
            jointHandle = self._handJointHandles[jointIndex]
        else:
            jointHandle = self._jointHandles[jointIndex]

        # if force has changed signs,
        # we need to change the target velocity sign
        joint_max_torque = 10000
        res = self.setJointForce(jointIndex, joint_max_torque, handJoint)
        if res!=0: raise Exception()

        # and now modulate the force
        res, curForce = vrep.simxGetJointForce(self._clientID,
                                               jointHandle,
                                               vrep.simx_opmode_blocking)
        if res !=0 : raise Exception()

        if(np.sign(curForce) < 0):
            inputVel *= (-1)

        res = self.setJointVelocity(jointIndex, inputVel, handJoint)
        if res!=0 : raise Exception()

        return res

    def applyAllJointActionVels(self, actionVels, handJoint = False):

        # !Note: len(self._jointNames) == len(self._jointHandles)
        jointCount = len(self._handJointNames) if handJoint else len(self._jointNames)
        for i in range(jointCount):
            self.doJointVelocityControl(i, actionVels[i], handJoint)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)

        return 1

    def applyJointSpecificActionVels(self, actionVels, jointIds, handJoint = False):
        for i in range(len(jointIds)):
            self.doJointVelocityControl(jointIds[i], actionVels[i], handJoint)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self._clientID)

        return 1

    # ========================================================================================================================================
    # JOINT VEL CONTROL ----------------------------------------------------------------------------------------------------------------------
    #
    def commandJointVibration(self, isStart):
        res = vrep.simxSetFloatSignal(self._clientID, "commandVibration", isStart,
                                      vrep.simx_opmode_oneshot_wait) # set the signal value: !!simx_opmode_oneshot_wait


    # For working with robotBot only
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
        #endTipPos = self.getEndTipWorldPosition()
        isTaskObjHandBalance    = RC.isTaskObjHandBalance()
        isTaskObjSuctionBalance = RC.isTaskObjSuctionBalance()
        isTaskObjHexapodBalance = RC.isTaskObjHexapodBalance()
        isTaskObjHold           = RC.isTaskObjHold()
        isTaskObjCatch          = RC.isTaskObjCatch()
        isTaskObjTimelyPick     = RC.isTaskObjTimelyPick()
        isTaskObjTimelyCatch    = RC.isTaskObjTimelyCatch()

        if(False): #isTaskObjTimelyPick
            inputInts    = [self._robotHandle] #[objHandles[16]]
            inputFloats  = []
            inputStrings = ''
            inputBuffer  = bytearray()
            ##inputBuffer.append(78)
            ##inputBuffer.append(42)
            res, retInts, randomInitJointPose, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CUR5_ARM_NAME,     \
                                                                                                   vrep.sim_scripttype_childscript,      \
                                                                                                   'gbGetRandomInitJointPoseFromClient', \
                                                                                                   inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                                   vrep.simx_opmode_oneshot_wait)

            observation.append(np.array(randomInitJointPose[0], dtype=np.float32))
            #self._jointHandles
            #for i in range(len(randomInitJointPose)):
                #observation.append(np.array(randomInitJointPose[i], dtype=np.float32))

        for i in range(len(self._jointHandles)):
            if(isTaskObjHexapodBalance):
                pos = RC.getJointPosition(self._jointHandles[i]) # Pos
                #
                observation.append(np.array(pos, dtype=np.float32))
            else:
                if(i == 0 or i == 3 or i == 4 or i == 5):
                    pos = RC.getJointPosition(self._jointHandles[i])
                    #vel = RC.getJointVelocity(self._jointHandles[i])
                    #
                    if(isTaskObjHandBalance or isTaskObjSuctionBalance):
                        observation.append(np.array(pos, dtype=np.float32)) # Pos
                        #observation.append(np.array(vel, dtype=np.float32)) # Vel
                    elif(isTaskObjHold):
                        force = RC.getJointForce(self._jointHandles[i])
                        observation.append(np.array(force, dtype=np.float32)) # Force

        if(isTaskObjHandBalance):
            # HAND INFO (!Hand is also one factor that makes up the object condition (pos & orient), therefore should
            # also be added into the environment)
            #
            # Hand Joint Pos
            handJointPoses = self.getCurrentHandJointPoses()
            observation.append(np.array(handJointPoses[2], dtype=np.float32))
            observation.append(np.array(handJointPoses[7], dtype=np.float32))

        elif(isTaskObjHold):
            # HAND INFO (!Hand is also one factor that makes up the object condition (pos & orient), therefore should
            # also be added into the environment)
            #
            # Hand Joint Pos
            handJointPoses = self.getCurrentHandJointPoses()
            observation.append(np.array(handJointPoses[2], dtype=np.float32))
            observation.append(np.array(handJointPoses[7], dtype=np.float32))

            handJointForces = [RC.getJointForce(self._handJointHandles[0]), RC.getJointForce(self._handJointHandles[1])]
            observation.append(np.array(handJointForces[0], dtype=np.float32))
            observation.append(np.array(handJointForces[1], dtype=np.float32))

        elif(isTaskObjCatch):
            pos0 = RC.getJointPosition(self._jointHandles[0])
            vel0 = RC.getJointVelocity(self._jointHandles[0])
            observation.append(np.array(pos0, dtype=np.float32))
            observation.append(np.array(vel0, dtype=np.float32))

        return observation

    def getCurrentBaseJointPos(self):
        res, jointHandle = vrep.simxGetObjectHandle(self._clientID, GB_CBASE_JOINT_NAME, vrep.simx_opmode_oneshot_wait)
        res, pos         = vrep.simxGetJointPosition(self._clientID, jointHandle, vrep.simx_opmode_streaming)

        # Wait until the first data has arrived (just any blocking funtion):
        vrep.simxGetPingTime(self._clientID)

        # Now you can read the data that is being continuously streamed:
        res, pos         = vrep.simxGetJointPosition(self._clientID, jointHandle, vrep.simx_opmode_buffer)
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
        # Wait here until the robot finish the reset
        #time.sleep(3)
        # ===================================================================================================================================
        #
        # move simulation ahead one time step (only meaningful in V-Rep server synchronous mode!
        vrep.simxSynchronousTrigger(self._clientID)

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

    def checkObjectCaught(self):
        inputInts    = [self._robotHandle] #[objHandles[16]]
        inputFloats  = [1]
        inputStrings = []
        inputBuffer  = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self._clientID, RC.CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                                     vrep.sim_scripttype_childscript,                   \
                                                                                     CSERVER_REMOTE_FUNC_ROBOT_CAUGHT_OBJ,              \
                                                                                     inputInts, inputFloats, inputStrings, inputBuffer, \
                                                                                     vrep.simx_opmode_oneshot_wait)
        if(retInts[0]):
            print('SOME OBJECT CAUGHT: ')
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

    def getHandWorldPosition(self):
        return RC.getObjectWorldPosition(RC.CBARRETT_HAND_NAME)

    def getHandOrientation(self):
        eulerAngles = RC.getObjectOrientation(RC.CBARRETT_HAND_NAME)
        return eulerAngles

    def getHandVelocity(self):
        vel = RC.getObjectVelocity(RC.CBARRETT_HAND_NAME)
        return vel

    def doInverseKinematicsCalculation(self):
        count = 0
        target_index = 0
        change_target_time = dt*1000
        vmax = 0.5
        kp = 200.0
        kv = np.sqrt(kp)

        # define variables to share with nengo
        q = np.zeros(len(joint_handles))
        dq = np.zeros(len(joint_handles))

        # NOTE: main loop starts here -----------------------------------------
        target_xyz = RC.getObjectWorldPosition("obstacle")

        track_target.append(np.copy(target_xyz))  # store for plotting
        target_xyz = np.asarray(target_xyz)

        for ii, joint_handle in enumerate(joint_handles):
            old_q = np.copy(q)
            # get the joint angles
            _, q[ii] = vrep.simxGetJointPosition(
                clientID,
                joint_handle,
                vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception()

            # get the joint velocity
            _, dq[ii] = vrep.simxGetObjectFloatParameter(
                clientID,
                joint_handle,
                2012,  # parameter ID for angular velocity of the joint
                vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception()

        # calculate position of the end-effector
        # derived in the ur5 calc_TnJ class
        xyz = robot_config.Tx('EE', q)

        # calculate the Jacobian for the end effector
        JEE = robot_config.J('EE', q)

        # calculate the inertia matrix in joint space
        Mq = robot_config.Mq(q)

        # calculate the effect of gravity in joint space
        Mq_g = robot_config.Mq_g(q)

        # convert the mass compensation into end effector space
        Mx_inv = np.dot(JEE, np.dot(np.linalg.inv(Mq), JEE.T))
        svd_u, svd_s, svd_v = np.linalg.svd(Mx_inv)
        # cut off any singular values that could cause control problems
        singularity_thresh = .00025
        for i in range(len(svd_s)):
            svd_s[i] = 0 if svd_s[i] < singularity_thresh else \
                1./float(svd_s[i])
        # numpy returns U,S,V.T, so have to transpose both here
        Mx = np.dot(svd_v.T, np.dot(np.diag(svd_s), svd_u.T))

        # calculate desired force in (x,y,z) space
        dx = np.dot(JEE, dq)
        # implement velocity limiting
        lamb = kp / kv
        x_tilde = xyz - target_xyz
        sat = vmax / (lamb * np.abs(x_tilde))
        scale = np.ones(3)
        if np.any(sat < 1):
            index = np.argmin(sat)
            unclipped = kp * x_tilde[index]
            clipped = kv * vmax * np.sign(x_tilde[index])
            scale = np.ones(3) * clipped / unclipped
            scale[index] = 1
        u_xyz = -kv * (dx - np.clip(sat / scale, 0, 1) *
                            -lamb * scale * x_tilde)
        u_xyz = np.dot(Mx, u_xyz)

        # transform into joint space, add vel and gravity compensation
        u = np.dot(JEE.T, u_xyz) - Mq_g

        # calculate the null space filter
        Jdyn_inv = np.dot(Mx, np.dot(JEE, np.linalg.inv(Mq)))
        null_filter = (np.eye(robot_config.num_joints) -
                       np.dot(JEE.T, Jdyn_inv))
        # calculate our secondary control signal
        q_des = np.zeros(robot_config.num_joints)
        dq_des = np.zeros(robot_config.num_joints)
        # calculated desired joint angle acceleration using rest angles
        for ii in range(1, robot_config.num_joints):
            if robot_config.rest_angles[ii] is not None:
                q_des[ii] = (
                    ((robot_config.rest_angles[ii] - q[ii]) + np.pi) %
                    (np.pi*2) - np.pi)
                dq_des[ii] = dq[ii]
        # only compensate for velocity for joints with a control signal
        nkp = kp * .1
        nkv = np.sqrt(nkp)
        u_null = np.dot(Mq, (nkp * q_des - nkv * dq_des))

        u += np.dot(null_filter, u_null)

        # get the (x,y,z) position of the center of the obstacle
        v = RC.getObjectWorldPosition('obstacle')
        v = np.asarray(v)

        # find the closest point of each link to the obstacle
        for ii in range(robot_config.num_joints):
            # get the start and end-points of the arm segment
            p1 = robot_config.Tx('joint%i' % ii, q=q)
            if ii == robot_config.num_joints - 1:
                p2 = robot_config.Tx('EE', q=q)
            else:
                p2 = robot_config.Tx('joint%i' % (ii + 1), q=q)

            # calculate minimum distance from arm segment to obstacle
            # the vector of our line
            vec_line = p2 - p1
            # the vector from the obstacle to the first line point
            vec_ob_line = v - p1
            # calculate the projection normalized by length of arm segment
            projection = np.dot(vec_ob_line, vec_line) / np.sum((vec_line)**2)
            if projection < 0:
                # then closest point is the start of the segment
                closest = p1
            elif projection > 1:
                # then closest point is the end of the segment
                closest = p2
            else:
                closest = p1 + projection * vec_line
            # calculate distance from obstacle vertex to the closest point
            dist = np.sqrt(np.sum((v - closest)**2))
            # account for size of obstacle
            rho = dist - obstacle_radius

            if rho < threshold:

                eta = .02  # feel like i saw 4 somewhere in the paper
                drhodx = (v - closest) / rho
                Fpsp = (eta * (1.0/rho - 1.0/threshold) *
                        1.0/rho**2 * drhodx)

                # get offset of closest point from link's reference frame
                T_inv = robot_config.T_inv('link%i' % ii, q=q)
                m = np.dot(T_inv, np.hstack([closest, [1]]))[:-1]
                # calculate the Jacobian for this point
                Jpsp = robot_config.J('link%i' % ii, x=m, q=q)[:3]

                # calculate the inertia matrix for the
                # point subjected to the potential space
                Mxpsp_inv = np.dot(Jpsp,
                                np.dot(np.linalg.pinv(Mq), Jpsp.T))
                svd_u, svd_s, svd_v = np.linalg.svd(Mxpsp_inv)
                # cut off singular values that could cause problems
                singularity_thresh = .00025
                for ii in range(len(svd_s)):
                    svd_s[ii] = 0 if svd_s[ii] < singularity_thresh else \
                        1./float(svd_s[ii])
                # numpy returns U,S,V.T, so have to transpose both here
                Mxpsp = np.dot(svd_v.T, np.dot(np.diag(svd_s), svd_u.T))

                u_psp = -np.dot(Jpsp.T, np.dot(Mxpsp, Fpsp))
                if rho < .01:
                    u = u_psp
                else:
                    u += u_psp

        # multiply by -1 because torque is opposite of expected
        u *= -1
        print('u: ', u)

        for ii, joint_handle in enumerate(joint_handles):
            # the way we're going to do force control is by setting
            # the target velocities of each joint super high and then
            # controlling the max torque allowed (yeah, i know)

            # get the current joint torque
            _, torque = \
                vrep.simxGetJointForce(
                    clientID,
                    joint_handle,
                    vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception()

            # if force has changed signs,
            # we need to change the target velocity sign
            if np.sign(torque) * np.sign(u[ii]) <= 0:
                joint_target_velocities[ii] = \
                    joint_target_velocities[ii] * -1
                _ = self.setJointVelocity(ii, joint_target_velocities[ii]) # target velocity
            if _ != 0:
                raise Exception()

            # and now modulate the force
            _ = self.setJointForce(ii, abs(u[ii]))  # force to apply
            if _ != 0:
                raise Exception()
