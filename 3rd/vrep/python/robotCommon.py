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

import numpy as np
from numpy.linalg import norm

GB_TRACE = 0
GB_MODE_TRAINING = 1 # 1: Training, 0: Enjoying/Running/Testing
GB_MODE_ENJOYING = 0

# ================================================================
# SERVER INFO ----------------------------------------------------
#
CSERVER_PORT = 19999
CSERVER_REMOTE_API_OBJECT_NAME = 'remoteApiCommandServer'

# ================================================================
# ROBOT IDS ------------------------------------------------------
#
CYOUBOT = 1
CJACO_ARM_HAND = 2
CKUKA_ARM_BARRETT_HAND = 3
CUR5_ARM_BARRETT_HAND  = 4
CKUKA_ARM_SUCTION_PAD  = CKUKA_ARM_BARRETT_HAND
CUR5_ARM_GRIPPER = 5
CHEXAPOD = 6

# ================================================================
# ROBOT OPERATION STATE-------------------------------------------
#
CROBOT_STATE_READY  = 1 # After Reset is finished
CROBOT_STATE_MOVING = 2 # Running an action
CROBOT_STATE_MOVING_ENDED = 3 # Running an action
CROBOT_STATE_RESETTING_ARM_ENDED  = 4 # Running Arm Reset
CROBOT_STATE_RESETTING_HAND = 5 # Running Hand Reset

# ================================================================
# SERVER ROBOT ID !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#
GB_CSERVER_ROBOT_ID = CUR5_ARM_GRIPPER #CKUKA_ARM_BARRETT_HAND
GB_CSERVER_ROBOT_NAME = ''

# ================================================================
# ROBOT NAMES ----------------------------------------------------
#
CUR5_ARM_NAME       = 'UR5'
CKUKA_ARM_NAME      = 'LBR_iiwa_14_R820' # 'LBR_iiwa_7_R800'
CYOUBOT_NAME        = 'youBot'# 'LBR4p'
CJACO_ARM_HAND_NAME = 'JacoHand'
CBARRETT_HAND_NAME  = 'BarrettHand'
CHEXAPOD_NAME       = 'hexapod'

# ================================================================
# OBJECT NAMES ---------------------------------------------------
#
CFALL_OBJS_NAMES      = ['Obj1']
CPLATE_OBJ_NAME       = 'Plate'
CBASE_PLATE_OBJ_NAME  = 'BasePlate'
CTABLE_OBJ_NAME       = 'Table'
CTUBE_OBJ_NAME        = 'Tube'
CBALL_OBJ_NAME        = 'Ball'
CCUBOID_OBJ_NAME      = 'Cuboid'
CCONVEYOR_BELT_SENSOR = 'conveyorBelt_sensor'
CSUCTION_PAD_NAME     = 'suctionPad'

# ================================================================
# TRAINING TASK NAMES --------------------------------------------
#
CTASK_ID_UNKNOWN             = -1
CTASK_ID_OBJ_HAND_BALANCE    = 1
CTASK_ID_OBJ_SUCTION_BALANCE_POS_ONLY = 2
CTASK_ID_OBJ_SUCTION_BALANCE_POS_VEL = 3
CTASK_ID_OBJ_HEXAPOD_BALANCE = 4
CTASK_ID_OBJ_HOLD            = 5
CTASK_ID_OBJ_CATCH           = 6
CTASK_ID_OBJ_BALANCE         = 7
CTASK_ID_OBJ_MOVE_CATCH      = 8
CTASK_ID_OBJ_AVOID           = 9
CTASK_ID_OBJ_TIMELY_PICK     = 10 # On conveyor belt

CTASK_OBJ_TIMELY_PICK_POSITION = 0.29

GB_TASK_ID = CTASK_ID_OBJ_TIMELY_PICK #CTASK_ID_OBJ_SUCTION_OBJECT_SUPPORT

def isUnknownTask():
    return GB_TASK_ID == CTASK_ID_UNKNOWN

def isTaskObjHandBalance():
    return GB_TASK_ID == CTASK_ID_OBJ_HAND_BALANCE

def isTaskObjSuctionBalance():
    return ((GB_TASK_ID == CTASK_ID_OBJ_SUCTION_BALANCE_POS_ONLY) or \
            (GB_TASK_ID == CTASK_ID_OBJ_SUCTION_BALANCE_POS_VEL))

def isTaskObjSuctionBalancePosVel():
    return GB_TASK_ID == CTASK_ID_OBJ_SUCTION_BALANCE_POS_VEL

def isTaskObjSuctionBalancePosOnly():
    return GB_TASK_ID == CTASK_ID_OBJ_SUCTION_BALANCE_POS_ONLY

def isTaskObjHexapodBalance():
    return GB_TASK_ID == CTASK_ID_OBJ_HEXAPOD_BALANCE

def isTaskObjHold():
    return GB_TASK_ID == CTASK_ID_OBJ_HOLD

def isTaskObjCatch():
    return GB_TASK_ID == CTASK_ID_OBJ_CATCH

def isTaskObjMoveCatch():
    return GB_TASK_ID == CTASK_ID_OBJ_MOVE_CATCH

def isTaskObjTimelyPick():
    return GB_TASK_ID == CTASK_ID_OBJ_TIMELY_PICK

def doJointVibration():
    return isTaskObjHandBalance() or isTaskObjHold()

# ================================================================
# ACTION, STATE DIMENSIONS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#
GB_ACTION_DIM = 1
GB_STATE_DIM  = 10

if(GB_CSERVER_ROBOT_ID == CKUKA_ARM_BARRETT_HAND):
    # 9 (7 Kuka arm joints & 2 Hand finger angle)
    if(isTaskObjHandBalance()):
        # 4 (1 Arm Twist joint, 1 Arm Wrist joint, 2 revolute hand finger base joints)
        GB_ACTION_DIM = 4
        GB_STATE_DIM  = 10
    if(isTaskObjSuctionBalancePosVel()):
        # 2 (1 Middle Twist joint, 1 Elbow joint, 1 Wrist joint), Base joint as fixed movement (environment role)
        GB_ACTION_DIM = 3
        GB_STATE_DIM  = 7
    elif(isTaskObjSuctionBalancePosOnly()):
        # 2 (1 Middle Twist joint, 1 Elbow joint, 1 Wrist joint), Base joint as fixed movement (environment role)
        GB_ACTION_DIM = 3
        GB_STATE_DIM  = 6
    elif(isTaskObjSuctionObjectSupport()):
        # 2 (1 Middle Twist joint, 1 Elbow joint, 1 Wrist joint), Base joint as fixed movement (environment role)
        GB_ACTION_DIM = 3
        GB_STATE_DIM  = 9
    elif(isTaskObjHold()):
        # 2 Hand open Close Joints (force) & 2 revolute hand finger base joints (vel)
        GB_ACTION_DIM = 4
        GB_STATE_DIM  = 13
    elif(isTaskObjCatch()):
        # 1 Base Joint 1 Wrist Joint
        GB_ACTION_DIM = 2
        GB_STATE_DIM  = 4

    GB_CSERVER_ROBOT_NAME = CKUKA_ARM_NAME

elif(GB_CSERVER_ROBOT_ID == CUR5_ARM_BARRETT_HAND):
    GB_ACTION_DIM = 8 # 8(6 Kuka arm joints & 2 Hand finger angle)
    GB_STATE_DIM  = 12
    GB_CSERVER_ROBOT_NAME = CUR5_ARM_NAME

elif(GB_CSERVER_ROBOT_ID == CUR5_ARM_GRIPPER): ## if(isTaskObjTimelyPick()):
    # Target Reaching Vel & Obj Gripping Vel
    GB_ACTION_DIM = 2
    GB_STATE_DIM  = 3 # (8) 6 joint vels and cuboid distance pos (3D & 2D distances)
    GB_CSERVER_ROBOT_NAME = CUR5_ARM_NAME

elif(GB_CSERVER_ROBOT_ID == CJACO_ARM_HAND):
    GB_ACTION_DIM = 1
    GB_STATE_DIM  = 10
    GB_CSERVER_ROBOT_NAME = CJACO_ARM_HAND_NAME

elif(GB_CSERVER_ROBOT_ID == CHEXAPOD):
    GB_ACTION_DIM = 3
    GB_STATE_DIM  = 20
    GB_CSERVER_ROBOT_NAME = CHEXAPOD_NAME

def GB_CLIENT_ID():
    return _GB_CLIENT_ID

def gbRobotHandle():
    return _gbRobotHandle

def initialize_vrep():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections

    global _GB_CLIENT_ID
    _GB_CLIENT_ID=vrep.simxStart('127.0.0.1', CSERVER_PORT,True,True,5000,5) # Connect to V-REP
    if _GB_CLIENT_ID!=-1:
        print ('Connected to remote API server',_GB_CLIENT_ID)

        # Start the simulation:
        startSimulation(_GB_CLIENT_ID)

        # Load a robot instance:    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(_GB_CLIENT_ID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'loadRobot',[],[0,0,0,0],['d:/v_rep/qrelease/release/test.ttm'],emptyBuff,vrep.simx_opmode_oneshot_wait)
        #    robotHandle=retInts[0]

        # Get scene objects data
        res, objHandles, intData, floatData, objNames = vrep.simxGetObjectGroupData(_GB_CLIENT_ID,vrep.sim_appobj_object_type, 0, vrep.simx_opmode_blocking)
        if res==vrep.simx_return_ok:
            print ('Number of objects in the scene: ',len(objHandles), len(objNames))
            for i in range(len(objHandles)):
                print('Obj:', objHandles[i], objNames[i])
        else:
            print ('Remote API function call returned with error code: ',res)

        # Retrieve some handles:
        global _gbRobotHandle
        res, _gbRobotHandle = vrep.simxGetObjectHandle(_GB_CLIENT_ID, GB_CSERVER_ROBOT_NAME, vrep.simx_opmode_oneshot_wait)

def finalize_vrep():
    endSimulation(_GB_CLIENT_ID)

def startSimulation(clientID):
    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm#synchronous
    # Set Simulation in Synchronous mode
    # Enables or disables the synchronous operation mode for the remote API server service that the client is connected to.
    # The function is blocking. While in synchronous operation mode, the client application is in charge of triggering the next simulation step. Only pre-enabled remote API server services will successfully execute this function.
    vrep.simxSynchronous(clientID,False)

    # Set Simulation Step Time
    dt = .005 # 5ms (Custom) as the same as physics engine, seen in Calculation module properties dialog(Dynamics)
    vrep.simxSetFloatingParameter(clientID,
                                  vrep.sim_floatparam_simulation_time_step,
                                  dt, # specify a simulation time step
                                  vrep.simx_opmode_oneshot)

    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)

def stopSimulation(clientID):
    return
    #vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

def endSimulation(clientID):
    # stop the simulation
    stopSimulation(clientID)

    # Before closing the connection to V-REP,
    #make sure that the last command sent out had time to arrive.
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
    print('V-REP Server Connection closed...')


def getObjectWorldPosition(objectName):
    res, objectHandle = vrep.simxGetObjectHandle(_GB_CLIENT_ID, objectName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the object position:
    res, objectPos    = vrep.simxGetObjectPosition(_GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    # Retrieves the time needed for a command to be sent to the server,
    # executed, and sent back.
    # That time depends on various factors like the client settings,
    # the network load, whether a simulation is running, whether the
    # simulation is real-time, the simulation time step, etc.
    # The function is blocking.
    vrep.simxGetPingTime(_GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, objectPos    = vrep.simxGetObjectPosition(_GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_buffer)

    # Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    # to stream unessesary data and eventually slow down.
    vrep.simxGetObjectPosition(_GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_discontinue)

    return objectPos

def getObjectOrientation(objectName):
    res, objectHandle = vrep.simxGetObjectHandle(_GB_CLIENT_ID, objectName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the object position:
    res, eulerAngles  = vrep.simxGetObjectOrientation(_GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(_GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, eulerAngles  = vrep.simxGetObjectOrientation(_GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_buffer)

    # Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    # to stream unessesary data and eventually slow down.
    vrep.simxGetObjectOrientation(_GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_discontinue)

    return eulerAngles

def getObjectVelocity(objectName):
    res, objectHandle = vrep.simxGetObjectHandle(_GB_CLIENT_ID, objectName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the object position:
    res, objectLinearVel, objectAngVel = vrep.simxGetObjectVelocity(_GB_CLIENT_ID, objectHandle, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(_GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, objectLinearVel, objectAngVel = vrep.simxGetObjectVelocity(_GB_CLIENT_ID, objectHandle, vrep.simx_opmode_buffer)

    # Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    # to stream unessesary data and eventually slow down.
    vrep.simxGetObjectVelocity(_GB_CLIENT_ID, objectHandle, vrep.simx_opmode_discontinue)

    return objectLinearVel

def getJointVelocity(jointHandle):
    # http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
    # 2012: vrep.sim_jointfloatparam_velocity
    res, jointVel = vrep.simxGetObjectFloatParameter(_GB_CLIENT_ID, jointHandle, 2012, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(_GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, jointVel = vrep.simxGetObjectFloatParameter(_GB_CLIENT_ID, jointHandle, 2012, vrep.simx_opmode_buffer)

    # Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    # to stream unessesary data and eventually slow down.
    vrep.simxGetObjectFloatParameter(_GB_CLIENT_ID, jointHandle, 2012, vrep.simx_opmode_discontinue)

    return jointVel

def getJointForce(jointHandle):
    #res, jointHandle = vrep.simxGetObjectHandle(_GB_CLIENT_ID, jointName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the joint force:
    res, jointForce = vrep.simxGetJointForce(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(_GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, jointForce = vrep.simxGetJointForce(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_buffer)

    # Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    # to stream unessesary data and eventually slow down.
    vrep.simxGetJointForce(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_discontinue)

    return jointForce

def getJointPosition(jointHandle):
    #res, jointHandle = vrep.simxGetObjectHandle(_GB_CLIENT_ID, jointName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the joint position:
    res, jointPos = vrep.simxGetJointPosition(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(_GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, jointPos = vrep.simxGetJointPosition(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_buffer)

    # Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    # to stream unessesary data and eventually slow down.
    vrep.simxGetJointPosition(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_discontinue)

    return jointPos

def getJointMatrix(jointHandle):
    res, jointMatrix = vrep.simxGetJointMatrix(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(_GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, jointMatrix = vrep.simxGetJointMatrix(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_buffer)

    # Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    # to stream unessesary data and eventually slow down.
    vrep.simxGetJointMatrix(_GB_CLIENT_ID, jointHandle, vrep.simx_opmode_discontinue)

    return jointMatrix

def getDistanceFromPointToLine(p, a, b):
    p1 = np.array(p, dtype=np.float32)
    a1 = np.array(a, dtype=np.float32)
    b1 = np.array(b, dtype=np.float32)
    return norm(np.cross(a1-p1, p1-b1))/norm(a1-p1)

def getAngleFromTwoVectors(v1, v2, acute=1):
    # v1 is your firsr vector
    # v2 is your second vector
    angle = np.arccos(np.dot(v1, v2) / (norm(v1) * norm(v2)))
    if (acute):
        return angle
    else:
        return 2 * np.pi - angle


################################################################################################################
# V-REP IMPORTANT KNOWLEDGE:
#
# 1 - CALL A CHILD SCRIPT FUNCTION -----------------------------------------------------------------------------
#
# http://www.forum.coppeliarobotics.com/viewtopic.php?f=7&t=5784
# Hello Jay,
#
# actually you can call functions in a threaded child script, from a remote API client. The documentation wasn't updated for last release.
# However:
#
# When you call a script function from outside (e.g. from a remote API client), then the function call will always be blocking, i.e.
# V-REP will run the function, but simulation will halt until the function has finished.
#
# So keeping in mind that the function call will block, you should use rather following scenario:
#
#     From the remote API client call a script function. In that script function, execute additional function calls, memorize variables, etc.
#     But don't spend too much time in here. since during the whole time, V-REP will block. Once you are done, you can set a variable or signal.
#     Once the threaded child script resumes normally, you can check if that variable or signal is set. If yes, then you can execute a longer
#     blocking operation, without having the simulation itself stop.
#
# This is actually illustrated in the scene that you can find here (in next release you will find that scene in scenes/motionPlanningServerDemo.ttt), and its remote API counterpart that you can find in programming/remoteApiBindings/python/python/pathPlanningTest.py


# 2 - SYNCHRONOUS MODE BETWEEN CLIENT & V-REP SERVER ------------------------------------------------------------
#
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

