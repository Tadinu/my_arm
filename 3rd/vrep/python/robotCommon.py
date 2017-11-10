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

GB_TRACE = 0
GB_MODE_TRAINING = 1 # 1: Training, 0: Enjoying/Running/Testing
GB_MODE_ENJOYING = 0

# ================================================================
# SERVER REMOTE API OBJECT NAME ----------------------------------
#
CSERVER_REMOTE_API_OBJECT_NAME = 'remoteApiCommandServer'

# ================================================================
# ROBOT IDS ------------------------------------------------------
#
CYOUBOT = 1
CJACO_ARM_HAND = 2
CKUKA_ARM_BARRETT_HAND = 3
CUR5_ARM_BARRETT_HAND = 4

# ================================================================
# SERVER ROBOT ID !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#
GB_CSERVER_ROBOT_ID = CKUKA_ARM_BARRETT_HAND #CUR5_ARM_BARRETT_HAND
GB_CSERVER_ROBOT_NAME = ''

# ================================================================
# ROBOT NAMES ----------------------------------------------------
#
CUR5_ARM_NAME  = 'UR5'
CKUKA_ARM_NAME = 'LBR_iiwa_7_R800' # 'LBR_iiwa_14_R820' #
CYOUBOT_NAME   = 'youBot'# 'LBR4p'
CJACO_ARM_HAND_NAME = 'JacoHand'
CBARRETT_HAND_NAME = 'BarrettHand'

# ================================================================
# OBJECT NAMES ---------------------------------------------------
#
CFALL_OBJS_NAMES = ['Obj1']
CPLATE_OBJ_NAME = 'Plate'
CTABLE_OBJ_NAME = 'Table'
CTUBE_OBJ_NAME  = 'Tube'

# ================================================================
# TRAINING TASK NAMES --------------------------------------------
#
CTASK_ID_OBJ_BALANCE = 1
CTASK_ID_OBJ_HOLD    = 2
GB_TASK_ID = CTASK_ID_OBJ_HOLD

def isTaskObjBalance():
    return GB_TASK_ID == CTASK_ID_OBJ_BALANCE

def isTaskObjHold():
    return GB_TASK_ID == CTASK_ID_OBJ_HOLD

# ================================================================
# ACTION, STATE DIMENSIONS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#
GB_ACTION_DIM = 1
GB_STATE_DIM  = 10

if(GB_CSERVER_ROBOT_ID == CKUKA_ARM_BARRETT_HAND):
    # 9 (7 Kuka arm joints & 2 Hand finger angle)
    if(isTaskObjBalance()):
        # 3 (1 Arm Wrist joint, 2 revolute hand finger base joints) OR
        GB_ACTION_DIM = 3
        GB_STATE_DIM  = 11
    elif(isTaskObjHold()):
        # 2 Hand open Close Joints (force) & 2 revolute hand finger base joints (vel)
        GB_ACTION_DIM = 4
        GB_STATE_DIM  = 13

    GB_CSERVER_ROBOT_NAME = CKUKA_ARM_NAME

elif(GB_CSERVER_ROBOT_ID == CUR5_ARM_BARRETT_HAND):
    GB_ACTION_DIM = 8 # 8(6 Kuka arm joints & 2 Hand finger angle)
    GB_STATE_DIM  = 12
    GB_CSERVER_ROBOT_NAME = CUR5_ARM_NAME

elif(GB_CSERVER_ROBOT_ID == CJACO_ARM_HAND):
    GB_ACTION_DIM = 1
    GB_STATE_DIM  = 10
    GB_CSERVER_ROBOT_NAME = CJACO_ARM_HAND_NAME

def init(clientID):
    global GB_CLIENT_ID
    GB_CLIENT_ID = clientID
    #print('CLIENTID', GB_CLIENT_ID)

def startSimulation(clientID):
    # Set Simulation in Synchronous mode
    vrep.simxSynchronous(clientID,True)

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
    RC.stopSimulation(clientID)

    # Before closing the connection to V-REP,
    #make sure that the last command sent out had time to arrive.
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
    print('V-REP Server Connection closed...')

def getObjectWorldPosition(objectName):
    res, objectHandle = vrep.simxGetObjectHandle(GB_CLIENT_ID, objectName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the object position:
    res, objectPos    = vrep.simxGetObjectPosition(GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    # Retrieves the time needed for a command to be sent to the server,
    # executed, and sent back.
    # That time depends on various factors like the client settings,
    # the network load, whether a simulation is running, whether the
    # simulation is real-time, the simulation time step, etc.
    # The function is blocking.
    vrep.simxGetPingTime(GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, objectPos    = vrep.simxGetObjectPosition(GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_buffer)
    return objectPos

def getObjectOrientation(objectName):
    res, objectHandle = vrep.simxGetObjectHandle(GB_CLIENT_ID, objectName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the object position:
    res, eulerAngles  = vrep.simxGetObjectOrientation(GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, eulerAngles  = vrep.simxGetObjectOrientation(GB_CLIENT_ID, objectHandle, -1, vrep.simx_opmode_buffer)
    return eulerAngles

def getObjectVelocity(objectName):
    res, objectHandle = vrep.simxGetObjectHandle(GB_CLIENT_ID, objectName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the object position:
    res, objectLinearVel, objectAngVel = vrep.simxGetObjectVelocity(GB_CLIENT_ID, objectHandle, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, objectLinearVel, objectAngVel = vrep.simxGetObjectVelocity(GB_CLIENT_ID, objectHandle, vrep.simx_opmode_buffer)

    return objectLinearVel

def getJointVelocity(jointHandle):
    # http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
    # 2012: vrep.sim_jointfloatparam_velocity
    res, jointVel = vrep.simxGetObjectFloatParameter(GB_CLIENT_ID, jointHandle, 2012, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, jointVel = vrep.simxGetObjectFloatParameter(GB_CLIENT_ID, jointHandle, 2012, vrep.simx_opmode_buffer)

    return jointVel

def getJointForce(jointHandle):
    #res, jointHandle = vrep.simxGetObjectHandle(GB_CLIENT_ID, jointName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the joint force:
    res, jointForce = vrep.simxGetJointForce(GB_CLIENT_ID, jointHandle, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, jointForce = vrep.simxGetJointForce(GB_CLIENT_ID, jointHandle, vrep.simx_opmode_buffer)

    return jointForce

def getJointPosition(jointHandle):
    #res, jointHandle = vrep.simxGetObjectHandle(GB_CLIENT_ID, jointName, vrep.simx_opmode_oneshot_wait)

    # Enabled streaming of the joint position:
    res, jointPos = vrep.simxGetJointPosition(GB_CLIENT_ID, jointHandle, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, jointPos = vrep.simxGetJointPosition(GB_CLIENT_ID, jointHandle, vrep.simx_opmode_buffer)

    return jointPos

def getJointMatrix(jointHandle):
    res, jointMatrix = vrep.simxGetJointMatrix(GB_CLIENT_ID, jointHandle, vrep.simx_opmode_streaming)

    # Wait until the first data has arrived (just any blocking funtion):
    vrep.simxGetPingTime(GB_CLIENT_ID)

    # Now you can read the data that is being continuously streamed:
    res, jointMatrix = vrep.simxGetJointMatrix(GB_CLIENT_ID, jointHandle, vrep.simx_opmode_buffer)

    return jointMatrix

