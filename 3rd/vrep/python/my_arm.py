# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simExtRemoteApiStart(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

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

import time

import robotCommon as RC
from RobotOperationEnv import RobotOperationEnvironment


CSERVER_PORT = 19999
CSERVER_ROBOT_NAME = 'youBot#' #'LBR4p#'
RC.GB_CSERVER_ROBOT_ID = RC.CYOUBOT
##############################################################################################################################################################
##############################################################################################################################################################

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1', CSERVER_PORT,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Start the simulation:
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)

    # Load a robot instance:    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'loadRobot',[],[0,0,0,0],['d:/v_rep/qrelease/release/test.ttm'],emptyBuff,vrep.simx_opmode_oneshot_wait)
    #    robotHandle=retInts[0]

    # Get scene objects data
    res, objHandles, intData, floatData, objNames = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type, 0, vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objHandles), len(objNames))
        for i in range(len(objHandles)):
            print('Obj:', objHandles[i], objNames[i])
    else:
        print ('Remote API function call returned with error code: ',res)

    # Retrieve some handles:
    res, robotHandle = vrep.simxGetObjectHandle(clientID, CSERVER_ROBOT_NAME, vrep.simx_opmode_oneshot_wait)

    env = RobotOperationEnvironment(clientID, RC.GB_CSERVER_ROBOT_ID, robotHandle)
    env.mainRobotTraining()

    ## -----------------------------------------------------------------------------------------------------------------------------
    # Now send some data to V-REP in a non-blocking fashion:
    env.showStatusBarMessage('Hello V-REP')

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')

print ('Pam ended')
