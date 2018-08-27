#include <functional> // std::bind
#include <iostream>
using namespace std;

#include <QThread>
#include <QVector3D>
#include "VREPAdapter.h"
#include "RbGlobal.h"

VREPAdapter* VREPAdapter::_instance = nullptr;
VREPAdapter* VREPAdapter::getInstance()
{
    if(_instance == nullptr) {
        _instance = new VREPAdapter();
    }

    return _instance;
}

bool VREPAdapter::checkInstance()
{
    return _instance != nullptr;
}

VREPAdapter::VREPAdapter():
             _vrepMutex(new QMutex()),
             _vrepRobotHandle(-1),
             _vrepClientId(-1)
{
}

VREPAdapter::~VREPAdapter()
{
    _vrepMutex->tryLock(500);
    _vrepMutex->unlock(); // futile if tryLock() failed!
    delete _vrepMutex;
}

void VREPAdapter::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

int VREPAdapter::initializeVREP()
{
     simxFinish(-1); // just in case, close all opened connections
    _vrepClientId = simxStart((simxChar*)"127.0.0.1",CB::CSERVER_PORT,true,true,2000,5);

    if (_vrepClientId == -1) {
        simxFinish(_vrepClientId);
        printf("NOT Connected yet to remote API server\n");
        //cout << "NOT Connected yet to remote API server: " << _vrepClientId;
        return(-1);
    }
    // ---------------------------------------------------------------------

    printf("Connected to remote API server: %d\n", _vrepClientId);

    extApi_sleepMs(5);
    return(0);
}

void VREPAdapter::finalizeVREP()
{
    VREPAdapter::endSimulation(_vrepClientId);
}

void VREPAdapter::startSimulation(int _vrepClientId)
{
    // http://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm#synchronous
    // Set Simulation in Synchronous mode
    // Enables or disables the synchronous operation mode for the remote API server service that the client is connected to.
    // The function is blocking. While in synchronous operation mode, the client application is in charge of triggering the next simulation step. Only pre-enabled remote API server services will successfully execute this function.
    simxSynchronous(_vrepClientId, false);

    // Set Simulation Step Time
    float dt = 0.005;
    simxSetFloatingParameter(_vrepClientId,
                             sim_floatparam_simulation_time_step,
                             dt, simx_opmode_oneshot);
    simxStartSimulation(_vrepClientId, simx_opmode_oneshot_wait);
}

void VREPAdapter::endSimulation(int _vrepClientId)
{
    // Before closing the connection to V-REP,
    // make sure that the last command sent out had time to reach the server.
    waitForCommandSentToServer();

    //simxStopSimulation(_vrepClientId, vrep.simx_opmode_blocking);
    simxFinish(_vrepClientId);
    cout << "V-REP Server Connection closed!";
}

void VREPAdapter::lock()
{
    _vrepMutex->lock();
}

void VREPAdapter::unlock()
{
    _vrepMutex->unlock();
}

int VREPAdapter::waitForCommandSentToServer()
{
    int waitTime = 0;
    simxGetPingTime(_vrepClientId, &waitTime);

    extApi_sleepMs(50);
    return waitTime;
}

bool VREPAdapter::isConnectedToServer()
{
    return (simxGetConnectionId(_vrepClientId)!=-1);
}

void VREPAdapter::testConnectionToServer()
{
    // Now try to retrieve data in a blocking fashion (i.e. a service call):
    int objectCount;
    int* objectHandles;
    int ret=simxGetObjects(_vrepClientId,sim_handle_all,&objectCount,&objectHandles,simx_opmode_blocking);
    if (ret==simx_return_ok)
        printf("Number of objects in the scene: %d\n",objectCount);
    else
        printf("Remote API function call returned with error code: %d\n",ret);

    extApi_sleepMs(2000);

    // Now retrieve streaming data (i.e. in a non-blocking fashion):
    int startTime=extApi_getTimeInMs();
    int mouseX;
    simxGetIntegerParameter(_vrepClientId,sim_intparam_mouse_x,&mouseX,simx_opmode_streaming); // Initialize streaming
    while (extApi_getTimeDiffInMs(startTime) < 5000)
    {
        ret=simxGetIntegerParameter(_vrepClientId,sim_intparam_mouse_x,&mouseX,simx_opmode_buffer); // Try to retrieve the streamed data
        if (ret==simx_return_ok) // After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            printf("Mouse position x: %d\n",mouseX); // Mouse position x is actualized when the cursor is over V-REP's window
    }

    // Now send some data to V-REP in a non-blocking fashion:
    simxAddStatusbarMessage(_vrepClientId,"Hello V-REP!",simx_opmode_oneshot);

    // Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    int pingTime;
    simxGetPingTime(_vrepClientId,&pingTime);

    // Now close the connection to V-REP:
    simxFinish(_vrepClientId);
}

void VREPAdapter::testCommands()
{
    // 1. First send a command to display a specific message in a dialog box:
    int retStringCnt;
    char* retStrings;
    int result=simxCallScriptFunction(_vrepClientId,"remoteApiCommandServer",
                                      sim_scripttype_childscript,
                                      "displayText_function",
                                      0,NULL,
                                      0,NULL,
                                      1,"Hello world!",
                                      0,NULL,
                                      NULL,NULL,
                                      NULL,NULL,
                                      &retStringCnt, &retStrings,
                                      NULL,NULL,
                                      simx_opmode_blocking);
    if (result==simx_return_ok)
        printf("Returned message: %s\n",retStrings); // display the reply from V-REP (in this case, just a string)
    else
        printf("Remote function call failed\n");

    // 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
    float coords[3]={0.1f,0.2f,0.3f};
    int retIntCnt;
    int* retInts;
    result=simxCallScriptFunction(_vrepClientId,"remoteApiCommandServer",
                                  sim_scripttype_childscript,
                                  "createDummy_function",
                                  0,NULL,
                                  3,coords,
                                  1,"MyDummyName",
                                  0,NULL,
                                  &retIntCnt, &retInts,
                                  NULL,NULL,NULL,NULL,NULL,NULL,simx_opmode_blocking);
    if (result==simx_return_ok)
        printf("Dummy handle: %i\n",retInts[0]); // display the reply from V-REP (in this case, the handle of the created dummy)
    else
        printf("Remote function call failed\n");

    // 3. Now send a code string to execute some random functions:
    char* code="local octreeHandle=simCreateOctree(0.5,0,1)\n" \
        "simInsertVoxelsIntoOctree(octreeHandle,0,{0.1,0.1,0.1},{255,0,255})\n" \
        "return 'done'";
    result=simxCallScriptFunction(_vrepClientId,"remoteApiCommandServer",
                                  sim_scripttype_childscript,
                                  "executeCode_function",
                                  0,NULL,
                                  0,NULL,
                                  1,code,
                                  0,NULL,
                                  NULL,NULL,
                                  NULL,NULL,
                                  &retStringCnt,&retStrings,
                                  NULL,NULL,
                                  simx_opmode_blocking);
    if (result==simx_return_ok)
        printf("Code execution returned: %s\n",retStrings);
    else
        printf("Remote function call failed\n");
}

// =============================================================================================================================
#define VREP_RETURN_QVECTOR3D_NOK() if(res != simx_return_ok) { \
                                        return QVector3D(0,0,0); \
                                    }

#define VREP_GET_OBJECT_HANDLE(opMode) simxGetObjectHandle(_vrepClientId, objectName, &objectHandle, opMode)
#define VREP_GET_OBJECT_POS(opMode)    simxGetObjectPosition(_vrepClientId, objectHandle, -1, objectPos, opMode)

QVector3D VREPAdapter::getObjectWorldPosition(const char* objectName)
{
    int objectHandle = -1;
    VUInt8 res = VREP_GET_OBJECT_HANDLE(simx_opmode_oneshot_wait);
    VREP_RETURN_QVECTOR3D_NOK()

    // Enabled streaming of the object position:
    simxFloat objectPos[3];
    res = VREP_GET_OBJECT_POS(simx_opmode_streaming);
    VREP_RETURN_QVECTOR3D_NOK()

    // Wait until the first data has arrived (just any blocking funtion):
    // Retrieves the time needed for a command to be sent to the server,
    // executed, and sent back.
    // That time depends on various factors like the client settings,
    // the network load, whether a simulation is running, whether the
    // simulation is real-time, the simulation time step, etc.
    // The function is blocking.
    waitForCommandSentToServer();

    // Now you can read the data that is being continuously streamed:
    res = VREP_GET_OBJECT_POS(simx_opmode_buffer);
    VREP_RETURN_QVECTOR3D_NOK()

    // Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    // to stream unessesary data and eventually slow down.
    res = VREP_GET_OBJECT_POS(simx_opmode_discontinue);
    VREP_RETURN_QVECTOR3D_NOK()

    return QVector3D(objectPos[0], objectPos[1], objectPos[2]);
}

#define VREP_GET_OBJECT_ORIENT(opMode)  simxGetObjectOrientation(_vrepClientId, objectHandle, -1, eulerAngles, opMode)
QVector3D VREPAdapter::getObjectOrientation(const char* objectName)
{
    int objectHandle = -1;
    VUInt8 res = VREP_GET_OBJECT_HANDLE(simx_opmode_oneshot_wait);
    VREP_RETURN_QVECTOR3D_NOK()

    // Enabled streaming of the object orientation:
    simxFloat eulerAngles[3] = {0,0,0};
    res = VREP_GET_OBJECT_ORIENT(simx_opmode_streaming);

    // Wait until the first data has arrived (just any blocking funtion):
    waitForCommandSentToServer();

    // Now you can read the data that is being continuously streamed:
    res = VREP_GET_OBJECT_ORIENT(simx_opmode_buffer);

    // Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    // to stream unessesary data and eventually slow down.
    res = VREP_GET_OBJECT_ORIENT(simx_opmode_discontinue);

    return QVector3D(eulerAngles[0], eulerAngles[1], eulerAngles[2]);
}

#define VREP_RETURN_FLOAT_NOK() if(res != simx_return_ok) {    \
                                    return 0.0f;               \
                                }


#define VREP_GET_OBJECT_ORIENT(opMode)  simxGetObjectVelocity(_vrepClientId, objectHandle, \
                                        &objectLinearVel, &objectAngVel, opMode)

float VREPAdapter::getObjectVelocity(const char* objectName)
{
    int objectHandle = -1;
    VUInt8 res = simxGetObjectHandle(_vrepClientId, objectName, &objectHandle, simx_opmode_oneshot_wait);
    VREP_RETURN_FLOAT_NOK()

    // Enabled streaming of the object position:
    simxFloat objectLinearVel = 0.0f, objectAngVel = 0.0f;
    res = VREP_GET_OBJECT_ORIENT(simx_opmode_streaming);
    VREP_RETURN_FLOAT_NOK()

    // Wait until the first data has arrived (just any blocking funtion):
    waitForCommandSentToServer();

    // Now you can read the data that is being continuously streamed:
    res = VREP_GET_OBJECT_ORIENT(simx_opmode_buffer);
    VREP_RETURN_FLOAT_NOK()

    // Inform the server (i.e. V-REP) to stop streaming that data, otherwise the server will continue
    // to stream unessesary data and eventually slow down.
    res = VREP_GET_OBJECT_ORIENT(simx_opmode_discontinue);

    return objectLinearVel;
}
