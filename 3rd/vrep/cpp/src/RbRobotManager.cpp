#include <string>
#include <QMutexLocker>
#include <QtMath>
#include <mutex>
#include <QByteArray>
#include <QString>

//#include <Eigen/Core>
//#include <Eigen/Geometry>
#include "RbRobotManager.h"
#include "RbGlobal.h"
#include "VREPAdapter.h"
#include "QMLAdapter.h"
#include "RbMainWindowAgent.h"
#include "RbRobotSensorAdapter.h"

//#include "bullet_server.h"

// ROBOT TO RUN
#define CRUN_ROBOT (RbGlobal::RB_ROBOT_CAR)
#define CORIG_ROBOT_QUERY_TIME_INTERVAL (500)

const QVector3D CX(1,0,0);
const QVector3D CY(0,1,0);
const QVector3D CZ(0,0,1);

const QVector3D CZERO(0,0,0);

void RbRobotManager::initializeStateMachine()
{
    static RbSMRule<RbRobotManager> machine[RbRobotManager::RB_ROBOT_MANAGE_STATE_TOTAL] =
    {
        /* 0 */ { &RbRobotManager::fTrue            , false, &RbRobotManager::initState,            0, 1 },
        /* 1 */ { &RbRobotManager::isRobotComFaulted, false, &RbRobotManager::troubleshootRobotCom, 0, 2 },
        /* 2 */ { &RbRobotManager::isSensorFaulted  , false, &RbRobotManager::troubleshootSensors,  1, 0 }
    };
    _stateMachine = machine;
    _currentStateRuleId = 0;
}

void RbRobotManager::runStateMachineOperation()
{
    int currentStateId = RbStateMachine<RbRobotManager>::run(this);
    if (currentStateId > 0) {
        this->setCurrentStateRuleId(currentStateId);
    }
}

// ==========================================================================================
//
RbRobotManager* RbRobotManager::_instance = nullptr;
RbRobotManager* RbRobotManager::getInstance()
{
    if(_instance == nullptr) {
        _instance = new RbRobotManager();
    }

    return _instance;
}

bool RbRobotManager::checkInstance()
{
    return _instance != nullptr;
}

void RbRobotManager::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

RbRobotManager::RbRobotManager()
            : _thread(nullptr),
              _mutex(nullptr),
              _currentStateRuleId(0)
{
    // Own state machine --
    initializeStateMachine();
}

RbRobotManager::~RbRobotManager()
{
    // Local resource mutex --
    //
    _mutex->tryLock(5000);
    _mutex->unlock();

    if(!_thread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
    {
        qWarning("Thread deadlock detected, abnormal things may have happened !!!");
        _thread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        _thread->wait(); //Note: We have to wait again here!
    }

    delete _mutex;
    delete _serviceTimer;
} //end destructor

bool RbRobotManager::startThreading()
{
    _thread = new QThread();
    _mutex = new QMutex();
    // -----------------------------------------------------------
    // MOVE THIS TO THE THREAD
    this->setParent(nullptr);
    this->moveToThread(_thread); // MAIN THREAD --> ROBOT MANAGEMENT THREAD

    // -----------------------------------------------------------
    // Start Service Timer, note that RbRobotManager may be blocked by its possible internal loop
    //
    _serviceTimer = new QTimer();
    _serviceTimer->setInterval(CORIG_ROBOT_QUERY_TIME_INTERVAL);
    connect(_serviceTimer, &QTimer::timeout, this, &RbRobotManager::runTask);
    _serviceTimer->start();
    _serviceTimer->moveToThread(_thread);
    connect(_thread, &QThread::finished, _serviceTimer, &QObject::deleteLater);

    // -----------------------------------------------------------
    // START EVENT LOOP OF THE THREAD:
    _thread->start(); // --> run() --> exec(), starts its internal event loop

    return true;
}//set up the thread

void RbRobotManager::runTask()
{
    // Run Robot Operation --
    this->runStateMachineOperation();

    // Robot Agent Operation --
    _robotAgent->runTask();

    // Sensor Agents Operation --
    const QVector<RbSensorAgent*>& sensorAgentList = RB_SENSOR_SYSTEM()->sensorAgentList();
    for(int i = 0; i < sensorAgentList.size(); i++) {
        sensorAgentList[i]->runTask();
    }
}

void RbRobotManager::setServiceTimeout(int timeout)
{
    printf("Set Service Timeout %d\n", timeout);
    static bool firstTime = true;
    if(firstTime) {
        firstTime = false;
        // Still on main thread, not allowed to reset timer from another thread
    }
    else {
        _serviceTimer->setInterval(timeout);
        printf("Robot Query service timeout reset %d\n", timeout);
    }
}

// =========================================================================================
// ROBOT AGENT SERVICE FUNCTIONS -----------------------------------------------------------
//

void RbRobotManager::startRobotAgent()
{
    printf("START ROBOT AGENT\n");
    //
    _robotAgent = new RbRobotAgent();
    connect(_robotAgent, &RbRobotAgent::queryOrientation, this, &RbRobotManager::queryRobotOrientation);
    _robotAgent->startThreading(); // Also run its state machine inside (being connected to QThread::started())
}

bool RbRobotManager::isRobotAgentHalted()
{
    return _robotAgent->isHalted();
}

void RbRobotManager::startSensorAgents()
{
    printf("START SENSOR AGENTS\n");
#ifdef ROBOT_SENSORS
    RB_SENSOR_SYSTEM()->initialize();
    //
    const QVector<RbSensorAgent*>& sensorAgentList = RB_SENSOR_SYSTEM()->sensorAgentList();
    for(int i = 0; i < sensorAgentList.size(); i++) {
        connect(sensorAgentList[i], &RbSensorAgent::querySensorData, this, &RbRobotManager::queryRobotSensorData);
    }
    //
    RB_SENSOR_SYSTEM()->runSensorOperation(); // Run their own state machine inside (being connected to QThread::started())
#endif
}

bool RbRobotManager::isSensorAgentsHalted()
{
#ifdef ROBOT_SENSORS
    return RB_SENSOR_SYSTEM()->isHalted();
#else
    return false;
#endif
}
void RbRobotManager::queryRobotOrientation()
{
    // Orientation --
    QVector3D orient(VREP_INSTANCE()->getObjectOrientation(RB_NAME));
    // --------------------------------------------------------------------
    // UPDATE TO UI
    _robotAgent->updateOrientationUI(orient);
    printf("Robot orientation updated: [%f]-[%f]-[%f]\n", orient.x(), orient.y(), orient.z());
}

void RbRobotManager::setRobotVel(float vel)
{
    int inputInts[]           = {};
    float inputFloats[]        = {vel};
    const char* inputStrings[] = {""};
    simxUChar inputBuffer[]    = {};

    int *outputInts;
    float *outputFloats;
    simxChar* outputStrings = {""};
    simxUChar *outputBuffer;

    int res = simxCallScriptFunction(VREP_INSTANCE()->vrepClientId(), CB::CSERVER_REMOTE_API_OBJECT_NAME,
                                     sim_scripttype_childscript,
                                     "setMaxVelocityFromClient",
                                     0, inputInts,
                                     1, inputFloats,
                                     1, inputStrings[0],
                                     0, inputBuffer,
                                     nullptr, &outputInts,
                                     nullptr, &outputFloats,
                                     nullptr, &outputStrings,
                                     nullptr, &outputBuffer,
                                     simx_opmode_oneshot_wait
                                     );
   if(res == simx_return_ok)
        printf("Set Robot Vel %f [deg/sec]\n", vel);
   else
        printf("[%d - Failed call to V-REP Server] on Set Robot Vel %f [deg/sec]\n", res, vel);
}

void RbRobotManager::setFallingObjTimeInterval(int timeInterval)
{
    int inputInts[1]           = {timeInterval};
    float inputFloats[]        = {};
    const char* inputStrings[] = {""};
    simxUChar inputBuffer[]    = {};

    int *outputInts;
    float *outputFloats;
    simxChar* outputStrings = {""};
    simxUChar *outputBuffer;

   int res = simxCallScriptFunction(VREP_INSTANCE()->vrepClientId(), CB::CSERVER_REMOTE_API_OBJECT_NAME,
                                    sim_scripttype_childscript,
                                    "setFallingObjTimeIntervalFromClient",
                                    1, inputInts,
                                    0, inputFloats,
                                    1, inputStrings[0],
                                    0, inputBuffer,
                                    nullptr, &outputInts,
                                    nullptr, &outputFloats,
                                    nullptr, &outputStrings,
                                    nullptr, &outputBuffer,
                                    simx_opmode_oneshot_wait
                                    );
   if(res == simx_return_ok)
        printf("Set Falling Obj Time Interval %d [msec]\n", timeInterval);
   else
        printf("[%d - Failed call to V-REP Server] on Set Falling Obj Time Interval %d [msec]\n", res, timeInterval);
}

// SENSOR AGENT SERVICE FUNCTIONS -----------------------------------------------------------
//
#define VREP_QUERY_SENSOR_DATA_CALL(opName, opMode)  simxCallScriptFunction(VREP_INSTANCE()->vrepClientId(), CB::CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                    sim_scripttype_childscript,                                          \
                                                                    opName,                                                              \
                                                                    2, inputInts,                                                        \
                                                                    1, inputFloats,                                                      \
                                                                    1, inputStrings[0],                                                  \
                                                                    1, inputBuffer,                                                      \
                                                                    &outputIntsCnt, &outputInts,                                                \
                                                                    &outputFloatsCnt, &outputFloats,                                     \
                                                                    nullptr, &outputStrings,                                             \
                                                                    nullptr, &outputBuffer,                                              \
                                                                    opMode                                                               \
                                                                    )
void RbRobotManager::queryRobotSensorData(int sensorType, int sensorId)
{
    // -------------------------------------------------------------------
    QList<QVariant> sensorData;  // Uset QList, not QVector to pass to QML (Reference QVariant support type)
    int inputInts[2]           = {sensorType, sensorId};
    float inputFloats[]        = {0.0f};
    const char* inputStrings[] = {""};
    simxUChar inputBuffer[]    = {0};

    int outputIntsCnt = 0;
    int *outputInts;
    int outputFloatsCnt = 0;
    float *outputFloats;
    simxChar* outputStrings = {""};
    simxUChar *outputBuffer;

    const char* sensorName = RB_SENSOR_SYSTEM()->sensorAgentList()[sensorId]->name();
    // TACTILE SENSOR TYPES ------------------------------------------------
    switch(sensorType) {

    case RbGlobal::RB_SENSOR_TYPE_TACTILE:
    {
        printf("Query Robot tactile sensor data...\n");
        int res = VREP_QUERY_SENSOR_DATA_CALL("detectCollisionWithObjectFromClient", simx_opmode_oneshot_wait);
        if(res == simx_return_ok && outputIntsCnt > 0) {
            static int lastState = _robotAgent->getUIState();

            int newState = (outputInts[0] != 0)? RbRobotAgent::COLLIDING : lastState;
            //printf("NEWSTATE: %d", newState);
            if(newState != lastState) {
                _robotAgent->setUIState(newState);
                printf("Robot state updated: %d\n", newState);
                if(newState == RbRobotAgent::COLLIDING)
                    printf("Robot - Object collision detected!\n");
            }
        }
        else {
            printf("(I) Failed calling VREP! - [%d]: %d - %d\n", outputIntsCnt, inputInts[0], inputInts[1]);
        }
    }
    break;

    //case RbGlobal::RB_SENSOR_TYPE_VISION:
    //{
    //    printf("Query Robot Camera data...\n");
    //    RB_SENSOR_SYSTEM()->setVisionSensorImage(sensorId,
    //                                             this->queryRobotCameraData(sensorName));
    //
    //    if(sensorId == RbRobotSensorAdapter::RB_SENSOR_FRONT_VISION)
    //        QMLAdapter::getInstance()->frontVisionImageProvider()->updateFromVisionSensor(sensorName);
    //    else if(sensorId == RbRobotSensorAdapter::RB_SENSOR_GROUND_VISION)
    //        QMLAdapter::getInstance()->groundVisionImageProvider()->updateFromVisionSensor(sensorName);
    //}
    //break;

    // OTHER SENSOR TYPES ------------------------------------------------
    default:
    {
#if 1
        int res = VREP_QUERY_SENSOR_DATA_CALL("getSensorDataFromClient", simx_opmode_oneshot_wait);
#else
        int res = VREP_QUERY_SENSOR_DATA_CALL(simx_opmode_streaming);
        // Wait until the first data has arrived (just any blocking funtion):
        VREP_INSTANCE()->waitForCommandSentToServer();
        res = VREP_QUERY_SENSOR_DATA_CALL(simx_opmode_buffer);
        res = VREP_QUERY_SENSOR_DATA_CALL(simx_opmode_discontinue);
#endif

        if(res == simx_return_ok && outputFloatsCnt > 0) {
            printf("Sensor Data[%s] - Sensor Type[%d] - Sensor Id[%d]: \n", sensorName, sensorType, sensorId);
            for (int i = 0; i < outputFloatsCnt; i++) {
                //printf("%d - Data [%d]: %f\n", sensorId, i, outputFloats[i]);
                sensorData << outputFloats[i];
                printf("%f    ", outputFloats[i]);
            }
            printf("\n================================================\n");

            // --------------------------------------------------------------------
            // UPDATE TO UI
            if(sensorType == RbGlobal::RB_SENSOR_TYPE_ULTRASONIC) {
                RB_QML_INVOKE_II(setUltraSonicSensorArrowVisible,
                                 sensorId-(RbRobotSensorAdapter::RB_SENSOR_ULTRASONIC_START - RbRobotSensorAdapter::RB_SENSOR_FIRST),
                                 sensorData[0] != 0);
            }
        }
        else {
            printf("(II) Failed calling VREP! - [%d]: %d - %d\n", outputFloatsCnt, inputInts[0], inputInts[1]);
        }
    }
    break;
    } // switch(sensorType)
}

void RbRobotManager::initState() {}
bool RbRobotManager::isRobotComFaulted()    { return true;}
bool RbRobotManager::isSensorFaulted()      { return true;}
void RbRobotManager::troubleshootRobotCom() { /* Call to RobotAgent trouble shooting function. */ }
void RbRobotManager::troubleshootSensors()  { /* Call to SensorAgent trouble shooting function.*/ }

#define VREP_GET_VISISON_SENSOR_IMAGE(opMode) simxGetVisionSensorImage(VREP_INSTANCE()->vrepClientId(), camHandle, &reso, (unsigned char**)img, 0, opMode)
QImage RbRobotManager::queryRobotCameraData(const char* visionSensorName)
{
    // Receiving and image from V-REP and sending it back:
    int camHandle;
    int res = simxGetObjectHandle(VREP_INSTANCE()->vrepClientId(), visionSensorName, &camHandle, simx_opmode_oneshot_wait);

    int reso = 512;
    unsigned char img[reso][reso];
    res = VREP_GET_VISISON_SENSOR_IMAGE(simx_opmode_streaming);

    while (simxGetConnectionId(VREP_INSTANCE()->vrepClientId())!=-1) {
        res = VREP_GET_VISISON_SENSOR_IMAGE(simx_opmode_buffer);
    }
    res = VREP_GET_VISISON_SENSOR_IMAGE(simx_opmode_discontinue);

    unsigned char imageData[reso*reso];
    for (int x = 0; x < reso; x++) {
        for (int y = 0; y < reso; y++) {
            imageData[x * reso + y] = img[y][x];
        }
    }

    return QImage::fromData(imageData, reso*reso);
    //#image obtained as a Image object. Use it according to need.
}
