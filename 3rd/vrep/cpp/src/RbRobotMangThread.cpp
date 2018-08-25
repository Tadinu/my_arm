#include <string>
#include <QMutexLocker>
#include <QtMath>
#include <mutex>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
#include "RbRobotMangThread.h"
#include "RbGlobal.h"
#include "VREPAdapter.h"
#include "QMLAdapter.h"
#include "RbMainWindowAgent.h"
#include "RbRobotSensorAdapter.h"

//#include "bullet_server.h"

// ROBOT TO RUN
#define CRUN_ROBOT (RbGlobal::RB_ROBOT_CAR)

const QVector3D CX(1,0,0);
const QVector3D CY(0,1,0);
const QVector3D CZ(0,0,1);

const QVector3D CZERO(0,0,0);

void RbRobotMangThread::initializeStateMachine()
{
    static RbSMRule<RbRobotMangThread> machine[RbRobotMangThread::RB_ROBOT_MANAGE_STATE_TOTAL] =
    {
        /* 0 */ { &RbRobotMangThread::fTrue            , false, &RbRobotMangThread::initState,            0, 1 },
        /* 1 */ { &RbRobotMangThread::isRobotComFaulted, false, &RbRobotMangThread::troubleshootRobotCom, 0, 2 },
        /* 2 */ { &RbRobotMangThread::isSensorFaulted  , false, &RbRobotMangThread::troubleshootSensors,  1, 1 },
    };
    _stateMachine = machine;
    _currentStateRuleId = 0;
}

void RbRobotMangThread::runStateMachineOperation()
{
    int currentStateId = RbStateMachine<RbRobotMangThread>::run(this);
    if (currentStateId > 0) {
        this->setCurrentStateRuleId(currentStateId);
    }
}

// ==========================================================================================
//
RbRobotMangThread::RbRobotMangThread(int argc, char** pArgv)
            : _init_argc(argc),
              _pInit_argv(pArgv),
              _robotId(CRUN_ROBOT),
              _robot_pos(QVector3D(0,0,0)),

              _thread(nullptr),
              _mutex(nullptr),
              _currentStateRuleId(0)
{
    // Own state machine --
    initializeStateMachine();
}

RbRobotMangThread::~RbRobotMangThread()
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

bool RbRobotMangThread::startThreading()
{
    _thread = new QThread();
    _mutex = new QMutex();
    // -----------------------------------------------------------
    // MOVE THIS TO THE THREAD
    this->setParent(nullptr);
    this->moveToThread(_thread); // MAIN THREAD --> ROBOT MANAGEMENT THREAD

    // -----------------------------------------------------------
    // Start Service Timer, note that RbRobotMangThread may be blocked by its possible internal loop
    //
    _serviceTimer = new QTimer();
    _serviceTimer->setInterval(500);
    connect(_serviceTimer, &QTimer::timeout, this, &RbRobotMangThread::runTask);
    _serviceTimer->start();

    // -----------------------------------------------------------
    // START EVENT LOOP OF THE THREAD:
    _thread->start(); // --> run() --> exec(), starts its internal event loop

    return true;
}//set up the thread

void RbRobotMangThread::runTask()
{
    // Run Robot Operation --
    this->runStateMachineOperation();
    RbMainWindowAgent::getInstance()->getRobotAgent()->runTask();

    const QVector<RbSensorAgent*>& sensorAgentList = RB_SENSOR_SYSTEM()->sensorAgentList();
    for(int i = 0; i < sensorAgentList.size(); i++) {
        sensorAgentList[i]->runTask();
    }
}

// =========================================================================================
// ROBOT AGENT SERVICE FUNCTIONS -----------------------------------------------------------
//
void RbRobotMangThread::queryRobotOrientation()
{
    // Orientation --
    QVector3D orient(VREP_INSTANCE()->getObjectOrientation(RB_NAME));
    // --------------------------------------------------------------------
    // UPDATE TO UI
    RbMainWindowAgent::getInstance()->getRobotAgent()->updateOrientationUI(orient);
}

// SENSOR AGENT SERVICE FUNCTIONS -----------------------------------------------------------
//
#define VREP_QUERY_SENSOR_DATA_CALL(opMode)  simxCallScriptFunction(VREP_INSTANCE()->vrepClientId(), CB::CSERVER_REMOTE_API_OBJECT_NAME, \
                                                                    sim_scripttype_childscript,                                          \
                                                                    "getSensorDataFromClient",                                           \
                                                                    2, inputInts,                                                        \
                                                                    1, inputFloats,                                                      \
                                                                    1, inputStrings[0],                                                  \
                                                                    1, inputBuffer,                                                      \
                                                                    nullptr, &outputInts,                                                \
                                                                    &outputFloatsCnt, &outputFloats,                                     \
                                                                    nullptr, &outputStrings,                                             \
                                                                    nullptr, &outputBuffer,                                              \
                                                                    opMode                                                               \
                                                                    )
void RbRobotMangThread::queryRobotSensorData(int sensorType, int sensorId)
{
    // -------------------------------------------------------------------
    QList<QVariant> sensorData;  // Uset QList, not QVector to pass to QML (Reference QVariant support type)
    int inputInts[2]           = {sensorType, sensorId};
    float inputFloats[]        = {0.0f};
    const char* inputStrings[] = {""};
    simxUChar inputBuffer[]    = {0};

    int *outputInts;
    int outputFloatsCnt = 0;
    float *outputFloats;
    simxChar* outputStrings = {""};
    simxUChar *outputBuffer;
#if 1
    int res = VREP_QUERY_SENSOR_DATA_CALL(simx_opmode_oneshot_wait);
#else
    int res = VREP_QUERY_SENSOR_DATA_CALL(simx_opmode_streaming);
    // Wait until the first data has arrived (just any blocking funtion):
    VREP_INSTANCE()->waitForCommandSentToServer();
    res = VREP_QUERY_SENSOR_DATA_CALL(simx_opmode_buffer);
    res = VREP_QUERY_SENSOR_DATA_CALL(simx_opmode_discontinue);
#endif

    if(res == simx_return_ok) {
        for (int i = 0; i < outputFloatsCnt; i++) {
            //printf("%d - Data [%d]: %f\n", sensorId, i, outputFloats[i]);
            sensorData << outputFloats[i];
        }
    }
    else {
        printf("Failed calling VREP!- %d - %d\n", inputInts[0], inputInts[1]);
    }
    // --------------------------------------------------------------------
    // UPDATE TO UI
    if(sensorType == RbGlobal::RB_SENSOR_TYPE_ULTRASONIC) {
        //printf("%d - %d",  this->id()-4, sensorData[0] != 0);
        RB_QML_INVOKE_II(setUltraSonicSensorArrowVisible, sensorId-4, sensorData[0] != 0);
    }
}

void RbRobotMangThread::initState() {}
bool RbRobotMangThread::isRobotComFaulted()    { return true;}
bool RbRobotMangThread::isSensorFaulted()      { return true;}
void RbRobotMangThread::troubleshootRobotCom() { /* Call to RobotAgent trouble shooting function. */ }
void RbRobotMangThread::troubleshootSensors()  { /* Call to SensorAgent trouble shooting function.*/ }
