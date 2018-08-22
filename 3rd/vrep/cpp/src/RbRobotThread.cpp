#include <string>
#include <QMutexLocker>
#include <QtMath>
#include <mutex>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
#include "RbRobotThread.h"
#include "RbGlobal.h"
#include "VREPAdapter.h"

#ifdef ROBOT_SENSORS
#include "RbRobotSensorAdapter.h"
#endif
//#include "bullet_server.h"

// ROBOT TO RUN
#define CRUN_ROBOT (RbGlobal::RB_ROBOT_CAR)

const QVector3D CX(1,0,0);
const QVector3D CY(0,1,0);
const QVector3D CZ(0,0,1);

const QVector3D CZERO(0,0,0);

void RbRobotThread::initializeStateMachine()
{
    //static RbSMRule<RbRobotThread> machine[RbRobotThread::RB_ROBOT_MANAGEMENT_TOTAL] =
    //{
    //    /* 0 */ { &RbRobotAgent::fTrue,          false, &RbRobotAgent::init,    0, 1 },
    //    /* 1 */ { &RbRobotAgent::checkIdle,      false, &RbRobotAgent::operate, 0, 2 },
    //    /* 2 */ { &RbRobotAgent::checkOperating, false, &RbRobotAgent::goIdle,  1, 1 }
    //};
    //_stateMachine = machine;
}

void RbRobotThread::startStateMachineOperation()
{
    int currentStateId = RbStateMachine<RbRobotThread>::run(this);
    if (currentStateId > 0) {
        this->setCurrentStateRuleId(currentStateId);
    }
}

// ==========================================================================================
RbRobotThread::RbRobotThread(int argc, char** pArgv)
            : _init_argc(argc),
              _pInit_argv(pArgv),
              _robotId(CRUN_ROBOT),

              _robot_pos(QVector3D(0,0,0)),
              _pMutex(new QMutex(QMutex::Recursive))
{
    initializeStateMachine();
}

RbRobotThread::~RbRobotThread()
{
    // RbRobotSensorAdapter --
    //
    RbRobotSensorAdapter::deleteInstance();

    // Local resource mutex --
    //
    _pMutex->tryLock(5000);
    _pMutex->unlock();

    if(!_pThread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
    {
        qWarning("Thread deadlock detected, bad things may have happened !!!");
        _pThread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        _pThread->wait(); //Note: We have to wait again here!
    }

    delete _pMutex;
} //end destructor

bool RbRobotThread::init()
{
    _pThread = new QThread();
    this->moveToThread(_pThread);

    //m_pMutex = new QMutex();

    connect(_pThread, &QThread::started, this, &RbRobotThread::run);
    _pThread->start();

    // -----------------------------------------------------------
    // Start Service Timer, which must be run on main thread since RbRobotThread is blocked by its internal ros loop
    //
    _serviceTimer = new QTimer();
    _serviceTimer->setInterval(500);
    QObject::connect(_serviceTimer, SIGNAL(timeout()), this, SLOT(onCommonServiceTimeout()));
    //_serviceTimer->start();

    // -----------------------------------------------------------
    // ROBOT SENSOR SYSTEM - INITIALIZE --------------------------
#ifdef ROBOT_SENSORS
    RB_SENSOR_SYSTEM()->initialize();
#endif

    return true;
}//set up the thread

void RbRobotThread::run()
{
    while(true) {
        for(VUInt8 i = RbRobotSensorAdapter::RB_SENSOR_FIRST; i < RbRobotSensorAdapter::RB_SENSOR_TOTAL; i++) {
            QVector<float> sensorData = RB_SENSOR_SYSTEM()->getSensorData(i);
            //printf("%s - %d: %d\n", RB_SENSOR_SYSTEM()->sensorAgentList()[i]->name(),
            //                        RB_SENSOR_SYSTEM()->sensorAgentList()[i]->id(),
            //                        sensorData.size());
        }
        printf("=======================================\n");
    }

    // Start Robot Operation --
    //this->runOperation(CRUN_ROBOT);
}

void RbRobotThread::onCommonServiceTimeout()
{
    _pMutex->lock();
    _pMutex->unlock();
}


// =========================================================================================
