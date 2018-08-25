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
    static RbSMRule<RbRobotThread> machine[RbRobotThread::RB_ROBOT_MANAGE_STATE_TOTAL] =
    {
        /* 0 */ { &RbRobotThread::fTrue,                 false, nullptr,                           0, 1 },
        /* 1 */ { &RbRobotThread::isRobotAgentFaulted,   false, &RbRobotThread::startRobotAgent,   0, 2 },
        /* 1 */ { &RbRobotThread::isSensorAgentsFaulted, false, &RbRobotThread::startSensorAgents, 1, 1 },
    };
    _stateMachine = machine;
}

void RbRobotThread::startStateMachineOperation()
{
    int currentStateId = RbStateMachine<RbRobotThread>::run(this);
    if (currentStateId > 0) {
        this->setCurrentStateRuleId(currentStateId);
    }
}

// ==========================================================================================
//
RbRobotThread::RbRobotThread(int argc, char** pArgv)
            : _init_argc(argc),
              _pInit_argv(pArgv),
              _robotId(CRUN_ROBOT),

              _robot_pos(QVector3D(0,0,0)),
              _mutex(new QMutex(QMutex::Recursive))
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
    _mutex->tryLock(5000);
    _mutex->unlock();

    if(!_thread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
    {
        qWarning("Thread deadlock detected, bad things may have happened !!!");
        _thread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        _thread->wait(); //Note: We have to wait again here!
    }

    delete _mutex;
} //end destructor

bool RbRobotThread::startThreading()
{
    _thread = new QThread();
    this->moveToThread(_thread);

    _mutex = new QMutex();

    connect(_thread, &QThread::started, this, &RbRobotThread::run);
    _thread->start();

    // -----------------------------------------------------------
    // Start Service Timer, which must be run on main thread since RbRobotThread is blocked by its internal ros loop
    //
    _serviceTimer = new QTimer();
    _serviceTimer->setInterval(500);
    QObject::connect(_serviceTimer, SIGNAL(timeout()), this, SLOT(onCommonServiceTimeout()));
    //_serviceTimer->start();

    return true;
}//set up the thread

void RbRobotThread::run()
{
    // Start Robot Operation --
    this->startStateMachineOperation();
}

void RbRobotThread::startSensorAgents()
{
#ifdef ROBOT_SENSORS
    RB_SENSOR_SYSTEM()->initialize();
    RB_SENSOR_SYSTEM()->runSensorOperation();
#endif
}

bool RbRobotThread::isSensorAgentsFaulted()
{
#ifdef ROBOT_SENSORS
    return RB_SENSOR_SYSTEM()->isFaulted();
#else
    return false;
#endif
}

void RbRobotThread::startRobotAgent()
{
    _robotAgent = new RbRobotAgent();
    _robotAgent->startThreading();
}

bool RbRobotThread::isRobotAgentFaulted()
{
    return _robotAgent->isFaulted();
}

void RbRobotThread::onCommonServiceTimeout()
{
    _mutex->lock();
    _mutex->unlock();
}
// =========================================================================================
