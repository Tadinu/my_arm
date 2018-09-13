#include <QThread>
#include <QtConcurrent/QtConcurrentRun>
#include <QMutex>
#include <QFuture>

#include "RbMainWindowAgent.h"
#include "RbGlobal.h"
#include "VREPAdapter.h"
#include "RbRobotManager.h"

#ifdef ROBOT_SENSORS
#include "RbRobotSensorAdapter.h"
#endif

#define ARRAY_SIZE(x)   (sizeof(x) / sizeof((x)[0]))

RbMainWindowAgent* RbMainWindowAgent::_instance = nullptr;

// --- CalibrationDialog ---
RbMainWindowAgent::RbMainWindowAgent(int argc, char **argv, QObject *parent)
    :QObject(parent)
{
}

RbMainWindowAgent::~RbMainWindowAgent()
{
    // RbRobotSensorAdapter --
    //
    RbRobotSensorAdapter::deleteInstance();
}

RbMainWindowAgent *RbMainWindowAgent::getInstance(int argc, char **argv)
{
    if (NULL == _instance) {
        _instance = new RbMainWindowAgent(argc, argv, nullptr);

        Q_ASSERT(_instance != NULL);
    }

    return _instance;
}

void RbMainWindowAgent::deleteInstance()
{
    if (NULL != _instance) {
        delete _instance;
        _instance = nullptr;
    }
}

void RbMainWindowAgent::setQmlCom(QObject* qmlCom)
{
    assert(qmlCom != nullptr);
    RbGlobal::regQMLCom(qmlCom);
}

void RbMainWindowAgent::initializeQMLContent()
{
    this->initializeQMLItemAgents();
    //RB_QML_INVOKE_I(setWindowTitle, "Robot Sensors");
    this->startRobotManager();
}

void RbMainWindowAgent::initializeQMLItemAgents()
{}

void RbMainWindowAgent::startRobotManager()
{
    // 0- Robot & Sensor Agents (must be done while this RbRobotManager is still running on Main thread and after root QML component is loaded!)
    RbRobotManager* robotManager = RbRobotManager::getInstance();
    robotManager->startRobotAgent();
    robotManager->startSensorAgents();

    // 1-
    robotManager->startThreading();

    // 2- (After robot manager thread starts
    connect(this, &RbMainWindowAgent::robotVelUpdateOrdered,    robotManager, &RbRobotManager::setRobotVel);
    connect(this, &RbMainWindowAgent::robotQueryTimeoutUpdated, robotManager, &RbRobotManager::setServiceTimeout);
    connect(this, &RbMainWindowAgent::fallingObjTimeInvervalUpdated, robotManager, &RbRobotManager::setFallingObjTimeInterval);
}

QVariant RbMainWindowAgent::getFrontVisionSensorImageId()
{
    return QMLAdapter::getInstance()->frontVisionImageProvider()->getImageId(0);
}

QVariant RbMainWindowAgent::getGroundVisionSensorImageId()
{
    return QMLAdapter::getInstance()->groundVisionImageProvider()->getImageId(0);
}

void RbMainWindowAgent::setRobotVel(float vel)
{
    printf("[Order] Reset robot velocity...\n");
    emit robotVelUpdateOrdered(vel);
}

void RbMainWindowAgent::setRobotQueryTimeout(int timeout)
{
    printf("[Order] Reset robot query timeout...\n");
    emit robotQueryTimeoutUpdated(timeout);
}

void RbMainWindowAgent::setFallingObjectTimeInterval(int timeInterval)
{
    printf("[Order] Reset falling obj time interval...\n");
    emit fallingObjTimeInvervalUpdated(timeInterval);
}
