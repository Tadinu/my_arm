#include <QThread>
#include <QtConcurrent/QtConcurrentRun>
#include <QMutex>
#include <QFuture>

#include "RbMainWindowAgent.h"
#include "RbGlobal.h"
#include "VREPAdapter.h"

#ifdef ROBOT_SENSORS
#include "RbRobotSensorAdapter.h"
#endif

#define ARRAY_SIZE(x)   (sizeof(x) / sizeof((x)[0]))

RbMainWindowAgent* RbMainWindowAgent::_instance = nullptr;

// --- CalibrationDialog ---
RbMainWindowAgent::RbMainWindowAgent(int argc, char **argv, QObject *parent)
    :QObject(parent),
     _robotThread(new RbRobotManager(argc, argv))
{
    _robotThread->startThreading();
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
}

void RbMainWindowAgent::initializeQMLItemAgents()
{
    // Robot & Sensor Agents (must be done while this RbRobotManager is still running on Main thread and after root QML component is loaded!)
    this->startRobotAgent();
    this->startSensorAgents();
}

void RbMainWindowAgent::startRobotAgent()
{
    printf("START ROBOT AGENT\n");
    //
    _robotAgent = new RbRobotAgent();
    connect(_robotAgent, &RbRobotAgent::queryOrientation, _robotThread, &RbRobotManager::queryRobotOrientation);
    _robotAgent->startThreading(); // Also run its state machine inside (being connected to QThread::started())
}

bool RbMainWindowAgent::isRobotAgentHalted()
{
    return _robotAgent->isHalted();
}

void RbMainWindowAgent::startSensorAgents()
{
    printf("START SENSOR AGENTS\n");
#ifdef ROBOT_SENSORS
    RB_SENSOR_SYSTEM()->initialize();
    //
    const QVector<RbSensorAgent*>& sensorAgentList = RB_SENSOR_SYSTEM()->sensorAgentList();
    for(int i = 0; i < sensorAgentList.size(); i++) {
        connect(sensorAgentList[i], &RbSensorAgent::querySensorData, _robotThread, &RbRobotManager::queryRobotSensorData);
    }
    //
    RB_SENSOR_SYSTEM()->runSensorOperation(); // Run their own state machine inside (being connected to QThread::started())
#endif
}

bool RbMainWindowAgent::isSensorAgentsHalted()
{
#ifdef ROBOT_SENSORS
    return RB_SENSOR_SYSTEM()->isHalted();
#else
    return false;
#endif
}

QVariant RbMainWindowAgent::getFrontVisionSensorImageId()
{
    return QMLAdapter::getInstance()->frontVisionImageProvider()->getImageId(0);
}

QVariant RbMainWindowAgent::getGroundVisionSensorImageId()
{
    return QMLAdapter::getInstance()->groundVisionImageProvider()->getImageId(0);
}
