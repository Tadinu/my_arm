#ifndef _RB_ROBOT_THREAD_H_
#define _RB_ROBOT_THREAD_H_

#include <QtCore>
#include <QThread>
#include <QVector3D>
#include <QQuaternion>
#include <QStringList>
#include <QMutex>
#include <QImage>
#include <iostream>
#include <assert.h>

#include "RbStateMachine.h"
#include "RbRobotAgent.h"

class RbRobotManager : public QObject {
    Q_OBJECT
    Q_ENUMS(RB_ROBOT_MANAGEMENT_STATE)

public:
    RbRobotManager();
    virtual ~RbRobotManager();

    enum RB_ROBOT_MANAGEMENT_STATE {
        INITIALIZED,
        ROBOT_CHECK,
        SENSOR_CHECK,
        SENSOR_SIGNAL_HANDLE,

        RB_ROBOT_MANAGE_STATE_TOTAL
    };

    static RbRobotManager* getInstance();
    static void deleteInstance();
    static bool checkInstance();

    // ##############################################################################
    // STATE MACHINE METHODS & PROPERTIES -------------------------------------------
    //
    bool fTrue() { return true; }
    void initializeStateMachine();
    void runStateMachineOperation();

    RbSMRule<RbRobotManager>* getStateMachine() {
        return _stateMachine;
    }

    int getCurrentStateRuleId()                 { return _currentStateRuleId;        }
    void setCurrentStateRuleId(int stateRuleId) { _currentStateRuleId = stateRuleId; }

    // Functionality methods --
    void initState();
    bool isRobotComFaulted();
    bool isSensorFaulted();
    void troubleshootRobotCom();
    void troubleshootSensors();

    QTimer* getServiceTimer() { return _serviceTimer; }

public slots:
    void runTask();
    void setServiceTimeout(int timeout);
    void setFallingObjTimeInterval(int timeInterval);

    // Robot Agent service -----------------
    void startRobotAgent();
    bool isRobotAgentHalted();

    void queryRobotOrientation();
    void setRobotVel(float vel);

    RbRobotAgent* getRobotAgent() { return _robotAgent; }

    // Sensor Agent service ----------------
    void startSensorAgents();
    bool isSensorAgentsHalted();

    void queryRobotSensorData(int sensorType, int sensorId);
    QImage queryRobotCameraData(const char* visionSensorName);

signals:

private:
    int _init_argc;
    char** _pInit_argv;

private:
    int _currentStateRuleId;
    RbSMRule<RbRobotManager>* _stateMachine; // pointing to static defined data
    RbRobotAgent* _robotAgent;

    // ##############################################################################
    // THREADING --------------------------------------------------------------------
    //
public:
    bool startThreading();

private:
    static RbRobotManager* _instance;
    QTimer* _serviceTimer;
    QThread *_thread;
    QMutex* _mutex;
};
#endif

