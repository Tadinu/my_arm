#ifndef _RB_ROBOT_THREAD_H_
#define _RB_ROBOT_THREAD_H_

#include <QtCore>
#include <QThread>
#include <QVector3D>
#include <QQuaternion>
#include <QStringList>
#include <QMutex>
#include <iostream>
#include <assert.h>

#include "RbStateMachine.h"
#include "RbRobotAgent.h"

class RbRobotMangThread : public QObject {
    Q_OBJECT
    Q_ENUMS(RB_ROBOT_MANAGEMENT_STATE)

public:
    RbRobotMangThread(int argc, char **pArgv);
    virtual ~RbRobotMangThread();

    enum RB_ROBOT_MANAGEMENT_STATE {
        INITIALIZED,
        ROBOT_AGENT_STARTED,
        SENSOR_AGENT_STARTED,

        RB_ROBOT_MANAGE_STATE_TOTAL
    };

    // ##############################################################################
    // STATE MACHINE METHODS & PROPERTIES -------------------------------------------
    //
    bool fTrue() { return true; }
    void initializeStateMachine();
    void runStateMachineOperation();

    RbSMRule<RbRobotMangThread>* getStateMachine() {
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

    // Robot Agent service -----------------
    void queryRobotOrientation();

    // Sensor Agent service ----------------
    void queryRobotSensorData(int sensorType, int sensorId);

signals:

private:
    int _init_argc;
    char** _pInit_argv;
    const char * _topic;

    int _robotId;

private:
    int _currentStateRuleId;
    RbSMRule<RbRobotMangThread>* _stateMachine; // pointing to static defined data

    // ##############################################################################
    // THREADING --------------------------------------------------------------------
    //
public:
    bool startThreading();

private:
    QTimer* _serviceTimer;
    QThread *_thread;
    QMutex* _mutex;
    QVector3D _robot_pos;
};
#endif

