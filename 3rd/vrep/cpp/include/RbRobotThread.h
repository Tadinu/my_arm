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

class RbRobotThread : public QObject {
    Q_OBJECT
    Q_ENUMS(RB_ROBOT_MANAGEMENT_STATE)

public:
    RbRobotThread(int argc, char **pArgv);
    virtual ~RbRobotThread();

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
    void startStateMachineOperation();

    RbSMRule<RbRobotThread>* getStateMachine() {
        return _stateMachine;
    }

    int getCurrentStateRuleId()                 { return _currentStateRuleId;        }
    void setCurrentStateRuleId(int stateRuleId) { _currentStateRuleId = stateRuleId; }

    // Functionality methods --
    void startRobotAgent();
    bool isRobotAgentFaulted();
    void startSensorAgents();
    bool isSensorAgentsFaulted();

public slots:
    void run();
    void onCommonServiceTimeout();

signals:

private:
    int _init_argc;
    char** _pInit_argv;
    const char * _topic;

    int _robotId;
    RbRobotAgent* _robotAgent;

private:
    int _currentStateRuleId;
    RbSMRule<RbRobotThread>* _stateMachine; // pointing to static defined data

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

