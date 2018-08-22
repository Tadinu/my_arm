#ifndef _RB_ROBOT_THREAD_H_
#define _RB_ROBOT_THREAD_H_

#include <QtCore>
#include <QThread>
#include <QVector3D>
#include <QQuaternion>
#include <QStringList>
#include <stdlib.h>
#include <QMutex>
#include <mutex>
#include <iostream>
#include "assert.h"

#include "RbStateMachine.h"

class RbRobotThread : public QObject {
    Q_OBJECT
    Q_ENUMS(RB_ROBOT_MANAGEMENT_STATE)

public:
    RbRobotThread(int argc, char **pArgv);
    virtual ~RbRobotThread();

    enum RB_ROBOT_MANAGEMENT_STATE {
        INIT,
        IDLE,
        OPERATING,

        RB_ROBOT_MANAGE_STATE_TOTAL
    };

    // ##############################################################################
    // STATE MACHINE MEMBER METHODS -------------------------------------------------
    //
    bool fTrue() { return true; }
    bool init();
    void initializeStateMachine();
    void startStateMachineOperation();

    RbSMRule<RbRobotThread>* getStateMachine() {
        return _stateMachine;
    }

    int getCurrentStateRuleId()                 { return _currentStateRuleId;        }
    void setCurrentStateRuleId(int stateRuleId) { _currentStateRuleId = stateRuleId; }

public slots:
    void run();
    void onCommonServiceTimeout();

signals:

private:
    int _init_argc;
    char** _pInit_argv;
    const char * _topic;

    int _robotId;

    QTimer* _serviceTimer;
    QThread * _pThread;
    QMutex* _pMutex; // std::mutex
    QVector3D _robot_pos;

private:
    // STATE MACHINE PROPERTIES -----------------------------------------------------
    //
    int _currentStateRuleId;
    RbSMRule<RbRobotThread>* _stateMachine; // pointing to static defined data
};
#endif

