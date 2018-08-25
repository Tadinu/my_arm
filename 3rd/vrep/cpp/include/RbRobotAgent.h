#ifndef RB_ROBOT_AGENT
#define RB_ROBOT_AGENT

//#include "obstacle.h"

#include <QList>
#include <QColor>

#include "RbStateMachine.h"
#include "QMLItemAgent.h"
#include "RbGlobal.h"

class RbRobotAgent : public QMLItemAgent
{
    Q_OBJECT
    Q_ENUMS(RB_ROBOT_STATE)
public:

    enum RB_ROBOT_STATE {
        INITIALIZED,
        IDLE,
        OPERATING,

        RB_ROBOT_STATE_TOTAL
    };

    RbRobotAgent();
    ~RbRobotAgent() {}

    // State machine methods ------------------------
    void initializeStateMachine();
    void startStateMachineOperation();

    RbSMRule<RbRobotAgent>* getStateMachine() {
        return _stateMachine;
    }

    // Functionality Methods ------------------------
    bool isOperating();
    bool isIdle();
    bool isFaulted();
    void initState();
    void operate();
    void goIdle();

public slots:
    void onStateChanged() {
        operate();
    }

private:
    int _robotId;

    RbSMRule<RbRobotAgent>* _stateMachine; // pointing to static defined data
};
#endif
