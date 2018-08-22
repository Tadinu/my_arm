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
        INIT,
        IDLE,
        OPERATING,

        RB_ROBOT_STATE_TOTAL
    };

    RbRobotAgent();
    ~RbRobotAgent() {}

    void init() {}
    void initializeStateMachine();
    void startStateMachineOperation();

    RbSMRule<RbRobotAgent>* getStateMachine() {
        return _stateMachine;
    }

    bool checkOperating();
    bool checkIdle();
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
