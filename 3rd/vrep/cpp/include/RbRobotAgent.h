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
        ERROR = -1,
        INITIALIZED,
        IDLE,
        OPERATING,
        COLLIDING,

        RB_ROBOT_STATE_TOTAL
    };

    RbRobotAgent();
    ~RbRobotAgent() {}

    // State machine methods ------------------------
    void initializeStateMachine();
    void runStateMachineOperation();

    RbSMRule<RbRobotAgent>* getStateMachine() {
        return _stateMachine;
    }

    // Functionality Methods ------------------------
    int getUIState();
    void setUIState(int state);
    bool isOperating();
    bool isIdle();
    bool isHalted();
    void initState();
    void operate();
    void goIdle();
    void updateOrientationUI(const QVector3D& orient);

signals:
    void queryOrientation();

public slots:
    void onStateChanged() {
        operate();
    }

private:
    int _robotId;

    RbSMRule<RbRobotAgent>* _stateMachine; // pointing to static defined data
};
#endif
