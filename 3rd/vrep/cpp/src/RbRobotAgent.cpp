#include "RbRobotAgent.h"
#include <math.h>

#include "QMLAdapter.h"
#include "RbGlobal.h"

void RbRobotAgent::initializeStateMachine()
{
    static RbSMRule<RbRobotAgent> machine[RbRobotAgent::RB_ROBOT_STATE_TOTAL] =
    {
        /* 0 */ { &RbRobotAgent::fTrue,       false, &RbRobotAgent::initState, 0, 1 },
        /* 1 */ { &RbRobotAgent::isIdle,      false, &RbRobotAgent::operate,   0, 2 },
        /* 2 */ { &RbRobotAgent::isOperating, false, &RbRobotAgent::goIdle,    1, 1 }
    };
    _stateMachine = machine;
}

RbRobotAgent::RbRobotAgent() :
              QMLItemAgent(),
              _robotId(0)
{
    setRunningOnThread(true);
    initializeStateMachine();
}

void RbRobotAgent::startStateMachineOperation()
{
    int currentStateId = RbStateMachine<RbRobotAgent>::run(this);
    if (currentStateId > 0) {
        this->setCurrentStateRuleId(currentStateId);
    }
}

// ==========================================================================================
void RbRobotAgent::initState()
{
    QML_ITEM_LOCAL_INVOKE_I(setState, INITIALIZED);
}

bool RbRobotAgent::isIdle()
{
    QVariant var;
    RB_THREAD_BLOCKING_INVOKE_RET(_itemUI, getState, var)
    return var.toInt() == IDLE;
}

bool RbRobotAgent::isOperating()
{
    QVariant var;
    RB_THREAD_BLOCKING_INVOKE_RET(_itemUI, getState, var)
    return var.toInt() == OPERATING;
}

void RbRobotAgent::operate() 
{
    QML_ITEM_LOCAL_INVOKE_I(setState, OPERATING);
}

void RbRobotAgent::goIdle()
{
    QML_ITEM_LOCAL_INVOKE_I(setState, IDLE);
}

bool RbRobotAgent::isFaulted()
{
    return false;
}
