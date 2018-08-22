#include "RbRobotAgent.h"
#include <math.h>

#include "QMLAdapter.h"
#include "RbGlobal.h"

void RbRobotAgent::initializeStateMachine()
{
    static RbSMRule<RbRobotAgent> machine[RbRobotAgent::RB_ROBOT_STATE_TOTAL] =
    {
        /* 0 */ { &RbRobotAgent::fTrue,          false, &RbRobotAgent::init,    0, 1 },
        /* 1 */ { &RbRobotAgent::checkIdle,      false, &RbRobotAgent::operate, 0, 2 },
        /* 2 */ { &RbRobotAgent::checkOperating, false, &RbRobotAgent::goIdle,  1, 1 }
    };
    _stateMachine = machine;
}

RbRobotAgent::RbRobotAgent() :
              QMLItemAgent(),
              _robotId(0)
{
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
bool RbRobotAgent::checkIdle()
{
    QVariant var;
    RB_THREAD_BLOCKING_INVOKE_RET(_itemUI, getState, var)
    return var.toInt() == 1;
}

bool RbRobotAgent::checkOperating()
{
    QVariant var;
    RB_THREAD_BLOCKING_INVOKE_RET(_itemUI, getState, var)
    return var.toInt() == 2;
}

void RbRobotAgent::operate() 
{
    QML_ITEM_LOCAL_INVOKE_I(setState, 2);
}

void RbRobotAgent::goIdle()
{
    QML_ITEM_LOCAL_INVOKE_I(setState, 1);
}
