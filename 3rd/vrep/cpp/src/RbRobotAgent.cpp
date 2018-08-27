#include "RbRobotAgent.h"
#include <math.h>

#include "QMLAdapter.h"
#include "RbGlobal.h"
#include "VREPAdapter.h"

void RbRobotAgent::initializeStateMachine()
{
    static RbSMRule<RbRobotAgent> machine[] =
    {
        /* 0 */ { &RbRobotAgent::fTrue, false, &RbRobotAgent::initState, 0, 1 },
        /* 1 */ { &RbRobotAgent::fTrue, false, &RbRobotAgent::operate,   0, 0 },
    };
    _stateMachine = machine;
}

RbRobotAgent::RbRobotAgent() :
              QMLItemAgent(),
              _robotId(0)
{
    setRunningOnThread(true);
    initializeStateMachine();
    QMLItemAgent::setupAgentAndUI("_robotCar");
}

void RbRobotAgent::runStateMachineOperation()
{
    int currentStateId = RbStateMachine<RbRobotAgent>::run(this);
    //printf("RbRobotAgent::runStateMachineOperation - %d - %d\n", currentStateId, this->getCurrentStateRuleId());
    if (currentStateId > 0) {
        this->setCurrentStateRuleId(currentStateId);
    }
}

// ==========================================================================================
void RbRobotAgent::initState()
{
    setUIState(INITIALIZED);
}

bool RbRobotAgent::isIdle()
{
    return getUIState() == IDLE;
}

bool RbRobotAgent::isOperating()
{
    return getUIState() == OPERATING;
}

void RbRobotAgent::operate()
{
    // 1 - OPERATE THE ROBOT
    // ...

    // 2 - UPDATE THE ROBOT CONDITION TO UI:
    // State --
    setUIState(OPERATING);

    //printf("Query Orientation\n");
    emit queryOrientation();
}

void RbRobotAgent::goIdle()
{
    setUIState(IDLE);
}

bool RbRobotAgent::isHalted()
{
    static bool halted = true;
    // First time
    if(halted) {
        halted = false;
        return true;
    }
    // Next times
    else {
        return halted;
    }
}

void RbRobotAgent::updateOrientationUI(const QVector3D& orient)
{
    _mutex->lock();
    RB_THREAD_INVOKE_I(_itemUI, setOrientation, orient);
    //QML_ITEM_LOCAL_INVOKE_I(setOrientation, orient);
    _mutex->unlock();
}

int RbRobotAgent::getUIState()
{
    QVariant var;
    RB_THREAD_BLOCKING_INVOKE_RET(_itemUI, getState, var)
    return var.toInt();
}

void RbRobotAgent::setUIState(int state)
{
    _mutex->lock();
    RB_THREAD_INVOKE_I(_itemUI, setState, state);
    _mutex->unlock();
}


