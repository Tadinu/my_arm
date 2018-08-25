#include <math.h>

#include "QMLAdapter.h"
#include "RbGlobal.h"
#include "RbSensorAgent.h"
#include "VREPAdapter.h"

void RbSensorAgent::initializeStateMachine()
{
    static RbSMRule<RbSensorAgent> machine[RbSensorAgent::RB_SENSOR_STATE_TOTAL] =
    {
        /* 0 */ { &RbSensorAgent::fTrue, false, nullptr,         0, 1 },
        /* 1 */ { &RbSensorAgent::fTrue, false, &RbSensorAgent::fetchSensorData, 0, 0 }
    };
    _stateMachine = machine;
}

void RbSensorAgent::runStateMachineOperation()
{
    //printf("RbSensorAgent::runStateMachineOperation\n");
    int currentStateId = RbStateMachine<RbSensorAgent>::run(this);
    //printf("RbSensorAgent::runStateMachineOperation - %d\n", currentStateId);
    if (currentStateId > 0) {
        this->setCurrentStateRuleId(currentStateId);
    }
}

// ==========================================================================================

RbSensorAgent::RbSensorAgent(const RbSensorProperties& prop) :
               QMLItemAgent(),
               _prop(prop)
{
    setRunningOnThread(true);
    initializeStateMachine();
}

RbSensorAgent::~RbSensorAgent()
{}

void RbSensorAgent::initState()
{
}

void RbSensorAgent::fetchSensorData()
{
    //printf("Query SENSOR DATA %d\n", this->id());
    emit querySensorData(this->type(), this->id());
}

bool RbSensorAgent::isHalted()
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
