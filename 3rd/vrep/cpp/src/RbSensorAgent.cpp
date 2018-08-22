#include <math.h>

#include "QMLAdapter.h"
#include "RbGlobal.h"
#include "RbSensorAgent.h"
#include "VREPAdapter.h"

void RbSensorAgent::initializeStateMachine()
{
    //static RbSMRule<RbSensorAgent> machine[RbSensorAgent::RB_SENSOR_STATE_TOTAL] =
    //{
    //    /* 0 */ { &RbRobotAgent::fTrue,          false, &RbRobotAgent::init,    0, 1 },
    //    /* 1 */ { &RbRobotAgent::checkIdle,      false, &RbRobotAgent::operate, 0, 2 },
    //    /* 2 */ { &RbRobotAgent::checkOperating, false, &RbRobotAgent::goIdle,  1, 1 }
    //};
    //_stateMachine = machine;
}

void RbSensorAgent::startStateMachineOperation()
{
    int currentStateId = RbStateMachine<RbSensorAgent>::run(this);
    if (currentStateId > 0) {
        this->setCurrentStateRuleId(currentStateId);
    }
}

// ==========================================================================================

RbSensorAgent::RbSensorAgent(const RbSensorProperties& prop) :
               QMLItemAgent(),
               _prop(prop)
{
    initializeStateMachine();
}

QVector<float> RbSensorAgent::getSensorData()
{
    QVector<float> sensorData;
    int inputInts[2]            = {this->type(), this->id()};
    float inputFloats[1]        = {0.0f};
    const char* inputStrings[1] = {""};
    simxUChar inputBuffer[1]    = {0};

    int *outputInts;
    int outputFloatsCnt = 0;
    float *outputFloats;
    simxChar* outputStrings = {""};
    simxUChar *outputBuffer;
    int res = simxCallScriptFunction(VREP_INSTANCE()->vrepClientId(), CB::CSERVER_REMOTE_API_OBJECT_NAME,
                                     sim_scripttype_childscript,
                                     "getSensorDataFromClient",
                                     1, inputInts,
                                     1, inputFloats,
                                     1, inputStrings[0],
                                     1, inputBuffer,
                                     nullptr, &outputInts,
                                     &outputFloatsCnt, &outputFloats,
                                     nullptr, &outputStrings,
                                     nullptr, &outputBuffer,
                                     simx_opmode_oneshot_wait
                                     );
    // if(res == simx_return_ok)
    for (int i = 0; i < outputFloatsCnt; i++) {
        //printf("%d - Data [%d]: %f\n", sensorId, i, outputFloats[i]);
        sensorData << outputFloats[i];
    }

    return sensorData;
}

