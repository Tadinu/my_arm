#ifndef RB_SENSOR_AGENT
#define RB_SENSOR_AGENT

//#include "obstacle.h"

#include <QList>
#include <QColor>

#include "QMLItemAgent.h"
#include "RbGlobal.h"
#include "RbStateMachine.h"

class RbRobotSensorAdapter; // As friend class

class RbSensorAgent : public QMLItemAgent
{
    Q_OBJECT
    Q_ENUMS(RB_SENSOR_STATE)

    friend class RbRobotSensorAdapter;

private:
    struct RbSensorProperties {
        int _type;
        int _id;
        const char* _name;
    };

public:
    enum RB_SENSOR_STATE {
        VOID_DATA,
        DATA,

        RB_SENSOR_STATE_TOTAL
    };

    RbSensorAgent(const RbSensorProperties& prop);
    ~RbSensorAgent() {
    }

    // General Properties --
    int type() const         { return _prop._type; }
    int id()   const         { return _prop._id;   }
    const char* name() const { return _prop._name; }

    QVector<float> getSensorData();

    // State machine --
    void init() {}
    void initializeStateMachine();

    RbSMRule<RbSensorAgent>* getStateMachine() {
        return _stateMachine;
    }

    void startStateMachineOperation();

public slots:

private:
    const RbSensorProperties& _prop; // Referencing static data
    RbSMRule<RbSensorAgent>* _stateMachine; // pointing to static defined data
};
#endif
