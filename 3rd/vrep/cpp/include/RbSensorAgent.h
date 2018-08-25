#ifndef RB_SENSOR_AGENT
#define RB_SENSOR_AGENT

//#include "obstacle.h"

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
        INITIALIZED,
        DATA_FETCHING,
        UI_UPDATING,

        RB_SENSOR_STATE_TOTAL
    };

    RbSensorAgent(const RbSensorProperties& prop);
    ~RbSensorAgent();

    // General Properties --------------------------
    int type() const         { return _prop._type; }
    int id()   const         { return _prop._id;   }
    const char* name() const { return _prop._name; }

    // State machine methods -----------------------
    void initializeStateMachine();
    void startStateMachineOperation();
    RbSMRule<RbSensorAgent>* getStateMachine() {
        return _stateMachine;
    }

    // Functionality methods -----------------------
    void initState();
    void fetchSensorData();
    void updateUI();
    QVector<float> getSensorData();
    bool isFaulted();

public slots:

private:
    const RbSensorProperties& _prop; // Referencing static data
    RbSMRule<RbSensorAgent>* _stateMachine; // pointing to static defined data
};
#endif
