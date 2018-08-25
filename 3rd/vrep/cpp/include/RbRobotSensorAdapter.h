#ifndef _ROBOT_SENSOR_ADAPTER_H_
#define _ROBOT_SENSOR_ADAPTER_H_

#include <QtCore>
#include <QMutex>
#include "RbGlobal.h"
#include "RbSensorAgent.h"

#ifdef ROBOT_SENSORS
#define RB_SENSOR_SYSTEM() RbRobotSensorAdapter::getInstance()
#endif

class RbRobotSensorAdapter : public QObject {
    Q_OBJECT

public:
    enum RB_SENSOR {
        RB_SENSOR_FIRST,
        RB_SENSOR_FRONT_VISION  = RB_SENSOR_FIRST,
        RB_SENSOR_GROUND_VISION,
        RB_SENSOR_FLOOR,
        RB_SENSOR_FORCE,
        RB_SENSOR_ULTRASONIC_1,
        RB_SENSOR_ULTRASONIC_2,
        RB_SENSOR_ULTRASONIC_3,
        RB_SENSOR_ULTRASONIC_4,
        RB_SENSOR_ULTRASONIC_5,
        RB_SENSOR_ULTRASONIC_6,
        RB_SENSOR_ULTRASONIC_7,
        RB_SENSOR_ULTRASONIC_8,
        RB_SENSOR_ULTRASONIC_9,
        RB_SENSOR_ULTRASONIC_10,
        RB_SENSOR_ULTRASONIC_11,
        RB_SENSOR_ULTRASONIC_12,
        RB_SENSOR_ULTRASONIC_13,
        RB_SENSOR_ULTRASONIC_14,
        RB_SENSOR_ULTRASONIC_15,
        RB_SENSOR_ULTRASONIC_16,

        RB_SENSOR_TOTAL
    };

public:
    static RbRobotSensorAdapter* getInstance();
    ~RbRobotSensorAdapter();
    static void deleteInstance();
    static bool checkInstance();

    void initialize();
    void initializeSensorAgents();
    void runSensorOperation();

    const QVector<RbSensorAgent*>& sensorAgentList() { return _sensorAgentList; }
    RbSensorAgent* sensorAgent(int sensorId);
    bool isHalted();
signals:


private:
    RbRobotSensorAdapter();
    static RbRobotSensorAdapter* _instance;

    QMutex* _mutex;
    QVector<RbSensorAgent*> _sensorAgentList;
};

#endif // _ROBOT_SENSOR_ADAPTER_H_

