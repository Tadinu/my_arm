#ifndef _RB_ROBOT_THREAD_H_
#define _RB_ROBOT_THREAD_H_

#include <QtCore>
#include <QThread>
#include <QVector3D>
#include <QQuaternion>
#include <QStringList>
#include <QMutex>
#include <QImage>
#include <iostream>
#include <assert.h>

#include "RbStateMachine.h"
#include "RbRobotAgent.h"

class RbRobotManager : public QObject {
    Q_OBJECT
    Q_ENUMS(RB_ROBOT_MANAGEMENT_STATE)

public:
    RbRobotManager(int argc, char **pArgv);
    virtual ~RbRobotManager();

    enum RB_ROBOT_MANAGEMENT_STATE {
        INITIALIZED,
        ROBOT_CHECK,
        SENSOR_CHECK,
        SENSOR_SIGNAL_HANDLE,

        RB_ROBOT_MANAGE_STATE_TOTAL
    };

    // ##############################################################################
    // STATE MACHINE METHODS & PROPERTIES -------------------------------------------
    //
    bool fTrue() { return true; }
    void initializeStateMachine();
    void runStateMachineOperation();

    RbSMRule<RbRobotManager>* getStateMachine() {
        return _stateMachine;
    }

    int getCurrentStateRuleId()                 { return _currentStateRuleId;        }
    void setCurrentStateRuleId(int stateRuleId) { _currentStateRuleId = stateRuleId; }

    // Functionality methods --
    void initState();
    bool isRobotComFaulted();
    bool isSensorFaulted();
    void troubleshootRobotCom();
    void troubleshootSensors();

    QTimer* getServiceTimer() { return _serviceTimer; }

public slots:
    void runTask();
    void setServiceTimeout(int timeout);
    void setFallingObjTimeInterval(int timeInterval);

    // Robot Agent service -----------------
    void queryRobotOrientation();
    void setRobotVel(float vel);

    // Sensor Agent service ----------------
    void queryRobotSensorData(int sensorType, int sensorId);
    QImage queryRobotCameraData(const char* visionSensorName);

signals:

private:
    int _init_argc;
    char** _pInit_argv;
    const char * _topic;

    int _robotId;

private:
    int _currentStateRuleId;
    RbSMRule<RbRobotManager>* _stateMachine; // pointing to static defined data
    unsigned char _vrepStrSignalData[1][100];
    int _vrepIntSignalData;

    // ##############################################################################
    // THREADING --------------------------------------------------------------------
    //
public:
    bool startThreading();

private:
    QTimer* _serviceTimer;
    QThread *_thread;
    QMutex* _mutex;
    QVector3D _robot_pos;
};
#endif

