#ifndef ___ROBOT_GAZEBO_ADAPTER_H___
#define ___ROBOT_GAZEBO_ADAPTER_H___

#include <QtCore>
#include <QMutex>
#include "KsGlobal.h"

#include <cstdio>
#include <cstdarg>
//#include <rviz/robot/robot.h>

class RobotGazeboAdapter : public QObject {
    Q_OBJECT

public:
    static RobotGazeboAdapter* getInstance();
    ~RobotGazeboAdapter();
    static void deleteInstance();

    void initDart(int argc, char* argv[]);

private:
    RobotGazeboAdapter();
    static RobotGazeboAdapter* _instance;
    //
    QTimer timer;
    QMutex* _pMutex;
};

#endif // ___ROBOT_GAZEBO_ADAPTER_H___

