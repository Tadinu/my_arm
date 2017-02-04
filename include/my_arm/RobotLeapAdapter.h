#ifndef ___ROBOT_LEAP_ADAPTER_H___
#define ___ROBOT_LEAP_ADAPTER_H___

#include <QtCore>
#include <QMutex>
#include "KsGlobal.h"
#include "LeapMotion/camera_listener.h"
#include "LeapMotion/hands_listener.h"

class RobotLeapAdapter : public QObject {
    Q_OBJECT

public:
    static RobotLeapAdapter* getInstance();
    ~RobotLeapAdapter();
    static void deleteInstance();

    void initLeapMotion(ros::NodeHandle* nodeHandle);
    HandsListener* getHandsListener() { return _hands_listener; }
    std::vector<std::vector<double>> getFingerJointValues(int hand_id);
    void emitFingerPosesChanged();

signals:
    void fingerPosesChanged();

private:
    RobotLeapAdapter();
    static RobotLeapAdapter* _instance;
    // Create a sample listener and controller
    CameraListener* _camera_listener;
    HandsListener*  _hands_listener;
    Controller      _controller;

    QMutex* _pMutex;
};

#endif // ___ROBOT_LEAP_ADAPTER_H___

