#ifndef ___ROBOT_LEAP_ADAPTER_H___
#define ___ROBOT_LEAP_ADAPTER_H___

#include <QtCore>
#include "KsGlobal.h"
#include "LeapMotion/camera_listener.h"
#include "LeapMotion/hands_listener.h"

class RobotLeapAdapter : public QObject {
    Q_OBJECT

public:
    RobotLeapAdapter() {}
    ~RobotLeapAdapter();

    void initLeapMotion();
private:
    // Create a sample listener and controller
    CameraListener _camera_listener;
    HandsListener  _hands_listener;
    Controller     _controller;
};

#endif // ___ROBOT_LEAP_ADAPTER_H___

