#include <functional> // std::bind
#include <QThread>
#include "RobotLeapAdapter.h"
#include "KsGlobal.h"

RobotLeapAdapter* RobotLeapAdapter::_instance = nullptr;
RobotLeapAdapter* RobotLeapAdapter::getInstance()
{
    if(_instance == nullptr) {
        _instance = new RobotLeapAdapter();
    }

    return _instance;
}

RobotLeapAdapter::RobotLeapAdapter():
                  _camera_listener(nullptr),
                  _hands_listener(nullptr),
                  _pMutex(new QMutex(QMutex::Recursive))
{
    HandsListener::regEmitFingerPosUpdatedCallback(
        std::bind(&RobotLeapAdapter::emitFingerPosesChanged, this));
}

RobotLeapAdapter::~RobotLeapAdapter()
{
    _pMutex->tryLock(500);
    // Remove the sample listener when done
    _controller.removeListener(*_camera_listener);
    V_DELETE_POINTER_ARRAY(_camera_listener);

    // Remove the sample listener when done
    _controller.removeListener(*_hands_listener);
    V_DELETE_POINTER_ARRAY(_hands_listener);

    _pMutex->unlock(); // infutile if tryLock() failed!
    delete _pMutex;
}

void RobotLeapAdapter::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

void RobotLeapAdapter::initLeapMotion(ros::NodeHandle* node_handle)
{
    assert(node_handle);

    // Have the sample listener receive events from the controller
    //_camera_listener = new CameraListener();
    //_controller.addListener(*_camera_listener);

    _hands_listener = new HandsListener(node_handle);
    _controller.addListener(*_hands_listener);
    _controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));

    return;
}

std::vector<std::vector<double>> RobotLeapAdapter::getFingerJointValues(int hand_id)
{
    QMutexLocker lock(_pMutex);
    return _hands_listener->getFingerJointValues(hand_id);
}

void RobotLeapAdapter::emitFingerPosesChanged()
{
    ROS_INFO("RobotLeapAdapter::emitFingerPosesChanged()");
    emit fingerPosesChanged();
}

