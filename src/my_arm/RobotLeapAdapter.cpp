#include <functional> // std::bind
#include <QThread>
#include <QVector3D>
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

bool RobotLeapAdapter::checkInstance()
{
    return _instance != nullptr;
}

RobotLeapAdapter::RobotLeapAdapter():
                  _camera_listener(nullptr),
                  _hands_listener(nullptr),
                  _pMutex(new QMutex(QMutex::Recursive))
{
    HandsListener::regEmitFingerPosUpdatedCallback(
        std::bind(&RobotLeapAdapter::emitFingerPosesChanged, this));

    HandsListener::regEmitFingerGestureCallback(HandsListener::FINGER_UP,
        std::bind(&RobotLeapAdapter::emitFingerUp, this, std::placeholders::_1));

    HandsListener::regEmitFingerGestureCallback(HandsListener::FINGER_DOWN,
        std::bind(&RobotLeapAdapter::emitFingerDown, this, std::placeholders::_1));

    HandsListener::regEmitFingerGestureCallback(HandsListener::FINGER_DOWN_MOVE,
        std::bind(&RobotLeapAdapter::emitFingerDownMove, this, std::placeholders::_1));
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

    _pMutex->unlock(); // futile if tryLock() failed!
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

std::vector<QVector3D> RobotLeapAdapter::getFingerTipsPoses(int hand_id)
{
    QMutexLocker lock(_pMutex);
    std::vector<QVector3D> fingerTipsPoses;
    std::vector<Vector> tips = _hands_listener->getFingerTipsPoses(hand_id);
    for(size_t i = 0; i < tips.size(); i++) {
        fingerTipsPoses.push_back(QVector3D(tips[i].x, tips[i].y, tips[i].z));
    }

    return fingerTipsPoses;
}

void RobotLeapAdapter::emitFingerPosesChanged()
{
    ROS_INFO("RobotLeapAdapter::emitFingerPosesChanged()");
    emit fingerPosesChanged();
}

void RobotLeapAdapter::emitFingerUp(const std::vector<float>& point)
{
    //ROS_INFO("RobotLeapAdapter::emitFingerUp(): %f, %f, %f", point[0], point[1], point[2]);
    emit fingerUp(QVector3D(point[0], point[1], point[2]));
}

void RobotLeapAdapter::emitFingerDown(const std::vector<float>& point)
{
    //ROS_INFO("RobotLeapAdapter::emitFingerDown(): %f, %f, %f", point[0], point[1], point[2]);
    emit fingerDown(QVector3D(point[0], point[1], point[2]));
}

void RobotLeapAdapter::emitFingerDownMove(const std::vector<float>& point)
{
    //ROS_INFO("RobotLeapAdapter::emitFingerDownMove(): %f, %f, %f", point[0], point[1], point[2]);
    emit fingerDownMove(QVector3D(point[0], point[1], point[2]));
}
