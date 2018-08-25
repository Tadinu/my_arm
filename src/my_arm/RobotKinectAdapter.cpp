#include <functional> // std::bind
#include <QThread>
#include "RobotKinectAdapter.h"

RobotKinectAdapter* RobotKinectAdapter::_instance = nullptr;
RobotKinectAdapter* RobotKinectAdapter::getInstance()
{
    if(_instance == nullptr) {
        _instance = new RobotKinectAdapter();
    }

    return _instance;
}

RobotKinectAdapter::RobotKinectAdapter():
                  _pMutex(new QMutex(QMutex::Recursive))
{
}

RobotKinectAdapter::~RobotKinectAdapter()
{
    _pMutex->tryLock(500);


    _pMutex->unlock(); // futile if tryLock() failed!
    delete _pMutex;
}

void RobotKinectAdapter::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

void RobotKinectAdapter::initRealSense(ros::NodeHandle* node_handle)
{
    assert(node_handle);

    return;
}

std::vector<std::vector<double>> RobotKinectAdapter::getFingerJointValues(int hand_id)
{
    QMutexLocker lock(_pMutex);
    return std::vector<std::vector<double>>();
}

void RobotKinectAdapter::emitFingerPosesChanged()
{
    ROS_INFO("RobotKinectAdapter::emitFingerPosesChanged()");
    emit fingerPosesChanged();
}
