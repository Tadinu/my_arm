#ifndef ___ROBOTTHREAD_H___
#define ___ROBOTTHREAD_H___

#include <QtCore>
#include <QThread>
#include <QVector3D>
#include <QQuaternion>
#include <QStringList>
#include <stdlib.h>
#include <QMutex>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class RobotThread : public QObject {
	Q_OBJECT
public:
    RobotThread(int argc, char **pArgv, const char * topic  = "/odom");
    virtual ~RobotThread();

    double getXPos();
    double getXSpeed();
    double getASpeed();
    double getYPos();
    double getAPos();

    bool init();

    void poseCallback(const nav_msgs::Odometry & msg);
    void runArmOperation(int armId);
    std::string getRobotNodeName(int armId);
    void SetSpeed(double speed, double angle);
    void setPose(QList<double> to_set);

    // Action command --
    //
    void moveRobot(geometry_msgs::TransformStamped& world_trans,
                   tf::TransformBroadcaster& broadcaster,
                   const tf::Quaternion& quaternion = tf::Quaternion());
    void rotateJoint(int jointId, double posDelta, bool updateBallPos = true);
    void moveBall(const tf::Vector3& distance);
    void setBallPos(const tf::Vector3& pos, bool armFollowOrder = false);
    tf::Vector3 determineEndTipPos();
    void determineRobotOperationLimit();
    void determineArmArrangement(const tf::Vector3& pos);
    void updateBallFollowingEndTip();

    // Publish/Send methods --
    //
    void publishJointState(int robotId,
                           const ros::Publisher& joint_pub,
                           sensor_msgs::JointState& jointState);
    void publishMarkers(const ros::Publisher& marker_pub, uint32_t& shape,
                        const tf::Vector3& scale = tf::Vector3(1.0f, 1.0f, 1.0f),
                        const tf::Vector3& pos = tf::Vector3(),
                        const tf::Quaternion& orient = tf::Quaternion());

    // Transforms --
    void detectFrameTransforms(int robotId, const tf::TransformListener& listener);

public slots:
    void run();

signals:
    void newPose(double, double, double);
    void jointPosUpdated(int jointId, const QVector3D& pos, double posDelta);

private:
    int _init_argc;
    char** _pInit_argv;
    const char * _topic;

    double _speed;
    double *_joint_poses;
    size_t _jointNo;
    size_t _linkNo;
    tf::StampedTransform* _frame_trans;
    tf::Vector3 _arm_reach_limit;

    double _xPos;
    double _yPos;
    double _aPos;

    double _maxRange;
    double _minRange;

    QThread * _pThread;
    QMutex* _pMutex;

    ros::Subscriber _pose_listener;
    ros::Publisher  _sim_velocity;

    tf::Vector3 _marker_pos;
};
#endif

