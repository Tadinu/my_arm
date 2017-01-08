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
#include <interactive_markers/interactive_marker_server.h>
#include "rviz/VMarker.h"

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
    void setRobotPos(const tf::Vector3 &pos);
    tf::Vector3 getRobotPos();
    void rotateJoint(int jointId, double posDelta, bool updateBallPos = true);
    void moveBall(const tf::Vector3& distance);
    void setBallPos(const tf::Vector3& pos, bool armFollowOrder = false);

    void determineArrangement_MyArm(tf::Vector3& world_target_pos);
    void determineArrangement_JacoArm(tf::Vector3& world_target_pos);

    void updateBallFollowingEndTip();
public slots:
    void run();
    void determineArmArrangement(const QVector3D& world_target_pos);
signals:
    void newPose(double, double, double);
    void jointPosUpdated(int jointId, const QVector3D& pos, double posDelta);

private:
    void publishRobotPose(int robotId,
                          geometry_msgs::TransformStamped& world_trans,
                          tf::TransformBroadcaster& broadcaster);
    // Publish/Send methods --
    //
    void publishJointState(int robotId,
                           const ros::Publisher& joint_pub,
                           sensor_msgs::JointState& jointState);
    void publishStaticMarkers(const ros::Publisher& marker_pub, visualization_msgs::Marker& marker);

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    tf::Vector3 determineEndTipLocalPos();
    void determineRobotOperationLimit();

    // Transforms --
    void detectFrameTransforms(int robotId, const tf::TransformListener& listener);
    tf::Vector3 convertWorld2RobotFramePos(const tf::Vector3& world_target_pos);
    tf::Vector3 convertRobotFrame2WorldPos(const tf::Vector3& robot_local_pos);

    double distanceToJoin(int jointId, const tf::Vector3& point);

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
    tf::Vector3 _robot_pos;
    // --------------------------------------------------------------------
    //
    double _xPos;
    double _yPos;
    double _aPos;

    double _maxRange;
    double _minRange;

    QThread * _pThread;
    QMutex* _pMutex;

    ros::Subscriber _pose_listener;
    ros::Publisher  _sim_velocity;
};
#endif

