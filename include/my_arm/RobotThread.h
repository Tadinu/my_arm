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

    void rotateJoint(int jointId, double angleDelta);
    void setMarkerPos(const QVector3D& markerPos);

    // Send methods --
    void sendRobot(int robotId, geometry_msgs::TransformStamped& world_trans,
                   tf::TransformBroadcaster& broadcaster,
                   ros::Publisher& joint_pub, sensor_msgs::JointState& jointState);
    void sendMarkers(const ros::Publisher& marker_pub, uint32_t& shape,
                     const QVector3D& pos = QVector3D(), const QVector3D& orient = QVector3D(),
                     double rotAngle = 1.0f,
                     const QVector3D& scale = QVector3D(1.0f, 1.0f, 1.0f));

public slots:
    void run();

signals:
    void newPose(double, double, double);
    void jointPosUpdated(int jointId, const QVector3D& pos, double angle);

private:
    int _init_argc;
    char** _pInit_argv;
    const char * _topic;

    double _speed;
    double _angle;
    double _angleDelta;

    double _xPos;
    double _yPos;
    double _aPos;

    double _maxRange;
    double _minRange;

    QThread * _pThread;
    QMutex* _pMutex;

    ros::Subscriber _pose_listener;
    ros::Publisher  _sim_velocity;

    QVector3D _marker_pos;
};
#endif

