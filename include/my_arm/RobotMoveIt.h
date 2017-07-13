#ifndef ___ROBOT_MOVEIT_H___
#define ___ROBOT_MOVEIT_H___

#include <QtCore>
#include <QMutex>
#include "KsGlobal.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// Kinematics
#include <moveit_msgs/GetPositionIK.h>

#define VMOVEIT() RobotMoveIt::getInstance()

class RobotMoveIt : public QObject {
    Q_OBJECT

public:
    static RobotMoveIt* getInstance();
    ~RobotMoveIt();
    static void deleteInstance();
    static bool checkInstance();

    void initMoveIt(ros::NodeHandle* nodeHandle);

    // Read Robot Model Info
    void fetchRobotModelInfo();

    // End-Effector
    void openRobotHand(trajectory_msgs::JointTrajectory &posture);
    void closeRobotHand(trajectory_msgs::JointTrajectory &posture);

    // Pick/Place
    void setupPickPlaceSettings();
    void placeObject(moveit::planning_interface::MoveGroupInterface &group);
    void pickObject(moveit::planning_interface::MoveGroupInterface &group);

    // Motion Planing
    // https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/pr2_tutorials/planning/src
#if 0
    void doMotionPlanning();
    void doMoveGroupInterface();
    void doMoveGroup();
    void doPlanningSceneRosAPI();
    void doPlanningPipeline();
    void doPlanningScene();
#endif
signals:

private:
    RobotMoveIt();
    static RobotMoveIt* _instance;
    QMutex* _pMutex;
    boost::shared_ptr<ros::NodeHandle> _node_handle;
    ros::Publisher _robot_state_publisher;
    robot_model::RobotModelPtr _kinematic_model;
    robot_state::RobotState* _kinematic_state;
    robot_state::JointModelGroup *_joint_model_group;
};

#endif // ___ROBOT_MOVEIT_H___

