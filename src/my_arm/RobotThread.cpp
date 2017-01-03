#include <string>
#include <QMutexLocker>
#include <QtMath>
#include <tf/exceptions.h>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
#include "RobotThread.h"
#include "KsGlobal.h"

// ROBOT TO RUN
#define CRUN_ROBOT (KsGlobal::VMY_ARM)

// NODE --
#define CMY_ARM_NODE_NAME   ("robotArmController")
#define CJACO_ARM_NODE_NAME ("jacoArmController")

// FRAMES --
#define CWORLD_FRAME ("world")
#define CBASE_LINK ("base_link")

#define ROS_TOPIC(linkName) (std::string("//") + linkName)

const double degree = M_PI/180;
const double TWO_PI = 2 * M_PI;

const double l0 = 0;
const double l1 = 0.7;
const double l2 = 0.7;
const double l3 = 0.15;
const double l4 = 0.025;

const tf::Vector3 CLOCAL_END_TIP(0, 0, l4);

const char* CMY_ARM_JOINTS[KsGlobal::VMY_ARM_JOINT_TOTAL] = {
    ("base_joint"),
    ("j10"),
    ("j1"),
    ("j20"),
    ("j2"),
    ("j3"),

    ("finger_1_prox_joint"),
    ("finger_1_med_joint"),
    ("finger_1_dist_joint"),

    ("finger_2_prox_joint"),
    ("finger_2_med_joint"),
    ("finger_2_dist_joint"),

    ("finger_3_med_joint"),
    ("finger_3_dist_joint")
};

const char* CMY_ARM_LINKS[KsGlobal::VMY_ARM_JOINT_TOTAL+1] = {
    CBASE_LINK,
    ("body1"),
    ("body10"),
    ("body2"),
    ("body20"),
    ("body3"),

    ("brHand"),
    ("finger_1_prox_link"),
    ("finger_1_med_liink"),
    ("finger_1_dist_link"),

    ("finger_2_prox_link"),
    ("finger_2_med_link"),
    ("finger_2_dist_link"),

    ("finger_3_med_link"),
    ("finger_3_dist_link")
};

const char* CJACO_ARM_JOINTS[KsGlobal::VJACO_ARM_JOINT_TOTAL] = {
    "jaco_arm_joint", // Fixed
    "jaco_base_internal", // Fixed

    "jaco_ring_1_joint", // Fixed
    "jaco_arm_0_joint",  // Continuous
    "jaco_ring_2_joint", // Fixed
    "jaco_arm_1_joint",  // Revolute
    "jaco_ring_3_joint", // Fixed
    "jaco_arm_2_joint",  // Revolute
    "jaco_ring_4_joint", // Fixed
    "jaco_arm_3_joint",  // Continuous
    "jaco_ring_5_joint", // Fixed
    "jaco_arm_4_joint",  // Continuous
    "jaco_ring_6_joint", // Fixed
    "jaco_arm_5_joint",  // Continuous

    "jaco_fingers_base_joint", // Fixed
    "jaco_finger_mount_index_fixed", // Fixed
    "jaco_finger_joint_0", // Revolute
    "jaco_finger_joint_1", // Fixed
    "jaco_finger_mount_thumb_fixed", // Fixed
    "jaco_finger_joint_2", // Revolute
    "jaco_finger_joint_3", // Fixed
    "jaco_finger_mount_pinkie_fixed" // Fixed
    "jaco_finger_joint_4", // Revolute
    "jaco_finger_joint_5", // Fixed
};

RobotThread::RobotThread(int argc, char** pArgv, const char * topic)
            : _init_argc(argc),
              _pInit_argv(pArgv),
              _topic(topic),
              _jointNo(0),
              _joint_poses(nullptr),
              _frame_trans(nullptr)
{/** Constructor for the robot thread **/
}

RobotThread::~RobotThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }//end if

    //m_pMutex->tryLock(5000);
    //m_pMutex->unlock();

    V_DELETE_POINTER_ARRAY(_joint_poses);
    V_DELETE_POINTER_ARRAY(_frame_trans);

    if(!_pThread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
    {
        qWarning("Thread deadlock detected, bad things may have happened !!!");
        _pThread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        _pThread->wait(); //Note: We have to wait again here!
    }
} //end destructor

bool RobotThread::init()
{
    _pThread = new QThread();
    this->moveToThread(_pThread);

    //m_pMutex = new QMutex();

    connect(_pThread, &QThread::started, this, &RobotThread::run);
    ros::init(_init_argc, _pInit_argv, "gui_command");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    _sim_velocity  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    _pose_listener = nh.subscribe(_topic, 10, &RobotThread::poseCallback, this);

    _pThread->start();
    return true;
}//set up the thread

void RobotThread::poseCallback(const nav_msgs::Odometry & msg)
{
    QMutex * pMutex = new QMutex();

    pMutex->lock();
    _xPos = msg.pose.pose.position.x;
    _yPos = msg.pose.pose.position.y;
    _aPos = msg.pose.pose.orientation.w;
    pMutex->unlock();

    delete pMutex;
    Q_EMIT newPose(_xPos, _yPos, _aPos);
}//callback method to update the robot's position.

void RobotThread::runArmOperation(int armId)
{
#if 1 // ducta ++
    ros::init(_init_argc, _pInit_argv, getRobotNodeName(armId)); // Name of the node specified in launch file
    ros::NodeHandle node_handler;

    // ROBOT --
    // message declarations
    geometry_msgs::TransformStamped world_trans;
    sensor_msgs::JointState joint_state;
    world_trans.header.frame_id = CWORLD_FRAME;
    world_trans.child_frame_id  = CBASE_LINK;

    ros::Publisher joint_pub = node_handler.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster trans_broadcaster;
    tf::TransformListener trans_listener;
    // ------------------------------------------------------------------------------
    // MARKERS --
    ros::Publisher marker_pub = node_handler.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    uint32_t shape = visualization_msgs::Marker::SPHERE;
#endif  // ducta --

    ros::Rate loop_rate(300);

    // Joint No
    _jointNo = KsGlobal::VMY_ARM == armId ? KsGlobal::VMY_ARM_JOINT_TOTAL   :
               KsGlobal::VJACO_ARM        ? KsGlobal::VJACO_ARM_JOINT_TOTAL : 0;
    _linkNo = _jointNo + 1;
    if(_jointNo <= 0) {
        ROS_ERROR_ONCE("The robot has no joint???");
        return;
    }

    // Joint Pos
    _joint_poses = new double[_jointNo];
    for(size_t i = 0; i < _jointNo; i++) {
        _joint_poses[i] = 0;
    }

    // Frame Transforms
    _frame_trans = new tf::StampedTransform[_jointNo+1];
    for(size_t i = 0; i < _jointNo+1; i++) {
        _frame_trans[i].setRotation(tf::Quaternion(0,0,0,0));
    }

    // Run the ros loop
    //
    while (ros::ok())
    {
        // =====================================================================================
        // Listen for frame transforms
        //
        detectFrameTransforms(armId, trans_listener);
        determineRobotOperationLimit();
        // =====================================================================================
        // Publish Joint State
        publishJointState(armId, joint_pub, joint_state);

        // =====================================================================================
        // Move Robot to a position
        moveRobot(world_trans, trans_broadcaster);

        // =====================================================================================
        // Send Markers
        publishMarkers(marker_pub, shape, tf::Vector3(0.1f, 0.1f, 0.1f), _marker_pos);

        // =====================================================================================
        // This will adjust as needed per iteration
        loop_rate.sleep();
    } //do ros things.
}

void RobotThread::detectFrameTransforms(int robotId, const tf::TransformListener& listener)
{
    QMutex * pMutex = new QMutex();
    tf::StampedTransform transform;
    //
    static bool flag = false;
    switch(robotId) {
        case KsGlobal::VMY_ARM:
            pMutex->lock();
            for(size_t i = 0; i < _linkNo; i++) {
                try {
                    listener.waitForTransform(CBASE_LINK, CMY_ARM_LINKS[i],
                                             ros::Time(0), ros::Duration(3.0)); // !!!
                    listener.lookupTransform(CBASE_LINK, CMY_ARM_LINKS[i],
                                             ros::Time(0), transform);
                }
                catch (tf::TransformException e) {
                    ROS_ERROR("Error looking up transform of %s relative to base_link", CMY_ARM_LINKS[i]);
                    ROS_ERROR("%s",e.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }

                // -----------------------------------------------------------------------------------
                // Store new transform!
                if(!(_frame_trans[i] == transform)) {
                    _frame_trans[i] = transform;
                    tf::Vector3 point(0,0,0);
                    tf::Vector3 point_bl = transform * point;
                    //ROS_INFO("%s: %f %f %f", CMY_ARM_LINKS[i], point_bl[0], point_bl[1], point_bl[2]);
                }
            }
        break;
    }
    pMutex->unlock();
    delete pMutex;
}

void RobotThread::moveRobot(geometry_msgs::TransformStamped& world_trans,
                            tf::TransformBroadcaster& broadcaster,
                            const tf::Quaternion& quaternion)
{
    // WHOLE ROBOT MOVEMENT --------------------------------------------------------
    // update transform
    // (moving in a circle with radius=2)
    world_trans.header.stamp            = ros::Time::now();
    world_trans.transform.translation.x = quaternion.x();
    world_trans.transform.translation.y = quaternion.y();
    world_trans.transform.translation.z = quaternion.z();
    world_trans.transform.rotation      = tf::createQuaternionMsgFromYaw(quaternion.w());

    // Broadcast the transform
    broadcaster.sendTransform(world_trans);
}

void RobotThread::publishJointState(int robotId,
                                    const ros::Publisher& joint_pub,
                                    sensor_msgs::JointState& joint_state)
{
    QMutex * pMutex = new QMutex();
    static double angle = 0, tilt = 0, tinc = degree, height=0, hinc=0.005;
    if(_jointNo <= 0) return;
    static int jointNo = 0;
    static double *current_joint_poses = nullptr;

    // Initialize local current joint poses info
    //
    if(jointNo != _jointNo) {
        jointNo = _jointNo;
        V_DELETE_POINTER_ARRAY(current_joint_poses);
        current_joint_poses = new double[_jointNo];
        for(size_t i = 0; i < _jointNo; i++) {
            current_joint_poses[i] = 0;
        }
    }

    // -----------------------------------------------------------------------------------
    // Joint State
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(_jointNo);
    joint_state.position.resize(_jointNo);

    switch(robotId) {
        case KsGlobal::VMY_ARM:
            for(size_t i = 0; i < _jointNo; i++) {
                joint_state.name[i]     = CMY_ARM_JOINTS[i];
                joint_state.position[i] = current_joint_poses[i];
            }

            // JOINT STATE PUBLISHING ====================================================
            //send the joint state and transform
            joint_pub.publish(joint_state);

            // JOINT STATE PARAMETERS PREPARATION ========================================
            // Create new robot state
            tilt += tinc;
            if (tilt<-.5 || tilt>0) tinc *= -1;
            height += hinc;
            if (height>.2 || height<0) hinc *= -1;

            // !NOTE: THE PROBLEM IS WE MUST LET THE JOINT KEEP PUBLISHING ITS STATE INFO
            // Each time _joint_poses[i] is set to a new value,
            // The joint rotates until current_joint_poses meets _joint_poses[i] then halts!
            pMutex->lock();
            for(size_t i = 0; i < _jointNo; i++) {
                // 1-
                if(qAbs(current_joint_poses[i] - _joint_poses[i]) <= degree) {
                    if(current_joint_poses[i] > M_PI) current_joint_poses[i] -= M_PI;
                    // Stop rotating, though still keep publishing its state with current angle!
                    _joint_poses[i] = current_joint_poses[i];
                }
                // 2.1-
                else if(current_joint_poses[i] < _joint_poses[i]) {
                    current_joint_poses[i] += degree;
                }
                // 2.2-
                else if(current_joint_poses[i] > _joint_poses[i]) {
                    current_joint_poses[i] -= degree;
                }
            }
            pMutex->unlock();
        break;
        // END MY ARM OPERATION ===================================================================================

        // START JACO ARM OPERATION ===============================================================================
        //
        case KsGlobal::VJACO_ARM:
            for(size_t i = 0; i < _jointNo; i++) {
                joint_state.name[i]     = CMY_ARM_JOINTS[i];
                joint_state.position[i] = _joint_poses[i];
            }

            // JOINT STATE PUBLISHING ====================================================
            //send the joint state and transform
            joint_pub.publish(joint_state);

            // JOINT STATE PARAMETERS PREPARATION ========================================
            // Create new robot state
            tilt += tinc;
            if (tilt<-.5 || tilt>0) tinc *= -1;
            height += hinc;
            if (height>.2 || height<0) hinc *= -1;

            // !NOTE: THE PROBLEM IS WE MUST LET THE JOINT KEEP PUBLISHING ITS STATE INFO
            // Each time _joint_poses[i] is set to a new value,
            // The joint rotates until current_joint_poses meets _joint_poses[i] then halts!
            pMutex->lock();
            for(size_t i = 0; i < _jointNo; i++) {
                // 1-
                if(qAbs(current_joint_poses[i] - _joint_poses[i]) <= degree) {
                    if(current_joint_poses[i] > M_PI) current_joint_poses[i] -= M_PI;
                    // Stop rotating, though still keep publishing its state with current angle!
                    _joint_poses[i] = current_joint_poses[i];
                }
                // 2.1-
                else if(current_joint_poses[i] < _joint_poses[i]) {
                    current_joint_poses[i] += degree;
                }
                // 2.2-
                else if(current_joint_poses[i] > _joint_poses[i]) {
                    current_joint_poses[i] -= degree;
                }
            }
            pMutex->unlock();
        break;
    } //switch(armId)

    V_DELETE_POINTER(pMutex);
}

void RobotThread::publishMarkers(const ros::Publisher& marker_pub, uint32_t& shape,
                                 const tf::Vector3& scale,
                                 const tf::Vector3& pos,
                                 const tf::Quaternion& orient)
{
    QMutex * pMutex = new QMutex();
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = CWORLD_FRAME;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    // geometry_msgs::Quaternion <- tf::Quaternion
    marker.pose.orientation.x = orient.x();
    marker.pose.orientation.y = orient.y();
    marker.pose.orientation.z = orient.z();
    marker.pose.orientation.w = orient.w();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = scale.x();
    marker.scale.y = scale.y();
    marker.scale.z = scale.z();

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);

#if 0
    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
#endif
    delete pMutex;
}

std::string RobotThread::getRobotNodeName(int robotId)
{
    switch(robotId) {
    case KsGlobal::VMY_ARM:
        return CMY_ARM_NODE_NAME;
    case KsGlobal::VJACO_ARM:
        return CJACO_ARM_NODE_NAME;
    }
}

void RobotThread::run()
{
    this->runArmOperation(CRUN_ROBOT);
#if 0
        pMutex = new QMutex();

        geometry_msgs::Twist cmd_msg;
        pMutex->lock();
        cmd_msg.linear.x = m_speed;
        cmd_msg.angular.z = m_angle;
        pMutex->unlock();

        sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
        delete pMutex;
#endif
}

void RobotThread::SetSpeed(double speed, double angle)
{
    //QMutex * pMutex = new QMutex();
    //pMutex->lock();
    //_speed = speed;
    //_angle = angle;
    //pMutex->unlock();
    //delete pMutex;
}//set the speed of the robot.

void RobotThread::rotateJoint(int jointId, double pos, bool updateBallPos)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();

    if(jointId >= 0 && jointId < _jointNo) {
        _joint_poses[jointId] = pos;
    }
    pMutex->unlock();
    delete pMutex;
    // -----------------------------------------------
    //
    if(updateBallPos) {
        this->updateBallFollowingEndTip();
    }
}

void RobotThread::updateBallFollowingEndTip()
{
    // Move Ball to the end-tip
    tf::Vector3 endTipPos = determineEndTipPos();
    //ROS_INFO("HAND POS: %f %f %f", endTipPos[0], endTipPos[1], endTipPos[2]);
    setBallPos(endTipPos);
}

void RobotThread::moveBall(const tf::Vector3& distance)
{
    QMutex * pMutex = new QMutex();

    pMutex->lock();
    _marker_pos += distance;
    this->determineArmArrangement(_marker_pos);
    pMutex->unlock();
    delete pMutex;
}

void RobotThread::setBallPos(const tf::Vector3& pos, bool armFollowOrder)
{
    QMutex * pMutex = new QMutex();

    pMutex->lock();
    _marker_pos = pos;
    pMutex->unlock();
    delete pMutex;
    // -----------------------------------------
    if(armFollowOrder) {
        this->determineArmArrangement(pos);
    }
}

double RobotThread::getXSpeed(){ return _speed; }
double RobotThread::getASpeed(){ return 0; }

double RobotThread::getXPos(){ return _xPos; }
double RobotThread::getYPos(){ return _yPos; }
double RobotThread::getAPos(){ return _aPos; }

void RobotThread::determineRobotOperationLimit()
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    _arm_reach_limit = (_frame_trans == nullptr) ? tf::Vector3(0,0,0) :
                       tf::Vector3(l2+l3+l4, l2+l3+l4, l0+l1+l2+l3+l4);
}

// FORWARD KINEMATICS ---------------------------------
//
tf::Vector3 RobotThread::determineEndTipPos()
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    return (_frame_trans != nullptr) ? (_frame_trans[KsGlobal::VJOINT3 + 1] * CLOCAL_END_TIP):
                                       tf::Vector3(0,0,0);
}

// INVERSE KINEMATICS ---------------------------------
//
void RobotThread::determineArmArrangement(const tf::Vector3& target_pos)
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    if(target_pos.x() > _arm_reach_limit.x() ||
       target_pos.y() > _arm_reach_limit.y() ||
       target_pos.z() > _arm_reach_limit.z()) {
        ROS_WARN_ONCE("The target is out of reach of the arm!");
        return;
    }

    // Theta 1 : Joint 2
    // CLOCAL_END_TIP already constains l4 info.
    tf::Vector3 vector = (_marker_pos - CLOCAL_END_TIP);
    double y = vector.dot(tf::Vector3(0,1,0));
    double x = vector.dot(tf::Vector3(1,0,0));
    double theta = std::atan2(y, x);

    rotateJoint(KsGlobal::VJOINT10, M_PI/2 - theta, false);
    ROS_INFO_ONCE("JOINT10: %f", theta);
    // Theta 2 : Joint 3
    //
    vector -= l1*tf::Vector3(1,0,0) + l0*tf::Vector3(1,0,0); // - l1*Z1 - l0*Z0;
    double value = (vector.length2() - l2*l2 - l3*l3) / (2*l2*l3);
    theta = std::acos(value);

    rotateJoint(KsGlobal::VJOINT20, theta, false);
    ROS_INFO_ONCE("JOINT20: %f", theta);
}
