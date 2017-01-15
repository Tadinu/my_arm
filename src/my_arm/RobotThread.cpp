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

#define ROS_TOPIC(linkName) (std::string("//") + linkName)

const double rad = M_PI/180;
const double TWO_PI = 2 * M_PI;

const double DELTA_ERR = 0.01;

const double l0 = 0;
const double l1 = 0.7;
const double l2 = 0.7;
const double l3 = 0.15;
const double l4 = 0.025;

const tf::Vector3 CLOCAL_END_TIP(0, 0, l4);

const tf::Vector3 CX(1,0,0);
const tf::Vector3 CY(0,1,0);
const tf::Vector3 CZ(0,0,1);

const tf::Vector3 CZERO(0,0,0);

const char* CMY_ARM_JOINTS[KsGlobal::VMY_ARM_JOINT_TOTAL] = {
    ("base_joint"),                // Revolute   Z : base_link <-> body1
    ("j1"),                        // Continuous Y : body1     <-> body10
    ("j10"),                       // Fixed        : body10    <-> body2
    ("j2"),                        // Continuous Y : body2     <-> body20
    ("j20"),                       // Fixed        : body20    <-> body3
    ("j3"),                        // Revolute   Z : body3     <-> brHand

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
              _frame_trans(nullptr),
              _arm_reach_limit(tf::Vector3(0,0,0)),
              _robot_pos(tf::Vector3(0,0,0))
{/** Constructor for the robot thread **/
    QObject::connect(VMARKER_INSTANCE(), &VMarker::markerPosChanged,
                     this, &RobotThread::determineArmArrangement);
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

void RobotThread::run()
{
    // Robot Arm --
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

    // -------------------------------------------------------------------------------------------------------
    // MARKERS --
    ros::Publisher marker_pub = node_handler.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    VMARKER_INSTANCE()->initializeMarker(CWORLD_FRAME, visualization_msgs::Marker::SPHERE, tf::Vector3(0.1f, 0.1f, 0.1f));

    // create a timer to update the published transforms
    ros::Timer frame_timer = node_handler.createTimer(ros::Duration(0.01), &VMarker::frameCallback);

    // -------------------------------------------------------------------------------------------------------
    // LEAP HANDS --
    _robotLeapAdapter.initLeapMotion();

    // =======================================================================================================
    // MAIN ROS SPINNING LOOP --
    //
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

    // -------------------------------------------------------------------------------------------------------
    // ROBOT --
    //
    // message declarations
    geometry_msgs::TransformStamped world_trans;
    sensor_msgs::JointState joint_state;
    world_trans.header.frame_id = CWORLD_FRAME;
    world_trans.child_frame_id  = CBASE_LINK;

    ros::Publisher joint_pub = node_handler.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster trans_broadcaster;
    tf::TransformListener trans_listener;

    // Joint Pos
    //
    _joint_poses = new double[_jointNo];
    for(size_t i = 0; i < _jointNo; i++) {
        _joint_poses[i] = 0;
    }

    // Frame Transforms
    //
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
        publishRobotPose(armId, world_trans, trans_broadcaster);

        // =====================================================================================
        // Send Static Markers
        //publishStaticMarkers(marker_pub, VMARKER_INSTANCE()->getMarker());

        // =====================================================================================
        // Callbacks registered from Interactive Marker Server:
        // ROS SPIN --
        // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
#if 0  // Multi-threaded
        ros::MultiThreadedSpinner spinner(4); // Use 4 threads
        spinner.spin(); // spin() will not return until the node has been shutdown

        OR

        ros::AsyncSpinner spinner(4); // Use 4 threads
        spinner.start();
        ros::waitForShutdown();
#else // Single-threaded
        ros::spinOnce();
        // <=> ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
#endif
        // =====================================================================================
        // This will adjust as needed per iteration
        loop_rate.sleep();
    } //do ros things.
}

void RobotThread::detectFrameTransforms(int robotId, const tf::TransformListener& listener)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    tf::StampedTransform transform;
    //
    static bool flag = false;
    switch(robotId) {
        case KsGlobal::VMY_ARM:
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
    V_UNLOCK_DELETE_PMUTEX(pMutex);
}

double RobotThread::distanceToJoin(int jointId, const tf::Vector3& point)
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    tf::Vector3 jointPoint = _frame_trans[jointId] * tf::Vector3(0, 0, 0);
    return jointPoint.distance(point);
}

tf::Vector3 RobotThread::getRobotPos()
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    return _robot_pos;
}

void RobotThread::setRobotPos(const tf::Vector3 &pos)
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    _robot_pos = pos;
}

void RobotThread::publishRobotPose(int robotId,
                                   geometry_msgs::TransformStamped& world_trans,
                                   tf::TransformBroadcaster& broadcaster)
{
    QMutex* pMutex = new QMutex();
    pMutex->lock();
    static tf::Vector3 current_robot_pos = tf::Vector3(0,0,0);
    tf::Vector3 delta = _robot_pos - current_robot_pos;

    double deltaLength = delta.length();
    if(deltaLength > DELTA_ERR) {
        current_robot_pos += (delta.normalize() * DELTA_ERR);
        //ROS_INFO("Current robot pos: %f %f %f", current_robot_pos.x(), current_robot_pos.y(), current_robot_pos.z());
    }
    else if(current_robot_pos != _robot_pos) {
        current_robot_pos = _robot_pos;
    }
    pMutex->unlock();
    //
    //
    // WHOLE ROBOT MOVEMENT --------------------------------------------------------
    // update transform
    // (moving in a circle with radius=2)
    world_trans.header.stamp            = ros::Time::now();
    world_trans.transform.translation.x = current_robot_pos.x();
    world_trans.transform.translation.y = current_robot_pos.y();
    world_trans.transform.translation.z = ((robotId == KsGlobal::VMY_ARM) ||
                                           (robotId == KsGlobal::VJACO_ARM)) ? 0 : current_robot_pos.z();
    world_trans.transform.rotation      = tf::createQuaternionMsgFromYaw(0.0f);

    // Broadcast the transform
    broadcaster.sendTransform(world_trans);
}

void RobotThread::publishJointState(int robotId,
                                    const ros::Publisher& joint_pub,
                                    sensor_msgs::JointState& joint_state)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    static double angle = 0, tilt = 0, tinc = rad, height=0, hinc=0.005;
    if(_jointNo <= 0) {
        V_UNLOCK_DELETE_PMUTEX(pMutex);
        return;
    }
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
            for(size_t i = 0; i < _jointNo; i++) {
                // 1-
                if(current_joint_poses[i] != _joint_poses[i] &&
                   qAbs(current_joint_poses[i] - _joint_poses[i]) <= rad) {
                    //if(current_joint_poses[i] > M_PI) current_joint_poses[i] -= M_PI;
                    // Stop rotating, though still keep publishing its state with current angle!
                    current_joint_poses[i] = _joint_poses[i];
                }
                // 2.1-
                else if(current_joint_poses[i] < _joint_poses[i]) {
                    current_joint_poses[i] += rad;
                }
                // 2.2-
                else if(current_joint_poses[i] > _joint_poses[i]) {
                    current_joint_poses[i] -= rad;
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
                if(current_joint_poses[i] != _joint_poses[i] &&
                   qAbs(current_joint_poses[i] - _joint_poses[i]) <= rad) {
                    //if(current_joint_poses[i] > M_PI) current_joint_poses[i] -= M_PI;
                    // Stop rotating, though still keep publishing its state with current angle!
                    current_joint_poses[i] = _joint_poses[i];
                }
                // 2.1-
                else if(current_joint_poses[i] < _joint_poses[i]) {
                    current_joint_poses[i] += rad;
                }
                // 2.2-
                else if(current_joint_poses[i] > _joint_poses[i]) {
                    current_joint_poses[i] -= rad;
                }
            }
            pMutex->unlock();
        break;
    } //switch(armId)

    V_UNLOCK_DELETE_PMUTEX(pMutex);
}

void RobotThread::publishStaticMarkers(const ros::Publisher& marker_pub,
                                       visualization_msgs::Marker& marker)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            V_UNLOCK_DELETE_PMUTEX(pMutex);
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);

#if 0
    // Cycle between different shapes
    switch (obj_type)
    {
    case visualization_msgs::Marker::CUBE:
      obj_type = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      obj_type = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      obj_type = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      obj_type = visualization_msgs::Marker::CUBE;
      break;
    }
#endif

    V_UNLOCK_DELETE_PMUTEX(pMutex);
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
         ROS_INFO("%s: %f", CMY_ARM_JOINTS[jointId], RAD_2_ANGLE(pos));
    }
    V_UNLOCK_DELETE_PMUTEX(pMutex);
    // -----------------------------------------------
    //
    if(updateBallPos) {
        this->updateBallFollowingEndTip();
    }
}

void RobotThread::updateBallFollowingEndTip()
{
    // Move Ball to the end-tip
    tf::Vector3 endTipPos = determineEndTipLocalPos();
    if(_frame_trans != nullptr) {
        endTipPos = this->convertRobotFrame2WorldPos(endTipPos);
        ROS_INFO("HAND POS: %f %f %f", endTipPos[0], endTipPos[1], endTipPos[2]);
        if(endTipPos != CZERO) {
            setBallPos(endTipPos);
        }
    }
}

void RobotThread::moveBall(const tf::Vector3& distance)
{
    VMARKER_INSTANCE()->moveInteractiveMarkerPos(distance);
    this->determineArmArrangement(V_TF_2_QVECTOR3D(VMARKER_INSTANCE()->getStaticMarkerPos()));
}

void RobotThread::setBallPos(const tf::Vector3& pos, bool armFollowOrder)
{
    VMARKER_INSTANCE()->setInteractiveMarkerPos(pos);
    // -----------------------------------------
    if(armFollowOrder) {
        this->determineArmArrangement(V_TF_2_QVECTOR3D(pos));
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

    VMARKER_INSTANCE()->setInteractiveMarkerPosLimit(tf::Vector3(10, 10, _arm_reach_limit.z()));
}

// FORWARD KINEMATICS ---------------------------------
//
tf::Vector3 RobotThread::determineEndTipLocalPos()
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    return (_frame_trans != nullptr) ? (_frame_trans[KsGlobal::VJOINT3 + 1] * CLOCAL_END_TIP):
                                       tf::Vector3(0,0,0);
}

// INVERSE KINEMATICS ---------------------------------
//
void RobotThread::determineArmArrangement(const QVector3D& world_target_pos)
{
    tf::Vector3 tf_world_target_pos = V_QVECTOR3D_2TF(world_target_pos); //VMARKER_INSTANCE()->getInteractiveMarkerPos()
    // Localize the target pos relative to base joint first (Base Joint == Robot Root Pos)
    //
    tf_world_target_pos = this->convertWorld2RobotFramePos(tf_world_target_pos);
    determineArrangement_MyArm(tf_world_target_pos);
}

tf::Vector3 RobotThread::convertWorld2RobotFramePos(const tf::Vector3& world_pos)
{
    return (_frame_trans != nullptr) ? _frame_trans[KsGlobal::VMYARM_BASE_JOINT].inverse() * world_pos :
                                       tf::Vector3(0,0,0);
}

tf::Vector3 RobotThread::convertRobotFrame2WorldPos(const tf::Vector3& robot_local_pos)
{
    return (_frame_trans != nullptr) ? _frame_trans[KsGlobal::VMYARM_BASE_JOINT] * robot_local_pos :
                                       tf::Vector3(0,0,0);
}

void RobotThread::determineArrangement_MyArm(tf::Vector3& target_pos) // as world target pos!
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    //double dis = tf_target_pos.distance2(VMARKER_INSTANCE()->getInteractiveMarkerPos());
    //ROS_INFO("DIFF: %f", dis);
    //
    // -------------------------------------------------------------------------------------------
    // VERIFY TARGET REACHABILITY
    //
    // [Within Arm Reach Limit]
    //
    tf::Vector3 robot_pos = getRobotPos();
    tf::Vector3 target_pos_x_y(target_pos.x(), target_pos.y(), 0);
    if(qAbs(target_pos.z() - robot_pos.z()) >= _arm_reach_limit.z() - DELTA_ERR) {
        ROS_WARN("The target is out of reach of the arm!");
        return;
    }
    else {
        double move_delta = target_pos_x_y.distance(robot_pos) - (_arm_reach_limit.x() - DELTA_ERR); // x, y reach limit are the same!
        if(move_delta > 0) {
            ROS_WARN("The target is out of reach of the arm horizontally! -> moving %f", move_delta);
            tf::Vector3 move_vector = (target_pos_x_y - robot_pos).normalize() * move_delta;
            // Move Robot until within arm reach limit
            setRobotPos(robot_pos + move_vector);
        }
    }

    // [Farther from joint2 than length2]
    //
    if (distanceToJoin(KsGlobal::VJOINT2, target_pos) < l2 - DELTA_ERR) {
        ROS_WARN("The target is within length 2!");
        double moveX = l2/std::sqrt(2) - DELTA_ERR;
        double moveY = l2/std::sqrt(2) - DELTA_ERR;
        setRobotPos(target_pos - tf::Vector3(moveX, moveY, 0));
    }

    // Re-Localize the target pos relative to base joint first (Base Joint == Robot Root Pos)
    //
    target_pos = this->convertWorld2RobotFramePos(target_pos);
    // -------------------------------------------------------------------------------------------
    //
    // Theta 1 : Joint 1
    // CLOCAL_END_TIP already constains l4 info.
    //
    tf::Vector3 vector = (target_pos - CLOCAL_END_TIP); // PE - l4*Z4;
    double y = vector.dot(CY);
    double x = vector.dot(CX);
    double theta1 = std::atan2(y, x);

    rotateJoint(KsGlobal::VMYARM_BASE_JOINT, theta1, false);
    ROS_INFO("BASE_JOINT: %f", RAD_2_ANGLE(theta1));

    // Theta 3: Joint 3
    //
    vector -= l1*CZ + l0*CZ; // (PE - l4*Z4) - l1*Z1 - l0*Z0;
    double cos_theta3  = (vector.length2() - l2*l2 - l3*l3) / (2*l2*l3);
    ROS_INFO("Cos theta3: %f", cos_theta3);
    //
    if(std::abs(cos_theta3) > 1) {
        cos_theta3 = 1;
        ROS_INFO("JOINT 3 value set to 0 rad!");
    }
    double theta3 = std::acos(cos_theta3);

    // Theta 2: Joint 2
    //
    double sinRad = vector.dot(CZ); // dot Z0
    if(std::abs(sinRad) > 1) sinRad = 1;
    double tanRad = (l2 + l3*cos_theta3) / (-l3 * std::sin(theta3));
    double theta2 = std::asin(sinRad) - std::atan(tanRad);
    //
    if(theta2 > M_PI/2)
        theta2-=M_PI/2;
    else if(theta2 < M_PI/2)
        theta2+=M_PI/2;

    rotateJoint(KsGlobal::VJOINT1, theta2, false);

    rotateJoint(KsGlobal::VJOINT2, theta3, false);
}

void RobotThread::determineArrangement_JacoArm(tf::Vector3& target_pos)
{}

// =========================================================================================
