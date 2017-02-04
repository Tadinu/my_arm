#include <string>
#include <QMutexLocker>
#include <QtMath>
#include <mutex>
#include <tf/exceptions.h>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
#include "RobotThread.h"
#include "KsGlobal.h"

// ROBOT TO RUN
#define CRUN_ROBOT (KsGlobal::VPISA_SOFT_HAND_ARM)

// NODE --
#define CMY_ARM_NODE_NAME ("robotArmController")

// TOPIC --
#define ROS_TOPIC(linkName) (std::string("//") + linkName)

const double rad = M_PI/180;
const double TWO_PI = 2 * M_PI;

const double DELTA_ERR = 0.01;

const double l0 = 0;
const double l1 = 0.7;
const double l2 = 0.7;
const double l3 = 0.25;
const double l4 = 0.15;

const tf::Vector3 CLOCAL_END_TIP(0, 0, l4);

const tf::Vector3 CX(1,0,0);
const tf::Vector3 CY(0,1,0);
const tf::Vector3 CZ(0,0,1);

const tf::Vector3 CZERO(0,0,0);

const char* CBRHAND_ARM_JOINTS[KsGlobal::VBRHAND_ARM_JOINT_TOTAL] = {
    ("base_joint"),                // Revolute   Z : base_link <-> body1z
    ("j2"),                        // Continuous Y : body1     <-> body10
    ("j20"),                       // Fixed        : body10    <-> body2
    ("j3"),                        // Continuous Y : body2     <-> body20
    ("j30"),                       // Fixed        : body20    <-> body3
    ("j4"),                        // Revolute   Z : body3     <-> brHand

    ("finger_1_prox_joint"),
    ("finger_1_med_joint"),
    ("finger_1_dist_joint"),

    ("finger_2_prox_joint"),
    ("finger_2_med_joint"),
    ("finger_2_dist_joint"),

    ("finger_3_med_joint"),
    ("finger_3_dist_joint")
};

const char* CBRHAND_ARM_LINKS[KsGlobal::VBRHAND_ARM_JOINT_TOTAL+1] = {
    CBASE_LINK,
    ("body2"),
    ("body20"),
    ("body3"),
    ("body30"),
    ("body4"),

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

const char* CPISA_SOFT_HAND_ARM_JOINTS[KsGlobal::VPISA_SOFT_HAND_ARM_JOINT_TOTAL] = {
    ("base_joint"),                // Revolute   Z : base_link <-> body1z
    ("j2"),                        // Continuous Y : body1     <-> body10
    ("j20"),                       // Fixed        : body10    <-> body2
    ("j3"),                        // Continuous Y : body2     <-> body20
    ("j30"),                       // Fixed        : body20    <-> body3
    ("j4"),                        // Revolute   Z : body3     <-> brHand

    ("softHand_thumb_abd_joint"),
    ("softHand_thumb_inner_joint"),
    ("softHand_thumb_outer_joint"),

    ("softHand_index_abd_joint"),
    ("softHand_index_inner_joint"),
    ("softHand_index_middle_joint"),
    ("softHand_index_outer_joint"),

    ("softHand_middle_abd_joint"),
    ("softHand_middle_inner_joint"),
    ("softHand_middle_middle_joint"),
    ("softHand_middle_outer_joint"),

    ("softHand_ring_abd_joint"),
    ("softHand_ring_inner_joint"),
    ("softHand_ring_middle_joint"),
    ("softHand_ring_outer_joint"),

    ("softHand_little_abd_joint"),
    ("softHand_little_inner_joint"),
    ("softHand_little_middle_joint"),
    ("softHand_little_outer_joint")
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
              _robot_pos(tf::Vector3(0,0,0)),
              _pMutex(new QMutex(QMutex::Recursive))
{/** Constructor for the robot thread **/

}

RobotThread::~RobotThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }//end if

    // VMarker--
    //
    VMarker::deleteInstace();

    // RobotLeapAdapter --
    //
    RobotLeapAdapter::deleteInstance();

    // Local resource mutex --
    //
    _pMutex->tryLock(5000);
    _pMutex->unlock();

    V_DELETE_POINTER_ARRAY(_joint_poses);
    V_DELETE_POINTER_ARRAY(_frame_trans);

    if(!_pThread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
    {
        qWarning("Thread deadlock detected, bad things may have happened !!!");
        _pThread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        _pThread->wait(); //Note: We have to wait again here!
    }

    delete _pMutex;
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
    ros::init(_init_argc, _pInit_argv, CMY_ARM_NODE_NAME); // Name of the node specified in launch file
    ros::NodeHandle node_handle;

    // -------------------------------------------------------------------------------------------------------
    // MARKERS --
    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>(CRVIZ_MARKER_TOPIC_NAME, 1);
    // 1- Initialize Interactive Server
    VMARKER_INSTANCE()->initialize();// -> !VMarker SERVICE MUST BE INITIALIZED AFTER ros::init(...)!!!
    // 2- Setup static marker properties
    // Target Ball --
    VMARKER_INSTANCE()->setStaticMarkerProperties(VMarker::TARGET_BALL, CWORLD_FRAME, visualization_msgs::Marker::SPHERE, tf::Vector3(0.1f, 0.1f, 0.1f));
    // Arrow Tool --
    VMARKER_INSTANCE()->setStaticMarkerProperties(VMarker::ARROW_TOOL,  CWORLD_FRAME, visualization_msgs::Marker::ARROW,  tf::Vector3(2.0f, 2.0f, 2.0f));
    // 3- Create Interactive Markers
    VMARKER_INSTANCE()->createInteractiveMarkers();

    QObject::connect(VMARKER_INSTANCE(), &VMarker::markerPosChanged,
                     this, &RobotThread::determineArmArrangement);

    // create a timer to update the published transforms
    ros::Timer frame_timer = node_handle.createTimer(ros::Duration(0.01), &VMarker::frameCallback);

    // -------------------------------------------------------------------------------------------------------
#ifdef ROBOT_LEAP_HANDS
    // LEAP HANDS --
    VLEAP_INSTANCE()->initLeapMotion(&node_handle);
#endif

    // =======================================================================================================
    // MAIN ROS SPINNING LOOP --
    //
#endif  // ducta --
    ros::Rate loop_rate(300);

    // Joint No
    _jointNo = KsGlobal::VBRHAND_ARM         == armId ? KsGlobal::VBRHAND_ARM_JOINT_TOTAL         :
               KsGlobal::VJACO_ARM           == armId ? KsGlobal::VJACO_ARM_JOINT_TOTAL           :
               KsGlobal::VPISA_SOFT_HAND_ARM == armId ? KsGlobal::VPISA_SOFT_HAND_ARM_JOINT_TOTAL : 0;
    if(_jointNo <= 0) {
        ROS_ERROR_ONCE("The robot has no joint???");
        return;
    }

    // -------------------------------------------------------------------------------------------------------
    // ROBOT --
    //
    // message declarations
    sensor_msgs::JointState joint_state;
    _world_trans.header.frame_id = CWORLD_FRAME;
    _world_trans.child_frame_id  = CBASE_LINK;

    ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
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
    // By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok(),
    // to return false if that happens.
    //
    // ros::ok() will return false if:
    //
    //     a SIGINT is received (Ctrl-C)
    //     we have been kicked off the network by another node with the same name
    //
    //     ros::shutdown() has been called by another part of the application.
    //
    //     all ros::NodeHandles have been destroyed
    //
    // Once ros::ok() returns false, all ROS calls will fail.
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
        publishRobotPose(armId, _world_trans, trans_broadcaster);

        // =====================================================================================
        // Send Static Markers

        // Arrow Marker
        //publishStaticMarkers(marker_pub, VMARKER_INSTANCE()->getStaticMarker(VMarker::TARGET_BALL));

        // =====================================================================================
        // Listen for Leap Motion hand data
        determineHandArrangmentOnLeapHands(armId);

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
        // To let the node receive some callbacks.
        // If we were to add a subscription into this application, and did not have ros::spinOnce() here,
        // your callbacks would never get called. So, add it for good measure!
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
        case KsGlobal::VBRHAND_ARM:
            for(size_t i = 0; i < (KsGlobal::VBRHAND_ARM_JOINT_TOTAL+1); i++) {
                try {
                    listener.waitForTransform(CBASE_LINK, CBRHAND_ARM_LINKS[i],
                                             ros::Time(0), ros::Duration(3.0)); // !!!
                    listener.lookupTransform(CBASE_LINK, CBRHAND_ARM_LINKS[i],
                                             ros::Time(0), transform);
                }
                catch (tf::TransformException e) {
                    ROS_ERROR("Error looking up transform of %s relative to base_link", CBRHAND_ARM_LINKS[i]);
                    ROS_ERROR("%s",e.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }

                // -----------------------------------------------------------------------------------
                // Store new transform!
                if(!(_frame_trans[i] == transform)) {
                    _frame_trans[i] = transform;
                    //tf::Vector3 point(0,0,0);
                    //tf::Vector3 point_bl = transform * point;
                    //ROS_INFO("%s: %f %f %f", CBRHAND_ARM_LINKS[i], point_bl[0], point_bl[1], point_bl[2]);
                }
            }
        break;
    }
    V_UNLOCK_DELETE_PMUTEX(pMutex);
}

double RobotThread::distanceToJoin(int jointId, const tf::Vector3& point)
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex); //std::lock_guard(mutex&)
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
    _world_trans.header.stamp            = ros::Time::now();
    _world_trans.transform.translation.x = pos.x();
    _world_trans.transform.translation.y = pos.y();
    _world_trans.transform.translation.z = 0; // current_robot_pos.z();
    _world_trans.transform.rotation      = tf::createQuaternionMsgFromYaw(0.0f);

    this->resetRobotPosture();
}

void RobotThread::resetRobotPosture()
{
    rotateJoint(KsGlobal::VBRHAND_ARM_BASE_JOINT, 0, false);
    rotateJoint(KsGlobal::VJOINT2          , 0, false);
    rotateJoint(KsGlobal::VJOINT3          , 0, false);
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
    world_trans.transform.translation.z = ((robotId == KsGlobal::VBRHAND_ARM) ||
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
        case KsGlobal::VBRHAND_ARM:
        case KsGlobal::VPISA_SOFT_HAND_ARM:
            for(size_t i = 0; i < _jointNo; i++) {
                joint_state.name[i]     = (KsGlobal::VBRHAND_ARM         == robotId) ? CBRHAND_ARM_JOINTS[i]         :
                                          (KsGlobal::VPISA_SOFT_HAND_ARM == robotId) ? CPISA_SOFT_HAND_ARM_JOINTS[i] : "";
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
                joint_state.name[i]     = CJACO_ARM_JOINTS[i];
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
    // This function maybe updated in the future where we have multiple robots in the playground
    return CMY_ARM_NODE_NAME;
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
        //ROS_INFO("ROBOT HAND POS LOCAL : %f %f %f", endTipPos[0], endTipPos[1], endTipPos[2]);
        endTipPos = this->convertRobotFrame2WorldPos(endTipPos);
        //ROS_INFO("ROBOT HAND POS GLOBAL : %f %f %f", endTipPos[0], endTipPos[1], endTipPos[2]);
        setBallPos(endTipPos);
    }
}

void RobotThread::moveBall(const tf::Vector3& distance)
{
    VMARKER_INSTANCE()->moveInteractiveMarkerPos(distance);
    this->determineArmArrangement(V_TF_2_QVECTOR3D(VMARKER_INSTANCE()->getStaticMarkerPos(VMarker::TARGET_BALL)));
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

    if(_frame_trans != nullptr) {
        VMARKER_INSTANCE()->setInteractiveMarkerPosLimit(tf::Vector3(10, 10, _arm_reach_limit.z()));
    }
}

// FORWARD KINEMATICS ---------------------------------
//
tf::Vector3 RobotThread::determineEndTipLocalPos()
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    return (_frame_trans != nullptr) ? (_frame_trans[KsGlobal::VJOINT4 + 1] * CLOCAL_END_TIP):
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
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    tf::StampedTransform world_stamped_trans;
    tf::transformStampedMsgToTF(_world_trans, world_stamped_trans);
    return  world_stamped_trans.inverse() * world_pos;
}

tf::Vector3 RobotThread::convertRobotFrame2WorldPos(const tf::Vector3& robot_local_pos)
{
    QMutex pMutex;
    QMutexLocker locker(&pMutex);
    tf::StampedTransform world_stamped_trans;
    tf::transformStampedMsgToTF(_world_trans, world_stamped_trans);
    return  world_stamped_trans * robot_local_pos;
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
#if 0
        double move_delta = target_pos_x_y.distance(robot_pos) - (_arm_reach_limit.x() - DELTA_ERR); // x, y reach limit are the same!
        if(move_delta > 0) {
            ROS_WARN("The target is out of reach of the arm horizontally! -> moving %f", move_delta);
            tf::Vector3 move_vector = (target_pos_x_y - robot_pos).normalize() * move_delta;
            // Move Robot until within arm reach limit
            setRobotPos(robot_pos + move_vector);
        }
#endif
    }

    // [Farther from joint2 than length2]
    //
    double distance_to_joint_2 = distanceToJoin(KsGlobal::VJOINT2, target_pos);
    bool within_length_2    = (distance_to_joint_2 < (l2 - DELTA_ERR));
    bool outside_length_234 = (distance_to_joint_2 > (l2 + l3 + l4 + DELTA_ERR));

#if 0
    if(within_length_2 ||  outside_length_234) {
        if(within_length_2) {
            ROS_WARN("The target is within length 2!");
        }
        else {
            ROS_WARN("The target is out of length 2-3-4!");
        }

        tf::Vector3 v1 = tf::Vector3(target_pos.x(), target_pos.y(), l1) - tf::Vector3(robot_pos.x(), robot_pos.y(), l1);
        double move_delta_1 = std::sqrt(std::pow(l2+l3+l4-DELTA_ERR, 2) - std::pow(target_pos.z() - l1, 2));
        double move_delta_2 = v1.length() - move_delta_1;

        //setRobotPos(robot_pos + (v1.normalize() * move_delta_2)); // move_delta_2 >0 : l2+l3+l4 <= distance_to_joint_2
        setRobotPos(tf::Vector3(target_pos.x(), target_pos.y(), 0) - (v1.normalize() * move_delta_1));
    }
#endif

    // Re-Localize the target pos relative to base joint first (Base Joint == Robot Root Pos)
    //
    target_pos = this->convertWorld2RobotFramePos(target_pos); // based on _world_trans, which holds _robot_pos
    // -------------------------------------------------------------------------------------------
    //
    // Theta 1: Joint 1
    // CLOCAL_END_TIP already constains l4 info.
    //
    tf::Vector3 vector = (target_pos - CLOCAL_END_TIP); // PE - l4*Z4;
    double y = vector.dot(CY);
    double x = vector.dot(CX);
    double theta1 = std::atan2(y, x);
    ROS_INFO("BASE_JOINT: %f (degree)", RAD_2_ANGLE(theta1));

    // Theta 3: Joint 3
    //
    vector -= l1*CZ + l0*CZ; // (PE - l4*Z4) - l1*Z1 - l0*Z0; // l0 == 0
    double cos_theta3  = (vector.length2() - l2*l2 - l3*l3) / (2*l2*l3);
    ROS_INFO("Cos theta3: %f", cos_theta3);
    //
    if(std::abs(cos_theta3) > 1) {
        cos_theta3 = 1;
        ROS_INFO("JOINT 3 value set to 0 rad!");
    }
    double theta3 = std::acos(cos_theta3); // [0, PI]

    // Theta 2: Joint 2
    //
    double sinRad = vector.dot(CZ); // dot Z0
    sinRad /= std::sqrt(std::pow(l2 + l3*cos_theta3, 2) + l3*l3*(1-cos_theta3*cos_theta3));

    if(std::abs(sinRad) > 1) sinRad = 1;
    double tanRad = (l2 + l3*cos_theta3) / (-l3 * std::sin(theta3));
    double theta2 =  (std::asin(sinRad) - std::atan(tanRad)); // M_PI -
    ROS_INFO("theta 2: %f (rad)", theta2);
    //
    //if(theta2 < M_PI/2)
    //    theta2 = M_PI/2;
    //else if(theta2 < (M_PI/2 - 50*DELTA_ERR))
    //    theta2 += M_PI/2;

    //
    rotateJoint(KsGlobal::VBRHAND_ARM_BASE_JOINT, theta1, false);
    rotateJoint(KsGlobal::VJOINT2          , theta2, false);
    ROS_INFO("JOINT 2: %f (degree)", RAD_2_ANGLE(theta2));
    rotateJoint(KsGlobal::VJOINT3          , theta3, false);
    ROS_INFO("JOINT 3: %f (degree)", RAD_2_ANGLE(theta3));
}

void RobotThread::determineArrangement_JacoArm(tf::Vector3& target_pos)
{}


void RobotThread::determineHandArrangmentOnLeapHands(int armId)
{
    // Take the first hand data for convenience:
    std::vector<std::vector<double>> jointValues = RobotLeapAdapter::getInstance()->getFingerJointValues(0);
    if(jointValues.size() == 0) {
        return;
    }

    //ROS_INFO("Fingers joints: %f %f %f", jointValues[0][0], jointValues[0][1], jointValues[0][2]);
    //ROS_INFO("Fingers joints: %f %f %f", jointValues[1][0], jointValues[1][1], jointValues[1][2]);
    //ROS_INFO("Fingers joints: %f %f %f", jointValues[2][0], jointValues[2][1], jointValues[2][2]);
    switch(armId) {
    case KsGlobal::VPISA_SOFT_HAND_ARM:
        rotateJoint(KsGlobal::VPISA_FINGER_THUMB_ABD_JOINT  , jointValues[0][0], false);
        rotateJoint(KsGlobal::VPISA_FINGER_THUMB_INNER_JOINT, jointValues[0][1], false);
        rotateJoint(KsGlobal::VPISA_FINGER_THUMB_OUTER_JOINT, jointValues[0][2], false);

        rotateJoint(KsGlobal::VPISA_FINGER_1_ABD_JOINT      , jointValues[1][0], false);
        rotateJoint(KsGlobal::VPISA_FINGER_1_INNER_JOINT    , jointValues[1][1], false);
        rotateJoint(KsGlobal::VPISA_FINGER_1_MIDDLE_JOINT   , jointValues[1][2], false);
        rotateJoint(KsGlobal::VPISA_FINGER_1_OUTER_JOINT    , jointValues[1][3], false);

        rotateJoint(KsGlobal::VPISA_FINGER_2_ABD_JOINT      , jointValues[2][0], false);
        rotateJoint(KsGlobal::VPISA_FINGER_2_INNER_JOINT    , jointValues[2][1], false);
        rotateJoint(KsGlobal::VPISA_FINGER_2_MIDDLE_JOINT   , jointValues[2][2], false);
        rotateJoint(KsGlobal::VPISA_FINGER_2_OUTER_JOINT    , jointValues[2][3], false);

        rotateJoint(KsGlobal::VPISA_FINGER_3_ABD_JOINT      , jointValues[3][0], false);
        rotateJoint(KsGlobal::VPISA_FINGER_3_INNER_JOINT    , jointValues[3][1], false);
        rotateJoint(KsGlobal::VPISA_FINGER_3_MIDDLE_JOINT   , jointValues[3][2], false);
        rotateJoint(KsGlobal::VPISA_FINGER_3_OUTER_JOINT    , jointValues[3][3], false);

        rotateJoint(KsGlobal::VPISA_FINGER_4_ABD_JOINT      , jointValues[4][0], false);
        rotateJoint(KsGlobal::VPISA_FINGER_4_INNER_JOINT    , jointValues[4][1], false);
        rotateJoint(KsGlobal::VPISA_FINGER_4_MIDDLE_JOINT   , jointValues[4][2], false);
        rotateJoint(KsGlobal::VPISA_FINGER_4_OUTER_JOINT    , jointValues[4][3], false);
        break;
    case KsGlobal::VBRHAND_ARM:
    case KsGlobal::VJACO_ARM:
    default:
        break;
    }
}

// =========================================================================================
