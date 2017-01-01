#include <QMutexLocker>
#include <QtMath>
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

const double degree = M_PI/180;
const double TWO_PI = 2 * M_PI;

const char* CMY_ARM_JOINTS[KsGlobal::VMY_ARM_JOINT_TOTAL] = {
    ("j20"),
    ("j2"),
    ("j3"),

    ("finger_1_prox_joint"),
    ("finger_1_med_joint"),
    ("finger_1_dist_joint"),

    ("finger_2_prox_joint"),
    ("finger_2_med_joint"),
    ("finger_2_dist_joint"),

    ("finger_3_prox_joint"),
    ("finger_3_med_joint"),
    ("finger_3_dist_joint")
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
              _angleDelta(0)
{/** Constructor for the robot thread **/}

RobotThread::~RobotThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }//end if

    //m_pMutex->tryLock(5000);
    //m_pMutex->unlock();

    if(!_pThread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
    {
        qWarning("Thread deadlock detected, bad things may have happened !!!");
        _pThread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        _pThread->wait(); //Note: We have to wait again here!
    }
}//end destructor

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
    tf::TransformBroadcaster broadcaster;

    // ------------------------------------------------------------------------------
    // MARKERS --
    ros::Publisher marker_pub = node_handler.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    uint32_t shape = visualization_msgs::Marker::CUBE;
#endif  // ducta --

    ros::Rate loop_rate(300);
    //
    while (ros::ok())
    {
        // =====================================================================================
        // Send Robot
        sendRobot(armId, world_trans, broadcaster, joint_pub, joint_state);

        // =====================================================================================
        // Send Markers
        //sendMarkers(marker_pub, shape, _marker_pos);

        // =====================================================================================
        // This will adjust as needed per iteration
        loop_rate.sleep();
    } //do ros things.
}
void RobotThread::setMarkerPos(const QVector3D& markerPos)
{
    QMutex * pMutex = new QMutex();

    pMutex->lock();
    _marker_pos = markerPos;
    pMutex->unlock();
    delete pMutex;
}

void RobotThread::sendRobot(int robotId, geometry_msgs::TransformStamped& world_trans,
                            tf::TransformBroadcaster& broadcaster,
                            ros::Publisher& joint_pub, sensor_msgs::JointState& joint_state)
{
    QMutex * pMutex = new QMutex();
    static double tilt = 0, tinc = degree, height=0, hinc=0.005;
    static double angle = 0;
    static double *joint_values = nullptr;
    static int jointNo = 0;

    joint_state.header.stamp = ros::Time::now();
    // Joint No
    jointNo = KsGlobal::VMY_ARM == robotId ? KsGlobal::VMY_ARM_JOINT_TOTAL   :
              KsGlobal::VJACO_ARM          ? KsGlobal::VJACO_ARM_JOINT_TOTAL : 0;
    if(jointNo <= 0)
        return;
    // ---------------------------------------------------------------------------
    joint_values = new double[jointNo];

    // Joint State
    joint_state.name.resize(jointNo);
    joint_state.position.resize(jointNo);

    // WHOLE ROBOT MOVEMENT --------------------------------------------------------
    // update transform
    // (moving in a circle with radius=2)
    world_trans.header.stamp            = ros::Time::now();
    world_trans.transform.translation.x = cos(angle)*2;
    world_trans.transform.translation.y = sin(angle)*2;
    world_trans.transform.translation.z = .7;
    world_trans.transform.rotation      = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

    switch(robotId) {
        case KsGlobal::VMY_ARM:
            for(size_t i = 0; i < jointNo; i++) {
                joint_state.name[i]     = CMY_ARM_JOINTS[i];
                joint_state.position[i] = joint_values[i];
            }

            // JOINT STATE PUBLISHING ====================================================
            //send the joint state and transform
            joint_pub.publish(joint_state);
            broadcaster.sendTransform(world_trans);

            // JOINT STATE PARAMETERS PREPARATION ========================================
            // Create new robot state
            tilt += tinc;
            if (tilt<-.5 || tilt>0) tinc *= -1;
            height += hinc;
            if (height>.2 || height<0) hinc *= -1;

            // !NOTE: THE PROBLEM IS WE MUST LET THE JOINT KEEP PUBLISHING ITS STATE INFO
            // Each time m_angleDelta is set to a new value,
            // The joint rotates until angleDeta reaches m_angleDelta then halts!
            pMutex->lock();
            if((angle - _angle) < _angleDelta) {
                joint_values[0] += degree;
                angle += degree/4;
            }
            else {
                if(angle > TWO_PI) angle -= TWO_PI;
                // Stop rotating, though still keep publishing its state with current angle!
                _angle = angle;
                _angleDelta = 0;
            }
            pMutex->unlock();
        break;
        // END MY ARM OPERATION ===================================================================================

        // START JACO ARM OPERATION ===============================================================================
        //
        case KsGlobal::VJACO_ARM:
            for(size_t i = 0; i < jointNo; i++) {
                joint_state.name[i]     = CMY_ARM_JOINTS[i];
                joint_state.position[i] = joint_values[i];
            }

            // JOINT STATE PUBLISHING ====================================================
            //send the joint state and transform
            joint_pub.publish(joint_state);
            broadcaster.sendTransform(world_trans);

            // JOINT STATE PARAMETERS PREPARATION ========================================
            // Create new robot state
            tilt += tinc;
            if (tilt<-.5 || tilt>0) tinc *= -1;
            height += hinc;
            if (height>.2 || height<0) hinc *= -1;

            // !NOTE: THE PROBLEM IS WE MUST LET THE JOINT KEEP PUBLISHING ITS STATE INFO
            // Each time m_angleDelta is set to a new value,
            // The joint rotates until angleDeta reaches m_angleDelta then halts!
            pMutex->lock();
            if((angle - _angle) < _angleDelta) {
                joint_values[0] += degree;
                angle += degree/4;
            }
            else {
                if(angle > TWO_PI) angle -= TWO_PI;
                // Stop rotating, though still keep publishing its state with current angle!
                _angle = angle;
                _angleDelta = 0;
            }
            pMutex->unlock();
        break;
    } //switch(armId)

    V_DELETE_POINTER(pMutex);
    V_DELETE_POINTER_ARRAY(joint_values);
}

void RobotThread::sendMarkers(const ros::Publisher& marker_pub, uint32_t& shape,
                              const QVector3D& pos, const QVector3D& orient, double rotAngle,
                              const QVector3D& scale)
{
    //QMutex * pMutex = new QMutex();
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
    marker.pose.orientation.x = orient.x();
    marker.pose.orientation.y = orient.y();
    marker.pose.orientation.z = orient.z();
    marker.pose.orientation.w = qCos(rotAngle);

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
    //delete pMutex;
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
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    _speed = speed;
    _angle = angle;
	pMutex->unlock();
	delete pMutex;
}//set the speed of the robot.

void RobotThread::rotateJoint(int jointId, double angleDelta)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    _angleDelta = angleDelta;
	pMutex->unlock();
	delete pMutex;
}

double RobotThread::getXSpeed(){ return _speed; }
double RobotThread::getASpeed(){ return _angle; }

double RobotThread::getXPos(){ return _xPos; }
double RobotThread::getYPos(){ return _yPos; }
double RobotThread::getAPos(){ return _aPos; }

