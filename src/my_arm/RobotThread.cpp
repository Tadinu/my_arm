#include "RobotThread.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <QMutexLocker>

// FRAMES --
#define CWORLD_FRAME ("world")

// ARM --
#define CBASE_LINK   ("base_link")
#define CBASE_JOINT  ("base_joint")
#define CJOINT_20    ("j20")
#define CJOINT_2     ("j2")
#define CJOINT_3     ("j3")

// HAND --
// FINGER 1 --
#define CFINGER_1_PROX_JOINT  ("finger_1_prox_joint")
#define CFINGER_1_MED_JOINT   ("finger_1_med_joint")
#define CFINGER_1_DIST_JOINT  ("finger_1_dist_joint")

#define CFINGER_2_PROX_JOINT  ("finger_2_prox_joint")
#define CFINGER_2_MED_JOINT   ("finger_2_med_joint")
#define CFINGER_2_DIST_JOINT  ("finger_2_dist_joint")

#define CFINGER_3_PROX_JOINT  ("finger_3_prox_joint")
#define CFINGER_3_MED_JOINT   ("finger_3_med_joint")
#define CFINGER_3_DIST_JOINT  ("finger_3_dist_joint")

RobotThread::RobotThread(int argc, char** pArgv, const char * topic)
            : m_Init_argc(argc),
              m_pInit_argv(pArgv),
              m_topic(topic),
              m_angleDelta(0)
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

    if(!m_pThread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
    {
        qWarning("Thread deadlock detected, bad things may happen !!!");
        m_pThread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        m_pThread->wait(); //Note: We have to wait again here!
    }
}//end destructor

bool RobotThread::init()
{
    m_pThread = new QThread();
    this->moveToThread(m_pThread);

    //m_pMutex = new QMutex();

    connect(m_pThread, &QThread::started, this, &RobotThread::run);
    ros::init(m_Init_argc, m_pInit_argv, "gui_command");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    sim_velocity  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pose_listener = nh.subscribe(m_topic, 10, &RobotThread::poseCallback, this);

    m_pThread->start();
    return true;
}//set up the thread

void RobotThread::poseCallback(const nav_msgs::Odometry & msg)
{
    QMutex * pMutex = new QMutex();

    pMutex->lock();
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_aPos = msg.pose.pose.orientation.w;
    pMutex->unlock();

    delete pMutex;
    Q_EMIT newPose(m_xPos, m_yPos, m_aPos);
}//callback method to update the robot's position.

void RobotThread::run()
{
#if 1 // ducta ++
    ros::init(m_Init_argc, m_pInit_argv, "robotArmController"); // Name of the node specified in launch file
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    const double degree = M_PI/180;
    const double TWO_PI = 2 * M_PI;

    // robot state
    double tilt = 0, tinc = degree, height=0, hinc=0.005;
    static double angle = 0;
    double jointValues[12] = {0};
    int jointNo = 12;
    // message declarations
    geometry_msgs::TransformStamped world_trans;
    sensor_msgs::JointState joint_state;
    world_trans.header.frame_id = CWORLD_FRAME;
    world_trans.child_frame_id  = CBASE_LINK;
#endif  // ducta --

    ros::Rate loop_rate(30);
    QMutex * pMutex;
    while (ros::ok())
    {
        pMutex = new QMutex();
#if 1 // ducta ++
        {
            //update joint_state
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(jointNo);
            joint_state.position.resize(jointNo);
            joint_state.name[0] = CBASE_JOINT;
            joint_state.position[0] = jointValues[0];
            joint_state.name[1] = CJOINT_20;
            joint_state.position[1] = jointValues[1];
            joint_state.name[2] = CJOINT_2;
            joint_state.position[2] = jointValues[2];
            joint_state.name[3] = CJOINT_3;
            joint_state.position[3] = jointValues[3];

            joint_state.name[4] = CFINGER_1_PROX_JOINT;
            joint_state.position[4] = jointValues[4];

            joint_state.name[5] = CFINGER_1_MED_JOINT;
            joint_state.position[5] = jointValues[5];

            joint_state.name[6] = CFINGER_1_DIST_JOINT;
            joint_state.position[6] = jointValues[6];

            joint_state.name[7] = CFINGER_2_PROX_JOINT;
            joint_state.position[7] = jointValues[7];

            joint_state.name[8] = CFINGER_2_MED_JOINT;
            joint_state.position[8] = jointValues[8];

            joint_state.name[9] = CFINGER_2_DIST_JOINT;
            joint_state.position[9] = jointValues[9];

            joint_state.name[10] = CFINGER_3_MED_JOINT;
            joint_state.position[10] = jointValues[10];

            joint_state.name[11] = CFINGER_3_DIST_JOINT;
            joint_state.position[11] = jointValues[11];

            // WHOLE ARM MOVEMENT --------------------------------------------------------
            // update transform
            // (moving in a circle with radius=2)
            world_trans.header.stamp = ros::Time::now();
            world_trans.transform.translation.x = cos(angle)*2;
            world_trans.transform.translation.y = sin(angle)*2;
            world_trans.transform.translation.z = .7;
            world_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

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
            if((angle - m_angle) < m_angleDelta) {
                jointValues[0] += degree;
                angle += degree/4;
            }
            else {
                if(angle > TWO_PI) angle -= TWO_PI;
                // Stop rotating, though still keep publishing its state with current angle!
                m_angle = angle;
                m_angleDelta = 0;
            }
            pMutex->unlock();
        }
        // This will adjust as needed per iteration
        loop_rate.sleep();
		delete pMutex;
#else
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
    }//do ros things.
}

void RobotThread::SetSpeed(double speed, double angle)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    m_speed = speed;
    m_angle = angle;
	pMutex->unlock();
	delete pMutex;
}//set the speed of the robot.

void RobotThread::rotateJoint(int jointId, double angleDelta)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    m_angleDelta = angleDelta;
	pMutex->unlock();
	delete pMutex;
}

double RobotThread::getXSpeed(){ return m_speed; }
double RobotThread::getASpeed(){ return m_angle; }

double RobotThread::getXPos(){ return m_xPos; }
double RobotThread::getYPos(){ return m_yPos; }
double RobotThread::getAPos(){ return m_aPos; }

