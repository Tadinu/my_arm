#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include "Gazebo/gazebo_my_arm_commander_plugin.h"
#include "RobotLeapAdapter.h"
#include "RobotVoxelyzeAdapter.h"
#include "RobotMoveIt.h"
#include "KsGlobal.h"

// ROBOT TO RUN
#define CRUN_ROBOT (KsGlobal::VSHADOW_HAND_UR_ARM)
//#define CRUN_ROBOT (KsGlobal::VPISA_SOFT_HAND_ARM)

#define CROBOT_NAME_SPACE_TAG ("robotNamespace")
#define CJOINT_NAME_TAG       ("jointName")
#define CUPDATE_RATE_TAG      ("updateRate")

#define CJOINT_STATES_TOPIC  ("/joint_states")
#define CJOINT_CONTROL_TOPIC ("joint_control")

namespace gazebo
{
    GazeboMyArmCommander::GazeboMyArmCommander()
    {
        // Make sure the ROS node for Gazebo has already been initialized, by placing the
        // plugin tag inside .gazebo file!
        if (!ros::isInitialized())
        {
            //ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            //                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            int argc = 0;
            char** argv = NULL;
            ros::init(argc, argv,
                      "gazebo_my_arm_commander",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
            return;
        }

        ROS_INFO("GazeboArmCommander Model plugin - Hello World!");
    }

    GazeboMyArmCommander::~GazeboMyArmCommander()
    {
        _rosnode->shutdown();
    }

    void GazeboMyArmCommander::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        cout << _sdf->GetNextElement() << " " << _sdf->GetDescription();
        // Store the pointer to the model
        this->_model = _parent;
        this->_world = _parent->GetWorld();
        this->_joint_controller = _parent->GetJointController(); //new physics::JointController(this->_model);

        //ROS_INFO("Joint count: %d", parent_->GetJointCount());
        this->_robot_namespace = _model->GetName();
        if (!_sdf->HasElement(CROBOT_NAME_SPACE_TAG)) {
            ROS_INFO_NAMED("joint_state_publisher", "GazeboMyArmCommander Plugin missing \"%s\", defaults to \"%s\"",
                           CROBOT_NAME_SPACE_TAG, this->_robot_namespace.c_str());
        } else {
            this->_robot_namespace = _sdf->GetElement(CROBOT_NAME_SPACE_TAG)->Get<std::string>();
            if (this->_robot_namespace.empty()) {
                this->_robot_namespace = _model->GetName();
            }
        }

        ROS_INFO("Robot Namespace: %s", this->_robot_namespace.c_str());
        if (!_robot_namespace.empty()) {
            this->_robot_namespace += "/";
        }
        _rosnode = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(CROS_MY_ARM_PACKAGE_NAME));
        _model_states_subscriber = _rosnode->subscribe("/gazebo/model_states", 1000,
                                                       &GazeboMyArmCommander::modelStateCallback, this);
        // Contact state info
        /*
        /contacts/rh_palm

        /contacts/rh_ff/distal
        /contacts/rh_ff/knuckle
        /contacts/rh_ff/middle
        /contacts/rh_ff/proximal

        /contacts/rh_lf/distal
        /contacts/rh_lf/knuckle
        /contacts/rh_lf/metacarpal
        /contacts/rh_lf/middle
        /contacts/rh_lf/proximal

        /contacts/rh_mf/distal
        /contacts/rh_mf/knuckle
        /contacts/rh_mf/middle
        /contacts/rh_mf/proximal

        /contacts/rh_rf/distal
        /contacts/rh_rf/knuckle
        /contacts/rh_rf/middle
        /contacts/rh_rf/proximal

        /contacts/rh_th/base
        /contacts/rh_th/distal
        /contacts/rh_th/hub
        /contacts/rh_th/middle
        /contacts/rh_th/proximal
        */
        for(size_t i = 0; i < 23; i++) {
            std::string linkName = KsGlobal::CSHADOWHAND_ARM_LINKS[i];
            if (i == 0) { // PALM
                _ros_bumper_subscribers.push_back(_rosnode->subscribe(linkName, 1000,
                                                                      &GazeboMyArmCommander::rosBumperCallback, this));
            }
            else {
                std::string linkPrefix = linkName.substr(0, 5);
                std::string linkAffix = linkName.substr(5);
                std::string topicName = "/contacts/" + linkPrefix + "/" + linkAffix;
                _ros_bumper_subscribers.push_back(_rosnode->subscribe(topicName, 1000,
                                                                      &GazeboMyArmCommander::rosBumperCallback, this));
            }
        }

        /*
         * /gazebo/link_states
           /gazebo/model_states
           /gazebo/bumper_states
           /gazebo/parameter_descriptions
           /gazebo/parameter_updates
           /gazebo/ros_bumper
           /gazebo/set_link_state
           /gazebo/set_model_state
         */
#ifdef ROBOT_LEAP_HANDS
        // LEAP HANDS --
        VLEAP_INSTANCE()->initLeapMotion(_rosnode.get());
        ros::Subscriber _leap_listener = _rosnode->subscribe(CLEAP_HANDS_TOPIC, 1000,
                                                             &GazeboMyArmCommander::leapCallback, this);
#endif

#ifdef ROBOT_VOXELYZE
        _voxel_mesh_listener = _rosnode->subscribe(CVOXEL_MESH_TOPIC, 1000,
                                                   &GazeboMyArmCommander::voxelMeshCallback, this);
#endif
        // ------------------------------------------------------------------------------------------
        // JOIN/LINK INFO --
        //
        if (!_sdf->HasElement(CJOINT_NAME_TAG)) {
            ROS_ASSERT("GazeboMyArmCommander Plugin missing jointNames");
        } else {
            sdf::ElementPtr element = _sdf->GetElement(CJOINT_NAME_TAG);
            std::string joint_names = element->Get<std::string>();
            boost::erase_all(joint_names, " ");
            boost::split(_joint_names, joint_names, boost::is_any_of(","));
        }

        this->_update_rate = 100.0;
        if (!_sdf->HasElement(CUPDATE_RATE_TAG)) {
            ROS_WARN_NAMED("joint_state_publisher", "GazeboMyArmCommander Plugin (ns = %s) missing <updateRate>, defaults to %f",
                           this->_robot_namespace.c_str(), this->_update_rate);
        } else {
            this->_update_rate = _sdf->GetElement(CUPDATE_RATE_TAG)->Get<double>();
        }

        // Initialize update rate stuff
        if ( this->_update_rate > 0.0 ) {
            this->update_period_ = 1.0 / this->_update_rate;
        } else {
            this->update_period_ = 0.0;
        }
        _last_update_time = this->_world->GetSimTime();

        _links  = this->_model->GetLinks();
        for(size_t i = 0; i < _joint_names.size(); i++) {
            // Do this to make use of self-defined jointId in KsGlobal
            _joints.push_back(this->_model->GetJoint(_joint_names[i]));
            ROS_INFO_NAMED("joint_state_publisher", "GazeboMyArmCommander is going to publish joint: %s", _joint_names[i].c_str() );
        }

        ROS_INFO_NAMED("joint_state_publisher", "Starting GazeboMyArmCommander Plugin (ns = %s)!, parent name: %s",
                       this->_robot_namespace.c_str(), _model->GetName ().c_str());

        _tf_prefix = tf::getPrefixParam ( *_rosnode );

        // JOINT STATE SUBSCRIBER
        _joint_state_subscriber = _rosnode->subscribe("/gazebo/joint_state", 1000,
                                                      &GazeboMyArmCommander::jointStateCallback, this);

        // JOINT STATE PUBLISHER
        //boost::shared_ptr<ros::NodeHandle> node;
        _joint_state_publisher = _rosnode->advertise<sensor_msgs::JointState>(CJOINT_STATES_TOPIC, 1000);
#if 0
        // POINT CLOUD PUBLISHER
        _point_cloud_publisher = _rosnode->advertise<sensor_msgs::PointCloud2>("contact_points", 1000);
#endif
        //_reset_time_publisher =  _rosnode->advertise<std_msgs::Float64>("/reset_time", 1000);
        _last_update_time = this->_world->GetSimTime();

        // -----------------------------------------------------------------------------------------
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        //
        this->_updateConnection = event::Events::ConnectWorldUpdateBegin (
                                     boost::bind ( &GazeboMyArmCommander::OnUpdate, this, _1 ));

        // =========================================================================================
#ifdef ROBOT_MOVEIT
        VMOVEIT()->initMoveIt(_rosnode);
        VMOVEIT()->fetchRobotModelInfo();
#endif
    }

    void GazeboMyArmCommander::updateJointPosition(int jointId,
                                                   double position)
    {
        if(_joints[jointId] == nullptr) {
            return;
        }
        //
        _joint_controller->SetJointPosition(_joints[jointId], position);
        //std::cout << "Joint: "       << _joints[jointId]->GetName()      << std::endl
        //          << "- Damping:   " << _joints[jointId]->GetDamping(0)  << std::endl
        //          << "- Stiffness: " << _joints[jointId]->GetStiffness(0)<< std::endl
        //          << "- Force: "     << _joints[jointId]->GetForce(0)    << std::endl
        //          << "- Velocity: "  << _joints[jointId]->GetVelocity(0) << std::endl;
    }

    void GazeboMyArmCommander::setJointVelocityPID(int jointId,
                                                   const common::PID& pid) {
        if(_joints[jointId] == nullptr) {
            return;
        }
#if 0
        std::map<std::string, physics::JointPtr> joints = this->_joint_controller->GetJoints();
        std::cout << "Joints SIZE: " << joints.size() << std::endl;
        for (auto const& x : joints)
        {
            std::cout << x.first  // string (key)
                      //<< ':'
                      //<< x.second // string's value
                      << std::endl ;
        }
        std::map<std::string, physics::JointPtr>::iterator it;
        for ( it = joints.begin(); it != joints.end(); it++ )
        {
            std::cout << it->first  // string (key)
                      << ':'
                      << it->second   // string's value
                      << std::endl ;
        }
#endif
        // Apply the P-controller to the joint.
        //this->_model->GetJoint(_joints[jointId]->GetScopedName())->SetVelocity(0);
        //std::cout << "Joint PID: " << _joints[jointId]->GetScopedName() << std::endl;
        _joint_controller->SetVelocityPID(_joints[jointId]->GetScopedName(), pid);
    }

    void GazeboMyArmCommander::setJointVelocity(int jointId,
                                                const double& vel)
    {
        if(_joints[jointId] == nullptr) {
            return;
        }

        // Set the joint's target velocity.
        //std::cout << "Joint VEL: " << _joints[jointId]->GetScopedName() << std::endl;
        _joint_controller->SetVelocityTarget(_joints[jointId]->GetScopedName(), vel);
    }

    void GazeboMyArmCommander::applyJointForce(int jointId,
                                               const double& force)
    {
        if(_joints[jointId] == nullptr) {
            return;
        }
        // apply the force on the joint
        _joints[jointId]->SetForce(0, force);
#if 0
        // compute the steptime for the PID
        common::Time currTime = this->model->GetWorld()->GetSimTime();
        common::Time stepTime = currTime - this->prevUpdateTime;
        this->prevUpdateTime  = currTime;

        // set the current position of the joint, and the target position,
        // and the maximum effort limit
        double pos_curr = this->_joints[jointId]->GetAngle(0).Radian();
        double max_cmd = this->joint_max_effort;

        // calculate the error between the current position and the target one
        double pos_err = pos_curr - pos_target;

        // compute the effort via the PID, which you will apply on the joint
        double effort_cmd = this->jointPID.Update(pos_err, stepTime);

        // check if the effort is larger than the maximum permitted one
        effort_cmd = effort_cmd > max_cmd ? max_cmd :
                    (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);

        // apply the force on the joint
        this->_joints[jointId]->SetForce(0, effort_cmd);
#endif
    }

    void GazeboMyArmCommander::determineHandArrangmentOnLeapHands(int armId)
    {
#ifdef ROBOT_LEAP_HANDS
        // Take the first hand data for convenience:
        std::vector<std::vector<double>> jointValues = VLEAP_INSTANCE()->getFingerJointValues(0);

        if(jointValues.size() > 0) {
            switch(armId) {
            case KsGlobal::VSHADOW_HAND_ARM:

                //ROS_INFO("Fingers joints: %f %f %f", jointValues[1][0], jointValues[1][1], jointValues[1][2]);

                //updateJointPosition(KsGlobal::VSHADOW_HAND_ARM_BASE_JOINT, 0);
                updateJointPosition(KsGlobal::VSHADOW_HAND_WRJ2, 0);
                updateJointPosition(KsGlobal::VSHADOW_HAND_WRJ1, 0);

                updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ5, jointValues[0][0]);
                updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ4, qAbs(jointValues[0][1]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ3, qAbs(jointValues[0][1]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ2, qAbs(jointValues[0][2]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ1, qAbs(jointValues[0][2]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_TIP, qAbs(jointValues[0][2]));

                updateJointPosition(KsGlobal::VSHADOW_FINGER_1_J4, jointValues[1][0]);
                updateJointPosition(KsGlobal::VSHADOW_FINGER_1_J3, qAbs(jointValues[1][1]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_1_J2, qAbs(jointValues[1][2]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_1_J1, qAbs(jointValues[1][3]));

                updateJointPosition(KsGlobal::VSHADOW_FINGER_2_J4, jointValues[2][0]);
                updateJointPosition(KsGlobal::VSHADOW_FINGER_2_J3, qAbs(jointValues[2][1]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_2_J2, qAbs(jointValues[2][2]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_2_J1, qAbs(jointValues[2][3]));

                updateJointPosition(KsGlobal::VSHADOW_FINGER_3_J4, -jointValues[3][0]);
                updateJointPosition(KsGlobal::VSHADOW_FINGER_3_J3, qAbs(jointValues[3][1]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_3_J2, qAbs(jointValues[3][2]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_3_J1, qAbs(jointValues[3][3]));

                updateJointPosition(KsGlobal::VSHADOW_FINGER_4_LFJ5, qAbs(jointValues[4][0]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_4_LFJ4, -jointValues[4][0]);
                updateJointPosition(KsGlobal::VSHADOW_FINGER_4_J3, qAbs(jointValues[4][1]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_4_J2, qAbs(jointValues[4][2]));
                updateJointPosition(KsGlobal::VSHADOW_FINGER_4_J1, qAbs(jointValues[4][3]));
                break;

            case KsGlobal::VSHADOW_HAND_UR_ARM:
                //ROS_INFO("Fingers joints: %f %f %f", jointValues[1][0], jointValues[1][1], jointValues[1][2]);
                //applyJointForce(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_PAN_JOINT, 100);

                //updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_BASE_JOINT, 0);
                updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_PAN_JOINT, 0);
                updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_LIFT_JOINT, M_PI/2);

                int jointId = KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_LIFT_JOINT;
                double a = this->_joint_controller->GetPositions()[_joint_names[jointId]];
                //std::cout << "Joint: "       << _joints[jointId]->GetName()      << std::endl
                //          << "- Damping:   " << _joints[jointId]->GetDamping(0)  << std::endl
                //          << "- Stiffness: " << _joints[jointId]->GetStiffness(0)<< std::endl
                //          << "- Force: "     << _joints[jointId]->GetForce(0)    << std::endl
                //          << "- Velocity: "  << _joints[jointId]->GetVelocity(0) << std::endl;
                //
                //ROS_INFO("ABC: %f/n", a);
                updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_ELBOW_JOINT, -M_PI/2);

                updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_WRIST_1_JOINT, -M_PI/2);
                updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_WRIST_2_JOINT, -M_PI/2);
                updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_WRIST_3_JOINT, -M_PI/2);

                updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_WRJ2, -M_PI/2);
                updateJointPosition(KsGlobal::VSHADOW_HAND_UR_ARM_WRJ1, -M_PI/2);

                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_THUMB_THJ5, jointValues[0][0]);
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_THUMB_THJ4, qAbs(jointValues[0][1]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_THUMB_THJ3, qAbs(jointValues[0][1]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_THUMB_THJ2, qAbs(jointValues[0][2]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_THUMB_THJ1, qAbs(jointValues[0][2]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_THUMB_TIP, qAbs(jointValues[0][2]));

                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_1_J4, jointValues[1][0]);
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_1_J3, qAbs(jointValues[1][1]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_1_J2, qAbs(jointValues[1][2]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_1_J1, qAbs(jointValues[1][3]));

                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_2_J4, jointValues[2][0]);
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_2_J3, qAbs(jointValues[2][1]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_2_J2, qAbs(jointValues[2][2]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_2_J1, qAbs(jointValues[2][3]));

                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_3_J4, -jointValues[3][0]);
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_3_J3, qAbs(jointValues[3][1]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_3_J2, qAbs(jointValues[3][2]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_3_J1, qAbs(jointValues[3][3]));

                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_4_LFJ5, qAbs(jointValues[4][0]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_4_LFJ4, -jointValues[4][0]);
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_4_J3, qAbs(jointValues[4][1]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_4_J2, qAbs(jointValues[4][2]));
                updateJointPosition(KsGlobal::VSHADOW_UR_FINGER_4_J1, qAbs(jointValues[4][3]));
                break;
            }
        }
#endif
    }

    void GazeboMyArmCommander::leapCallback(const visualization_msgs::MarkerArray&)
    {
    }

    void GazeboMyArmCommander::voxelMeshCallback(const my_arm::voxel_mesh& voxelMeshInfo)
    {
        _mMutex.lock();
        //std::cout << "My Arm Voxel Mesh Message" << voxelMeshInfo.vertices.size() << std::endl;
        _voxel_mesh_info = voxelMeshInfo;
        _mMutex.unlock();
    }

#include <gazebo/physics/bullet/BulletMesh.hh>
#include <gazebo/physics/ode/ODEMesh.hh>
//#include <gazebo/physics/ModelState.hh>
//#include <gazebo/physics/dart/DARTMesh.hh>
     //const gazebo::physics::ModelState
    void GazeboMyArmCommander::modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
#if 0
        for(size_t i = 0; i < msg->name.size(); i++) {
            cout << "Gazebo Model State: "<< msg->name[i] <<
                    msg->pose[i].position.x <<
                    msg->pose[i].position.y <<
                    msg->pose[i].position.z << endl;
        }
#endif
    }

    void GazeboMyArmCommander::jointStateCallback(const sensor_msgs::JointState& jointstate)
    {
        _mMutex.lock();
        ROS_INFO("Updating joint state...");
        _joint_state = jointstate;
        _mMutex.unlock();
    }

    void GazeboMyArmCommander::rosBumperCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
    {
#if 1
        //int a;
        //a = sizeof(msg);
        //std::stringstream ss;
        //ss << a;
        //ROS_INFO("The size of recieved ContactState message is: %d", a);
        if(msg->states.size() == 0)
            return;
#endif
        //geometry_msgs::Vector3 force  = msg->states[0].total_wrench.force;
        //geometry_msgs::Vector3 torque = msg->states[0].total_wrench.torque;
        //std::cout<< msg->states[0].contact_positions.size() << " - Force : " << force.x <<std::endl;
        //std::cout<< std::string(msg->states[0].collision1_name) << std::endl;
        //std::cout<< std::string(msg->states[0].collision2_name) << std::endl;
        // info
        // collision1_name
        // collision2_name
        // wrenches
        // total_wrench
        // contact_positions
        // contact_normals
        // depths

        _mMutex.lock();
        for (size_t i = 0; i < msg->states.size(); ++i)
        {
            //if(msg->states[i].collision2_name.find("unit") == std::string::npos)
            //    continue;
            //
            //std::cout << "State size: " << msg->states.size() << std::endl;
            //std::cout << "Collision between[" << msg->states[i].collision1_name
            //          << "] and [" << msg->states[i].collision2_name << "]\n";

            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            //
            point_cloud->width  = msg->states[i].contact_positions.size();
            point_cloud->height = 1;
            for (size_t j = 0; j < msg->states[i].contact_positions.size(); ++j)
            {
                point_cloud->points.push_back(pcl::PointXYZ(msg->states[i].contact_positions[j].x,
                                                            msg->states[i].contact_positions[j].y,
                                                            msg->states[i].contact_positions[j].z));

                //std::cout << j << "  Position:"
                //          << msg->states[i].contact_positions[j].x << " "
                //          << msg->states[i].contact_positions[j].y << " "
                //          << msg->states[i].contact_positions[j].z << "\n";
                //std::cout << "   Normal:"
                //          << msg->states[i].contact_normals[j].x<< " "
                //          << msg->states[i].contact_normals[j].y<< " "
                //          << msg->states[i].contact_normals[j].z<< "\n";
                //std::cout << "   Depth:" << msg->states[i].depths[j] << "\n";
            }
            //std::cout << " ------------------------------------- " << std::endl;

            //Convert the point cloud to ROS message
            pcl::toROSMsg (*point_cloud, _contact_point_cloud);
        }
        _mMutex.unlock();
    }

    void GazeboMyArmCommander::calculateVoxelMeshCollision()
    {
#if 0
        /* (1) SINCE GAZEBO WORKS ON PRE-PROVIDED MODEL COLLISION INFORMATION IN WORLD/SDF FILE
         * -> IT IS INFEASIBLE TO LOAD A DUMMY MODEL IN WORLD FILE & ATTACH MESH TO IT!
         *
         * (2) -> ANOTHER SOLUTION: A WORLD PLUGIN
         * + GET MODEL STATE TO KNOW ABOUT ALL COLLISION INFO OF HAND,
         * TOGETHER WITH CURRENT HAND POSE.
         * + PUBLISH ALL THOSE INFO TO A TOPIC.
         * + IN MY_ARM_MAIN_PROGRAM, SUBSCRIBE TO THAT TOPIC AND PASS IT TO VOXCAD.
         * + VOXCAD CALCULATE THE COLLISION THEN DEFORM THE MESH -> PUBLISH TO VOXEL_MESH, FROM WHICH
         * GAZEBO_VOXEL_MESH_RENDERER COULD SUBSCRIBE THEN RENDER IT ON GAZEBO.
         */
        _mMutex.lock();
        common::Mesh voxelMesh;
        common::SubMesh voxelSubMesh;
        for(size_t i = 0; i < _voxel_mesh_info.vertices.size(); i++) {
            voxelSubMesh.AddVertex(_voxel_mesh_info.vertices[i].vertex.x + _voxel_mesh_info.vertices[i].offset.x,
                                   _voxel_mesh_info.vertices[i].vertex.y + _voxel_mesh_info.vertices[i].offset.y,
                                   _voxel_mesh_info.vertices[i].vertex.z + _voxel_mesh_info.vertices[i].offset.z);
            voxelSubMesh.AddNormal(_voxel_mesh_info.vertices[i].normal.x,
                                   _voxel_mesh_info.vertices[i].normal.y,
                                   _voxel_mesh_info.vertices[i].normal.z);
        }

        for(size_t i = 0; i < _voxel_mesh_info.facets.size(); i++) {
            voxelSubMesh.AddIndex(_voxel_mesh_info.facets[i].vi0);
            voxelSubMesh.AddIndex(_voxel_mesh_info.facets[i].vi1);
            voxelSubMesh.AddIndex(_voxel_mesh_info.facets[i].vi2);
        }

        voxelMesh.AddSubMesh(&voxelSubMesh);
        common::MeshManager::Instance()->AddMesh(&voxelMesh);

        //physics::LinkPtr voxellink;
        //physics::ModelPtr model = _world->GetModel(modelName);

        //gazebo::physics::ODEMesh mesh;
        //gazebo::physics::ODECollisionPtr meshCollisionPtr;
        //mesh.Init(voxelMesh, meshCollisionPtr, math::Vector3(1,1,1));
        //
        //gazebo::rendering::SelectionObj obj;
        //obj.InsertMesh(voxelMesh, "voxel_mesh");
        //obj.CreateDynamicLine();
        _mMutex.unlock();
#else
#endif
    }

    void GazeboMyArmCommander::OnUpdate (const common::UpdateInfo & _info)
    {
        // TO QUICKLY HANDLE CALLBACKS
        //
        ros::spinOnce();
        //
        //this->publishPointCloud();
        // ------------------------------------------------------------------------------
        // Apply a small linear velocity to the model.
        common::Time current_time = this->_world->GetSimTime();
        double seconds_since_last_update = ( current_time - _last_update_time ).Double();
        if ( seconds_since_last_update > update_period_ ) {

            determineHandArrangmentOnLeapHands(CRUN_ROBOT);
            //std::cout << "--------------------------------------------------"<< std::endl;
            //for(size_t i = 0; i < _links.size(); i++) {
            //    std::cout << "Link: "        << _links[i]->GetSensorName(0)   << std::endl
            //              << "World force:"  << _links[i]->GetWorldForce().z  << std::endl
            //              << "World torque:" << _links[i]->GetWorldTorque().z << std::endl;
            //}
#if 1
            // --------------------------------------------------------------------------
            // Setup a P-controller, with a gain of 0.1.
            this->setJointVelocityPID(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_PAN_JOINT,   common::PID(0.1, 0, 0));
            this->setJointVelocityPID(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_LIFT_JOINT,  common::PID(0.1, 0, 0));
            this->setJointVelocityPID(KsGlobal::VSHADOW_HAND_UR_ARM_ELBOW_JOINT,          common::PID(0.1, 0, 0));
            this->setJointVelocity(KsGlobal::VSHADOW_HAND_UR_ARM_WRIST_1_JOINT, 20);
#else
            // Publish Joint States to CJOINT_CONTROL_TOPIC
            //this->setJointVelocityPID(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_PAN_JOINT, common::PID(100, 1.0, 0));
            //this->setJointVelocity(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_PAN_JOINT, 20);
            //
            //this->setJointVelocityPID(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_LIFT_JOINT, common::PID(100, 1.0, 0));
            //this->setJointVelocity(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_LIFT_JOINT, 20);
            //
            //this->setJointVelocityPID(KsGlobal::VSHADOW_HAND_UR_ARM_ELBOW_JOINT, common::PID(100, 1.0, 0));
            //this->setJointVelocity(KsGlobal::VSHADOW_HAND_UR_ARM_ELBOW_JOINT, 20);
            //
            //this->applyJointForce(KsGlobal::VSHADOW_HAND_UR_ARM_SHOULDER_PAN_JOINT, 100);

            this->publishJointStates();
#endif
            _last_update_time+= common::Time ( update_period_ );
        }
    }

    void GazeboMyArmCommander::publishJointStates()
    {
        ros::Time current_time = ros::Time::now();

        _mMutex.lock();
        _joint_state.header.stamp = current_time;
        _joint_state.name.resize ( _joints.size() );
        _joint_state.position.resize ( _joints.size() );

        math::Angle angle;
        double      vel;
        for ( int i = 0; i < _joints.size(); i++ ) {
            //ROS_INFO("Joint: %s, %d", joint_names_[i].c_str(), this->parent_->GetJoint(joint_names_[i]).get());
            if(_joints[i] != nullptr) {
                angle        = _joints[i]->GetAngle(0);
                vel          = _joints[i]->GetVelocity(0);
                _joint_state.name[i]     = _joints[i]->GetName();
                //_joint_state.position[i] = angle.Radian() ;
                _joint_state.velocity[i] = vel;
                ROS_INFO("JOINT: %d %s %f", i, _joint_state.name[i].c_str(), _joint_state.position[i]);
            }
            else {
                ROS_INFO("JOINT NULL: %d", i);
            }
        }
        _joint_state_publisher.publish(_joint_state);
        _mMutex.unlock();
    }

    void GazeboMyArmCommander::publishPointCloud()
    {
        _mMutex.lock();
        _contact_point_cloud.header.frame_id = KsGlobal::baseLinkName(CRUN_ROBOT);
        _contact_point_cloud.header.stamp    = ros::Time(ros::Time::now().toNSec());
        _point_cloud_publisher.publish(_contact_point_cloud);
        _mMutex.unlock();
    }

    // =========================================================================================================
    //
}

/*
I understood it ,so I will describe here for anyone who have a trouble. In order to execute a example_voxelgrid.cpp, I had to input three command on three terminals. As an additional information, I use a kinect.

roslaunch openni_launch openni.launch
rosrun my_pcl_tutorial example input:=/camera/depth/points
rosrun rviz rviz

The Input on the second sentence means a topic that is shown with rostopic list. On Rviz, you have to change some parameters.

Fixed Frame: map -> camera_link
PointCloud2/Topic: /output

This output on above means 37 line in example_voxelgrid.cpp. The results of executing the preceding statement are shown below.
*/
