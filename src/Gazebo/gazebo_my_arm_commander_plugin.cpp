#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include "Gazebo/gazebo_my_arm_commander_plugin.h"
#include "RobotLeapAdapter.h"
#include "RobotVoxelyzeAdapter.h"
#include "KsGlobal.h"

#define CROBOT_NAME_SPACE_TAG ("robotNamespace")
#define CJOINT_NAME_TAG       ("jointName")
#define CUPDATE_RATE_TAG      ("updateRate")

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
        V_DELETE_POINTER(_joint_controller);
    }

    void GazeboMyArmCommander::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        cout << _sdf->GetNextElement() << " " << _sdf->GetDescription();
        // Store the pointer to the model
        this->_model = _parent;
        this->_world = _parent->GetWorld();
        this->_joint_controller = new physics::JointController(this->_model);

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
        _model_states_subscriber = _rosnode->subscribe("/gazebo/model_states", 100,
                                                       &GazeboMyArmCommander::modelStatecallback, this);

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

        for ( unsigned int i = 0; i< _joint_names.size(); i++ ) {
            _joints.push_back (this->_model->GetJoint(_joint_names[i]));
            ROS_INFO_NAMED("joint_state_publisher", "GazeboMyArmCommander is going to publish joint: %s", _joint_names[i].c_str() );
        }

        ROS_INFO_NAMED("joint_state_publisher", "Starting GazeboMyArmCommander Plugin (ns = %s)!, parent name: %s", this->_robot_namespace.c_str(), _model->GetName ().c_str() );

        _tf_prefix = tf::getPrefixParam ( *_rosnode );
        _joint_state_publisher = _rosnode->advertise<sensor_msgs::JointState>("joint_states", 1000);

        _last_update_time = this->_world->GetSimTime();

        // -----------------------------------------------------------------------------------------
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        //
        this->_updateConnection = event::Events::ConnectWorldUpdateBegin (
                                     boost::bind ( &GazeboMyArmCommander::OnUpdate, this, _1 ));
    }

    void GazeboMyArmCommander::updateJointPosition(int jointId,
                                                   double position)
    {
        if(_joints[jointId] != nullptr) {
            this->_joint_controller->SetJointPosition(this->_model->GetJoint(_joints[jointId]->GetName()), position);
        }
    }

    void GazeboMyArmCommander::determineHandArrangmentOnLeapHands()
    {
#ifdef ROBOT_LEAP_HANDS
        // Take the first hand data for convenience:
        std::vector<std::vector<double>> jointValues = VLEAP_INSTANCE()->getFingerJointValues(0);

        if(jointValues.size() > 0) {
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
    void GazeboMyArmCommander::modelStatecallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
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
        // ------------------------------------------------------------------------------
        // Apply a small linear velocity to the model.
        common::Time current_time = this->_world->GetSimTime();
        double seconds_since_last_update = ( current_time - _last_update_time ).Double();
        if ( seconds_since_last_update > update_period_ ) {
#if 1
            determineHandArrangmentOnLeapHands();
#else
            // Publish Joint States
            publishJointStates();
#endif
            _last_update_time+= common::Time ( update_period_ );
        }
    }

    void GazeboMyArmCommander::publishJointStates()
    {
        ros::Time current_time = ros::Time::now();

        _joint_state.header.stamp = current_time;
        _joint_state.name.resize ( _joints.size() );
        _joint_state.position.resize ( _joints.size() );

        for ( int i = 0; i < _joints.size(); i++ ) {
            //ROS_INFO("Joint: %s, %d", joint_names_[i].c_str(), this->parent_->GetJoint(joint_names_[i]).get());
            if(_joints[i] != nullptr) {
                math::Angle angle        = _joints[i]->GetAngle(0);
                _joint_state.name[i]     = _joints[i]->GetName();
                _joint_state.position[i] = angle.Radian() ;
                //ROS_INFO("JOINT: %d %s %f", i, _joint_state_.name[i].c_str(), _joint_state_.position[i]);
            }
            else {
                ROS_INFO("JOINT NULL: %d", i);
            }
        }
        _joint_state_publisher.publish(_joint_state);
    }


    // =========================================================================================================
    //
}
