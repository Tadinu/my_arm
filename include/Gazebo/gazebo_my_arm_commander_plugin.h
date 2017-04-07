#ifndef GAZEBO_MY_ARM_COMMANDER_PLUGIN_H
#define GAZEBO_MY_ARM_COMMANDER_PLUGIN_H

// NATIVE --
#include <stdio.h>

// QT --
#include <QMutex>

// BOOST --
#include <boost/bind.hpp>

// GAZEBO --
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Mesh.hh>
#include <gazebo/transport/TransportIface.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>

#include <gazebo/rendering/DynamicLines.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>


// ROS --
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "my_arm/voxel_mesh_msg.h"

// OGRE --
#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreEntity.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>
#include <OgreSceneQuery.h>
#include <OgreManualObject.h>
//using namespace Ogre;

// Usage in URDF:
//   <gazebo>
//       <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
// 		<robotNamespace>/pioneer2dx</robotNamespace>
// 		<jointName>chassis_swivel_joint, swivel_wheel_joint, left_hub_joint, right_hub_joint</jointName>
// 		<updateRate>100.0</updateRate>
// 		<alwaysOn>true</alwaysOn>
//       </plugin>
//   </gazebo>

namespace gazebo {
class GazeboMyArmCommander : public ModelPlugin {
public:
    GazeboMyArmCommander();
    ~GazeboMyArmCommander();
    void Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
    void OnUpdate ( const common::UpdateInfo & _info );
    void leapCallback(const visualization_msgs::MarkerArray&);
    void voxelMeshCallback(const my_arm::voxel_mesh& voxelMeshInfo);
    void publishJointStates();
    void determineHandArrangmentOnLeapHands(int armId);
    void updateJointPosition(int jointId,
                             double position);
    void applyJointForce(int jointId,
                         double force);
    void calculateVoxelMeshCollision();

    void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void rosBumperCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
private:
    QMutex _mMutex;
    event::ConnectionPtr _updateConnection;
    physics::JointController* _joint_controller;
    physics::WorldPtr _world;
    physics::ModelPtr _model;
    std::vector<physics::JointPtr> _joints;
    std::vector<physics::LinkPtr>  _links;

    // ROS STUFF
    boost::shared_ptr<ros::NodeHandle> _rosnode;
    sensor_msgs::JointState _joint_state;
    ros::Publisher _joint_state_publisher;
    std::string _tf_prefix;
    std::string _robot_namespace;
    std::vector<std::string> _joint_names;

    // Voxel --
    ros::Subscriber _voxel_mesh_listener;
    my_arm::voxel_mesh _voxel_mesh_info;

    // Gazebo --
    ros::Subscriber _model_states_subscriber;
    std::vector<ros::Subscriber> _ros_bumper_subscribers;

    // Update Rate
    double _update_rate;
    double update_period_;
    common::Time _last_update_time;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (GazeboMyArmCommander)
}

#endif // GAZEBO_MY_ARM_COMMANDER_PLUGIN_H
