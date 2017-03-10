#ifndef GAZEBO_MY_ARM_COMMANDER_PLUGIN_H
#define GAZEBO_MY_ARM_COMMANDER_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
    void publishJointStates();
    void determineHandArrangmentOnLeapHands();
    void updateJointPosition(int jointId,
                             double position);

private:
    event::ConnectionPtr updateConnection;
    physics::JointController* joint_controller_;
    physics::WorldPtr world_;
    physics::ModelPtr parent_;
    std::vector<physics::JointPtr> joints_;

    // ROS STUFF
    boost::shared_ptr<ros::NodeHandle> rosnode_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;
    std::string tf_prefix_;
    std::string robot_namespace_;
    std::vector<std::string> joint_names_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (GazeboMyArmCommander)
}

#endif // GAZEBO_MY_ARM_COMMANDER_PLUGIN_H
