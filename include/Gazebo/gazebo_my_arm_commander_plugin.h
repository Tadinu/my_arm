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


//#define USE_GAZEBO_RENDERING
#define USE_IGNITION_RENDERING

#ifdef USE_GAZEBO_RENDERING
#include <rendering/rendering.hh>
#include <rendering/RenderTypes.hh>
#include <rendering/Scene.hh>
#include <rendering/Visual.hh>
#include <rendering/Material.hh>
#include <rendering/Camera.hh>
#include <rendering/Light.hh>

#include <common/common.hh>
#include <common/Mesh.hh>
#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreSceneManager.h>

#include <ignition/math2/ignition/math/Vector3.hh>
#include <ignition/math2/ignition/math/Quaternion.hh>

#elif defined USE_IGNITION_RENDERING
#include <ignition/rendering.hh> // Extending gazebo built-in <rendering/rendering.hh>
#include <ignition/rendering/SceneManager.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Mesh.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Light.hh>
#endif

//#include <gazebo/physics/bullet/BulletMesh.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Mesh.hh>


// Usage in URDF:
//   <gazebo>
//       <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
// 		<robotNamespace>/pioneer2dx</robotNamespace>
// 		<jointName>chassis_swivel_joint, swivel_wheel_joint, left_hub_joint, right_hub_joint</jointName>
// 		<updateRate>100.0</updateRate>
// 		<alwaysOn>true</alwaysOn>
//       </plugin>
//   </gazebo>

#ifdef USE_GAZEBO_RENDERING
using namespace gazebo;
using namespace gazebo::rendering;
#elif defined USE_IGNITION_RENDERING
#define CGZ_RENDERING_ENGINE ("ogre") // ("optix")
using namespace ignition;
using namespace ignition::rendering;
#endif


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
    //
    // Pointer to the model
#ifdef USE_GAZEBO_RENDERING
    Ogre::MeshPtr createMesh();
    ScenePtr createScene();
    CameraPtr createCamera(const ScenePtr& scene);
#elif defined USE_IGNITION_RENDERING
    ignition::rendering::MeshPtr createMesh(const ignition::rendering::ScenePtr& scene);
    ignition::rendering::ScenePtr createScene(const std::string &_engine);
    ignition::rendering::CameraPtr createCamera(const ignition::rendering::ScenePtr& scene);
#endif
    void startSceneViewer();

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
