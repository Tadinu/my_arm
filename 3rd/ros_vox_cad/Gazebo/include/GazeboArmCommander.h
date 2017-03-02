#ifndef GAZEBO_MY_ARM_COMMANDER_PLUGIN_H
#define GAZEBO_MY_ARM_COMMANDER_PLUGIN_H

#include <gazebo/common/Console.hh>
#include <gazebo/transport/TransportIface.hh>

#include <ignition/rendering.hh> // Extending gazebo built-in <rendering/rendering.hh>
#include <ignition/rendering/SceneManager.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Mesh.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Light.hh>

#define CGZ_RENDERING_ENGINE ("ogre") // ("optix")
using namespace ignition;
using namespace ignition::rendering;

namespace ir = ignition::rendering;

class GazeboMyArmCommander {
public:
    GazeboMyArmCommander();
    ~GazeboMyArmCommander();
    //
    // Pointer to the model
    void connectToGazebo();
    ir::MeshPtr createMesh(const ir::ScenePtr& scene);
    ir::ScenePtr createScene(const std::string &_engine);
    ir::CameraPtr createCamera(const ir::ScenePtr& scene);
    void addSceneContent(const ir::ScenePtr& scene);

private:
};

#endif // GAZEBO_MY_ARM_COMMANDER_H
