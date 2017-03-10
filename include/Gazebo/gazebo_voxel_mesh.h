#ifndef GAZEBO_VOXEL_MESH_H
#define GAZEBO_VOXEL_MESH_H

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
// if you want some positions of the model use this....
#include <gazebo_msgs/ModelStates.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

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

#include "gazebo_selection_box.h"

//#define USER_CAMERA

#define USE_GAZEBO_RENDERING
//#define USE_IGNITION_RENDERING

#include <gazebo/common/common.hh>
#include <gazebo/common/Mesh.hh>

#ifdef USE_GAZEBO_RENDERING
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Material.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Light.hh>

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

#ifdef USE_GAZEBO_RENDERING
//using namespace gazebo;
//using namespace gazebo::rendering;
#elif defined USE_IGNITION_RENDERING
#define CGZ_RENDERING_ENGINE ("ogre") // ("optix")
using namespace ignition;
using namespace ignition::rendering;
#endif

namespace gazebo
{
    namespace rendering
    {
        class GazeboVoxelMesh : public SystemPlugin
        {
            public:
                GazeboVoxelMesh();
                virtual ~GazeboVoxelMesh();

                /// \brief Called after the plugin has been constructed.
                void Load(int /*_argc*/, char ** /*_argv*/);

                // \brief Called once after Load
                void Init();

                //
                // Pointer to the model
#ifdef USE_GAZEBO_RENDERING
                //Ogre::MeshPtr createMesh();
                gazebo::rendering::CameraPtr createCamera(const gazebo::rendering::ScenePtr& scene);
#elif defined USE_IGNITION_RENDERING
                ignition::rendering::MeshPtr createMesh(const ignition::rendering::ScenePtr& scene);
                ignition::rendering::ScenePtr createScene(const std::string &_engine);
                ignition::rendering::CameraPtr createCamera(const ignition::rendering::ScenePtr& scene);
#endif
                void startSceneViewer();

            private:
                /// \brief Called every PreRender event. See the Load function.
                void Update();

                // Voxel Mesh --
                void createVoxelMeshComplex();
                void createVoxelMeshSimple();
                void updateVoxelMeshSimple();

                // Movable Text --
                void createMovableText(const std::string &_name,
                                       const std::string &_text,
                                       const std::string &_fontName = "Arial",
                                       float _charHeight = 1.0,
                                       const common::Color &_color = common::Color::White);

                // Create Selection Box --
                void createSelectionBox();

                // Mesh Decal --
                void createMeshDecal();
                void setMeshDecal(Ogre::Real x, Ogre::Real z, Ogre::Real rad);
                Ogre::Real getTerrainHeight(Ogre::Real x, Ogre::Real z);

#ifdef USER_CAMERA
            /// Pointer the user camera.
            private: rendering::UserCameraPtr userCam;
#endif
            /// All the event connections.
            private:
                std::vector<event::ConnectionPtr> connections;

                // -----------------------------------------------------------
                gazebo::rendering::ScenePtr mRenderingScene;
                //
                Ogre::Root* mRoot;
                Ogre::SceneNode* mRootSceneNode;
                Ogre::SceneManager* mSceneMgr;
                SelectionBox* mSelectionBox;
                std::list<Ogre::MovableObject*> mSelected;

                Ogre::ManualObject* mManualObj;
                Ogre::ManualObject* mMeshDecal;
                Ogre::RaySceneQuery* mRaySceneQuery;
                Ogre::MeshPtr mVoxelMesh;
        };
    }
}
#endif
