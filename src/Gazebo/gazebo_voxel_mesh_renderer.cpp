/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Include Rand.hh first due to compilation error on osx (boost #5010)
// https://svn.boost.org/trac/boost/ticket/5010
#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/ArrowVisual.hh>
#include <gazebo/gui/ModelMaker.hh>
#include <gazebo/rendering/MovableText.hh>

#include "Gazebo/gazebo_selection_box.h"
#include "Gazebo/gazebo_voxel_mesh_renderer.h"

#include <ros/ros.h>
#include "ros_vox_cad/Voxelyze/Utils/Mesh.h"
#include "my_arm/RobotVoxelyzeAdapter.h"
#include "KsGlobal.h"
#include <std_msgs/Float32.h>
using namespace std;

#include <gazebo/physics/bullet/BulletMesh.hh>
#include <gazebo/physics/ode/ODEMesh.hh>
//#include <gazebo/physics/dart/DARTMesh.hh>

namespace gazebo
{
    namespace rendering
    {
    GazeboVoxelMeshRenderer::GazeboVoxelMeshRenderer():
                     _ros_node_handle(nullptr),
                     mRenderingScene(nullptr),
                     mRoot(nullptr),
                     mSceneMgr(nullptr),
                     mSelectionBox(nullptr),
                     mRootSceneNode(nullptr)
    {
        // start ros node
        if (!ros::isInitialized())
        {
          cout << "Voxel Mesh Renderer initialized!" << endl;
          int argc = 0;
          char** argv = NULL;
          ros::init(argc, argv,
                    "gazebo_voxel_mesh",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        }
    }

    GazeboVoxelMeshRenderer::~GazeboVoxelMeshRenderer()
    {
        this->connections.clear();
#ifdef USER_CAMERA
        if (this->userCam)
            this->userCam->EnableSaveFrame(false);
        this->userCam.reset();
#endif
        // ----------------------------------
        // Finalize the plugin
        this->_ros_node_handle->shutdown();
        delete this->_ros_node_handle;
    }

    /// \brief Called after the plugin has been constructed.
    void GazeboVoxelMeshRenderer::Load(int /*_argc*/, char ** /*_argv*/)
    {
        if(!this->_ros_node_handle) {
            this->_ros_node_handle = new ros::NodeHandle(std::string(CROS_MY_ARM_PACKAGE_NAME));
#if 0
            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so = ros::SubscribeOptions::create<my_arm::voxel_mesh>(
                                                              CVOXEL_MESH_TOPIC,
                                                              1000,
                                                              boost::bind(&GazeboVoxelMesh::voxelMeshCallback, this, _1),
                                                              ros::VoidPtr(), &this->_rosQueue);
            _voxel_mesh_listener = _ros_node_handle->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
              std::thread(std::bind(&GazeboVoxelMesh::queueThread, this));
#else
            _voxel_mesh_listener = _ros_node_handle->subscribe(CVOXEL_MESH_TOPIC, 100,
                                                               &GazeboVoxelMeshRenderer::voxelMeshCallback, this);
#endif
        }

        this->connections.push_back(
            event::Events::ConnectPreRender(
                boost::bind(&GazeboVoxelMeshRenderer::Update, this)));
    }

    // \brief Called once after Load
    void GazeboVoxelMeshRenderer::Init()
    {

    }

    void GazeboVoxelMeshRenderer::voxelMeshCallback(const my_arm::voxel_mesh& voxelMeshInfo)
    {
        //cout << "Voxel Mesh Message: " << voxelMeshInfo.vertices.size()<<endl;
        //_mMutex.lock();
        _voxel_mesh_info = voxelMeshInfo;
        //_mMutex.unlock();
    }

    /// \brief ROS helper function that processes messages
    void GazeboVoxelMeshRenderer::queueThread()
    {
        static const double timeout = 0.01;
        while (this->_ros_node_handle->ok())
        {
            this->_rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    void GazeboVoxelMeshRenderer::Update()
    {
        // TO QUICKLY HANDLE CALLBACKS
        //
        ros::spinOnce();
        // ----------------------------------------------------------------------------------------
        if(mRenderingScene == nullptr) {
            ROS_INFO("Initializing Rendering Scene...");
            mRenderingScene = gazebo::rendering::get_scene();
        }
        //ROS_INFO("WAIT UNTIL RENDERED!");
        // Wait until the scene is initialized.
        if (!mRenderingScene || !mRenderingScene->Initialized()) {
            ROS_WARN("Rendering Scene not yet initialized...!");
            return;
        }
        //ROS_INFO("SCENE RENDERED!");
        // -----------------------------------------------------------------------------------------
        //
        if(mSceneMgr == nullptr) {
            mSceneMgr = mRenderingScene->GetVisual(0)->GetSceneNode()->getCreator();
            mRootSceneNode=mSceneMgr->getRootSceneNode();
            Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

            cout << "Scene name:"  << rendering::RenderEngine::Instance()->GetScene()->Name() << endl;
            cout << "Scene count:" << rendering::RenderEngine::Instance()->SceneCount() << endl;

            // -------------------------------------------------------------------------------------
            // START ATTACHING OBJECTS INTO THE SCENE FROM HERE ------------------------------------
            //
            // Line Object
            //mRootSceneNode->createChildSceneNode("trajectory_visual_line")->attachObject(line_object);

            // Voxel Mesh
            this->createVoxelMeshSimple(true);
        }
        else {
            this->createVoxelMeshSimple(false);
        }

#ifdef USER_CAMERA
        // -----------------------------------------------------------------------------
        //
        if (!this->userCam)
        {
            // Get a pointer to the active user camera
            this->userCam = gui::get_active_camera();

            // Enable saving frames
            this->userCam->EnableSaveFrame(true);

            // Specify the path to save frames into
            this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");
        }

        // Look for a specific visual by name.
        if (mRenderingScene->GetVisual("ground_plane"))
            std::cout << "Has ground plane visual\n";
#endif
    }

    void GazeboVoxelMeshRenderer::createSelectionBox()
    {
        if(mSelectionBox == nullptr) {
            mSelectionBox = new SelectionBox("SelectionBox");
            mRootSceneNode->createChildSceneNode("selection_box")->attachObject(mSelectionBox);
        }
    }

//#define RENDER_TRIANGLE
    void GazeboVoxelMeshRenderer::createMovableText(const std::string &_name,
                                            const std::string &_text,
                                            const std::string &_fontName,
                                            float _charHeight,
                                            const common::Color &_color)
    {
        gazebo::rendering::MovableText* msg = new gazebo::rendering::MovableText();
        msg->Load(_name,_text, _fontName, _charHeight, _color);
        msg->SetTextAlignment(gazebo::rendering::MovableText::H_CENTER,
                              gazebo::rendering::MovableText::V_ABOVE); // Center horizontally and display above the node
        /* msg->setAdditionalHeight( 2.0f ); //msg->setAdditionalHeight( ei.getRadius() ) // apparently not needed from 1.7*/
        mRootSceneNode->createChildSceneNode("movable_text")->attachObject(msg);
    }

    void GazeboVoxelMeshRenderer::calculateVoxelMeshCollision()
    {
#if 0
        Ogre::MeshPtr voxelMesh = mManualObj->convertToMesh("voxel_mesh");
        gazebo::physics::ODEMesh mesh;
        gazebo::physics::ODECollisionPtr meshCollisionPtr;
        mesh.Init(voxelMesh, meshCollisionPtr, math::Vector3(1,1,1));

        gazebo::rendering::SelectionObj obj;
        obj.InsertMesh(voxelMesh, "voxel_mesh");
        obj.CreateDynamicLine();
#endif
    }

    // http://www.ogre3d.org/tikiwiki/ManualObject
    void GazeboVoxelMeshRenderer::createVoxelMeshSimple(bool isInitialShown)
    {
        _mMutex.lock();
        if( _voxel_mesh_info.vertices.size() > 0)
        {
            //cout << "VERTEX COUNT: " << _voxel_mesh_info.vertices.size() << endl;
            // -----------------------------------------------------------------------------------
            if(isInitialShown) {
                mManualObj = mSceneMgr->createManualObject("voxel_object");
                mManualObj->setDynamic(true); // To allow future beginUpdate() to update the mesh
#ifdef RENDER_TRIANGLE
                mManualObj->estimateIndexCount(_voxel_mesh_info.vertices.size());
                mManualObj->estimateVertexCount(_voxel_mesh_info.vertices.size());
                mManualObj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
#else
                mManualObj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
#endif
            }
            else {
                //mManualObj->clear();
                mManualObj->beginUpdate(0);
            }

#ifdef RENDER_TRIANGLE
            // define vertex position of index 0..3
            for(size_t i = 0; i < _voxel_mesh_info.vertices.size(); i++) {
                mManualObj->position(5*(_voxel_mesh_info.vertices[i].vertex.x + _voxel_mesh_info.vertices[i].offset.x),
                                     5*(_voxel_mesh_info.vertices[i].vertex.y + _voxel_mesh_info.vertices[i].offset.y),
                                     2 + 1*(_voxel_mesh_info.vertices[i].vertex.z + _voxel_mesh_info.vertices[i].offset.z));
                mManualObj->normal(_voxel_mesh_info.vertices[i].normal.x,
                                   _voxel_mesh_info.vertices[i].normal.y,
                                   _voxel_mesh_info.vertices[i].normal.z);
                mManualObj->colour(_voxel_mesh_info.vertices[i].color[0],
                                   _voxel_mesh_info.vertices[i].color[1],
                                   _voxel_mesh_info.vertices[i].color[2],
                                   _voxel_mesh_info.vertices[i].color[3]);
            }
            for(size_t i = 0; i < _voxel_mesh_info.vertices.size(); i++) {
                mManualObj->index((int)i);
            }
#else
            // define vertex position of index 0..3
            for(size_t i = 0; i < _voxel_mesh_info.edges.size(); i++) {
                // Ver 0 --
                voxel_mesh_msgs::vertex ver0 = _voxel_mesh_info.vertices[_voxel_mesh_info.edges[i].vi0];
                mManualObj->position(5*(ver0.vertex.x + ver0.offset.x),
                                     5*(ver0.vertex.y + ver0.offset.y),
                                     5*(ver0.vertex.z + ver0.offset.z));
                mManualObj->normal(ver0.normal.x,
                                   ver0.normal.y,
                                   ver0.normal.z);
                mManualObj->colour(ver0.color[0],
                                   ver0.color[1],
                                   ver0.color[2],
                                   ver0.color[3]);
                // Ver 1 --
                voxel_mesh_msgs::vertex ver1 = _voxel_mesh_info.vertices[_voxel_mesh_info.edges[i].vi1];
                mManualObj->position(5*(ver1.vertex.x + ver1.offset.x),
                                     5*(ver1.vertex.y + ver1.offset.y),
                                     5*(ver1.vertex.z + ver1.offset.z));
                mManualObj->normal(ver1.normal.x,
                                   ver1.normal.y,
                                   ver1.normal.z);
                mManualObj->colour(ver1.color[0],
                                   ver1.color[1],
                                   ver1.color[2],
                                   ver1.color[3]);
            }
#endif
            // define usage of vertices by refering to the indexes
            mManualObj->end();

            // ATTACH VOXEL MESH OBJECT TO THE SCENE NODE --
            //
            if(isInitialShown) {
                mRootSceneNode->createChildSceneNode("voxel_mesh")->attachObject(mManualObj);
            }
        } // if( _voxel_mesh_info.vertices.size() > 0)
        _mMutex.unlock();
    }

    // https://www.grahamedgecombe.com/blog/2011/08/05/custom-meshes-in-ogre3d
    void GazeboVoxelMeshRenderer::createVoxelMeshComplex()
    {
        if(!mVoxelMesh.isNull())
            return;
        ROS_INFO("START CREATING MESH...");
        // ----------------------------------------------------------------------------------
        /// Create the mesh via the MeshManager
        mVoxelMesh = Ogre::MeshManager::getSingleton().createManual("ColourCube", "General");

        /// Create one submesh
        Ogre::SubMesh* sub = mVoxelMesh->createSubMesh();

        const float sqrt13 = 0.577350269f; /* sqrt(1/3) */

        /// Define the vertices (8 vertices, each have 3 floats for position and 3 for normal)
        const size_t nVertices = 8;
        const size_t vbufCount = 3*2*nVertices;
        float vertices[vbufCount] = {
                -1.0,1.0,-1.0,        //0 position
                -sqrt13,sqrt13,-sqrt13,     //0 normal
                1.0,1.0,-1.0,         //1 position
                sqrt13,sqrt13,-sqrt13,      //1 normal
                1.0,-1.0,-1.0,        //2 position
                sqrt13,-sqrt13,-sqrt13,     //2 normal
                -1.0,-1.0,-1.0,       //3 position
                -sqrt13,-sqrt13,-sqrt13,    //3 normal
                -1.0,1.0,1.0,         //4 position
                -sqrt13,sqrt13,sqrt13,      //4 normal
                1.0,1.0,1.0,          //5 position
                sqrt13,sqrt13,sqrt13,       //5 normal
                1.0,-1.0,1.0,         //6 position
                sqrt13,-sqrt13,sqrt13,      //6 normal
                -1.0,-1.0,1.0,        //7 position
                -sqrt13,-sqrt13,sqrt13,     //7 normal
        };

        Ogre::RenderSystem* rs = Ogre::Root::getSingleton().getRenderSystem();
        Ogre::RGBA colours[nVertices];
        Ogre::RGBA *pColour = colours;
        // Use render system to convert colour value since colour packing varies
        rs->convertColourValue(Ogre::ColourValue(1.0,0.0,0.0), pColour++); //0 colour
        rs->convertColourValue(Ogre::ColourValue(1.0,1.0,0.0), pColour++); //1 colour
        rs->convertColourValue(Ogre::ColourValue(0.0,1.0,0.0), pColour++); //2 colour
        rs->convertColourValue(Ogre::ColourValue(0.0,0.0,0.0), pColour++); //3 colour
        rs->convertColourValue(Ogre::ColourValue(1.0,0.0,1.0), pColour++); //4 colour
        rs->convertColourValue(Ogre::ColourValue(1.0,1.0,1.0), pColour++); //5 colour
        rs->convertColourValue(Ogre::ColourValue(0.0,1.0,1.0), pColour++); //6 colour
        rs->convertColourValue(Ogre::ColourValue(0.0,0.0,1.0), pColour++); //7 colour

        /// Define 12 triangles (two triangles per cube face)
        /// The values in this table refer to vertices in the above table
        const size_t ibufCount = 36;
        unsigned short faces[ibufCount] = {
                0,2,3,
                0,1,2,
                1,6,2,
                1,5,6,
                4,6,5,
                4,7,6,
                0,7,4,
                0,3,7,
                0,5,1,
                0,4,5,
                2,7,3,
                2,6,7
        };

        /// Create vertex data structure for 8 vertices shared between submeshes
        mVoxelMesh->sharedVertexData = new Ogre::VertexData();
        mVoxelMesh->sharedVertexData->vertexCount = nVertices;

        /// Create declaration (memory format) of vertex data
        Ogre::VertexDeclaration* decl = mVoxelMesh->sharedVertexData->vertexDeclaration;
        size_t offset = 0;
        // 1st buffer
        decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
        decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
        /// Allocate vertex buffer of the requested number of vertices (vertexCount)
        /// and bytes per vertex (offset)
        Ogre::HardwareVertexBufferSharedPtr vbuf =
            Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            offset, mVoxelMesh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
        /// Upload the vertex data to the card
        vbuf->writeData(0, vbuf->getSizeInBytes(), vertices, true);

        /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
        Ogre::VertexBufferBinding* bind = mVoxelMesh->sharedVertexData->vertexBufferBinding;
        bind->setBinding(0, vbuf);

        // 2nd buffer
        offset = 0;
        decl->addElement(1, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
        /// Allocate vertex buffer of the requested number of vertices (vertexCount)
        /// and bytes per vertex (offset)
        vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            offset, mVoxelMesh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
        /// Upload the vertex data to the card
        vbuf->writeData(0, vbuf->getSizeInBytes(), colours, true);

        /// Set vertex buffer binding so buffer 1 is bound to our colour buffer
        bind->setBinding(1, vbuf);

        /// Allocate index buffer of the requested number of vertices (ibufCount)
        Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().
            createIndexBuffer(
            Ogre::HardwareIndexBuffer::IT_16BIT,
            ibufCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

        /// Upload the index data to the card
        ibuf->writeData(0, ibuf->getSizeInBytes(), faces, true);

        /// Set parameters of the submesh
        sub->useSharedVertices = true;
        sub->indexData->indexBuffer = ibuf;
        sub->indexData->indexCount = ibufCount;
        sub->indexData->indexStart = 0;

        /// Set bounding information (for culling)
        mVoxelMesh->_setBounds(Ogre::AxisAlignedBox(-100,-100,-100,100,100,100));
        mVoxelMesh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3*100*100));

        /// Notify -Mesh object that it has been loaded
        mVoxelMesh->load();

        // -----------------------------------------------------------------------------------
        /*
         material Test/ColourTest
         {
             technique
             {
                 pass
                 {
                     ambient vertexcolour
                 }
             }
         }
         */
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
              "Test/ColourTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);

        Ogre::Entity* thisEntity = mSceneMgr->createEntity("cc", "ColourCube");
        thisEntity->setMaterialName("Test/ColourTest");
        Ogre::SceneNode* thisSceneNode = mRootSceneNode->createChildSceneNode("voxel_mesh");
        thisSceneNode->setPosition(-35, 0, 0);
        thisSceneNode->attachObject(thisEntity);
    }

#ifdef USE_GAZEBO_RENDERING
    gazebo::rendering::CameraPtr GazeboVoxelMeshRenderer::createCamera(const gazebo::rendering::ScenePtr& scene)
    {
        gazebo::rendering::CameraPtr camera = scene->CreateCamera("camera");
        camera->SetWorldPosition(ignition::math::Vector3d(0.0, 0.0, 1.5));
        camera->SetWorldRotation(ignition::math::Quaterniond(0.0, 0.20, M_PI / 2));
        camera->SetImageWidth(640);
        camera->SetImageHeight(480);
        camera->SetAspectRatio(1.333);
        camera->SetHFOV(ignition::math::Angle(M_PI / 2));
        return camera;
    }

#if 0
    Ogre::MeshPtr GazeboVoxelMesh::createMesh()
    {
        gazebo::common::Mesh*  plane_mesh = new gazebo::common::Mesh;
        gazebo::common::SubMesh* submesh  = new gazebo::common::SubMesh;

        submesh->AddVertex(-1,-1,0);
        submesh->AddVertex(1, -1,0);
        submesh->AddVertex(-1,1,0);
        submesh->AddVertex(1, 1,0);

        submesh->AddIndex(0);
        submesh->AddIndex(1);
        submesh->AddIndex(2);

        submesh->AddIndex(1);
        submesh->AddIndex(3);
        submesh->AddIndex(2);

        plane_mesh->AddSubMesh(submesh);
        plane_mesh->SetName("myMesh");
        gazebo::common::MeshManager::Instance()->AddMesh(plane_mesh);

        Ogre::MeshPtr mesh = gazebo::common::MeshManager::Instance()->mes
        return plane_mesh;
    }
#endif

    void GazeboVoxelMeshRenderer::startSceneViewer()
    {
        // Connect SceneManager to Gazebo
        //gazebo::common::Console::SetQuiet(false);
        //gazebo::transport::init();
        //gazebo::transport::run();


        // ----------------------------------------------
        //
        //ScenePtr scene = createScene();
        //Ogre::SceneManager manager;
        //manager.AddScene(scene);
        VisualPtr root = mRenderingScene->WorldVisual();

        // MESH --
        //ignition::rendering::MeshPtr mesh = this->createMesh(scene);
        //root->AddGeometry(mesh);
        return;
    }

#elif defined USE_IGNITION_RENDERING
    ignition::rendering::ScenePtr GazeboVoxelMesh::createScene(const std::string &_engine)
    {
      ignition::rendering::RenderEngine *engine = ignition::rendering::get_engine(_engine);
      ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
      scene->SetBackgroundColor(0.25, 0.25, 0.25);
      return scene;
    }

    ignition::rendering::CameraPtr GazeboVoxelMesh::createCamera(const ignition::rendering::ScenePtr& scene)
    {
        ignition::rendering::CameraPtr camera = scene->CreateCamera("camera");
        camera->SetLocalPosition(0.0, 0.0, 1.5);
        camera->SetLocalRotation(0.0, 0.20, M_PI / 2);
        camera->SetImageWidth(640);
        camera->SetImageHeight(480);
        camera->SetAntiAliasing(2);
        camera->SetAspectRatio(1.333);
        camera->SetHFOV(M_PI / 2);
        return camera;
    }

    ignition::rendering::MeshPtr GazeboVoxelMesh::createMesh(const ignition::rendering::ScenePtr& scene)
    {
        gazebo::common::Mesh*  plane_mesh = new gazebo::common::Mesh;
        gazebo::common::SubMesh* submesh  = new gazebo::common::SubMesh;

        submesh->AddVertex(-1,-1,0);
        submesh->AddVertex(1, -1,0);
        submesh->AddVertex(-1,1,0);
        submesh->AddVertex(1, 1,0);

        submesh->AddIndex(0);
        submesh->AddIndex(1);
        submesh->AddIndex(2);

        submesh->AddIndex(1);
        submesh->AddIndex(3);
        submesh->AddIndex(2);

        plane_mesh->AddSubMesh(submesh);
        plane_mesh->SetName("myMesh");
        gazebo::common::MeshManager::Instance()->AddMesh(plane_mesh);

        // ------------------------------------------------------------------
        ignition::rendering::MeshDescriptor meshDesc("myMesh");
        ignition::rendering::MeshPtr mesh = scene->CreateMesh(meshDesc);

        ignition::rendering::MaterialPtr planeMat = scene->CreateMaterial();
        planeMat->SetAmbient(gazebo::common::Color::Black);
        planeMat->SetReflectivity(0.5);
        mesh->SetMaterial(planeMat);

        return mesh;
    }

    void GazeboVoxelMesh::startSceneViewer()
    {
        // Connect SceneManager to Gazebo
        //gazebo::common::Console::SetQuiet(false);
        //gazebo::transport::init();
        //gazebo::transport::run();

        ignition::rendering::SceneManager* manager = ignition::rendering::SceneManager::Instance();
        manager->Load();
        manager->Init();

        // ----------------------------------------------
        //
        ignition::rendering::ScenePtr scene = this->createScene(CGZ_RENDERING_ENGINE);
        manager->AddScene(scene);
        ignition::rendering::VisualPtr root = scene->GetRootVisual();

        // LIGHT --
        ignition::rendering::PointLightPtr light = scene->CreatePointLight();
        light->SetLocalPosition(0, 5, 5);
        root->AddChild(light);

        ignition::rendering::MaterialPtr planeMat = scene->CreateMaterial();
        planeMat->SetAmbient(gazebo::common::Color::Black);
        planeMat->SetReflectivity(0.5);

        ignition::rendering::VisualPtr plane = scene->CreateVisual();
        plane->AddGeometry(scene->CreatePlane());
        plane->SetLocalPosition(1, 0, -0.5);
        plane->SetLocalScale(5, 5, 1);
        plane->SetMaterial(planeMat);
        root->AddChild(plane);

        ignition::rendering::MaterialPtr sphereMat = scene->CreateMaterial();
        sphereMat->SetAmbient(0.5, 0, 0);
        sphereMat->SetDiffuse(0.8, 0 ,0);

        ignition::rendering::VisualPtr sphere = scene->CreateVisual();
        sphere->AddGeometry(scene->CreateSphere());
        sphere->SetLocalPosition(1, 0 , 0);
        sphere->SetMaterial(sphereMat);
        root->AddChild(sphere);

        // CAMERA --
        ignition::rendering::CameraPtr camera = this->createCamera(scene);
        root->AddChild(camera);

        // MESH --
        //ignition::rendering::MeshPtr mesh = this->createMesh(scene);
        //root->AddGeometry(mesh);
        return;
    }
#endif

    // Initial creation of the mesh decal
    /*
    material Crosshairs
    {
        technique
        {
            pass
            {
                scene_blend alpha_blend
                depth_write off
                texture_unit
                {
                    texture Crosshairs.png
                    scale 1 1
                }
            }
        }
    }
    */
    void GazeboVoxelMeshRenderer::createMeshDecal()
    {
        mMeshDecal = new Ogre::ManualObject("MeshDecal");
        mRootSceneNode->attachObject(mMeshDecal);

        int x_size = 4;  // number of polygons
        int z_size = 4;

        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
              "Crosshairs", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);

        mMeshDecal->begin("Crosshairs", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        for (int i=0; i<=x_size; i++)
        {
           for (int j=0; j<=z_size; j++)
           {
              mMeshDecal->position(Ogre::Vector3(i, 0, j));
              mMeshDecal->textureCoord((float)i / (float)x_size, (float)j / (float)z_size);
           }
        }

        for (int i=0; i<x_size; i++)
        {
            for (int j=0; j<z_size; j++)
            {
                mMeshDecal->quad(i * (x_size+1) + j,
                                 i * (x_size+1) + j + 1,
                                 (i + 1) * (x_size+1) + j + 1,
                                 (i + 1) * (x_size+1) + j);
            }
        }
        mMeshDecal->end();
    }

    // Update mesh decal when terrain selection happens
    void GazeboVoxelMeshRenderer::setMeshDecal(Ogre::Real x, Ogre::Real z, Ogre::Real rad)
    {
        Ogre::Real x1 = x - rad;
        Ogre::Real z1 = z - rad;

        int x_size = 4;  // number of polygons
        int z_size = 4;

        Ogre::Real x_step = rad * 2/ x_size;
        Ogre::Real z_step = rad * 2/ z_size;

        mMeshDecal->beginUpdate(0);
        // redefine vertices
        for (int i=0; i<=x_size; i++)
        {
            for (int j=0; j<=z_size; j++)
            {
                mMeshDecal->position(Ogre::Vector3(x1, getTerrainHeight(x1, z1) + 1, z1));
                mMeshDecal->textureCoord((float)i / (float)x_size, (float)j / (float)z_size);
                z1 += z_step;
            }
            x1 += x_step;
            z1 = z - rad;
        }
        // redefine quads
        for (int i=0; i<x_size; i++)
        {
            for (int j=0; j<z_size; j++)
            {
                mMeshDecal->quad(i * (x_size+1) + j,
                                 i * (x_size+1) + j + 1,
                                 (i + 1) * (x_size+1) + j + 1,
                                 (i + 1) * (x_size+1) + j);
            }
        }
        mMeshDecal->end();
    }

    Ogre::Real GazeboVoxelMeshRenderer::getTerrainHeight(Ogre::Real x, Ogre::Real z)
    {
        Ogre::Ray* verticalRay = new Ogre::Ray( Ogre::Vector3(x, 5000, z), Ogre::Vector3::NEGATIVE_UNIT_Y );
        mRaySceneQuery = mSceneMgr->createRayQuery(*verticalRay);

        // Execute query
        Ogre::RaySceneQueryResult &result = mRaySceneQuery->execute();
        Ogre::RaySceneQueryResult::iterator itr = result.begin( );

        if ( itr != result.end() && itr->worldFragment )
        {
            Ogre::Vector3 intersection = itr->worldFragment->singleIntersection;
            return intersection.y;
        }
        else
        {
           return 0;
        }
    }

    // The lib name is set in CMakeLists.txt, and built into devel/lib dir
    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(GazeboVoxelMeshRenderer)
    }
}
