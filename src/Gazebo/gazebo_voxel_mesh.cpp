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
#include "Gazebo/gazebo_voxel_mesh.h"

#include <ros/ros.h>
#include "my_arm/RobotVoxelyzeAdapter.h"
#include "ros_vox_cad/Voxelyze/Utils/Mesh.h"

using namespace std;

namespace gazebo
{
    namespace rendering
    {
    GazeboVoxelMesh::GazeboVoxelMesh():
                     mRenderingScene(nullptr),
                     mRoot(nullptr),
                     mSceneMgr(nullptr),
                     mSelectionBox(nullptr),
                     mRootSceneNode(nullptr)
    {
    }

    GazeboVoxelMesh::~GazeboVoxelMesh()
    {
        this->connections.clear();
#ifdef USER_CAMERA
        if (this->userCam)
            this->userCam->EnableSaveFrame(false);
        this->userCam.reset();
#endif
    }

    /// \brief Called after the plugin has been constructed.
    void GazeboVoxelMesh::Load(int /*_argc*/, char ** /*_argv*/)
    {
        this->connections.push_back(
            event::Events::ConnectPreRender(
                boost::bind(&GazeboVoxelMesh::Update, this)));

        VVOXELYZE_ADAPTER()->initVoxelyze(false);
    }

    // \brief Called once after Load
    void GazeboVoxelMesh::Init()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    void GazeboVoxelMesh::Update()
    {
        if(mRenderingScene == nullptr) {
            ROS_INFO("Initializing Rendering Scene...");
            mRenderingScene = gazebo::rendering::get_scene();
        }
        ROS_INFO("WAIT UNTIL RENDERED!");
        // Wait until the scene is initialized.
        if (!mRenderingScene || !mRenderingScene->Initialized()) {
            ROS_WARN("Rendering Scene not yet initialized...!");
            return;
        }
        ROS_INFO("SCENE RENDERED!");
        // -----------------------------------------------------------------------------------------
        //
        if(mSceneMgr == nullptr) {
            mSceneMgr = mRenderingScene->GetVisual(0)->GetSceneNode()->getCreator();
            mRootSceneNode=mSceneMgr->getRootSceneNode();
            Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

            cout << rendering::RenderEngine::Instance()->GetScene()->Name() << endl;
            cout << rendering::RenderEngine::Instance()->SceneCount() << endl;

            // -------------------------------------------------------------------------------------
            // START ATTACHING OBJECTS INTO THE SCENE FROM HERE ------------------------------------
            //
            // Line Object
            //mRootSceneNode->createChildSceneNode("trajectory_visual_line")->attachObject(line_object);

            // Voxel Mesh
            this->createVoxelMeshSimple();
        }
        else {
            //this->updateVoxelMeshSimple();
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

    void GazeboVoxelMesh::createSelectionBox()
    {
        if(mSelectionBox == nullptr) {
            mSelectionBox = new SelectionBox("SelectionBox");
            mRootSceneNode->createChildSceneNode("selection_box")->attachObject(mSelectionBox);
        }
    }

    void GazeboVoxelMesh::createMovableText(const std::string &_name,
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

    // http://www.ogre3d.org/tikiwiki/ManualObject
    void GazeboVoxelMesh::createVoxelMeshSimple()
    {
        // -----------------------------------------------------------------------------------
        mManualObj = mSceneMgr->createManualObject("voxel");
        mManualObj->setDynamic(true); // To allow future beginUpdate() to update the mesh
        //mManualObj->estimateIndexCount(...);
        //mManualObj->estimateVertexCount(...);
        mManualObj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

        // define vertex position of index 0..3
        mManualObj->position(-1.0, -1.0, 0.0);
        mManualObj->position( 1.0, -1.0, 0.0);
        mManualObj->position( 1.0,  1.0, 0.0);
        mManualObj->position(-1.0,  1.0, 0.0);

        // define usage of vertices by refering to the indexes
        mManualObj->index(0);
        mManualObj->index(1);
        mManualObj->index(2);
        mManualObj->index(3);
        mManualObj->index(0);

        mManualObj->end();
        mRootSceneNode->createChildSceneNode("voxel_mesh")->attachObject(mManualObj);
    }

    void GazeboVoxelMesh::updateVoxelMeshSimple()
    {
        if(mManualObj) {
            std::vector<CFacet> facets    = VVOXELYZE_ADAPTER()->getVoxelMeshFaces();
            std::vector<CVertex> vertices = VVOXELYZE_ADAPTER()->getVoxelMeshVertices();
            for (int i = 0; i < (int)facets.size(); i++) {
                //glNormal3d(Facets[i].n.x, Facets[i].n.y, Facets[i].n.z);
                for (int j = 0; j < 3; j++) {
                    CVertex& CurVert = vertices[facets[i].vi[j]]; //just a local reference for readability
                    //glColor3d(CurVert.VColor.r, CurVert.VColor.g, CurVert.VColor.b);

                    // Point 1
                    mManualObj->position(CurVert.v.x + CurVert.DrawOffset.x,
                                         CurVert.v.y + CurVert.DrawOffset.y,
                                         CurVert.v.z + CurVert.DrawOffset.z
                                         );
                }
            }
            //
            mManualObj->beginUpdate(0);
            mManualObj->end();
        }
    }

    void GazeboVoxelMesh::createVoxelMeshComplex()
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
    gazebo::rendering::CameraPtr GazeboVoxelMesh::createCamera(const gazebo::rendering::ScenePtr& scene)
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

    void GazeboVoxelMesh::startSceneViewer()
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
    void GazeboVoxelMesh::createMeshDecal()
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
    void GazeboVoxelMesh::setMeshDecal(Ogre::Real x, Ogre::Real z, Ogre::Real rad)
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

    Ogre::Real GazeboVoxelMesh::getTerrainHeight(Ogre::Real x, Ogre::Real z)
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

    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(GazeboVoxelMesh)
    }
}
