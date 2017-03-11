#include "Gazebo/gazebo_marker_plugin.h"
#include "KsGlobal.h"

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    GazeboMarkerPlugin::GazeboMarkerPlugin():
        line(nullptr),
        mRoot(0),
        mSceneMgr(0),
        mSelectionBox(0)
    {

    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    GazeboMarkerPlugin::~GazeboMarkerPlugin()
    {
        if (mSelectionBox)
            delete mSelectionBox;

        //Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
        //windowClosed(mWindow);

        delete mRoot;
        // ----------------------------------
        // Finalize the visualizer
        this->rosnode_->shutdown();
        delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void GazeboMarkerPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
        this->visual_ = _parent;

        this->visual_namespace_ = _parent->GetName();
        if (!this->visual_namespace_.empty()) {
            this->visual_namespace_ += "/";
        }
        // start ros node
        if (!ros::isInitialized())
        {
          int argc = 0;
          char** argv = NULL;
          ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        }

        this->rosnode_ = new ros::NodeHandle(std::string(CROS_MY_ARM_PACKAGE_NAME) + "/" + this->visual_namespace_);
        this->force_sub_ = this->rosnode_->subscribe("joint_states", 1000, &GazeboMarkerPlugin::VisualizeForceOnLink, this);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectRender(
            boost::bind(&GazeboMarkerPlugin::UpdateChild, this));

      // ----------------------------------------------------------------------------
#if 0
        setup();
        mRoot->startRendering();
#endif
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void GazeboMarkerPlugin::UpdateChild()
    {
        ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void GazeboMarkerPlugin::VisualizeForceOnLink(const geometry_msgs::PointConstPtr &force_msg)
    {
        this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
#if     0
        //TODO: Get the current link position
        link_pose = CurrentLinkPose();
        //TODO: Get the current end position
        endpoint = CalculateEndpointOfForceVector(link_pose, force_msg);

        // Add two points to a connecting line strip from link_pose to endpoint
        this->line->AddPoint(
          ignition::math::Vector3d(
            link_pose.position.x,
            link_pose.position.y,
            link_pose.position.z
            )
          );
        this->line->AddPoint(ignition::math::Vector3d(endpoint.x, endpoint.y, endpoint.z));
#endif

      this->line->AddPoint(ignition::math::Vector3d(12, 12, 0));
      this->line->AddPoint(ignition::math::Vector3d(15, 15, 0));

      // set the Material of the line, in this case to purple
      this->line->setMaterial("Gazebo/Purple");
      this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
    }

    bool GazeboMarkerPlugin::setup()
    {
        mRoot = new Ogre::Root();
        mSceneMgr = mRoot->createSceneManager(Ogre::ST_EXTERIOR_CLOSE);
        Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
        createScene();
        return true;
    }

    void GazeboMarkerPlugin::createScene()
    {
        mSceneMgr->setAmbientLight(Ogre::ColourValue(0.7, 0.7, 0.7));

        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                Ogre::Entity* ent = mSceneMgr->createEntity("robot.mesh");

                Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
                node->setPosition(Ogre::Vector3(i * 15, 0, j * 15));
                node->attachObject(ent);
                node->setScale(0.2, 0.2, 0.2);
            }
        }

        mSelectionBox = new SelectionBox("SelectionBox");
        mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(mSelectionBox);
    }

    void GazeboMarkerPlugin::createColourCubeManual()
    {
        // ------------------------------------------------------------------------------------------------------------------------
        Ogre::ManualObject* manual = mSceneMgr->createManualObject("manual");
        manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

        // define vertex position of index 0..3
        manual->position(-1.0, -1.0, 0.0);
        manual->position( 1.0, -1.0, 0.0);
        manual->position( 1.0,  1.0, 0.0);
        manual->position(-1.0,  1.0, 0.0);

        // define usage of vertices by refering to the indexes
        manual->index(0);
        manual->index(1);
        manual->index(2);
        manual->index(3);
        manual->index(0);

        manual->end();
        mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(manual);
    }

    void GazeboMarkerPlugin::createColourCube()
    {
        /// Create the mesh via the MeshManager
        Ogre::MeshPtr msh = Ogre::MeshManager::getSingleton().createManual("ColourCube", "General");

        /// Create one submesh
        Ogre::SubMesh* sub = msh->createSubMesh();

        const float sqrt13 = 0.577350269f; /* sqrt(1/3) */

        /// Define the vertices (8 vertices, each have 3 floats for position and 3 for normal)
        const size_t nVertices = 8;
        const size_t vbufCount = 3*2*nVertices;
        float vertices[vbufCount] = {
                -100.0,100.0,-100.0,        //0 position
                -sqrt13,sqrt13,-sqrt13,     //0 normal
                100.0,100.0,-100.0,         //1 position
                sqrt13,sqrt13,-sqrt13,      //1 normal
                100.0,-100.0,-100.0,        //2 position
                sqrt13,-sqrt13,-sqrt13,     //2 normal
                -100.0,-100.0,-100.0,       //3 position
                -sqrt13,-sqrt13,-sqrt13,    //3 normal
                -100.0,100.0,100.0,         //4 position
                -sqrt13,sqrt13,sqrt13,      //4 normal
                100.0,100.0,100.0,          //5 position
                sqrt13,sqrt13,sqrt13,       //5 normal
                100.0,-100.0,100.0,         //6 position
                sqrt13,-sqrt13,sqrt13,      //6 normal
                -100.0,-100.0,100.0,        //7 position
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
        msh->sharedVertexData = new Ogre::VertexData();
        msh->sharedVertexData->vertexCount = nVertices;

        /// Create declaration (memory format) of vertex data
        Ogre::VertexDeclaration* decl = msh->sharedVertexData->vertexDeclaration;
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
            offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
        /// Upload the vertex data to the card
        vbuf->writeData(0, vbuf->getSizeInBytes(), vertices, true);

        /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
        Ogre::VertexBufferBinding* bind = msh->sharedVertexData->vertexBufferBinding;
        bind->setBinding(0, vbuf);

        // 2nd buffer
        offset = 0;
        decl->addElement(1, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
        /// Allocate vertex buffer of the requested number of vertices (vertexCount)
        /// and bytes per vertex (offset)
        vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
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
        msh->_setBounds(Ogre::AxisAlignedBox(-100,-100,-100,100,100,100));
        msh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3*100*100));

        /// Notify -Mesh object that it has been loaded
        msh->load();

        // ------------------------------------------------------------------------------------------------------------------------
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
              "Test/ColourTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);

        Ogre::Entity* thisEntity = mSceneMgr->createEntity("cc", "ColourCube");
        thisEntity->setMaterialName("Test/ColourTest");
        Ogre::SceneNode* thisSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        thisSceneNode->setPosition(-35, 0, 0);
        thisSceneNode->attachObject(thisEntity);
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(GazeboMarkerPlugin)
  }
}
