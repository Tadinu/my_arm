#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <gazebo/common/Console.hh>
#include <gazebo/transport/TransportIface.hh>
#include "Gazebo/gazebo_my_arm_commander_plugin.h"

#include "RobotLeapAdapter.h"
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
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        ROS_INFO("Hello World!");
    }

    GazeboMyArmCommander::~GazeboMyArmCommander()
    {
        rosnode_->shutdown();
        V_DELETE_POINTER(joint_controller_);
    }

    void GazeboMyArmCommander::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->parent_ = _parent;
        this->world_  = _parent->GetWorld();
        this->joint_controller_ = new physics::JointController(this->parent_);

        //ROS_INFO("Joint count: %d", parent_->GetJointCount());
        this->robot_namespace_ = parent_->GetName();
        if (!_sdf->HasElement(CROBOT_NAME_SPACE_TAG)) {
            ROS_INFO_NAMED("joint_state_publisher", "GazeboMyArmCommander Plugin missing \"%s\", defaults to \"%s\"",
                           CROBOT_NAME_SPACE_TAG, this->robot_namespace_.c_str());
        } else {
            this->robot_namespace_ = _sdf->GetElement(CROBOT_NAME_SPACE_TAG)->Get<std::string>();
            if (this->robot_namespace_.empty()) {
                this->robot_namespace_ = parent_->GetName();
            }
        }

        ROS_INFO("Robot Namespace: %s", this->robot_namespace_.c_str());
        if (!robot_namespace_.empty()) {
            this->robot_namespace_ += "/";
        }
        rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(this->robot_namespace_));

#ifdef ROBOT_LEAP_HANDS
        // LEAP HANDS --
        VLEAP_INSTANCE()->initLeapMotion(rosnode_.get());
        ros::Subscriber _leap_listener = rosnode_->subscribe(CLEAP_HANDS_TOPIC, 1, &GazeboMyArmCommander::leapCallback, this);
#endif

        if (!_sdf->HasElement(CJOINT_NAME_TAG)) {
            ROS_ASSERT("GazeboMyArmCommander Plugin missing jointNames");
        } else {
            sdf::ElementPtr element = _sdf->GetElement(CJOINT_NAME_TAG);
            std::string joint_names = element->Get<std::string>();
            boost::erase_all(joint_names, " ");
            boost::split(joint_names_, joint_names, boost::is_any_of(","));
        }

        this->update_rate_ = 100.0;
        if (!_sdf->HasElement(CUPDATE_RATE_TAG)) {
            ROS_WARN_NAMED("joint_state_publisher", "GazeboMyArmCommander Plugin (ns = %s) missing <updateRate>, defaults to %f",
                           this->robot_namespace_.c_str(), this->update_rate_);
        } else {
            this->update_rate_ = _sdf->GetElement(CUPDATE_RATE_TAG)->Get<double>();
        }

        // Initialize update rate stuff
        if ( this->update_rate_ > 0.0 ) {
            this->update_period_ = 1.0 / this->update_rate_;
        } else {
            this->update_period_ = 0.0;
        }
        last_update_time_ = this->world_->GetSimTime();

        for ( unsigned int i = 0; i< joint_names_.size(); i++ ) {
            joints_.push_back (this->parent_->GetJoint(joint_names_[i]));
            ROS_INFO_NAMED("joint_state_publisher", "GazeboMyArmCommander is going to publish joint: %s", joint_names_[i].c_str() );
        }

        ROS_INFO_NAMED("joint_state_publisher", "Starting GazeboMyArmCommander Plugin (ns = %s)!, parent name: %s", this->robot_namespace_.c_str(), parent_->GetName ().c_str() );

        tf_prefix_ = tf::getPrefixParam ( *rosnode_ );
        joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1000);

        last_update_time_ = this->world_->GetSimTime();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin (
                                     boost::bind ( &GazeboMyArmCommander::OnUpdate, this, _1 ));
    }

    void GazeboMyArmCommander::updateJointPosition(int jointId,
                                                   double position)
    {
        if(joints_[jointId] != nullptr) {
            this->joint_controller_->SetJointPosition(this->parent_->GetJoint(joints_[jointId]->GetName()), position);
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

    void GazeboMyArmCommander::OnUpdate (const common::UpdateInfo & _info)
    {
        // Apply a small linear velocity to the model.
        common::Time current_time = this->world_->GetSimTime();
        double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
        if ( seconds_since_last_update > update_period_ ) {
#if 1
            determineHandArrangmentOnLeapHands();
#else
            // Publish Joint States
            publishJointStates();
#endif
            last_update_time_+= common::Time ( update_period_ );
        }
    }

    void GazeboMyArmCommander::publishJointStates()
    {
        ros::Time current_time = ros::Time::now();

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize ( joints_.size() );
        joint_state_.position.resize ( joints_.size() );

        for ( int i = 0; i < joints_.size(); i++ ) {
            //ROS_INFO("Joint: %s, %d", joint_names_[i].c_str(), this->parent_->GetJoint(joint_names_[i]).get());
            if(joints_[i] != nullptr) {
                math::Angle angle        = joints_[i]->GetAngle(0);
                joint_state_.name[i]     = joints_[i]->GetName();
                joint_state_.position[i] = angle.Radian() ;
                //ROS_INFO("JOINT: %d %s %f", i, joint_state_.name[i].c_str(), joint_state_.position[i]);
            }
            else {
                ROS_INFO("JOINT NULL: %d", i);
            }
        }
        joint_state_publisher_.publish(joint_state_);
    }


    // =========================================================================================================
    //
#ifdef USE_GAZEBO_RENDERING
    ScenePtr GazeboMyArmCommander::createScene()
    {
        RenderEngine engine;
        ScenePtr scene = engine.CreateScene("scene", true);
        scene->SetBackgroundColor(common::Color::White);
        return scene;
    }

    CameraPtr GazeboMyArmCommander::createCamera(const ScenePtr& scene)
    {
        CameraPtr camera = scene->CreateCamera("camera");
        camera->SetWorldPosition(ignition::math::Vector3d(0.0, 0.0, 1.5));
        camera->SetWorldRotation(ignition::math::Quaterniond(0.0, 0.20, M_PI / 2));
        camera->SetImageWidth(640);
        camera->SetImageHeight(480);
        camera->SetAspectRatio(1.333);
        camera->SetHFOV(M_PI / 2);
        return camera;
    }

    Ogre::MeshPtr GazeboMyArmCommander::createMesh()
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

        //Ogre::MeshPtr mesh = sceneManager.mesh
        return plane_mesh;
    }

    void GazeboMyArmCommander::startSceneViewer()
    {
        // Connect SceneManager to Gazebo
        //gazebo::common::Console::SetQuiet(false);
        //gazebo::transport::init();
        //gazebo::transport::run();

        Ogre::SceneManager manager;

        // ----------------------------------------------
        //
        ScenePtr scene = createScene();
        //manager.AddScene(scene);
        VisualPtr root = scene->GetWorldVisual();

        // MESH --
        //ignition::rendering::MeshPtr mesh = this->createMesh(scene);
        //root->AddGeometry(mesh);
        return;
    }

#elif defined USE_IGNITION_RENDERING
    ignition::rendering::ScenePtr GazeboMyArmCommander::createScene(const std::string &_engine)
    {
      ignition::rendering::RenderEngine *engine = ignition::rendering::get_engine(_engine);
      ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
      scene->SetBackgroundColor(0.25, 0.25, 0.25);
      return scene;
    }

    ignition::rendering::CameraPtr GazeboMyArmCommander::createCamera(const ignition::rendering::ScenePtr& scene)
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

    ignition::rendering::MeshPtr GazeboMyArmCommander::createMesh(const ignition::rendering::ScenePtr& scene)
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

    void GazeboMyArmCommander::startSceneViewer()
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
        ScenePtr scene = createScene(CGZ_RENDERING_ENGINE);
        manager->AddScene(scene);
        VisualPtr root = scene->GetRootVisual();

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
}