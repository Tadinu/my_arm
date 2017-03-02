#include <gazebo/common/common.hh>
#include <gazebo/common/Mesh.hh>
#include "GazeboArmCommander.h"

GazeboMyArmCommander::GazeboMyArmCommander()
{
    connectToGazebo();
}

GazeboMyArmCommander::~GazeboMyArmCommander()
{
     gazebo::transport::fini();
}

void GazeboMyArmCommander::connectToGazebo()
{
    gazebo::common::Console::SetQuiet(false);
    gazebo::transport::init();
    gazebo::transport::run();

    ir::SceneManager* manager = ir::SceneManager::Instance();
    manager->Load();
    manager->Init();
}

ir::ScenePtr GazeboMyArmCommander::createScene(const std::string &_engine)
{
    ir::RenderEngine *engine = get_engine(_engine);
    ir::ScenePtr scene = engine->CreateScene("scene");
    scene->SetBackgroundColor(0.25, 0.25, 0.25);
    return scene;
}

ir::CameraPtr GazeboMyArmCommander::createCamera(const ir::ScenePtr& scene)
{
    ir::CameraPtr camera = scene->CreateCamera("camera");
    camera->SetLocalPosition(-3.0, -2.0, 1.0);
    camera->SetLocalRotation(0.0, 0.35, -0.175);
    camera->SetImageWidth(1920);
    camera->SetImageHeight(1080);
    camera->SetAntiAliasing(2);
    camera->SetAspectRatio(1.333);
    camera->SetHFOV(M_PI / 2);

    return camera;
}

ir::MeshPtr GazeboMyArmCommander::createMesh(const ir::ScenePtr& scene)
{
#if 1
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
#endif
    // ------------------------------------------------------------------
    ir::MeshDescriptor meshDesc("myMesh");
    ir::MeshPtr mesh = scene->CreateMesh(meshDesc);

    ir::MaterialPtr planeMat = scene->CreateMaterial();
    planeMat->SetAmbient(0, 2, 0);
    planeMat->SetReflectivity(0.5);
    mesh->SetMaterial(planeMat);

    return mesh;
}

void GazeboMyArmCommander::addSceneContent(const ir::ScenePtr& scene)
{
    SceneManager::Instance()->AddScene(scene);
    ir::VisualPtr root = scene->GetRootVisual();

    // LIGHT --
    ir::PointLightPtr light = scene->CreatePointLight();
    light->SetLocalPosition(0, 5, 5);
    root->AddChild(light);
#if 0
    ir::MaterialPtr planeMat = scene->CreateMaterial();
    planeMat->SetAmbient(0, 0, 0);
    planeMat->SetReflectivity(0.5);

    ir::VisualPtr plane = scene->CreateVisual();
    plane->AddGeometry(scene->CreatePlane());
    plane->SetLocalPosition(1, 0, -0.5);
    plane->SetLocalScale(5, 5, 1);
    plane->SetMaterial(planeMat);
    root->AddChild(plane);
#endif

    ir::MaterialPtr sphereMat = scene->CreateMaterial();
    sphereMat->SetAmbient(0.5, 0, 0);
    sphereMat->SetDiffuse(0.8, 0 ,0);

    ir::VisualPtr sphere = scene->CreateVisual();
    sphere->AddGeometry(scene->CreateSphere());
    sphere->SetLocalPosition(1, 0 , 0);
    sphere->SetMaterial(sphereMat);
    root->AddChild(sphere);

    // MESH --
    ir::MeshPtr mesh = this->createMesh(scene);
    root->AddGeometry(mesh);
    return;
}
