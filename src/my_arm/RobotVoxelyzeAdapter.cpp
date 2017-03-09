#include <functional> // std::bind
#include <iostream>
#include <QThread>
#include <QVector3D>
#include "RobotVoxelyzeAdapter.h"
#include "ros_vox_cad/VoxCad/QVX_Interfaces.h"
#include "ros_vox_cad/Voxelyze/VX_MeshUtil.h"

//#define VOXELYZE_RVIZ_MARKER
#define UPDATE_VOXEL_MESH_USING_LOCAL_TIMER
#ifdef VOXELYZE_RVIZ_MARKER
#include "Rviz/VMarker.h"
#endif

RobotVoxelyzeAdapter* RobotVoxelyzeAdapter::_instance = nullptr;
RobotVoxelyzeAdapter* RobotVoxelyzeAdapter::getInstance()
{
    if(_instance == nullptr) {
        _instance = new RobotVoxelyzeAdapter();
    }

    return _instance;
}

RobotVoxelyzeAdapter::RobotVoxelyzeAdapter():
                      _pMutex(new QMutex(QMutex::Recursive)),
                      _voxCad(nullptr), _voxelMesh(nullptr)
{
    timer.setInterval(500);
#ifdef UPDATE_VOXEL_MESH_USING_LOCAL_TIMER
    QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(updateVoxelMesh()));
#endif
}

RobotVoxelyzeAdapter::~RobotVoxelyzeAdapter()
{
    _pMutex->tryLock(500);
    delete _voxCad; _voxCad = nullptr;

    _pMutex->unlock(); // infutile if tryLock() failed!
    delete _pMutex;
}

void RobotVoxelyzeAdapter::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

void RobotVoxelyzeAdapter::initVoxelyze(bool isShowMainWindow)
{
    // INIT VOXEL MESH
    _voxCad = new VoxCad();

    bool test = loadVXA();
    if(test) {
        //ROS_INFO("LOAD VXA SUCCESSFULLY : %d", _voxCad->MainSim.pEnv->pObj->GetNumVox());
        std::cout << "LOAD VXA SUCCESSFULLY : %d" << _voxCad->MainSim.pEnv->pObj->GetNumVox();
    }
    //
    QObject::connect(&_voxCad->MainSim, SIGNAL(ReqGLUpdate()), this, SLOT(updateVoxelMesh()));
#ifndef UPDATE_VOXEL_MESH_USING_LOCAL_TIMER
    QObject::connect(_voxCad->GLWindow, SIGNAL(DrawGL(bool)), this, SLOT(updateVoxelMesh()));
    //QObject::connect(&_voxCad->MainObj, SIGNAL(GetCurGLSelected(int*)), this, SLOT(GetCurGLSelected(int*)));
    //QObject::connect(&_voxCad->MainFEA, SIGNAL(GetCurGLSelected(int*)), this, SLOT(GetCurGLSelected(int*)));
#endif

    if(isShowMainWindow) {
        _voxCad->show();
    }
    _voxCad->EnterVMMode(VoxCad::VM_PHYSICS);

    // Publish 1st time, then wait for the next GLUpdate() from VoxCad Simulator
    this->updateVoxelMesh();
    //-------------------------------------------------------------------------------------------------
    timer.start();
}

void RobotVoxelyzeAdapter::updateVoxelMesh()
{
    // ------------------------------------------------------------------------------------------------
    if(_voxelMesh == nullptr && _voxCad->MainSim.pSimView != nullptr) {
        _voxelMesh = &_voxCad->MainSim.pSimView->VoxMesh;
    }
    if(_voxelMesh == nullptr) {
        return;
    }
    //_voxelMesh->printMeshVertices();

    static CVX_MeshUtil lastVoxelMesh;
    //if(_voxelMesh->DefMesh.Vertices.size() != lastVoxelMesh.DefMesh.Vertices.size() ||
    //   _voxelMesh->DefMesh.Lines.size()    != lastVoxelMesh.DefMesh.Lines.size()    ||
    //   _voxelMesh->DefMesh.Facets.size()   != lastVoxelMesh.DefMesh.Facets.size()
    //   )
    {
        //ROS_INFO("VOXEL MESH REALLY UPDATED!: %d %d %d", _voxelMesh.DefMesh.Facets.size(),
        //                                                 _voxelMesh.DefMesh.Lines.size(),
        //                                                 _voxelMesh.DefMesh.Vertices.size()
        //         );
#ifdef VOXELYZE_RVIZ_MARKER
        VMarker::setVoxelMeshInfo(_voxelMesh->DefMesh.Vertices,
                                  _voxelMesh->DefMesh.Lines,
                                  _voxelMesh->DefMesh.Facets);
#endif
        //lastVoxelMesh = *_voxelMesh;
#if 0
        _voxelMesh->ToStl("Voxel.stl", static_cast<CVX_Object*>(&_voxCad->MainObj), true); // !! THIS DOES NOT WORK!
#endif
        emitVoxelMeshUpdated();
    }
    return;
}

void RobotVoxelyzeAdapter::emitVoxelMeshUpdated()
{
    //ROS_INFO("RobotVoxelyzeAdapter::emitVoxelMeshUpdated()");
    emit voxelMeshUpdated();
}

#if 0
CVX_Voxel* RobotVoxelyzeAdapter::queryVoxelMesh()
{
    //if (ros::service::call("voxel_mesh", &BulletServer::addCompound))
    ros::ServiceClient client = nh_.serviceClient<bullet_server::AddCompound>("add_compound");
    bullet_server::AddCompound srv;
    srv.request.soft_body.push_back();

    if (client.call(srv))
    {

    }
    else
    {
      ROS_ERROR("Failed to call service add_compound");
      return 1;
    }
}
#endif

bool RobotVoxelyzeAdapter::loadVXA()
{
    QString fileName;
    return _voxCad->MainSim.OpenVXA(&fileName);
}

const std::vector<CFacet>& RobotVoxelyzeAdapter::getVoxelMeshFaces()
{
    //_voxelMesh.DefMesh.Lines.size(),
    //_voxelMesh.DefMesh.Vertices.size()
    if(_voxelMesh) {
        return _voxelMesh->DefMesh.Facets;
    }
    return std::vector<CFacet>();
}

const std::vector<CVertex>& RobotVoxelyzeAdapter::getVoxelMeshVertices()
{
    //_voxelMesh.DefMesh.Lines.size(),
    //_voxelMesh.DefMesh.Vertices.size()
    if(_voxelMesh) {
        return _voxelMesh->DefMesh.Vertices;
    }
    return std::vector<CVertex>();
}
