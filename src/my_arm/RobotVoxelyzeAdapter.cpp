#include <functional> // std::bind
#include <iostream>
#include <QThread>
#include <QVector3D>
#include "RobotVoxelyzeAdapter.h"
#include "ros_vox_cad/VoxCad/QVX_Interfaces.h"
#include "ros_vox_cad/Voxelyze/VX_MeshUtil.h"
#include "ros_vox_cad/Voxelyze/Utils/Mesh.h"
#include "my_arm/voxel_mesh_msg.h"

#include "RobotLeapAdapter.h"

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

void RobotVoxelyzeAdapter::initVoxelyze(ros::NodeHandle* nodeHandle, bool isShowMainWindow)
{
    _node_handle = nodeHandle;
    // ADVERTISE VOXEL MESH TOPIC
    //
    if(_node_handle) {
        _pub_voxel_mesh = _node_handle->advertise<my_arm::voxel_mesh>(CVOXEL_MESH_TOPIC, 1000);
    }
    // -------------------------------------------------------------------------------------------------------
    // INIT VOXEL MESH
    //
    _voxCad = new VoxCad();
    _voxCad->MainSim.pSimView->loadFingerVoxels();

    ROS_INFO("VOX_CAD START");
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

// void CVXS_SimGLView::Draw()
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

#ifdef ROBOT_LEAP_HANDS// FOR NOW, ONLY SHOW FINGER TIPS IF LEAP MOTION IS AVAILABLE!
    // Take the first hand data for convenience:
    if(RobotLeapAdapter::checkInstance()) {
        std::vector<Vec3D<>> voxelPoses;
        std::vector<QVector3D> fingerTipsPoses = VLEAP_INSTANCE()->getFingerTipsPoses(0);

        static std::vector<QVector3D> latestFingerTipsPoses;
        if(latestFingerTipsPoses.size() == 0) {
            latestFingerTipsPoses.resize(fingerTipsPoses.size());
        }
        //
        bool isNewTipPoses = false;
        for(size_t i = 0; i < fingerTipsPoses.size(); i++) {
            if((latestFingerTipsPoses[i] - fingerTipsPoses[i]).length() >= 2) {
                isNewTipPoses = true;
                break;
            }
        }
        //
        if(isNewTipPoses) {
            for(size_t i = 0; i < fingerTipsPoses.size(); i++) {
                voxelPoses.push_back(Vec3D<>(std::abs(fingerTipsPoses[i].x()/10),
                                             std::abs(fingerTipsPoses[i].y()/10),
                                             std::abs(fingerTipsPoses[i].z()/10)));
                //std::cout << "D " << fingerTipsPoses[i].x()
                //                  << fingerTipsPoses[i].y()
                //                  << fingerTipsPoses[i].z() << std::endl;
            }

            // Detect interfacing voxels
            _voxCad->MainSim.detectInterfacingVoxels(voxelPoses);

            // Draw
            _voxCad->MainSim.pSimView->setFingerVoxelPos(voxelPoses);
        }
    }
#endif
    //ROS_INFO("UPDATE VOXEL--");
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
        //
        // ------------------------------------------------------------------------------------------
        // PUBLISH VOXEL MESH MESSAGE
        //
        my_arm::voxel_mesh voxel_mesh_msg;

        // Facets
        voxel_mesh_msgs::facet facet;
        for(size_t i = 0; i < _voxelMesh->DefMesh.Facets.size(); i++) {
            facet.normal.x = _voxelMesh->DefMesh.Facets[i].n.x;
            facet.normal.y = _voxelMesh->DefMesh.Facets[i].n.y;
            facet.normal.z = _voxelMesh->DefMesh.Facets[i].n.z;

            facet.vi0      = _voxelMesh->DefMesh.Facets[i].vi[0];
            facet.vi1      = _voxelMesh->DefMesh.Facets[i].vi[1];
            facet.vi2      = _voxelMesh->DefMesh.Facets[i].vi[2];

            facet.color[0] = _voxelMesh->DefMesh.Facets[i].FColor.r;
            facet.color[1] = _voxelMesh->DefMesh.Facets[i].FColor.g;
            facet.color[2] = _voxelMesh->DefMesh.Facets[i].FColor.b;
            facet.color[3] = _voxelMesh->DefMesh.Facets[i].FColor.a;
            //
            voxel_mesh_msg.facets.push_back(facet);
        }

        // Vertices
        voxel_mesh_msgs::vertex vertex;
        for(size_t i = 0; i < _voxelMesh->DefMesh.Vertices.size(); i++) {
            vertex.normal.x = _voxelMesh->DefMesh.Vertices[i].n.x;
            vertex.normal.y = _voxelMesh->DefMesh.Vertices[i].n.y;
            vertex.normal.z = _voxelMesh->DefMesh.Vertices[i].n.z;

            vertex.vertex.x = _voxelMesh->DefMesh.Vertices[i].v.x;
            vertex.vertex.y = _voxelMesh->DefMesh.Vertices[i].v.y;
            vertex.vertex.z = _voxelMesh->DefMesh.Vertices[i].v.z;

            vertex.offset.x = _voxelMesh->DefMesh.Vertices[i].DrawOffset.x;
            vertex.offset.y = _voxelMesh->DefMesh.Vertices[i].DrawOffset.y;
            vertex.offset.z = _voxelMesh->DefMesh.Vertices[i].DrawOffset.z;

            vertex.color[0] = _voxelMesh->DefMesh.Vertices[i].VColor.r;
            vertex.color[1] = _voxelMesh->DefMesh.Vertices[i].VColor.g;
            vertex.color[2] = _voxelMesh->DefMesh.Vertices[i].VColor.b;
            vertex.color[3] = _voxelMesh->DefMesh.Vertices[i].VColor.a;
            //
            voxel_mesh_msg.vertices.push_back(vertex);
        }

        // Edges
        voxel_mesh_msgs::edge line;
        for(size_t i = 0; i < _voxelMesh->DefMesh.Lines.size(); i++) {
            line.vi0 = _voxelMesh->DefMesh.Lines[i].vi[0];
            line.vi1 = _voxelMesh->DefMesh.Lines[i].vi[1];
            voxel_mesh_msg.edges.push_back(line);
        }

        //ROS_INFO("Publish Voxel Message %d", voxel_mesh_msg.vertices.size());
        _pub_voxel_mesh.publish(voxel_mesh_msg);
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
