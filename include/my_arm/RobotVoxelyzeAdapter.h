#ifndef ___ROBOT_VOXELYZE_ADAPTER_H___
#define ___ROBOT_VOXELYZE_ADAPTER_H___

#include <QtCore>
#include <QMutex>
#include <ros/ros.h>
#include <ros/common.h>
#include <ros/cpp_common_decl.h>
//#include "KsGlobal.h"
//#include "Voxelyze.h"

#include "ros_vox_cad/VoxCad/QVX_Edit.h"
#include "ros_vox_cad/VoxCad/QVX_Interfaces.h"
#include "ros_vox_cad/QTUtils/QOpenGL.h"

#include "ros_vox_cad/VoxCad/VoxCad.h"
//#include <rviz/robot/robot.h>

// VOXELYZE --
//
#define ROBOT_VOXELYZE
#ifdef ROBOT_VOXELYZE
#define VVOXELYZE_ADAPTER() RobotVoxelyzeAdapter::getInstance()
#define CVOXEL_MESH_TOPIC  ("voxel_mesh")
#endif

class RobotVoxelyzeAdapter : public QObject {
    Q_OBJECT

public:
    static RobotVoxelyzeAdapter* getInstance();
    ~RobotVoxelyzeAdapter();
    static void deleteInstance();

    void initVoxelyze(ros::NodeHandle* nodeHandle = nullptr,
                      bool isShowMainWindow = true);
    void emitVoxelMeshUpdated();

    bool loadVXA();
    const std::vector<CFacet>& getVoxelMeshFaces();
    const std::vector<CVertex>& getVoxelMeshVertices();
    const std::vector<CLine>& getVoxelMeshLines();
signals:
    void voxelMeshUpdated();

public slots:
    void updateVoxelMesh();

private:
    RobotVoxelyzeAdapter();
    static RobotVoxelyzeAdapter* _instance;
    //
    QTimer timer;
    QMutex* _pMutex;

    ros::NodeHandle* _node_handle;
    ros::Publisher _pub_voxel_mesh;

    // VoxCad --
    CVX_MeshUtil* _voxelMesh;
    VoxCad* _voxCad;
    //rviz::robot
    //rviz::Robot robot;
};

#endif // ___ROBOT_VOXELYZE_ADAPTER_H___

