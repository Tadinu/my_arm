#ifndef ___ROBOT_VOXELYZE_ADAPTER_H___
#define ___ROBOT_VOXELYZE_ADAPTER_H___

#include <QtCore>
#include <QMutex>
#include <ros/ros.h>
#include <ros/common.h>
#include <ros/cpp_common_decl.h>
//#include "KsGlobal.h"

// VOXCAD/VOXELYZE --
// SET IN PRO or CMakeLists.txt FILE
//#define VOX_CAD
//#define VOXELYZE_PURE

#ifdef VOX_CAD
#include "ros_vox_cad/VoxCad/QVX_Edit.h"
#include "ros_vox_cad/VoxCad/QVX_Interfaces.h"
#include "ros_vox_cad/QTUtils/QOpenGL.h"
#include "ros_vox_cad/VoxCad/VoxCad.h"
#include "ros_vox_cad/Voxelyze/Utils/Mesh.h"
#include "ros_vox_cad/Voxelyze/VX_MeshUtil.h"
#elif defined VOXELYZE_PURE
#include "Voxelyze/include/Voxelyze.h"
#include "Voxelyze/include/VX_MeshRender.h"
#include "Voxelyze/include/Vec3D.h"
#endif

//#include <rviz/robot/robot.h>

#define VVOXELYZE_ADAPTER() RobotVoxelyzeAdapter::getInstance()
#define CVOXEL_MESH_TOPIC  ("/voxel_mesh")
class RobotVoxelyzeAdapter : public QObject {
    Q_OBJECT

public:
    static RobotVoxelyzeAdapter* getInstance();
    ~RobotVoxelyzeAdapter();
    static void deleteInstance();
    static bool checkInstance();

    void initVoxelyze(ros::NodeHandle* nodeHandle = nullptr,
                      bool isShowMainWindow = true);
    void emitVoxelMeshUpdated();

#ifdef VOX_CAD
    bool loadVXA();
    const std::vector<CFacet>& getVoxelMeshFaces();
    const std::vector<CVertex>& getVoxelMeshVertices();
    const std::vector<CLine>& getVoxelMeshLines();
#elif defined VOXELYZE_PURE
    const std::vector<int>& getVoxelMeshQuads();
    const std::vector<float>& getVoxelMeshVertices();
    const std::vector<int>& getVoxelMeshLines();
    QVector3D getVoxelMeshSize();

    void makeCantileverBeam();
    void makeVoxelMesh();
    void drawVoxelMesh();
    void updateVoxelCollisionInfo(const QVector3D& pos, const QVector3D& force);
    void doVoxelyzeTimeStep();
#endif
signals:
    void voxelMeshUpdated();

public slots:
    // UPDATE THE VOXEL MESH DUE TO THE IMPACT FROM EXTERNAL FACTOR (External force, gravity, etc.)
    // Voxelyze's doTimeStep called here-in(), besides it is also called in client app internal loop!
    void updateVoxelMesh();

private:
    RobotVoxelyzeAdapter();
    static RobotVoxelyzeAdapter* _instance;
    //
    QTimer timer;
    QMutex* _pMutex;

    ros::NodeHandle* _node_handle;
    ros::Publisher _pub_voxel_mesh;

#ifdef VOX_CAD
    // VoxCad --
    CVX_MeshUtil* _voxelMesh;
    VoxCad* _voxCad;
#elif defined VOXELYZE_PURE
    QMutex* _voxelMutex;
    CVoxelyze* _voxelyze;
    CVX_MeshRender* _voxelMeshRender;
    CVX_Material* _voxelMaterial;
    int _voxelTimeStep;
    Vec3D<> _voxelCollisionPos;
    Vec3D<> _voxelCollisionForce;

    // !NOTE: Actually, the voxel is always updated due to environment factors like gravity or temperature,
    // not only from some external force. Therefore, a flag for that meaning may not make much sense!
    //bool _voxelMeshUpdated;
#endif
    //rviz::robot
    //rviz::Robot robot;
};

#endif // ___ROBOT_VOXELYZE_ADAPTER_H___

