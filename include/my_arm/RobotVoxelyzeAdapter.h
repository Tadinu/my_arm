#ifndef ___ROBOT_VOXELYZE_ADAPTER_H___
#define ___ROBOT_VOXELYZE_ADAPTER_H___

#include <QtCore>
#include <QMutex>
#include "KsGlobal.h"
//#include "Voxelyze.h"

#include "ros_vox_cad/VoxCad/QVX_Edit.h"
#include "ros_vox_cad/VoxCad/QVX_Interfaces.h"
#include "ros_vox_cad/QTUtils/QOpenGL.h"

#include "ros_vox_cad/VoxCad/VoxCad.h"
//#include <rviz/robot/robot.h>

class RobotVoxelyzeAdapter : public QObject {
    Q_OBJECT

public:
    static RobotVoxelyzeAdapter* getInstance();
    ~RobotVoxelyzeAdapter();
    static void deleteInstance();

    void initVoxelyze();
    void emitVoxelMeshUpdated();

    bool loadVXA();

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

    // VoxCad --
    CVX_MeshUtil* _voxelMesh;
    VoxCad* _voxCad;
    //rviz::robot
    //rviz::Robot robot;
};

#endif // ___ROBOT_VOXELYZE_ADAPTER_H___

