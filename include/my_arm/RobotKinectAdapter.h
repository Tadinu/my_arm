#ifndef ___ROBOT_KINECT_ADAPTER_H___
#define ___ROBOT_KINECT_ADAPTER_H___

#include <QtCore>
#include <QMutex>
#include "KsGlobal.h"
#include "RealSense/camera/sr300_nodelet.h"

class RobotKinectAdapter : public QObject {
    Q_OBJECT

public:
    static RobotKinectAdapter* getInstance();
    ~RobotKinectAdapter();
    static void deleteInstance();

    void initRealSense(ros::NodeHandle* nodeHandle);
    std::vector<std::vector<double>> getFingerJointValues(int hand_id);
    void emitFingerPosesChanged();

    int doHandAnalyzing();
signals:
    void fingerPosesChanged();

private:
    RobotKinectAdapter();
    static RobotKinectAdapter* _instance;
    //
    QMutex* _pMutex;
};

#endif // ___ROBOT_REAL_SENSE_ADAPTER_H___

