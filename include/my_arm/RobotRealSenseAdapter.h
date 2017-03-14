#ifndef ___ROBOT_REAL_SENSE_ADAPTER_H___
#define ___ROBOT_REAL_SENSE_ADAPTER_H___

#include <QtCore>
#include <QMutex>
#include "KsGlobal.h"
#include "RealSense/camera/sr300_nodelet.h"

#ifdef ROBOT_REAL_SENSE_HANDS
#define VREAL_SENSE_INSTANCE() RobotRealSenseAdapter::getInstance()
#endif

class RobotRealSenseAdapter : public QObject {
    Q_OBJECT

public:
    static RobotRealSenseAdapter* getInstance();
    ~RobotRealSenseAdapter();
    static void deleteInstance();

    void initRealSense(ros::NodeHandle* nodeHandle);
    std::vector<std::vector<double>> getFingerJointValues(int hand_id);
    void emitFingerPosesChanged();

    int doHandAnalyzing();
signals:
    void fingerPosesChanged();

private:
    RobotRealSenseAdapter();
    static RobotRealSenseAdapter* _instance;
    //
    QMutex* _pMutex;
};

#endif // ___ROBOT_REAL_SENSE_ADAPTER_H___

