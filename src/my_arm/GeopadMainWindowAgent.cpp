//#include "CommonLib/commondefines.h"
#include "GeopadMainWindowAgent.h"
#include "KsGlobal.h"
//#include "commonresource.h"

#include <QtGui>
#include <QThread>
#include <QtConcurrent/QtConcurrentRun>
#include <QUrl>
#include <QMutex>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>
#include <QDateTime>
#include <QFile>
//#include <QFileDialog>

//#include <tchar.h>
#include <functional> // std::bind

//#include <QSystemTrayIcon>
//#include "guiutil.h"
//#include "fileutil.h"
//#include "CommonLib/KConvert.h"
//#include "CommonLib/KVersionComparator.h"
//#include <JlCompress.h>
//#include "GeopadUtil.h"
//#include "fileutil.h"
//#include "fvupdater.h"

//#include "RobotAgent.h"
#define ARRAY_SIZE(x)		(sizeof(x) / sizeof((x)[0]))

GeopadMainWindowAgent* GeopadMainWindowAgent::_instance = nullptr;

// --- CalibrationDialog ---
GeopadMainWindowAgent::GeopadMainWindowAgent(int argc, char **argv, QObject *parent)
    :QObject(parent),
     _robotThread(argc, argv)
{
    // For Common KStudio Download Manager
    //connect(&_kDownloader, SIGNAL(progressDownloading(quint8)), this, SLOT(updateDownloadProgress(quint8)));
    //connect(&_kDownloader, SIGNAL(completedDownloading(int, const QString&, quint64, const QString&, int)), this, SLOT(finalizeUpdateGeopadData(int, const QString&, quint64, const QString&, int)));
    // - CANCEL DOWNLOADING JOB: TO BE CONNECTED ON DOWNLOAD ACTION BEING TRIGGERED
    // connect(&_progressDialog, SIGNAL(canceled()), &_kDownloader, SLOT(cancelDownloading()));

    connect(&_robotThread, &RobotThread::newPose, this, &GeopadMainWindowAgent::updatePoseDisplay);
    connect(&_robotThread, &RobotThread::jointPosUpdated, this, &GeopadMainWindowAgent::updateJointPosInfo);

    _robotThread.init();
}

GeopadMainWindowAgent::~GeopadMainWindowAgent()
{
}

GeopadMainWindowAgent *GeopadMainWindowAgent::getInstance(int argc, char **argv)
{
    if (NULL == _instance) {
        _instance = new GeopadMainWindowAgent(argc, argv, nullptr);

        Q_ASSERT(_instance != NULL);
    }

    return _instance;
}

void GeopadMainWindowAgent::deleteInstance()
{
    if (NULL != _instance) {
        delete _instance;
        _instance = nullptr;
    }
}

void GeopadMainWindowAgent::setK3DQmlCom(QObject* k3dQMLCom)
{
    assert(k3dQMLCom != nullptr);
    KsGlobal::regK3DQMLCom(k3dQMLCom);
}

void GeopadMainWindowAgent::initializeQMLContent()
{
    this->initializeQMLItemAgents();
    //K3D_QML_INVOKE_I(setWindowTitle, cmmInf::geopad_AppName);
}

void GeopadMainWindowAgent::initializeQMLItemAgents()
{
}


void GeopadMainWindowAgent::qmlLog(QVariant logVariant)
{
}

void GeopadMainWindowAgent::goForward(){_robotThread.SetSpeed(0.25, 0);}
void GeopadMainWindowAgent::goBackward(){_robotThread.SetSpeed(-0.25, 0);}
void GeopadMainWindowAgent::goRight(){_robotThread.SetSpeed(0, -V_PI / 6.0);}
void GeopadMainWindowAgent::goLeft(){_robotThread.SetSpeed(0, V_PI / 6.0);}
void GeopadMainWindowAgent::halt(){ _robotThread.SetSpeed(0, 0); }

void GeopadMainWindowAgent::updatePoseDisplay(double x, double y, double theta)
{
    QString xPose, yPose, aPose;
    //xPose.setNum(x);
    //yPose.setNum(y);
    //aPose.setNum(theta);
    //
    //p_xDisplay->setText(xPose);
    //p_yDisplay->setText(yPose);
    //p_aDisplay->setText(aPose);
}//update the display.

void GeopadMainWindowAgent::updateJointPosInfo(int jointId, const QVector3D& jointPos, double jointRotAngle)
{

}

void GeopadMainWindowAgent::setRobotJointPos(int elementId, double pos)
{
     _robotThread.rotateJoint(elementId, pos);
}

void GeopadMainWindowAgent::moveTarget(const QVector3D& distance)
{
    _robotThread.moveBall(tf::Vector3(distance.x(), distance.y(), distance.z()));
}

void GeopadMainWindowAgent::setTargetPos(const QVector3D& pos)
{
    _robotThread.setBallPos(tf::Vector3(pos.x(), pos.y(), pos.z()), true);
}

void GeopadMainWindowAgent::onRobotPostureReset()
{
    _robotThread.updateBallFollowingEndTip(); // Joint values mean angle values, Link values mean length values
}
