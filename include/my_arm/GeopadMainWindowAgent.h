#ifndef GEOPAD_MAINWINDOW_AGENT_H
#define GEOPAD_MAINWINDOW_AGENT_H

#include <QObject>
#include <QDateTime>
#include <QtCore>
//#include <QtSerialPort/QSerialPort>
#include "GeopadQMLAdapter.h"
#include "K3DQMLItemAgent.h"
#include <QMessageBox>

#include <QAbstractListModel>
#include <QtQml>
#include <QtNetwork/QLocalServer>
#include <QThread>
#include <QMutex>
#include <QFuture>

#include <QTimer>
//#include "KDownloadManager.h"
//#include "KProgressDialog.h"
//
//#include "SecurityLib/Security.h"

//#include "RobotAgent.h"
#include "RobotThread.h"

class GeopadMainWindowAgent : public QObject
{
    Q_OBJECT
public:

public:
    GeopadMainWindowAgent(int argc = 0, char **argv = 0, QObject *parent = 0);
    ~GeopadMainWindowAgent();
    Q_INVOKABLE void qmlLog(QVariant logVariant);

    static GeopadMainWindowAgent *getInstance(int argc = 0, char **argv = 0);
    static void deleteInstance();
    void initializeQMLItemAgents();
    void initializeQMLContent();
    void setK3DQmlCom(QObject* k3dQMLCom);

public slots:
    void updatePoseDisplay(double x, double y, double theta);
    void goForward();
    void goBackward();
    void goRight();
    void goLeft();
    void halt();

    // GeopadQMLAdapter::VROBOT_ARM_ELEMENT
    void updateJointPosInfo(int jointId, const QVector3D& jointPos, double jointRotAngle);
    Q_INVOKABLE void setRobotJointPos(int elementId, double pos);
    Q_INVOKABLE void moveTarget(const QVector3D& distance);
    Q_INVOKABLE void setTargetPos(const QVector3D& pos);
    Q_INVOKABLE void resetRobotPosture();

private:
    static GeopadMainWindowAgent *_instance;

    //KDownloadManager  _kDownloader;
    //KProgressDialog   _progressDialog;

    RobotThread _robotThread;
};

#endif // GEOPAD_MAINWINDOW_AGENT_H
