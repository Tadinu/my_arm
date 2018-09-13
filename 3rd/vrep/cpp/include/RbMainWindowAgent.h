#ifndef RB_MAINWINDOW_AGENT_H
#define RB_MAINWINDOW_AGENT_H

#include <QObject>
#include <QtCore>
#include "QMLAdapter.h"
#include "QMLItemAgent.h"

class RbMainWindowAgent : public QObject
{
    Q_OBJECT
public:

public:
    RbMainWindowAgent(int argc = 0, char **argv = 0, QObject *parent = 0);
    ~RbMainWindowAgent();

    static RbMainWindowAgent *getInstance(int argc = 0, char **argv = 0);
    static void deleteInstance();
    void initializeQMLItemAgents();
    void initializeQMLContent();
    void setQmlCom(QObject* qmlCom);

    void startRobotManager();
    Q_INVOKABLE QVariant getFrontVisionSensorImageId();
    Q_INVOKABLE QVariant getGroundVisionSensorImageId();
    Q_INVOKABLE void setRobotVel(float vel);
    Q_INVOKABLE void setRobotQueryTimeout(int timeout);
    Q_INVOKABLE void setFallingObjectTimeInterval(int timeInterval);
public slots:

signals:
    void robotVelUpdateOrdered(float vel);
    void robotQueryTimeoutUpdated(int timeout);
    void fallingObjTimeInvervalUpdated(int timeInterval);

private:
    static RbMainWindowAgent *_instance;
};

#endif // RB_MAINWINDOW_AGENT_H
