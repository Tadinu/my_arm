#ifndef RB_MAINWINDOW_AGENT_H
#define RB_MAINWINDOW_AGENT_H

#include <QObject>
#include <QtCore>
#include "QMLAdapter.h"
#include "QMLItemAgent.h"
#include "RbRobotManager.h"

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

    void startRobotAgent();
    bool isRobotAgentHalted();
    void startSensorAgents();
    bool isSensorAgentsHalted();

    RbRobotAgent* getRobotAgent() { return _robotAgent; }
    Q_INVOKABLE QVariant getFrontVisionSensorImageId();
    Q_INVOKABLE QVariant getGroundVisionSensorImageId();
public slots:

private:
    static RbMainWindowAgent *_instance;
    RbRobotManager* _robotThread;
    RbRobotAgent* _robotAgent;
};

#endif // RB_MAINWINDOW_AGENT_H
