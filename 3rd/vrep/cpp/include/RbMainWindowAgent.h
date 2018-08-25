#ifndef RB_MAINWINDOW_AGENT_H
#define RB_MAINWINDOW_AGENT_H

#include <QObject>
#include <QDateTime>
#include <QtCore>
//#include <QtSerialPort/QSerialPort>
#include "QMLAdapter.h"
#include "QMLItemAgent.h"
#include <QMessageBox>

#include <QAbstractListModel>
#include <QtQml>
#include <QtNetwork/QLocalServer>
#include <QThread>
#include <QMutex>
#include <QFuture>

#include <QTimer>

#include "RbRobotThread.h"

class RbMainWindowAgent : public QObject
{
    Q_OBJECT
public:

public:
    RbMainWindowAgent(int argc = 0, char **argv = 0, QObject *parent = 0);
    ~RbMainWindowAgent();
    Q_INVOKABLE void qmlLog(QVariant logVariant);

    static RbMainWindowAgent *getInstance(int argc = 0, char **argv = 0);
    static void deleteInstance();
    void initializeQMLItemAgents();
    void initializeQMLContent();
    void setQmlCom(QObject* qmlCom);
public slots:

    // QMLAdapter::VROBOT_ARM_ELEMENT
private:
    static RbMainWindowAgent *_instance;
    RbRobotThread _robotThread;
};

#endif // RB_MAINWINDOW_AGENT_H
