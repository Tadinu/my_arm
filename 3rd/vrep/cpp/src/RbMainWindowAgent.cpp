//#include "CommonLib/commondefines.h"
#include "RbMainWindowAgent.h"
#include "RbGlobal.h"
//#include "commonresource.h"

#include <QtGui>
#include <QThread>
#include <QtConcurrent/QtConcurrentRun>
#include <QUrl>
#include <QMutex>
#include <QFuture>
#include <QDateTime>
#include <QFile>
//#include <QFileDialog>

//#include <tchar.h>
#include <functional> // std::bind

//#include "RbRobotAgent.h"

#define ARRAY_SIZE(x)		(sizeof(x) / sizeof((x)[0]))

RbMainWindowAgent* RbMainWindowAgent::_instance = nullptr;

// --- CalibrationDialog ---
RbMainWindowAgent::RbMainWindowAgent(int argc, char **argv, QObject *parent)
    :QObject(parent),
     _robotThread(argc, argv)
{
    //connect(&_robotThread, &RbRobotThread::jointPosUpdated, this, &RbMainWindowAgent::updateJointPosInfo);
    _robotThread.init();
}

RbMainWindowAgent::~RbMainWindowAgent()
{
}

RbMainWindowAgent *RbMainWindowAgent::getInstance(int argc, char **argv)
{
    if (NULL == _instance) {
        _instance = new RbMainWindowAgent(argc, argv, nullptr);

        Q_ASSERT(_instance != NULL);
    }

    return _instance;
}

void RbMainWindowAgent::deleteInstance()
{
    if (NULL != _instance) {
        delete _instance;
        _instance = nullptr;
    }
}

void RbMainWindowAgent::setQmlCom(QObject* qmlCom)
{
    assert(qmlCom != nullptr);
    RbGlobal::regQMLCom(qmlCom);
}

void RbMainWindowAgent::initializeQMLContent()
{
    this->initializeQMLItemAgents();
    //RB_QML_INVOKE_I(setWindowTitle, "Robot Sensors");
}

void RbMainWindowAgent::initializeQMLItemAgents()
{
}


void RbMainWindowAgent::qmlLog(QVariant logVariant)
{
}
