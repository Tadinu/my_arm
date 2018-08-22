#include <stdio.h>
using namespace std;
#include <iostream>

//#include <QMutexLocker>
#include <QWriteLocker>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>
#include <QDebug>

#include "RbGlobal.h"

RbGlobal* RbGlobal::_instance  = nullptr;
QApplication* RbGlobal::_rbApp = nullptr;
QObject* RbGlobal::_qmlCom  = nullptr;

RbGlobal::RbGlobal(QObject* parent):
          QObject(parent)
{

}

RbGlobal* RbGlobal::getInstance()
{
    if (!_instance)
        _instance = new RbGlobal();

    return _instance;
}

RbGlobal::~RbGlobal(void)
{
    delete _instance;
}

QObject* RbGlobal::qmlCom()
{
    return _qmlCom;
}

void RbGlobal::regQMLCom(QObject* qmlCom)
{
    _qmlCom = qmlCom;
}

void RbGlobal::startWaitInfoTimer(int interval)
{
#if 0
    // Main thread only
    // 
    _waitInfoTimer.setInterval(interval);
    _waitInfoTimer.setSingleShot(true);

    connect(&_waitInfoTimer, SIGNAL(timeout()), _instance, SLOT(clearWaitingInfo()));
    _waitInfoTimer.start();
#endif
}

void RbGlobal::stopWaitInfoTimer()
{
#if 0
    // Main thread only
    // 
    _waitInfoTimer.stop();
    disconnect(&_waitInfoTimer, SIGNAL(timeout()), _instance, SLOT(clearWaitingInfo()));
#endif
}

void RbGlobal::clearWaitingInfo()
{
    RB_CLEAR_WAITING_INFO(); // This is thread-safe!
}

QApplication* RbGlobal::rbApp()                     { return _rbApp;}
