#include <QXmlStreamWriter>
#include <QDesktopWidget>

//#include <QMutexLocker>
#include <QWriteLocker>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>

#include <QCryptographicHash> // For hashing the user info
#include <QDebug>

#include "KsGlobal.h"

KsGlobal* KsGlobal::_instance  = nullptr;
QApplication* KsGlobal::_ksApp = nullptr;
QObject* KsGlobal::_k3dQMLCom  = nullptr;

KsGlobal::KsGlobal(QObject* parent):
          QObject(parent)
{

}

KsGlobal* KsGlobal::getInstance()
{
    if (!_instance)
        _instance = new KsGlobal();

    return _instance;
}

KsGlobal::~KsGlobal(void)
{
    delete _instance;
}

QObject* KsGlobal::k3dQMLCom()
{
    return _k3dQMLCom;
}

void KsGlobal::regK3DQMLCom(QObject* k3dQMLCom)
{
    _k3dQMLCom = k3dQMLCom;
}

void KsGlobal::startWaitInfoTimer(int interval)
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

void KsGlobal::stopWaitInfoTimer()
{
 #if 0
    // Main thread only
    // 
    _waitInfoTimer.stop();
    disconnect(&_waitInfoTimer, SIGNAL(timeout()), _instance, SLOT(clearWaitingInfo()));
#endif
}

void KsGlobal::clearWaitingInfo()
{
    K3D_CLEAR_WAITING_INFO(); // This is thread-safe!
}

QApplication* KsGlobal::ksApp()                     { return _ksApp;}
