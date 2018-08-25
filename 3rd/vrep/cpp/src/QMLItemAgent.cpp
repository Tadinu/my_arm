#include <QtGlobal> // Q_OS_WIN
#ifdef Q_OS_WIN
#include <qt_windows.h>  // Resolve Qt5 - qdatetime - std::numeric_limits<qint64>::min()
#endif
#include <QQmlProperty>
#include "QMLItemAgent.h"
#include "RbGlobal.h"

/* SINGLETON =====================================================================
QMLItemAgent *QMLItemAgent::getInstance()
{
    if (nullptr == _instance) {
        _instance = new MaterialDialog();*

        Q_ASSERT(_instance != nullptr);
    }

    return _instance;
}

void QMLItemAgent::deleteInstance()
{
    if (nullptr != _instance) {
        delete _instance;
        _instance = nullptr; // NULLIFY _instance to mark it DELETED
    }
}
================================================================================== */

QMLItemAgent::QMLItemAgent(QObject *parent,
                           int itemId, QObject* itemUI)
            : QObject(parent),
              // UI properties --
              _itemId(itemId),
              _itemUI(itemUI),
              //
              // State machine properties --
              _currentStateRuleId(0),
              //
              // Threading properties --
              _isRunningOnThread(false),
              _thread(nullptr),
              _mutex(nullptr)
{}

QMLItemAgent::~QMLItemAgent()
{
    // IF SINGLETON:
    //     s_instance = nullptr;
    // => Nullify _instance upon deletion (here in case it was forgotten doing so)!

    _mutex->tryLock(5000);
    _mutex->unlock();

    if(!_thread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
    {
        qWarning("Thread deadlock detected, bad things may have happened !!!");
        _thread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        _thread->wait(); //Note: We have to wait again here!
    }

    delete _mutex;
}

void QMLItemAgent::init()
{}

void QMLItemAgent::setupUI()
{}

int QMLItemAgent::itemId() const
{
    return _itemId;
}

void QMLItemAgent::setupAgentAndUI(const QString& qmlPropertyName)
{
    // [QML Item UI]
    QQmlProperty prop(RbGlobal::qmlCom(), qmlPropertyName);
    _itemUI = qvariant_cast<QObject *>(prop.read()); //!!!

    // [C++ Item Agent]
    QMetaObject::invokeMethod(_itemUI, "regItemAgent", 
                              Q_ARG(QVariant, QVariant::fromValue(this)));

    //
    setupUI();
}

QObject* QMLItemAgent::UI() const
{
    return _itemUI;
}

void QMLItemAgent::setRunningOnThread(bool isYes)
{
    _isRunningOnThread = isYes;
}

void QMLItemAgent::startThreading()
{
    if(!_isRunningOnThread) {
        return;
    }
    // -----------------------------------------------------------
    // Threading start
    _thread = new QThread();
    this->moveToThread(_thread);
    _mutex = new QMutex();

    connect(_thread, &QThread::started, this, &QMLItemAgent::run);
    _thread->start();
}
