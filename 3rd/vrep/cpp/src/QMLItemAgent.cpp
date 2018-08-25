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

    if(_thread != nullptr && !_thread->wait(5000)) //Wait until it actually has terminated (max. 5 sec)
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
    RB_THREAD_INVOKE_I(_itemUI, regItemAgent, QVariant::fromValue(this));

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
    // -----------------------------------------------------------
    // Threading start
    if(!_isRunningOnThread) {
        return;
    }
    //
    _thread = new QThread();
    _mutex = new QMutex();

    // -----------------------------------------------------------
    // MOVE THIS TO THE THREAD
    this->setParent(nullptr);
    this->moveToThread(_thread);
    /* !Note:
        You can only use moveToThread in case when

            Your object has no parent (because otherwise the parent will have different thread affinity)
            You are on the object's OWNER thread so you actually 'push' the object from current thread to another

        So your error message says you're violating the second case.
        !!!You MUST call object's moveToThread from the thread that created the object.!!!
    */

    // -----------------------------------------------------------
    // START EVENT LOOP OF THE THREAD:
    _thread->start(); // --> run() --> exec(), starts its internal event loop
}

void QMLItemAgent::runTask()
{
    //printf("QMLItemAgent - RUN TASK\n");
    this->runStateMachineOperation();
}
