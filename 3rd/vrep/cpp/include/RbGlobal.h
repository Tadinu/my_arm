#pragma once

#include "commondefines.h"

// QT --
//
#include <QApplication>
#include <QWaitCondition>
//#include <QReadWritelock>
#include <QXmlStreamWriter>
#include <QEventLoop>
#include <QMetaObject>

#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QThread>
#include <QMutex>


#define RB_APP     QUrl(QStringLiteral("qrc:///qml/MainWindow.qml")) // pragma Singleton here-in
#define RB_MAIN_APP (RB_APP)
#define RB_NAME ("Pioneer_p3dx")

#define RB_MEMFUNC_CALL(object, method) ((object).*(method))
#define RB_CHECK_SIGNAL_CONNECTED(signal) (isSignalConnected(QMetaMethod::fromSignal(signal))) // Call inside QObject class

// INVOKE METHODS RUNNING ON ANOTHER THREAD -----------------------------------------
//
#define RB_THREAD_INVOKE(instance, func)                                            \
        QMetaObject::invokeMethod(instance, #func);

#define RB_THREAD_BLOCKING_INVOKE(instance, func)                                   \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection);

#define RB_THREAD_MAIN_INVOKE_RET(instance, func, ret)                              \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_RETURN_ARG(QVariant, ret));

#define RB_THREAD_BLOCKING_INVOKE_RET(instance, func, ret)                          \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_RETURN_ARG(QVariant, ret));

#define RB_THREAD_INVOKE_I(instance, func, param)                                   \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param));

#define RB_THREAD_BLOCKING_INVOKE_I(instance, func, param)                          \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_ARG(QVariant, param));

#define RB_THREAD_MAIN_INVOKE_RET_I(instance, func, ret, param)                     \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param));

#define RB_THREAD_BLOCKING_INVOKE_RET_I(instance, func, ret, param)                 \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param));

#define RB_THREAD_INVOKE_II(instance, func, param1, param2)                         \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define RB_THREAD_BLOCKING_INVOKE_II(instance, func, param1, param2)                \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define RB_THREAD_MAIN_INVOKE_RET_II(instance, func, ret, param1, param2)           \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define RB_THREAD_BLOCKING_INVOKE_RET_II(instance, func, ret, param1, param2)       \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define RB_THREAD_INVOKE_III(instance, func, param1, param2, param3)                \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3));

#define RB_THREAD_BLOCKING_INVOKE_III(instance, func, param1, param2, param3)       \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3));

#define RB_THREAD_INVOKE_IV(instance, func, param1, param2, param3, param4)         \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4));

#define RB_THREAD_INVOKE_IIX(instance, func, param1, param2, param3, param4,        \
                                              param5, param6, param7, param8)        \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4),                           \
                                  Q_ARG(QVariant, param5),                           \
                                  Q_ARG(QVariant, param6),                           \
                                  Q_ARG(QVariant, param7),                           \
                                  Q_ARG(QVariant, param8));

#define RB_THREAD_BLOCKING_INVOKE_IV(instance, func, param1, param2, param3, param4)\
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4));

// INVOKE METHODS RUNNING ON QML THREAD  --------------------------------------------
//
//!NOTE if RbGlobal::qmlCom()[QML} is disabled, these function invoke calls also does not work:
// MAINAPP.QML funcs:
#define RB_QML_INVOKE(func)                                                         \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func); // RbGlobal::qmlCom() != nullptr is already checked inside!

#define RB_QML_INVOKE_I(func, param)                                                \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_ARG(QVariant, param));

#define RB_QML_INVOKE_II(func, param1, param2)                                      \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define RB_QML_INVOKE_III(func, param1, param2, param3)                             \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3));

#define RB_QML_INVOKE_IV(func, param1, param2, param3, param4)                      \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4));

#define RB_QML_INVOKE_V(func, param1, param2, param3, param4, param5)               \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4),                           \
                                  Q_ARG(QVariant, param5));

#define RB_QML_INVOKE_VI(func, param1, param2, param3, param4, param5, param6)      \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4),                           \
                                  Q_ARG(QVariant, param5),                           \
                                  Q_ARG(QVariant, param6));


#define RB_QML_INVOKE_RET(func, ret)                                                 \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_RETURN_ARG(QVariant, ret));

#define RB_QML_INVOKE_RET_I(func, ret, param)                                        \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param));

#define RB_QML_INVOKE_RET_II(func, ret, param1, param2)                              \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));
#define RB_QML_INVOKE_RET_III(func, ret, param1, param2, param3)                     \
        QMetaObject::invokeMethod(RbGlobal::qmlCom(), #func,                      \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3));

//!NOTE: Normally, when a worker thread requests a Waiting Info message, it does not care about
// when main thread really pops up and clear the message. Worker thread just do its job then finish.
// That's why we could just ust Qt::AutoConnection(default) as connection type.
#define RB_SHOW_WAITING_INFO(message) {                                              \
        RB_QML_INVOKE_I(showWaitingInfo, message);                                   \
        /* No call qApp->processEvents() in worker thread */                         \
        if(guiutil::isCurrentlyInMainThread()) {                                     \
            /*qApp->processEvents(QEventLoop::ExcludeUserInputEvents); */            \
        }                                                                            \
        QMetaObject::invokeMethod(RbGlobal::getInstance(), "startWaitInfoTimer",     \
                                  Q_ARG(int, 15000));                                \
    }

#define RB_SHOW_WAITING_INFO_SET_TIME(message, timeout) {                            \
        RB_QML_INVOKE_I(showWaitingInfo, message);                                   \
        /* No call qApp->processEvents() in worker thread */                         \
        if(guiutil::isCurrentlyInMainThread()) {                                     \
            /*qApp->processEvents(QEventLoop::ExcludeUserInputEvents); */            \
        }                                                                            \
        QMetaObject::invokeMethod(RbGlobal::getInstance(), "startWaitInfoTimer",     \
                                  Q_ARG(int, timeout));                              \
    }

// Use for Clearing 'Waiting Info' only:
#define RB_CLEAR_WAITING_INFO() {                                                    \
        RB_QML_INVOKE(hideWaitingInfo);                                              \
        QMetaObject::invokeMethod(RbGlobal::getInstance(), "stopWaitInfoTimer");     \
    }

// Use for Hiding Notification with answer: QMessageBox::
#define RB_CLEAR_NOTIFICATION(answer) {                                              \
        RB_QML_INVOKE_I(hideNotification, answer);                                   \
    }

#define RB_TIMER_SINGLE_SHOT(codeSnippet) {                                          \
        QTimer::singleShot(500, [=] {                                                \
            codeSnippet;                                                             \
        });                                                                          \
    }

// -----------------------------------------------------------------------------------
#define RB_DELETE_POINTER(pt) { \
    delete pt; pt = nullptr; \
}

#define RB_DELETE_POINTER_ARRAY(ptArr) { \
    delete [] ptArr; ptArr = nullptr;    \
}

#define RB_UNLOCK_DELETE_PMUTEX(pMutex) { \
    pMutex->unlock(); delete pMutex; pMutex = nullptr; \
}

class RbGlobal : public QObject
{
    Q_OBJECT
public:

    enum RB_STATE_MACHINE {
        RB_ROBOT_STATES,
        RB_SENSOR_MANAGE_STATES,
        RB_STATE_MACHINE_TOTAL
    };

    enum RB_ROBOT_TYPE {
        RB_ROBOT_CAR = 0x01
    };

    static constexpr const char* CPIONEER_P3DX_CAR_NAME = "Pioneer_p3dx";

    static const int CSERVER_ROBOT_ID = RB_ROBOT_CAR;
    static constexpr const char* CSERVER_ROBOT_NAME = CPIONEER_P3DX_CAR_NAME;
    // error: ‘constexpr’ needed for in-class initialization of static data member ‘const char* RbGlobal::CSERVER_ROBOT_NAME’ of non-integral type [-fpermissive]

    // ROBOT CAR -----------------------------------------
    //
    enum RB_ROBOT_CAR_MOTORS {
        RB_CAR_LEFT_MOTOR,
        RB_CAR_RIGHT_MOTOR
    };

    enum RB_ROBOT_CAR_SENSOR_TYPE {
        RB_SENSOR_TYPE_VISION = 1,
        RB_SENSOR_TYPE_FLOOR,
        RB_SENSOR_TYPE_FORCE,
        RB_SENSOR_TYPE_ULTRASONIC
    };

public:
    RbGlobal(QObject* parent = nullptr);
    static RbGlobal* getInstance();

    static QApplication* rbApp();
    static QObject* qmlCom();
    static void regQMLCom(QObject* qmlCom);

    void clearWaitingInfo();

    // To be invoked from worker threads
    Q_INVOKABLE static void startWaitInfoTimer(int interval);
    Q_INVOKABLE static void stopWaitInfoTimer();

    static double rand( double min, double max )
    {
        double t = (double)std::rand() / (double)RAND_MAX;
        return min + t*(max-min);
    }

signals:

private: 
    ~RbGlobal(void);
    static RbGlobal* _instance;

    static QTimer    _waitInfoTimer;

    static QApplication* _rbApp;
    static QObject* _qmlCom;
};

// =============================================================================================
// [Class] RbEventLoop --------------------------------------------------------------------------
// 
class RbEventLoop : public QEventLoop
{
    Q_OBJECT
public:
    RbEventLoop(QObject* parent = nullptr) :
                QEventLoop(parent), 
                _returnCode(0) {};

    int getReturnCode()    { return _returnCode;      }
    int isSuccessfulExit() { return _returnCode == 0; }
public slots:
    void quitError() {
        quitRet(1);
    }
    void quitRet(int returnCode) {
        _returnCode = returnCode;
        quit(); // exit(0);
    }

    void quitBool(bool result = false) {
        _returnCode = result ? 0 : 1;
        quit();
    }
private:
    int _returnCode;
};
