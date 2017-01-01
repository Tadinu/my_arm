#pragma once
#include <QApplication>
#include <QWaitCondition>
//#include <QReadWritelock>
#include <QXmlStreamWriter>
#include <QEventLoop>
#include <QMetaObject>

#include <ros/console.h>

#if   ( (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)) &&  (QT_VERSION < QT_VERSION_CHECK(6, 0, 0)) )
#define K3DS_QT5
#elif ( (QT_VERSION >= QT_VERSION_CHECK(4, 0, 0)) &&  (QT_VERSION < QT_VERSION_CHECK(5, 0, 0)) )
#define K3DS_QT4
#endif

#define PI (3.14)

#define K3D_MEMFUNC_CALL(object, method) ((object).*(method))

#define K3D_THREAD_INVOKE(instance, func)                                            \
        QMetaObject::invokeMethod(instance, #func);

#define K3D_THREAD_BLOCKING_INVOKE(instance, func)                                   \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection);

#define K3D_THREAD_MAIN_INVOKE_RET(instance, func, ret)                              \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_RETURN_ARG(QVariant, ret));

#define K3D_THREAD_BLOCKING_INVOKE_RET(instance, func, ret)                          \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_RETURN_ARG(QVariant, ret));

#define K3D_THREAD_INVOKE_I(instance, func, param)                                   \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param));

#define K3D_THREAD_BLOCKING_INVOKE_I(instance, func, param)                          \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_ARG(QVariant, param));

#define K3D_THREAD_MAIN_INVOKE_RET_I(instance, func, ret, param)                     \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param));

#define K3D_THREAD_BLOCKING_INVOKE_RET_I(instance, func, ret, param)                 \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param));

#define K3D_THREAD_INVOKE_II(instance, func, param1, param2)                         \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define K3D_THREAD_BLOCKING_INVOKE_II(instance, func, param1, param2)                \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define K3D_THREAD_MAIN_INVOKE_RET_II(instance, func, ret, param1, param2)           \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define K3D_THREAD_BLOCKING_INVOKE_RET_II(instance, func, ret, param1, param2)       \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define K3D_THREAD_INVOKE_III(instance, func, param1, param2, param3)                \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3));

#define K3D_THREAD_BLOCKING_INVOKE_III(instance, func, param1, param2, param3)       \
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3));

#define K3D_THREAD_INVOKE_IV(instance, func, param1, param2, param3, param4)         \
        QMetaObject::invokeMethod(instance, #func,                                   \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4));

#define K3D_THREAD_INVOKE_IIX(instance, func, param1, param2, param3, param4,        \
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

#define K3D_THREAD_BLOCKING_INVOKE_IV(instance, func, param1, param2, param3, param4)\
        QMetaObject::invokeMethod(instance, #func, Qt::BlockingQueuedConnection,     \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4));

//!NOTE if KsGlobal::k3dQMLCom()[QML} is disabled, these function invoke call is useless:
// K3DBUILDER.QML funcs:
#define K3D_QML_INVOKE(func)                                                         \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func); // KsGlobal::k3dQMLCom() != nullptr is already checked inside!

#define K3D_QML_INVOKE_I(func, param)                                                \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_ARG(QVariant, param));

#define K3D_QML_INVOKE_II(func, param1, param2)                                      \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));

#define K3D_QML_INVOKE_III(func, param1, param2, param3)                             \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3));

#define K3D_QML_INVOKE_IV(func, param1, param2, param3, param4)                      \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4));

#define K3D_QML_INVOKE_V(func, param1, param2, param3, param4, param5)               \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4),                           \
                                  Q_ARG(QVariant, param5));

#define K3D_QML_INVOKE_VI(func, param1, param2, param3, param4, param5, param6)      \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3),                           \
                                  Q_ARG(QVariant, param4),                           \
                                  Q_ARG(QVariant, param5),                           \
                                  Q_ARG(QVariant, param6));


#define K3D_QML_INVOKE_RET(func, ret)                                                \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_RETURN_ARG(QVariant, ret));

#define K3D_QML_INVOKE_RET_I(func, ret, param)                                       \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param));

#define K3D_QML_INVOKE_RET_II(func, ret, param1, param2)                             \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2));
#define K3D_QML_INVOKE_RET_III(func, ret, param1, param2, param3)                    \
        QMetaObject::invokeMethod(KsGlobal::k3dQMLCom(), #func,                      \
                                  Q_RETURN_ARG(QVariant, ret),                       \
                                  Q_ARG(QVariant, param1),                           \
                                  Q_ARG(QVariant, param2),                           \
                                  Q_ARG(QVariant, param3));

//!NOTE: Normally, when a worker thread requests a Waiting Info message, it does not care about
// when main thread really pops up and clear the message. Worker thread just do its job then finish.
// That's why we could just ust Qt::AutoConnection(default) as connection type.
#define K3D_SHOW_WAITING_INFO(message) {                                             \
        K3D_QML_INVOKE_I(showWaitingInfo, message);                                  \
        /* No call qApp->processEvents() in worker thread */                         \
        if(guiutil::isCurrentlyInMainThread()) {                                     \
            /*qApp->processEvents(QEventLoop::ExcludeUserInputEvents); */            \
        }                                                                            \
        QMetaObject::invokeMethod(KsGlobal::getInstance(), "startWaitInfoTimer",     \
                                  Q_ARG(int, 15000));                                \
    }

#define K3D_SHOW_WAITING_INFO_SET_TIME(message, timeout) {                                             \
        K3D_QML_INVOKE_I(showWaitingInfo, message);                                  \
        /* No call qApp->processEvents() in worker thread */                         \
        if(guiutil::isCurrentlyInMainThread()) {                                     \
            /*qApp->processEvents(QEventLoop::ExcludeUserInputEvents); */            \
        }                                                                            \
        QMetaObject::invokeMethod(KsGlobal::getInstance(), "startWaitInfoTimer",     \
                                  Q_ARG(int, timeout));                                \
    }

// Use for Clearing 'Waiting Info' only:
#define K3D_CLEAR_WAITING_INFO() {                                                   \
        K3D_QML_INVOKE(hideWaitingInfo);                                             \
        QMetaObject::invokeMethod(KsGlobal::getInstance(), "stopWaitInfoTimer");     \
    }

// Use for Hiding Notification with answer: QMessageBox::
#define K3D_CLEAR_NOTIFICATION(answer) {                                             \
        K3D_QML_INVOKE_I(hideNotification, answer);                                  \
    }

#define K3D_TIMER_SINGLE_SHOT(codeSnippet) {                                         \
        QTimer::singleShot(500, [=] {                                                \
            codeSnippet;                                                             \
        });                                                                          \
    }

// -----------------------------------------------------------------------------------
// Shared Memory keys between Nyomo application processes (Unyk, UC)
#define GB_SHARED_KEY_UC_STATE   (QLatin1String("KevSharedPrinterState"))
#define GB_SHARED_KEY_PRINT_INFO (QLatin1String("KevSharedPrintInfo"))

#define V_DELETE_POINTER(pt) { \
    delete pt; pt = nullptr; \
}

#define V_DELETE_POINTER_ARRAY(ptArr) { \
    delete [] ptArr; ptArr = nullptr;    \
}

class KsGlobal : public QObject
{
    Q_OBJECT
public:

    enum ERROR_DOWNLOAD {
        ERROR_DOWNLOAD_OK,
        ERROR_DOWNLOAD_NULL_URL,
        ERROR_DOWNLOAD_OPEN_FILE,
        ERROR_DOWNLOAD_NETWORK
    };

    enum VROBOT_TYPE {
        VMY_ARM   = 0x01,
        VJACO_ARM = 0x02
    };

    enum VMY_ARM_JOINT {
        VMYARM_BASE_JOINT,    // Revolute
        VJOINT20,             // Fixed
        VJOINT2,              // Revolute
        VJOINT3,              // Revolute

        // Fingers --
        //
        VFINGER_1_PROX_JOINT,
        VFINGER_1_MED_JOINT,
        VFINGER_1_DIST_JOINT,

        VFINGER_2_PROX_JOINT,
        VFINGER_2_MED_JOINT,
        VFINGER_2_DIST_JOINT,

        VFINGER_3_MED_JOINT,
        VFINGER_3_DIST_JOINT,

        VMY_ARM_JOINT_TOTAL
    };

    enum VJACO_ARM_JOINT {
        VJACO_ARM_BASE_JOINT, // Fixed
        VBASE_INTERNAL_JOINT, // Fixed

        VRING_1_JOINT,        // Fixed
        VARM_0_JOINT,         // Continuous

        VRING_2_JOINT,        // Fixed
        VARM_1_JOINT,         // Revolute

        VRING_3_JOINT,        // Fixed
        VARM_2_JOINT,         // Revolute

        VRING_4_JOINT,        // Fixed
        VARM_3_JOINT,         // Continuous

        VRING_5_JOINT,        // Fixed
        VARM_4_JOINT,         // Continuous

        VRING_6_JOINT,        // Fixed
        VARM_5_JOINT,         // Continuous

        // Fingers --
        //
        VFINGERS_BASE_JOINT,  // Fixed
        VFINGERS_MOUNT_INDEX_JOINT, // Fixed

        VFINGERS_JOINT_0,     // Revolute
        VFINGERS_JOINT_1,     // Fixed

        VFINGERS_MOUNT_THUMB_JOINT, // Fixed
        VFINGERS_JOINT_2,     // Revolute

        VFINGERS_JOINT_3,     // Fixed
        VFINGERS_MOUNT_PINKIE_JOINT, // Fixed

        VFINGERS_JOINT_4,     // Revolute
        VFINGERS_JOINT_5,     // Fixed

        VJACO_ARM_JOINT_TOTAL
    };

public:
    KsGlobal(QObject* parent = nullptr);
    static KsGlobal* getInstance();

    static QApplication* ksApp();
    static QObject* k3dQMLCom();
    static void regK3DQMLCom(QObject* k3dQMLCom);

    void clearWaitingInfo();

    // To be invoked from worker threads
    Q_INVOKABLE static void startWaitInfoTimer(int interval);
    Q_INVOKABLE static void stopWaitInfoTimer();

signals:

private: 
    ~KsGlobal(void);
    static KsGlobal* _instance;
    static QTimer    _waitInfoTimer;

    static QApplication* _ksApp;
    static QObject* _k3dQMLCom;
};

// =============================================================================================
// [Class] KEventLoop --------------------------------------------------------------------------
// 
class KEventLoop : public QEventLoop
{
    Q_OBJECT
public:
    KEventLoop(QObject* parent = nullptr) :
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
