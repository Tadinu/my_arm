#pragma once
#include <QApplication>
#include <QWaitCondition>
//#include <QReadWritelock>
#include <QXmlStreamWriter>
#include <QEventLoop>
#include <QMetaObject>

#include <ros/ros.h>
#include <ros/console.h>

#if   ( (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)) &&  (QT_VERSION < QT_VERSION_CHECK(6, 0, 0)) )
#define K3DS_QT5
#elif ( (QT_VERSION >= QT_VERSION_CHECK(4, 0, 0)) &&  (QT_VERSION < QT_VERSION_CHECK(5, 0, 0)) )
#define K3DS_QT4
#endif

#define V_PI (3.1415926535897931)

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

#define V_UNLOCK_DELETE_PMUTEX(pMutex) { \
    pMutex->unlock(); delete pMutex; pMutex = nullptr; \
}

#define V_TF_2_QVECTOR3D(tfVector) (QVector3D(tfVector.x(), tfVector.y(), tfVector.z()))
#define V_QVECTOR3D_2TF(qVector)   (tf::Vector3(qVector.x(), qVector.y(), qVector.z()))
// ----------------------------------------------------------------------------------
// ROS COMMON DEFINE


// PACKAGE NAME --
#define CROS_PACKAGE_NAME ("my_arm")

// FRAMES --
#define CWORLD_FRAME ("world")
#define CBASE_LINK ("base_link")
#define RAD_2_ANGLE(rad)   ((rad*180)/V_PI)
#define ANGLE_2_RAD(angle) ((angle*V_PI)/180)

// VMARKER --
#define VMARKER_INSTANCE() VMarker::getInstance()

// LEAP / REALSENSE HANDS --
//
#define ROBOT_LEAP_HANDS
//#define ROBOT_REAL_SENSE_HANDS

#ifdef ROBOT_LEAP_HANDS
#define VLEAP_INSTANCE() RobotLeapAdapter::getInstance()
#elif defined ROBOT_REAL_SENSE_HANDS
#define VREAL_SENSE_INSTANCE() RobotRealSenseAdapter::getInstance()
#endif
#define CLEAP_BASE_FRAME (CWORLD_FRAME)

// DART --
#define ROBOT_DART
#ifdef ROBOT_DART
#define VDART_ADAPTER() RobotDartAdapter::getInstance()
#endif

//#define USING_PISA_SOFT_HAND_ONLY

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
        VBRHAND_ARM         = 0x01,
        VJACO_ARM           = 0x02,
        VPISA_SOFT_HAND_ARM = 0x03,
        VSHADOW_HAND_ARM    = 0x04
    };

    // BRHAND ARM ----------------------------------------
    //
    enum VBRHAND_ARM_JOINT {
        // Arm --
        //
        VBRHAND_ARM_BASE_JOINT,  // Revolute Z
        VJOINT2,                 // Continuous Y
        VJOINT20,                // Fixed - at Joint1 pos
        VJOINT3,                 // Continuous Y
        VJOINT30,                // Fixed - at Joint2 pos
        VJOINT4,                 // Revolute Z

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

        VBRHAND_ARM_JOINT_TOTAL
    };

    // JACO ARM ---------------------------------------
    //
    enum VJACO_ARM_JOINT {
        // Arm --
        //
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

    // PISA SOFT HAND ------------------------------------
    //
    enum VPISA_SOFT_HAND_ARM_JOINT {
#ifndef USING_PISA_SOFT_HAND_ONLY
        // Arm --
        //
        VPISA_SOFT_HAND_ARM_BASE_JOINT,  // Revolute Z
        VPISA_SOFT_HAND_JOINT2,          // Continuous Y
        VPISA_SOFT_HAND_JOINT20,         // Fixed - at Joint2 pos
        VPISA_SOFT_HAND_JOINT3,          // Continuous Y
        VPISA_SOFT_HAND_JOINT30,         // Fixed - at Joint3 pos
        VPISA_SOFT_HAND_JOINT4,          // Revolute Z
#endif
        // PACMAN VERION ALREADY INCLUDES THE COUPLER, CLAMP AND BASE
        //
        // Kuka Coupler --
        //VPISA_SOFT_HAND_KUKA_COUPLER_JOINT,        // Fixed
        //VPISA_SOFT_HAND_KUKA_COUPLER_BASE_JOINT,   // Fixed

        // Clamp --
        //VPISA_SOFT_HAND_CLAMP_JOINT,               // Fixed

        // Palm --
        VPISA_SOFT_HAND_PALM_JOINT,                  // Fixed

        // Synergy --
        VPISA_SOFT_HAND_SYNERGY_JOINT,               // Revolute

        // Fingers --
        //
        // Thumb
        VPISA_FINGER_THUMB_ABD_JOINT,
        VPISA_FINGER_THUMB_INNER_JOINT,
        VPISA_FINGER_THUMB_INNER_MIMIC_JOINT,
        VPISA_FINGER_THUMB_OUTER_JOINT,
        VPISA_FINGER_THUMB_OUTER_MIMIC_JOINT,

        // Index
        VPISA_FINGER_1_ABD_JOINT,
        VPISA_FINGER_1_INNER_JOINT,
        VPISA_FINGER_1_INNER_MIMIC_JOINT,
        VPISA_FINGER_1_MIDDLE_JOINT,
        VPISA_FINGER_1_MIDDLE_MIMIC_JOINT,
        VPISA_FINGER_1_OUTER_JOINT,
        VPISA_FINGER_1_OUTER_MIMIC_JOINT,

        // Middle
        VPISA_FINGER_2_ABD_JOINT,
        VPISA_FINGER_2_INNER_JOINT,
        VPISA_FINGER_2_INNER_MIMIC_JOINT,
        VPISA_FINGER_2_MIDDLE_JOINT,
        VPISA_FINGER_2_MIDDLE_MIMIC_JOINT,
        VPISA_FINGER_2_OUTER_JOINT,
        VPISA_FINGER_2_OUTER_MIMIC_JOINT,

        // Ring
        VPISA_FINGER_3_ABD_JOINT,
        VPISA_FINGER_3_INNER_JOINT,
        VPISA_FINGER_3_INNER_MIMIC_JOINT,
        VPISA_FINGER_3_MIDDLE_JOINT,
        VPISA_FINGER_3_MIDDLE_MIMIC_JOINT,
        VPISA_FINGER_3_OUTER_JOINT,
        VPISA_FINGER_3_OUTER_MIMIC_JOINT,

        // Little
        VPISA_FINGER_4_ABD_JOINT,
        VPISA_FINGER_4_INNER_JOINT,
        VPISA_FINGER_4_INNER_MIMIC_JOINT,
        VPISA_FINGER_4_MIDDLE_JOINT,
        VPISA_FINGER_4_MIDDLE_MIMIC_JOINT,
        VPISA_FINGER_4_OUTER_JOINT,
        VPISA_FINGER_4_OUTER_MIMIC_JOINT,

        VPISA_SOFT_HAND_ARM_JOINT_TOTAL
    };

    // PISA SOFT HAND ------------------------------------
    //
    enum VSHADOW_HAND_ARM_JOINT {
        VSHADOW_HAND_ARM_BASE_JOINT,                      // "Root World Joint"

        // Forearm
        // Wrist
        VSHADOW_HAND_WRJ2,                                // Revolute
        // Palm
        VSHADOW_HAND_WRJ1,                                // Revolute

        // Fingers
        // Thumb
        VSHADOW_FINGER_THUMB_THJ5,                        // Thumb base, revolute
        VSHADOW_FINGER_THUMB_THJ4,                        // Thumb proximal, revolute
        VSHADOW_FINGER_THUMB_THJ3,                        // Thumb hub, revolute
        VSHADOW_FINGER_THUMB_THJ2,                        // Thumb middle, revolute
        VSHADOW_FINGER_THUMB_THJ1,                        // Thumb distal, revolute
        VSHADOW_FINGER_THUMB_TIP,                         // Thumb tip

        // Index
        VSHADOW_FINGER_1_J4,                              // knuckle, Revolute
        VSHADOW_FINGER_1_J3,                              // Proximal, Revolute
        VSHADOW_FINGER_1_J2,                              // Standard Middle , Revolute
        VSHADOW_FINGER_1_J1,                              // Distal , Revolute

        // Middle
        VSHADOW_FINGER_2_J4,                              // knuckle, Revolute
        VSHADOW_FINGER_2_J3,                              // Proximal, Revolute
        VSHADOW_FINGER_2_J2,                              // Standard Middle , Revolute
        VSHADOW_FINGER_2_J1,                              // Distal , Revolute

        // Ring
        VSHADOW_FINGER_3_J4,                              // knuckle, Revolute
        VSHADOW_FINGER_3_J3,                              // Proximal, Revolute
        VSHADOW_FINGER_3_J2,                              // Standard Middle , Revolute
        VSHADOW_FINGER_3_J1,                              // Distal , Revolute

        // Little Finger (Pinkie)
        VSHADOW_FINGER_4_LFJ5,                            // lfmetacarpal, Revolute
        VSHADOW_FINGER_4_LFJ4,                            // knuckle, Revolute
        VSHADOW_FINGER_4_J3,                              // Proximal, Revolute
        VSHADOW_FINGER_4_J2,                              // Standard Middle , Revolute
        VSHADOW_FINGER_4_J1,                              // Distal , Revolute

        VSHADOW_HAND_ARM_JOINT_TOTAL
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

    static double rand( double min, double max )
    {
        double t = (double)std::rand() / (double)RAND_MAX;
        return min + t*(max-min);
    }

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
