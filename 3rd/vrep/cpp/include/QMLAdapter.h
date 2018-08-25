#ifndef QML_ADAPTER_H
#define QML_ADAPTER_H

#include <QObject>
#include <QQmlProperty>
#include "RbGlobal.h"
class QJSEngine;
class QQmlEngine;
class RbMainWindowAgent;

// =======================================================================================================================
//
typedef QVariant (RbMainWindowAgent::*RBOP_VOID)(void);
typedef QVariant (RbMainWindowAgent::*RBOP_VAR)(QVariant);
typedef QVariant (RbMainWindowAgent::*RBOP_VAR_VAR)(QVariant, QVariant);

// VOID PARAM --
#define RB_VOID_OPERATION_INVOKE(rbOpId)                                              \
        if(_rbOpVoid[rbOpId] != nullptr)          {                                   \
            return RB_MEMFUNC_CALL(*RbMainWindowAgent::getInstance(), _rbOpVoid[rbOpId])(); \
        }                                                                               \
        else {                                                                          \
            return -1;                             \
        }

// SINGLE VARIANT PARAM --
#define RB_1VARIANT_OPERATION_INVOKE(rbOpId, param)                                           \
        if(_rbOpVariant[rbOpId] != nullptr)       {                                           \
            return RB_MEMFUNC_CALL(*RbMainWindowAgent::getInstance(), _rbOpVariant[rbOpId])(param); \
        }                                                                                       \
        else {                                                                                  \
            return -1;                                                                          \
        }

// DOUBLE VARIANT PARAM --
#define RB_2VARIANT_OPERATION_INVOKE(rbOpId, param1, param2)                                                 \
        if(_rbOpDoubleVariant[rbOpId] != nullptr) {                                                          \
            return RB_MEMFUNC_CALL(*RbMainWindowAgent::getInstance(), _rbOpDoubleVariant[rbOpId])(param1, param2); \
        }                                                                                                      \
        else {                                                                                                 \
            return -1;                                                   \
        }

// DIALOG OPERATION --
#define RB_DIALOG_OPERATION_INVOKE(dialogId, eventType)                                                                    \
        if(_rbOpDialogAgent != nullptr)            {                                                                          \
            return RB_MEMFUNC_CALL(*RbMainWindowAgent::getInstance(), _rbOpDialogAgent)(QVariant(dialogId), QVariant(eventType));\
        }                                                                                                                      \
        else {                                                                                                                 \
            return -1;                                                                    \
        }

// -----------------------------------------------------------------------------------
// LOCAL <QML_ITEM>.QML funcs:
#define QML_ITEM_INVOKE(qmlItemId, func)                                                       \
        QMetaObject::invokeMethod(QMLAdapter::getInstance()->getRbQMLItem(qmlItemId), #func);
#define QML_ITEM_LOCAL_INVOKE(func)                                                            \
        QMetaObject::invokeMethod(this->QMLItemAgent::UI(), #func);

#define QML_ITEM_INVOKE_I(qmlItemId, func, param)                                              \
        QMetaObject::invokeMethod(QMLAdapter::getInstance()->getRbQMLItem(qmlItemId), #func,  \
                                  Q_ARG(QVariant, param));
#define QML_ITEM_LOCAL_INVOKE_I(func, param)                                                   \
        QMetaObject::invokeMethod(this->QMLItemAgent::UI(), #func,                               \
                                  Q_ARG(QVariant, param));

#define QML_ITEM_INVOKE_II(qmlItemId, func, param1, param2)                                    \
        QMetaObject::invokeMethod(QMLAdapter::getInstance()->getRbQMLItem(qmlItemId), #func,  \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2));
#define QML_ITEM_LOCAL_INVOKE_II(func, param1, param2)                                         \
        QMetaObject::invokeMethod(this->QMLItemAgent::UI(), #func,                               \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2));

#define QML_ITEM_INVOKE_III(qmlItemId, func, param1, param2, param3)                           \
        QMetaObject::invokeMethod(QMLAdapter::getInstance()->getRbQMLItem(qmlItemId), #func,  \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2),                                          \
                                  Q_ARG(QVariant, param3));

#define QML_ITEM_LOCAL_INVOKE_III(func, param1, param2, param3)                                \
        QMetaObject::invokeMethod(this->QMLItemAgent::UI(), #func,                               \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2),                                          \
                                  Q_ARG(QVariant, param3));

#define QML_ITEM_INVOKE_RET(qmlItemId, func, ret)                                              \
        QMetaObject::invokeMethod(QMLAdapter::getInstance()->getRbQMLItem(qmlItemId), #func,  \
                                  Q_RETURN_ARG(QVariant, ret));
#define QML_ITEM_LOCAL_INVOKE_RET(func, ret)                                                   \
        QMetaObject::invokeMethod(this->QMLItemAgent::UI(), #func,                               \
                                  Q_RETURN_ARG(QVariant, ret));

#define QML_ITEM_INVOKE_RET_I(qmlItemId, func, ret, param)                                     \
        QMetaObject::invokeMethod(QMLAdapter::getInstance()->getRbQMLItem(qmlItemId), #func,  \
                                  Q_RETURN_ARG(QVariant, ret),                                      \
                                  Q_ARG(QVariant, param));
#define QML_ITEM_LOCAL_INVOKE_RET_I(func, ret, param)                                          \
        QMetaObject::invokeMethod(this->QMLItemAgent::UI(), #func,                               \
                                  Q_RETURN_ARG(QVariant, ret),                                      \
                                  Q_ARG(QVariant, param));

#define QML_ITEM_INVOKE_RET_II(qmlItemId, func, ret, param1, param2)                           \
        QMetaObject::invokeMethod(QMLAdapter::getInstance()->getRbQMLItem(qmlItemId), #func,  \
                                  Q_RETURN_ARG(QVariant, ret),                                      \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2));
#define QML_ITEM_LOCAL_INVOKE_RET_II(func, ret, param1, param2)                                \
         QMetaObject::invokeMethod(this->QMLItemAgent::UI(), #func,                              \
                                  Q_RETURN_ARG(QVariant, ret),                                      \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2));

class QMLAdapter : public QObject {
    Q_OBJECT
    Q_ENUMS(RB_VOID_OPERATION)
    Q_ENUMS(RB_PARAM_OPERATION)
    Q_ENUMS(RB_DOUBLE_PARAM_OPERATION)
    Q_ENUMS(RB_TRIPLE_PARAM_OPERATION)
    Q_ENUMS(QML_ITEM)

    Q_ENUMS(QML_ROBOT_TYPE)
public:

    static const int OBJECT_IMAGE_WIDTH  = 80;
    static const int OBJECT_IMAGE_HEIGHT = 80;

    enum RB_ACTION {
        RB_ACTION_VOID,
        RB_ACTION_TOTAL
    };

    enum RB_QML_CONTEXT_PROPERTY {
        ROBOT_APPLICATION,
        MAIN_WINDOW_AGENT,
        RB_QML_ADAPTER,


        RB_QML_CONTEXT_PROPERTY_TOTAL
    };

    // These names are canonical, which will be used DIRECTLY in QML:
    static const QString CONTEXT_PROPERTY[RB_QML_CONTEXT_PROPERTY_TOTAL];

    enum RB_VOID_OPERATION {
        RB_OP_V,
        RB_OP_VOID_TOTAL
    };

    enum RB_PARAM_OPERATION {
        RB_OP_SINGLE_PARAM,
        RB_OP_SINGLE_PARAM_TOTAL
    };

    enum RB_DOUBLE_PARAM_OPERATION {
        RB_OP_DOUBLE_PARAM,
        RB_OP_DOUBLE_PARAM_TOTAL
    };

    enum RB_TRIPLE_PARAM_OPERATION {
        RB_OP_TRIPLE_PARAM,
        RB_OP_TRIPLE_PARAM_TOTAL
    };
    
    // -----------------------------------
    // FUNCTION OPERATION DIALOS --
    //
    enum RB_OPERATION_DIALOG_EVENT {
        RB_DIALOG_EVENT,
        RB_DIALOG_EVENT_TOTAL
    };

    enum QML_ITEM {
        // DIALOGS --
        QML_ITEM_I,

        /* Add more dialog id here */
        QML_ITEM_TOTAL
    };
    static const QString QML_PROPERTY_ITEM[QML_ITEM_TOTAL];

    //KDialog* getKsDialog(int dialogId);
    //
    enum QML_ROBOT_TYPE {
        ROBOT_CAR = RbGlobal::RB_ROBOT_CAR
    };

public:
    QMLAdapter();
    static QObject* qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine);
    static QMLAdapter* getInstance();
    ~QMLAdapter();

    RBOP_VOID _rbOpVoid[RB_OP_VOID_TOTAL];
    Q_INVOKABLE QVariant runOpVoid(int rbOpId);

    RBOP_VAR  _rbOpVariant[RB_OP_SINGLE_PARAM_TOTAL];
    Q_INVOKABLE QVariant runOpParam(int rbOpId, QVariant param);

    RBOP_VAR_VAR  _rbOpDoubleVariant[RB_OP_DOUBLE_PARAM_TOTAL];
    Q_INVOKABLE QVariant runOpDoubleParam(int rbOpId, QVariant param1, QVariant param2);

    RBOP_VAR_VAR _rbOpDialogAgent;
    Q_INVOKABLE QVariant runDialogOp(int dialogId, int eventType);

    void registerOperations();

    // QML Items --
    void openQMLItem(int qmlItemId); // Open a QML Item
    QObject* getRbQMLItem(int qmlItemId);

private:
    static QMLAdapter* s_instance;
};

#endif // QML_ADAPTER_H

