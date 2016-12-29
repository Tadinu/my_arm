#ifndef GEOPAD_QML_ADAPTER_H
#define GEOPAD_QML_ADAPTER_H

#include <QObject>
#include <QQmlProperty>
#include "KsGlobal.h"
class QJSEngine;
class QQmlEngine;
class GeopadMainWindowAgent;

// =======================================================================================================================
//
typedef QVariant (GeopadMainWindowAgent::*GEOPADOP_VOID)(void);
typedef QVariant (GeopadMainWindowAgent::*GEOPADOP_VAR)(QVariant);
typedef QVariant (GeopadMainWindowAgent::*GEOPADOP_VAR_VAR)(QVariant, QVariant);

// VOID PARAM --
#define GEOPAD_VOID_OPERATION_INVOKE(k3dOpId)                                              \
        if(_geopadOpVoid[k3dOpId] != nullptr)          {                                   \
            return K3D_MEMFUNC_CALL(*GeopadMainWindowAgent::getInstance(), _geopadOpVoid[k3dOpId])(); \
        }                                                                               \
        else {                                                                          \
            return -1;                             \
        }

// SINGLE VARIANT PARAM --
#define GEOPAD_1VARIANT_OPERATION_INVOKE(k3dOpId, param)                                           \
        if(_geopadOpVariant[k3dOpId] != nullptr)       {                                           \
            return K3D_MEMFUNC_CALL(*GeopadMainWindowAgent::getInstance(), _geopadOpVariant[k3dOpId])(param); \
        }                                                                                       \
        else {                                                                                  \
            return -1;                                                                          \
        }

// DOUBLE VARIANT PARAM --
#define GEOPAD_2VARIANT_OPERATION_INVOKE(k3dOpId, param1, param2)                                                 \
        if(_geopadOpDoubleVariant[k3dOpId] != nullptr) {                                                          \
            return K3D_MEMFUNC_CALL(*GeopadMainWindowAgent::getInstance(), _geopadOpDoubleVariant[k3dOpId])(param1, param2); \
        }                                                                                                      \
        else {                                                                                                 \
            return -1;                                                   \
        }

// DIALOG OPERATION --
#define GEOPAD_DIALOG_OPERATION_INVOKE(k3dDialogId, eventType)                                                                    \
        if(_geopadOpDialogAgent != nullptr)            {                                                                          \
            return K3D_MEMFUNC_CALL(*GeopadMainWindowAgent::getInstance(), _geopadOpDialogAgent)(QVariant(k3dDialogId), QVariant(eventType));\
        }                                                                                                                      \
        else {                                                                                                                 \
            return -1;                                                                    \
        }

// -----------------------------------------------------------------------------------
// K3PM's LOCAL <QML_ITEM>.QML funcs:
#define GEOPAD_QML_ITEM_INVOKE(qmlItemId, func)                                                       \
        QMetaObject::invokeMethod(GeopadQMLAdapter::getInstance()->getGeopadQMLItem(qmlItemId), #func);
#define GEOPAD_QML_ITEM_LOCAL_INVOKE(func)                                                            \
        QMetaObject::invokeMethod(this->K3DQMLItemAgent::UI(), #func);

#define GEOPAD_QML_ITEM_INVOKE_I(qmlItemId, func, param)                                              \
        QMetaObject::invokeMethod(GeopadQMLAdapter::getInstance()->getGeopadQMLItem(qmlItemId), #func,  \
                                  Q_ARG(QVariant, param));
#define GEOPAD_QML_ITEM_LOCAL_INVOKE_I(func, param)                                                   \
        QMetaObject::invokeMethod(this->K3DQMLItemAgent::UI(), #func,                               \
                                  Q_ARG(QVariant, param));

#define GEOPAD_QML_ITEM_INVOKE_II(qmlItemId, func, param1, param2)                                    \
        QMetaObject::invokeMethod(GeopadQMLAdapter::getInstance()->getGeopadQMLItem(qmlItemId), #func,  \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2));
#define GEOPAD_QML_ITEM_LOCAL_INVOKE_II(func, param1, param2)                                         \
        QMetaObject::invokeMethod(this->K3DQMLItemAgent::UI(), #func,                               \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2));

#define GEOPAD_QML_ITEM_INVOKE_III(qmlItemId, func, param1, param2, param3)                           \
        QMetaObject::invokeMethod(GeopadQMLAdapter::getInstance()->getGeopadQMLItem(qmlItemId), #func,  \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2),                                          \
                                  Q_ARG(QVariant, param3));

#define GEOPAD_QML_ITEM_LOCAL_INVOKE_III(func, param1, param2, param3)                                \
        QMetaObject::invokeMethod(this->K3DQMLItemAgent::UI(), #func,                               \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2),                                          \
                                  Q_ARG(QVariant, param3));

#define GEOPAD_QML_ITEM_INVOKE_RET(qmlItemId, func, ret)                                              \
        QMetaObject::invokeMethod(GeopadQMLAdapter::getInstance()->getGeopadQMLItem(qmlItemId), #func,  \
                                  Q_RETURN_ARG(QVariant, ret));
#define GEOPAD_QML_ITEM_LOCAL_INVOKE_RET(func, ret)                                                   \
        QMetaObject::invokeMethod(this->K3DQMLItemAgent::UI(), #func,                               \
                                  Q_RETURN_ARG(QVariant, ret));

#define GEOPAD_QML_ITEM_INVOKE_RET_I(qmlItemId, func, ret, param)                                     \
        QMetaObject::invokeMethod(GeopadQMLAdapter::getInstance()->getGeopadQMLItem(qmlItemId), #func,  \
                                  Q_RETURN_ARG(QVariant, ret),                                      \
                                  Q_ARG(QVariant, param));
#define GEOPAD_QML_ITEM_LOCAL_INVOKE_RET_I(func, ret, param)                                          \
        QMetaObject::invokeMethod(this->K3DQMLItemAgent::UI(), #func,                               \
                                  Q_RETURN_ARG(QVariant, ret),                                      \
                                  Q_ARG(QVariant, param));

#define GEOPAD_QML_ITEM_INVOKE_RET_II(qmlItemId, func, ret, param1, param2)                           \
        QMetaObject::invokeMethod(GeopadQMLAdapter::getInstance()->getGeopadQMLItem(qmlItemId), #func,  \
                                  Q_RETURN_ARG(QVariant, ret),                                      \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2));
#define GEOPAD_QML_ITEM_LOCAL_INVOKE_RET_II(func, ret, param1, param2)                                \
         QMetaObject::invokeMethod(this->K3DQMLItemAgent::UI(), #func,                              \
                                  Q_RETURN_ARG(QVariant, ret),                                      \
                                  Q_ARG(QVariant, param1),                                          \
                                  Q_ARG(QVariant, param2));

class GeopadQMLAdapter : public QObject {
    Q_OBJECT
    Q_ENUMS(GEOPAD_VOID_OPERATION)
    Q_ENUMS(GEOPAD_PARAM_OPERATION)
    Q_ENUMS(GEOPAD_DOUBLE_PARAM_OPERATION)
    Q_ENUMS(GEOPAD_TRIPLE_PARAM_OPERATION)
    Q_ENUMS(GEOPAD_QML_ITEM)
    Q_ENUMS(VROBOT_ARM_ELEMENT)
public:

    static const int OBJECT_IMAGE_WIDTH  = 80;
    static const int OBJECT_IMAGE_HEIGHT = 80;

    enum K3D_ACTION {
        K3D_ACTION_VOID,
        K3D_ACTION_TOTAL
    };

    enum K3D_QML_CONTEXT_PROPERTY {
        GEOPAD_APPLICATION,
        MAIN_WINDOW_AGENT,
        GEOPAD_QML_ADAPTER,


        GEOPAD_QML_CONTEXT_PROPERTY_TOTAL
    };

    // These names are canonical, which will be used DIRECTLY in QML:
    static const QString CONTEXT_PROPERTY[GEOPAD_QML_CONTEXT_PROPERTY_TOTAL];

    enum GEOPAD_VOID_OPERATION {
        GEOPADOP_V,
        GEOPADOP_VOID_TOTAL
    };

    enum GEOPAD_PARAM_OPERATION {
        GEOPADOP_SINGLE_PARAM,
        GEOPADOP_SINGLE_PARAM_TOTAL
    };

    enum GEOPAD_DOUBLE_PARAM_OPERATION {
        GEOPADOP_DOUBLE_PARAM,
        GEOPADOP_DOUBLE_PARAM_TOTAL
    };

    enum GEOPAD_TRIPLE_PARAM_OPERATION {
        GEOPADOP_TRIPLE_PARAM,
        GEOPADOP_TRIPLE_PARAM_TOTAL
    };
    
    // -----------------------------------
    // FUNCTION OPERATION DIALOS --
    //
    enum K3PM_OPERATION_DIALOG_EVENT {      
        GEOPAD_DIALOG_EVENT,
        GEOPAD_DIALOG_EVENT_TOTAL
    };

    enum GEOPAD_QML_ITEM {
        // DIALOGS --
        GEOPAD_QML_ITEM_I,

        /* Add more dialog id here */
        GEOPAD_QML_ITEM_TOTAL
    };
    static const QString QML_PROPERTY_ITEM[GEOPAD_QML_ITEM_TOTAL];

    //KDialog* getKsDialog(int dialogId);

    enum VROBOT_ARM_ELEMENT {
        VWHOLE_ARM,
        VBASE_JOINT,
        VJOINT20,
        VJOINT2,
        VJOINT3,

        VFINGER_1_PROX_JOINT,
        VFINGER_1_MED_JOINT,
        VFINGER_1_DIST_JOINT,

        VFINGER_2_PROX_JOINT,
        VFINGER_2_MED_JOINT,
        VFINGER_2_DIST_JOINT,

        VFINGER_3_PROX_JOINT,
        VFINGER_3_MED_JOINT,
        VFINGER_3_DIST_JOINT,

        VFINGER_TOTAL
    };

public:
    GeopadQMLAdapter();
    static QObject* qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine);
    static GeopadQMLAdapter* getInstance();
    ~GeopadQMLAdapter();

    GEOPADOP_VOID _geopadOpVoid[GEOPADOP_VOID_TOTAL];
    Q_INVOKABLE QVariant k3dRunOpVoid(int k3dOpId);

    GEOPADOP_VAR  _geopadOpVariant[GEOPADOP_SINGLE_PARAM_TOTAL];
    Q_INVOKABLE QVariant k3dRunOpParam(int k3dOpId, QVariant param);

    GEOPADOP_VAR_VAR  _geopadOpDoubleVariant[GEOPADOP_DOUBLE_PARAM_TOTAL];
    Q_INVOKABLE QVariant k3dRunOpDoubleParam(int k3dOpId, QVariant param1, QVariant param2);

    GEOPADOP_VAR_VAR _geopadOpDialogAgent;
    Q_INVOKABLE QVariant k3dRunDialogOp(int k3dDialogId, int eventType);

    void registerOperations();

    // Global Quick Image Provider
    //K3PMQuickImageProvider* imageProvider() { return &_imageProvider; }

    // QML Items --
    void openQMLItem(int qmlItemId); // Open a QML Item
    QObject* getGeopadQMLItem(int qmlItemId);

private:
    static GeopadQMLAdapter* s_instance;
    //NyolicQuickImageProvider _imageProvider;

    //QQmlProperty* _k3pmQMLItemList[K3pmQMLAdapter::K3PM_QML_ITEM_TOTAL];
};


class GeopadQMLItemInfo : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString text READ text WRITE setText NOTIFY textChanged)
    Q_PROPERTY(int     opId READ opId WRITE setOpId NOTIFY opIdChanged)
public:

    GeopadQMLItemInfo(const QString& text, int opId, 
                QObject* parent = nullptr) : QObject(parent) {
        _text = text;
        _opId = opId;
    }

    // text --
    QString text() { return _text; }

    void setText(const QString&  text) {
        if (_text != text) {
            _text = text;
            emit textChanged(_text);
        }
    }

    // opId --
    int opId()     { return _opId; }
    void setOpId(int opId) {
        if (_opId != opId) {
            _opId = opId;
            emit opIdChanged(_opId);
        }
    }

signals:
    void textChanged(QString text);
    void opIdChanged(int opId);

private:
    QString _text;
    int _opId;

    /*
        int _listIndex;
        bool _operationNeedIndex;

        int  _subOpId;
        bool _visible;
        bool _enabled;
        int  _btnType;
        QString _tooltip;
        int _tooltipPos;
        QString _textColor;
        int _textPixelSize;
        int _textPosition;
        int _textRotation;
        int _textVerticalAlignment;
        int _textHorizontalAlignment;
        QString _normalSource;
        QString _hoveredSource;
        QString _pressedSource;
        QString _functionBtnSourceNormal;
        QString _functionBtnSourceHover;

        bool _isExtended;
        int _extendedDirection;
        */
};

#endif // GEOPAD_QML_ADAPTER_H

