#include "QMLAdapter.h"
#include "RbMainWindowAgent.h"
#include "RbGlobal.h"

QMLAdapter* QMLAdapter::s_instance = nullptr;

const QString QMLAdapter::CONTEXT_PROPERTY[QMLAdapter::RB_QML_CONTEXT_PROPERTY_TOTAL] = {
    "_rbApp",
    "_rbMainWindowAgent",
    "_rbQMLAdapter"
};

const QString QMLAdapter::QML_PROPERTY_ITEM[QMLAdapter::QML_ITEM_TOTAL] = {
    "_qmlItem1"
};

// [QML SINGLETON]
QObject* QMLAdapter::qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine)
{
    Q_UNUSED(engine)
    Q_UNUSED(scriptEngine)

    return QMLAdapter::getInstance();
}

// [C++ SINGLETON]
QMLAdapter* QMLAdapter::getInstance()
{
    if (!s_instance)
    { 
        s_instance = new QMLAdapter();
    }

    return s_instance;
}


QMLAdapter::QMLAdapter():
            _frontVisionImageProvider(QMLQuickImageProvider::FRONT_VISION_SENSOR_IMAGE),
            _groundVisionImageProvider(QMLQuickImageProvider::GROUND_VISION_SENSOR_IMAGE)
{
    if (s_instance)
    {
        delete s_instance;
    }
    s_instance = this;

    registerOperations();
}

QMLAdapter::~QMLAdapter()
{
}

void QMLAdapter::registerOperations()
{
    // VOID OPERATION =======================================================================================
    memset(_rbOpVoid, 0x00, sizeof(_rbOpVoid));

    // ##########################################################################################################
    // ==========================================================================================================
    // INT OPERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    memset(_rbOpVariant, 0x00, sizeof(_rbOpVariant));
    
    // ##########################################################################################################
    // ==========================================================================================================
    // DOUBLE PARAM OPERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    memset(_rbOpDoubleVariant, 0x00, sizeof(_rbOpDoubleVariant));
    

    // ##########################################################################################################
    // ==========================================================================================================
    // OPERATION DIALOG AGENT FUNCT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //_opDialogAgent = &MainDialog::onDialogEvent;
}

QVariant QMLAdapter::runOpVoid(int opId)
{
    if (opId >= 0 && opId < RB_OP_VOID_TOTAL) {
        RB_VOID_OPERATION_INVOKE(opId);
    }
    else return -1;
}

QVariant QMLAdapter::runOpParam(int opId, QVariant param)
{
    if (opId >= 0 && opId < RB_OP_SINGLE_PARAM_TOTAL) {
        RB_1VARIANT_OPERATION_INVOKE(opId, param);
    }
    else return -1;
}

QVariant QMLAdapter::runOpDoubleParam(int opId, QVariant param1, QVariant param2)
{
    if (opId >= 0 && opId < RB_OP_DOUBLE_PARAM_TOTAL) {
        RB_2VARIANT_OPERATION_INVOKE(opId, param1, param2);
    }
    else return -1;
}

QVariant QMLAdapter::runDialogOp(int dialogId, int eventType)
{
    if (dialogId >= 0 && dialogId < QML_ITEM_TOTAL) {
        RB_DIALOG_OPERATION_INVOKE(dialogId, eventType);
    }
    else return -1;
}

QObject* QMLAdapter::getRbQMLItem(int qmlItemId)
{
    assert(qmlItemId >= 0 && qmlItemId < QMLAdapter::QML_ITEM_TOTAL);

    QQmlProperty prop(RbGlobal::qmlCom(), QMLAdapter::QML_PROPERTY_ITEM[qmlItemId]);
    QObject * obj = qvariant_cast<QObject *>(prop.read()); //!!!
    return obj;
}

void QMLAdapter::openQMLItem(int qmlItemId)
{
    assert(qmlItemId >= 0 && qmlItemId < QMLAdapter::QML_ITEM_TOTAL);
    RB_QML_INVOKE_I(openQMLItem, qmlItemId);
}
