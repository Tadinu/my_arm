#include "GeopadQMLAdapter.h"
#include "GeopadMainWindowAgent.h"
#include "KsGlobal.h"

GeopadQMLAdapter* GeopadQMLAdapter::s_instance = nullptr;

const QString GeopadQMLAdapter::CONTEXT_PROPERTY[GeopadQMLAdapter::GEOPAD_QML_CONTEXT_PROPERTY_TOTAL] = {
    "_geopadApp",
    "_geopadMainWindow",
    "_geopadQMLAdapter"
};

const QString GeopadQMLAdapter::QML_PROPERTY_ITEM[GeopadQMLAdapter::GEOPAD_QML_ITEM_TOTAL] = {
    "_geopadcqmlItem1"
};

// [QML SINGLETON]
QObject* GeopadQMLAdapter::qmlInstance(QQmlEngine *engine, QJSEngine *scriptEngine)
{
    Q_UNUSED(engine)
    Q_UNUSED(scriptEngine)

    return GeopadQMLAdapter::getInstance();
}

// [C++ SINGLETON]
GeopadQMLAdapter* GeopadQMLAdapter::getInstance()
{
    if (!s_instance)
    { 
        s_instance = new GeopadQMLAdapter();
    }

    return s_instance;
}


GeopadQMLAdapter::GeopadQMLAdapter()
{
    if (s_instance)
    {
        delete s_instance;
    }
    s_instance = this;

    registerOperations();
}

GeopadQMLAdapter::~GeopadQMLAdapter()
{
}

void GeopadQMLAdapter::registerOperations()
{
    // K3D VOID OPERATION =======================================================================================
    memset(_geopadOpVoid, 0x00, sizeof(_geopadOpVoid));

    // ##########################################################################################################
    // ==========================================================================================================
    // K3D INT OPERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    memset(_geopadOpVariant, 0x00, sizeof(_geopadOpVariant));
    
    // ##########################################################################################################
    // ==========================================================================================================
    // K3D DOUBLE PARAM OPERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    memset(_geopadOpDoubleVariant, 0x00, sizeof(_geopadOpDoubleVariant));
    

    // ##########################################################################################################
    // ==========================================================================================================
    // K3D OPERATION DIALOG AGENT FUNCT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //_k3dOpDialogAgent = &MainDialog::onDialogEvent;
}

QVariant GeopadQMLAdapter::k3dRunOpVoid(int k3pmOpId)
{
    if (k3pmOpId >= 0 && k3pmOpId < GEOPADOP_VOID_TOTAL) {
        GEOPAD_VOID_OPERATION_INVOKE(k3pmOpId);
    }
    else return -1;
}

QVariant GeopadQMLAdapter::k3dRunOpParam(int k3pmOpId, QVariant param)
{
    if (k3pmOpId >= 0 && k3pmOpId < GEOPADOP_SINGLE_PARAM_TOTAL) {
        GEOPAD_1VARIANT_OPERATION_INVOKE(k3pmOpId, param);
    }
    else return -1;
}

QVariant GeopadQMLAdapter::k3dRunOpDoubleParam(int k3pmOpId, QVariant param1, QVariant param2)
{
    if (k3pmOpId >= 0 && k3pmOpId < GEOPADOP_DOUBLE_PARAM_TOTAL) {
        GEOPAD_2VARIANT_OPERATION_INVOKE(k3pmOpId, param1, param2);
    }
    else return -1;
}

QVariant GeopadQMLAdapter::k3dRunDialogOp(int k3dDialogId, int eventType)
{
    if (k3dDialogId >= 0 && k3dDialogId < GEOPAD_QML_ITEM_TOTAL) {
        GEOPAD_DIALOG_OPERATION_INVOKE(k3dDialogId, eventType);
    }
    else return -1;
}

QObject* GeopadQMLAdapter::getGeopadQMLItem(int qmlItemId)
{
    assert(qmlItemId >= 0 && qmlItemId < GeopadQMLAdapter::GEOPAD_QML_ITEM_TOTAL);

#if 1
    QQmlProperty prop(KsGlobal::k3dQMLCom(), GeopadQMLAdapter::QML_PROPERTY_ITEM[qmlItemId]);
    QObject * obj = qvariant_cast<QObject *>(prop.read()); //!!!
    return obj;
#else
    if (_k3pmQMLItemList[qmlItemId] != nullptr) {
        QObject * obj = qvariant_cast<QObject *>(_k3pmQMLItemList[qmlItemId]->read()); //!!!
        return obj;
    }
    else
        return nullptr;
#endif
}

void GeopadQMLAdapter::openQMLItem(int qmlItemId)
{
    assert(qmlItemId >= 0 && qmlItemId < GeopadQMLAdapter::GEOPAD_QML_ITEM_TOTAL);
    K3D_QML_INVOKE_I(openQMLItem, qmlItemId);
}
