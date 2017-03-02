//#include "CommonLib/commondefines.h"
#include "GeopadMainWindowAgent.h"
#include "KsGlobal.h"
//#include "commonresource.h"

#include <QtGui>
#include <QThread>
#include <QtConcurrent/QtConcurrentRun>
#include <QUrl>
#include <QMutex>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>
#include <QDateTime>
#include <QFile>
//#include <QFileDialog>

//#include <tchar.h>
#include <functional> // std::bind

//#include <QSystemTrayIcon>
//#include "guiutil.h"
//#include "fileutil.h"
//#include "CommonLib/KConvert.h"
//#include "CommonLib/KVersionComparator.h"
//#include <JlCompress.h>
//#include "GeopadUtil.h"
//#include "fileutil.h"
//#include "fvupdater.h"

//#include "RobotAgent.h"

#include <OGRE/Overlay/OgreOverlayManager.h>

#define ARRAY_SIZE(x)		(sizeof(x) / sizeof((x)[0]))

GeopadMainWindowAgent* GeopadMainWindowAgent::_instance = nullptr;

// --- CalibrationDialog ---
GeopadMainWindowAgent::GeopadMainWindowAgent(int argc, char **argv, QObject *parent)
    :QObject(parent),
     _robotThread(argc, argv)
{
    // For Common KStudio Download Manager
    //connect(&_kDownloader, SIGNAL(progressDownloading(quint8)), this, SLOT(updateDownloadProgress(quint8)));
    //connect(&_kDownloader, SIGNAL(completedDownloading(int, const QString&, quint64, const QString&, int)), this, SLOT(finalizeUpdateGeopadData(int, const QString&, quint64, const QString&, int)));
    // - CANCEL DOWNLOADING JOB: TO BE CONNECTED ON DOWNLOAD ACTION BEING TRIGGERED
    // connect(&_progressDialog, SIGNAL(canceled()), &_kDownloader, SLOT(cancelDownloading()));

    connect(&_robotThread, &RobotThread::newPose, this, &GeopadMainWindowAgent::updatePoseDisplay);
    connect(&_robotThread, &RobotThread::jointPosUpdated, this, &GeopadMainWindowAgent::updateJointPosInfo);

    _robotThread.init();
}

GeopadMainWindowAgent::~GeopadMainWindowAgent()
{
}

GeopadMainWindowAgent *GeopadMainWindowAgent::getInstance(int argc, char **argv)
{
    if (NULL == _instance) {
        _instance = new GeopadMainWindowAgent(argc, argv, nullptr);

        Q_ASSERT(_instance != NULL);
    }

    return _instance;
}

void GeopadMainWindowAgent::deleteInstance()
{
    if (NULL != _instance) {
        delete _instance;
        _instance = nullptr;
    }
}

void GeopadMainWindowAgent::setK3DQmlCom(QObject* k3dQMLCom)
{
    assert(k3dQMLCom != nullptr);
    KsGlobal::regK3DQMLCom(k3dQMLCom);
}

void GeopadMainWindowAgent::initializeQMLContent()
{
    this->initializeQMLItemAgents();
    //K3D_QML_INVOKE_I(setWindowTitle, cmmInf::geopad_AppName);
}

void GeopadMainWindowAgent::initializeQMLItemAgents()
{
}


void GeopadMainWindowAgent::qmlLog(QVariant logVariant)
{
}

void GeopadMainWindowAgent::goForward(){_robotThread.SetSpeed(0.25, 0);}
void GeopadMainWindowAgent::goBackward(){_robotThread.SetSpeed(-0.25, 0);}
void GeopadMainWindowAgent::goRight(){_robotThread.SetSpeed(0, -V_PI / 6.0);}
void GeopadMainWindowAgent::goLeft(){_robotThread.SetSpeed(0, V_PI / 6.0);}
void GeopadMainWindowAgent::halt(){ _robotThread.SetSpeed(0, 0); }

void GeopadMainWindowAgent::updatePoseDisplay(double x, double y, double theta)
{
    QString xPose, yPose, aPose;
    //xPose.setNum(x);
    //yPose.setNum(y);
    //aPose.setNum(theta);
    //
    //p_xDisplay->setText(xPose);
    //p_yDisplay->setText(yPose);
    //p_aDisplay->setText(aPose);
}//update the display.

void GeopadMainWindowAgent::updateJointPosInfo(int jointId, const QVector3D& jointPos, double jointRotAngle)
{

}

void GeopadMainWindowAgent::setRobotJointPos(int elementId, double pos)
{
     //_robotThread.rotateJoint(elementId, pos);
}

void GeopadMainWindowAgent::moveTarget(const QVector3D& distance)
{
    _robotThread.moveBall(tf::Vector3(distance.x(), distance.y(), distance.z()));
}

void GeopadMainWindowAgent::setTargetPos(const QVector3D& pos)
{
    _robotThread.setBallPos(tf::Vector3(pos.x(), pos.y(), pos.z()), true);
}

void GeopadMainWindowAgent::onRobotPostureReset()
{
    _robotThread.updateBallFollowingEndTip(); // Joint values mean angle values, Link values mean length values
}

void GeopadMainWindowAgent::createOverlayManager()
{
#if 0
    QString name("my_arm");
    Ogre::OverlayManager &overlay_manager = Ogre::OverlayManager::getSingleton();
    overlay_ = overlay_manager.create(name);

    material_ = Ogre::MaterialManager::getSingleton().create(
                name + "_OverlayMaterial",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
    );

    Ogre::OverlayContainer *overlay_panel =
        static_cast<Ogre::OverlayContainer *>( overlay_manager.createOverlayElement( "Panel", name + "_Panel" ));

    overlay_panel->setPosition(0.0, 0.0);
    overlay_panel->setDimensions(1.0, 1.0);
    overlay_panel->setMaterialName(name + "_OverlayMaterial");

    overlay_->add2D(overlay_panel);
    overlay_->show();

    // Init Qt Stuff
    QGraphicsScene* scene_ = new QGraphicsScene( this );
    QGraphics* view_ = new QGraphicsView( scene_ );
    view_->setAlignment( Qt::AlignLeft | Qt::AlignTop );
    view_->setMouseTracking(true);

    QEvent activate_event( QEvent::WindowActivate );
    QApplication::sendEvent( scene_, &activate_event );

    if ( texture_.isNull()
          || texture_->getWidth() != render_panel_->width()
          || texture_->getHeight() != render_panel_->height()) {
       if ( !texture_.isNull()) {
         Ogre::TextureManager::getSingleton().remove( name_ + "_OverlayTexture" );
         material_->getTechnique( 0 )->getPass( 0 )->removeAllTextureUnitStates();
       }
       texture_ = Ogre::TextureManager::getSingleton().createManual(
           name_ + "_OverlayTexture",
           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
           Ogre::TEX_TYPE_2D,
           (uint) render_panel_->width(),
           (uint) render_panel_->height(),
           0,
           Ogre::PF_A8R8G8B8,
           Ogre::TU_DEFAULT
       );
       material_->getTechnique( 0 )->getPass( 0 )->createTextureUnitState( name_ + "_OverlayTexture" );
       material_->getTechnique( 0 )->getPass( 0 )->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
     }

     // Setting up Overlay
     Ogre::HardwarePixelBufferSharedPtr buffer = texture_->getBuffer();
     buffer->lock( Ogre::HardwareBuffer::HBL_DISCARD );
     const Ogre::PixelBox &pixel_box = buffer->getCurrentLock();

     {
       QImage hud((uchar *) pixel_box.data, (int) pixel_box.getWidth(), (int) pixel_box.getHeight(),
                  QImage::Format_ARGB32 );
       hud.fill( 0 );

       QPainter painter( &hud );

       view_->render( &painter, QRect( QPoint( 0, 0 ), view_->size()), QRect( QPoint( 0, 0 ), view_->size()));
     }
#endif
}
