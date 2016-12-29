#include <QApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QtQml>
#include <QtQml/QQmlEngine>
#include <QDebug>

#include "RobotArmControllerMain.h"
#include "GeopadMainWindowAgent.h"
#include "GeopadQMLAdapter.h"
// #include "FileUtil.h"
#include "K3DMaskedMouseArea.h"
#include "K3DQMLItemInfo.h"

//#define USING_QMLAPPLICATION_ENGINE
#define USING_QQMLCOMPONENT
//#define USING_QQUICKVIEW
//#define USING_QQUICKWIDGET

int main(int argc, char *argv[])
{
    try
    {
        QApplication app(argc, argv);

        // NOTE: This is important, adding the path for the executable to look for plugins
        // In application folder, prepare a folder named 'plugins', which contains required plugins.
        // (It's recommended to copy from Qt installation folder)
        //app.addLibraryPath(app.applicationDirPath() + "/" + cmmInf::ks_PluginFolderName);
        //app.setWindowIcon(QIcon(":/res/Geopad.ico"));

        //app.setOrganizationName(cmmInf::kev_CompanyName);
        //app.setOrganizationDomain(cmmInf::kev_CompanyDomain);
        //app.setApplicationName(cmmInf::geopad_AppName);


        //qmlRegisterSingletonType("K3DS.widget", 1, 0, "K3DGLWidget",);
        //qmlRegisterRevision<MainGLWidget,1>("K3DGLWidget", 1, 1);

        // Register 'K3DQMLAdapter' to be used in QML:
        qmlRegisterType<GeopadQMLAdapter>("com.k3d.qmladapter", 1, 0, "K3DQMLAdapter");
        //qmlRegisterType<K3DMaskedMouseArea>("com.k3d.qmladapter", 1, 0, "K3DMaskedMouseArea");
        //qmlRegisterType<K3DQMLItemInfo>("com.k3d.qmladapter", 1, 0, "K3DQMLItemInfo");

        // K3DObjectTool
        //qmlRegisterSingletonType<K3DObjectTool>("com.k3d.k3dobjecttool", 1, 0, "K3DObjectTool", K3DObjectTool::singletontype_provider);
        // K3DGBObject.qml- Note the type name (MainGBSingletonObject) is kept the same in K3DStudio & K3PM:
        qmlRegisterSingletonType(QUrl("qrc:///qml/MainGBSingletonObject.qml"), "MainGBSingletonObject", 1, 0, "MAINGB");
        // Nyolic.qml
        //qmlRegisterSingletonType(VAPPLICATION, "GEOPAD", 1, 0, "GeopadMainWindow");

        // -- QML COMPONENT INIT -----------------------------------------------------------
#ifdef USING_QMLAPPLICATION_ENGINE
        QQmlApplicationEngine mainQmlEngine(VAPPLICATION);
        mainQmlEngine.addImportPath("qrc:///../K3DStudio/qml"); // For Identified Module Import

        // 1.3 - SET MAIN CONTEXT PROPERTY (_app used by used as an property of VAPPLICATION QML type)

        mainQmlEngine.rootContext()->setContextProperty(GeopadQMLAdapter::CONTEXT_PROPERTY[GeopadQMLAdapter::OMNY_APPLICATION], QCoreApplication::instance());
        mainQmlEngine.rootContext()->setContextProperty(GeopadQMLAdapter::CONTEXT_PROPERTY[GeopadQMLAdapter::MAIN_WINDOW_AGENT], MainWindowOmnyAgent::getInstance());
        mainQmlEngine.rootContext()->setContextProperty(GeopadQMLAdapter::CONTEXT_PROPERTY[GeopadQMLAdapter::OMNY_QML_ADAPTER], GeopadQMLAdapter::getInstance());
        //_mainQuickWidget->rootContext()->setContextObject(MainWindow::getInstance());

        // 1.4 - Add Image Provider:
        //engine.addImageProvider(QLatin1String("objectImages"), K3DQMLAdapter::getInstance()->imageProvider());

        // 2 - SET MainWindow's VAPPLICATION object
        MainWindowOmnyAgent::getInstance()->setK3DQmlCom(mainQmlEngine.rootObjects().at(0));

        // 3 - INITIALIZE MainWindow's QML content
        MainWindowOmnyAgent::getInstance()->initializeQMLContent();

        // ALREADY DONE IN QQmlApplicationEngine
        // QObject::connect(engine.rootObjects().at(0), SIGNAL(quit()), &app, SLOT(quit()));

#elif defined USING_QQMLCOMPONENT
        //// Using QQmlComponent
#if 0
        foreach(QScreen * screen, QGuiApplication::screens())
            screen->setOrientationUpdateMask(Qt::LandscapeOrientation | Qt::PortraitOrientation |
            Qt::InvertedLandscapeOrientation | Qt::InvertedPortraitOrientation);
#endif

        QQmlEngine mainQmlEngine;
        QQmlComponent mainQmlComponent(&mainQmlEngine);
        // The object is expected to be a Window, thus typed as one
        QQuickWindow::setDefaultAlphaBuffer(true);
        mainQmlEngine.addImportPath("qrc:///qml/k3dBase"); // For Identified Module Import
        mainQmlEngine.addImportPath("qrc:///qml"); // For Identified Module Import

        mainQmlComponent.loadUrl(VAPPLICATION);

        if (!mainQmlComponent.isReady()) {
            qWarning() << mainQmlComponent.errorString();
            return -1;
        }

        // 1.3 - SET MAIN CONTEXT PROPERTY (_app used by used as an property of VAPPLICATION QML type)

        mainQmlEngine.rootContext()->setContextProperty(GeopadQMLAdapter::CONTEXT_PROPERTY[GeopadQMLAdapter::GEOPAD_APPLICATION], QCoreApplication::instance());
        mainQmlEngine.rootContext()->setContextProperty(GeopadQMLAdapter::CONTEXT_PROPERTY[GeopadQMLAdapter::MAIN_WINDOW_AGENT],  GeopadMainWindowAgent::getInstance());
        mainQmlEngine.rootContext()->setContextProperty(GeopadQMLAdapter::CONTEXT_PROPERTY[GeopadQMLAdapter::GEOPAD_QML_ADAPTER], GeopadQMLAdapter::getInstance());
        //_mainQuickWidget->rootContext()->setContextObject(MainWindow::getInstance());

        // 1.4 - Add Image Provider:
        //engine.addImageProvider(QLatin1String("objectImages"), K3DQMLAdapter::getInstance()->imageProvider());

        // 2 - SET MainWindow's VAPPLICATION object
        QObject *topLevel = mainQmlComponent.create(mainQmlEngine.rootContext());
        assert(topLevel);
        GeopadMainWindowAgent::getInstance(argc, argv)->setK3DQmlCom(topLevel);

        // 3 - INITIALIZE MainWindow's QML content
        QQuickWindow *window = qobject_cast<QQuickWindow *>(topLevel);
#if 1
        window->resize(800, 500);
        window->setPosition(400, 250);
#else
        QWidget* currentScreen = QApplication::desktop()->screen();
        window->resize(currentScreen->height() / 1.478, currentScreen->height() / 2.5);
        window->setX((currentScreen->width() - window->width()) / 2);
        window->setY((currentScreen->height() - window->height()) / 2);
#endif
        GeopadMainWindowAgent::getInstance()->initializeQMLContent();

        QObject::connect((QObject*)&mainQmlEngine, SIGNAL(quit()), &app, SLOT(quit()));

        // 4 - SHOW MAIN WINDOW
        QSurfaceFormat surfaceFormat = window->requestedFormat();
        window->setFormat(surfaceFormat);
        window->show();
#if 0
        QObject::connect(window, SIGNAL(screenChanged(QScreen*)),
            MainWindowGeopadAgent::getInstance(), SLOT(onCurrentScreenChanged(QScreen*)));
#endif
        /*
        // THIS CONNECTION MUST BE MADE AFTER _centralQuickContainer IS SHOWN (AS QT SUPPORT GUY SAID)
        // https://account.qt.io/support/request/INC-880000
        QWindow * ptr = _centralQuickContainer->window()->windowHandle();
        connect(_centralQuickContainer->window()->windowHandle(), SIGNAL(screenChanged(QScreen*)),
        this, SLOT(onCurrentScreenChanged(QScreen*)));
        */
#elif defined USING_QQUICKVIEW
        // Using QQuickView
        QQuickView quickView;
        quickView.setSource(VAPPLICATION);
        quickView.engine()->addImportPath("qrc:///../K3DStudio/qml"); // For Identified Module Import
        quickView.show();
        QObject::connect((QObject*)quickView.rootObject(), SIGNAL(quit()), &app, SLOT(quit()));
#elif defined USING_QQUICKWIDGET
        // Using QQuickView
        K3DQuickWidget *mainQuickWidget = new K3DQuickWidget();
        mainQuickWidget->setResizeMode(QQuickWidget::SizeViewToRootObject); // The view resizes with the root item in the QML
        //_mainQuickWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);
        // The view will automatically resize the root item to the size of the view.

        //mainQuickWidget->setFormat(surfaceFormat);
        //mainQuickWidget->setClearBeforeRendering(true);
        //mainQuickWidget->setWindowFlags(Qt::FramelessWindowHint);
        mainQuickWidget->setClearColor(Qt::transparent);
        mainQuickWidget->setAttribute(Qt::WA_TranslucentBackground);
        mainQuickWidget->setAttribute(Qt::WA_AlwaysStackOnTop, true);

        mainQuickWidget->resize(QApplication::desktop()->screenGeometry(mainQuickWidget).width() / 2,
            QApplication::desktop()->screenGeometry(mainQuickWidget).height() / 2);
        mainQuickWidget->window()->update();
        // THIS DOES NOT WORK:
        {
            //        QPalette semiTransparent(QColor(255,0,0,128));
            //        semiTransparent.setBrush(QPalette::Text, Qt::white);
            //        semiTransparent.setBrush(QPalette::WindowText, Qt::white);

            //        _mainQuickWidget->setPalette(semiTransparent);
        }

        //mainQuickWidget->move(0, 0);

        // 1.1 - SET MAIN QML IMPORT PATH (For QML PLUGINS WRITTEN IN C++):
        mainQuickWidget->engine()->addImportPath(qApp->applicationDirPath() + "/nyoqmlplugins");
        //_mainQuickWidget->engine()->addPluginPath(qApp->applicationDirPath() + "/k3dqmlplugins");
        mainQuickWidget->engine()->addImportPath("qrc:///../K3DStudio/qml"); // For Identified Module Import

        // 1.2 - SET MAIN QML SOURCE:
        mainQuickWidget->setSource(VAPPLICATION);

        // 1.3 - SET MAIN CONTEXT PROPERTY (_app used by used as an property of VAPPLICATION QML type)
        //this->setQMLGlobalContextProperty(K3DQMLAdapter::K3D_APPLICATION, QCoreApplication::instance());
        //this->setQMLGlobalContextProperty(K3DQMLAdapter::MAIN_WINDOW, MainWindow::getInstance());
        //this->setQMLGlobalContextProperty(K3DQMLAdapter::MAIN_QUICK_CONTAINER, MainWindow::getInstance()->getMainQuickContainer());
        //this->setQMLGlobalContextProperty(K3DQMLAdapter::QML_ADAPTER, K3DQMLAdapter::getInstance());

        //_mainQuickWidget->rootContext()->setContextObject(MainDialog::getInstance());

        // 1.4 - Add Image Provider:
        //_mainQuickWidget->engine()->addImageProvider(QLatin1String("objectImages"), K3DQMLAdapter::getInstance()->imageProvider());

        // 2 - SET MainWindow's VAPPLICATION object
        MainWindowOmnyAgent::getInstance()->setK3DQmlCom((QObject*)mainQuickWidget->rootObject());
        //K3D_QML_INVOKE_II(resize, mainQuickWidget->width(), mainQuickWidget->height());
        K3D_QML_INVOKE_II(resize, 200, 200);

        // 3 - INITIALIZE MainWindow's QML content
        MainWindowOmnyAgent::getInstance()->initializeQMLContent();

        QObject::connect((QObject*)mainQuickWidget->rootObject(), SIGNAL(quit()), &app, SLOT(quit()));
#endif

        ////////////////////////////////////////////////////////////////////////////////////
        // -- APPLICATION EXEC -----------------------------------------------------
        return app.exec();
        ////////////////////////////////////////////////////////////////////////////////////

        // NOTE: THE FOLLOWING CODES IS RECOMMENDED TO BE PUT IN A SLOT THAT IS CONNECTED TO
        // QCoreApplication::aboutToQuit () [signal]
        // Reference:
        // - http://qt-project.org/doc/qt-4.8/qcoreapplication.html#exec
        // - MainDialog::finalizingMainApp() [slot]
    }
    
    catch (std::exception* e)
    {
    }
}
