#include <QApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QtQml>
#include <QtQml/QQmlEngine>
#include <QDebug>

#include "RbRobotControllerMain.h"
#include "RbMainWindowAgent.h"
#include "VREPAdapter.h"
#include "QMLAdapter.h"
#include "QMLItemInfo.h"

int main(int argc, char *argv[])
{
    ////////////////////////////////////////////////////////////////////////////////////
    // -- VREP -------------------------------------------------------------------------
    VREP_INSTANCE()->initializeVREP();

    try
    {
        QApplication app(argc, argv);
        QCoreApplication::setOrganizationName("BRHM");
        QCoreApplication::setOrganizationDomain("BRHM");
        QCoreApplication::setApplicationName("RobotSensors");

        ////////////////////////////////////////////////////////////////////////////////////
        // -- QML -------------------------------------------------------------------------
        //
        // NOTE: This is important, adding the path for the executable to look for plugins
        // In application folder, prepare a folder named 'plugins', which contains required plugins.
        // (It's recommended to copy from Qt installation folder)
        //app.addLibraryPath(app.applicationDirPath() + "/" + cmmInf::ks_PluginFolderName);
        //app.setWindowIcon(QIcon(":/res/robot.ico"));

        //app.setApplicationName("Robot");

        // Register 'QMLAdapter' to be used in QML:
        qmlRegisterType<QMLAdapter>("com.rb.qmladapter", 1, 0, "QMLAdapter");
        //qmlRegisterType<QMLMaskedMouseArea>("com.rb.qmladapter", 1, 0, "QMLMaskedMouseArea");
        //qmlRegisterType<QMLItemInfo>("com.rb.qmladapter", 1, 0, "QMLItemInfo");

        qmlRegisterSingletonType(QUrl("qrc:///qml/MainGBSingletonObject.qml"), "MainGBSingletonObject", 1, 0, "MAINGB");

        // -- QML COMPONENT INIT -----------------------------------------------------------
#if 0
        foreach(QScreen * screen, QGuiApplication::screens())
            screen->setOrientationUpdateMask(Qt::LandscapeOrientation | Qt::PortraitOrientation |
            Qt::InvertedLandscapeOrientation | Qt::InvertedPortraitOrientation);
#endif

        QQmlEngine mainQmlEngine;
        QQmlComponent mainQmlComponent(&mainQmlEngine);
        // The object is expected to be a Window, thus typed as one
        QQuickWindow::setDefaultAlphaBuffer(true);
        //mainQmlEngine.addImportPath("qrc:///qml/qmlBase"); // For Identified Module Import
        mainQmlEngine.addImportPath("qrc:///qml"); // For Identified Module Import

        mainQmlComponent.loadUrl(RB_APPLICATION);

        if (!mainQmlComponent.isReady()) {
            qWarning() << mainQmlComponent.errorString();
            return -1;
        }

        // 1.3 - SET MAIN CONTEXT PROPERTY (_app used by used as an property of RB_APPLICATION QML type)

        mainQmlEngine.rootContext()->setContextProperty(QMLAdapter::CONTEXT_PROPERTY[QMLAdapter::ROBOT_APPLICATION], QCoreApplication::instance());
        mainQmlEngine.rootContext()->setContextProperty(QMLAdapter::CONTEXT_PROPERTY[QMLAdapter::MAIN_WINDOW_AGENT],  RbMainWindowAgent::getInstance());
        mainQmlEngine.rootContext()->setContextProperty(QMLAdapter::CONTEXT_PROPERTY[QMLAdapter::RB_QML_ADAPTER], QMLAdapter::getInstance());
        //_mainQuickWidget->rootContext()->setContextObject(MainWindow::getInstance());

        // 1.4 - Add Image Provider:
        //engine.addImageProvider(QLatin1String("objectImages"), K3DQMLAdapter::getInstance()->imageProvider());

        // 2 - SET MainWindow's RB_APPLICATION object
        QObject *topLevel = mainQmlComponent.create(mainQmlEngine.rootContext());
        assert(topLevel);
        RbMainWindowAgent::getInstance(argc, argv)->setQmlCom(topLevel);

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
        RbMainWindowAgent::getInstance()->initializeQMLContent();

        QObject::connect((QObject*)&mainQmlEngine, SIGNAL(quit()), &app, SLOT(quit()));

        // 4 - SHOW MAIN WINDOW
        QSurfaceFormat surfaceFormat = window->requestedFormat();
        window->setFormat(surfaceFormat);
        window->show();

        ////////////////////////////////////////////////////////////////////////////////////
        // -- APPLICATION EXEC ------------------------------------------------------------
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
