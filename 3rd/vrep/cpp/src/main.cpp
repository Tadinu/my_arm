#include <QApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QtQml>
#include <Qt3DQuickExtras/qt3dquickwindow.h>
#include <Qt3DQuick/QQmlAspectEngine>
#include <QQmlContext>
#include <QQmlEngine>

#include <QDebug>

#include "RbMainWindowAgent.h"
#include "VREPAdapter.h"
#include "QMLAdapter.h"

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
        qmlRegisterSingletonType(QUrl("qrc:///qml/MainGBSingletonObject.qml"), "MainGBSingletonObject", 1, 0, "MAINGB");

        // -- QML COMPONENT INIT -----------------------------------------------------------
        // 1 - Main Window --
        //
        QQuickView mainWindow;
        mainWindow.resize(1600, 800);

        // 2 - Main QML Engine --
        //
        QQmlEngine* mainQmlEngine = mainWindow.engine();
        mainQmlEngine->rootContext()->setContextProperty("_mainWindow", &mainWindow);
        mainQmlEngine->addImportPath("qrc:///qml"); // For Identified Module Import
        mainQmlEngine->addImageProvider(QMLQuickImageProvider::frontVisionSensorImageName, QMLAdapter::getInstance()->frontVisionImageProvider());
        mainQmlEngine->addImageProvider(QMLQuickImageProvider::floorVisionSensorImageName, QMLAdapter::getInstance()->groundVisionImageProvider());

        // 2.1 - LOAD MAIN QML MAIN WINDOW
        mainWindow.setSource(RB_MAIN_APP);

        // 2.2 - INITIATE MAIN WINDOW AGENT, STARTING ROBOT MANAGEMENT THREAD
        RbMainWindowAgent::getInstance(argc, argv)->setQmlCom((QObject*)mainWindow.rootObject());

        // 2.3 - SET MAIN QML CONTEXT PROPERTY (_app used by used as an property of RB_MAIN_APP QML type)
        mainQmlEngine->rootContext()->setContextProperty(QMLAdapter::CONTEXT_PROPERTY[QMLAdapter::ROBOT_APPLICATION], QCoreApplication::instance());
        mainQmlEngine->rootContext()->setContextProperty(QMLAdapter::CONTEXT_PROPERTY[QMLAdapter::MAIN_WINDOW_AGENT], RbMainWindowAgent::getInstance());
        mainQmlEngine->rootContext()->setContextProperty(QMLAdapter::CONTEXT_PROPERTY[QMLAdapter::RB_QML_ADAPTER], QMLAdapter::getInstance());

        // 2.4 - INITIALIZE QML CONTENT
        RbMainWindowAgent::getInstance()->initializeQMLContent();

        // 2.5 - REGISTER: QML ENGINE QUIT -> MAIN APP QUIT
        QObject::connect((QObject*)mainQmlEngine, SIGNAL(quit()), &app, SLOT(quit()));

        // 5 - SHOW MAIN WINDOW
        mainWindow.show();

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
