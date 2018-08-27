QT += 3dcore 3drender 3dinput 3dquick 3dquickextras qml quick core gui xml network opengl concurrent
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotSensors
TEMPLATE = app

CONFIG += c++11
CONFIG += console
CONFIG -= app_bundle

# http://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm#c
DEFINES -= UNICODE
DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255
DEFINES += QT_COMPIL

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-stringsQMLItemInfo

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}


win32 {
    LIBS += -lwinmm
    LIBS += -lWs2_32
}

macx {
}

unix:!macx {
    LIBS += -lrt
}

INCLUDEPATH+= /usr/include \
              /usr/local/include \
              ./include        \
              \ ## V-REP
              /home/brhm/V-REP/programming \
              /home/brhm/V-REP/programming/remoteApi

SOURCES += \
    src/extApi.c \
    src/extApiPlatform.c \
    src/commondefines.cpp \
    src/QMLAdapter.cpp \
    src/QMLItemAgent.cpp \
    src/RbGlobal.cpp \
    src/RbMainWindowAgent.cpp \
    src/RbRobotAgent.cpp \
    src/RbRobotSensorAdapter.cpp \
    src/RbStateMachine.cpp \
    src/VREPAdapter.cpp \
    src/RbSensorAgent.cpp \
    src/main.cpp \
    src/RbRobotManager.cpp \
    src/QMLQuickImageProvider.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS NON_MATLAB_PARSING

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    include/extApi.h \
    include/extApiPlatform.h \
    include/v_repConst.h \
    include/commondefines.h \
    include/RbGlobal.h \
    include/QMLAdapter.h \
    include/QMLItemAgent.h \
    include/RbMainWindowAgent.h \
    include/RbRobotAgent.h \
    include/RbRobotSensorAdapter.h \
    include/RbStateMachine.h \
    include/RbGlobal.h \
    include/VREPAdapter.h \
    include/RbSensorAgent.h \
    include/RbRobotManager.h \
    include/QMLQuickImageProvider.h

DISTFILES += \
    qml/RobotPanel.qml \
    javascript/qmlcommonresource.js \
    javascript/qmlcommonutil.js \
    res/images/joystick_background.png \
    res/images/joystick_thumb.png \
    res/images/joypad.svg

RESOURCES += \
    qml.qrc \
    obj.qrc
