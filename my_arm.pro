QT += qml quick core widgets network

CONFIG += c++11

SOURCES += main.cpp \
    src/my_arm/GeopadMainWindowAgent.cpp \
    src/my_arm/GeopadQMLAdapter.cpp \
    src/my_arm/RobotThread.cpp \
    src/my_arm/K3DQMLItemInfo.cpp \
    src/my_arm/K3DQMLItemAgent.cpp \
    src/my_arm/K3DStateMachine.cpp \
    src/my_arm/KsGlobal.cpp \
    src/my_arm/K3DMaskedMouseArea.cpp \
    src/my_arm/RobotArmControllerMain.cpp

RESOURCES += qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

INCLUDEPATH+= ./include/my_arm \
              /opt/ros/kinetic/include
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    launch/my_arm_commander.launch \
    models/myArm.urdf \
    models/myArm2.urdf \
    worlds/robotArm.world \
    meshes/base_link.STL \
    meshes/finger_1_dist_link.STL \
    meshes/finger_1_med_liink.STL \
    meshes/finger_1_prox_link.STL \
    meshes/finger_2_dist_link.STL \
    meshes/finger_2_med_link.STL \
    meshes/finger_2_prox_link.STL \
    meshes/finger_3_dist_link.STL \
    meshes/finger_3_med_link.STL \
    package.xml \
    CMakeLists.txt \
    worlds/robotArm.world \
    launch/my_arm_world.launch \
    models/myArm2.xacro \
    models/materials.xacro \
    models/myArm2.gazebo

HEADERS += \
    include/my_arm/GeopadMainWindowAgent.h \
    include/my_arm/GeopadQMLAdapter.h \
    include/my_arm/RobotThread.h \
    include/my_arm/K3DQMLItemInfo.h \
    include/my_arm/K3DQMLItemAgent.h \
    include/my_arm/K3DStateMachine.h \
    include/my_arm/K3DMaskedMouseArea.h \
    include/my_arm/KsGlobal.h \
    include/my_arm/RobotArmControllerMain.h
