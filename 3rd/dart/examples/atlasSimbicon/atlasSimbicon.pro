QT += core
QT -= gui

CONFIG += c++11

TARGET = atlasSimbicon
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += Controller.cpp \
    Humanoid.cpp \
    Main.cpp \
    MyWindow.cpp \
    State.cpp \
    StateMachine.cpp \
    TerminalCondition.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

DEFINES += MY_ARM QT_XML_LIB QT_OPENGL_LIB USE_ZLIB_COMPRESSION USE_OPEN_GL PREC_MED ## QT_DLL // For VoxCad

MY_ARM_PROJ_DIR = /home/brhm/DUC/RobotArm/src/my_arm
INCLUDEPATH+= /usr/include \
              /usr/local/include \
              \ ## ROS
              /opt/ros/kinetic/include \
              \ ## GAZEBO
              /usr/include/gazebo-7 \
              /usr/include/gazebo-7/gazebo \
              \ ## OGRE
              /usr/include/OGRE \
              \ ## my_arm
              $${MY_ARM_PROJ_DIR}/include        \
              $${MY_ARM_PROJ_DIR}/include/my_arm \
              $${MY_ARM_PROJ_DIR}/3rd \
              \ ## Kinect
              /home/brhm/LeapSDK/include \
              \ ## ros_vox_cad
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad \
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/GeneratedFiles \
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/VoxCad \
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/Voxelyze \
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/Voxelyze/Utils \
              \ ## bullet_server
              $${MY_ARM_PROJ_DIR}/3rd/bullet_server/include \
              \ ## dart
              $${MY_ARM_PROJ_DIR}/3rd/dart \
              $${MY_ARM_PROJ_DIR}/3rd/dart/build

CONFIG(release, debug|release) {
    message(Release)
    LIBS += \ ## -L/usr/lib/x86_64-linux-gnu/ \ ## qt_version_tag error (Ubuntu default conflict with Qt5.7.1) since QtCreator automatically add this!
            -L/usr/local/lib/ \
            -lassimp \
            -lboost_system \
            -lglut \
            -L$${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/lib \
            -lVoxCad \
            -lz \
            \ ## Ros Kinetic --
            -L/opt/ros/kinetic/lib \
            -lcpp_common \
            -lxmlrpcpp \
            -lrospack \
            -lrosconsole \
            -lrosconsole_log4cxx \
            -lrosconsole_backend_interface \
            -lrosconsole_bridge \
            -lroscpp \
            \ # -lroscpp_common \
            -lroslz4 \
            -lroslib \
            -lroscpp_serialization \
            -lrostime \
            -lrosconsole \
            -lrosconsole_log4cxx \
            -lrosconsole_backend_interface \
            -linteractive_markers \
            -ltf \
            -ltf_conversions \
            -ltf2_ros \
            ## \ ## rviz --
            ## -lrviz \
            ## \ ## moveit_visual_tools --
            ## -lrviz_visual_tools  \
            \ ## dart --
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/lib \
            -ldart \
            -ldart-gui \
            -ldart-planning \
            -ldart-collision-bullet \
            -ldart-utils \
            -ldart-utils-urdf
}

CONFIG(debug, debug|release) {
    message(Debug)
    LIBS += \ ## -L/usr/lib/x86_64-linux-gnu/ \ ## qt_version_tag error (Ubuntu default conflict with Qt5.7.1) since QtCreator automatically add this!
            -L/usr/local/lib/ \
            -lassimp \
            -lboost_system \
            -lglut \
            -L$${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/lib \
            -lVoxCad \
            -lz \
            \ ## Ros Kinetic --
            -L/opt/ros/kinetic/lib \
            -lcpp_common \
            -lxmlrpcpp \
            -lrospack \
            -lrosconsole \
            -lrosconsole_log4cxx \
            -lrosconsole_backend_interface \
            -lrosconsole_bridge \
            -lroscpp \
            -lroslib \
            \ # -lroscpp_common \
            -lroslz4 \
            -lroscpp_serialization \
            -lrostime \
            -lrosconsole \
            -linteractive_markers \
            -ltf \
            -ltf_conversions \
            -ltf2_ros \
            ## \ ## rviz --
            ## -lrviz \
            ## \ ## moveit_visual_tools --
            ## -lrviz_visual_tools  \
            \ ## dart --
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/lib \
            -ldart \
            -ldart-gui \
            -ldart-planning \
            -ldart-collision-bullet \
            -ldart-utils \
            -ldart-utils-urdf
}

HEADERS += \
    Controller.hpp \
    Humanoid.hpp \
    MyWindow.hpp \
    State.hpp \
    StateMachine.hpp \
    TerminalCondition.hpp

DISTFILES += \
    CMakeLists.txt \
    ../../data/sdf/atlas/atlas_v3_no_head.urdf \
    ../../data/sdf/atlas/atlas_v3_no_head_soft_feet.sdf \
    ../../data/sdf/atlas/atlas_v3_no_head.sdf
