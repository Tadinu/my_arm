QT += core gui xml opengl concurrent network qml quick widgets
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TARGET = tutorials
TEMPLATE = app
DEFINES += MY_ARM VOXELYZE_PURE QT_XML_LIB QT_OPENGL_LIB USE_ZLIB_COMPRESSION USE_OPEN_GL PREC_MED USE_OMP ## QT_DLL // VOX_CAD For VoxCad, USE_OMP & VOXELYZE_PURE, PARDISO_5 for Voxelyze
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
              \ ## LEAP
              /home/brhm/LeapSDK/include \
              \ ## ros_vox_cad
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad \
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/GeneratedFiles \
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/VoxCad \
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/Voxelyze \
              $${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/Voxelyze/Utils \
              \ ## Voxelyze
              $${MY_ARM_PROJ_DIR}/3rd/Voxelyze/include \
              $${MY_ARM_PROJ_DIR}/3rd/Voxelyze/include\rapidjson \
              $${MY_ARM_PROJ_DIR}/3rd/Voxelyze/include\rapidjson\error \
              $${MY_ARM_PROJ_DIR}/3rd/Voxelyze/include\rapidjson\internal \
              $${MY_ARM_PROJ_DIR}/3rd/Voxelyze/include\rapidjson\msinttypes \
              \ ## bullet_server
              $${MY_ARM_PROJ_DIR}/3rd/bullet_server/include \
              \ ## dart
              $${MY_ARM_PROJ_DIR}/3rd/dart \
              $${MY_ARM_PROJ_DIR}/3rd/dart/build
MOC_DIR += .
##OBJECTS_DIR += release
UI_DIR += .
RCC_DIR += .
CONFIG(release, debug|release) {
    message(Release)
    LIBS += \ ## -L/usr/lib/x86_64-linux-gnu/ \ ## qt_version_tag error (Ubuntu default conflict with Qt5.7.1) since QtCreator automatically add this!
            -L/usr/local/lib/ \
            -lassimp \
            -lboost_system \
            -lglut \
            \ ## VOXCAD --
            -L$${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/lib \
            -lVoxCad \
            \ ## VOXELYZE --
            -L$${MY_ARM_PROJ_DIR}/3rd/Voxelyze/lib \
            -lVoxelyze \
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
            -ldart-utils-urdf \
            \ ## LEAP --
            -L/home/brhm/LeapSDK/lib/x64 \
            -lLeap \
            -lcamera_info_manager \
            -lcamera_calibration_parsers
}

CONFIG(debug, debug|release) {
    message(Debug)
    LIBS += \ ## -L/usr/lib/x86_64-linux-gnu/ \ ## qt_version_tag error (Ubuntu default conflict with Qt5.7.1) since QtCreator automatically add this!
            -L/usr/local/lib/ \
            -lassimp \
            -lboost_system \
            -lglut \
            \ ## VOXCAD --
            -L$${MY_ARM_PROJ_DIR}/3rd/ros_vox_cad/lib \
            -lVoxCad \
            \ ## VOXELYZE --
            -L$${MY_ARM_PROJ_DIR}/3rd/Voxelyze/lib \
            -lVoxelyze \
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
            -ldart-utils-urdf \
            \ ## LEAP --
            -L/home/brhm/LeapSDK/lib/x64 \
            -lLeap \
            -lcamera_info_manager \
            -lcamera_calibration_parsers
}

SOURCES += \
    tutorialCollisions-Finished.cpp \
    ../../../src/my_arm/RobotVoxelyzeAdapter.cpp \
    ../../../src/LeapMotion/hands_listener.cpp \
    ../../../src/LeapMotion/camera_listener.cpp \
    ../../../src/my_arm/RobotLeapAdapter.cpp \
    ../../../src/my_arm/KsGlobal.cpp
HEADERS += \
    ../../../include/my_arm/RobotVoxelyzeAdapter.h \
    ../../../include/LeapMotion/hands_listener.h \
    ../../../include/LeapMotion/camera_listener.h \
    ../../../include/LeapMotion/leap_msg.h \
    ../../../include/LeapMotion/leapros_msg.h \
    ../../../include/my_arm/RobotLeapAdapter.h \
    ../../../include/my_arm/KsGlobal.h


# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

DISTFILES += \
    CMakeLists.txt \
    ../data/sdf/shadow_hand/shadow_hand.sdf \
    ../data/sdf/shadow_hand/shadow_hand_full.world \
    ../data/urdf/shadow_hand/shadow_hand.urdf \
    ../data/skel/soft_cubes.skel \
    ../data/skel/soft_open_chain.skel \
    ../data/skel/softBodies.skel \
    ../data/skel/softVoxel.skel \
    ../data/skel/softVoxel_2.skel
