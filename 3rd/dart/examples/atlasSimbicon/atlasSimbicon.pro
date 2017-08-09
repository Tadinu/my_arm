QT += core gui xml opengl concurrent network qml quick widgets
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TARGET = atlasSimbicon
CONFIG += app
#CONFIG -= app_bundle

TEMPLATE = app

SOURCES += Controller.cpp \
    Humanoid.cpp \
    Main.cpp \
    MyWindow.cpp \
    State.cpp \
    StateMachine.cpp \
    TerminalCondition.cpp \
    ../../../../src/my_arm/KsGlobal.cpp \
    ../../../../src/my_arm/commondefines.cpp \
    ../../../../src/my_arm/RobotVoxelyzeAdapter.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

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
              \ ## BULLET
              /usr/include/bullet \
              \ ## my_arm
              $${MY_ARM_PROJ_DIR}/include        \
              $${MY_ARM_PROJ_DIR}/include/my_arm \
              $${MY_ARM_PROJ_DIR}/include/dart \
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
            -lLinearMath \
            ##-lfcl \ ## Open Flexible Collision library
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
            ## https://dartsim.github.io/install_dart_on_ubuntu.html
            ## -> The fcl version required by dart is libfcl-dev, while moveit requires libfcl0.5-dev
            ## -> THEREFORE, FOR NOW WE CANNOT USE -lfcl
            ##
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/lib \
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/build/dart/external/lodepng \
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/build/dart/external/imgui \
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/build/dart/external/odelcpsolver \
            -ldart \
            -ldart-gui \
            -ldart-planning \
            -ldart-collision-bullet \
            -ldart-utils \
            -ldart-utils-urdf \
            -ldart-external-imgui \
            -ldart-external-lodepng \
            -ldart-external-odelcpsolver
}

CONFIG(debug, debug|release) {
    message(Debug)
    LIBS +=  \ ## -L/usr/lib/x86_64-linux-gnu/ \ ## qt_version_tag error (Ubuntu default conflict with Qt5.7.1) since QtCreator automatically add this!
            -L/usr/local/lib/ \
            -lassimp \
            -lboost_system \
            -lglut \
            -lLinearMath \
            ##-lfcl \ ## Open Flexible Collision library
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
            ## https://dartsim.github.io/install_dart_on_ubuntu.html
            ## -> The fcl version required by dart is libfcl-dev, while moveit requires libfcl0.5-dev
            ## -> THEREFORE, FOR NOW WE CANNOT USE -lfcl
            ##
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/lib \
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/build/dart/external/lodepng \
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/build/dart/external/imgui \
            -L$${MY_ARM_PROJ_DIR}/3rd/dart/build/dart/external/odelcpsolver \
            -ldart \
            -ldart-gui \
            -ldart-planning \
            -ldart-collision-bullet \
            -ldart-utils \
            -ldart-utils-urdf \
            -ldart-external-imgui \
            -ldart-external-lodepng \
            -ldart-external-odelcpsolver
}

HEADERS += \
    Controller.hpp \
    Humanoid.hpp \
    MyWindow.hpp \
    State.hpp \
    StateMachine.hpp \
    TerminalCondition.hpp \
    ../../../../include/my_arm/KsGlobal.h \
    ../../../../include/my_arm/commondefines.h \
    ../../../../include/my_arm/RobotVoxelyzeAdapter.h

DISTFILES += \
    CMakeLists.txt \
    ../../data/sdf/atlas/atlas_v3_no_head.urdf \
    ../../data/sdf/atlas/atlas_v3_no_head_soft_feet.sdf \
    ../../data/sdf/atlas/atlas_v3_no_head.sdf \
    ../../data/skel/softGround.skel \
    ../../data/sdf/atlas/ground2.urdf \
    ../../data/sdf/atlas/ground.urdf
