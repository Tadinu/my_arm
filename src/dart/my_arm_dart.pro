QT += core gui xml opengl concurrent network qml quick widgets
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TARGET = my_arm_dart
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
              \ ## BULLET
              /usr/include/bullet \
              \ ## my_arm
              $${MY_ARM_PROJ_DIR}/include        \
              $${MY_ARM_PROJ_DIR}/include/my_arm \
              $${MY_ARM_PROJ_DIR}/include/dart \
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
              $${MY_ARM_PROJ_DIR}/3rd/dart/build \
              \ ## ReflexxesTypeII
              $${MY_ARM_PROJ_DIR}/3rd/ReflexxesTypeII/include
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
            -leigen_conversions \ # ros-eigen conversions
            -linteractive_markers \
            -ltf \
            -ltf_conversions \
            -ltf2_ros \
            \ ## orocos_kdl
            -lorocos-kdl \
            \ ## \ ## rviz --
            \ ## -lrviz \
            \ ## \ ## moveit_visual_tools --
            \ ## -lrviz_visual_tools  \
            \ ## dart --
            \ ## https://dartsim.github.io/install_dart_on_ubuntu.html
            \ ## -> The fcl version required by dart is libfcl-dev, while moveit requires libfcl0.5-dev
            \ ## -> THEREFORE, FOR NOW WE CANNOT USE -lfcl
            \ ##
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
            -ldart-external-odelcpsolver \
            \ ## LEAP --
            -L/home/brhm/LeapSDK/lib/x64 \
            -lLeap \
            -lcamera_info_manager \
            -lcamera_calibration_parsers \
            \ ## ReflexxesTypeII --
            -L$${MY_ARM_PROJ_DIR}/3rd/ReflexxesTypeII/Linux/x64/release/lib/shared \
            -lReflexxesTypeII
}

CONFIG(debug, debug|release) {
    message(Debug)
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
            -lroslib \
            -lroslz4 \
            -lroscpp_serialization \
            -lrostime \
            -lrosconsole \
            -lrosconsole_log4cxx \
            -lrosconsole_backend_interface \
            -leigen_conversions \ # ros-eigen conversions
            -linteractive_markers \
            -ltf \
            -ltf_conversions \
            -ltf2_ros \
            \ ## orocos_kdl
            -lorocos-kdl \
            \ ## \ ## rviz --
            \ ## -lrviz \
            \ ## \ ## moveit_visual_tools --
            \ ## -lrviz_visual_tools  \
            \ ## dart --
            \ ## https://dartsim.github.io/install_dart_on_ubuntu.html
            \ ## -> The fcl version required by dart is libfcl-dev, while moveit requires libfcl0.5-dev
            \ ## -> THEREFORE, FOR NOW WE CANNOT USE -lfcl
            \ ##
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
            -ldart-external-odelcpsolver \
            \ ## LEAP --
            -L/home/brhm/LeapSDK/lib/x64 \
            -lLeap \
            -lcamera_info_manager \
            -lcamera_calibration_parsers \
            \ ## ReflexxesTypeII --
            -L$${MY_ARM_PROJ_DIR}/3rd/ReflexxesTypeII/Linux/x64/release/lib/shared \
            -lReflexxesTypeII
}

SOURCES += \
    main.cpp \
    MainWindow.cpp \
    DartUtils.cpp \
    ../my_arm/RobotVoxelyzeAdapter.cpp \
    ../LeapMotion/hands_listener.cpp \
    ../LeapMotion/camera_listener.cpp \
    ../my_arm/RobotLeapAdapter.cpp \
    ../my_arm/KsGlobal.cpp \
    ../my_arm/commondefines.cpp \
    DartRobotController.cpp
HEADERS += \
    ../../include/dart/MainWindow.h \
    ../../include/dart/DartUtils.h \
    ../../include/my_arm/RobotVoxelyzeAdapter.h \
    ../../include/LeapMotion/hands_listener.h \
    ../../include/LeapMotion/camera_listener.h \
    ../../include/LeapMotion/leap_msg.h \
    ../../include/LeapMotion/leapros_msg.h \
    ../../include/my_arm/RobotLeapAdapter.h \
    ../../include/my_arm/KsGlobal.h \
    ../../include/my_arm/commondefines.h \
    ../../include/dart/DartRobotController.hpp

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
    ../../3rd/dart/data/sdf/shadow_hand/shadow_hand.sdf \
    ../../3rd/dart/data/sdf/shadow_hand/shadow_hand_full.world \
    ../../3rd/dart/data/urdf/shadow_hand/shadow_hand.urdf \
    ../../3rd/dart/data/skel/soft_cubes.skel \
    ../../3rd/dart/data/skel/soft_open_chain.skel \
    ../../3rd/dart/data/skel/softBodies.skel \
    ../../3rd/dart/data/skel/softVoxel.skel \
    ../../3rd/dart/data/skel/softVoxel.skel \
    ../../3rd/dart/data/skel/mesh_collision.skel \
    ../../3rd/dart/data/skel/spheres.skel \
    ../../3rd/dart/data/urdf/KR5/ground.urdf \
    ../../3rd/dart/data/urdf/KR5/KR5_R650.urdf \
    ../../3rd/dart/data/urdf/KR5/KR5 sixx R650.urdf \
    ../../3rd/dart/data/skel/empty.skel \
    ../../3rd/dart/data/skel/ground.skel
