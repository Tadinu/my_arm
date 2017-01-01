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
    models/materials.xacro \
    models/myArm2.gazebo \
    models/jaco_joint_control_vel.xacro \
    models/jaco_joint_control.xacro \
    models/jaco_robot_multi.urdf.xacro \
    models/jaco_robot.urdf.xacro \
    models/jaco.urdf.xacro \
    README.md \
    launch/jaco_gazebo.launch \
    launch/jaco_standalone_gazebo.launch \
    launch/test_two_arms.launch \
    models/jaco_robot.urdf \
    launch/jaco_arm_commander.launch \
    meshes/jaco/0_baseA.STL \
    meshes/jaco/0_baseB_limb.STL \
    meshes/jaco/1_shoulder_limb.STL \
    meshes/jaco/2_upperarm_limb.STL \
    meshes/jaco/3_forearm_limb.STL \
    meshes/jaco/4_upperwrist_limb.STL \
    meshes/jaco/5_lowerwrist_limb.STL \
    meshes/jaco/6_hand_limb.STL \
    meshes/jaco/7_index_finger.STL \
    meshes/jaco/7_pinkie_finger.STL \
    meshes/jaco/7_thumb_finger.STL \
    meshes/jaco/ring_1.STL \
    meshes/jaco/ring_2.STL \
    meshes/jaco/ring_3.STL \
    meshes/jaco/ring_4.STL \
    meshes/jaco/ring_5.STL \
    meshes/jaco/ring_6.STL \
    meshes/jaco2/0_baseA.STL \
    meshes/jaco2/0_baseB_limb.STL \
    meshes/jaco2/1_shoulder_limb.STL \
    meshes/jaco2/2_upperarm_limb.STL \
    meshes/jaco2/3_forearm_limb.STL \
    meshes/jaco2/4_upperwrist_limb.STL \
    meshes/jaco2/5_lowerwrist_limb.STL \
    meshes/jaco2/6_hand_limb.STL \
    meshes/jaco2/7_finger_mount_index.STL \
    meshes/jaco2/7_finger_mount_pinkie.STL \
    meshes/jaco2/7_finger_mount_thumb.STL \
    meshes/jaco2/8_finger_index.STL \
    meshes/jaco2/8_finger_pinkie.STL \
    meshes/jaco2/8_finger_thumb.STL \
    meshes/jaco2/9_finger_index_tip.STL \
    meshes/jaco2/9_finger_pinkie_tip.STL \
    meshes/jaco2/9_finger_thumb_tip.STL \
    meshes/jaco2/ring_1.STL \
    meshes/jaco2/ring_2.STL \
    meshes/jaco2/ring_3.STL \
    meshes/jaco2/ring_4.STL \
    meshes/jaco2/ring_5.STL \
    meshes/jaco2/ring_6.STL \
    models/myArm_gripper.urdf \
    models/myArm_brHand.urdf \
    models/myArm_brHand2.urdf \
    models/myArm_brHand.xacro

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
