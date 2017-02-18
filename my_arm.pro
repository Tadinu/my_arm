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
    src/my_arm/RobotArmControllerMain.cpp \
    src/my_arm/rviz/VMarker.cpp \
    src/LeapMotion/hands_listener.cpp \
    src/LeapMotion/camera_listener.cpp \
    src/my_arm/RobotLeapAdapter.cpp \
    src/kinect/hand_interaction/analyze_hands.cpp \
    src/kinect/hand_interaction/detect_hands_wskel.cpp \
    src/kinect/hand_interaction/detect_hands.cpp

RESOURCES += qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

INCLUDEPATH+= ./include        \
              ./include/my_arm \
              /opt/ros/kinetic/include \
              /home/brhm/LeapSDK/include

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
    models/myArm_brHand.xacro \
    msg/LeapMotion/leapros.msg \
    msg/LeapMotion/leap.msg \
    scripts/LeapMotion/leap_interface.pyc \
    scripts/LeapMotion/Leap.pyc \
    scripts/LeapMotion/LeapPython.so \
    scripts/LeapMotion/libLeap.so \
    scripts/LeapMotion/leap_interface.py \
    scripts/LeapMotion/Leap.py \
    scripts/LeapMotion/sender.py \
    scripts/LeapMotion/skeleton_sender.py \
    scripts/LeapMotion/subscriber.py \
    config/LeapMotion/leap_cal_right.yml \
    config/LeapMotion/leap_cal_left.yml \
    test/LeapMotion/test_leap_motion.test \
    test/LeapMotion/test_sender.test \
    launch/LeapMotion/sensor_sender.launch \
    launch/LeapMotion/leap_camera.launch \
    launch/LeapMotion/leap_stereo.launch \
    launch/kinect/hand_interaction/finger_detector.launch \
    launch/kinect/hand_interaction/hand_detector.launch \
    config/kinect/hand_interaction/fingerdetection.vcg \
    config/kinect/hand_interaction/handdetection.vcg \
    models/myArm_softHand.xacro \
    models/pisa_iit_soft_hand/materials.urdf.xacro \
    models/pisa_iit_soft_hand/soft_hand.gazebo.xacro \
    models/pisa_iit_soft_hand/soft_hand.inertia.xacro \
    models/pisa_iit_soft_hand/soft_hand.transmission.xacro \
    models/pisa_iit_soft_hand/soft_hand.urdf.xacro \
    models/pisa_iit_soft_hand/accesories/clamp.urdf.xacro \
    models/pisa_iit_soft_hand/accesories/kuka_coupler.urdf.xacro \
    models/pisa_iit_soft_hand/accesories/softhand_base.urdf.xacro \
    meshes/pisa_iit_soft_hand/fingertip.stl \
    meshes/pisa_iit_soft_hand/knuckle.stl \
    meshes/pisa_iit_soft_hand/palm_left.stl \
    meshes/pisa_iit_soft_hand/palm_right.stl \
    meshes/pisa_iit_soft_hand/phalanx.stl \
    meshes/pisa_iit_soft_hand/softhand_base_left.stl \
    meshes/pisa_iit_soft_hand/thumb_knuckle_left.stl \
    meshes/pisa_iit_soft_hand/thumb_knuckle_right.stl \
    meshes/pisa_iit_soft_hand/fingertip_collision.stl \
    meshes/pisa_iit_soft_hand/knuckle_collision.stl \
    meshes/pisa_iit_soft_hand/palm_left_collision.stl \
    meshes/pisa_iit_soft_hand/palm_right_collision.stl \
    meshes/pisa_iit_soft_hand/phalanx_collision.stl \
    meshes/pisa_iit_soft_hand/thumb_knuckle_left_collision.stl \
    meshes/pisa_iit_soft_hand/thumb_knuckle_right_collision.stl \
    meshes/pisa_iit_soft_hand/accesories/clamp.stl \
    meshes/pisa_iit_soft_hand/accesories/kuka_coupler.stl \
    meshes/pisa_iit_soft_hand/accesories/softhand_base_left.stl \
    meshes/pisa_iit_soft_hand/accesories/softhand_base_right.stl \
    launch/my_arm_pisa_iit_soft_hand.launch \
    models/mySoftHand.xacro \
    launch/my_soft_hand.launch \
    launch/my_shadow_robot_hand.launch \
    models/shadow_hand/arm/base/shadowarm_base.urdf.xacro \
    models/shadow_hand/arm/hand_support/shadowarm_handsupport_motor.transmission.xacro \
    models/shadow_hand/arm/hand_support/shadowarm_handsupport_muscle.transmission.xacro \
    models/shadow_hand/arm/hand_support/shadowarm_handsupport_motor.urdf.xacro \
    models/shadow_hand/arm/hand_support/shadowarm_handsupport_muscle.urdf.xacro \
    models/shadow_hand/arm/lower_arm/shadowarm_lowerarm.transmission.xacro \
    models/shadow_hand/arm/lower_arm/shadowarm_lowerarm.urdf.xacro \
    models/shadow_hand/arm/trunk/shadowarm_trunk.transmission.xacro \
    models/shadow_hand/arm/trunk/shadowarm_trunk.urdf.xacro \
    models/shadow_hand/arm/upper_arm/shadowarm_upperarm.transmission.xacro \
    models/shadow_hand/arm/upper_arm/shadowarm_upperarm.urdf.xacro \
    models/shadow_hand/arm/full_arm_muscle.urdf.xacro \
    models/shadow_hand/arm/full_arm_motor.urdf.xacro \
    models/shadow_hand/arm/full_arm_motor_with_kinect.urdf.xacro \
    models/shadow_hand/hand/finger/biotac/biotac.urdf.xacro \
    models/shadow_hand/hand/finger/distal/distal.gazebo.xacro \
    models/shadow_hand/hand/finger/distal/distal.transmission.xacro \
    models/shadow_hand/hand/finger/distal/distal.urdf.xacro \
    models/shadow_hand/hand/finger/knuckle/knuckle.gazebo.xacro \
    models/shadow_hand/hand/finger/knuckle/knuckle.transmission.xacro \
    models/shadow_hand/hand/finger/knuckle/knuckle.urdf.xacro \
    models/shadow_hand/hand/finger/lfmetacarpal/lfmetacarpal.gazebo.xacro \
    models/shadow_hand/hand/finger/lfmetacarpal/lfmetacarpal.transmission.xacro \
    models/shadow_hand/hand/finger/lfmetacarpal/lfmetacarpal.urdf.xacro \
    models/shadow_hand/hand/finger/middle/middle.gazebo.xacro \
    models/shadow_hand/hand/finger/middle/middle.transmission.xacro \
    models/shadow_hand/hand/finger/middle/middle.urdf.xacro \
    models/shadow_hand/hand/finger/proximal/proximal.gazebo.xacro \
    models/shadow_hand/hand/finger/proximal/proximal.transmission.xacro \
    models/shadow_hand/hand/finger/proximal/proximal.urdf.xacro \
    models/shadow_hand/hand/finger/fingers.urdf.xacro \
    models/shadow_hand/hand/forearm/forearm.urdf.xacro \
    models/shadow_hand/hand/forearm/forearm_lite.urdf.xacro \
    models/shadow_hand/hand/palm/palm.gazebo.xacro \
    models/shadow_hand/hand/palm/palm.transmission.xacro \
    models/shadow_hand/hand/palm/palm.urdf.xacro \
    models/shadow_hand/hand/palm/palm_lite.urdf.xacro \
    models/shadow_hand/hand/thumb/thhub.gazebo.xacro \
    models/shadow_hand/hand/thumb/thbase.gazebo.xacro \
    models/shadow_hand/hand/thumb/thmiddle.gazebo.xacro \
    models/shadow_hand/hand/thumb/thproximal.gazebo.xacro \
    models/shadow_hand/hand/thumb/thdistal.gazebo.xacro \
    models/shadow_hand/hand/thumb/thhub.transmission.xacro \
    models/shadow_hand/hand/thumb/thbase.transmission.xacro \
    models/shadow_hand/hand/thumb/thdistal.transmission.xacro \
    models/shadow_hand/hand/thumb/thmiddle.transmission.xacro \
    models/shadow_hand/hand/thumb/thproximal.transmission.xacro \
    models/shadow_hand/hand/thumb/thumb.urdf.xacro \
    models/shadow_hand/hand/thumb/thbase.urdf.xacro \
    models/shadow_hand/hand/thumb/thproximal.urdf.xacro \
    models/shadow_hand/hand/thumb/thhub.urdf.xacro \
    models/shadow_hand/hand/thumb/thmiddle.urdf.xacro \
    models/shadow_hand/hand/thumb/thdistal.urdf.xacro \
    models/shadow_hand/hand/wrist/wrist.transmission.xacro \
    models/shadow_hand/hand/wrist/wrist.urdf.xacro \
    models/shadow_hand/hand/full_hand.urdf.xacro \
    models/shadow_hand/hand/one_finger_unit.urdf.xacro \
    models/shadow_hand/hand/three_fingers_hand.urdf.xacro \
    models/shadow_hand/hand/hand_E_no_mf_rf.urdf.xacro \
    models/shadow_hand/hand/hand_lite.urdf.xacro \
    models/shadow_hand/hand/three_biotac_fingers_hand.urdf.xacro \
    models/shadow_hand/hand/th_ff_rf_ellipsoid_hand.urdf.xacro \
    models/shadow_hand/hand/ff_biotac_hand.urdf.xacro \
    models/shadow_hand/robots/shadowhand_motor.urdf.xacro \
    models/shadow_hand/robots/sr_arm_muscle.urdf.xacro \
    models/shadow_hand/robots/sr_arm_motor.urdf.xacro \
    models/shadow_hand/robots/arm_and_hand_muscle_biotac.urdf.xacro \
    models/shadow_hand/robots/arm_and_hand_muscle.urdf.xacro \
    models/shadow_hand/robots/arm_and_hand_motor_three_finger.urdf.xacro \
    models/shadow_hand/robots/sr_one_finger_motor.urdf.xacro \
    models/shadow_hand/robots/sr_three_finger_edc_muscle_biotac.urdf.xacro \
    models/shadow_hand/robots/arm_and_hand_motor_biotac.urdf.xacro \
    models/shadow_hand/robots/arm_and_hand_motor_ellipsoid.urdf.xacro \
    models/shadow_hand/robots/arm_and_hand_motor.urdf.xacro \
    models/shadow_hand/robots/shadowhand_motor_btsp.urdf.xacro \
    models/shadow_hand/robots/shadowhand_motor_ellipsoid.urdf.xacro \
    models/shadow_hand/robots/sr_three_finger_motor.urdf.xacro \
    models/shadow_hand/robots/shadowhand_left_motor_biotac.urdf.xacro \
    models/shadow_hand/robots/shadowhand_left_motor.urdf.xacro \
    models/shadow_hand/robots/shadowhand_extra_lite.urdf.xacro \
    models/shadow_hand/robots/shadowhand_lite.urdf.xacro \
    models/shadow_hand/robots/bimanual_shadowhand_motor.urdf.xacro \
    models/shadow_hand/robots/shadowhand_motor_ff_biotac.urdf.xacro \
    models/shadow_hand/robots/shadowhand_motor_no_lf.urdf.xacro \
    models/shadow_hand/robots/shadowhand_motor_no_mf_rf_lf.urdf.xacro \
    models/shadow_hand/robots/shadowhand_motor_th_ff_rf_ellipsoid.urdf.xacro \
    models/shadow_hand/robots/shadowhand_motor_biotac.urdf.xacro \
    models/shadow_hand/robots/arm_and_sr_one_finger_motor.urdf.xacro \
    models/shadow_hand/robots/desk_arm_and_hand_motor.urdf.xacro \
    models/shadow_hand/robots/shadowhand_edc_muscle.urdf.xacro \
    models/shadow_hand/robots/shadowhand_left_edc_muscle.urdf.xacro \
    models/shadow_hand/robots/shadowhand_muscle.urdf.xacro \
    models/shadow_hand/robots/shadowhand_left_muscle.urdf.xacro \
    models/shadow_hand/robots/shadowhand_edc_muscle_biotac.urdf.xacro \
    models/shadow_hand/robots/shadowhand_muscle_biotac.urdf.xacro \
    models/shadow_hand/materials.urdf.xacro \
    config/moveit.rviz \
    models/shadow_hand/other/gazebo/gazebo.urdf.xacro

HEADERS += \
    include/my_arm/GeopadMainWindowAgent.h \
    include/my_arm/GeopadQMLAdapter.h \
    include/my_arm/RobotThread.h \
    include/my_arm/K3DQMLItemInfo.h \
    include/my_arm/K3DQMLItemAgent.h \
    include/my_arm/K3DStateMachine.h \
    include/my_arm/K3DMaskedMouseArea.h \
    include/my_arm/KsGlobal.h \
    include/my_arm/RobotArmControllerMain.h \
    include/my_arm/rviz/VMarker.h \
    include/LeapMotion/camera_listener.h \
    include/LeapMotion/hands_listener.h \
    include/my_arm/RobotLeapAdapter.h
