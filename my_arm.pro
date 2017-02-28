QT += qml quick core widgets network

CONFIG += c++11
## CONFIG += console
## CONFIG -= app_bundle

#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = robotArmController
TEMPLATE = app

CONFIG(release, debug|release) {
    message(Release)
    LIBS += -L./opencv/lib64/ \
            -lopencv_core2413 \
            \#-lopencv_imgcodecs2413 \
            -lopencv_highgui2413 \
            -lopencv_contrib2413 \
            -lopencv_imgproc2413 \
}

CONFIG(debug, debug|release) {
    message(Debug)
    LIBS += -L./opencv/lib64/ \
              -lopencv_core2413d \
              \#-lopencv_imgcodecs2413d \
              -lopencv_highgui2413d \
              -lopencv_contrib2413d \
              -lopencv_imgproc2413d \
}

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
    src/Rviz/VMarker.cpp \
    src/LeapMotion/hands_listener.cpp \
    src/LeapMotion/camera_listener.cpp \
    src/my_arm/RobotLeapAdapter.cpp \
    src/Kinect/hand_interaction/analyze_hands.cpp \
    src/Kinect/hand_interaction/detect_hands_wskel.cpp \
    src/Kinect/hand_interaction/detect_hands.cpp \
    src/my_arm/RobotRealSenseAdapter.cpp \
    src/RealSense/camera/f200_nodelet.cpp \
    src/RealSense/camera/sr300_nodelet.cpp \
    src/RealSense/camera/r200_nodelet.cpp \
    src/RealSense/camera/zr300_nodelet.cpp \
    src/RealSense/camera/base_nodelet.cpp \
    src/RealSense/hands_publisher.cpp \
    src/Kinect/pcl_tools/bag_to_pcd.cpp \
    src/Kinect/pcl_tools/pcl_utils.cpp \
    src/Kinect/pcl_tools/segfast.cpp \
    src/my_arm/RobotKinectAdapter.cpp \
    src/Gazebo/gazebo_my_arm_commander_plugin.cpp \
    3rd/Voxelyze/src/VX_LinearSolver.cpp \
    3rd/Voxelyze/src/VX_External.cpp \
    3rd/Voxelyze/src/VX_Collision.cpp \
    3rd/Voxelyze/src/Voxelyze.cpp \
    3rd/Voxelyze/src/VX_Link.cpp \
    3rd/Voxelyze/src/VX_Material.cpp \
    3rd/Voxelyze/src/VX_MaterialLink.cpp \
    3rd/Voxelyze/src/VX_MaterialVoxel.cpp \
    3rd/Voxelyze/src/VX_Voxel.cpp \
    3rd/Voxelyze/src/VX_MeshRender.cpp \
    3rd/Voxelyze/test/VoxelyzeUnitTests.cpp \
    src/Gazebo/gazebo_camera_publisher_plugin.cpp \
    src/Rviz/mesh_display_custom.cpp \
    3rd/bullet_server/src/bullet_server.cpp

RESOURCES += qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

INCLUDEPATH+= /usr/include \
              /usr/local/include \
              /opt/ros/kinetic/include \
              /usr/include/gazebo-7 \
              /usr/include/gazebo-7/gazebo \
              \
              include        \
              include/my_arm \
              3rd \
              \
              /home/brhm/LeapSDK/include \
              ./include/Kinect    \
              /usr/include/pcl-1.7 \
              \
              ./include/RealSense \
              ./include/RealSense/pxc \
              \
              3rd/Voxelyze/include \
              \
              3rd/bullet_server/include

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
    models/shadow_hand/other/gazebo/gazebo.urdf.xacro \
    msg/Kinect/body_msgs/Hand.msg \
    msg/Kinect/body_msgs/Skeleton.msg \
    msg/Kinect/body_msgs/SkeletonJoint.msg \
    msg/Kinect/body_msgs/Skeletons.msg \
    models/myArm.gazebo \
    worlds/shadowhand.world \
    src/my_arm/my_arm_controller_py/main.py \
    worlds/universe.world \
    3rd/Voxelyze/makefile \
    3rd/Voxelyze/README.md \
    src/bullet_server/utility/__init__.py \
    src/bullet_server/utility/utility.py \
    3rd/bullet_server/config/bullet_server.perspective \
    3rd/bullet_server/config/bullet_server.rviz \
    3rd/bullet_server/scripts/imarker_spawn.py \
    3rd/bullet_server/scripts/init_sim.py \
    3rd/bullet_server/scripts/load_urdf.py \
    3rd/bullet_server/scripts/make_wall.py \
    3rd/bullet_server/scripts/mesh.py \
    3rd/bullet_server/scripts/pos_to_vel.py \
    3rd/bullet_server/scripts/random_body.py \
    3rd/bullet_server/scripts/soft_body.py \
    3rd/bullet_server/scripts/soft_tetra.py \
    3rd/bullet_server/scripts/soft_vehicle.py \
    3rd/bullet_server/scripts/stewart_platform.py \
    3rd/bullet_server/scripts/terrain.py \
    3rd/bullet_server/scripts/test.py \
    3rd/bullet_server/scripts/tracks.py \
    3rd/bullet_server/msg/Anchor.msg \
    3rd/bullet_server/msg/Body.msg \
    3rd/bullet_server/msg/Constraint.msg \
    3rd/bullet_server/msg/Face.msg \
    3rd/bullet_server/msg/Heightfield.msg \
    3rd/bullet_server/msg/Impulse.msg \
    3rd/bullet_server/msg/Link.msg \
    3rd/bullet_server/msg/Material.msg \
    3rd/bullet_server/msg/Node.msg \
    3rd/bullet_server/msg/SoftBody.msg \
    3rd/bullet_server/msg/SoftConfig.msg \
    3rd/bullet_server/msg/Tetra.msg \
    3rd/bullet_server/src/utility/utility.py \
    3rd/bullet_server/srv/AddBody.srv \
    3rd/bullet_server/srv/AddCompound.srv \
    3rd/bullet_server/srv/AddConstraint.srv \
    3rd/bullet_server/srv/AddHeightfield.srv \
    3rd/bullet_server/srv/AddImpulse.srv

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
    include/Rviz/VMarker.h \
    include/LeapMotion/camera_listener.h \
    include/LeapMotion/hands_listener.h \
    include/my_arm/RobotLeapAdapter.h \
    include/my_arm/RobotRealSenseAdapter.h \
    include/my_arm/RobotKinectAdapter.h \
    include/RealSense/camera/f200_nodelet.h \
    include/RealSense/camera/sr300_nodelet.h \
    include/RealSense/camera/r200_nodelet.h \
    include/RealSense/camera/zr300_nodelet.h \
    include/RealSense/camera/constants.h \
    include/RealSense/camera/base_nodelet.h \
    include/RealSense/pxc/pxc3dscan.h \
    include/RealSense/pxc/pxc3dseg.h \
    include/RealSense/pxc/pxcaddref.h \
    include/RealSense/pxc/pxcaudio.h \
    include/RealSense/pxc/pxcaudiosource.h \
    include/RealSense/pxc/pxcbase.h \
    include/RealSense/pxc/pxcblobconfiguration.h \
    include/RealSense/pxc/pxcblobdata.h \
    include/RealSense/pxc/pxcblobmodule.h \
    include/RealSense/pxc/pxccalibration.h \
    include/RealSense/pxc/pxccapture.h \
    include/RealSense/pxc/pxccapturemanager.h \
    include/RealSense/pxc/pxccursorconfiguration.h \
    include/RealSense/pxc/pxccursordata.h \
    include/RealSense/pxc/pxcdefs.h \
    include/RealSense/pxc/pxcenhancedphoto.h \
    include/RealSense/pxc/pxcenhancedvideo.h \
    include/RealSense/pxc/pxcfaceconfiguration.h \
    include/RealSense/pxc/pxcfacedata.h \
    include/RealSense/pxc/pxcfacemodule.h \
    include/RealSense/pxc/pxchandconfiguration.h \
    include/RealSense/pxc/pxchandcursormodule.h \
    include/RealSense/pxc/pxchanddata.h \
    include/RealSense/pxc/pxchandmodule.h \
    include/RealSense/pxc/pxcimage.h \
    include/RealSense/pxc/pxcmetadata.h \
    include/RealSense/pxc/pxcobjectrecognitionconfiguration.h \
    include/RealSense/pxc/pxcobjectrecognitiondata.h \
    include/RealSense/pxc/pxcobjectrecognitionmodule.h \
    include/RealSense/pxc/pxcpersontrackingconfiguration.h \
    include/RealSense/pxc/pxcpersontrackingdata.h \
    include/RealSense/pxc/pxcpersontrackingmodule.h \
    include/RealSense/pxc/pxcphoto.h \
    include/RealSense/pxc/pxcplatformcameracontrol.h \
    include/RealSense/pxc/pxcpowerstate.h \
    include/RealSense/pxc/pxcprojection.h \
    include/RealSense/pxc/pxcsceneperception.h \
    include/RealSense/pxc/pxcsensemanager.h \
    include/RealSense/pxc/pxcsession.h \
    include/RealSense/pxc/pxcspeechrecognition.h \
    include/RealSense/pxc/pxcspeechsynthesis.h \
    include/RealSense/pxc/pxcstatus.h \
    include/RealSense/pxc/pxcsyncpoint.h \
    include/RealSense/pxc/pxctouchlesscontroller.h \
    include/RealSense/pxc/pxctracker.h \
    include/RealSense/pxc/pxctrackerutils.h \
    include/RealSense/pxc/pxcversion.h \
    include/RealSense/pxc/pxcvideomodule.h \
    include/RealSense/pxc/service/pxcloggingservice.h \
    include/RealSense/pxc/service/pxcpowerstateserviceclient.h \
    include/RealSense/pxc/service/pxcschedulerservice.h \
    include/RealSense/pxc/service/pxcserializableservice.h \
    include/RealSense/pxc/service/pxcsessionservice.h \
    include/RealSense/pxc/service/pxcsmartasyncimpl.h \
    include/RealSense/pxc/service/pxcsyncpointservice.h \
    include/RealSense/pxc/utilities/pxcpointconverter.h \
    include/RealSense/pxc/utilities/pxcrotation.h \
    include/RealSense/pxc/utilities/pxcsmoother.h \
    include/RealSense/Service/LoggingService.h \
    include/RealSense/Service/PowerStateServiceClient.h \
    include/RealSense/Service/SampleReaderService.h \
    include/RealSense/Service/SenseManagerService.h \
    include/RealSense/Service/SerializableService.h \
    include/RealSense/Service/SessionService.h \
    include/RealSense/Service/VideoModuleImpl.h \
    include/RealSense/Utility/PointConverter.h \
    include/RealSense/Utility/Rotation.h \
    include/RealSense/Utility/Smoother.h \
    include/RealSense/Base.h \
    include/RealSense/Calibration.h \
    include/RealSense/Capture.h \
    include/RealSense/CaptureManager.h \
    include/RealSense/Image.h \
    include/RealSense/Metadata.h \
    include/RealSense/PlatformCameraControl.h \
    include/RealSense/Playback.h \
    include/RealSense/PowerState.h \
    include/RealSense/Projection.h \
    include/RealSense/Recording.h \
    include/RealSense/Reference.h \
    include/RealSense/Sample.h \
    include/RealSense/SampleReader.h \
    include/RealSense/SenseManager.h \
    include/RealSense/Session.h \
    include/RealSense/Status.h \
    include/RealSense/Type.h \
    include/RealSense/VideoModule.h \
    include/RealSense/VideoModuleCommon.h \
    src/Kinect/pcl_tools/printing_tools.h \
    include/Kinect/body_msgs/Hand.h \
    include/Kinect/body_msgs/Hands.h \
    include/Kinect/body_msgs/Skeleton.h \
    include/Kinect/body_msgs/SkeletonJoint.h \
    include/Kinect/body_msgs/Skeletons.h \
    include/Kinect/geometric_shapes_msgs/Shape.h \
    include/Kinect/mapping_msgs/AttachedCollisionObject.h \
    include/Kinect/mapping_msgs/CollisionMap.h \
    include/Kinect/mapping_msgs/CollisionObject.h \
    include/Kinect/mapping_msgs/CollisionObjectOperation.h \
    include/Kinect/mapping_msgs/OrientedBoundingBox.h \
    include/Kinect/mapping_msgs/PolygonalMap.h \
    include/Kinect/nnn/nnn.hpp \
    include/Kinect/pcl_tools/clusterevaluation.hpp \
    include/Kinect/pcl_tools/pcl_utils.h \
    include/Kinect/pcl_tools/segfast.hpp \
    include/Gazebo/gazebo_my_arm_commander_plugin.h \
    3rd/Voxelyze/include/Array3D.h \
    3rd/Voxelyze/include/Quat3D.h \
    3rd/Voxelyze/include/Vec3D.h \
    3rd/Voxelyze/include/Voxelyze.h \
    3rd/Voxelyze/include/VX_Collision.h \
    3rd/Voxelyze/include/VX_External.h \
    3rd/Voxelyze/include/VX_LinearSolver.h \
    3rd/Voxelyze/include/VX_Link.h \
    3rd/Voxelyze/include/VX_Material.h \
    3rd/Voxelyze/include/VX_MaterialLink.h \
    3rd/Voxelyze/include/VX_MaterialVoxel.h \
    3rd/Voxelyze/include/VX_MeshRender.h \
    3rd/Voxelyze/include/VX_Utils.h \
    3rd/Voxelyze/include/VX_Voxel.h \
    3rd/Voxelyze/include/rapidjson/error/en.h \
    3rd/Voxelyze/include/rapidjson/error/error.h \
    3rd/Voxelyze/include/rapidjson/internal/dtoa.h \
    3rd/Voxelyze/include/rapidjson/internal/itoa.h \
    3rd/Voxelyze/include/rapidjson/internal/meta.h \
    3rd/Voxelyze/include/rapidjson/internal/pow10.h \
    3rd/Voxelyze/include/rapidjson/internal/stack.h \
    3rd/Voxelyze/include/rapidjson/internal/strfunc.h \
    3rd/Voxelyze/include/rapidjson/msinttypes/inttypes.h \
    3rd/Voxelyze/include/rapidjson/msinttypes/stdint.h \
    3rd/Voxelyze/include/rapidjson/allocators.h \
    3rd/Voxelyze/include/rapidjson/document.h \
    3rd/Voxelyze/include/rapidjson/encodedstream.h \
    3rd/Voxelyze/include/rapidjson/encodings.h \
    3rd/Voxelyze/include/rapidjson/filereadstream.h \
    3rd/Voxelyze/include/rapidjson/filestream.h \
    3rd/Voxelyze/include/rapidjson/filewritestream.h \
    3rd/Voxelyze/include/rapidjson/memorybuffer.h \
    3rd/Voxelyze/include/rapidjson/memorystream.h \
    3rd/Voxelyze/include/rapidjson/prettywriter.h \
    3rd/Voxelyze/include/rapidjson/rapidjson.h \
    3rd/Voxelyze/include/rapidjson/reader.h \
    3rd/Voxelyze/include/rapidjson/stringbuffer.h \
    3rd/Voxelyze/include/rapidjson/writer.h \
    3rd/Voxelyze/test/tArray3D.h \
    3rd/Voxelyze/test/tVoxelyze.h \
    3rd/Voxelyze/test/tVX_Material.h \
    3rd/Voxelyze/test/tVX_MaterialLink.h \
    3rd/Voxelyze/test/tVX_MaterialVoxel.h \
    3rd/Voxelyze/test/tVX_Voxel.h \
    include/Rviz/mesh_display_custom.h \
    3rd/bullet_server/include/bullet_server.h
