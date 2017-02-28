## http://wiki.ros.org/catkin/CMakeLists.txt
##
cmake_minimum_required(VERSION 2.8.3)
project(my_arm)

## The ROS Client Libraries implement message generators that translate .msg files into source code.
## These message generators must be invoked from your build script, though most of the gory details
## are taken care of by including some common build rules. By convention, all msg files are stored
## in a directory within your package called "msg." If you have msgs defined there, you simply have
## to add the line rosbuild_genmsg() to your CMakeLists.txt file. Here is an example:
##
## cmake_minimum_required(VERSION 2.6)
## include(rosbuild)
## rosbuild_init()
## rosbuild_genmsg()

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/Qt5.7.1/5.7/gcc64 $ENV{HOME}/DUC/RobotArm/src/my_arm/3rd ${CMAKE_PREFIX_PATH}")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(ROS_KINETIC_INCLUDE "/opt/ros/kinetic/include")
set(BULLET_INCLUDE "/usr/local/include/bullet")

##TEST VAR
##message("CATKIN DIRS " $ENV{librealsense_INCLUDE_DIRS} $ENV{librealsense_LIBRARY_DIRS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospack
  roscpp
  roslib
  rospy
  geometry_msgs
  sensor_msgs
  visualization_msgs
  interactive_markers
  moveit_visual_tools
  std_msgs
  tf

  ## rviz --
  rviz

  ## Gazebo --
  gazebo_plugins
  gazebo_ros

  ## Kinect --
  pcl_ros

  ## Leap Motion --
  camera_calibration_parsers
  camera_info_manager
  image_transport
  message_generation
  message_runtime

  ## Realsense --
  librealsense
  realsense_camera
  cv_bridge
  dynamic_reconfigure

  ## OpenMP --
  #OpenMP

  ## Graspit --
  graspit
  ## graspit_ros_planning_msgs

  ## Bullet --
  bullet_server
)
find_package(gazebo REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Qml REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Network REQUIRED)

SET(QT_LIB_DIR "$ENV{HOME}/Qt5.7.1/5.7/gcc64/lib")
get_target_property(QtCore_location Qt5::Core    ${QT_LIB_DIR})
get_target_property(QtCore_location Qt5::Gui     ${QT_LIB_DIR})
get_target_property(QtCore_location Qt5::Widgets ${QT_LIB_DIR})
get_target_property(QtCore_location Qt5::Quick   ${QT_LIB_DIR})
get_target_property(QtCore_location Qt5::Qml     ${QT_LIB_DIR})
get_target_property(QtCore_location Qt5::Network ${QT_LIB_DIR})
#find_package(Qt5QuickCompiler)
#qtquick_compiler_add_resources(RESOURCES qml.qrc)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# https://cmake.org/cmake/help/v3.0/module/FindBoost.html
# http://www.orocos.org/files/FindBoost.cmake

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(
    FILES
    LeapMotion/leap.msg
    LeapMotion/leapros.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myArm
#  CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs std_msgs tf
#  DEPENDS system_lib
   DEPENDS
      rospack roscpp roslib rospy sensor_msgs std_msgs tf
      gazebo_ros
      camera_calibration_parsers
      camera_info_manager
      geometry_msgs
      image_transport
      message_runtime
      librealsense
      realsense_camera
      ##graspit_ros_planning_msgs
)

###########
## Build ##
###########
qt5_add_resources(QT_RESOURCES_CPP qml.qrc)
qt5_wrap_cpp(QT_MOC_HPP include/my_arm/GeopadMainWindowAgent.h
                        include/my_arm/GeopadQMLAdapter.h
                        include/my_arm/K3DMaskedMouseArea.h
                        include/my_arm/K3DQMLItemAgent.h
                        include/my_arm/K3DQMLItemInfo.h
                        include/my_arm/K3DStateMachine.h
                        include/my_arm/KsGlobal.h
                        include/my_arm/RobotThread.h
                        include/my_arm/RobotLeapAdapter.h
                        include/my_arm/RobotRealSenseAdapter.h
                        include/my_arm/RobotKinectAdapter.h
                        include/Rviz/VMarker.h
                        include/Rviz/mesh_display_custom.h
                        )

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## my_arm & ROS catkin --
include_directories(include
                    include/my_arm
                    3rd

                    ## catkin --
                    ${catkin_INCLUDE_DIRS}

                    ## rviz --
                    ${rviz_INCLUDE_DIRS}

                    ## Gazebo --
                    ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS}

                    ## Qt --
                    ${Qt5Core_INCLUDE_DIRS}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5Gui_INCLUDE_DIRS}
                    ${Qt5Qml_INCLUDE_DIRS}
                    ${Qt5Quick_INCLUDE_DIRS}
                    ${Qt5Network_INCLUDE_DIRS}

                    ## Kinect --
                    include/Kinect
                    include_directories(/usr/include/pcl-1.7)

                    ## Leap Motion --
                    include/LeapMotion
                    if($ENV{LEAP_SDK} $ENV{LEAP_SDK}/include)

                    ## Realsense --
                    # librealsense & realsense_camera are already installed as catkin packages in ROS!
                    include/RealSense
                    include/RealSense/camera
                    include/RealSense/pxc

                    ## Voxelyze --
                    3rd/Voxelyze/include
                    3rd/Voxelyze/include/rapidjson

                    ## Bullet --
                    ${BULLET_INCLUDE}
                    /usr/include/bullet
                    3rd/bullet_server/include
                    )

## Graspit include files should now come directly from the graspit package
set(GRASPIT_ROOT $ENV{GRASPIT})
include_directories(${GRASPIT_ROOT})
include_directories(${GRASPIT_ROOT}/src)
include_directories(${GRASPIT_ROOT}/include)

## OpenNI2 ##
# set(OPENNI2_DIR "{location of OpenNI2 library} /Linux-x64/OpenNI-Linux-x64-2.2.0/")
# rosbuild_init()
#
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#
# link_directories("${OPENNI2_DIR}/Redist/")
# include_directories("${OPENNI2_DIR}/Include")
#
# rosbuild_add_executable(capri src/sensor.cpp)
# target_link_libraries(capri boost_filesystem boost_system OpenNI2)


## Declare a C++ library
# add_library(myArm
#   src/${PROJECT_NAME}/myArm.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(myArm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(robotArmController src/my_arm/RobotArmControllerMain.cpp
                                  src/my_arm/GeopadMainWindowAgent.cpp
                                  src/my_arm/GeopadQMLAdapter.cpp
                                  src/my_arm/K3DMaskedMouseArea.cpp
                                  src/my_arm/K3DQMLItemAgent.cpp
                                  src/my_arm/K3DQMLItemInfo.cpp
                                  src/my_arm/K3DStateMachine.cpp
                                  src/my_arm/KsGlobal.cpp
                                  src/my_arm/RobotArmControllerMain.cpp
                                  src/my_arm/RobotThread.cpp
                                  src/my_arm/RobotLeapAdapter.cpp
                                  src/my_arm/RobotRealSenseAdapter.cpp
                                  src/my_arm/RobotKinectAdapter.cpp
                                  src/Rviz/VMarker.cpp
                                  src/Rviz/mesh_display_custom.cpp

                                  ## Kinect --
                                  src/Kinect/hand_interaction/analyze_hands.cpp
                                  ##src/Kinect/hand_interaction/detect_hands.cpp
                                  ##src/Kinect/hand_interaction/detect_hands_wskel.cpp

                                  ## LeapMotion --
                                  src/LeapMotion/hands_listener.cpp
                                  src/LeapMotion/camera_listener.cpp

                                  ## RealSense --
                                  ##include/RealSense/Base.h
                                  ##include/RealSense/Calibration.h
                                  ##include/RealSense/Capture.h
                                  ##include/RealSense/CaptureManager.h
                                  ##include/RealSense/hands_publisher.h
                                  ##include/RealSense/Image.h
                                  ##include/RealSense/Metadata.h
                                  ##include/RealSense/PlatformCameraControl.h
                                  ##include/RealSense/Playback.h
                                  ##include/RealSense/PowerState.h
                                  ##include/RealSense/Projection.h
                                  ##include/RealSense/Recording.h
                                  ##include/RealSense/Reference.h
                                  ##include/RealSense/Sample.h
                                  ##include/RealSense/SampleReader.h
                                  ##include/RealSense/SenseManager.h
                                  ##include/RealSense/Session.h
                                  ##include/RealSense/Status.h
                                  ##include/RealSense/Type.h
                                  ##include/RealSense/VideoModuleCommon.h
                                  ##include/RealSense/VideoModule.h
                                  ##
                                  ##src/RealSense/hands_publisher.cpp
                                  src/RealSense/camera/base_nodelet.cpp
                                  src/RealSense/camera/sr300_nodelet.cpp

                                  ## Voxelyze --
                                  3rd/Voxelyze/src/Voxelyze.cpp
                                  3rd/Voxelyze/src/VX_Collision.cpp
                                  3rd/Voxelyze/src/VX_External.cpp
                                  3rd/Voxelyze/src/VX_LinearSolver.cpp
                                  3rd/Voxelyze/src/VX_Link.cpp
                                  3rd/Voxelyze/src/VX_Material.cpp
                                  3rd/Voxelyze/src/VX_MaterialLink.cpp
                                  3rd/Voxelyze/src/VX_MaterialVoxel.cpp
                                  3rd/Voxelyze/src/VX_MeshRender.cpp
                                  3rd/Voxelyze/src/VX_Voxel.cpp

                                  ## bullet_server --
                                  3rd/bullet_server/src/bullet_server.cpp

                                  ## QT moc & Resource --
                                  ${QT_RESOURCES_CPP}
                                  ${QT_MOC_HPP}
                                  )

add_library(gazeboRobotArmController SHARED src/Gazebo/gazebo_my_arm_commander_plugin.cpp
                                            src/my_arm/GeopadMainWindowAgent.cpp
                                            src/my_arm/GeopadQMLAdapter.cpp
                                            src/my_arm/K3DMaskedMouseArea.cpp
                                            src/my_arm/K3DQMLItemAgent.cpp
                                            src/my_arm/K3DQMLItemInfo.cpp
                                            src/my_arm/K3DStateMachine.cpp
                                            src/my_arm/KsGlobal.cpp
                                            src/my_arm/RobotArmControllerMain.cpp
                                            src/my_arm/RobotThread.cpp
                                            src/my_arm/RobotLeapAdapter.cpp
                                            src/my_arm/RobotRealSenseAdapter.cpp
                                            src/my_arm/RobotKinectAdapter.cpp
                                            src/Rviz/VMarker.cpp
                                            src/Rviz/mesh_display_custom.cpp

                                            ## LeapMotion --
                                            src/LeapMotion/hands_listener.cpp
                                            src/LeapMotion/camera_listener.cpp

                                            ## Voxelyze --
                                            3rd/Voxelyze/src/Voxelyze.cpp
                                            3rd/Voxelyze/src/VX_Collision.cpp
                                            3rd/Voxelyze/src/VX_External.cpp
                                            3rd/Voxelyze/src/VX_LinearSolver.cpp
                                            3rd/Voxelyze/src/VX_Link.cpp
                                            3rd/Voxelyze/src/VX_Material.cpp
                                            3rd/Voxelyze/src/VX_MaterialLink.cpp
                                            3rd/Voxelyze/src/VX_MaterialVoxel.cpp
                                            3rd/Voxelyze/src/VX_MeshRender.cpp
                                            3rd/Voxelyze/src/VX_Voxel.cpp

                                            ## bullet_server --
                                            3rd/bullet_server/src/bullet_server.cpp

                                            ## QT --
                                            ${QT_RESOURCES_CPP}
                                            ${QT_MOC_HPP}
                                            )

add_library(gazeboCameraPublisher SHARED src/Gazebo/gazebo_camera_publisher_plugin.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(myArm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## http://wiki.ros.org/catkin/CMakeLists.txt
## Specify libraries to link a library or executable target against

## link_directories(${GAZEBO_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu/gazebo-7/plugins)

## CATKIN & QT --
target_link_libraries(robotArmController ${catkin_LIBRARIES}
                                         Qt5::Core
                                         Qt5::Gui
                                         Qt5::Widgets
                                         Qt5::Qml
                                         Qt5::Quick
                                         Qt5::Network
                                         )
target_link_libraries(gazeboRobotArmController ${catkin_LIBRARIES}
                                         Qt5::Core
                                         Qt5::Gui
                                         Qt5::Widgets
                                         Qt5::Qml
                                         Qt5::Quick
                                         Qt5::Network
                                         )

target_link_libraries(gazeboCameraPublisher ${catkin_LIBRARIES}
                                         Qt5::Core
                                         Qt5::Gui
                                         Qt5::Widgets
                                         Qt5::Qml
                                         Qt5::Quick
                                         Qt5::Network
                                         )

## RVIZ --
target_link_libraries(robotArmController ${rviz_LIBRARIES})
target_link_libraries(gazeboRobotArmController ${rviz_LIBRARIES})

## GAZEBO --
target_link_libraries(robotArmController ${GAZEBO_LIBRARIES})
target_link_libraries(gazeboRobotArmController ${GAZEBO_LIBRARIES})
target_link_libraries(gazeboCameraPublisher ${GAZEBO_LIBRARIES} /usr/lib/x86_64-linux-gnu/gazebo-7/plugins/libCameraPlugin.so)

## BOOST ++
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system program_options regex)
#find_package(Boost 1.45.0 COMPONENTS *boost libraries here*)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
#    add_executable(progname file1.cxx file2.cxx)
    target_link_libraries(robotArmController ${Boost_FILESYSTEM_LIBRARY}
                                             ${Boost_LIBRARIES}
                                             ${Boost_SYSTEM_LIBRARY}
                                             ${Boost_PROGRAM_OPTIONS_LIBRARY}
                                             ${Boost_REGEX_LIBRARY})

    target_link_libraries(gazeboRobotArmController ${Boost_FILESYSTEM_LIBRARY}
                                                   ${Boost_LIBRARIES}
                                                   ${Boost_SYSTEM_LIBRARY}
                                                   ${Boost_PROGRAM_OPTIONS_LIBRARY}
                                                   ${Boost_REGEX_LIBRARY})

    target_link_libraries(gazeboCameraPublisher ${Boost_FILESYSTEM_LIBRARY}
                                                ${Boost_LIBRARIES}
                                                ${Boost_SYSTEM_LIBRARY}
                                                ${Boost_PROGRAM_OPTIONS_LIBRARY}
                                                ${Boost_REGEX_LIBRARY})
endif()
# BOOST --

# KINECT ++
## libpcl is installed as a standalone library in Ubuntu
# KINECT --

# LEAP MOTION ++
if(DEFINED ENV{LEAP_SDK})
    target_link_libraries(gazeboRobotArmController $ENV{LEAP_SDK}/lib/x64/libLeap.so)
    target_link_libraries(robotArmController $ENV{LEAP_SDK}/lib/x64/libLeap.so)
    message("Found libLeap...")
endif()
# LEAP MOTION --

# REAL SENSE ++
if(librealsense_FOUND)
    target_link_libraries(gazeboRobotArmController ${librealsense_LIBRARIES})
    target_link_libraries(robotArmController ${librealsense_LIBRARIES})
    message("Found librealsense...")
endif()
# REAL SENSE --

# Voxelyze ++
target_link_libraries(robotArmController $ENV{HOME}/DUC/RobotArm/src/my_arm/3rd/Voxelyze/lib/libvoxelyze.0.9.a)
# Voxelyze --

# Bullet ++
##if(bullet_FOUND)
    target_link_libraries(robotArmController /usr/local/lib/libBullet2FileLoader.so
                                             /usr/local/lib/libBullet3Collision.so
                                             /usr/local/lib/libBullet3Common.so
                                             /usr/local/lib/libBullet3Dynamics.so
                                             /usr/local/lib/libBullet3Geometry.so
                                             /usr/local/lib/libBullet3OpenCL_clew.so
                                             /usr/local/lib/libBulletCollision.so
                                             /usr/local/lib/libBulletDynamics.so
                                             /usr/local/lib/libBulletInverseDynamics.so
                                             /usr/local/lib/libBulletSoftBody.so
                                             /usr/local/lib/libLinearMath.so
                                             )
    target_link_libraries(gazeboRobotArmController /usr/local/lib/libBullet2FileLoader.so
                                                   /usr/local/lib/libBullet3Collision.so
                                                   /usr/local/lib/libBullet3Common.so
                                                   /usr/local/lib/libBullet3Dynamics.so
                                                   /usr/local/lib/libBullet3Geometry.so
                                                   /usr/local/lib/libBullet3OpenCL_clew.so
                                                   /usr/local/lib/libBulletCollision.so
                                                   /usr/local/lib/libBulletDynamics.so
                                                   /usr/local/lib/libBulletInverseDynamics.so
                                                   /usr/local/lib/libBulletSoftBody.so
                                                   /usr/local/lib/libLinearMath.so
                                                   )
    ##message("Found bullet...")
##endif()
# Bullet --

# Ignition Rendering ++
##if(libignition-rendering_FOUND)
    target_link_libraries(gazeboRobotArmController ignition-rendering ignition-rendering-base ignition-rendering-ogre OgreMain)
    target_link_libraries(robotArmController ignition-rendering ignition-rendering-base ignition-rendering-ogre OgreMain)
    ##message("Found libignition-rendering...")
##endif()
# Ignition Rendering --

# NVIDIA OPTIX ++
if(DEFINED ENV{OPTIX_SDK})
    target_link_libraries(gazeboRobotArmController $ENV{OPTIX_SDK}/lib64/liboptix.so)
    target_link_libraries(robotArmController $ENV{OPTIX_SDK}/lib64/liboptix.so)
    message("Found libOptix...")
endif()
# NVIDIA OPTIX --

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(PROGRAMS
    scripts/LeapMotion/leap_interface.py scripts/LeapMotion/sender.py scripts/LeapMotion/subscriber.py
    3rd/bullet_server/scripts/soft_body.py
    3rd/bullet_server/scripts/imarker_spawn.py
    3rd/bullet_server/scripts/init_sim.py
    3rd/bullet_server/scripts/load_urdf.py
    3rd/bullet_server/scripts/make_wall.py
    3rd/bullet_server/scripts/mesh.py
    3rd/bullet_server/scripts/pos_to_vel.py
    3rd/bullet_server/scripts/random_body.py
    3rd/bullet_server/scripts/soft_body.py
    3rd/bullet_server/scripts/soft_tetra.py
    3rd/bullet_server/scripts/soft_vehicle.py
    3rd/bullet_server/scripts/stewart_platform.py
    3rd/bullet_server/scripts/terrain.py
    3rd/bullet_server/scripts/test.py

    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS myArm myArm_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## TODO ++
#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#install(DIRECTORY meshes
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#install(DIRECTORY models
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
## TODO --




## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_myArm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#<export>
#   <cpp cflags="`pkg-config --cflags eigen3` -I${prefix}/include `rosboost-cfg --cflags`"
#        lflags="-Wl,-rpath,${prefix}/lib -L${prefix}/lib -lplanning_models"/>
#</export>

#
#find_package(catkin REQUIRED cmake_modules)
#find_package(Eigen REQUIRED)
#
#catkin_package(
#               INCLUDE_DIRS
#               LIBRARIES
#               CATKIN_DEPENDS
#               DEPENDS Eigen )
#
#include_directories(
#                    ... ${Eigen_INCLUDE_DIRS} )
#                    target_link_libraries(my_target
#                    ....
#                    ${Eigen_LIBRARIES}
#                    )
