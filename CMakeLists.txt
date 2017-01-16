cmake_minimum_required(VERSION 2.8.3)
project(my_arm)

# http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_CXX_FLAGS "-std=c++11 ${GAZEBO_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/Qt5.7.1/5.7/gcc64 ${CMAKE_PREFIX_PATH}")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(ROS_KINETIC_INCLUDE "/opt/ros/kinetic/include")

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
  std_msgs
  tf

  ## Gazebo --
  gazebo_ros

  ## Leap Motion --
  camera_calibration_parsers
  camera_info_manager
  image_transport
  message_generation
  message_runtime

  ## OpenMP --
  #OpenMP

  ## Graspit --
  graspit
  ## graspit_ros_planning_msgs
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
                        include/my_arm/rviz/VMarker.h
                        include/my_arm/RobotLeapAdapter.h
                        )

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## my_arm & ROS catkin --
include_directories(include include/my_arm include/LeapMotion
                    ${catkin_INCLUDE_DIRS})

## Gazebo --
include_directories(${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

## Qt --
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})
include_directories(${Qt5Qml_INCLUDE_DIRS})
include_directories(${Qt5Quick_INCLUDE_DIRS})
include_directories(${Qt5Network_INCLUDE_DIRS})

## Leap Motion --
include_directories(
    if($ENV{LEAP_SDK} $ENV{LEAP_SDK}/include)
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
                                  src/my_arm/rviz/VMarker.cpp
                                  src/my_arm/RobotLeapAdapter.cpp

                                  src/LeapMotion/hands_listener.cpp
                                  src/LeapMotion/camera_listener.cpp

                                  ${QT_RESOURCES_CPP}
                                  ${QT_MOC_HPP})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(myArm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robotArmController ${catkin_LIBRARIES}
                                         Qt5::Core
                                         Qt5::Gui
                                         Qt5::Widgets
                                         Qt5::Qml
                                         Qt5::Quick
                                         Qt5::Network
                                         )
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
endif()
# BOOST --

# LEAP MOTION ++
if(DEFINED ENV{LEAP_SDK})
    target_link_libraries(robotArmController $ENV{LEAP_SDK}/lib/x64/libLeap.so)
endif()
# LEAP MOTION --

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
