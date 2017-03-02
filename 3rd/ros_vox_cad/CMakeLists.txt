## ~/.config/QtProject/qtcreator/styles, where contain .xml style file for QtCreator theme!
## http://wiki.ros.org/catkin/CMakeLists.txt
## QT INSTALLATION CONFIG:
## $ qtchooser -print-env
## QT_SELECT="5"
## QTTOOLDIR="/usr/lib/x86_64-linux-gnu/qt5/bin"
## QTLIBDIR="/usr/lib/x86_64-linux-gnu"

## Edit the paths in following files:
## /usr/lib/x86_64-linux-gnu/qt-default/qtchooser/default.conf
## /usr/lib/x86_64-linux-gnu/qtchooser/4.conf
## /usr/lib/x86_64-linux-gnu/qtchooser/5.conf
## /usr/lib/x86_64-linux-gnu/qtchooser/qt4.conf
## /usr/lib/x86_64-linux-gnu/qtchooser/qt5.conf
## /usr/share/qtchooser/qt5-x86_64-linux-gnu.conf
##
cmake_minimum_required(VERSION 2.8.3)
project(ros_vox_cad)

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
    set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC ${CMAKE_CXX_FLAGS}")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

## http://doc.qt.io/qt-5/cmake-manual.html
# In order for find_package to be successful, Qt 5 must be found below the CMAKE_PREFIX_PATH,
# or the Qt5<Module>_DIR must be set in the CMake cache to the location of the Qt5WidgetsConfig.cmake
# file. The easiest way to use CMake is to set the CMAKE_PREFIX_PATH environment variable to the install
# prefix of Qt 5.
set(QT_DIR "$ENV{HOME}/Qt5.7.1/5.7/gcc_64")
message(STATUS "QT_DIR:" ${QT_DIR})
set(CMAKE_PREFIX_PATH "${QT_DIR} ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_PREFIX_PATH: " ${CMAKE_PREFIX_PATH})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON) //!! TURN THIS ON WOULD AUTOMOC THE TARGET AS WELL -> NO NEEDED!

# Definitions
add_definitions(-DQT_XML_LIB -DQT_OPENGL_LIB -DUSE_ZLIB_COMPRESSION -DUSE_OPEN_GL -DPREC_MED)

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
find_package(Qt5Xml REQUIRED)
find_package(Qt5XmlPatterns REQUIRED)
find_package(Qt5Qml REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Concurrent REQUIRED)

SET(QT_LIB_DIR "${QT_DIR}/lib")
get_target_property(QtCore_location        Qt5::Core        ${QT_LIB_DIR})
get_target_property(QtGui_location         Qt5::Gui         ${QT_LIB_DIR})
get_target_property(QtWidgets_location     Qt5::Widgets     ${QT_LIB_DIR})
get_target_property(QtXml_location         Qt5::Xml         ${QT_LIB_DIR})
get_target_property(QtXmlPatterns_location Qt5::XmlPatterns ${QT_LIB_DIR})
get_target_property(QtQuick_location       Qt5::Quick       ${QT_LIB_DIR})
get_target_property(QtQml_location         Qt5::Qml         ${QT_LIB_DIR})
get_target_property(QtNetwork_location     Qt5::Network     ${QT_LIB_DIR})
get_target_property(QtOpenGL_location      Qt5::OpenGL      ${QT_LIB_DIR})
get_target_property(QtConcurrent_location  Qt5::Concurrent  ${QT_LIB_DIR})

## set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_RELEASE} -fprofile-arcs -ftest-coverage)

# set up a mapping so that the Release configuration for the Qt imported target is
# used in the COVERAGE CMake configuration.
set_target_properties(Qt5::Core        PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::Gui         PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::Widgets     PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::Xml         PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::XmlPatterns PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::Quick       PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::Qml         PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::Network     PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::OpenGL      PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
set_target_properties(Qt5::Concurrent  PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")

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
      geometry_msgs
      ##graspit_ros_planning_msgs
)

###########
## Build ##
###########
qt5_add_resources(QT_RESOURCES_CPP VoxCad.qrc)
qt5_wrap_ui(QT_UI_HPP VoxCad/VoxCad.ui
                      VoxCad/vBCs.ui
                      VoxCad/vFEAInfo.ui
                      VoxCad/VoxCad.ui
                      VoxCad/vPalette.ui
                      VoxCad/vPhysics.ui
                      VoxCad/vPrimEdit.ui
                      VoxCad/vStructureEditor.ui
                      VoxCad/vTensile.ui
                      VoxCad/vVideoGen.ui
                      VoxCad/vVoxInfo.ui
                      VoxCad/vWorkSpace.ui
                      )
##qt5_wrap_cpp(QT_MOC_HPP)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

SET(QT_COMMON_INCLUDE_DIR  "${QT_DIR}/include")
SET(Qt5Core_INCLUDE        "${QT_COMMON_INCLUDE_DIR}/QtCore")
SET(Qt5Widgets_INCLUDE     "${QT_COMMON_INCLUDE_DIR}/QtWidgets")
SET(Qt5Gui_INCLUDE         "${QT_COMMON_INCLUDE_DIR}/QtGui")
SET(Qt5Xml_INCLUDE         "${QT_COMMON_INCLUDE_DIR}/QtXml")
SET(Qt5XmlPatterns         "${QT_COMMON_INCLUDE_DIR}/QtXml")
SET(Qt5Quick_INCLUDE       "${QT_COMMON_INCLUDE_DIR}/QtQuick")
SET(Qt5Qml_INCLUDE         "${QT_COMMON_INCLUDE_DIR}/QtQml")
SET(Qt5Network_INCLUDE     "${QT_COMMON_INCLUDE_DIR}/QtNetwork")
SET(Qt5OpenGL_INCLUDE      "${QT_COMMON_INCLUDE_DIR}/QtOpenGL")
SET(Qt5Concurrent_INCLUDE  "${QT_COMMON_INCLUDE_DIR}/QtConcurrent")

## my_arm & ROS catkin --
include_directories(${CMAKE_CURRENT_BINARY_DIR}
                    VoxCad
                    Voxelyze
                    Voxelyze/Utils
                    QTUtils

                    ## catkin --
                    ${catkin_INCLUDE_DIRS}

                    ## rviz --
                    ${rviz_INCLUDE_DIRS}

                    ## Gazebo --
                    ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS}

                    ## Qt --
                    ${QT_COMMON_INCLUDE_DIR}
                    ${Qt5Core_INCLUDE}
                    ${Qt5Widgets_INCLUDE}
                    ${Qt5Gui_INCLUDE}
                    ${Qt5Xml_INCLUDE}
                    ${Qt5XmlPatterns_INCLUDE}
                    ${Qt5Quick_INCLUDE}
                    ${Qt5Qml_INCLUDE}
                    ${Qt5Network_INCLUDE}
                    ${Qt5OpenGL_INCLUDE}
                    ${Qt5Concurrent_INCLUDE}

                    ##include_directories(/usr/include/pcl-1.7)

                    ## Bullet --
                    ${BULLET_INCLUDE}
                    /usr/include/bullet
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
add_executable(rosVoxCad VoxCad/main.cpp
                         ## QT moc & Resource --
                         ${QT_RESOURCES_CPP}
                         ${QT_UI_HPP}
                         ${QT_MOC_HPP}
                         )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(myArm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## http://wiki.ros.org/catkin/CMakeLists.txt
## Specify libraries to link a library or executable target against

##link_directories(${GAZEBO_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu/gazebo-7/plugins)

## CATKIN & QT --
SET(Qt5Core_LIB       "${QT_LIB_DIR}/libQt5Core.so")
SET(Qt5Widgets_LIB    "${QT_LIB_DIR}/libQt5Widgets.so")
SET(Qt5Gui_LIB        "${QT_LIB_DIR}/libQt5Gui.so")
SET(Qt5Xml_LIB        "${QT_LIB_DIR}/libQt5Xml.so")
SET(Qt5Quick_LIB      "${QT_LIB_DIR}/libQt5Quick.so")
SET(Qt5Qml_LIB        "${QT_LIB_DIR}/libQt5Qml.so")
SET(Qt5Network_LIB    "${QT_LIB_DIR}/libQt5Network.so")
SET(Qt5OpenGL_LIB     "${QT_LIB_DIR}/libQt5OpenGL.so")
SET(Qt5Concurrent_LIB "${QT_LIB_DIR}/libQt5Concurrent.so")
target_link_libraries(rosVoxCad ${catkin_LIBRARIES}
                                         ${Qt5Core_LIB}
                                         ${Qt5Widgets_LIB}
                                         ${Qt5Gui_LIB}
                                         ${Qt5Xml_LIB}
                                         ${Qt5XmlPatterns_LIB}
                                         ${Qt5Quick_LIB}
                                         ${Qt5Qml_LIB}
                                         ${Qt5Network_LIB}
                                         ${Qt5OpenGL_LIB}
                                         ${Qt5Concurrent_LIB}
                                         )

## RVIZ --
target_link_libraries(rosVoxCad ${rviz_LIBRARIES})
## GAZEBO --
target_link_libraries(rosVoxCad ${GAZEBO_LIBRARIES})

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
    target_link_libraries(rosVoxCad ${Boost_FILESYSTEM_LIBRARY}
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
    target_link_libraries(rosVoxCad $ENV{LEAP_SDK}/lib/x64/libLeap.so)
    message("Found libLeap...")
endif()
# LEAP MOTION --

# REAL SENSE ++
if(librealsense_FOUND)
    target_link_libraries(rosVoxCad ${librealsense_LIBRARIES})
    message("Found librealsense...")
endif()
# REAL SENSE --

# VoxCad ++
target_link_libraries(rosVoxCad $ENV{HOME}/DUC/SOFT_BODY_DYNAMICS/RosVoxCad/src/ros_vox_cad/release/libVoxCad.a) ## The static for easy debugging!
# VoxCad --

# Bullet ++
##if(bullet_FOUND)
    target_link_libraries(rosVoxCad /usr/local/lib/libBullet2FileLoader.so
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
    target_link_libraries(rosVoxCad ignition-rendering ignition-rendering-base ignition-rendering-ogre OgreMain)
    ##message("Found libignition-rendering...")
##endif()
# Ignition Rendering --

# NVIDIA OPTIX ++
if(DEFINED ENV{OPTIX_SDK})
    target_link_libraries(rosVoxCad $ENV{OPTIX_SDK}/lib64/liboptix.so)
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
