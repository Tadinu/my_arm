/*
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "DartUtils.h"
#include "MainWindow.h"

#define CSTR_DART_WORLD_PATH (DART_DATA_PATH"skel/ground.skel")
#define CSTR_SHADOW_HAND_WORLD_PATH (DART_DATA_PATH"sdf/shadow_hand/shadow_hand_full.world")
#define CSTR_DART_GROUND_PATH (DART_DATA_PATH"urdf/KR5/ground.urdf")

int main(int argc, char* argv[])
{
    QApplication app(argc, argv); // To run VoxCad

    ros::init(argc, argv, "dart", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    ros::NodeHandle* _node_handle = new ros::NodeHandle(CROS_MY_ARM_PACKAGE_NAME);
    //
    if (!ros::master::check()) {
        std::cout << "ROS INITIALIZATION FAILED!" << std::endl;
        return -1; //do not start without ros.
    }
    // -----------------------------------------------------------------------

    // Initialize the world
    bool isEmptyWorld;
#if 1
    isEmptyWorld = false;
    WorldPtr myWorld = DartUtils::initializeDartWorld(CSTR_DART_WORLD_PATH);
#else
    isEmptyWorld = true;
    WorldPtr myWorld(new World); // std::make_shared<World>
    // Gravity --
    //
    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/1000);

    // load ground
    dart::utils::DartLoader dl;
    dart::dynamics::SkeletonPtr ground = dl.parseSkeleton(CSTR_DART_GROUND_PATH);
    myWorld->addSkeleton(ground);
#endif

    //myWorld->addSkeleton(DartUtils::createGround());
    //myWorld->addSkeleton(DartUtils::createWall());

#ifdef DART_VOXEL_MESH
    VVOXELYZE_ADAPTER()->initVoxelyze(_node_handle, false);
    QThread::msleep(1000);
    MainWindow window(myWorld);
#ifdef VOX_CAD
    VVOXELYZE_ADAPTER()->deleteInstance();
#endif
#else
    MainWindow window(world, dartUtil::createBall(), dartUtil::createSoftBody(),
                    dartUtil::createHybridBody(),
                    dartUtil::createRigidChain(),
                    dartUtil::createRigidRing());
#endif

#ifdef ROBOT_LEAP_HANDS
    // !!! REQUIRE ros::init() EARLIER !!!
    // LEAP HANDS --
    VLEAP_INSTANCE()->initLeapMotion(_node_handle);
    ros::Subscriber _leap_listener = _node_handle->subscribe(CLEAP_HANDS_TOPIC, 1000,
                                                         &MainWindow::leapCallback, &window);
#endif

    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'1': toss a rigid ball" << std::endl;
    std::cout << "'2': toss a soft body" << std::endl;
    std::cout << "'3': toss a hybrid soft/rigid body" << std::endl;
    std::cout << "'4': toss a rigid chain" << std::endl;
    std::cout << "'5': toss a ring of rigid bodies" << std::endl;

    std::cout << "\n'd': delete the oldest object" << std::endl;
    std::cout <<   "'r': toggle randomness" << std::endl;

    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'p': playback/stop" << std::endl;
    std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
    std::cout << "'v': visualization on/off" << std::endl;
    std::cout << "'q','w','a','s','z','x': programmed interaction" << std::endl;

    std::cout << "\nWarning: Let objects settle before tossing a new one, or the simulation could explode." << std::endl;
    std::cout <<   "         If the simulation freezes, you may need to force quit the application.\n" << std::endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Collisions");
    glutIdleFunc(&DartUtils::gbHandleRosCallback);
    glutMainLoop();
}
