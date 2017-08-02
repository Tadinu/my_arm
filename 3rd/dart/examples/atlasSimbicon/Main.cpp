/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
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

#include <iostream>

#include "MyWindow.hpp"
#include "Controller.hpp"

#include "my_arm/RobotVoxelyzeAdapter.h"

// ==================================================================================================
MyWindow* gbWindow = nullptr;
void gbDoIdleTasks()
{
}

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
#if 1
    dart::simulation::WorldPtr myWorld
        = dart::utils::SkelParser::readWorld(
          DART_DATA_PATH"skel/softGround.skel");
#else
    // Create empty soft world
    WorldPtr myWorld(new World);

    // Load ground and Atlas robot and add them to the world
    DartLoader urdfLoader;
    SkeletonPtr ground = urdfLoader.parseSkeleton(
          DART_DATA_PATH"sdf/atlas/ground2.urdf");
    std::string test = myWorld->addSkeleton(ground);

    // Set gravity of the world
    myWorld->setGravity(Vector3d(0.0, -9.81, 0.0));
#endif

#ifdef DART_VOXEL_MESH
    VVOXELYZE_ADAPTER()->initVoxelyze(_node_handle, false);
    QThread::msleep(1000);
    gbWindow = new MyWindow(myWorld);
    gbWindow->setWorld(myWorld);
#ifdef VOX_CAD
    VOXELYZE_ADAPTER()->deleteInstance();
#endif
#else
    // Create a window and link it to the world
    MyWindow window(new Controller(atlas, myWorld->getConstraintSolver()));
    window.setWorld(myWorld);
#endif

    // Print manual
    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << endl;
    cout << "'h': harness pelvis on/off" << endl;
    cout << "'j': harness left foot on/off" << endl;
    cout << "'k': harness right foot on/off" << endl;
    cout << "'r': reset robot" << endl;
    cout << "'n': transite to the next state manually" << endl;
    cout << endl;
    cout << "'a/s': push forward/backward" << endl;
    cout << "'d/f': push right/left" << endl;
    cout << endl;
    cout << "'1': standing controller" << endl;
    cout << "'2': walking controller" << endl;

    // Run glut loop
    glutInit(&argc, argv);
    gbWindow->initWindow(640, 480, "Atlas Robot");
    //glutIdleFunc(&gbDoIdleTasks);
    glutMainLoop();

    return 0;
}
