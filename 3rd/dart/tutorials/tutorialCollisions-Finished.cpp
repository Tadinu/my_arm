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

#include "MyWindow.h"
#include "Util.h"

// ==================================================================================================
MyWindow* gbWindow = nullptr;
void gbDoIdleTasks()
{
    if(gbWindow) {
        gbWindow->doIdleTasks();
    }
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
#if 0
  WorldPtr world = dart::utils::SdfParser::readWorld(
              DART_DATA_PATH"sdf/shadow_hand/shadow_hand_full.world");
#else
  // load a skeleton file
  // create and initialize the world
  dart::simulation::WorldPtr myWorld
      = dart::utils::SkelParser::readWorld(
        DART_DATA_PATH"skel/softVoxel_2.skel");
  assert(myWorld != nullptr);

  for(std::size_t i=0; i<myWorld->getNumSkeletons(); ++i)
  {
    dart::dynamics::SkeletonPtr skel = myWorld->getSkeleton(i);
    for(std::size_t j=0; j<skel->getNumBodyNodes(); ++j)
    {
      dart::dynamics::BodyNode* bn = skel->getBodyNode(j);
      Eigen::Vector3d color = dart::Color::Random();
      auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
      for(auto shapeNode : shapeNodes)
        shapeNode->getVisualAspect(true)->setColor(color);
    }
  }

  //WorldPtr world = std::make_shared<World>();
  //myWorld->addSkeleton(createGround());
  //myWorld->addSkeleton(createWall());
#endif

  //
#ifdef DART_VOXEL_MESH
  VVOXELYZE_ADAPTER()->initVoxelyze(_node_handle, false);
  QThread::msleep(1000);
  gbWindow = new MyWindow(myWorld);
#ifdef VOX_CAD
  VVOXELYZE_ADAPTER()->deleteInstance();
#endif
#else
  MyWindow window(world, createBall(), createSoftBody(), createHybridBody(),
                  createRigidChain(), createRigidRing());
#endif

#ifdef ROBOT_LEAP_HANDS
    // !!! REQUIRE ros::init() EARLIER !!!
    // LEAP HANDS --
    cout << "ROBOT_LEAP_HANDS" <<endl;
    VLEAP_INSTANCE()->initLeapMotion(_node_handle);
    ros::Subscriber _leap_listener = _node_handle->subscribe(CLEAP_HANDS_TOPIC, 50,
                                                         &MyWindow::leapCallback, gbWindow); // Require ros::spinOnce()
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
  std::cout << "         If the simulation freezes, you may need to force quit the application.\n" << std::endl;

  glutInit(&argc, argv);
  gbWindow->initWindow(640, 480, "Collisions");

  //Util *util = new Util(gbWindow);
  //QTimer * _serviceTimer = new QTimer();
  //_serviceTimer->setInterval(500);
  //QObject::connect(_serviceTimer, SIGNAL(timeout()), util, SLOT(qDoIdleTask()));
  //_serviceTimer->start();

  //ros::Timer idle_task_timer = _node_handle->createTimer(ros::Duration(0.05), &Util::qDoIdelTask);

  glutIdleFunc(&gbDoIdleTasks);
  glutMainLoop();
}
