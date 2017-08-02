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

#ifndef EXAMPLES_ATLASSIMBICON_MYWINDOW_HPP_
#define EXAMPLES_ATLASSIMBICON_MYWINDOW_HPP_
#include <QtCore>
#include <QtWidgets/QApplication>
#include <QThread>
#include <QtGui/QVector3D>

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

#include "examples/atlasSimbicon/Controller.hpp"

#include "my_arm/KsGlobal.h"
#include "my_arm/RobotVoxelyzeAdapter.h"

#define DART_VOXEL_MESH
#define WORK_IN_WORLD
#define FORCE_ON_RIGIDBODY 25.0
#define FORCE_ON_VERTEX 1.00

using namespace std;
using namespace Eigen;
using namespace dart::common;
using namespace dart::math;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

/// \brief class MyWindow
class MyWindow : public dart::gui::SoftSimWindow
{
  const double default_shape_density = 1000; // kg/m^3
  const double default_shape_height  = 0.1;  // m
  const double default_shape_width   = 0.03; // m
  const double default_skin_thickness = 1e-3; // m

  const double default_start_height = 0;  // m

  const double minimum_start_v = 2.5; // m/s
  const double maximum_start_v = 4.0; // m/s
  const double default_start_v = 3.5; // m/s

  const double minimum_launch_angle = 30.0*M_PI/180.0; // rad
  const double maximum_launch_angle = 70.0*M_PI/180.0; // rad
  const double default_launch_angle = 45.0*M_PI/180.0; // rad

  const double maximum_start_w = 6*M_PI; // rad/s
  const double default_start_w = 3*M_PI;  // rad/s

  const double ring_spring_stiffness = 0.5;
  const double ring_damping_coefficient = 0.05;
  const double default_damping_coefficient = 0.001;

  const double default_ground_width = 2;
  const double default_wall_thickness = 0.1;
  const double default_wall_height = 1;
  const double default_spawn_range = 0.9*default_ground_width/2;

  const double default_restitution = 0.6;

  const double default_vertex_stiffness = 500.0;
  const double default_edge_stiffness = 0.0;
  const double default_soft_damping = 5.0;

public:
  /// \brief Constructor
  explicit MyWindow(const WorldPtr& world);

  /// \brief Destructor
  virtual ~MyWindow();

  // Documentatin inherited
  void displayTimer(int _val) override;

  // Documentation inherited
  void timeStepping() override;

  // Documentation inherited
  void drawSkels() override;

  // Documentation inherited
  void keyboard(unsigned char _key, int _x, int _y) override;

  // Idle Tasks
  void doIdleTasks();

  void setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color);

  void loadRobot();

#ifdef DART_VOXEL_MESH
  SoftBodyNode::UniqueProperties makeMeshProperties(
                                  const Eigen::Vector3d& _size,
                                  const Eigen::Isometry3d& _localTransform,
                                  double _totalMass,
                                  double _vertexStiffness,
                                  double _edgeStiffness,
                                  double _dampingCoeff);

  /// Add a soft body with the specified Joint type to a chain
  template<class JointType>
  BodyNode* addSoftVoxelBody(const SkeletonPtr& chain, const std::string& name,
                             BodyNode* parent = nullptr);
  void updateSoftGround();
#endif
public:
  dart::collision::CollisionDetectorPtr _mCollisionDetector;

private:
  /// \brief External force to exert on Atlas robot
  Eigen::Vector3d mForce;

  /// \brief Number of frames for applying external force
  int mImpulseDuration;

  /// \brief Constroller
  Controller* mController;
  SkeletonPtr mRobot;

#ifdef DART_VOXEL_MESH
  /// A blueprint Skeleton that we will use to spawn soft bodies
  SkeletonPtr mSoftGround;
  QMutex mSoftGroundMutex;
  void determineRobotAndSoftGroundCollision();
#endif
};

#endif  // EXAMPLES_ATLASSIMBICON_MYWINDOW_HPP_
