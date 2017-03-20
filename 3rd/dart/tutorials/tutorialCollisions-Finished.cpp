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

#include <QtWidgets/QApplication>
#include <QThread>
#include <random>

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

#include "my_arm/KsGlobal.h"
#include "my_arm/RobotVoxelyzeAdapter.h"
#include "my_arm/RobotLeapAdapter.h"
#include "Voxelyze/Utils/Mesh.h"

#define DART_VOXEL_MESH
#define FORCE_ON_RIGIDBODY 25.0
#define FORCE_ON_VERTEX 1.00

const double default_shape_density = 1000; // kg/m^3
const double default_shape_height  = 0.1;  // m
const double default_shape_width   = 0.03; // m
const double default_skin_thickness = 1e-3; // m

const double default_start_height = 0.4;  // m

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

const double default_vertex_stiffness = 100.0;
const double default_edge_stiffness = 0.0;
const double default_soft_damping = 5.0;

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
// using namespace dart::common;
// using namespace dart::math;
// using namespace dart::dynamics;
// using namespace dart::utils;

SkeletonPtr createManipulator();

void setupRing(const SkeletonPtr& ring)
{
  // Set the spring and damping coefficients for the degrees of freedom
  for(std::size_t i=6; i < ring->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = ring->getDof(i);
    dof->setSpringStiffness(ring_spring_stiffness);
    dof->setDampingCoefficient(ring_damping_coefficient);
  }

  // Compute the joint angle needed to form a ring
  std::size_t numEdges = ring->getNumBodyNodes();
  double angle = 2*M_PI/numEdges;

  // Set the BallJoints so that they have the correct rest position angle
  for(std::size_t i=1; i < ring->getNumJoints(); ++i)
  {
    Joint* joint = ring->getJoint(i);
    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d(0, 1, 0));
    Eigen::Vector3d restPos = BallJoint::convertToPositions(
          Eigen::Matrix3d(rotation));

    for(std::size_t j=0; j<3; ++j)
      joint->setRestPosition(j, restPos[j]);
  }

  // Set the Joints to be in their rest positions
  for(std::size_t i=6; i < ring->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = ring->getDof(i);
    dof->setPosition(dof->getRestPosition());
  }
}

void gbHandleRosCallback()
{
    // --------------------------------------------------------------------
    // HANDLE ROS CALLBACKS --
    //std::cout << "Ros spinning once!";
    ros::spinOnce();
}

class MyWindow : public dart::gui::SoftSimWindow
{
public:

#ifdef DART_VOXEL_MESH
    MyWindow(const WorldPtr& world,
             const SkeletonPtr& softVoxelBody,
             const SkeletonPtr& shadowHand)
      : mRandomize(true),
        mRD(),
        mMT(mRD()),
        mDistribution(-1.0, std::nextafter(1.0, 2.0)),
        mOriginalSoftVoxelBody(softVoxelBody),
        mShadowHand(shadowHand),
        mSkelCount(0),
        mForceOnRigidBody(Eigen::Vector3d::Zero()),
        mForceOnVertex(Eigen::Vector3d::Zero()),
        mImpulseDuration(0.0)
    {
      setWorld(world);
    }
#else
  MyWindow(const WorldPtr& world, const SkeletonPtr& ball,
           const SkeletonPtr& softBody, const SkeletonPtr& hybridBody,
           const SkeletonPtr& rigidChain, const SkeletonPtr& rigidRing)
    : mRandomize(true),
      mRD(),
      mMT(mRD()),
      mDistribution(-1.0, std::nextafter(1.0, 2.0)),
      mOriginalBall(ball),
      mOriginalSoftBody(softBody),
      mOriginalHybridBody(hybridBody),
      mOriginalRigidChain(rigidChain),
      mOriginalRigidRing(rigidRing),
      mSkelCount(0)
  {
    setWorld(world);
  }
#endif

  /// \brief
  void timeStepping() override
  {
#ifdef DART_VOXEL_MESH
    if(mOriginalSoftVoxelBody != nullptr) {
      dart::dynamics::SoftBodyNode* softBodyNode = mOriginalSoftVoxelBody->getSoftBodyNode(0);
#else
      if(mOriginalHybridBody != nullptr) {
        dart::dynamics::SoftBodyNode* softBodyNode = mOriginalHybridBody->getSoftBodyNode(0);
#endif
      if(softBodyNode) {
          softBodyNode->addExtForce(mForceOnRigidBody);
      }

      mWorld->step();

      // for perturbation test
      mImpulseDuration--;
      if (mImpulseDuration <= 0)
      {
        mImpulseDuration = 0;
        mForceOnRigidBody.setZero();
      }

      mForceOnVertex /= 2.0;
    }
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
#ifdef DART_VOXEL_MESH
      case '0':
        addObject(mOriginalSoftVoxelBody->clone());
        mWorld->addSkeleton(mShadowHand);
        //mWorld->addSkeleton(createManipulator());
      break;
#else
      case '1':
        addObject(mOriginalBall->clone());
        break;

      case '2':
        addObject(mOriginalSoftBody->clone());
        break;

      case '3':
        addObject(mOriginalHybridBody->clone());
        break;

      case '4':
        addObject(mOriginalRigidChain->clone());
        break;

      case '5':
        addRing(mOriginalRigidRing->clone());
        break;
#endif
      case 'd':
        if(mWorld->getNumSkeletons() > 2)
          removeSkeleton(mWorld->getSkeleton(2));
        std::cout << "Remaining objects: " << mWorld->getNumSkeletons()-2
                  << std::endl;
        break;

      case 'r':
        mRandomize = !mRandomize;
        std::cout << "Randomization: " << (mRandomize? "on" : "off")
                  << std::endl;
        break;

    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating)
        mPlay = false;
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay)
        mSimulating = false;
      break;
    case '[':  // step backward
      if (!mSimulating)
      {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating)
      {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case 'n':
      mShowPointMasses = !mShowPointMasses;
      break;
    case 'm':
      mShowMeshs = !mShowMeshs;
      break;
    case 'w':  // upper right force
      mForceOnRigidBody[0] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 'q':  // upper right force
      mForceOnRigidBody[0] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 's':  // upper left force
      mForceOnRigidBody[1] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 'a':  // upper right force
      mForceOnRigidBody[1] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 'x':  // upper right force
      mForceOnRigidBody[2] = -FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
    case 'z':  // upper right force
      mForceOnRigidBody[2] = FORCE_ON_RIGIDBODY;
      mImpulseDuration = 100;
      break;
      default:
        SimWindow::keyboard(key, x, y);
    }
    glutPostRedisplay();
  }

  void drawWorld() const override
  {
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    dart::dynamics::SkeletonPtr Skeleton0 = mWorld->getSkeleton(0);
    //
    if(Skeleton0 != nullptr) {
        Eigen::Vector4d color;
        color << 0.5, 0.8, 0.6, 1.0;
        drawSkeleton(mWorld->getSkeleton(0).get(), color, false);
    }

    // draw arrow
#ifdef DART_VOXEL_MESH
    if (mImpulseDuration > 0 && mOriginalSoftVoxelBody != nullptr)
    {
        dart::dynamics::SoftBodyNode* softBodyNode = mOriginalSoftVoxelBody->getSoftBodyNode(0);
#else
    if (mImpulseDuration > 0 && mOriginalHybridBody != nullptr)
    {
        dart::dynamics::SoftBodyNode* softBodyNode = mOriginalHybridBody->getSoftBodyNode(0);
#endif
        if(softBodyNode) {
            softBodyNode->addExtForce(mForceOnRigidBody);
            Eigen::Vector3d poa
                = softBodyNode->getTransform() * Eigen::Vector3d(0.0, 0.0, 0.0);
            Eigen::Vector3d start = poa - mForceOnRigidBody / 25.0;
            double len = mForceOnRigidBody.norm() / 25.0;
            dart::gui::drawArrow3D(start, mForceOnRigidBody, len, 0.025, 0.05);
        }
    }

    SimWindow::drawWorld();
  }

  void displayTimer(int _val) override
  {
    // We remove playback and baking, because we want to be able to add and
    // remove objects during runtime
    int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
    if (mSimulating)
    {
      for (int i = 0; i < numIter; i++)
        timeStepping();
    }
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
  }

  void leapCallback(const visualization_msgs::MarkerArray&)
  {
    determineHandArrangmentOnLeapHands();
  }

  void updateJointPosition(int jointId, double position)
  {
      if(jointId < 0 || jointId >= KsGlobal::VSHADOW_HAND_ARM_JOINT_TOTAL) {
          return;
      }
      // -------------------------------------------------------------------
      std::string jointName = KsGlobal::CSHADOWHAND_ARM_JOINTS[jointId];
#if 1 // Set position using JOINT
      std::size_t numJoints = mShadowHand->getNumJoints();
      Joint* joint;
      for(size_t i = 0; i < numJoints; i++) {
          joint = mShadowHand->getJoint(i);
          if(joint->getName() == jointName) {
              //std::cout << "Set position for jointName" << jointName << std::endl;
              joint->setPosition(0, position);
              break;
          }
      }
#else // Set position using DOF
      std::size_t numDofs = mShadowHand->getNumDofs();
      DegreeOfFreedom* dof;
      for(size_t i = 0; i < numDofs; i++) {
          dof = mShadowHand->getDof(i);
          if(dof->getName() == jointName) {
              std::cout << "Set position for jointName" << jointName << std::endl;
              dof->setPosition(position);
              break;
          }
      }
#endif
  }

  void determineHandArrangmentOnLeapHands()
  {
#ifdef ROBOT_LEAP_HANDS
      // Take the first hand data for convenience:
      std::vector<std::vector<double>> jointValues = VLEAP_INSTANCE()->getFingerJointValues(0);
      //std::cout << "JOINT VALUE SIZE: " << jointValues.size();
      if(jointValues.size() > 0) {
          //ROS_INFO("Fingers joints: %f %f %f", jointValues[1][0], jointValues[1][1], jointValues[1][2]);
          //std::cout << "Fingers joints :" << jointValues[1][0] << " " << jointValues[1][1] << " " << jointValues[1][2];
          //updateJointPosition(KsGlobal::VSHADOW_HAND_ARM_BASE_JOINT, 0);
          updateJointPosition(KsGlobal::VSHADOW_HAND_WRJ2, 0);
          updateJointPosition(KsGlobal::VSHADOW_HAND_WRJ1, 0);

          updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ5, jointValues[0][0]);
          updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ4, qAbs(jointValues[0][1]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ3, qAbs(jointValues[0][1]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ2, qAbs(jointValues[0][2]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_THJ1, qAbs(jointValues[0][2]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_THUMB_TIP, qAbs(jointValues[0][2]));

          updateJointPosition(KsGlobal::VSHADOW_FINGER_1_J4, jointValues[1][0]);
          updateJointPosition(KsGlobal::VSHADOW_FINGER_1_J3, qAbs(jointValues[1][1]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_1_J2, qAbs(jointValues[1][2]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_1_J1, qAbs(jointValues[1][3]));

          updateJointPosition(KsGlobal::VSHADOW_FINGER_2_J4, jointValues[2][0]);
          updateJointPosition(KsGlobal::VSHADOW_FINGER_2_J3, qAbs(jointValues[2][1]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_2_J2, qAbs(jointValues[2][2]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_2_J1, qAbs(jointValues[2][3]));

          updateJointPosition(KsGlobal::VSHADOW_FINGER_3_J4, -jointValues[3][0]);
          updateJointPosition(KsGlobal::VSHADOW_FINGER_3_J3, qAbs(jointValues[3][1]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_3_J2, qAbs(jointValues[3][2]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_3_J1, qAbs(jointValues[3][3]));

          updateJointPosition(KsGlobal::VSHADOW_FINGER_4_LFJ5, qAbs(jointValues[4][0]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_4_LFJ4, -jointValues[4][0]);
          updateJointPosition(KsGlobal::VSHADOW_FINGER_4_J3, qAbs(jointValues[4][1]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_4_J2, qAbs(jointValues[4][2]));
          updateJointPosition(KsGlobal::VSHADOW_FINGER_4_J1, qAbs(jointValues[4][3]));
      }
#endif
  }
protected:

  /// Add an object to the world and toss it at the wall
  bool addObject(const SkeletonPtr& object)
  {
    // Set the starting position for the object
    Eigen::Vector6d positions(Eigen::Vector6d::Zero());

    // If randomization is on, we will randomize the starting y-location
    if(mRandomize)
      positions[4] = default_spawn_range * mDistribution(mMT);

    positions[5] = default_start_height;
    std::string objectName = object->getName();
    object->getJoint(0)->setPositions(positions);

    // Add the object to the world
    object->setName(object->getName()+std::to_string(mSkelCount++));

    // Look through the collisions to see if the new object would start in
    // collision with something
    auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
    auto collisionGroup = mWorld->getConstraintSolver()->getCollisionGroup();
    auto newGroup = collisionEngine->createCollisionGroup(object.get());

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collision = collisionGroup->collide(newGroup.get(), option, &result);

    // If the new object is not in collision
    if(!collision)
    {
      mWorld->addSkeleton(object);
    }
    else
    {
      // or refuse to add the object if it is in collision
      std::cout << "The new object " << objectName << " spawned in a collision. "
                << "It will not be added to the world." << std::endl;
      return false;
    }

    // Create reference frames for setting the initial velocity
    Eigen::Isometry3d centerTf(Eigen::Isometry3d::Identity());
    centerTf.translation() = object->getCOM();
    dart::dynamics::SimpleFrame center(dart::dynamics::Frame::World(), "center", centerTf);

    // Set the velocities of the reference frames so that we can easily give the
    // Skeleton the linear and angular velocities that we want
    double angle = default_launch_angle;
    double speed = default_start_v;
    double angular_speed = default_start_w;
    if(mRandomize)
    {
      angle = (mDistribution(mMT) + 1.0)/2.0 *
          (maximum_launch_angle - minimum_launch_angle) + minimum_launch_angle;

      speed = (mDistribution(mMT) + 1.0)/2.0 *
          (maximum_start_v - minimum_start_v) + minimum_start_v;

      angular_speed = mDistribution(mMT) * maximum_start_w;
    }

    Eigen::Vector3d v = speed * Eigen::Vector3d(cos(angle), 0.0, sin(angle));
    Eigen::Vector3d w = angular_speed * Eigen::Vector3d::UnitY();
    center.setClassicDerivatives(v, w);

    dart::dynamics::SimpleFrame ref(&center, "root_reference");
    ref.setRelativeTransform(object->getBodyNode(0)->getTransform(&center));

    // Use the reference frames to set the velocity of the Skeleton's root
    object->getJoint(0)->setVelocities(ref.getSpatialVelocity());

    return true;
  }

  /// Add a ring to the world, and create a BallJoint constraint to ensure that
  /// it stays in a ring shape
  void addRing(const SkeletonPtr& ring)
  {
    setupRing(ring);

    if(!addObject(ring))
      return;

    // Create a closed loop to turn the chain into a ring
    BodyNode* head = ring->getBodyNode(0);
    BodyNode* tail = ring->getBodyNode(ring->getNumBodyNodes()-1);

    // Compute the offset where the JointConstraint should be located
    Eigen::Vector3d offset = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
    offset = tail->getWorldTransform() * offset;
    auto constraint = std::make_shared<dart::constraint::BallJointConstraint>(
          head, tail, offset);

    mWorld->getConstraintSolver()->addConstraint(constraint);
    mJointConstraints.push_back(constraint);
  }

  /// Remove a Skeleton and get rid of the constraint that was associated with
  /// it, if one existed
  void removeSkeleton(const SkeletonPtr& skel)
  {
    for(std::size_t i=0; i<mJointConstraints.size(); ++i)
    {
      const dart::constraint::JointConstraintPtr& constraint =
          mJointConstraints[i];

      if(constraint->getBodyNode1()->getSkeleton() == skel
         || constraint->getBodyNode2()->getSkeleton() == skel)
      {
        mWorld->getConstraintSolver()->removeConstraint(constraint);
        mJointConstraints.erase(mJointConstraints.begin()+i);
        break; // There should only be one constraint per skeleton
      }
    }

    mWorld->removeSkeleton(skel);
  }

  /// Flag to keep track of whether or not we are randomizing the tosses
  bool mRandomize;

  // std library objects that allow us to generate high-quality random numbers
  std::random_device mRD;
  std::mt19937 mMT;
  std::uniform_real_distribution<double> mDistribution;

  /// History of the active JointConstraints so that we can properly delete them
  /// when a Skeleton gets removed
  std::vector<dart::constraint::JointConstraintPtr> mJointConstraints;

  /// A blueprint Skeleton that we will use to spawn balls
  SkeletonPtr mOriginalBall;

  /// A blueprint Skeleton that we will use to spawn soft bodies
  SkeletonPtr mOriginalSoftBody;

#ifdef DART_VOXEL_MESH
  /// A blueprint Skeleton that we will use to spawn soft bodies
  SkeletonPtr mOriginalSoftVoxelBody;
  SkeletonPtr mShadowHand;
#endif

  /// A blueprint Skeleton that we will use to spawn hybrid bodies
  SkeletonPtr mOriginalHybridBody;

  /// A blueprint Skeleton that we will use to spawn rigid chains
  SkeletonPtr mOriginalRigidChain;

  /// A blueprint Skeleton that we will use to spawn rigid rings
  SkeletonPtr mOriginalRigidRing;

  /// Keep track of how many Skeletons we spawn to ensure we can give them all
  /// unique names
  std::size_t mSkelCount;

  /// \brief
  Eigen::Vector3d mForceOnRigidBody;

  /// \brief Number of frames for applying external force
  int mImpulseDuration;

  /// \brief
  Eigen::Vector3d mForceOnVertex;
};
// End class MyWindow : public dart::gui::SimWindow

/// Add a rigid body with the specified Joint type to a chain
template<class JointType>
BodyNode* addRigidBody(const SkeletonPtr& chain, const std::string& name,
                       Shape::ShapeType type, BodyNode* parent = nullptr)
{
  // Set the Joint properties
  typename JointType::Properties properties;
  properties.mName = name+"_joint";
  if(parent)
  {
    // If the body has a parent, we should position the joint to be in the
    // middle of the centers of the two bodies
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
    properties.mT_ParentBodyToJoint = tf;
    properties.mT_ChildBodyToJoint = tf.inverse();
  }

  // Create the Joint and Body pair
  BodyNode* bn = chain->createJointAndBodyNodePair<JointType>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  // Make the shape based on the requested Shape type
  ShapePtr shape;
  if(Shape::BOX == type)
  {
    shape = std::make_shared<BoxShape>(Eigen::Vector3d(
                                         default_shape_width,
                                         default_shape_width,
                                         default_shape_height));
  }
  else if(Shape::CYLINDER == type)
  {
    shape = std::make_shared<CylinderShape>(default_shape_width/2.0,
                                            default_shape_height);
  }
  else if(Shape::ELLIPSOID == type)
  {
    shape = std::make_shared<EllipsoidShape>(
          default_shape_height*Eigen::Vector3d::Ones());
  }

  bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);

  // Setup the inertia for the body
  Inertia inertia;
  double mass = default_shape_density * shape->getVolume();
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  bn->setInertia(inertia);

  // Set the coefficient of restitution to make the body more bouncy
  bn->setRestitutionCoeff(default_restitution);

  // Set damping to make the simulation more stable
  if(parent)
  {
    Joint* joint = bn->getParentJoint();
    for(std::size_t i=0; i < joint->getNumDofs(); ++i)
      joint->getDof(i)->setDampingCoefficient(default_damping_coefficient);
  }

  return bn;
}

enum SoftShapeType {
  SOFT_BOX = 0,
  SOFT_CYLINDER,
  SOFT_ELLIPSOID
};

/// Add a soft body with the specified Joint type to a chain
template<class JointType>
BodyNode* addSoftBody(const SkeletonPtr& chain, const std::string& name,
                      SoftShapeType type, BodyNode* parent = nullptr)
{
  // Set the Joint properties
  typename JointType::Properties joint_properties;
  joint_properties.mName = name+"_joint";
  if(parent)
  {
    // If the body has a parent, we should position the joint to be in the
    // middle of the centers of the two bodies
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
    joint_properties.mT_ParentBodyToJoint = tf;
    joint_properties.mT_ChildBodyToJoint = tf.inverse();
  }

  // Set the properties of the soft body
  SoftBodyNode::UniqueProperties soft_properties;
  // Use the SoftBodyNodeHelper class to create the geometries for the
  // SoftBodyNode
  if(SOFT_BOX == type)
  {
    // Make a wide and short box
    double width = default_shape_height, height = 2*default_shape_width;
    Eigen::Vector3d dims(width, width, height);

    double mass = 2*dims[0]*dims[1] + 2*dims[0]*dims[2] + 2*dims[1]*dims[2];
    mass *= default_shape_density * default_skin_thickness;
    soft_properties = SoftBodyNodeHelper::makeBoxProperties(
          dims, Eigen::Isometry3d::Identity(), Eigen::Vector3i(4,4,4), mass);
  }
  else if(SOFT_CYLINDER == type)
  {
    // Make a wide and short cylinder
    double radius = default_shape_height/2.0, height = 2*default_shape_width;

    // Mass of center
    double mass = default_shape_density * height * 2*M_PI*radius
                  * default_skin_thickness;
    // Mass of top and bottom
    mass += 2 * default_shape_density * M_PI*pow(radius,2)
                * default_skin_thickness;
    soft_properties = SoftBodyNodeHelper::makeCylinderProperties(
          radius, height, 8, 3, 2, mass);
  }
  else if(SOFT_ELLIPSOID == type)
  {
    double radius = default_shape_height/2.0;
    Eigen::Vector3d dims = 2*radius*Eigen::Vector3d::Ones();
    double mass = default_shape_density * 4.0*M_PI*pow(radius, 2)
                  * default_skin_thickness;
    soft_properties = SoftBodyNodeHelper::makeEllipsoidProperties(
          dims, 6, 6, mass);
  }
  soft_properties.mKv = default_vertex_stiffness;
  soft_properties.mKe = default_edge_stiffness;
  soft_properties.mDampCoeff = default_soft_damping;

  // Create the Joint and Body pair
  SoftBodyNode::Properties body_properties(BodyNode::AspectProperties(name),
                                           soft_properties);
  SoftBodyNode* bn = chain->createJointAndBodyNodePair<JointType, SoftBodyNode>(
        parent, joint_properties, body_properties).second;

  // Zero out the inertia for the underlying BodyNode
  Inertia inertia;
  inertia.setMoment(1e-8*Eigen::Matrix3d::Identity());
  inertia.setMass(1e-8);
  bn->setInertia(inertia);

  // Make the shape transparent
  auto visualAspect = bn->getShapeNodesWith<VisualAspect>()[0]->getVisualAspect();
  Eigen::Vector4d color = visualAspect->getRGBA();
  color[3] = 0.4;
  visualAspect->setRGBA(color);

  return bn;
}

#ifdef DART_VOXEL_MESH //ducta
SoftBodyNode::UniqueProperties makeMeshProperties(
                                const Eigen::Vector3d& _size,
                                const Eigen::Isometry3d& _localTransform,
                                double _totalMass,
                                double _vertexStiffness,
                                double _edgeStiffness,
                                double _dampingCoeff)
{
    SoftBodyNode::UniqueProperties properties(
                  _vertexStiffness, _edgeStiffness, _dampingCoeff);
    //----------------------------------------------------------------------------
    // Point masses
    //----------------------------------------------------------------------------
    // Number of point masses
    const std::vector<CVertex>& vertices = VVOXELYZE_ADAPTER()->getVoxelMeshVertices();
    const std::vector<CLine>& lines      = VVOXELYZE_ADAPTER()->getVoxelMeshLines();
    std::size_t nPointMasses = vertices.size();
    properties.mPointProps.resize(nPointMasses);

    // Mass per vertices
    double pointMass = _totalMass / nPointMasses;

    // Resting positions for each point mass
    std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                            Eigen::Vector3d::Zero());

    for(size_t i = 0; i < nPointMasses; i++) {
        restingPos[i] = _size.cwiseProduct(Eigen::Vector3d(vertices[i].v.x, vertices[i].v.y, vertices[i].v.z)) * 0.5;
    }

    // Point masses
    for (std::size_t i = 0; i < nPointMasses; ++i) {
        properties.mPointProps[i].mX0 = _localTransform * restingPos[i];
        properties.mPointProps[i].mMass = pointMass;
    }

    //----------------------------------------------------------------------------
    // Edges
    //----------------------------------------------------------------------------
    for(size_t i = 0; i < lines.size(); i++) {
        properties.connectPointMasses(lines[i].vi[0], lines[i].vi[1]);
    }

    //----------------------------------------------------------------------------
    // Faces
    //----------------------------------------------------------------------------
    const std::vector<CFacet>& facets = VVOXELYZE_ADAPTER()->getVoxelMeshFaces();
    //
    Eigen::Vector3i face;
    for(size_t i = 0; i < facets.size(); i++) {
        face[0] = facets[i].vi[0];
        face[1] = facets[i].vi[1];
        face[2] = facets[i].vi[2];
        properties.addFace(face);
    }

    return properties;
}

/// Add a soft body with the specified Joint type to a chain
template<class JointType>
BodyNode* addSoftVoxelBody(const SkeletonPtr& chain, const std::string& name,
                           BodyNode* parent = nullptr)
{
  // Set the Joint properties
  typename JointType::Properties joint_properties;
  joint_properties.mName = name+"_joint";
  if(parent)
  {
    // If the body has a parent, we should position the joint to be in the
    // middle of the centers of the two bodies
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
    joint_properties.mT_ParentBodyToJoint = tf;
    joint_properties.mT_ChildBodyToJoint = tf.inverse();
  }

  // Set the properties of the soft body
  SoftBodyNode::UniqueProperties soft_properties;
  double mass = 0;
  {
      // Add Pointmass
      PointMass::Properties pointMass;
      const std::vector<CLine>& lines = VVOXELYZE_ADAPTER()->getVoxelMeshLines();
      const std::vector<CVertex>& vertices = VVOXELYZE_ADAPTER()->getVoxelMeshVertices();
      for(size_t i = 0; i < lines.size(); i++) {
          // Ver 0 --
          Vec3D<> ver0(vertices[lines[i].vi[0]].v.x + vertices[lines[i].vi[0]].DrawOffset.x,
                       vertices[lines[i].vi[0]].v.y + vertices[lines[i].vi[0]].DrawOffset.y,
                       vertices[lines[i].vi[0]].v.z + vertices[lines[i].vi[0]].DrawOffset.z);
          // Ver 1 --
          Vec3D<> ver1(vertices[lines[i].vi[1]].v.x + vertices[lines[i].vi[1]].DrawOffset.x,
                       vertices[lines[i].vi[1]].v.y + vertices[lines[i].vi[1]].DrawOffset.y,
                       vertices[lines[i].vi[1]].v.z + vertices[lines[i].vi[1]].DrawOffset.z);
          mass += ver1.Dist(ver0);
      }
      mass *= default_shape_density * default_skin_thickness;
      soft_properties = makeMeshProperties(
            Eigen::Vector3d(20,20,5), Eigen::Isometry3d::Identity(),
            mass, default_vertex_stiffness, default_edge_stiffness, default_soft_damping);
  }

  soft_properties.mKv = default_vertex_stiffness;
  soft_properties.mKe = default_edge_stiffness;
  soft_properties.mDampCoeff = default_soft_damping;

  // Create the Joint and Body pair
  SoftBodyNode::Properties body_properties(BodyNode::AspectProperties(name),
                                           soft_properties);
  SoftBodyNode* bn = chain->createJointAndBodyNodePair<JointType, SoftBodyNode>(
        parent, joint_properties, body_properties).second;

  // Zero out the inertia for the underlying BodyNode
  Inertia inertia;
  inertia.setMoment(1e-8*Eigen::Matrix3d::Identity());
  inertia.setMass(1e-8);
  bn->setInertia(inertia);

  // Make the shape transparent
  auto visualAspect = bn->getShapeNodesWith<VisualAspect>()[0]->getVisualAspect();
  Eigen::Vector4d color = visualAspect->getRGBA();
  color[3] = 0.4;
  visualAspect->setRGBA(color);

  return bn;
}
#endif

void setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color)
{
  // Set the color of all the shapes in the object
  for(std::size_t i=0; i < object->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = object->getBodyNode(i);
    auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();
    for(auto visualShapeNode : visualShapeNodes)
      visualShapeNode->getVisualAspect()->setColor(color);
  }
}

SkeletonPtr createBall()
{
  SkeletonPtr ball = Skeleton::create("rigid_ball");

  // Give the ball a body
  addRigidBody<FreeJoint>(ball, "rigid ball", Shape::ELLIPSOID);

  setAllColors(ball, dart::Color::Red());

  return ball;
}

SkeletonPtr createRigidChain()
{
  SkeletonPtr chain = Skeleton::create("rigid_chain");

  // Add bodies to the chain
  BodyNode* bn = addRigidBody<FreeJoint>(chain, "rigid box 1", Shape::BOX);
  bn = addRigidBody<BallJoint>(chain, "rigid cyl 2", Shape::CYLINDER, bn);
  bn = addRigidBody<BallJoint>(chain, "rigid box 3", Shape::BOX, bn);

  setAllColors(chain, dart::Color::Orange());

  return chain;
}

SkeletonPtr createRigidRing()
{
  SkeletonPtr ring = Skeleton::create("rigid_ring");

  // Add bodies to the ring
  BodyNode* bn = addRigidBody<FreeJoint>(ring, "rigid box 1", Shape::BOX);
  bn = addRigidBody<BallJoint>(ring, "rigid cyl 2", Shape::CYLINDER, bn);
  bn = addRigidBody<BallJoint>(ring, "rigid box 3", Shape::BOX, bn);
  bn = addRigidBody<BallJoint>(ring, "rigid cyl 4", Shape::CYLINDER, bn);
  bn = addRigidBody<BallJoint>(ring, "rigid box 5", Shape::BOX, bn);
  bn = addRigidBody<BallJoint>(ring, "rigid cyl 6", Shape::CYLINDER, bn);

  setAllColors(ring, dart::Color::Blue());

  return ring;
}

SkeletonPtr createSoftBody()
{
  SkeletonPtr soft = Skeleton::create("soft");

  // Add a soft body
  BodyNode* bn = addSoftBody<FreeJoint>(soft, "soft box", SOFT_BOX);

  // Add a rigid collision geometry and inertia
  double width = default_shape_height, height = 2*default_shape_width;
  Eigen::Vector3d dims(width, width, height);
  dims *= 0.6;
  std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(dims);
  bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);

  Inertia inertia;
  inertia.setMass(default_shape_density * box->getVolume());
  inertia.setMoment(box->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);

  setAllColors(soft, dart::Color::Fuchsia());

  return soft;
}

#ifdef DART_VOXEL_MESH
SkeletonPtr createSoftVoxelMesh()
{
    SkeletonPtr soft = Skeleton::create("softVoxel");

    // Add a soft body
    BodyNode* bn = addSoftVoxelBody<FreeJoint>(soft, "soft voxel");

    // Add a rigid collision geometry and inertia
    double width = default_shape_height, height = 2*default_shape_width;
    Eigen::Vector3d dims(width, width, height);
    dims *= 0.6;
    std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(dims);
    bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);

    Inertia inertia;
    inertia.setMass(default_shape_density * box->getVolume());
    inertia.setMoment(box->computeInertia(inertia.getMass()));
    bn->setInertia(inertia);

    setAllColors(soft, dart::Color::Fuchsia());

    return soft;
}

SkeletonPtr loadShadowHand()
{
#if 0
    SkeletonPtr shadow_hand = dart::utils::SdfParser::readSkeleton(
          DART_DATA_PATH"sdf/shadow_hand/shadow_hand.sdf"); // THIS DOES NOT WORK, THE MODEL LINKS FALL DOWN ALL!!!
#else
    /* !NOTE:
     * This must resort to conversiong of xacro to urdf file since intially,
     * Shadow Hand package does not provide an urdf file!
     *
     * 1. source devel/setup.sh in my_arm catkin package folder (Local package)
     * 2. XACRO => URDF: http://docs.ros.org/diamondback/api/sr_hand/html/index.html
     *    (xacro.py is deprecated, replaced by just xacro)
          rosrun xacro xacro --inorder -o `rospack find my_arm`/models/shadow_hand/robots/shadowhand_motor.urdf `rospack find my_arm`/models/shadow_hand/robots/shadowhand_motor.urdf.xacro
       3. Since DART does not understand ros package (package://my_arm/<path_to_stl> file in link's geometry mesh tag in .urdf)
          => Replace them with absolute path to the stl file
     */
    //
    // Load the Skeleton from a file
    dart::utils::DartLoader loader;
    SkeletonPtr shadow_hand = loader.parseSkeleton(DART_DATA_PATH"urdf/shadow_hand/shadow_hand.urdf");
    shadow_hand->setName("ShadowHand");
#endif
#if 0
    // Position its base in a reasonable way
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
    shadow_hand->getJoint(0)->setTransformFromParentBodyNode(tf);
    tf.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
    shadow_hand->getJoint(1)->setTransformFromParentBodyNode(tf);

#if 1
    std::size_t numJoints = shadow_hand->getNumJoints();
    std::size_t numDofs = shadow_hand->getNumDofs();
    std::cout <<"JOINT:" << std::endl;
    for(size_t i = 0; i < numJoints; i++) {
        std::cout << shadow_hand->getJoint(i)->getName() << std::endl;
        shadow_hand->getJoint(i)->setDampingCoefficient(0, 0.5);
    }
    std::cout <<"DOF:" << std::endl;
    for(size_t i = 0; i < numDofs; i++) {
        std::cout << shadow_hand->getDof(i)->getName() << std::endl;
        shadow_hand->getDof(i)->setRestPosition(0);
    }
#endif
#endif

    // Get it into a useful configuration
    //shadow_hand->getDof(1)->setPosition(0);
    //shadow_hand->getDof(2)->setPosition(0);

    return shadow_hand;
}
#endif

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createManipulator()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr manipulator =
      loader.parseSkeleton(DART_DATA_PATH"urdf/KR5/KR5 sixx R650.urdf");
  manipulator->setName("manipulator");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(-0.65, 0.0, 0.0);
  manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

  // Get it into a useful configuration
  manipulator->getDof(1)->setPosition(140.0 * M_PI / 180.0);
  manipulator->getDof(2)->setPosition(-140.0 * M_PI / 180.0);

#if 1
    std::size_t numJoints = manipulator->getNumJoints();
    std::size_t numDofs = manipulator->getNumDofs();
    std::cout <<"JOINT:" << std::endl;
    for(size_t i = 0; i < numJoints; i++) {
        std::cout << manipulator->getJoint(i)->getName() << std::endl;
    }
    std::cout <<"DOF:" << std::endl;
    for(size_t i = 0; i < numDofs; i++) {
        std::cout << manipulator->getDof(i)->getName() << std::endl;
    }
#endif

  return manipulator;
}

SkeletonPtr createHybridBody()
{
  SkeletonPtr hybrid = Skeleton::create("hybrid");

  // Add a soft body
  BodyNode* bn = addSoftBody<FreeJoint>(hybrid, "soft sphere", SOFT_ELLIPSOID);

  // Add a rigid body attached by a WeldJoint
  bn = hybrid->createJointAndBodyNodePair<WeldJoint>(bn).second;
  bn->setName("rigid box");

  double box_shape_height = default_shape_height;
  std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
        box_shape_height*Eigen::Vector3d::Ones());
  bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(box_shape_height/2.0, 0, 0);
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  Inertia inertia;
  inertia.setMass(default_shape_density * box->getVolume());
  inertia.setMoment(box->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);

  setAllColors(hybrid, dart::Color::Green());

  return hybrid;
}

SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  BodyNode* bn = ground->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_ground_width, default_ground_width,
                        default_wall_thickness));
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 1.0, 1.0));

  return ground;
}

SkeletonPtr createWall()
{
  SkeletonPtr wall = Skeleton::create("wall");

  BodyNode* bn = wall->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_wall_thickness, default_ground_width,
                        default_wall_height));
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(
        (default_ground_width + default_wall_thickness)/2.0, 0.0,
        (default_wall_height  - default_wall_thickness)/2.0);
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  bn->setRestitutionCoeff(0.2);

  return wall;
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
  WorldPtr world = std::make_shared<World>();
#endif
  world->addSkeleton(createGround());
  world->addSkeleton(createWall());
  //
#ifdef DART_VOXEL_MESH
  VVOXELYZE_ADAPTER()->initVoxelyze(_node_handle, false);
  QThread::msleep(1000);
  MyWindow window(world, createSoftVoxelMesh(), loadShadowHand());
#else
  MyWindow window(world, createBall(), createSoftBody(), createHybridBody(),
                  createRigidChain(), createRigidRing());
#endif

#ifdef ROBOT_LEAP_HANDS
    // !!! REQUIRE EARLIER ros::init() !!!
    // LEAP HANDS --
    VLEAP_INSTANCE()->initLeapMotion(_node_handle);
    ros::Subscriber _leap_listener = _node_handle->subscribe(CLEAP_HANDS_TOPIC, 1000,
                                                         &MyWindow::leapCallback, &window);
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
  glutIdleFunc(&gbHandleRosCallback);
  glutMainLoop();
}
