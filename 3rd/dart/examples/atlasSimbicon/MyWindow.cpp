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

#include "examples/atlasSimbicon/MyWindow.hpp"

#define CDART_SOFT_GROUND_NAME  ("DartSoftGround")
#define CROBOT_POS_X            (1)
#define CROBOT_POS_Y            (1.5)
#define CROBOT_POS_Z            (1)

#define CDART_SOFT_GROUND_POS_X (0)
#define CDART_SOFT_GROUND_POS_Y (0.1)
#define CDART_SOFT_GROUND_POS_Z (0)

//==============================================================================
MyWindow::MyWindow(const WorldPtr& world)
  : SoftSimWindow()
{
  setWorld(world);
  mForce = Eigen::Vector3d::Zero();
  mImpulseDuration = 0.0;
  dart::gui::SoftSimWindow::mSimulating  = false;
  dart::gui::SoftSimWindow::mShowMarkers = false;
  dart::gui::SoftSimWindow::mShowPointMasses = false;
  dart::gui::SoftSimWindow::mShowMeshs = false;

  //1- Collision Detector
  _mCollisionDetector = world->getConstraintSolver()->getCollisionDetector();

  //2- Load Atlas Robot
  loadRobot();// mRobot loaded here-in

  //3- Create Robot Controller
  mController = new Controller(mRobot, world->getConstraintSolver());

  //4- Voxel Object
  updateSoftGround(); /* softVoxel.skel: world->getSkeleton(1); */

  //cout << "TimeStep: " << mWorld->getTimeStep() << endl;
  //mWorld->setTimeStep(0.0005);
}

//==============================================================================
MyWindow::~MyWindow()
{
  delete mController;
}

//==============================================================================
void MyWindow::timeStepping()
{
  // External force
  mWorld->getSkeleton("drc_skeleton")->getBodyNode("pelvis")->addExtForce(
        mForce);

  // Internal force
  mController->update(mWorld->getTime());

  // simulate one step
  mWorld->step();

  // for perturbation test
  mImpulseDuration--;
  if (mImpulseDuration <= 0)
  {
    mImpulseDuration = 0;
    mForce.setZero();
  }

  // -----------------------------
  doIdleTasks();
#ifdef DAT_SOFT_GROUND_AS_VOXEL
  determineRobotAndSoftGroundCollision();
#endif
}

//==============================================================================
void MyWindow::displayTimer(int _val)
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

void MyWindow::drawSkels()
{
//  glEnable(GL_LIGHTING);
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
    drawSkeleton(mWorld->getSkeleton(i).get());

  // draw arrow
    if (mImpulseDuration > 0)
    {
        Eigen::Vector3d poa
            =  mWorld->getSkeleton("drc_skeleton")->getBodyNode(
                 "pelvis")->getTransform()
               * Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d start = poa - mForce / 500.0;
        double len = mForce.norm() / 500.0;
        dart::gui::drawArrow3D(start, mForce, len, 0.05, 0.1);
    }

  // VOXEL MESH --
#ifdef VOXELYZE_PURE
  if(RobotVoxelyzeAdapter::checkInstance()) {
      VVOXELYZE_ADAPTER()->drawVoxelMesh();
  }
#endif
}

//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  switch (_key)
  {
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
  case 'i':  // print debug information
    mController->printDebugInfo();
    break;
  case 'v':  // show or hide markers
    mShowMarkers = !mShowMarkers;
    break;
  case 'a':  // upper right force
    mForce[0] = 500;
    mImpulseDuration = 100;
    std::cout << "push forward" << std::endl;
    break;
  case 's':  // upper right force
    mForce[0] = -500;
    mImpulseDuration = 100;
    std::cout << "push backward" << std::endl;
    break;
  case 'd':  // upper left force
    mForce[2] = 500;
    mImpulseDuration = 100;
    std::cout << "push right" << std::endl;
    break;
  case 'f':  // upper left force
    mForce[2] = -500;
    mImpulseDuration = 100;
    std::cout << "push left" << std::endl;
    break;
  default:
    Win3D::keyboard(_key, _x, _y);
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y, mWorld->getTime());

  glutPostRedisplay();
}

void MyWindow::loadRobot()
{
    //  SkeletonPtr atlas = SoftSdfParser::readSkeleton(
    //        DART_DATA_PATH"sdf/atlas/atlas_v3_no_head.sdf");
    mRobot = SdfParser::readSkeleton(
          DART_DATA_PATH"sdf/atlas/atlas_v3_no_head_soft_feet.sdf");
                //DART_DATA_PATH"sdf/shadow_hand/shadow_hand.sdf");
    std::string test = mWorld->addSkeleton(mRobot);

    // Set initial configuration for Atlas robot
    VectorXd q = mRobot->getPositions();
    q[0] = -0.5 * constantsd::pi();
    q[3] = CROBOT_POS_X;
    q[4] = CROBOT_POS_Y;
    q[5] = CROBOT_POS_Z;
    mRobot->setPositions(q);
}

#ifdef DART_VOXEL_MESH
SoftBodyNode::UniqueProperties MyWindow::makeMeshProperties(
                                const Eigen::Vector3d& _size,
                                const Eigen::Isometry3d& _localTransform,
                                double _totalMass,
                                double _vertexStiffness,
                                double _edgeStiffness,
                                double _dampingCoeff)
{
    SoftBodyNode::UniqueProperties properties(
                  _vertexStiffness, _edgeStiffness, _dampingCoeff);
#if defined VOX_CAD
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
#elif defined VOXELYZE_PURE
    //----------------------------------------------------------------------------
    // Point masses
    //----------------------------------------------------------------------------
    // Number of point masses
    const std::vector<float>& vertices = VVOXELYZE_ADAPTER()->getVoxelMeshVertices();
    const std::vector<int>& lines      = VVOXELYZE_ADAPTER()->getVoxelMeshLines();
    const std::vector<int>& quads      = VVOXELYZE_ADAPTER()->getVoxelMeshQuads();
    std::size_t nPointMasses = vertices.size()/3;
    properties.mPointProps.resize(nPointMasses);

    // Mass per vertices
    double pointMass = _totalMass / nPointMasses;

    // Resting positions for each point mass
    std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                            Eigen::Vector3d::Zero());

    for(size_t i = 0; i < nPointMasses; i++) {
        restingPos[i] = _size.cwiseProduct(Eigen::Vector3d(vertices[3*i], vertices[3*i+1], vertices[3*i+2])) * 0.5;
    }

    // Point masses
    for (std::size_t i = 0; i < nPointMasses; ++i) {
        properties.mPointProps[i].mX0   = _localTransform * restingPos[i];
        properties.mPointProps[i].mMass = pointMass;
    }

    //----------------------------------------------------------------------------
    // Edges
    //----------------------------------------------------------------------------
    size_t lineCount = lines.size()/2;
    for(size_t i = 0; i < lineCount; i++) {
        properties.connectPointMasses(lines[2*i], lines[2*i+1]);
    }

    //----------------------------------------------------------------------------
    // Faces
    //----------------------------------------------------------------------------
    //
    Eigen::Vector3i face;
    int qCount = quads.size()/4;
    for(size_t i = 0; i < qCount; i++) {
        face[0] = quads[4*i];
        face[1] = quads[4*i+1];
        face[2] = quads[4*i+2];
        properties.addFace(face);

        face[0] = quads[4*i+2];
        face[1] = quads[4*i+3];
        face[2] = quads[4*i];
        properties.addFace(face);
    }
#endif
    return properties;
}

/// Add a soft body with the specified Joint type to a chain
template<class JointType>
BodyNode* MyWindow::createSoftVoxelBody(const SkeletonPtr& chain, const std::string& name,
                                        BodyNode* parent)
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
#ifdef VOX_CAD
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
#elif defined VOXELYZE_PURE
      const std::vector<int>& lines      = VVOXELYZE_ADAPTER()->getVoxelMeshLines();
      const std::vector<float>& vertices = VVOXELYZE_ADAPTER()->getVoxelMeshVertices();
      int lCount = lines.size()/2;
      for(int i = 0; i < lCount; i++) {
          // Ver 0 --
          Vec3D<> ver0(vertices[3*lines[2*i]], vertices[3*lines[2*i]+1], vertices[3*lines[2*i]+2]);
          // Ver 1 --
          Vec3D<> ver1(vertices[3*lines[2*i+1]], vertices[3*lines[2*i+1]+1], vertices[3*lines[2*i+1]+2]);
          mass += ver1.Dist(ver0);

          static bool test = false;
          //if(!test)
          {
              //cout << ver0.getX() << "-" << ver0.getY() << "-" << ver0.getZ() << endl;
              //cout << ver1.getX() << "-" << ver1.getY() << "-" << ver1.getZ() << endl;
              test = true;
          }
      }
#endif
      mass *= default_shape_density * default_skin_thickness;
      QVector3D size = VVOXELYZE_ADAPTER()->getVoxelMeshSize();
      soft_properties = makeMeshProperties(
            Eigen::Vector3d(size.x(), size.y(), size.z()), Eigen::Isometry3d::Identity(),
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
  dart::dynamics::Inertia inertia;
  inertia.setMoment(1e-8*Eigen::Matrix3d::Identity());
  inertia.setMass(1e-8);
  bn->setInertia(inertia);

  // Make the shape transparent
  auto visualAspect = bn->getShapeNodesWith<VisualAspect>()[0]->getVisualAspect();
  Eigen::Vector4d color = visualAspect->getRGBA();
  color[3] = 0.8;
  visualAspect->setRGBA(color);

  return bn;
}

void MyWindow::setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color)
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

void MyWindow::updateSoftGround()
{
    // 1 - LOAD NEW SOFT GROUND FROM SOFT VOXEL
    //
    SkeletonPtr newSoftGround = Skeleton::create(CDART_SOFT_GROUND_NAME);

    //Eigen::Vector6d positions(Eigen::Vector6d::Zero());
    //positions[4] = 1;
    //mOriginalSoftVoxelBody->setPositions(positions);

    // Add a soft body
    BodyNode* bn = createSoftVoxelBody<FreeJoint>(newSoftGround, "Dart Soft Voxel");
    setAllColors(newSoftGround, dart::Color::Orange());

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    //Eigen::Vector3d axis = Eigen::Vector3d::UnitX(); // Any vector in the X/Y plane can be used
    tf.translation() = Eigen::Vector3d(CDART_SOFT_GROUND_POS_X,
                                       CDART_SOFT_GROUND_POS_Y,
                                       CDART_SOFT_GROUND_POS_Z);
    newSoftGround->getJoint(0)->setTransformFromParentBodyNode(tf);

    // ----------------------------------------------------------------------------------------------
    // 2 - REPLACE THE CURRENT SOFT GROUND WITH THE NEW
    //
    mSoftGroundMutex.lock();
    //std::cout << "Rebuild Dart soft voxel body" << std::endl;
    if(mSoftGround != nullptr) {
        mWorld->removeSkeleton(mSoftGround);
        mSoftGround.reset();
    }

    if(newSoftGround != nullptr) {
        mWorld->addSkeleton(newSoftGround);
        mSoftGround = newSoftGround;
    }
    mSoftGroundMutex.unlock();

    return;
}

void MyWindow::doIdleTasks()
{
#ifdef DAT_SOFT_GROUND_AS_VOXEL
    if(RobotVoxelyzeAdapter::checkInstance()) {
#if 0
        QFuture<void> future = QtConcurrent::run(this, &MyWindow::updateSoftGround);
#else
        updateSoftGround();
#endif
        // ----------------------------------------------------------------
        //VVOXELYZE_ADAPTER()->doVoxelyzeTimeStep();
        VVOXELYZE_ADAPTER()->updateVoxelMesh();
    }
#endif
    // --------------------------------------------------------------------
    // HANDLE ROS CALLBACKS --
    //std::cout << "Ros spinning once!";
    ros::spinOnce();
}

void MyWindow::determineRobotAndSoftGroundCollision()
{
    auto shapeNodes       = mRobot->getRootBodyNode(0)->getShapeNodesWith<dart::dynamics::VisualAspect>();
    const auto& meshShape = static_cast<const MeshShape*>(shapeNodes[0]->getShape().get());
    aiMesh** mesh         = meshShape->getMesh()->mMeshes;
    //cout << "Hand Volume: " << meshShape->getVolume();
    // Look through the collisions to see if the new object would start in
    // collision with something
    auto worldCollisionGroup      = mWorld->getConstraintSolver()->getCollisionGroup();
    auto softGroundCollisionGroup = _mCollisionDetector->createCollisionGroup(mSoftGround.get());
    auto robotCollisionGroup      = _mCollisionDetector->createCollisionGroup(mRobot.get());

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collision = softGroundCollisionGroup->collide(robotCollisionGroup.get(), option, &result);

    size_t collisionCount = result.getNumContacts();
    if(collisionCount == 0) {
      return;
    }

    //cout << "Collision Count" << collisionCount << endl;
    for(size_t i = 0; i < collisionCount; ++i)
    {
        const dart::collision::Contact& contact = result.getContact(i);
#ifdef VOXELYZE_PURE
        if(i == 0) {
          VVOXELYZE_ADAPTER()->updateVoxelCollisionInfo(QVector3D(contact.point[0], contact.point[1], contact.point[2]),
                                                        QVector3D(contact.force[0], contact.force[1], contact.force[2]));
          //break;
        }
#endif
        //cout << "-" << contact.point[0] << "-" << contact.point[1] << "-" << contact.point[2] << endl
        if(contact.force[0] != 0 || contact.force[1] != 0 || contact.force[2] != 0)
            cout  << "Force: " << contact.force[0] << "-" << contact.force[1] << "-" << contact.force[2] << endl;
    }
}

#endif // #ifdef DART_VOXEL_MESH
