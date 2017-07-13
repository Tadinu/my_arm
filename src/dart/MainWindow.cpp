#include "MainWindow.h"

#ifdef DART_VOXEL_MESH
MyWindow::MyWindow(const WorldPtr& world,
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
MyWindow::MyWindow(const WorldPtr& world, const SkeletonPtr& ball,
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
void MyWindow::timeStepping()
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

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key)
    {
#ifdef DART_VOXEL_MESH
    case '0':
        //addObject(mOriginalSoftVoxelBody->clone());
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

void MyWindow::drawWorld() const
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

#ifdef VOXELYZE_PURE
    // VOXEL MESH --
    if(RobotVoxelyzeAdapter::checkInstance()) {
        VVOXELYZE_ADAPTER()->drawVoxelMesh();
    }
#endif

    // SHADOW HAND --
    drawSkeleton(mShadowHand.get());

#if 0
    // WORLD --
    SimWindow::drawWorld();
#endif
}

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

void MyWindow::leapCallback(const visualization_msgs::MarkerArray&)
{
    determineHandArrangmentOnLeapHands();

    auto shapeNodes       = mShadowHand->getRootBodyNode(0)->getShapeNodesWith<dart::dynamics::VisualAspect>();
    const auto& meshShape = static_cast<const MeshShape*>(shapeNodes[0]->getShape().get());
    //aiMesh** mesh         = meshShape->getMesh()->mMeshes;

#ifdef VOXELYZE_PURE
    //joint = mShadowHand->getJoint(i);
    VVOXELYZE_ADAPTER()->updateVoxelCollisionInfo(QVector3D(0.1,0.1,0.1), QVector3D(0.1,0.1,0.1));
#endif
}

void MyWindow::updateJointPosition(int jointId, double position)
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

void MyWindow::determineHandArrangmentOnLeapHands()
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

/// Add an object to the world and toss it at the wall
bool MyWindow::addObject(const SkeletonPtr& object)
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
void MyWindow::addRing(const SkeletonPtr& ring)
{
    dartUtil::setupRing(ring);

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
void MyWindow::removeSkeleton(const SkeletonPtr& skel)
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

