#include "DartUtils.h"
#include "RobotVoxelyzeAdapter.h"

#define CSTR_DART_SOFT_BODY_NAME_DFT ("DartSoftBody")
#define CSTR_MANIPULATOR_NAME_DFT    ("Manipulator")
#define CSTR_SHADOW_HAND_NAME_DFT    ("ShadowHand")

void DartUtils::gbHandleRosCallback()
{
#if 1
    if(RobotVoxelyzeAdapter::checkInstance()) {
        //std::cout << "Rebuild Dart soft voxel body" << std::endl;
        //mWorld->removeSkeleton(mDartSoftBody);
        //mDartSoftBody.reset();
        //mDartSoftBody = loadSoftVoxelMesh();
        //mWorld->addSkeleton(mDartSoftBody);
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

WorldPtr DartUtils::initializeDartWorld(const Uri& uri, bool isSkeletonWorld)
{
    // Create and initialize the world --
    //
    WorldPtr myWorld = nullptr;
    if(isSkeletonWorld) {
        myWorld = dart::utils::SkelParser::readWorld(uri);
    }
    else {
        myWorld = dart::utils::SdfParser::readWorld(uri);
    }
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

    // Gravity --
    //
    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/1000);

    return myWorld;
}


void DartUtils::setupRing(const SkeletonPtr& ring)
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

/// Add a rigid body with the specified Joint type to a chain
template<class JointType>
BodyNode* DartUtils::createRigidBody(const SkeletonPtr& chain, const std::string& name,
                                     Shape::ShapeType type, BodyNode* parent)
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
    dart::dynamics::Inertia inertia;
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

/// Add a soft body with the specified Joint type to a chain
template<class JointType>
BodyNode* DartUtils::createSoftBody(const SkeletonPtr& chain, const std::string& name,
                                    SoftShapeType type, BodyNode* parent)
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
    dart::dynamics::Inertia inertia;
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

SoftBodyNode::UniqueProperties  DartUtils::makeMeshProperties(
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
    std::size_t nPointMasses = vertices.size();
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
    for(size_t i = 0; i < lines.size(); i++) {
        properties.connectPointMasses(lines[2*i], lines[2*i+1]);
    }

    //----------------------------------------------------------------------------
    // Faces
    //----------------------------------------------------------------------------
    //
    Eigen::Vector3i face;
    int qCount = quads.size()/4;
    for(int i = 0; i < qCount; i++) {
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
BodyNode*  DartUtils::createSoftVoxelBody(const SkeletonPtr& chain, const std::string& name,
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
        }
#endif
        mass *= default_shape_density * default_skin_thickness;
        soft_properties = makeMeshProperties(
              Eigen::Vector3d(20,20,5), Eigen::Isometry3d::Identity(),
              0.5/*mass*/, default_vertex_stiffness, default_edge_stiffness, default_soft_damping);
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
    color[3] = 0.4;
    visualAspect->setRGBA(color);

    return bn;
}

void  DartUtils::setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color)
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

SkeletonPtr DartUtils::createBall()
{
    SkeletonPtr ball = Skeleton::create("rigid_ball");

    // Give the ball a body
    createRigidBody<FreeJoint>(ball, "rigid ball", Shape::ELLIPSOID);

    setAllColors(ball, dart::Color::Red());

    return ball;
}

SkeletonPtr  DartUtils::createRigidChain()
{
    SkeletonPtr chain = Skeleton::create("rigid_chain");

    // Add bodies to the chain
    BodyNode* bn = createRigidBody<FreeJoint>(chain, "rigid box 1", Shape::BOX);
    bn = createRigidBody<BallJoint>(chain, "rigid cyl 2", Shape::CYLINDER, bn);
    bn = createRigidBody<BallJoint>(chain, "rigid box 3", Shape::BOX, bn);

    setAllColors(chain, dart::Color::Orange());

    return chain;
}

SkeletonPtr  DartUtils::createRigidRing()
{
    SkeletonPtr ring = Skeleton::create("rigid_ring");

    // Add bodies to the ring
    BodyNode* bn = createRigidBody<FreeJoint>(ring, "rigid box 1", Shape::BOX);
    bn = createRigidBody<BallJoint>(ring, "rigid cyl 2", Shape::CYLINDER, bn);
    bn = createRigidBody<BallJoint>(ring, "rigid box 3", Shape::BOX, bn);
    bn = createRigidBody<BallJoint>(ring, "rigid cyl 4", Shape::CYLINDER, bn);
    bn = createRigidBody<BallJoint>(ring, "rigid box 5", Shape::BOX, bn);
    bn = createRigidBody<BallJoint>(ring, "rigid cyl 6", Shape::CYLINDER, bn);

    setAllColors(ring, dart::Color::Blue());

    return ring;
}

SkeletonPtr  DartUtils::createSoftBody()
{
    SkeletonPtr soft = Skeleton::create("soft");

    // Add a soft body
    BodyNode* bn = createSoftBody<FreeJoint>(soft, "soft box", SOFT_BOX);

    // Add a rigid collision geometry and inertia
    double width = default_shape_height, height = 2*default_shape_width;
    Eigen::Vector3d dims(width, width, height);
    dims *= 0.6;
    std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(dims);
    bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);

    dart::dynamics::Inertia inertia;
    inertia.setMass(default_shape_density * box->getVolume());
    inertia.setMoment(box->computeInertia(inertia.getMass()));
    bn->setInertia(inertia);

    setAllColors(soft, dart::Color::Fuchsia());

    return soft;
}

SkeletonPtr DartUtils::createSoftVoxelMesh()
{
    SkeletonPtr soft = Skeleton::create("softVoxel");

    // Add a soft body
    BodyNode* bn = createSoftVoxelBody<FreeJoint>(soft, "soft voxel");
    setAllColors(soft, dart::Color::Fuchsia());

    return soft;
}

SkeletonPtr DartUtils::loadShadowHand()
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
#if 1
    // Position its base in a reasonable way
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX(); // Any vector in the X/Y plane can be used
    tf.rotate(Eigen::AngleAxisd(M_PI/2, axis));
    tf.translation() = Eigen::Vector3d(-0.05, 0.1, 0.07);
    shadow_hand->getJoint(0)->setTransformFromParentBodyNode(tf);
#if 0
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

SkeletonPtr DartUtils::createFloor()
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

SkeletonPtr DartUtils::createManipulator(const Uri& uri)
{
    // Load the Skeleton from a file
    dart::utils::DartLoader loader;
    SkeletonPtr manipulator = loader.parseSkeleton(uri);
    manipulator->setName(CSTR_MANIPULATOR_NAME_DFT);

#if 1
    //note that you have to create a Translation because multiplying a
    //Transform with a vector will _apply_ the transform to the vector
    Eigen::Translation<double, 3> translation(Vector3d(0.5, -1, 0.5));
    Eigen::Quaterniond rotation; rotation = AngleAxisd(-M_PI/2, Vector3d::UnitX());
    //Eigen::Transform<double, 3, Affine> combined =
    //      translation * rotation;

    // Position its base in a reasonable way
#if 1
    // Eigen::Isometry3d = Eigen::Translation<double, 3> * Eigen::Quaterniond;
    Eigen::Isometry3d tf = translation * rotation;
#else
    geometry_msgs::Pose pose;
    Eigen::Isometry3d tf  = Eigen::Isometry3d::Identity();
    //tf.translation() = trans_vec_A;
    tf::poseMsgToEigen(pose, tf);
#endif
    manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

    // Get it into a useful configuration
    manipulator->getDof(1)->setPosition(140.0 * M_PI / 180.0);
    manipulator->getDof(2)->setPosition(-140.0 * M_PI / 180.0);
#endif

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

SkeletonPtr DartUtils::createHybridBody()
{
    SkeletonPtr hybrid = Skeleton::create("hybrid");

    // Add a soft body
    BodyNode* bn = createSoftBody<FreeJoint>(hybrid, "soft sphere", SOFT_ELLIPSOID);

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

    dart::dynamics::Inertia inertia;
    inertia.setMass(default_shape_density * box->getVolume());
    inertia.setMoment(box->computeInertia(inertia.getMass()));
    bn->setInertia(inertia);

    setAllColors(hybrid, dart::Color::Green());

    return hybrid;
}

SkeletonPtr DartUtils::createGround()
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

SkeletonPtr DartUtils::createWall()
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
