#include <functional> // std::bind
#include <QThread>
#include <QVector3D>
#include "Rviz/VMarker.h"
#include "RobotDartAdapter.h"

RobotDartAdapter* RobotDartAdapter::_instance = nullptr;
RobotDartAdapter* RobotDartAdapter::getInstance()
{
    if(_instance == nullptr) {
        _instance = new RobotDartAdapter();
    }

    return _instance;
}

#define UPDATE_VOXEL_MESH_USING_LOCAL_TIMER
RobotDartAdapter::RobotDartAdapter():
                      _pMutex(new QMutex(QMutex::Recursive))
{
}

RobotDartAdapter::~RobotDartAdapter()
{
    _pMutex->tryLock(500);
    _pMutex->unlock(); // futile if tryLock() failed!
    delete _pMutex;
}

void RobotDartAdapter::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

void RobotDartAdapter::initDart(int argc, char* argv[])
{
#if 1 // Ducta
    // load a skeleton file
    // create and initialize the world
    dart::simulation::WorldPtr myWorld
        = dart::utils::SkelParser::readWorld(
          DART_DATA_PATH"skel/softBodies.skel");
    assert(myWorld != nullptr);
#endif

    // Create Left Leg skeleton
    dart::dynamics::SkeletonPtr LeftLegSkel = dart::dynamics::Skeleton::create();

    double mass = 1.0;

    // BodyNode 1: Left Hip Yaw (LHY)
    dart::dynamics::BodyNode::Properties body;
    body.mName = "LHY";
    dart::dynamics::ShapePtr shape(
          new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));
    body.mInertia.setMass(mass);

    dart::dynamics::RevoluteJoint::Properties joint;
    joint.mName = "LHY";
    joint.mAxis = Eigen::Vector3d(0.0, 0.0, 1.0);
    joint.mPositionLowerLimits[0] = -dart::math::constantsd::pi();
    joint.mPositionUpperLimits[0] =  dart::math::constantsd::pi();

    // You can get the newly created Joint and BodyNode pointers like this
    std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> pair =
        LeftLegSkel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          nullptr, joint, body);
    pair.second->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape);
    dart::dynamics::BodyNode* parent = pair.second;

    // BodyNode 2: Left Hip Roll (LHR) whose parent is: LHY
    body = dart::dynamics::BodyNode::Properties(); // create a fresh properties container
    body.mName = "LHR";
    shape = dart::dynamics::ShapePtr(
          new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));

    joint.mName = "LHR";
    joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 0.5);

    // You can get the specific type of Joint Pointer instead of just a basic Joint pointer
    std::pair<dart::dynamics::RevoluteJoint*, dart::dynamics::BodyNode*> pair1 =
    LeftLegSkel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          parent, joint, body);
    pair1.first->setAxis(Eigen::Vector3d(1.0, 0.0, 0.0));
    auto shapeNode1 = pair1.second->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape);
    shapeNode1->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
    pair1.second->setLocalCOM(shapeNode1->getRelativeTranslation());
    pair1.second->setMass(mass);

    // BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR
    body = dart::dynamics::BodyNode::Properties(); // create a fresh properties container
    body.mName = "LHP";
    shape = dart::dynamics::ShapePtr(
          new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));

    joint.mName = "LHP";
    joint.mAxis = Eigen::Vector3d(0.0, 1.0, 0.0);
    joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 1.0);

    // Or you can completely ignore the return value of this function
    std::pair<dart::dynamics::RevoluteJoint*, dart::dynamics::BodyNode*> pair2 =
    LeftLegSkel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          LeftLegSkel->getBodyNode(1), joint, body);
    auto shapeNode2 = pair2.second->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape);
    shapeNode2->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
    pair2.second->setLocalCOM(shapeNode2->getRelativeTranslation());
    pair2.second->setMass(mass);

    // Window stuff
    MyWindow window(LeftLegSkel);
    window.setWorld(myWorld);
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Skeleton example");
    glutMainLoop();
}


// ###########################################################################################
void MyWindow::draw()
{
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  drawSkeleton(skel.get());
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  static bool inverse = false;
  static const double dDOF = 0.1;
  switch (_key) {
    case '-': {
      inverse = !inverse;
    } break;
    case '1':
    case '2':
    case '3': {
      std::size_t dofIdx = _key - 49;
      Eigen::VectorXd pose = skel->getPositions();
      pose(dofIdx) = pose(dofIdx) + (inverse ? -dDOF : dDOF);
      skel->setPositions(pose);
      std::cout << "Updated pose DOF " << dofIdx << ": " << pose.transpose()
                << std::endl;
      glutPostRedisplay();
    } break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}
