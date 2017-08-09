#ifndef MY_ARM_DART_UTILS_H
#define MY_ARM_DART_UTILS_H

#include <QtCore>
#include <QtWidgets/QApplication>
#include <QThread>
#include <QtGui/QVector3D>
#include <random>

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#if HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/bullet.hpp"
#endif

using namespace std;
using namespace Eigen;
using namespace dart::common;
using namespace dart::math;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

#define FORCE_ON_RIGIDBODY 25.0
#define FORCE_ON_VERTEX 1.00

static const double default_shape_density = 1000; // kg/m^3
static const double default_shape_height  = 0.1;  // m
static const double default_shape_width   = 0.03; // m
static const double default_skin_thickness = 1e-3; // m
static const double default_start_height = 0.4;  // m

static const double minimum_start_v = 2.5; // m/s
static const double maximum_start_v = 4.0; // m/s
static const double default_start_v = 3.5; // m/s

static const double minimum_launch_angle = 30.0*M_PI/180.0; // rad
static const double maximum_launch_angle = 70.0*M_PI/180.0; // rad
static const double default_launch_angle = 45.0*M_PI/180.0; // rad

static const double maximum_start_w = 6*M_PI; // rad/s
static const double default_start_w = 3*M_PI;  // rad/s

static const double ring_spring_stiffness = 0.5;
static const double ring_damping_coefficient = 0.05;
static const double default_damping_coefficient = 0.001;

static const double default_ground_width = 2;
static const double default_wall_thickness = 0.1;
static const double default_wall_height = 1;
static const double default_spawn_range = 0.9*default_ground_width/2;

static const double default_restitution = 0.6;

static const double default_vertex_stiffness = 500.0;
static const double default_edge_stiffness = 0.0;
static const double default_soft_damping = 5.0;

class DartUtils {

    enum SoftShapeType {
        SOFT_BOX = 0,
        SOFT_CYLINDER,
        SOFT_ELLIPSOID
    };

public:
    static WorldPtr initializeDartWorld(const Uri& uri, bool isSkeletonWorld = true);
    static void gbHandleRosCallback();
    static void setupRing(const SkeletonPtr& ring);

    /// Add a rigid body with the specified Joint type to a chain
    template<class JointType>
    static BodyNode* createRigidBody(const SkeletonPtr& chain, const std::string& name,
                                     Shape::ShapeType type, BodyNode* parent = nullptr);

    /// Add a soft body with the specified Joint type to a chain
    template<class JointType>
    static BodyNode* createSoftBody(const SkeletonPtr& chain, const std::string& name,
                                    SoftShapeType type, BodyNode* parent = nullptr);

    static SoftBodyNode::UniqueProperties makeMeshProperties(
                                          const Eigen::Vector3d& _size,
                                          const Eigen::Isometry3d& _localTransform,
                                          double _totalMass,
                                          double _vertexStiffness,
                                          double _edgeStiffness,
                                          double _dampingCoeff);

    /// Add a soft body with the specified Joint type to a chain
    template<class JointType>
    static BodyNode* createSoftVoxelBody(const SkeletonPtr& chain, const std::string& name,
                                         BodyNode* parent = nullptr);

    static void setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color);

    static SkeletonPtr createBall();
    static SkeletonPtr createRigidChain();
    static SkeletonPtr createRigidRing();
    static SkeletonPtr createSoftBody();

    static SkeletonPtr createSoftVoxelMesh();
    static SkeletonPtr loadShadowHand();

    static SkeletonPtr createFloor();
    static SkeletonPtr createManipulator(const Uri& uri);
    static SkeletonPtr createHybridBody();
    static SkeletonPtr createGround();
    static SkeletonPtr createWall();
};
#endif // MY_ARM_DART_UTIL_H
