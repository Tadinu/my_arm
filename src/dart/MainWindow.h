#ifndef MY_ARM_DART_MAIN_WINDOW_H
#define MY_ARM_DART_MAIN_WINDOW_H

#include "my_arm/KsGlobal.h"
#include "my_arm/RobotVoxelyzeAdapter.h"
#include "my_arm/RobotLeapAdapter.h"
#include "dart_utils.h"

class MyWindow : public dart::gui::SoftSimWindow
{
public:

#ifdef DART_VOXEL_MESH
    MyWindow(const WorldPtr& world,
             const SkeletonPtr& softVoxelBody,
             const SkeletonPtr& shadowHand);
#else
    MyWindow(const WorldPtr& world, const SkeletonPtr& ball,
             const SkeletonPtr& softBody, const SkeletonPtr& hybridBody,
             const SkeletonPtr& rigidChain, const SkeletonPtr& rigidRing);
#endif

    /// \brief
    void timeStepping() override;

    void keyboard(unsigned char key, int x, int y) override;

    void drawWorld() const override;
    void displayTimer(int _val) override;

    void leapCallback(const visualization_msgs::MarkerArray&);

    void updateJointPosition(int jointId, double position);

    void determineHandArrangmentOnLeapHands();
protected:

    /// Add an object to the world and toss it at the wall
    bool addObject(const SkeletonPtr& object);

    /// Add a ring to the world, and create a BallJoint constraint to ensure that
    /// it stays in a ring shape
    void addRing(const SkeletonPtr& ring);

    /// Remove a Skeleton and get rid of the constraint that was associated with
    /// it, if one existed
    void removeSkeleton(const SkeletonPtr& skel);

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
#endif // MY_ARM_DART_MAIN_WINDOW_H
