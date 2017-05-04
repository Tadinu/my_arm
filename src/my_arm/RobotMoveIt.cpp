#include <functional> // std::bind
#include <QThread>
#include <QVector3D>
#include "RobotMoveIt.h"
#include "KsGlobal.h"

// ROBOT TO RUN
#define CRUN_ROBOT (KsGlobal::VSHADOW_HAND_UR_ARM)

#define CROBOT_DESCRIPTION    ("robot_description")
#define CMAIN_ARM_GROUP_NAME  ("main_arm")
#define CMAIN_HAND_GROUP_NAME ("main_hand")

RobotMoveIt* RobotMoveIt::_instance = nullptr;
RobotMoveIt* RobotMoveIt::getInstance()
{
    if(_instance == nullptr) {
        _instance = new RobotMoveIt();
    }

    return _instance;
}

bool RobotMoveIt::checkInstance()
{
    return _instance != nullptr;
}

RobotMoveIt::RobotMoveIt():
                  _pMutex(new QMutex(QMutex::Recursive)),
                  _node_handle(nullptr)
{
}

RobotMoveIt::~RobotMoveIt()
{
    _pMutex->tryLock(500);
    _pMutex->unlock(); // infutile if tryLock() failed!
    delete _pMutex;
}

void RobotMoveIt::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

void RobotMoveIt::initMoveIt(boost::shared_ptr<ros::NodeHandle> node_handle)
{
    assert(node_handle);
    _node_handle = node_handle;

    // Start a service client
    ros::ServiceClient service_client = node_handle->serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    ros::Publisher robot_state_publisher =
        node_handle->advertise<moveit_msgs::DisplayRobotState>("my_arm_state", 1);

    while (!service_client.exists())
    {
      ROS_INFO("Waiting for service");
      sleep(1.0);
    }

    moveit_msgs::GetPositionIK::Request service_request;
    moveit_msgs::GetPositionIK::Response service_response;

    service_request.ik_request.group_name = CMAIN_ARM_GROUP_NAME;
    service_request.ik_request.pose_stamped.header.frame_id = KsGlobal::baseLinkName(CRUN_ROBOT);
    service_request.ik_request.pose_stamped.pose.position.x = 0.75;
    service_request.ik_request.pose_stamped.pose.position.y = 0.188;
    service_request.ik_request.pose_stamped.pose.position.z = 0.0;

    service_request.ik_request.pose_stamped.pose.orientation.x = 0.0;
    service_request.ik_request.pose_stamped.pose.orientation.y = 0.0;
    service_request.ik_request.pose_stamped.pose.orientation.z = 0.0;
    service_request.ik_request.pose_stamped.pose.orientation.w = 1.0;

    /* Call the service */
    service_client.call(service_request, service_response);
    ROS_INFO_STREAM(
        "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                   << service_response.error_code.val);

    /* Filling in a seed state */
    robot_model_loader::RobotModelLoader robot_model_loader(CROBOT_DESCRIPTION);
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(CMAIN_ARM_GROUP_NAME);

    /* Get the names of the joints in the right_arm*/
    service_request.ik_request.robot_state.joint_state.name = joint_model_group->getJointModelNames();

    /* Get the joint values and put them into the message, this is where you could put in your own set of values as
     * well.*/
    kinematic_state->setToRandomPositions(joint_model_group);
    kinematic_state->copyJointGroupPositions(joint_model_group,
                                             service_request.ik_request.robot_state.joint_state.position);

    /* Call the service again*/
    service_client.call(service_request, service_response);
    ROS_INFO_STREAM(
        "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                   << service_response.error_code.val);

    /* Check for collisions*/
    service_request.ik_request.avoid_collisions = true;

    /* Call the service again*/
    service_client.call(service_request, service_response);

    ROS_INFO_STREAM(
        "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                   << service_response.error_code.val);

    /* Visualize the result*/
    moveit_msgs::DisplayRobotState msg;
    kinematic_state->setVariableValues(service_response.solution.joint_state);
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    robot_state_publisher.publish(msg);

    // Sleep to let the message go through
    ros::Duration(2.0).sleep();
    return;
}

void RobotMoveIt::fetchRobotModelInfo()
{
    // Start
    // ^^^^^
    // Setting up to start using the RobotModel class is very easy. In
    // general, you will find that most higher-level components will
    // return a shared pointer to the RobotModel. You should always use
    // that when possible. In this example, we will start with such a
    // shared pointer and discuss only the basic API. You can have a
    // look at the actual code API for these classes to get more
    // information about how to use more features provided by these
    // classes.
    //
    // We will start by instantiating a
    // `RobotModelLoader`_
    // object, which will look up
    // the robot description on the ROS parameter server and construct a
    // :moveit_core:`RobotModel` for us to use.
    //
    // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
    robot_model_loader::RobotModelLoader robot_model_loader(CROBOT_DESCRIPTION);
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Moveit Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :moveit_core:`RobotState` that maintains the configuration
    // of the robot. We will set all joints in the state to their
    // default values. We can then get a
    // :moveit_core:`JointModelGroup`, which represents the robot
    // model for a particular group, e.g. the "right_arm" of the PR2
    // robot.
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(CMAIN_ARM_GROUP_NAME);

    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    // Get Joint Values
    // ^^^^^^^^^^^^^^^^
    // We can retreive the current set of joint values stored in the state for the right arm.
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Joint Limits
    // ^^^^^^^^^^^^
    // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
    /* Set one joint in the right arm outside its joint limit */
    joint_values[0] = 1.57;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // Forward Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // Now, we can compute forward kinematics for a set of random joint
    // values. Note that we would like to find the pose of the
    // "r_wrist_roll_link" which is the most distal link in the
    // "right_arm" of the robot.
    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("ra_wrist_1_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // We can now solve inverse kinematics (IK) for the right arm of the
    // PR2 robot. To solve IK, we will need the following:
    // * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain):
    // end_effector_state that we computed in the step above.
    // * The number of attempts to be made at solving IK: 5
    // * The timeout for each attempt: 0.1 s
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }

    // Get the Jacobian
    // ^^^^^^^^^^^^^^^^
    // We can also get the Jacobian from the :moveit_core:`RobotState`.
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,
                                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                 reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: " << jacobian);
}



void RobotMoveIt::openRobotHand(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(KsGlobal::VSHADOW_HAND_UR_ARM_JOINT_TOTAL - KsGlobal::VSHADOW_HAND_UR_ARM_WRJ2);
    int j = 0;
    for(int i = KsGlobal::VSHADOW_HAND_UR_ARM_WRJ2; i < KsGlobal::VSHADOW_HAND_UR_ARM_JOINT_TOTAL; i++) {
        posture.joint_names[j] = KsGlobal::CSHADOWHAND_UR_ARM_JOINTS[i];
        j++;
    }

    posture.points.resize(1);
    posture.points[0].positions.resize(posture.joint_names.size());
    for(int j = 0; j < posture.joint_names.size(); j++) {
        posture.points[0].positions[j] = 0;
    }
}

void RobotMoveIt::closeRobotHand(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(KsGlobal::VSHADOW_HAND_UR_ARM_JOINT_TOTAL - KsGlobal::VSHADOW_HAND_UR_ARM_WRJ2);
    int j = 0;
    for(int i = KsGlobal::VSHADOW_HAND_UR_ARM_WRJ2; i < KsGlobal::VSHADOW_HAND_UR_ARM_JOINT_TOTAL; i++) {
        posture.joint_names[j] = KsGlobal::CSHADOWHAND_UR_ARM_JOINTS[i];
        j++;
    }

    posture.points.resize(1);
    posture.points[0].positions.resize(posture.joint_names.size());
    for(int j = 0; j < posture.joint_names.size(); j++) {
        posture.points[0].positions[j] = 0.47;
    }
}

void RobotMoveIt::setupPickPlaceSettings()
{
    ros::Publisher pub_co  = _node_handle->advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    ros::Publisher pub_aco = _node_handle->advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

    ros::WallDuration(1.0).sleep();

    moveit::planning_interface::MoveGroupInterface group("right_arm");
    group.setPlanningTime(45.0);

    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_footprint";

    // remove pole
    co.id = "pole";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    // add pole
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = 0.7;
    co.primitive_poses[0].position.y = -0.4;
    co.primitive_poses[0].position.z = 0.85;
    co.primitive_poses[0].orientation.w = 1.0;
    pub_co.publish(co);

    // remove table
    co.id = "table";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    // add table
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
    co.primitive_poses[0].position.x = 0.7;
    co.primitive_poses[0].position.y = -0.2;
    co.primitive_poses[0].position.z = 0.175;
    pub_co.publish(co);

    co.id = "part";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    moveit_msgs::AttachedCollisionObject aco;
    aco.object = co;
    pub_aco.publish(aco);

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

    co.primitive_poses[0].position.x = 0.6;
    co.primitive_poses[0].position.y = -0.7;
    co.primitive_poses[0].position.z = 0.5;
    pub_co.publish(co);

    // wait a bit for ros things to initialize
    ros::WallDuration(1.0).sleep();

    pickObject(group);

    ros::WallDuration(1.0).sleep();

    placeObject(group);
}

void RobotMoveIt::pickObject(moveit::planning_interface::MoveGroupInterface &group)
{
    std::vector<moveit_msgs::Grasp> grasps;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "base_footprint";
    p.pose.position.x = 0.34;
    p.pose.position.y = -0.7;
    p.pose.position.z = 0.5;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::Grasp g;
    g.grasp_pose = p;

    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
    g.pre_grasp_approach.min_distance = 0.2;
    g.pre_grasp_approach.desired_distance = 0.4;

    g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.25;

    openRobotHand(g.pre_grasp_posture);

    closeRobotHand(g.grasp_posture);

    grasps.push_back(g);
    group.setSupportSurfaceName("table");
    group.pick("part", grasps);
}

void RobotMoveIt::placeObject(moveit::planning_interface::MoveGroupInterface &group)
{
    std::vector<moveit_msgs::PlaceLocation> loc;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "base_footprint";
    p.pose.position.x = 0.7;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.5;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::PlaceLocation g;
    g.place_pose = p;

    g.pre_place_approach.direction.vector.z = -1.0;
    g.post_place_retreat.direction.vector.x = -1.0;
    g.post_place_retreat.direction.header.frame_id = "base_footprint";
    g.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
    g.pre_place_approach.min_distance = 0.1;
    g.pre_place_approach.desired_distance = 0.2;
    g.post_place_retreat.min_distance = 0.1;
    g.post_place_retreat.desired_distance = 0.25;

    openRobotHand(g.post_place_posture);

    loc.push_back(g);
    group.setSupportSurfaceName("table");

    // add path constraints
    moveit_msgs::Constraints constr;
    constr.orientation_constraints.resize(1);
    moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
    ocm.link_name = "r_wrist_roll_link";
    ocm.header.frame_id = p.header.frame_id;
    ocm.orientation.x = 0.0;
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = M_PI;
    ocm.weight = 1.0;
    //  group.setPathConstraints(constr);
    group.setPlannerId("RRTConnectkConfigDefault");

    group.place("part", loc);
}
