#include <functional> // std::bind
#include <QThread>
#include <QVector3D>
#include "RobotMoveIt.h"
#include "KsGlobal.h"

// ROBOT TO RUN
#define CRUN_ROBOT (KsGlobal::VSHADOW_HAND_UR_ARM)

#define CROBOT_DESCRIPTION    ("robot_description")

// https://github.com/ros-planning/moveit_pr2/tree/hydro-devel/pr2_moveit_tutorials/planning/src


// sr_multi_moveit/sr_multi_moveit_config/config/generated_robot.srdf
#define CMAIN_ARM_GROUP_NAME           ("right_arm")
#define CMAIN_ARM_HAND_GROUP_NAME      ("right_arm_and_hand")
#define CMAIN_ARM_WRIST_GROUP_NAME     ("right_arm_and_wrist")

#define CMAIN_HAND_GROUP_NAME          ("right_hand")
#define CMAIN_HAND_GROUP_WRIST         ("rh_wrist")
#define CMAIN_HAND_GROUP_FIRST_FINGER  ("rh_first_finger")
#define CMAIN_HAND_GROUP_MIDDLE_FINGER ("rh_middle_finger")
#define CMAIN_HAND_GROUP_RING_FINGER   ("rh_ring_finger")
#define CMAIN_HAND_GROUP_LITTLE_FINGER ("rh_little_finger")
#define CMAIN_HAND_GROUP_THUMB         ("rh_thumb")
#define CMAIN_HAND_GROUP_FINGERS       ("rh_fingers")

// UR10.SRDF
// sr_interface/sr_multi_moveit/sr_multi_moveit_config/config/ur/ur10.srdf
// smart_grasping_sandbox/smart_grasp_moveit_config/config/ur10.srdf

// SHADOW_HAND.SRDF
// sr_interface/sr_moveit_hand_config/config/generated_shadowhand.srdf


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

    if(_kinematic_state)
        delete _kinematic_state;
    _kinematic_state = nullptr;
}

void RobotMoveIt::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

void RobotMoveIt::initMoveIt(ros::NodeHandle* node_handle)
{
    assert(node_handle);
    _node_handle = boost::shared_ptr<ros::NodeHandle>(node_handle);

    // Start a service client
    ros::ServiceClient service_client = _node_handle->serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    _robot_state_publisher = _node_handle->advertise<moveit_msgs::DisplayRobotState>("my_arm_state", 1);

    while (!service_client.exists())
    {
      ROS_INFO("Waiting for service");
      sleep(1.0);
    }

    moveit_msgs::GetPositionIK::Request service_request;
    moveit_msgs::GetPositionIK::Response service_response;

    service_request.ik_request.group_name = CMAIN_ARM_WRIST_GROUP_NAME;
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

    // ====================================================================================================================
    //
    /* Filling in a seed state */
    //
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
    _kinematic_model = robot_model_loader.getModel();
    ROS_INFO("++ Moveit Model frame: %s", _kinematic_model->getModelFrame().c_str());

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :moveit_core:`RobotState` that maintains the configuration
    // of the robot. We will set all joints in the state to their
    // default values. We can then get a
    // :moveit_core:`JointModelGroup`, which represents the robot
    // model for a particular group, e.g. the "right_arm" of the PR2
    // robot.
    _kinematic_state = new robot_state::RobotState(_kinematic_model);
    _kinematic_state->setToDefaultValues();
    _joint_model_group = _kinematic_model->getJointModelGroup(CMAIN_ARM_WRIST_GROUP_NAME);

    /* Get the names of the joints in the right_arm*/
    service_request.ik_request.robot_state.joint_state.name = _joint_model_group->getJointModelNames();

    /* Get the joint values and put them into the message, this is where you could put in your own set of values as
     * well.*/
    _kinematic_state->setToRandomPositions(_joint_model_group);
    _kinematic_state->copyJointGroupPositions(_joint_model_group,
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
    _kinematic_state->setVariableValues(service_response.solution.joint_state);
    robot_state::robotStateToRobotStateMsg(*_kinematic_state, msg.state);
    _robot_state_publisher.publish(msg);

    // Sleep to let the message go through
    ros::Duration(2.0).sleep();
    return;
}

void RobotMoveIt::fetchRobotModelInfo()
{
    assert(_joint_model_group);
    const std::vector<std::string> &joint_names = _joint_model_group->getJointModelNames();

    // Get Joint Values
    // ^^^^^^^^^^^^^^^^
    // We can retreive the current set of joint values stored in the state for the right arm.
    std::vector<double> joint_values;
    _kinematic_state->copyJointGroupPositions(_joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("++ Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Joint Limits
    // ^^^^^^^^^^^^
    // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
    /* Set one joint in the right arm outside its joint limit */
    joint_values[0] = 1.57;
    _kinematic_state->setJointGroupPositions(_joint_model_group, joint_values);

    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("++ Current state is " << (_kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    /* Enforce the joint limits for this state and check again*/
    _kinematic_state->enforceBounds();
    ROS_INFO_STREAM("++ Current state is " << (_kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // Forward Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // Now, we can compute forward kinematics for a set of random joint
    // values. Note that we would like to find the pose of the
    // "r_wrist_roll_link" which is the most distal link in the
    // "right_arm" of the robot.
    _kinematic_state->setToRandomPositions(_joint_model_group);
    const Eigen::Affine3d &end_effector_state = _kinematic_state->getGlobalLinkTransform("ra_wrist_1_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("++ Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("++ Rotation: " << end_effector_state.rotation());

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // We can now solve inverse kinematics (IK) for the right arm of the
    // robot. To solve IK, we will need the following:
    // * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain):
    // end_effector_state that we computed in the step above.
    // * The number of attempts to be made at solving IK: 5
    // * The timeout for each attempt: 0.1 s
    bool found_ik = _kinematic_state->setFromIK(_joint_model_group, end_effector_state, 10, 0.1);

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
      _kinematic_state->copyJointGroupPositions(_joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("++ Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }
    else
    {
      ROS_INFO("++ Did not find IK solution");
    }

    // Get the Jacobian
    // ^^^^^^^^^^^^^^^^
    // We can also get the Jacobian from the :moveit_core:`RobotState`.
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    _kinematic_state->getJacobian(_joint_model_group,
                                 _kinematic_state->getLinkModel(_joint_model_group->getLinkModelNames().back()),
                                 reference_point_position, jacobian);
    ROS_INFO_STREAM("++ Jacobian: " << jacobian);
    /* Visualize the result*/
    moveit_msgs::DisplayRobotState msg;
    robot_state::robotStateToRobotStateMsg(*_kinematic_state, msg.state);
    _robot_state_publisher.publish(msg);

    // _joint_model_group
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

    moveit::planning_interface::MoveGroupInterface group(CMAIN_HAND_GROUP_NAME);
    group.setPlanningTime(45.0);

    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = KsGlobal::baseLinkName(CRUN_ROBOT);

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
    p.header.frame_id = KsGlobal::baseLinkName(CRUN_ROBOT);
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
    g.pre_grasp_approach.direction.header.frame_id = "rh_wrist";
    g.pre_grasp_approach.min_distance = 0.2;
    g.pre_grasp_approach.desired_distance = 0.4;

    g.post_grasp_retreat.direction.header.frame_id = KsGlobal::baseLinkName(CRUN_ROBOT);
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.25;

    openRobotHand(g.pre_grasp_posture);

    closeRobotHand(g.grasp_posture);

    grasps.push_back(g);
    group.setSupportSurfaceName("table");
    group.pick("part", grasps);
    ROS_INFO("PICK OBJECT ------------------------>");
}

void RobotMoveIt::placeObject(moveit::planning_interface::MoveGroupInterface &group)
{
    std::vector<moveit_msgs::PlaceLocation> loc;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = KsGlobal::baseLinkName(CRUN_ROBOT);
    p.pose.position.x = 0.7;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.5;
    p.pose.orientation.x     = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::PlaceLocation g;
    g.place_pose = p;

    g.pre_place_approach.direction.vector.z = -1.0;
    g.post_place_retreat.direction.vector.x = -1.0;
    g.post_place_retreat.direction.header.frame_id = KsGlobal::baseLinkName(CRUN_ROBOT);
    g.pre_place_approach.direction.header.frame_id = "rh_wrist";
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
    ocm.link_name = "rh_wrist";
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
    ROS_INFO("PLACE OBJECT ------------------------>");
}

#if 0
void RobotMoveIt::doMotionPlanning()
{
#if 0
    ros::AsyncSpinner spinner(1);
    spinner.start();
#endif

    // BEGIN_TUTORIAL
    // Start
    // ^^^^^
    // Setting up to start using a planner is pretty easy. Planners are
    // setup as plugins in MoveIt! and you can use the ROS pluginlib
    // interface to load any planner that you want to use. Before we
    // can load the planner, we need two objects, a RobotModel
    // and a PlanningScene.
    // We will start by instantiating a
    // `RobotModelLoader`_
    // object, which will look up
    // the robot description on the ROS parameter server and construct a
    // :moveit_core:`RobotModel` for us to use.
    //
    // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :planning_scene:`PlanningScene` that maintains the state of
    // the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // We will get the name of planning plugin we want to load
    // from the ROS param server, and then load the planner
    // making sure to catch all exceptions.
    if (!_node_handle.getParam("planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, _node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
    }

    /* Sleep a little to allow time to startup rviz, etc. */
    ros::WallDuration sleep_time(15.0);
    sleep_time.sleep();

    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the PR2
    // specifying the desired pose of the end-effector as input.
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "torso_lift_link";
    pose.pose.position.x = 0.75;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // We will create the request as a constraint using a helper function available
    // from the
    // `kinematic_constraints`_
    // package.
    //
    // .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
    req.group_name = CMAIN_ARM_GROUP_NAME;
    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("rh_wrist", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    ros::Publisher display_publisher =
        _node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    sleep_time.sleep();

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(response.trajectory_start);
    const robot_state::JointModelGroup* _joint_model_group = robot_state.getJointModelGroup(CMAIN_ARM_GROUP_NAME);
    robot_state.setJointGroupPositions(_joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    // Now, setup a joint space goal
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values(7, 0.0);
    joint_values[0] = -2.0;
    joint_values[3] = -0.2;
    joint_values[5] = -0.15;
    goal_state.setJointGroupPositions(_joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, _joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Call the planner and visualize the trajectory
    /* Re-construct the planning context */
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    /* Call the Planner */
    context->solve(res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }
    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    /* Now you should see two planned trajectories in series*/
    display_publisher.publish(display_trajectory);

    /* We will add more goals. But first, set the state in the planning
       scene to the final state of the last plan */
    robot_state.setJointGroupPositions(_joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    /* Now, we go back to the first goal*/
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    // Adding Path Constraints
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
    /* Let's create a new pose goal */
    pose.pose.position.x = 0.65;
    pose.pose.position.y = -0.2;
    pose.pose.position.z = -0.1;
    moveit_msgs::Constraints pose_goal_2 =
        kinematic_constraints::constructGoalConstraints("rh_wrist", pose, tolerance_pose, tolerance_angle);
    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state.setJointGroupPositions(_joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    /* Now, let's try to move to this new pose goal*/
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal_2);

    /* But, let's impose a path constraint on the motion.
       Here, we are asking for the end-effector to stay level*/
    geometry_msgs::QuaternionStamped quaternion;
    quaternion.header.frame_id = "torso_lift_link";
    quaternion.quaternion.w = 1.0;
    req.path_constraints = kinematic_constraints::constructGoalConstraints("rh_wrist", quaternion);

    // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
    // (the workspace of the robot)
    // because of this, we need to specify a bound for the allowed planning volume as well;
    // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
    // but that is not being used in this example).
    // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
    // in this volume
    // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
        req.workspace_parameters.min_corner.z = -2.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
        req.workspace_parameters.max_corner.z = 2.0;

    // Call the planner and visualize all the plans created so far.
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);
    // Now you should see four planned trajectories in series
    display_publisher.publish(display_trajectory);

    // END_TUTORIAL
    sleep_time.sleep();
    ROS_INFO("Done");
    planner_instance.reset();

    return 0;
}

void RobotMoveIt::doMoveGroupInterface()
{
#if 0
    ros::AsyncSpinner spinner(1);
    spinner.start();
#endif
    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "right_arm";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *_joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in Rviz
    visual_tools.loadRemoteControl();

    // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75; // above head of PR2
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.7;
    target_pose1.position.z = 1.0;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = move_group.plan(my_plan);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in Rviz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, _joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
    /* move_group.move() */

    // Planning to a joint-space goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Let's set a joint space goal and move towards it.  This will replace the
    // pose target we set above.
    //
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(_joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = -1.0;  // radians
    move_group.setJointValueTarget(joint_group_positions);

    success = move_group.plan(my_plan);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, _joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // Planning with Path Constraints
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Path constraints can easily be specified for a link on the robot.
    // Let's specify a path constraint and a pose goal for our group.
    // First define the path constraint.
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "r_wrist_roll_link";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

    // We will reuse the old goal that we had and plan to it.
    // Note that this will only work if the current state already
    // satisfies the path constraints. So, we need to set the start
    // state to a new pose.
    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;
    start_state.setFromIK(_joint_model_group, start_pose2);
    move_group.setStartState(start_state);

    // Now we will plan to the earlier pose target from the new
    // start state that we have just created.
    move_group.setPoseTarget(target_pose1);

    // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
    // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
    move_group.setPlanningTime(10.0);

    success = move_group.plan(my_plan);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    // Visualize the plan in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start");
    visual_tools.publishAxisLabeled(target_pose1, "goal");
    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, _joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // When done with the path constraint be sure to clear it.
    move_group.clearPathConstraints();

    // Cartesian Paths
    // ^^^^^^^^^^^^^^^
    // You can plan a cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note that we are starting
    // from the new start state above.  The initial pose (start state) does not
    // need to be added to the waypoint list but adding it can help with visualizations
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);

    geometry_msgs::Pose target_pose3 = start_pose2;

    target_pose3.position.z += 0.2;
    waypoints.push_back(target_pose3);  // up

    target_pose3.position.y -= 0.1;
    waypoints.push_back(target_pose3);  // left

    target_pose3.position.z -= 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3);  // down and right

    // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
    // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
    // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
    move_group.setMaxVelocityScalingFactor(0.1);

    // We want the cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
      visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // Adding/Removing Objects and Attaching/Detaching Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "box1";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

    //Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.6;
    box_pose.position.y = -0.4;
    box_pose.position.z = 1.2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in Rviz of status
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Sleep to allow MoveGroup to recieve and process the collision object message
    ros::Duration(1.0).sleep();

    // Now when we plan a trajectory it will avoid the obstacle
    move_group.setStartState(*move_group.getCurrentState());
    move_group.setPoseTarget(target_pose1);

    success = move_group.plan(my_plan);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

    // Visualize the plan in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, _joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // Now, let's attach the collision object to the robot.
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    move_group.attachObject(collision_object.id);

    // Show text in Rviz of status
    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Sleep to allow MoveGroup to recieve and process the attached collision object message */
    ros::Duration(1.0).sleep();

    // Now, let's detach the collision object from the robot.
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    move_group.detachObject(collision_object.id);

    // Show text in Rviz of status
    visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Sleep to allow MoveGroup to recieve and process the detach collision object message */
    ros::Duration(1.0).sleep();

    // Now, let's remove the collision object from the world.
    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    // Show text in Rviz of status
    visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Sleep to give Rviz time to show the object is no longer there.*/
    ros::Duration(1.0).sleep();

    // Dual-arm pose goals
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // First define a new group for addressing the two arms.
    static const std::string PLANNING_GROUP2 = "arms";
    moveit::planning_interface::MoveGroupInterface two_arms_move_group(PLANNING_GROUP2);

    // Define two separate pose goals, one for each end-effector. Note that
    // we are reusing the goal for the right arm above
    two_arms_move_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

    geometry_msgs::Pose target_pose4;
    target_pose4.orientation.w = 1.0;
    target_pose4.position.x = 0.7;
    target_pose4.position.y = 0.15;
    target_pose4.position.z = 1.0;

    two_arms_move_group.setPoseTarget(target_pose4, "l_wrist_roll_link");

    // Now, we can plan and visualize
    moveit::planning_interface::MoveGroupInterface::Plan two_arms_plan;

    success = two_arms_move_group.plan(two_arms_plan);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (dual arm plan) %s", success ? "" : "FAILED");

    // Visualize the plan in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose1, "goal1");
    visual_tools.publishAxisLabeled(target_pose4, "goal2");
    visual_tools.publishText(text_pose, "Two Arm Goal", rvt::WHITE, rvt::XLARGE);
    _joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);
    visual_tools.publishTrajectoryLine(two_arms_plan.trajectory_, _joint_model_group);
    visual_tools.trigger();

    // END_TUTORIAL
}

void RobotMoveIt::doMoveGroup()
{
#if 0
    ros::AsyncSpinner spinner(1);
    spinner.start();
#endif

    /* First put an object into the scene*/
    /* Advertise the collision object message publisher*/
    ros::Publisher collision_object_publisher =
        node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    while (collision_object_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
    /* Define the object message */
    moveit_msgs::CollisionObject object;
    /* The header must contain a valid TF frame */
    object.header.frame_id = "r_wrist_roll_link";
    /* The id of the object */
    object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);

    /* An attach operation requires an ADD */
    object.operation = attached_object.object.ADD;

    /* Publish and sleep (to view the visualized results) */
    collision_object_publisher.publish(object);
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

    /* CHECK IF A STATE IS VALID */
    /* PUT THE OBJECT IN THE ENVIRONMENT */
    ROS_INFO("Putting the object back into the environment");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    sleep_time.sleep();

    /* Load the robot model */
    robot_model_loader::RDFLoader robot_model_loader("robot_description");
    /* Get a shared pointer to the model and construct a state */
    robot_model::RobotModelPtr _kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState current_state(_kinematic_model);
    current_state.getJointStateGroup("right_arm")->setToRandomValues();

    /* Construct a robot state message */
    moveit_msgs::RobotState robot_state;
    robot_state::robotStateToRobotStateMsg(current_state, robot_state);

    /* Construct the service request */
    moveit_msgs::GetStateValidity::Request get_state_validity_request;
    moveit_msgs::GetStateValidity::Response get_state_validity_response;
    get_state_validity_request.robot_state = robot_state;
    get_state_validity_request.group_name = "right_arm";

    /* Service client for checking state validity */
    ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");

    /* Publisher for display */
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1);
    moveit_msgs::DisplayRobotState display_state;

    for (std::size_t i = 0; i < 20; ++i)
    {
        /* Make the service call */
        service_client.call(get_state_validity_request, get_state_validity_response);
        if (get_state_validity_response.valid)
          ROS_INFO("State %d was valid", (int)i);
        else
          ROS_ERROR("State %d was invalid", (int)i);

        /* Visualize the state */
        display_state.state = robot_state;
        display_publisher.publish(display_state);

        /* Generate a new state and put it into the request */
        current_state.getJointStateGroup("right_arm")->setToRandomValues();
        robot_state::robotStateToRobotStateMsg(current_state, robot_state);
        get_state_validity_request.robot_state = robot_state;
        sleep_time.sleep();
    }
}

void RobotMoveIt::doPlanningPipeline()
{
    // BEGIN_TUTORIAL
    // Start
    // ^^^^^
    // Setting up to start using a planning pipeline is pretty easy.
    // Before we can load the planner, we need two objects, a RobotModel
    // and a PlanningScene.
    // We will start by instantiating a
    // `RobotModelLoader`_
    // object, which will look up
    // the robot description on the ROS parameter server and construct a
    // :moveit_core:`RobotModel` for us to use.
    //
    // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :planning_scene:`PlanningScene` that maintains the state of
    // the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // We can now setup the PlanningPipeline
    // object, which will use the ROS param server
    // to determine the set of request adapters and the
    // planning plugin to use
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

    /* Sleep a little to allow time to startup rviz, etc. */
    ros::WallDuration sleep_time(20.0);
    sleep_time.sleep();

    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the PR2
    // specifying the desired pose of the end-effector as input.
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "torso_lift_link";
    pose.pose.position.x = 0.75;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // We will create the request as a constraint using a helper function available
    // from the
    // `kinematic_constraints`_
    // package.
    //
    // .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
    req.group_name = "right_arm";
    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(planning_scene, req, res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    ros::Publisher display_publisher =
        node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    sleep_time.sleep();

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(response.trajectory_start);
    const robot_model::JointModelGroup* _joint_model_group = robot_state.getJointModelGroup("right_arm");
    robot_state.setJointGroupPositions(_joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    // Now, setup a joint space goal
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values(7, 0.0);
    joint_values[0] = -2.0;
    joint_values[3] = -0.2;
    joint_values[5] = -0.15;
    goal_state.setJointGroupPositions(_joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, _joint_model_group);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Call the pipeline and visualize the trajectory
    planning_pipeline->generatePlan(planning_scene, req, res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }
    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    // Now you should see two planned trajectories in series
    display_publisher.publish(display_trajectory);
    sleep_time.sleep();

    // Using a Planning Request Adapter
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // A planning request adapter allows us to specify a series of operations that
    // should happen either before planning takes place or after the planning
    // has been done on the resultant path

    // First, let's purposefully set the initial state to be outside the
    // joint limits and let the
    // planning request adapter deal with it
    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(response.trajectory_start);
    robot_state.setJointGroupPositions(_joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    // Now, set one of the joints slightly outside its upper limit
    const robot_model::JointModel* joint_model = _joint_model_group->getJointModel("r_shoulder_pan_joint");
    const robot_model::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
    std::vector<double> tmp_values(1, 0.0);
    tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
    robot_state.setJointPositions(joint_model, tmp_values);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);

    // Call the planner again and visualize the trajectories
    planning_pipeline->generatePlan(planning_scene, req, res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }
    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    /* Now you should see three planned trajectories in series*/
    display_publisher.publish(display_trajectory);

    sleep_time.sleep();
    ROS_INFO("Done");
    return 0;
}

void RobotMoveIt::doPlanningSceneRosAPI()
{
    //ros::Duration sleep_time(10.0);
    //sleep_time.sleep();
    //sleep_time.sleep();

    // BEGIN_TUTORIAL
    //
    // ROS API
    // ^^^^^^^
    // The ROS API to the planning scene publisher is through a topic interface
    // using "diffs". A planning scene diff is the difference between the current
    // planning scene (maintained by the move_group node) and the new planning
    // scene desired by the user.
    //
    // Advertise the required topic
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Note that this topic may need to be remapped in the launch file
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }

    // Define the attached object message
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // We will use this message to add or
    // subtract the object from the world
    // and to attach the object to the robot
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "r_wrist_roll_link";
    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = "r_wrist_roll_link";
    /* The id of the object */
    attached_object.object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    // Note that attaching an object to the robot requires
    // the corresponding operation to be specified as an ADD operation
    attached_object.object.operation = attached_object.object.ADD;

    // Add an object into the environment
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Add the object into the environment by adding it to
    // the set of collision objects in the "world" part of the
    // planning scene. Note that we are using only the "object"
    // field of the attached_object message here.
    ROS_INFO("Adding the object into the world at the location of the right wrist.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    sleep_time.sleep();

    // Interlude: Synchronous vs Asynchronous updates
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // There are two separate mechanisms available to interact
    // with the move_group node using diffs:
    //
    // * Send a diff via a rosservice call and block until
    //   the diff is applied (synchronous update)
    // * Send a diff via a topic, continue even though the diff
    //   might not be applied yet (asynchronous update)
    //
    // While most of this tutorial uses the latter mechanism (given the long sleeps
    // inserted for visualization purposes asynchronous updates do not pose a problem),
    // it would is perfectly justified to replace the planning_scene_diff_publisher
    // by the following service client:
    ros::ServiceClient planning_scene_diff_client =
        node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
    // and send the diffs to the planning scene via a service call:
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
    // Note that this does not continue until we are sure the diff has been applied.
    //
    // Attach an object to the robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // When the robot picks up an object from the environment, we need to
    // "attach" the object to the robot so that any component dealing with
    // the robot model knows to account for the attached object, e.g. for
    // collision checking.
    //
    // Attaching an object requires two operations
    //  * Removing the original object from the environment
    //  * Attaching the object to the robot

    /* First, define the REMOVE object message*/
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = "box";
    remove_object.header.frame_id = "odom_combined";
    remove_object.operation = remove_object.REMOVE;

    // Note how we make sure that the diff message contains no other
    // attached objects or collisions objects by clearing those fields
    // first.
    /* Carry out the REMOVE + ATTACH operation */
    ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_diff_publisher.publish(planning_scene);

    sleep_time.sleep();

    // Detach an object from the robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Detaching an object from the robot requires two operations
    //  * Detaching the object from the robot
    //  * Re-introducing the object into the environment

    /* First, define the DETACH object message*/
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = "box";
    detach_object.link_name = "r_wrist_roll_link";
    detach_object.object.operation = attached_object.object.REMOVE;

    // Note how we make sure that the diff message contains no other
    // attached objects or collisions objects by clearing those fields
    // first.
    /* Carry out the DETACH + ADD operation */
    ROS_INFO("Detaching the object from the robot and returning it to the world.");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene_diff_publisher.publish(planning_scene);

    sleep_time.sleep();

    // Remove the object from the collision world
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Removing the object from the collision world just requires
    // using the remove object message defined earlier.
    // Note, also how we make sure that the diff message contains no other
    // attached objects or collisions objects by clearing those fields
    // first.
    ROS_INFO("Removing the object from the world.");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene_diff_publisher.publish(planning_scene);
    // END_TUTORIAL

    return 0;
}

void RobotMoveIt::doPlanningScene()
{
    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // The :planning_scene:`PlanningScene` class can be easily setup and
    // configured using a :moveit_core:`RobotModel` or a URDF and
    // SRDF. This is, however, not the recommended way to instantiate a
    // PlanningScene. The :planning_scene_monitor:`PlanningSceneMonitor`
    // is the recommended method to create and maintain the current
    // planning scene (and is discussed in detail in the next tutorial)
    // using data from the robot's joints and the sensors on the robot. In
    // this tutorial, we will instantiate a PlanningScene class directly,
    // but this method of instantiation is only intended for illustration.

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr _kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(_kinematic_model);

    // Collision Checking
    // ^^^^^^^^^^^^^^^^^^
    //
    // Self-collision checking
    // ~~~~~~~~~~~~~~~~~~~~~~~
    //
    // The first thing we will do is check whether the robot in its
    // current state is in *self-collision*, i.e. whether the current
    // configuration of the robot would result in the robot's parts
    // hitting each other. To do this, we will construct a
    // :collision_detection_struct:`CollisionRequest` object and a
    // :collision_detection_struct:`CollisionResult` object and pass them
    // into the collision checking function. Note that the result of
    // whether the robot is in self-collision or not is contained within
    // the result. Self collision checking uses an *unpadded* version of
    // the robot, i.e. it directly uses the collision meshes provided in
    // the URDF with no extra padding added on.

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Change the state
    // ~~~~~~~~~~~~~~~~
    //
    // Now, let's change the current state of the robot. The planning
    // scene maintains the current state internally. We can get a
    // reference to it and change it and then check for collisions for the
    // new robot configuration. Note in particular that we need to clear
    // the collision_result before making a new collision checking
    // request.

    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Checking for a group
    // ~~~~~~~~~~~~~~~~~~~~
    //
    // Now, we will do collision checking only for the right_arm of the
    // PR2, i.e. we will check whether there are any collisions between
    // the right arm and other parts of the body of the robot. We can ask
    // for this specifically by adding the group name "right_arm" to the
    // collision request.

    collision_request.group_name = "right_arm";
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Getting Contact Information
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // First, manually set the right arm to a position where we know
    // internal (self) collisions do happen. Note that this state is now
    // actually outside the joint limits of the PR2, which we can also
    // check for directly.

    std::vector<double> joint_values;
    const robot_model::JointModelGroup* _joint_model_group = current_state.getJointModelGroup("right_arm");
    current_state.copyJointGroupPositions(_joint_model_group, joint_values);
    joint_values[0] = 1.57;  // hard-coded since we know collisions will happen here
    current_state.setJointGroupPositions(_joint_model_group, joint_values);
    ROS_INFO_STREAM("Current state is " << (current_state.satisfiesBounds(_joint_model_group) ? "valid" : "not valid"));

    // Now, we can get contact information for any collisions that might
    // have happened at a given configuration of the right arm. We can ask
    // for contact information by filling in the appropriate field in the
    // collision request and specifying the maximum number of contacts to
    // be returned as a large number.

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    //

    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 4: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
    {
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }

    // Modifying the Allowed Collision Matrix
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // The :collision_detection_class:`AllowedCollisionMatrix` (ACM)
    // provides a mechanism to tell the collision world to ignore
    // collisions between certain object: both parts of the robot and
    // objects in the world. We can tell the collision checker to ignore
    // all collisions between the links reported above, i.e. even though
    // the links are actually in collision, the collision checker will
    // ignore those collisions and return not in collision for this
    // particular state of the robot.
    //
    // Note also in this example how we are making copies of both the
    // allowed collision matrix and the current state and passing them in
    // to the collision checking function.

    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
    {
      acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Full Collision Checking
    // ~~~~~~~~~~~~~~~~~~~~~~~
    //
    // While we have been checking for self-collisions, we can use the
    // checkCollision functions instead which will check for both
    // self-collisions and for collisions with the environment (which is
    // currently empty).  This is the set of collision checking
    // functions that you will use most often in a planner. Note that
    // collision checks with the environment will use the padded version
    // of the robot. Padding helps in keeping the robot further away
    // from obstacles in the environment.*/
    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Constraint Checking
    // ^^^^^^^^^^^^^^^^^^^
    //
    // The PlanningScene class also includes easy to use function calls
    // for checking constraints. The constraints can be of two types:
    // (a) constraints chosen from the
    // :kinematic_constraints:`KinematicConstraint` set:
    // i.e. :kinematic_constraints:`JointConstraint`,
    // :kinematic_constraints:`PositionConstraint`,
    // :kinematic_constraints:`OrientationConstraint` and
    // :kinematic_constraints:`VisibilityConstraint` and (b) user
    // defined constraints specified through a callback. We will first
    // look at an example with a simple KinematicConstraint.
    //
    // Checking Kinematic Constraints
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // We will first define a simple position and orientation constraint
    // on the end-effector of the right_arm of the PR2 robot. Note the
    // use of convenience functions for filling up the constraints
    // (these functions are found in the :moveit_core_files:`utils.h<utils_8h>` file from the
    // kinematic_constraints directory in moveit_core).

    std::string end_effector_name = _joint_model_group->getLinkModelNames().back();

    geometry_msgs::PoseStamped desired_pose;
    desired_pose.pose.orientation.w = 1.0;
    desired_pose.pose.position.x = 0.75;
    desired_pose.pose.position.y = -0.185;
    desired_pose.pose.position.z = 1.3;
    desired_pose.header.frame_id = "base_footprint";
    moveit_msgs::Constraints goal_constraint =
        kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

    // Now, we can check a state against this constraint using the
    // isStateConstrained functions in the PlanningScene class.

    copied_state.setToRandomPositions();
    copied_state.update();
    bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
    ROS_INFO_STREAM("Test 7: Random state is " << (constrained ? "constrained" : "not constrained"));

    // There's a more efficient way of checking constraints (when you want
    // to check the same constraint over and over again, e.g. inside a
    // planner). We first construct a KinematicConstraintSet which
    // pre-processes the ROS Constraints messages and sets it up for quick
    // processing.

    kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(_kinematic_model);
    kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
    bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
    ROS_INFO_STREAM("Test 8: Random state is " << (constrained_2 ? "constrained" : "not constrained"));

    // There's a direct way to do this using the KinematicConstraintSet
    // class.

    kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
        kinematic_constraint_set.decide(copied_state);
    ROS_INFO_STREAM("Test 9: Random state is " << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

    // User-defined constraints
    // ~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // CALL_SUB_TUTORIAL userCallback

    // Now, whenever isStateFeasible is called, this user-defined callback
    // will be called.

    planning_scene.setStateFeasibilityPredicate(userCallback);
    bool state_feasible = planning_scene.isStateFeasible(copied_state);
    ROS_INFO_STREAM("Test 10: Random state is " << (state_feasible ? "feasible" : "not feasible"));

    // Whenever isStateValid is called, three checks are conducted: (a)
    // collision checking (b) constraint checking and (c) feasibility
    // checking using the user-defined callback.

    bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "right_arm");
    ROS_INFO_STREAM("Test 10: Random state is " << (state_valid ? "valid" : "not valid"));

    // Note that all the planners available through MoveIt! and OMPL will
    // currently perform collision checking, constraint checking and
    // feasibility checking using user-defined callbacks.
    // END_TUTORIAL

    return 0;
}
#endif
