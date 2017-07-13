#!/usr/bin/env python

# This example demonstrates how to move the right hand and arm through a sequence of joint goals.
# At the start and end of the sequence, both the hand and arm will spend 20s in teach mode,
# This allows the user to manually move the hand and arm. New goals can be easily generated
# using the script 'sr_print_joints_position.py
# PLEASE NOTE: move_to_joint_value_target_unsafe is used in this script, so collision checks
# between the hand and arm are not made!

# Native Python
import sys
import copy
from copy import deepcopy
from math import pi
import time
from threading import Timer

from std_msgs.msg import String
from std_srvs.srv import Empty

# ROS Python Modules
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
import rospy
from rospy import Timer

import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import WrenchStamped
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from tf.transformations import quaternion_from_euler
from tf_conversions import posemath, toMsg

import PyKDL

# GAZEBO modules
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, SpawnModel, DeleteModel
from actionlib import SimpleActionClient

# RVIZ modules
from visualization_msgs.msg import Marker

# MOVEIT modules
import moveit_commander
from moveit_commander import MoveGroupCommander

from moveit_msgs.srv import GetPlanningScene
from moveit_commander import MoveGroupCommander

# https://github.com/ros-planning/moveit_msgs/tree/kinetic-devel/msg
import moveit_msgs.msg
from moveit_msgs.msg import DisplayRobotState, PlanningScene, PlanningSceneComponents

# SHADOWHAND package modules
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


# ########################################################################################################
# GLOBAL HELPER MEMBERS

# ########################################################################################################
# ROSPY INIT
# http://wiki.ros.org/rospy/Overview/Services
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
WORLD_FRAME = "world"

# sr_multi_moveit/sr_multi_moveit_config/config/generated_robot.srdf
MAIN_ARM_GROUP_NAME           = "right_arm"  # Just the arm itself, not including the hand!
MAIN_ARM_HAND_GROUP_NAME      = "right_arm_and_hand"
MAIN_ARM_WRIST_GROUP_NAME     = "right_arm_and_wrist"

MAIN_HAND_GROUP_NAME          = "right_hand"
MAIN_HAND_GROUP_WRIST         = "rh_wrist"
MAIN_HAND_GROUP_FIRST_FINGER  = "rh_first_finger"
MAIN_HAND_GROUP_MIDDLE_FINGER = "rh_middle_finger"
MAIN_HAND_GROUP_RING_FINGER   = "rh_ring_finger"
MAIN_HAND_GROUP_LITTLE_FINGER = "rh_little_finger"
MAIN_HAND_GROUP_THUMB         = "rh_thumb"
MAIN_HAND_GROUP_FINGERS       = "rh_fingers"

GAZEBO_MODEL_PLATE_PATH   = "/home/brhm/DUC/RobotArm/src/my_arm/models/plate/model.sdf"
GAZEBO_MODEL_PLATE_NAME   = "plate"
GAZEBO_MODEL_HAMMER_PATH  = "/usr/share/gazebo-7/models/hammer/model.sdf"
GAZEBO_MODEL_HAMMER_NAME  = "hammer"
GAZEBO_MODEL_CRICKET_NAME = "cricket_ball"

rospy.init_node("my_arm_controller", anonymous=True)
# r = rospy.Rate(10) # 10hz
# while not rospy.is_shutdown():
#     pub.publish("hello")
#     r.sleep()

# ########################################################################################################
# GAZEBO SERVICES INIT
#
# http://gazebosim.org/tutorials?tut=ros_comm
# http://mirror.umd.edu/roswiki/doc/diamondback/api/gazebo/html/msg/ModelState.html
# Services: State and property getters
# ~/get_model_properties   : gazebo_msgs/GetModelProperties- This service returns the properties of a model in simulation.
#
# ~/get_model_state        : gazebo_msgs/GetModelState - This service returns the states of a model in simulation.
#
# ~/get_world_properties   : gazebo_msgs/GetWorldProperties - This service returns the properties of the simulation world.
#
# ~/get_joint_properties   : gazebo_msgs/GetJointProperties - This service returns the properties of a joint in simulation.
#
# ~/get_link_properties    : gazebo_msgs/GetLinkProperties - This service returns the properties of a link in simulation.
#
# ~/get_link_state         : gazebo_msgs/GetLinkState - This service returns the states of a link in simulation.
#
# ~/get_physics_properties : gazebo_msgs/GetPhysicsProperties - This service returns the properties of the physics engine used in simulation.
#
rospy.wait_for_service("/gazebo/get_model_state", 10.0)
rospy.wait_for_service("/gazebo/reset_world", 10.0)
# ducta ++
rospy.wait_for_service("/gazebo/spawn_sdf_model", 10.0)
# ducta --
gb_gz_srv_reset_world     = rospy.ServiceProxy("/gazebo/reset_world", Empty)
gb_gz_srv_get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
gb_gz_srv_spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
gb_gz_srv_delete_model    = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

rospy.wait_for_service("/gazebo/pause_physics")
gb_gz_srv_pause_physics   = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
rospy.wait_for_service("/gazebo/unpause_physics")
gb_gz_srv_unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
rospy.wait_for_service("/controller_manager/switch_controller")
gb_gz_srv_switch_ctrl     = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
rospy.wait_for_service("/gazebo/set_model_configuration")
gb_gz_srv_set_model       = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

rospy.wait_for_service('/get_planning_scene', 10.0)
gb_gz_srv_get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
gb_gz_pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)

# Spawn a model into Gazebo
def gb_gz_spawn_model(model_name, model_path, model_pose):
    model_file  = open(model_path,'r')
    spawn_model = model_file.read()
    gb_gz_srv_delete_model(model_name)
    gb_gz_srv_spawn_sdf_model(model_name, spawn_model, "", model_pose, WORLD_FRAME) # <-- SPAWNING MODEL HERE !!!

# Spawn PLATE model into Gazebo
def gb_gz_spawn_plate_model():
    mpose = geometry_msgs.msg.Pose()
    mpose.orientation.w = 1.0
    mpose.position.x = 1.17
    mpose.position.y = -0.29
    mpose.position.z = 1.29

    gb_gz_spawn_model(GAZEBO_MODEL_PLATE_NAME, GAZEBO_MODEL_PLATE_PATH, mpose)

# Spawn a model in loop with lapse_time
def gb_gz_spawn_model_loop(model_name, model_path, model_pose, lapse_time):
    model_file  = open(model_path,'r')
    spawn_model = model_file.read()

    while not rospy.is_shutdown():
        gb_gz_srv_delete_model(model_name)
        gb_gz_srv_spawn_sdf_model(model_name, spawn_model, "", model_pose, WORLD_FRAME) # <-- SPAWNING MODEL HERE !!!
        #time.sleep(lapse_time)
        rospy.sleep(lapse_time)

# Spawn PLATE model in loop with lapse_time
def gb_gz_spawn_plate_model_loop(lapse_time):
    mpose = geometry_msgs.msg.Pose()
    mpose.orientation.w = 1.0
    mpose.position.x = 1.17
    mpose.position.y = -0.29
    mpose.position.z = 1.29

    gb_gz_spawn_model_loop(GAZEBO_MODEL_PLATE_NAME, GAZEBO_MODEL_PLATE_PATH, mpose, lapse_time)

    #print "START SPAWNING MODEL IN SEPARATE THREAD..."
    #try:
    #    thread.start_new_thread(gb_gz_spawn_model_loop, (GAZEBO_MODEL_PLATE_NAME, GAZEBO_MODEL_PLATE_PATH, mpose, lapse_time))
    #except:
    #    print "Error: unable to start thread"

def gb_gz_get_model_pose(model_name, base_frame_name):
    """
    Gets the pose of the model in the specified base frame.

    @return The pose of the ball.
    """
    return gb_gz_srv_get_model_state.call(model_name, base_frame_name).pose

def gb_gz_get_ball_pose():
    """
    Gets the pose of the ball in the world frame.

    @return The pose of the ball.
    """
    return gb_gz_get_model_pose(GAZEBO_MODEL_CRICKET_NAME, WORLD_FRAME)

# ########################################################################################################
# ROSPY GLOBAL TIMER CALLBACK
#
def gb_ros_timer_callback():
    rospy.loginfo("ABC DEF GHI\n")
    gb_gz_spawn_plate_model()

gb_ros_timer = rospy.Timer(rospy.Duration(1), gb_ros_timer_callback)
#gb_ros_timer.start() , already starts

# RVIZ PUBLISHER INIT
#
# # We create this DisplayTrajectory publisher which is used below to publish
# # trajectories for RVIZ to visualize.
gb_rviz_display_trajectory_publisher = rospy.Publisher(
                                             '/move_group/display_planned_path',
                                             moveit_msgs.msg.DisplayTrajectory)

# # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
print "============ Waiting for RVIZ..."
#rospy.sleep(10)
print "============ Starting tutorial "

# ########################################################################################################
# INIT MOVE IT COMMANDER
#
# First initialize moveit_commander and rospy.
moveit_commander.roscpp_initialize(sys.argv)

# Instantiate a RobotCommander object.  This object is an interface to
# the robot as a whole.
# http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
gb_robot = moveit_commander.RobotCommander()
# gb_robot.get_group_names()
# gb_robot.get
# gb_robot.get_current_state()
# gb_robot.get_current_variable_values

# Instantiate a PlanningSceneInterface object.  This object is an interface
# to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object.  This object is an interface
# to one group of joints.  In this case the group is the joints in the left
# arm.  This interface can be used to plan and execute motions on the left arm.
gb_arm_joint_group  = moveit_commander.MoveGroupCommander(MAIN_ARM_GROUP_NAME)
gb_hand_joint_group = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_NAME)

gb_TH_joint_group   = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_THUMB)
gb_FF_joint_group   = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_FIRST_FINGER)
gb_MF_joint_group   = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_MIDDLE_FINGER)
#gb_RF_joint_group   = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_RING_FINGER)
gb_LF_joint_group   = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_LITTLE_FINGER)

# ########################################################################################################
# LEAP MOTION
#
import leap_interface
import Leap

FREQUENCY_ROSTOPIC_DEFAULT = 0.01
NODENAME = 'leap_skeleton_pub'
PARAMNAME_FREQ = 'freq'
PARAMNAME_FREQ_ENTIRE = '/' + NODENAME + '/' + PARAMNAME_FREQ

def catchHandLeapMotionData():
    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    # pub_ros   = rospy.Publisher('leapmotion/data',leapros)

    while not rospy.is_shutdown():
        timenow=rospy.Time.now()

        if li.listener.left_hand:
            analyzeHandLeapMotionData(li.listener.left_hand)
        elif li.listener.right_hand:
            analyzeHandLeapMotionData(li.listener.right_hand)

        fingerNames = ['thumb', 'index', 'middle', 'ring', 'pinky']
        fingerPointNames = ['metacarpal', 'proximal',
                            'intermediate', 'distal', 'tip']

        for fingerName in fingerNames:
            for fingerPointName in fingerPointNames:
                pos = li.get_finger_point(fingerName, fingerPointName)

                # print(fingerName)
                # print(fingerPointName)
                # for iDim, dimName in enumerate(['x', 'y', 'z']):
                #     print(dimName)
                #     print(pos[iDim])
                #     print('\n')

        # save some CPU time, circa 100Hz publishing.
        rospy.sleep(0.01)
    print('ROSPY SHUT DOWN')

def analyzeHandLeapMotionData(hand):
    for finger in hand.fingers:
        finger_name=hand_name+"_"+finger_names[finger.type()]

        prev_bone_name=hand_name
        for num in range(0,4):
            bone=finger.bone(num)
            bone_name=finger_name+"_"+bones_names[num]

            prev_bone_name=bone_name
            prev_bone_absolute=bone_absolute

# ########################################################################################################
# SHADOW HAND CONTROL
#
#
# HandFinder is used to access the hand parameters
gb_hand_finder = HandFinder()
gb_hand_parameters = gb_hand_finder.get_hand_parameters()
prefix = gb_hand_parameters.mapping.values()
rospy.loginfo('HAND TYPE:' + prefix[0])
gb_hand_serial = gb_hand_parameters.mapping.keys()[0]
rospy.loginfo('HAND SERIAL:' + gb_hand_serial)

# Arm & Hand Commander
gb_arm_commander  = SrArmCommander(name=MAIN_ARM_GROUP_NAME, set_ground=False)
gb_hand_commander = SrHandCommander(name=MAIN_HAND_GROUP_NAME, prefix="", \
                                    hand_parameters=gb_hand_parameters,   \
                                    hand_serial=gb_hand_serial)
#gb_hand_commander = SrHandCommander(name="right_hand", prefix="rh")

hand_tactile_type  = gb_hand_commander.get_tactile_type()
rospy.loginfo('HAND TACTILE TYPE:' + gb_hand_serial)
hand_tactile_state = gb_hand_commander.get_tactile_state() # as a msg from a callback
#rospy.loginfo('HAND TACTILE STATE:' + hand_tactile_state)

# Hand Mapping(to do the grasp pose)
gb_hand_mapping = gb_hand_parameters.mapping[gb_hand_serial]

# Hand Joints
gb_joints = gb_hand_finder.get_hand_joints()[gb_hand_mapping]

deep_joint_poses = [0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.5707]
deep_hand_joint_names = {0: 'rh_FFJ1', 1 : 'rh_FFJ2', 2 : 'rh_FFJ3',  3: 'rh_FFJ4',
                         4: 'rh_MFJ1', 5 : 'rh_MFJ2', 6 : 'rh_MFJ3',  7: 'rh_MFJ4',
                         8: 'rh_RFJ1', 9 : 'rh_RFJ2', 10: 'rh_RFJ3', 11: 'rh_RFJ4',
                         12:'rh_LFJ1', 13: 'rh_LFJ2', 14: 'rh_LFJ3', 15: 'rh_LFJ4', 16: 'rh_LFJ5',
                         17:'rh_THJ1', 18: 'rh_THJ2', 19: 'rh_THJ3', 20: 'rh_THJ4', 21: 'rh_THJ5' }

gb_arm_joint_goals =  {'ra_shoulder_pan_joint' : 0.0,
                       'ra_shoulder_lift_joint': 0.0,
                       'ra_elbow_joint': 0.0,
                       'ra_wrist_1_joint': 0.0,
                       'ra_wrist_2_joint': 0.0,
                       'ra_wrist_3_joint': 0.0,
                       'rh_WRJ2': 0.0,
                       'rh_WRJ1': 0.0}


def gbMoveArmInitialPos():
    gb_arm_joint_goals['ra_shoulder_pan_joint']  = 1.0
    gb_arm_joint_goals['ra_shoulder_lift_joint'] = 0.1
    gb_arm_joint_goals['ra_elbow_joint']         = 0.0

    gb_arm_joint_goals['ra_wrist_1_joint']       = 0.0
    gb_arm_joint_goals['ra_wrist_2_joint']       = 0.0
    gb_arm_joint_goals['ra_wrist_3_joint']       = pi

    gb_arm_joint_goals['rh_WRJ2']                = 0.0
    gb_arm_joint_goals['rh_WRJ1']                = 0.5

    gb_arm_commander.move_to_joint_value_target_unsafe(gb_arm_joint_goals, 3.0, True)
    return

#gbMoveArmInitialPos()

def gbMoveArm(joint_goals):
    gb_arm_joint_goals['ra_wrist_2_joint']      = joint_goals[0]
    gb_arm_joint_goals['ra_wrist_1_joint']      = joint_goals[1]
    gb_arm_joint_goals['ra_shoulder_pan_joint'] = joint_goals[2]
    gb_arm_joint_goals['ra_wrist_3_joint']      = joint_goals[3]
    gb_arm_joint_goals['rh_WRJ2']               = joint_goals[4]
    gb_arm_joint_goals['rh_WRJ1']               = joint_goals[5]
    gb_arm_commander.move_to_joint_value_target_unsafe(gb_arm_joint_goals, 3.0, True)
    return

# def onJointState(state):
#     print('ON JOINT STATE ##################################################\n')
#     i = 0
#     for jointName in state.joint_state.name:
#         rospy.loginfo(jointName + ':' + state.joint_state.position[i] + '\n')
#         for name in gb_arm_joint_goals.keys():
#             if(jointName == name):
#                 gb_arm_joint_goals[name] = state.joint_state.position[i]
#                 break
#     else:
#         gb_arm_commander.move_to_joint_value_target_unsafe(gb_arm_joint_goals, 3.0, True)
#     #print(statemulti_dof_joint_state)
#
# rospy.Subscriber("/my_arm_state",
#                  DisplayRobotState, onJointState)



# ########################################################################################################
#
# Reading the optoforce tactiles from the hand
#
def gb_hand_optoforce_callback(data):
    rospy.loginfo("At:" + str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs) +
                  " sensor:" + str(data.header.frame_id) +
                  " Force:" + str(data.wrench.force.x) + "," +
                  str(data.wrench.force.y) + "," + str(data.wrench.force.z))

def gb_hand_optoforce_listener():
    num_sensors = 5
    for sensor_num in range(num_sensors):
        rospy.Subscriber("/optoforce_" + str(sensor_num), WrenchStamped, gb_hand_optoforce_callback)

gb_hand_optoforce_listener()

# ########################################################################################################
# DO ARM TEACHING
#
# Specify goals for hand and arm
hand_joint_goals_1 = {'rh_RFJ2': 1.59, 'rh_RFJ3': 1.49, 'rh_RFJ1': 1.47, 'rh_RFJ4': -0.01, 'rh_LFJ4': 0.02,
                      'rh_LFJ5': 0.061, 'rh_LFJ1': 1.41, 'rh_LFJ2': 1.60, 'rh_LFJ3': 1.49, 'rh_THJ2': 0.64,
                      'rh_THJ3': -0.088, 'rh_THJ1': 0.43, 'rh_THJ4': 0.49, 'rh_THJ5': 0.35, 'rh_FFJ4': -0.02,
                      'rh_FFJ2': 1.71, 'rh_FFJ3': 1.49, 'rh_FFJ1': 1.25, 'rh_MFJ3': 1.49, 'rh_MFJ2': 1.66,
                      'rh_MFJ1': 1.31, 'rh_MFJ4': -0.02}

hand_joint_goals_2 = {'rh_RFJ2': 0.55, 'rh_RFJ3': 0.08, 'rh_RFJ1': 0.03, 'rh_RFJ4': -0.15, 'rh_LFJ4': -0.35,
                      'rh_LFJ5': 0.23, 'rh_LFJ1': 0.02, 'rh_LFJ2': 0.49, 'rh_LFJ3': -0.02, 'rh_THJ2': -0.08,
                      'rh_THJ3': -0.08, 'rh_THJ1': 0.15, 'rh_THJ4': 0.56, 'rh_THJ5': -0.17, 'rh_FFJ4': -0.34,
                      'rh_FFJ2': 0.30, 'rh_FFJ3': 0.16, 'rh_FFJ1': 0.01, 'rh_MFJ3': 0.19, 'rh_MFJ2': 0.50,
                      'rh_MFJ1': 0.00, 'rh_MFJ4': -0.07}

hand_joint_goals_3 = {'rh_RFJ2': 0.63, 'rh_RFJ3': 0.77, 'rh_RFJ1': 0.033, 'rh_RFJ4': -0.02, 'rh_LFJ4': -0.32,
                      'rh_LFJ5': 0.67, 'rh_LFJ1': 0.02, 'rh_LFJ2': 0.73, 'rh_LFJ3': 0.21, 'rh_THJ2': -0.06,
                      'rh_THJ3': -0.04, 'rh_THJ1': 0.39, 'rh_THJ4': 0.85, 'rh_THJ5': 0.40, 'rh_FFJ4': -0.35,
                      'rh_FFJ2': 0.90, 'rh_FFJ3': 0.56, 'rh_FFJ1': 0.02, 'rh_MFJ3': 0.59, 'rh_MFJ2': 0.84,
                      'rh_MFJ1': 0.05, 'rh_MFJ4': -0.08}

hand_joint_goals_4 = {'rh_RFJ2': 0.57, 'rh_RFJ3': 0.27, 'rh_RFJ1': 0.04, 'rh_RFJ4': -0.01, 'rh_LFJ4': -0.28,
                      'rh_LFJ5': 0.39, 'rh_LFJ1': 0.01, 'rh_LFJ2': 0.72, 'rh_LFJ3': -0.12, 'rh_THJ2': -0.19,
                      'rh_THJ3': -0.05, 'rh_THJ1': 0.38, 'rh_THJ4': 0.85, 'rh_THJ5': -0.12, 'rh_FFJ4': -0.32,
                      'rh_FFJ2': 0.64, 'rh_FFJ3': -0.03, 'rh_FFJ1': 0.04, 'rh_MFJ3': 0.04, 'rh_MFJ2': 0.83,
                      'rh_MFJ1': 0.01, 'rh_MFJ4': -0.05}

hand_joint_goals_5 = {'rh_RFJ2': 1.58, 'rh_RFJ3': 1.52, 'rh_RFJ1': 1.34, 'rh_RFJ4': -0.06, 'rh_LFJ4': -0.20,
                      'rh_LFJ5': 0.09, 'rh_LFJ1': 1.47, 'rh_LFJ2': 1.57, 'rh_LFJ3': 1.40, 'rh_THJ2': -0.01,
                      'rh_THJ3': -0.041, 'rh_THJ1': 0.29, 'rh_THJ4': 0.59, 'rh_THJ5': -1.36, 'rh_FFJ4': 0.03,
                      'rh_FFJ2': 1.72, 'rh_FFJ3': 1.41, 'rh_FFJ1': 1.21, 'rh_MFJ3': 1.39, 'rh_MFJ2': 1.65,
                      'rh_MFJ1': 1.33, 'rh_MFJ4': 0.12}

arm_joint_goals_1 = {'ra_shoulder_lift_joint': -1.87, 'ra_elbow_joint': 1.76, 'ra_wrist_2_joint': 0.03,
                     'ra_wrist_1_joint': -0.86, 'ra_shoulder_pan_joint': -2.64, 'ra_wrist_3_joint': 0.69,
                     'rh_WRJ2': -0.02, 'rh_WRJ1': 0.03}

arm_joint_goals_2 = {'ra_shoulder_lift_joint': -1.86, 'ra_elbow_joint': 1.85, 'ra_wrist_2_joint': -0.19,
                     'ra_wrist_1_joint': -0.96, 'ra_shoulder_pan_joint': -1.78, 'ra_wrist_3_joint': 1.06,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': -0.02}

arm_joint_goals_3 = {'ra_shoulder_lift_joint': -1.86, 'ra_elbow_joint': 1.90, 'ra_wrist_2_joint': -0.18,
                     'ra_wrist_1_joint': -0.96, 'ra_shoulder_pan_joint': -1.78, 'ra_wrist_3_joint': 1.06,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.15}

arm_joint_goals_4 = {'ra_shoulder_lift_joint': -1.33, 'ra_elbow_joint': 1.11, 'ra_wrist_2_joint': 1.00,
                     'ra_wrist_1_joint': 0.13, 'ra_shoulder_pan_joint': -1.49, 'ra_wrist_3_joint': 3.27,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.15}

arm_joint_goals_5 = {'ra_shoulder_lift_joint': -1.45, 'ra_elbow_joint': 1.11, 'ra_wrist_2_joint': 0.90,
                     'ra_wrist_1_joint': 0.45, 'ra_shoulder_pan_joint': -0.95, 'ra_wrist_3_joint': 0.09,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.16}

arm_joint_goals_6 = {'ra_shoulder_lift_joint': -1.35, 'ra_elbow_joint': 1.16, 'ra_wrist_2_joint': 0.96,
                     'ra_wrist_1_joint': 0.39, 'ra_shoulder_pan_joint': -0.91, 'ra_wrist_3_joint': 0.09,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.15}

arm_joint_goals_7 = {'ra_shoulder_lift_joint': -1.35, 'ra_elbow_joint': 1.04, 'ra_wrist_2_joint': 1.55,
                     'ra_wrist_1_joint': 0.08, 'ra_shoulder_pan_joint': -1.64, 'ra_wrist_3_joint': -1.41,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.15}

arm_joint_goals_8 = {'ra_shoulder_lift_joint': -1.55, 'ra_elbow_joint': 1.41, 'ra_wrist_2_joint': 0.02,
                     'ra_wrist_1_joint': 0.61, 'ra_shoulder_pan_joint': -1.55, 'ra_wrist_3_joint': -0.57,
                     'rh_WRJ2': -0.04, 'rh_WRJ1': 0.16}


# sleep for some time (default 20s) during which the arm can be moved around by pushing it
# but be careful to get away before the time runs out. You are warned
rospy.loginfo("Set arm teach mode ON")
gb_arm_commander.set_teach_mode(True)
rospy.sleep(1.0)

rospy.loginfo("Set arm teach mode OFF")
gb_arm_commander.set_teach_mode(False)

# Move through each goal
# joint states are sent to the commanders, with a time for execution and a flag as to whether
# or not the commander should wait for the command to complete before moving to the next command.

def do_move_arm_hand():
    # Move hand and arm
    joint_goals = hand_joint_goals_1
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    gb_hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
    joint_goals = arm_joint_goals_1
    rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
    gb_arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

    # Move hand and arm
    joint_goals = hand_joint_goals_2
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    gb_hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
    joint_goals = arm_joint_goals_2
    rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
    gb_arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

    # Move arm
    joint_goals = arm_joint_goals_3
    rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
    gb_arm_commander.move_to_joint_value_target_unsafe(joint_goals, 1.0, True)

    # Move hand
    joint_goals = hand_joint_goals_3
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    gb_hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

    # Move arm
    joint_goals = arm_joint_goals_6
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    gb_arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

    # Move hand
    joint_goals = hand_joint_goals_4
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    gb_hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

    # Move arm
    joint_goals = arm_joint_goals_5
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    gb_arm_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

    # Move arm and hand
    joint_goals = arm_joint_goals_7
    rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
    gb_arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
    joint_goals = hand_joint_goals_5
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    gb_hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

#do_move_arm_hand()

# ########################################################################################################
# DO HAND TEACHING
#
gb_hand_commander.set_teach_mode(True)
# sleep for some time (default 20s) during which the hand joints can be moved manually
rospy.sleep(1.0)
rospy.loginfo("Set hand teach mode OFF")
gb_hand_commander.set_teach_mode(False)
# ...

# ########################################################################################################
# DO SINUISOID FINGERS
#
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from numpy import sin, cos, pi, arange

def do_sinuisoid_fingers():
    # cycles per second of sine wave
    f = 1
    # angular frequency, rads/s
    w = 2 * pi * f
    # time for motion to complete
    ts = 20

    # specify 2 joints to move
    joint_names = [prefix[0] + '_FFJ3', prefix[0] + '_RFJ3']

    # set max and min joint positions
    min_pos_J3 = 0.0
    max_pos_J3 = pi / 2

    rospy.sleep(rospy.Duration(2))

    hand_joints_goal = {joint_names[0]: 0.0, joint_names[1]: 0.0}

    rospy.loginfo("Running joints trajectory")

    # initialising the joint trajectory message
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = rospy.Time.now()
    joint_trajectory.joint_names = list(hand_joints_goal.keys())
    joint_trajectory.points = []

    # generate sinusoidal list of data points, two joints moving out of phase
    for t in arange(0.002, ts, 0.02):
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = rospy.Duration.from_sec(float(t))
        trajectory_point.positions = []
        trajectory_point.velocities = []
        trajectory_point.accelerations = []
        trajectory_point.effort = []

        for key in joint_trajectory.joint_names:
            if key in joint_names[0]:  # generate joint positions for first joint
                joint_position = sin(w * t) * (max_pos_J3 - min_pos_J3) / 2 + (max_pos_J3 - min_pos_J3) / 2 + min_pos_J3
                trajectory_point.positions.append(joint_position)
            elif key in joint_names[1]:  # generate joint positions for second joint
                joint_position = cos(w * t) * (max_pos_J3 - min_pos_J3) / 2 + (max_pos_J3 - min_pos_J3) / 2 + min_pos_J3
                trajectory_point.positions.append(joint_position)
            else:
                trajectory_point.positions.append(hand_joints_goal[key])

            trajectory_point.velocities.append(0.0)
            trajectory_point.accelerations.append(0.0)
            trajectory_point.effort.append(0.0)

        joint_trajectory.points.append(trajectory_point)

    # Send trajectory to gb_hand_commander
    gb_hand_commander.run_joint_trajectory_unsafe(joint_trajectory)

    rospy.sleep(rospy.Duration(15))

#do_sinuisoid_fingers()

# ########################################################################################################
# DO FINGERS TRAJECTORY
#
import numpy as np
from control_msgs.msg import JointTrajectoryControllerState,\
    FollowJointTrajectoryActionResult, FollowJointTrajectoryActionGoal
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PartialTrajListener():

    def __init__(self):
        self.start_time_goals = []
        self.start_time_goals_trajectory = []
        self.goal_joint_names = []
        self.trajectories = []
        self.start_goals = False
        self.finish_goals = False

        self.joints_time = []
        self.ffj3_actual = []
        self.ffj3_desired = []
        self.ffj3_error = []
        self.rfj3_actual = []
        self.rfj3_desired = []
        self.rfj3_error = []
        self.ffj3_vel_actual = []
        self.rfj3_vel_actual = []
        self.ffj3_vel_desired = []
        self.rfj3_vel_desired = []
        self.ffj3_vel_error = []
        self.rfj3_vel_error = []

        rospy.Subscriber("/rh_trajectory_controller/state",
                         JointTrajectoryControllerState, self.callback)
        rospy.Subscriber(
            "/rh_trajectory_controller/follow_joint_trajectory/result",
            FollowJointTrajectoryActionResult, self.callback_result)
        rospy.Subscriber(
            "/rh_trajectory_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal, self.callback_goal)

    def callback(self, state):
        self.joint_names = state.joint_names
        self.ffj3_index = self.joint_names.index("rh_FFJ3")
        self.rfj3_index = self.joint_names.index("rh_RFJ3")

        if self.start_goals and not self.finish_goals:
            self.joints_time.append(state.header.stamp.to_sec())
            self.ffj3_actual.append(state.actual.positions[self.ffj3_index])
            self.ffj3_desired.append(state.desired.positions[self.ffj3_index])
            self.ffj3_error.append(state.error.positions[self.ffj3_index])
            self.rfj3_actual.append(state.actual.positions[self.rfj3_index])
            self.rfj3_desired.append(state.desired.positions[self.rfj3_index])
            self.rfj3_error.append(state.error.positions[self.rfj3_index])

            self.ffj3_vel_actual.append(state.actual.velocities[self.ffj3_index])
            self.rfj3_vel_actual.append(state.actual.velocities[self.rfj3_index])
            self.ffj3_vel_desired.append(state.desired.velocities[self.ffj3_index])
            self.rfj3_vel_desired.append(state.desired.velocities[self.rfj3_index])
            self.ffj3_vel_error.append(state.error.velocities[self.ffj3_index])
            self.rfj3_vel_error.append(state.error.velocities[self.rfj3_index])

    def callback_result(self, result):
        print ("Trajectory Goal: " + result.status.goal_id.id +
               " finished with status: " + str(result.status.status))

    def callback_goal(self, goal):
        self.start_goals = True
        self.goal_joint_names.append(goal.goal.trajectory.joint_names)
        self.start_time_goals.append(goal.header.stamp.to_sec())
        self.start_time_goals_trajectory.append(goal.goal.trajectory.header.stamp.to_sec())
        self.trajectories.append(goal.goal.trajectory.points)

    def plot_settings(self, plt):
        ax = plt.gca()
        plt.grid(which='both', axis='both')
        plt.setp(ax.get_xticklabels(), fontsize=8)
        plt.setp(ax.get_yticklabels(), fontsize=8)
        plt.xlabel('Time (s)')
        ax.xaxis.label.set_size(10)
        ax.yaxis.label.set_size(10)

    def graph(self):
        time_zero = self.joints_time[0]
        time = np.array(self.joints_time) - time_zero

        plt.figure()

        # Plot goal trajectories waypoints
        time_ffj3_traj = []
        angle_ffj3_traj = []
        time_rfj3_traj = []
        angle_rfj3_traj = []
        for i, traj in enumerate(self.trajectories):
            ffj3_goal_index = self.goal_joint_names[i].index("rh_FFJ3") if "rh_FFJ3" in self.goal_joint_names[i] else -1
            rfj3_goal_index = self.goal_joint_names[i].index("rh_RFJ3") if "rh_RFJ3" in self.goal_joint_names[i] else -1

            for point in traj:
                if ffj3_goal_index > -1:
                    time_ffj3_traj.append(point.time_from_start.to_sec() +
                                          self.start_time_goals_trajectory[i] -
                                          time_zero)
                    angle_ffj3_traj.append(point.positions[ffj3_goal_index])
                if rfj3_goal_index > -1:
                    time_rfj3_traj.append(point.time_from_start.to_sec() +
                                          self.start_time_goals_trajectory[i] -
                                          time_zero)
                    angle_rfj3_traj.append(point.positions[rfj3_goal_index])

            if ffj3_goal_index > -1:
                plt.subplot(3, 2, 1)
                plt.plot(time_ffj3_traj, angle_ffj3_traj, 'o',
                         label="Traj " + str(i + 1))

            if rfj3_goal_index > -1:
                plt.subplot(3, 2, 2)
                plt.plot(time_rfj3_traj, angle_rfj3_traj, 'o',
                         label="Traj " + str(i + 1))

            time_ffj3_traj = []
            angle_ffj3_traj = []
            time_rfj3_traj = []
            angle_rfj3_traj = []

        # Plot trajectories
        plt.subplot(3, 2, 1)
        plt.plot(time, self.ffj3_actual, 'black', label="Actual traj")
        plt.plot(time, self.ffj3_desired, 'green', label="Desired traj")
        plt.ylabel('FFJ3 Actual position (rad)')
        self.plot_settings(plt)
        plt.ylim(ymax=2.2, ymin=-0.1)
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=5,
                   mode="expand", borderaxespad=0., prop={'size': 8})

        plt.subplot(3, 2, 2)
        plt.plot(time, self.rfj3_actual, 'black', label="Actual traj")
        plt.plot(time, self.rfj3_desired, 'green', label="Desired traj")
        plt.ylabel('RFJ3 Actual position (rad)')
        self.plot_settings(plt)
        plt.ylim(ymax=2.2, ymin=-0.1)
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4,
                   mode="expand", borderaxespad=0., prop={'size': 8})

        plt.subplot(3, 2, 3)
        plt.plot(time, self.ffj3_vel_actual, 'black', label="Actual traj")
        plt.plot(time, self.ffj3_vel_desired, 'green', label="Desired traj")
        plt.ylabel('FFJ3 Actual velocity')
        plt.xlim(xmax=16, xmin=0)
        self.plot_settings(plt)
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4,
                   mode="expand", borderaxespad=0., prop={'size': 8})

        plt.subplot(3, 2, 4)
        plt.plot(time, self.rfj3_vel_actual, 'black', label="Actual traj")
        plt.plot(time, self.rfj3_vel_desired, 'green', label="Desired traj")
        plt.ylabel('RFJ3 Actual velocity')
        plt.xlim(xmax=16, xmin=0)
        self.plot_settings(plt)
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4,
                   mode="expand", borderaxespad=0., prop={'size': 8})

        plt.subplot(3, 2, 5)
        plt.plot(time, self.ffj3_vel_error, 'red', label="Error traj")
        plt.ylabel('FFJ3 Velocity Error')
        plt.xlim(xmax=16, xmin=0)
        self.plot_settings(plt)

        plt.subplot(3, 2, 6)
        plt.plot(time, self.rfj3_vel_error, 'red', label="Error traj")
        plt.ylabel('RFJ3 Velocity Error')
        plt.xlim(xmax=16, xmin=0)
        self.plot_settings(plt)

        plt.subplots_adjust(left=0.07, right=0.96, bottom=0.083, top=0.90)
        plt.show()


def construct_trajectory_point(joint_trajectory, posture, duration):
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration.from_sec(float(duration))
    for key in joint_trajectory.joint_names:
        trajectory_point.positions.append(posture[key])
    return trajectory_point

# 6 position goals are specified
grasp1 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.5235, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.5235, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp2 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.0472, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.0472, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp3 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.4, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.4, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp4 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.5, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.5, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

grasp5 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
          'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
          'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
          'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
          'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
          'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

# Two partial trajectories are defined here for joint rh_FFJ3
grasp_partial_1 = {'rh_FFJ3': 1.06}
grasp_partial_2 = {'rh_FFJ3': 1.2}

open_hand_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
                  'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
                  'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
                  'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
                  'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
                  'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

def do_fingers_trajectory():
    listener = PartialTrajListener()

    # Adjust poses according to the hand loaded
    open_hand_current = dict([(i, open_hand_pose[i]) for i in gb_joints if i in open_hand_pose])
    grasp1_current = dict([(i, grasp1[i]) for i in gb_joints if i in grasp1])
    grasp2_current = dict([(i, grasp2[i]) for i in gb_joints if i in grasp2])
    grasp3_current = dict([(i, grasp3[i]) for i in gb_joints if i in grasp3])
    grasp4_current = dict([(i, grasp4[i]) for i in gb_joints if i in grasp4])
    grasp5_current = dict([(i, grasp5[i]) for i in gb_joints if i in grasp5])

    start_time = rospy.Time.now()

    # Opening hand
    rospy.loginfo("Moving hand to open position")
    trajectory_start_time = 1.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(open_hand_current.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(joint_trajectory, open_hand_current, 1.0)
    joint_trajectory.points.append(trajectory_point)
    gb_hand_commander.run_joint_trajectory_unsafe(joint_trajectory, True)

    # Closing index and middle fingers. Trajectories are generated from grasp1 - grasp5 and run with gb_hand_commander
    rospy.loginfo("Closing index and ring fingers")
    trajectory_start_time = 4.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(grasp1_current.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(joint_trajectory, grasp1_current, 1.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(joint_trajectory, grasp2_current, 4.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(joint_trajectory, grasp3_current, 6.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(joint_trajectory, grasp4_current, 8.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(joint_trajectory, grasp5_current, 10.0)
    joint_trajectory.points.append(trajectory_point)
    gb_hand_commander.run_joint_trajectory_unsafe(joint_trajectory, False)

    # Interrupting trajectory of index using two partial trajectories
    rospy.loginfo("Moving index finger to partial trajectory goals")
    rospy.sleep(2)
    trajectory_start_time = 8.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(grasp_partial_1.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(joint_trajectory, grasp_partial_1, 1.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(joint_trajectory, grasp_partial_2, 3.0)
    joint_trajectory.points.append(trajectory_point)
    gb_hand_commander.run_joint_trajectory_unsafe(joint_trajectory, False)

    graphs_finished = False

    rate = rospy.Rate(10) # 10hz

    # Do not exit until graphs closed
    while not rospy.is_shutdown():
        if len(listener.joints_time) > 5 and\
                (listener.joints_time[-1] - listener.joints_time[0]) > 15 and\
                not graphs_finished:
            listener.finish_goals = True
            listener.graph()
            graphs_finished = True
            break
        rate.sleep()

# do_fingers_trajectory()

# ########################################################################################################
# DO GRASP POSE
#

joint_names = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4',
               'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5',
               'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4',
               'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4',
               'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5']

position = [1.07, 0.26, 0.88, -0.34, 0.85, 0.60,
            0.21, -0.23, 0.15, 1.06, 0.16, 1.04,
            0.05, 1.04, 0.34, 0.68, -0.24, 0.35,
            0.69, 0.18, 1.20, -0.11]

def gbGraspPose():
    grasp_pose = dict(zip(joint_names, position))

    # Adjust poses according to the hand loaded
    open_hand_current  = dict([(i, open_hand_pose[i]) for i in gb_joints if i in open_hand_pose])
    grasp_pose_current = dict([(i, grasp_pose[i]) for i in gb_joints if i in grasp_pose])

    # Partial list of goals
    grasp_partial_1 = {'rh_FFJ3': 0.50}

    start_time = rospy.Time.now()

    # Move hand using move_to_joint_value_target_unsafe to 1st position
    gb_hand_commander.move_to_joint_value_target_unsafe(open_hand_current, 1.0, True)

    rospy.sleep(2)

    # Move hand using run_joint_trajectory_unsafe to joint angles specified in 'position' list
    gb_hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

    trajectory_start_time = 2.0
    joint_trajectory = JointTrajectory()

    # Construct and send partial trajectory for joint listed in grasp_partial_1
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(grasp_partial_1.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(joint_trajectory, grasp_partial_1, 1.0)
    joint_trajectory.points.append(trajectory_point)

    gb_hand_commander.run_joint_trajectory_unsafe(joint_trajectory, True)
    rospy.sleep(2)

#gbGraspPose()

# ########################################################################################################
# MOVEIT
# https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/scripts/move_group_python_interface_tutorial.py
#
def move_group():
    # # Getting Basic Information
    # # ^^^^^^^^^^^^^^^^^^^^^^^^^
    # #
    # # We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % gb_arm_joint_group.get_planning_frame()

    # # We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % gb_arm_joint_group.get_end_effector_link()

    # # We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print gb_robot.get_group_names()

    # # Sometimes for debugging it is useful to print the entire state of the
    # # robot.
    print "============ Printing robot state"
    print gb_robot.get_current_state()
    print "============"


    # # Planning to a Pose goal
    # # ^^^^^^^^^^^^^^^^^^^^^^^
    # # We can plan a motion for this group to a desired pose for the
    # # end-effector
    print "============ Generating plan 1"
    #pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.w = 1.0
    #pose_target.position.x = 0.150031
    #pose_target.position.y = -0.000046
    #pose_target.position.z = 0.773986
    #ball_pose = gb_gz_get_ball_pose()
    ball_pose = gb_gz_get_model_pose("cricket_ball", WORLD_FRAME)
    spawn_model_pose = ball_pose
    spawn_model_pose.position.x += 0.1


    f = open(GAZEBO_MODEL_PLATE_PATH,'r')
    spawn_model = f.read()
    gb_gz_srv_spawn_sdf_model("Plate", spawn_model, "", spawn_model_pose, WORLD_FRAME)

    ball_pose.position.z += 0.3
    gb_arm_joint_group.set_pose_target(ball_pose)

    # # Now, we call the planner to compute the plan
    # # and visualize it if successful
    # # Note that we are just planning, not asking move_group
    # # to actually move the robot
    arm_plan = gb_arm_joint_group.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)

    # # You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
    # # gb_arm_joint_group.plan() method does this automatically so this is not that useful
    # # here (it just displays the same trajectory again).
    print "============ Visualizing plan1"
    #display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #
    #display_trajectory.trajectory_start = gb_robot.get_current_state()
    #display_trajectory.trajectory.append(plan1)
    #gb_rviz_display_trajectory_publisher.publish(display_trajectory);

    #print "============ Waiting while plan1 is visualized (again)..."
    #rospy.sleep(5)


    # # Moving to a pose goal
    # # ^^^^^^^^^^^^^^^^^^^^^
    # #
    # # Moving to a pose goal is similar to the step above
    # # except we now use the go() function. Note that
    # # the pose goal we had set earlier is still active
    # # and so the robot will try to move to that goal. We will
    # # not use that function in this tutorial since it is
    # # a blocking function and requires a controller to be active
    # # and report success on execution of a trajectory.

    # Uncomment below line when working with a real robot
    # gb_arm_joint_group.go(wait=True)

    # Use execute instead if you would like the robot to follow
    # the plan that has already been computed
    # gb_arm_joint_group.execute(plan1)

    # ducta Move Plan 0++
    #group_variable_values = gb_arm_joint_group.get_current_joint_values()
    #print "============ Joint values 0: ", group_variable_values
    #gbMoveArm(group_variable_values)
    # OR
    gb_arm_joint_group.execute(arm_plan)
    #rospy.sleep(3)

    # Move FF to the ball
    gb_FF_joint_group.set_pose_target(hammer_pose)
    ff_plan  = gb_FF_joint_group.plan()
    rospy.sleep(2)
    gb_FF_joint_group.execute(ff_plan)

    # Move TH to the ball
    gb_TH_joint_group.set_pose_target(hammer_pose)
    th_plan  = gb_TH_joint_group.plan()
    rospy.sleep(2)
    gb_TH_joint_group.execute(th_plan)

    return
    # ducta --

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # # Planning to a joint-space goal
    # # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    # #
    # # Let's set a joint space goal and move towards it.
    # # First, we will clear the pose target we had just set.

    gb_arm_joint_group.clear_pose_targets()

    # # Then, we will get the current set of joint values for the group
    group_variable_values = gb_arm_joint_group.get_current_joint_values()
    print "============ Joint values 1: ", group_variable_values

    # # Now, let's modify one of the joints, plan to the new joint
    # # space goal and visualize the plan
    group_variable_values[0] = 1.0
    gb_arm_joint_group.set_joint_value_target(group_variable_values)

    plan2 = gb_arm_joint_group.plan()
    #display_trajectory.trajectory.append(plan2)
    #gb_rviz_display_trajectory_publisher.publish(display_trajectory);

    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(5)
    # ducta Move Plan 2++
    group_variable_values = gb_arm_joint_group.get_current_joint_values()
    print "============ Joint values 2: ", group_variable_values
    gbMoveArm(group_variable_values)
    # ducta --

    # # Cartesian Paths
    # # ^^^^^^^^^^^^^^^
    # # You can plan a cartesian path directly by specifying a list of waypoints
    # # for the end-effector to go through.
    waypoints = []

    # start with the current pose
    waypoints.append(gb_arm_joint_group.get_current_pose().pose)

    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x + 0.1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    # second move down
    wpose.position.z -= 0.10
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

    # # We want the cartesian path to be interpolated at a resolution of 1 cm
    # # which is why we will specify 0.01 as the eef_step in cartesian
    # # translation.  We will specify the jump threshold as 0.0, effectively
    # # disabling it.
    (plan3, fraction) = gb_arm_joint_group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.01,        # eef_step
                                 0.0)         # jump_threshold

    #display_trajectory.trajectory.append(plan3)
    #gb_rviz_display_trajectory_publisher.publish(display_trajectory);
    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(5)
    # ducta Move Plan 3++
    group_variable_values = gb_arm_joint_group.get_current_joint_values()
    print "============ Joint values 3: ", group_variable_values
    gbMoveArm(group_variable_values)
    # ducta --

    # # Adding/Removing Objects and Attaching/Detaching Objects
    # # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    # # First, we will define the collision object message
    collision_object = moveit_msgs.msg.CollisionObject()

    # # END_TUTORIAL
    print "============ STOPPING"

#move_group()


# ########################################################################################################
# SMART GRASPING
#

# smart_grasping_sandbox.py
# https://github.com/shadow-robot/smart_grasping_sandbox/blob/master/smart_grasping_sandbox/src/smart_grasping_sandbox/smart_grasper.py
#
class SmartGrasp(object):
    """
    This is the helper library to easily access the different functionalities of the simulated robot
    from python.
    """

    __last_joint_state = None

    def __init__(self):
        self.__joint_state_sub = rospy.Subscriber("/joint_states", JointState,
                                                  self.__joint_state_cb, queue_size=1)

        self.arm_commander  = MoveGroupCommander(MAIN_ARM_GROUP_NAME)
        self.hand_commander = MoveGroupCommander(MAIN_HAND_GROUP_NAME)
        # !NOTE: These below don't work!
        #self.arm_commander  = gb_arm_commander._move_group_commander
        #self.hand_commander = gb_hand_commander._move_group_commander

        # self.__hand_traj_client = SimpleActionClient("/hand_controller/follow_joint_trajectory",
        #                                              FollowJointTrajectoryAction)
        # self.__arm_traj_client = SimpleActionClient("/arm_controller/follow_joint_trajectory",
        #                                             FollowJointTrajectoryAction)
        #
        # if self.__hand_traj_client.wait_for_server(timeout=rospy.Duration(4.0)) is False:
        #     rospy.logfatal("Failed to connect to /hand_controller/follow_joint_trajectory in 4sec.")
        #     raise Exception("Failed to connect to /hand_controller/follow_joint_trajectory in 4sec.")
        #
        # if self.__arm_traj_client.wait_for_server(timeout=rospy.Duration(4.0)) is False:
        #     rospy.logfatal("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")
        #     raise Exception("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")

        self.reset_world()
        arm_target = self.__compute_arm_target_for_ball()
        rospy.loginfo('ARM_TARGET:' + str(arm_target.position.x) + '-' + \
                                      str(arm_target.position.y) + '-' + \
                                      str(arm_target.position.z) + '\n')
        rospy.loginfo('ARM GROUP NAME:'  + self.arm_commander.get_name())
        rospy.loginfo('HAND GROUP NAME:' + self.hand_commander.get_name())

        self.__lift(arm_target)
        self.__pre_grasp(arm_target)
        self.__grasp(arm_target)
        # OR
        #self.pick()

    def reset_world(self):
        """
        Resets the object poses in the world and the robot joint angles.
        """
        gb_gz_srv_switch_ctrl.call(start_controllers=[],
                                stop_controllers=["hand_controller", "arm_controller", "joint_state_controller"],
                                strictness=SwitchControllerRequest.BEST_EFFORT)
        gb_gz_srv_pause_physics.call()

        joint_names = ['ra_shoulder_pan_joint', 'ra_shoulder_lift_joint', 'ra_elbow_joint',
                       'ra_wrist_1_joint', 'ra_wrist_2_joint', 'ra_wrist_3_joint']
        joint_positions = [1.2, 0.5, -1.5, -0.5, -1.5, 0.0]

        gb_gz_srv_set_model.call(model_name="ur10srh",
                              urdf_param_name="robot_description",
                              joint_names=joint_names,
                              joint_positions=joint_positions)

        timer = Timer(0.0, self.__start_ctrl)
        timer.start()

        time.sleep(0.1)
        gb_gz_srv_unpause_physics.call()

        gb_gz_srv_reset_world.call()

    def get_tip_pose(self):
        """
        Gets the current pose of the robot's tooltip in the world frame.
        @return the tip pose
        """
        return self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose

    def move_tip_absolute(self, target):
        """
        Moves the tooltip to the absolute target in the world frame
        @param target is a geometry_msgs.msg.Pose
        @return True on success
        """
        self.arm_commander.set_start_state_to_current_state()
        self.arm_commander.set_pose_targets([target])
        plan = self.arm_commander.plan()
        if not self.arm_commander.execute(plan):
            return False
        return True

    def move_tip(self, x=0., y=0., z=0., roll=0., pitch=0., yaw=0.):
        """
        Moves the tooltip in the world frame by the given x,y,z / roll,pitch,yaw.
        @return True on success
        """
        transform = PyKDL.Frame(PyKDL.Rotation.RPY(pitch, roll, yaw),
                                PyKDL.Vector(-x, -y, -z))

        tip_pose = self.get_tip_pose()
        tip_pose_kdl = posemath.fromMsg(tip_pose)
        final_pose = toMsg(tip_pose_kdl * transform)

        self.arm_commander.set_start_state_to_current_state()
        self.arm_commander.set_pose_targets([final_pose])
        plan = self.arm_commander.plan()
        if not  self.arm_commander.execute(plan):
            return False
        return True

    def send_command(self, command, duration=0.2):
        """
        Send a dictionnary of joint targets to the arm and hand directly.

        @param command: a dictionnary of joint names associated with a target:
                        {"H1_F1J1": -1.0, "shoulder_pan_joint": 1.0}
        @param duration: the amount of time it will take to get there in seconds. Needs to be bigger than 0.0
        """
        hand_goal = None
        arm_goal = None

        for joint, target in command.items():
            if "rh_FFJ1" in joint:
                if not hand_goal:
                    hand_goal = FollowJointTrajectoryGoal()

                    point = JointTrajectoryPoint()
                    point.time_from_start = rospy.Duration.from_sec(duration)

                    hand_goal.trajectory.points.append(point)

                hand_goal.trajectory.joint_names.append(joint)
                hand_goal.trajectory.points[0].positions.append(target)
            else:
                if not arm_goal:
                    arm_goal = FollowJointTrajectoryGoal()

                    point = JointTrajectoryPoint()
                    point.time_from_start = rospy.Duration.from_sec(duration)

                    arm_goal.trajectory.points.append(point)

                arm_goal.trajectory.joint_names.append(joint)
                arm_goal.trajectory.points[0].positions.append(target)

        if arm_goal:
            self.__arm_traj_client.send_goal_and_wait(arm_goal)
        if hand_goal:
            self.__hand_traj_client.send_goal_and_wait(hand_goal)

    def get_current_joint_state(self):
        """
        Gets the current state of the robot.

        @return joint positions, velocity and efforts as three dictionnaries
        """
        joints_position = {n: p for n, p in
                           zip(self.__last_joint_state.name,
                               self.__last_joint_state.position)}
        joints_velocity = {n: v for n, v in
                           zip(self.__last_joint_state.name,
                           self.__last_joint_state.velocity)}
        joints_effort = {n: v for n, v in
                         zip(self.__last_joint_state.name,
                         self.__last_joint_state.effort)}
        return joints_position, joints_velocity, joints_effort

    def open_hand(self):
        """
        Opens the hand.

        @return True on success
        """
        self.hand_commander.set_named_target("open") # <group_state> defined in SRDF file
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            return False

        return True

    def close_hand(self):
        """
        Closes the hand.

        @return True on success
        """
        self.hand_commander.set_named_target("pack") # <group_state> defined in SRDF file
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            return False

        return True

    def check_fingers_collisions(self, enable=True):
        """
        Disables or enables the collisions check between the fingers and the objects / table

        @param enable: set to True to enable / False to disable
        @return True on success
        """
        objects = ["cricket_ball__link", "drill__link", "cafe_table__link"]

        while gb_gz_pub_planning_scene.get_num_connections() < 1:
            rospy.loginfo("waiting for someone to subscribe to the /planning_scene")
            rospy.sleep(0.1)

        request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = gb_gz_srv_get_planning_scene(request)
        acm = response.scene.allowed_collision_matrix

        for object_name in objects:
            if object_name not in acm.entry_names:
                # add object to allowed collision matrix
                acm.entry_names += [object_name]
                for row in range(len(acm.entry_values)):
                    acm.entry_values[row].enabled += [False]

                new_row = deepcopy(acm.entry_values[0])
                acm.entry_values += {new_row}

        for index_entry_values, entry_values in enumerate(acm.entry_values):
            if "H1_F" in acm.entry_names[index_entry_values]:
                for index_value, _ in enumerate(entry_values.enabled):
                    if acm.entry_names[index_value] in objects:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
            elif acm.entry_names[index_entry_values] in objects:
                for index_value, _ in enumerate(entry_values.enabled):
                    if "H1_F" in acm.entry_names[index_value]:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True

        planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        gb_gz_pub_planning_scene.publish(planning_scene_diff)
        rospy.sleep(1.0)

        return True

    def pick(self):
        """
        Does its best to pick the ball.
        """
        #rospy.loginfo("Moving to Pregrasp")
        #self.open_hand()
        #time.sleep(0.1)

        ball_pose = gb_gz_get_ball_pose()
        rospy.loginfo('BALL POSE:' + str(ball_pose.position.x) + '-' + str(ball_pose.position.y) + '-' + str(ball_pose.position.z) + '\n')
        ball_pose.position.z += 0.5

        #setting an absolute orientation (from the top)
        quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
        ball_pose.orientation.x = quaternion[0]
        ball_pose.orientation.y = quaternion[1]
        ball_pose.orientation.z = quaternion[2]
        ball_pose.orientation.w = quaternion[3]

        self.move_tip_absolute(ball_pose)
        time.sleep(0.1)

        rospy.loginfo("Pick - Grasping")
        self.move_tip(y=-0.16)
        time.sleep(0.1)
        self.check_fingers_collisions(False)
        time.sleep(0.1)
        #gbGraspPose()
        self.close_hand()
        time.sleep(0.1)

        rospy.loginfo("Pick - Lifting")
        for _ in range(50):
            self.move_tip(y=0.001)
            time.sleep(0.1)

        self.check_fingers_collisions(True)

    def __compute_arm_target_for_ball(self):
        ball_pose = gb_gz_get_ball_pose()

        # come at it from the top
        arm_target = ball_pose
        arm_target.position.z += 0.5

        quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
        arm_target.orientation.x = quaternion[0]
        arm_target.orientation.y = quaternion[1]
        arm_target.orientation.z = quaternion[2]
        arm_target.orientation.w = quaternion[3]

        return arm_target

    def __pre_grasp(self, arm_target):
        self.hand_commander.set_named_target("open")
        plan = self.hand_commander.plan()
        self.hand_commander.execute(plan, wait=True)

        for _ in range(10):
            self.arm_commander.set_start_state_to_current_state()
            self.arm_commander.set_pose_targets([arm_target])
            plan = self.arm_commander.plan()
            if self.arm_commander.execute(plan):
                rospy.loginfo("__pre_grasp done!")
                return True
            rospy.loginfo("__pre_grasp failed!")

    def __grasp(self, arm_target):
        waypoints = []
        waypoints.append(self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z -= 0.12
        waypoints.append(arm_above_ball)

        self.arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print fraction
        if not self.arm_commander.execute(plan):
            return False

        self.hand_commander.set_named_target("pack")
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            rospy.loginfo("__grasp failed!")
            return False

        rospy.loginfo("__grasp failed!")
        self.hand_commander.attach_object("cricket_ball__link")

    def __lift(self, arm_target):
        waypoints = []
        waypoints.append(self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z += 0.1
        waypoints.append(arm_above_ball)

        self.arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print fraction
        if not self.arm_commander.execute(plan):
            rospy.loginfo("__lift failed!")
            return False
        rospy.loginfo("__lift done!")

    def __start_ctrl(self):
        rospy.loginfo("STARTING CONTROLLERS")
        gb_gz_srv_switch_ctrl.call(start_controllers=["hand_controller", "arm_controller", "joint_state_controller"],
                                stop_controllers=[], strictness=1)

    def __joint_state_cb(self, msg):
        self.__last_joint_state = msg

def gbGraspObject():
    SmartGrasp()

#gbGraspObject()
# ########################################################################################################
# START TO DO FINGER LEARNING
#

# 1 - Keep moving fingers
#
def deep_keep_moving_fingers():
    for i in range(0,20):
        hand_joint_goals_1[deep_hand_joint_names[i]] = deep_joint_poses[i]
    while not rospy.is_shutdown():
        for i in range(0,20):
            if ((i%4 != 3) and (i!=20)):
                if (deep_joint_poses[i] >= 1.5):
                    deep_joint_poses[i] = 0.0
                else: deep_joint_poses[i] += 0.5
                joint_name = deep_hand_joint_names[i]
                #rospy.loginfo('MOVE to joint name ' + joint_name + "\n")
                hand_joint_goals_1[joint_name] = deep_joint_poses[i]
                rospy.loginfo("Moving fingers to joint states\n" + str(hand_joint_goals_1) + "\n")
                gb_hand_commander.move_to_joint_value_target_unsafe(hand_joint_goals_1, 0.5, True)
                hand_tactile_state = gb_hand_commander.get_tactile_state() # as a msg from a callback
                rospy.loginfo('HAND TACTILE STATE:' + str(hand_tactile_state))

#deep_keep_moving_fingers()

#2 - Spawning Plate Model Loop
#gb_gz_spawn_plate_model_loop(1.5)

# #######################################
from my_arm_utils import mk_grasp
#mk_grasp(hand_joint_goals_1)

# ########################################################################################################
# SHUT DOWN MOVE IT COMMANDER
#
# # When finished shut down moveit_commander.
moveit_commander.roscpp_shutdown()

rospy.spin()
# The spin() code simply sleeps until the is_shutdown() flag is True. It is mainly used to prevent your Python
# Main thread from exiting.
