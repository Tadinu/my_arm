#!/usr/bin/env python

#http://infohost.nmt.edu/tcc/help/pubs/python/web/new-new-method.html
#__new__ is static class method, while __init__ is instance method. __new__ has to create the instance first, so __init__ can initialize it. Note that __init__ takes self as parameter. Until you create instance there is no self.
#
#Now, I gather, that you're trying to implement singleton pattern in Python. There are a few ways to do that.
#
#Also, as of Python 2.6, you can use class decorators.
#
#def singleton(cls):
#    instances = {}
#    def getinstance():
#        if cls not in instances:
#            instances[cls] = cls()
#        return instances[cls]
#    return getinstance


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
import collections as col
import math
from math import degrees
from math import pi
import time
import threading
from threading import Timer
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_srvs.srv import Empty

# ROS Python Modules
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
import rospy
from rospy import Timer
import rospkg
import rosgraph.masterapi

import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Quaternion
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import tf as tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
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

# EigenGrasp
from eigen_grasp import EigenGrasp, EigenGraspInterface

# MY ARM modules
from my_arm_utils import mk_grasp
from partial_trajectory_listener import PartialTrajListener

# ########################################################################################################
# GLOBAL HELPER MEMBERS

# msg = rospy.wait_for_message("my_topic", MyType)
#
# This returns the message, and shuts down after receiving one message.
# You can specify a timeout as the third argument. It also doesn't require using a global.

# ########################################################################################################
# ROSPY INFO
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

GAZEBO_MODEL_PLATE_PATH       = "/home/brhm/DUC/RobotArm/src/my_arm/models/plate/model.sdf"
GAZEBO_MODEL_PLATE_NAME       = "plate"
GAZEBO_MODEL_THIN_PLATE_PATH  = "/home/brhm/DUC/RobotArm/src/my_arm/models/thin_plate/model.sdf"
GAZEBO_MODEL_THIN_PLATE_NAME  = "thin_plate"
GAZEBO_MODEL_HAMMER_PATH      = "/usr/share/gazebo-7/models/hammer/model.sdf"
GAZEBO_MODEL_HAMMER_NAME      = "hammer"
GAZEBO_MODEL_CRICKET_NAME     = "cricket_ball"

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
# GAZEBO SERVICES
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
    mpose.position.y = -0.39
    mpose.position.z = 1.29

    gb_gz_spawn_model(GAZEBO_MODEL_THIN_PLATE_NAME, GAZEBO_MODEL_THIN_PLATE_PATH, mpose)

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
    return gb_gz_get_model_pose(GAZEBO_MODEL_CRICKET_NAME, WORLD_FRAME)

def gb_gz_get_plate_pose():
    return gb_gz_get_model_pose(GAZEBO_MODEL_THIN_PLATE_NAME, WORLD_FRAME)

# ########################################################################################################
# ROSPY GLOBAL TIMER CALLBACK
#

# ########################################################################################################
# SHADOW HAND INFO
#
GB_CJOINT_STATES_TOPIC = "joint_states"
GB_CHAND_MOVE_TIME = 0.1
gb_hand_joint_names = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4',
                       'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4',
                       'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4',
                       'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5',
                       'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5',
                       'rh_WRJ1', 'rh_WRJ2']

gb_hand_joint_names_dict = {0: 'rh_FFJ1', 1 : 'rh_FFJ2', 2 : 'rh_FFJ3',  3: 'rh_FFJ4',
                            4: 'rh_MFJ1', 5 : 'rh_MFJ2', 6 : 'rh_MFJ3',  7: 'rh_MFJ4',
                            8: 'rh_RFJ1', 9 : 'rh_RFJ2', 10: 'rh_RFJ3', 11: 'rh_RFJ4',
                            12:'rh_LFJ1', 13: 'rh_LFJ2', 14: 'rh_LFJ3', 15: 'rh_LFJ4', 16: 'rh_LFJ5',
                            17:'rh_THJ1', 18: 'rh_THJ2', 19: 'rh_THJ3', 20: 'rh_THJ4', 21: 'rh_THJ5',
                            22:'rh_WRJ1', 23: 'rh_WRJ2'}

gb_hand_joint_goals_1 = {'rh_FFJ1': 1.25, 'rh_FFJ2': 1.71, 'rh_FFJ3': 1.49, 'rh_FFJ4': -0.02,
                         'rh_MFJ1': 1.31, 'rh_MFJ2': 1.66, 'rh_MFJ3': 1.49, 'rh_MFJ4': -0.02,
                         'rh_RFJ1': 1.47, 'rh_RFJ2': 1.59, 'rh_RFJ3': 1.49, 'rh_RFJ4': -0.01,
                         'rh_LFJ1': 1.41, 'rh_LFJ2': 1.60, 'rh_LFJ3': 1.49, 'rh_LFJ4': 0.02, 'rh_LFJ5': 0.061,
                         'rh_THJ1': 0.43, 'rh_THJ2': 0.64, 'rh_THJ3': -0.088, 'rh_THJ4': 0.49, 'rh_THJ5': 0.35,
                         'rh_WRJ1':0.00, 'rh_WRJ2':0.00}

GB_CHAND_DOF_NO = len(gb_hand_joint_names)

# From GRASPIT/graspit/models/robots/HumanHand/eigen/human_eigen_cgdb_refined.xml
gb_hand_eigengrasp_values = [ [0.050409 ,-0.41296 ,-0.042016 ,-0.00754 ,0.00000,
                               -0.51469 ,-0.02456 ,-0.01305 ,-0.04461 ,-0.53003,
                               0.01607 ,-0.06576 ,-0.10093 ,-0.45099 ,-0.17029,
                               -0.07539 ,0.05174 ,-0.12964 ,-0.06365 ,-0.01094],

                              [-0.049675 , 0.20746 , -0.33611 , -0.20765 , 0.00000,
                               0.18604 , -0.35635 , -0.33362 , 0.09129 , 0.02716,
                               -0.38659 , -0.26735 , 0.02125 , -0.19259 , -0.34394,
                               -0.26400 , -0.20613 , 0.00719 , -0.15979 , -0.04356],

                              [-0.15071 ,0.14512 ,-0.17381 ,-0.04835 ,0.00000,
                               0.16485 ,-0.06025 ,0.05891 ,0.16019 ,-0.11095,
                               0.17620 ,0.26620 ,0.17407 ,-0.40847 ,0.39977,
                               0.12555 ,0.07599 ,0.17580 ,-0.58248 ,0.02899],


                              [-0.067079, 0.37761, 0.12263, 0.09017, 0.00000,
                               0.26142, 0.25866, 0.10387, 0.19362, -0.14321,
                               0.08207, 0.22040, 0.04200, -0.50834, -0.24976,
                               -0.08701, -0.01666, -0.08333, 0.48276, 0.00090],

                              [6.02E-03, 4.74E-02, 4.59E-02, -0.33341, 0.00000,
                               -0.05043, -0.14934, -0.56798, -0.00096, -0.10468,
                               0.19486, -0.05894, -0.03538, -0.08053, 0.43649,
                               0.25710, -0.10565, -0.00662, 0.39313, -0.23341],

                              [-0.14599, -0.15442, 0.27379, -0.24099, 0.00000,
                               0.07924, -0.12278, 0.23768, 0.08843, 0.05421,
                               0.51388, -0.33799, 0.24779, -0.01156, -0.02338,
                               -0.45430, -0.18844, 0.12005, -0.01788, -0.19021]
                             ]

gb_hand_eigengrasp_ori = [0.18102, 0.45792, 0.63217, 0.29167, 0.00000,
                          0.47316, 0.77439, 0.29466, -0.19238, 0.51225,
                          0.61343, 0.38257, -0.29595, 0.44209, 0.37250,
                          0.22419, 0.04983, -0.72210, 0.64246, 0.06027]

GB_CEIGENGRASP_NO = len(gb_hand_eigengrasp_values)

# ########################################################################################################
# DO ARM TEACHING
#
# Specify goals for hand and arm

gb_hand_joint_goals_2 = {'rh_RFJ2': 0.55, 'rh_RFJ3': 0.08, 'rh_RFJ1': 0.03, 'rh_RFJ4': -0.15, 'rh_LFJ4': -0.35,
                      'rh_LFJ5': 0.23, 'rh_LFJ1': 0.02, 'rh_LFJ2': 0.49, 'rh_LFJ3': -0.02, 'rh_THJ2': -0.08,
                      'rh_THJ3': -0.08, 'rh_THJ1': 0.15, 'rh_THJ4': 0.56, 'rh_THJ5': -0.17, 'rh_FFJ4': -0.34,
                      'rh_FFJ2': 0.30, 'rh_FFJ3': 0.16, 'rh_FFJ1': 0.01, 'rh_MFJ3': 0.19, 'rh_MFJ2': 0.50,
                      'rh_MFJ1': 0.00, 'rh_MFJ4': -0.07}

gb_hand_joint_goals_3 = {'rh_RFJ2': 0.63, 'rh_RFJ3': 0.77, 'rh_RFJ1': 0.033, 'rh_RFJ4': -0.02, 'rh_LFJ4': -0.32,
                      'rh_LFJ5': 0.67, 'rh_LFJ1': 0.02, 'rh_LFJ2': 0.73, 'rh_LFJ3': 0.21, 'rh_THJ2': -0.06,
                      'rh_THJ3': -0.04, 'rh_THJ1': 0.39, 'rh_THJ4': 0.85, 'rh_THJ5': 0.40, 'rh_FFJ4': -0.35,
                      'rh_FFJ2': 0.90, 'rh_FFJ3': 0.56, 'rh_FFJ1': 0.02, 'rh_MFJ3': 0.59, 'rh_MFJ2': 0.84,
                      'rh_MFJ1': 0.05, 'rh_MFJ4': -0.08}

gb_hand_joint_goals_4 = {'rh_RFJ2': 0.57, 'rh_RFJ3': 0.27, 'rh_RFJ1': 0.04, 'rh_RFJ4': -0.01, 'rh_LFJ4': -0.28,
                      'rh_LFJ5': 0.39, 'rh_LFJ1': 0.01, 'rh_LFJ2': 0.72, 'rh_LFJ3': -0.12, 'rh_THJ2': -0.19,
                      'rh_THJ3': -0.05, 'rh_THJ1': 0.38, 'rh_THJ4': 0.85, 'rh_THJ5': -0.12, 'rh_FFJ4': -0.32,
                      'rh_FFJ2': 0.64, 'rh_FFJ3': -0.03, 'rh_FFJ1': 0.04, 'rh_MFJ3': 0.04, 'rh_MFJ2': 0.83,
                      'rh_MFJ1': 0.01, 'rh_MFJ4': -0.05}

gb_hand_joint_goals_5 = {'rh_RFJ2': 1.58, 'rh_RFJ3': 1.52, 'rh_RFJ1': 1.34, 'rh_RFJ4': -0.06, 'rh_LFJ4': -0.20,
                      'rh_LFJ5': 0.09, 'rh_LFJ1': 1.47, 'rh_LFJ2': 1.57, 'rh_LFJ3': 1.40, 'rh_THJ2': -0.01,
                      'rh_THJ3': -0.041, 'rh_THJ1': 0.29, 'rh_THJ4': 0.59, 'rh_THJ5': -1.36, 'rh_FFJ4': 0.03,
                      'rh_FFJ2': 1.72, 'rh_FFJ3': 1.41, 'rh_FFJ1': 1.21, 'rh_MFJ3': 1.39, 'rh_MFJ2': 1.65,
                      'rh_MFJ1': 1.33, 'rh_MFJ4': 0.12}

gb_arm_joint_goals_1 = {'ra_shoulder_lift_joint': -1.87, 'ra_elbow_joint': 1.76, 'ra_wrist_2_joint': 0.03,
                     'ra_wrist_1_joint': -0.86, 'ra_shoulder_pan_joint': -2.64, 'ra_wrist_3_joint': 0.69,
                     'rh_WRJ2': -0.02, 'rh_WRJ1': 0.03}

gb_arm_joint_goals_2 = {'ra_shoulder_lift_joint': -1.86, 'ra_elbow_joint': 1.85, 'ra_wrist_2_joint': -0.19,
                     'ra_wrist_1_joint': -0.96, 'ra_shoulder_pan_joint': -1.78, 'ra_wrist_3_joint': 1.06,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': -0.02}

gb_arm_joint_goals_3 = {'ra_shoulder_lift_joint': -1.86, 'ra_elbow_joint': 1.90, 'ra_wrist_2_joint': -0.18,
                     'ra_wrist_1_joint': -0.96, 'ra_shoulder_pan_joint': -1.78, 'ra_wrist_3_joint': 1.06,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.15}

gb_arm_joint_goals_4 = {'ra_shoulder_lift_joint': -1.33, 'ra_elbow_joint': 1.11, 'ra_wrist_2_joint': 1.00,
                     'ra_wrist_1_joint': 0.13, 'ra_shoulder_pan_joint': -1.49, 'ra_wrist_3_joint': 3.27,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.15}

gb_arm_joint_goals_5 = {'ra_shoulder_lift_joint': -1.45, 'ra_elbow_joint': 1.11, 'ra_wrist_2_joint': 0.90,
                     'ra_wrist_1_joint': 0.45, 'ra_shoulder_pan_joint': -0.95, 'ra_wrist_3_joint': 0.09,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.16}

gb_arm_joint_goals_6 = {'ra_shoulder_lift_joint': -1.35, 'ra_elbow_joint': 1.16, 'ra_wrist_2_joint': 0.96,
                     'ra_wrist_1_joint': 0.39, 'ra_shoulder_pan_joint': -0.91, 'ra_wrist_3_joint': 0.09,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.15}

gb_arm_joint_goals_7 = {'ra_shoulder_lift_joint': -1.35, 'ra_elbow_joint': 1.04, 'ra_wrist_2_joint': 1.55,
                     'ra_wrist_1_joint': 0.08, 'ra_shoulder_pan_joint': -1.64, 'ra_wrist_3_joint': -1.41,
                     'rh_WRJ2': -0.03, 'rh_WRJ1': 0.15}

gb_arm_joint_goals_8 = {'ra_shoulder_lift_joint': -1.55, 'ra_elbow_joint': 1.41, 'ra_wrist_2_joint': 0.02,
                     'ra_wrist_1_joint': 0.61, 'ra_shoulder_pan_joint': -1.55, 'ra_wrist_3_joint': -0.57,
                     'rh_WRJ2': -0.04, 'rh_WRJ1': 0.16}

gb_deep_joint_poses = [0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 1.5707]

gb_arm_joint_goals =  {'ra_shoulder_pan_joint' : 0.0,
                       'ra_shoulder_lift_joint': 0.0,
                       'ra_elbow_joint': 0.0,
                       'ra_wrist_1_joint': 0.0,
                       'ra_wrist_2_joint': 0.0,
                       'ra_wrist_3_joint': 0.0,
                       'rh_WRJ2': 0.0,
                       'rh_WRJ1': 0.0}

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from numpy import sin, cos, pi, arange

class ShadowHandAgent(object):

    def __init__(self):
        self.ros_initialize_services()
        # rospy.init_node("shadow_hand_agent", anonymous=True)
        # r = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        #     pub.publish("hello")
        #     r.sleep()

        # Initialize Gazebo Services
        self.gz_initialize_gazebo_services()

        # Initialize Gazebo Services
        self.moveit_initialize_services()
        #
        # HandFinder is used to access the hand parameters
        self._hand_finder = HandFinder()
        self._hand_parameters = self._hand_finder.get_hand_parameters()
        self._hand_prefix = self._hand_parameters.mapping.values()
        rospy.loginfo('HAND TYPE:' + self._hand_prefix[0])
        self._hand_serial = self._hand_parameters.mapping.keys()[0]
        rospy.loginfo('HAND SERIAL:' + self._hand_serial)

        # Arm & Hand Commander
        self._arm_commander  = SrArmCommander(name=MAIN_ARM_GROUP_NAME, set_ground=False)
        self._hand_commander = SrHandCommander(name=MAIN_HAND_GROUP_NAME, prefix="", \
                                                 hand_parameters=self._hand_parameters,   \
                                                 hand_serial=self._hand_serial)
        #self._hand_commander = SrHandCommander(name="right_hand", prefix="rh")

        hand_tactile_type  = self._hand_commander.get_tactile_type()
        rospy.loginfo('HAND TACTILE TYPE:' + self._hand_serial)
        hand_tactile_state = self._hand_commander.get_tactile_state() # as a msg from a callback
        if(hand_tactile_state != None):
            rospy.loginfo('HAND TACTILE STATE:' + hand_tactile_state)

        # Hand Mapping(to do the grasp pose)
        self._hand_mapping = self._hand_parameters.mapping[self._hand_serial]

        # Hand Joints
        self._hand_joints = self._hand_finder.get_hand_joints()[self._hand_mapping]
        print('HAND JOINTS: ', len(self._hand_joints), self._hand_joints)

        self._joint_states_lock = threading.Lock()
        self._joint_states_listener = rospy.Subscriber(
                                     GB_CJOINT_STATES_TOPIC, JointState, self.joint_states_callback)
        # ---------------------------------------------------------------------------------------------------
        # Initialize EigenGrasps Info (after initializing the hand specifics)
        self.eigen_grasp_initialize()

        self._hand_commander.move_to_joint_value_target_unsafe(gb_hand_joint_goals_5, 0.01, True)

        #threading.Thread(None, rospy.spin)

    def __del__(self):
        # ####################################################################################################
        # SHUT DOWN MOVE IT COMMANDER
        #
        # # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    def reset(self):
        gb_gz_spawn_plate_model()

    # ########################################################################################################
    # INIT EIGEN GRASPS
    #
    def eigen_grasp_initialize(self):

        # Eigen Graps List
        #
        global gb_hand_eigengrasps
        gb_hand_eigengrasps = [EigenGrasp(self.getJointsCount(), 0.0) for i in range(GB_CEIGENGRASP_NO)]
        for i in range(len(gb_hand_eigengrasps)):
            gb_hand_eigengrasps[i].setVals(gb_hand_eigengrasp_values[i])

        # Eigen Graps Interface
        #
        global gb_hand_eigengrasp_interface
        gb_hand_eigengrasp_interface = EigenGraspInterface(self, gb_hand_eigengrasps, gb_hand_eigengrasp_ori)

    # ########################################################################################################
    # INIT ROS SERVICES
    #
    def ros_initialize_services(self):
        #global gb_ros_timer
        #gb_ros_timer = rospy.Timer(rospy.Duration(1), self.ros_timer_callback)
        ##!NOTE: No need for gb_ros_timer.start() , already starts

        # RVIZ PUBLISHER INIT
        #
        # # We create this DisplayTrajectory publisher which is used below to publish
        # # trajectories for RVIZ to visualize.
        global gb_rviz_display_trajectory_publisher
        gb_rviz_display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                                moveit_msgs.msg.DisplayTrajectory)

        # # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
        print "============ Waiting for RVIZ..."
        #rospy.sleep(10)
        print "============ Starting tutorial "

        # HERE WE PUT THE rospy spin in the one using the hand agent class!
        #rospy.spin()
        # The spin() code simply sleeps until the is_shutdown() flag is True. It is mainly used to prevent your Python
        # Main thread from exiting.
        #
        # The final addition, rospy.spin() simply keeps your node from exiting
        # until the node has been shutdown. Unlike roscpp, rospy.spin() does not
        # effect the subscriber callback functions, it simply puts the node to
        # sleep until it has been shut down. In rospy, each subscriber has its
        # own thread which handles its callback functions automatically.
        #
        # !Note: There is no rospy spinonce!!! You don't need spinOnce in rospy as the I/O is multithreaded.

    def ros_timer_callback(self, event):
        #rospy.loginfo("Ros Timer Callback\n")
        #Testing:
        #gb_gz_spawn_plate_model()
        return

    # ########################################################################################################
    # INIT MOVEIT COMMANDER SERVICES
    #
    def moveit_initialize_services(self):
        # First initialize moveit_commander and rospy.
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a RobotCommander object.  This object is an interface to
        # the robot as a whole.
        # http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
        global gb_robot
        gb_robot = moveit_commander.RobotCommander()
        # gb_robot.get_group_names()
        # gb_robot.get
        # gb_robot.get_current_state()
        # gb_robot.get_current_variable_values

        # Instantiate a PlanningSceneInterface object.  This object is an interface
        # to the world surrounding the robot.
        #scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the left
        # arm.  This interface can be used to plan and execute motions on the left arm.
        global gb_arm_joint_group
        gb_arm_joint_group = moveit_commander.MoveGroupCommander(MAIN_ARM_GROUP_NAME)
        global gb_hand_joint_group
        gb_hand_joint_group = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_NAME)

        global gb_TH_joint_group
        gb_TH_joint_group = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_THUMB)
        global gb_FF_joint_group
        gb_FF_joint_group = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_FIRST_FINGER)
        #global gb_MF_joint_group
        #gb_MF_joint_group = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_MIDDLE_FINGER)
        #global gb_RF_joint_group
        # gb_RF_joint_group = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_RING_FINGER)
        global gb_LF_joint_group
        gb_LF_joint_group = moveit_commander.MoveGroupCommander(MAIN_HAND_GROUP_LITTLE_FINGER)

    # ########################################################################################################
    # INIT GAZEBO SERVICES
    # http://gazebosim.org/tutorials?tut=ros_comm#Services:Stateandpropertygetters
    def gz_initialize_gazebo_services(self):
        # [Service]: Reset World --
        rospy.wait_for_service("/gazebo/reset_world", 10.0)
        global gb_gz_srv_reset_world
        gb_gz_srv_reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

        # [Service]: Get Model State --
        rospy.wait_for_service("/gazebo/get_model_state", 10.0)
        global gb_gz_srv_get_model_state
        gb_gz_srv_get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        # [Service]: Spawn SDF model
        rospy.wait_for_service("/gazebo/spawn_sdf_model", 10.0)
        global gb_gz_srv_spawn_sdf_model
        gb_gz_srv_spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        # [Service]: Delete Model
        rospy.wait_for_service("/gazebo/delete_model")
        global gb_gz_srv_delete_model
        gb_gz_srv_delete_model    = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        # [Service]: Pause Simulation
        # Pause simulation to make observation
        rospy.wait_for_service("/gazebo/pause_physics")
        global gb_gz_srv_pause_physics
        gb_gz_srv_pause_physics   = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        try:
            gb_gz_srv_pause_physics()
        except rospy.ServiceException as e:
            print ("/gazebo/pause_physics service call failed")

        # [Service]: UnPause Simulation
        rospy.wait_for_service("/gazebo/unpause_physics")
        global gb_gz_srv_unpause_physics
        gb_gz_srv_unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        try:
            gb_gz_srv_unpause_physics()
        except rospy.ServiceException as e:
            print ("/gazebo/unpause_physics service call failed")

        # [Service]: Switch Controller
        rospy.wait_for_service("/controller_manager/switch_controller")
        global gb_gz_srv_switch_ctrl
        gb_gz_srv_switch_ctrl     = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

        # [Service]: Model Configuration
        rospy.wait_for_service("/gazebo/set_model_configuration")
        global gb_gz_srv_set_model
        gb_gz_srv_set_model       = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

        # [Service]: Get Planning Scene
        rospy.wait_for_service('/get_planning_scene', 10.0)
        global gb_gz_srv_get_planning_scene
        gb_gz_srv_get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

        # [Service]: Planning Scene
        global gb_gz_pub_planning_scene
        gb_gz_pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)

       # [Service]: Reset Simulation
        global gb_reset_simulation
        gb_reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    def getCurrentJointPoses(self):
        #for joint in self._hand_joints
        #    observation.extend(np.array(joint.pos, dtype=np.float32))
        all_joints_state = self._hand_commander.get_joints_position()
        scale = 1
        #if angle_type == "degrees":
        #    scale = 1 * (180/pi)

        hand_joints_poses = {
            k: (v * scale) for k, v in all_joints_state.items() if k.startswith(self._hand_prefix[0] + "_")}
        # As the dict { joint_name: joint_pos}

        #print("Hand joints position \n " + str(hand_joints_poses) + "\n")
        return hand_joints_poses

    def getCurrentJointVelocities(self):
        all_joints_vel = self._hand_commander.get_joints_velocity()
        print('JOINTS VEL', al_joints_vel)
        return all_joints_vel

    def getDOFRange(self, joint_index):
        # TBD...
        return -3.14, 3.14

    def getCurrentDofs(self):
        hand_joints_poses = self.getCurrentJointPoses()
        dofs = []
        for joint_name, joint_pos in hand_joints_poses.items():
            if(not joint_name in ['rh_WRJ1', 'rh_WRJ2', 'rh_LFJ1', 'rh_THJ1']):
                dofs.append(joint_pos)

        return dofs

    def getJointsCount(self):
        return len(gb_hand_eigengrasp_ori)
        #return len(self._hand_joints)

    # http://www.cs.columbia.edu/~cmatei/graspit/html-manual/graspit-manual_11.html
    # The EigenGraps are discovered through human hand survey by Santello et al.
    # (see M. Santello, M. Flanders, and J. F. Soechting, Postural hand synergies for tool use,
    # Journal of Neuroscience, vol. 18, no. 23, 1998)
    def getEigenGraspsCount(self):
        return len(gb_hand_eigengrasps)

    def getCurrentEigenGraspAmps(self):
        dofs = self.getCurrentDofs()
        amps = gb_hand_eigengrasp_interface.toEigenGrasp(dofs)
        return amps

    def getActionDimension(self):
        return self.getEigenGraspsCount()

    def getObservation(self):
        names = ['amps0',
                 'amps1',
                 'amps2',
                 'amps3',
                 'amps4',
                 'amps5',
                 'plate_posX',
                 'plate_posY',
                 'plate_posZ',
                 'plate_ornX',
                 'plate_ornY',
                 'plate_ornZ',
                 'plate_ornW']
        Observation = col.namedtuple('Observation', names)

        # Hand EigenGrasps Info
        # !NOTE: Since we don't user velocity control in joint command, we don't include
        # the joint velocity into the observation!
        amps = self.getCurrentEigenGraspAmps()

        # Plate Position & Orientation
        plate_pose = gb_gz_get_plate_pose()
        #######################################################################

        return Observation(amps0 = np.array(amps[0], dtype=np.float32),
                           amps1 = np.array(amps[1], dtype=np.float32),
                           amps2 = np.array(amps[2], dtype=np.float32),
                           amps3 = np.array(amps[3], dtype=np.float32),
                           amps4 = np.array(amps[4], dtype=np.float32),
                           amps5 = np.array(amps[5], dtype=np.float32),
                           plate_posX = np.array(plate_pose.position.x, dtype=np.float32),
                           plate_posY = np.array(plate_pose.position.y, dtype=np.float32),
                           plate_posZ = np.array(plate_pose.position.z, dtype=np.float32),
                           plate_ornX = np.array(plate_pose.orientation.x, dtype=np.float32),
                           plate_ornY = np.array(plate_pose.orientation.y, dtype=np.float32),
                           plate_ornZ = np.array(plate_pose.orientation.z, dtype=np.float32),
                           plate_ornW = np.array(plate_pose.orientation.w, dtype=np.float32))

    def applyAction(self, action):
        #print('ACTION', action)
        hand_eigengrasp_amps = action
        hand_joint_poses = gb_hand_eigengrasp_interface.toDOF(hand_eigengrasp_amps)
        self.applyActionFromJointPoses(hand_joint_poses)

    def applyActionFromJointPoses(self, joint_poses):
        #print('Apply Joint Poses', joint_poses)
        hand_joint_poses = []
        i,j = 0,0
        while i < len(gb_hand_joint_goals_1.items()):
            if(i == 12 or i == 17 or i == 22 or i == 23):
                hand_joint_poses.append(0.0)
            else:
                hand_joint_poses.append(joint_poses[j])
                j = j+1
            i=i+1

        # MAYBE VALIDATION OF action HERE:
        # TBD...
        #
        for l in range(len(gb_hand_joint_goals_1.items())):
            gb_hand_joint_goals_1[gb_hand_joint_names[l]] = hand_joint_poses[l]

        new_dict = dict(gb_hand_joint_goals_1)
        del new_dict['rh_WRJ1']
        del new_dict['rh_WRJ2']
        rospy.loginfo("Moving fingers to joint states\n" + str(new_dict) + "\n")
        self._hand_commander.move_to_joint_value_target_unsafe(new_dict, GB_CHAND_MOVE_TIME, True)
        hand_tactile_state = self._hand_commander.get_tactile_state() # as a msg from a callback
        rospy.loginfo('HAND TACTILE STATE:' + str(hand_tactile_state))
        return

    def getReward(self):
        reward = 1000

        plate_pose     = gb_gz_get_plate_pose()

        # Distance from the palm center
        #hand_palm_pose = self._hand_commander.get_current_pose(WORLD_FRAME)
        #xyDistance = math.sqrt(math.pow(plate_pose.position.x-hand_palm_pose.position.x, 2) + \
        #                       math.pow(plate_pose.position.y-hand_palm_pose.position.y, 2))
        #print('XY DISTANCE', xyDistance)
        #if(xyDistance < 0.01):
        #    reward -= 1

        # The angle from the horizontal plane
        plate_orn = (plate_pose.orientation.x,
                     plate_pose.orientation.y,
                     plate_pose.orientation.z,
                     plate_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(plate_orn)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        #print ('EULER: ', euler)

        #q = np.array([.5, .5, .5, .5])
        #euler = lambda q: numpy.array([q.x, q.y, q.z, q.w])

        reward = reward - (abs(roll) + abs(pitch))
        return reward

    # Termination judgement
    def isPlateOnGround(self):
        plate_pose = gb_gz_get_plate_pose()
        #print('PLATE POSE', plate_pose.position.z)
        return plate_pose.position.z < 0.5

    def moveArmInitialPos(self):
        gb_arm_joint_goals['ra_shoulder_pan_joint']  = 1.0
        gb_arm_joint_goals['ra_shoulder_lift_joint'] = 0.1
        gb_arm_joint_goals['ra_elbow_joint']         = 0.0

        gb_arm_joint_goals['ra_wrist_1_joint']       = 0.0
        gb_arm_joint_goals['ra_wrist_2_joint']       = 0.0
        gb_arm_joint_goals['ra_wrist_3_joint']       = pi

        gb_arm_joint_goals['rh_WRJ2']                = 0.0
        gb_arm_joint_goals['rh_WRJ1']                = 0.5

        self._arm_commander.move_to_joint_value_target_unsafe(gb_arm_joint_goals, 3.0, True)
        return

    def moveArm(self, arm_joint_goals):
        gb_arm_joint_goals['ra_wrist_2_joint']      = arm_joint_goals[0]
        gb_arm_joint_goals['ra_wrist_1_joint']      = arm_joint_goals[1]
        gb_arm_joint_goals['ra_shoulder_pan_joint'] = arm_joint_goals[2]
        gb_arm_joint_goals['ra_wrist_3_joint']      = arm_joint_goals[3]
        gb_arm_joint_goals['rh_WRJ2']               = arm_joint_goals[4]
        gb_arm_joint_goals['rh_WRJ1']               = arm_joint_goals[5]
        self._arm_commander.move_to_joint_value_target_unsafe(gb_arm_joint_goals, 3.0, True)
        return

    def joint_states_callback(self, joint_states):
        """
        The callback function for the topic joint_states.
        It will store the received joint velocity and effort information in two dictionaries
        Velocity will be converted to degrees/s.
        Effort units are kept as they are (currently ADC units, as no calibration is performed on the strain gauges)

        @param joint_state: the message containing the joints data.
        """
        print('ON JOINT STATES ##################################################\n')

        with self._joint_states_lock:
            #print(joint_states)
            #print(statemulti_dof_joint_state)

            self._joint_positions  = {n: p
                                      for n, p in zip(joint_states.name, joint_states.position)}
            self._joint_velocities = {n: math.degrees(v)
                                      for n, v in zip(joint_states.name, joint_states.velocity)}
            self._joint_efforts    = {n: e
                                      for n, e in zip(joint_states.name, joint_states.effort)}

            #for n,v in *self._joint_velocities :
            #    rospy.loginfo(n + ':' + v + '\n')

            for finger in ['FF', 'MF', 'RF', 'LF']:
                for dic in [self._joint_velocities, self._joint_efforts]:
                    if (finger + 'J1') in dic and (finger + 'J2') in dic:
                        dic[finger + 'J0'] = dic[
                            finger + 'J1'] + dic[finger + 'J2']
                        del dic[finger + 'J1']
                        del dic[finger + 'J2']

    # ########################################################################################################
    #
    # Read the optoforce tactiles from the hand
    #
    def hand_optoforce_callback(self, data):
        rospy.loginfo("At:" + str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs) +
                      " sensor:" + str(data.header.frame_id) +
                      " Force:" + str(data.wrench.force.x) + "," +
                      str(data.wrench.force.y) + "," + str(data.wrench.force.z))

    def hand_optoforce_listener(self):
        num_sensors = 5
        for sensor_num in range(num_sensors):
            rospy.Subscriber("/optoforce_" + str(sensor_num), WrenchStamped, self.hand_optoforce_callback)

    # ########################################################################################################
    # DO ARM TEACHING
    #
    def setArmTeachMode(self):
        # sleep for some time (default 20s) during which the arm can be moved around by pushing it
        # but be careful to get away before the time runs out. You are warned
        rospy.loginfo("Set arm teach mode ON")
        self._arm_commander.set_teach_mode(True)
        rospy.sleep(1.0)

        rospy.loginfo("Set arm teach mode OFF")
        self._arm_commander.set_teach_mode(False)

        # Move through each goal
        # joint states are sent to the commanders, with a time for execution and a flag as to whether
        # or not the commander should wait for the command to complete before moving to the next command.

    # ########################################################################################################
    # DO HAND TEACHING
    #
    def setHandTeachMode(self):
        self._hand_commander.set_teach_mode(True)
        # sleep for some time (default 20s) during which the hand joints can be moved manually
        rospy.sleep(1.0)
        rospy.loginfo("Set hand teach mode OFF")
        self._hand_commander.set_teach_mode(False)
        # ...

    def do_move_arm_hand(self):
        # Move hand and arm
        joint_goals = gb_hand_joint_goals_1
        rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
        self._hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
        joint_goals = gb_arm_joint_goals_1
        rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
        self._arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

        # Move hand and arm
        joint_goals = gb_hand_joint_goals_2
        rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
        self._hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
        joint_goals = gb_arm_joint_goals_2
        rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
        self._arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

        # Move arm
        joint_goals = gb_arm_joint_goals_3
        rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
        self._arm_commander.move_to_joint_value_target_unsafe(joint_goals, 1.0, True)

        # Move hand
        joint_goals = gb_hand_joint_goals_3
        rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
        self._hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

        # Move arm
        joint_goals = gb_arm_joint_goals_6
        rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
        self._arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

        # Move hand
        joint_goals = gb_hand_joint_goals_4
        rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
        self._hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

        # Move arm
        joint_goals = gb_arm_joint_goals_5
        rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
        self._arm_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

        # Move arm and hand
        joint_goals = gb_arm_joint_goals_7
        rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
        self._arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
        joint_goals = gb_hand_joint_goals_5
        rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
        self._hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)


    # ########################################################################################################
    # DO SINUISOID FINGERS
    #
    def do_sinuisoid_fingers(self):
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
        hand_joint_trajectory = JointTrajectory()
        hand_joint_trajectory.header.stamp = rospy.Time.now()
        hand_joint_trajectory.joint_names = list(hand_joints_goal.keys())
        hand_joint_trajectory.points = []

        # generate sinusoidal list of data points, two joints moving out of phase
        for t in arange(0.002, ts, 0.02):
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.time_from_start = rospy.Duration.from_sec(float(t))
            trajectory_point.positions = []
            trajectory_point.velocities = []
            trajectory_point.accelerations = []
            trajectory_point.effort = []

            for key in hand_joint_trajectory.joint_names:
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

            hand_joint_trajectory.points.append(trajectory_point)

        # Send trajectory to self._hand_commander
        self._hand_commander.run_joint_trajectory_unsafe(hand_joint_trajectory)

        rospy.sleep(rospy.Duration(15))

    # ########################################################################################################
    # DO FINGERS TRAJECTORY
    #
    def construct_trajectory_point(self, joint_trajectory, posture, duration):
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = rospy.Duration.from_sec(float(duration))
        for key in joint_trajectory.joint_names:
            trajectory_point.positions.append(posture[key])
        return trajectory_point

    def do_fingers_trajectory(self):
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

        listener = PartialTrajListener()

        # Adjust poses according to the hand loaded
        open_hand_current = dict([(i, open_hand_pose[i]) for i in self._hand_joints if i in open_hand_pose])
        grasp1_current = dict([(i, grasp1[i]) for i in self._hand_joints if i in grasp1])
        grasp2_current = dict([(i, grasp2[i]) for i in self._hand_joints if i in grasp2])
        grasp3_current = dict([(i, grasp3[i]) for i in self._hand_joints if i in grasp3])
        grasp4_current = dict([(i, grasp4[i]) for i in self._hand_joints if i in grasp4])
        grasp5_current = dict([(i, grasp5[i]) for i in self._hand_joints if i in grasp5])

        start_time = rospy.Time.now()

        # Opening hand
        rospy.loginfo("Moving hand to open position")
        trajectory_start_time = 1.0
        hand_joint_trajectory = JointTrajectory()
        hand_joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
        hand_joint_trajectory.joint_names = list(open_hand_current.keys())
        hand_joint_trajectory.points = []
        trajectory_point = self.construct_trajectory_point(hand_joint_trajectory, open_hand_current, 1.0)
        hand_joint_trajectory.points.append(trajectory_point)
        self._hand_commander.run_joint_trajectory_unsafe(hand_joint_trajectory, True)

        # Closing index and middle fingers. Trajectories are generated from grasp1 - grasp5 and run with self._hand_commander
        rospy.loginfo("Closing index and ring fingers")
        trajectory_start_time = 4.0
        hand_joint_trajectory = JointTrajectory()
        hand_joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
        hand_joint_trajectory.joint_names = list(grasp1_current.keys())
        hand_joint_trajectory.points = []
        trajectory_point = construct_trajectory_point(hand_joint_trajectory, grasp1_current, 1.0)
        hand_joint_trajectory.points.append(trajectory_point)
        trajectory_point = construct_trajectory_point(hand_joint_trajectory, grasp2_current, 4.0)
        hand_joint_trajectory.points.append(trajectory_point)
        trajectory_point = construct_trajectory_point(hand_joint_trajectory, grasp3_current, 6.0)
        hand_joint_trajectory.points.append(trajectory_point)
        trajectory_point = construct_trajectory_point(hand_joint_trajectory, grasp4_current, 8.0)
        hand_joint_trajectory.points.append(trajectory_point)
        trajectory_point = construct_trajectory_point(hand_joint_trajectory, grasp5_current, 10.0)
        hand_joint_trajectory.points.append(trajectory_point)
        self._hand_commander.run_joint_trajectory_unsafe(hand_joint_trajectory, False)

        # Interrupting trajectory of index using two partial trajectories
        rospy.loginfo("Moving index finger to partial trajectory goals")
        rospy.sleep(2)
        trajectory_start_time = 8.0
        hand_joint_trajectory = JointTrajectory()
        hand_joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
        hand_joint_trajectory.joint_names = list(grasp_partial_1.keys())
        hand_joint_trajectory.points = []
        trajectory_point = construct_trajectory_point(hand_joint_trajectory, grasp_partial_1, 1.0)
        hand_joint_trajectory.points.append(trajectory_point)
        trajectory_point = construct_trajectory_point(hand_joint_trajectory, grasp_partial_2, 3.0)
        hand_joint_trajectory.points.append(trajectory_point)
        self._hand_commander.run_joint_trajectory_unsafe(hand_joint_trajectory, False)

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



    # ########################################################################################################
    # DO GRASP POSE
    #
    def graspPose(self):
        position = [1.07, 0.26, 0.88, -0.34,
                    0.85, 0.60, 0.21, -0.23,
                    0.15, 1.06, 0.16, 1.04,
                    0.05, 1.04, 0.34, 0.68, -0.24,
                    0.35, 0.69, 0.18, 1.20, -0.11,
                    0.35, 0.69]

        grasp_pose = dict(zip(gb_hand_joint_names, position))

        # Adjust poses according to the hand loaded
        open_hand_current  = dict([(i, open_hand_pose[i]) for i in self._hand_joints if i in open_hand_pose])
        grasp_pose_current = dict([(i, grasp_pose[i]) for i in self._hand_joints if i in grasp_pose])

        # Partial list of goals
        grasp_partial_1 = {'rh_FFJ3': 0.50}

        start_time = rospy.Time.now()

        # Move hand using move_to_joint_value_target_unsafe to 1st position
        self._hand_commander.move_to_joint_value_target_unsafe(open_hand_current, 1.0, True)

        rospy.sleep(2)

        # Move hand using run_joint_trajectory_unsafe to joint angles specified in 'position' list
        self._hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

        trajectory_start_time = 2.0
        hand_joint_trajectory = JointTrajectory()

        # Construct and send partial trajectory for joint listed in grasp_partial_1
        hand_joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
        hand_joint_trajectory.hand_joint_names = list(grasp_partial_1.keys())
        hand_joint_trajectory.points = []
        trajectory_point = construct_trajectory_point(hand_joint_trajectory, grasp_partial_1, 1.0)
        hand_joint_trajectory.points.append(trajectory_point)

        self._hand_commander.run_joint_trajectory_unsafe(hand_joint_trajectory, True)
        rospy.sleep(2)

    # ########################################################################################################
    # MOVEIT
    # https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/scripts/move_group_python_interface_tutorial.py
    #
    def move_group(self):
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
        #self.moveArm(group_variable_values)
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
        self.moveArm(group_variable_values)
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
        self.moveArm(group_variable_values)
        # ducta --

        # # Adding/Removing Objects and Attaching/Detaching Objects
        # # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # # First, we will define the collision object message
        collision_object = moveit_msgs.msg.CollisionObject()

        # # END_TUTORIAL
        print "============ STOPPING"


    # ########################################################################################################
    # DO FINGER LEARNING
    #

    # 1 - Keep moving fingers
    #
    def deep_keep_moving_fingers(self):
        for i in range(0,20):
            gb_hand_joint_goals_1[gb_hand_joint_names_dict[i]] = gb_deep_joint_poses[i]
        while not rospy.is_shutdown():
            for i in range(0,20):
                if ((i%4 != 3) and (i!=20)):
                    if (gb_deep_joint_poses[i] >= 1.5):
                        gb_deep_joint_poses[i] = 0.0
                    else: gb_deep_joint_poses[i] += 0.5
                    joint_name = gb_hand_joint_names_dict[i]
                    #rospy.loginfo('MOVE to joint name ' + joint_name + "\n")
                    gb_hand_joint_goals_1[joint_name] = gb_deep_joint_poses[i]
                    rospy.loginfo("Moving fingers to joint states\n" + str(gb_hand_joint_goals_1) + "\n")
                    self._hand_commander.move_to_joint_value_target_unsafe(gb_hand_joint_goals_1, 0.5, True)
                    hand_tactile_state = self._hand_commander.get_tactile_state() # as a msg from a callback
                    rospy.loginfo('HAND TACTILE STATE:' + str(hand_tactile_state))

    #deep_keep_moving_fingers()

    #2 - Spawning Plate Model Loop
    #gb_gz_spawn_plate_model_loop(1.5)

    # #######################################
    #mk_grasp(gb_hand_joint_goals_1)
