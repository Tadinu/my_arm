#!/usr/bin/env python

# This example demonstrates how to move the right hand and arm through a sequence of joint goals.
# At the start and end of the sequence, both the hand and arm will spend 20s in teach mode,
# This allows the user to manually move the hand and arm. New goals can be easily generated
# using the script 'sr_print_joints_position.py
# PLEASE NOTE: move_to_joint_value_target_unsafe is used in this script, so collision checks
# between the hand and arm are not made!

import sys
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

# #####################################################################################
# LEAP MOTION
#
import leap_interface
import Leap

FREQUENCY_ROSTOPIC_DEFAULT = 0.01
NODENAME = 'leap_skeleton_pub'
PARAMNAME_FREQ = 'freq'
PARAMNAME_FREQ_ENTIRE = '/' + NODENAME + '/' + PARAMNAME_FREQ

def catchHandData():
    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    # pub_ros   = rospy.Publisher('leapmotion/data',leapros)

    while not rospy.is_shutdown():
        timenow=rospy.Time.now()

        if li.listener.left_hand:
            analyzeHandData(li.listener.left_hand)
        elif li.listener.right_hand:
            analyzeHandData(li.listener.right_hand)

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

def analyzeHandData(hand):
    for finger in hand.fingers:
        finger_name=hand_name+"_"+finger_names[finger.type()]

        prev_bone_name=hand_name
        for num in range(0,4):
            bone=finger.bone(num)
            bone_name=finger_name+"_"+bones_names[num]

            prev_bone_name=bone_name
            prev_bone_absolute=bone_absolute

# #####################################################################################
# SHADOW HAND CONTROL
#
rospy.init_node("basic_hand_arm_example", anonymous=True)

# handfinder is used to access the hand parameters
hand_finder = HandFinder()
hand_parameters = hand_finder.get_hand_parameters()
prefix = hand_parameters.mapping.values()
rospy.loginfo('HAND TYPE:' + prefix[0])
hand_serial = hand_parameters.mapping.keys()[0]
rospy.loginfo('HAND SERIAL:' + hand_serial)

#hand_commander = SrHandCommander(name="right_hand", prefix="rh")
hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)
arm_commander = SrArmCommander(name="right_arm", set_ground=False)

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

deep_joint_poses = [0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.5707]
deep_hand_joint_names = {0: 'rh_FFJ1', 1 : 'rh_FFJ2', 2 : 'rh_FFJ3',  3: 'rh_FFJ4',
                         4: 'rh_MFJ1', 5 : 'rh_MFJ2', 6 : 'rh_MFJ3',  7: 'rh_MFJ4',
                         8: 'rh_RFJ1', 9 : 'rh_RFJ2', 10: 'rh_RFJ3', 11: 'rh_RFJ4',
                         12:'rh_LFJ1', 13: 'rh_LFJ2', 14: 'rh_LFJ3', 15: 'rh_LFJ4',
                         16:'rh_THJ1', 17: 'rh_THJ2', 18: 'rh_THJ3', 19: 'rh_THJ4', 20: 'rh_THJ5' }

# ####################################################################################
# DO ARM TEACHING
#
# sleep for some time (default 20s) during which the arm can be moved around by pushing it
# but be careful to get away before the time runs out. You are warned
rospy.loginfo("Set arm teach mode ON")
arm_commander.set_teach_mode(True)
rospy.sleep(1.0)

rospy.loginfo("Set arm teach mode OFF")
arm_commander.set_teach_mode(False)

# Move through each goal
# joint states are sent to the commanders, with a time for execution and a flag as to whether
# or not the commander should wait for the command to complete before moving to the next command.

# # Move hand and arm
# joint_goals = hand_joint_goals_1
# rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
# hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
# joint_goals = arm_joint_goals_1
# rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)
#
# # Move hand and arm
# joint_goals = hand_joint_goals_2
# rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
# hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
# joint_goals = arm_joint_goals_2
# rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)
#
# # Move arm
# joint_goals = arm_joint_goals_3
# rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joint_goals, 1.0, True)
#
# # Move hand
# joint_goals = hand_joint_goals_3
# rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
# hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)
#
# # Move arm
# joint_goals = arm_joint_goals_6
# rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)
#
# # Move hand
# joint_goals = hand_joint_goals_4
# rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
# hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)
#
# # Move arm
# joint_goals = arm_joint_goals_5
# rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)
#
# # Move arm and hand
# joint_goals = arm_joint_goals_7
# rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
# joint_goals = hand_joint_goals_5
# rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
# hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

# ####################################################################################
# DO HAND TEACHING
#
hand_commander.set_teach_mode(True)
# sleep for some time (default 20s) during which the hand joints can be moved manually
rospy.sleep(1.0)
rospy.loginfo("Set hand teach mode OFF")
hand_commander.set_teach_mode(False)

# ####################################################################################
# DO SINUISOID FINGERS
#
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from numpy import sin, cos, pi, arange

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

# Send trajectory to hand_commander
hand_commander.run_joint_trajectory_unsafe(joint_trajectory)

rospy.sleep(rospy.Duration(15))

# ####################################################################################
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


def construct_trajectory_point(posture, duration):
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

def do_fingers_trajectory():
    listener = PartialTrajListener()
    hand_finder = HandFinder()

    hand_parameters = hand_finder.get_hand_parameters()
    hand_serial = hand_parameters.mapping.keys()[0]

    hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                     hand_serial=hand_serial)

    hand_mapping = hand_parameters.mapping[hand_serial]

    # Hand joints are detected
    joints = hand_finder.get_hand_joints()[hand_mapping]

    # Adjust poses according to the hand loaded
    open_hand_current = dict([(i, open_hand[i]) for i in joints if i in open_hand])
    grasp1_current = dict([(i, grasp1[i]) for i in joints if i in grasp1])
    grasp2_current = dict([(i, grasp2[i]) for i in joints if i in grasp2])
    grasp3_current = dict([(i, grasp3[i]) for i in joints if i in grasp3])
    grasp4_current = dict([(i, grasp4[i]) for i in joints if i in grasp4])
    grasp5_current = dict([(i, grasp5[i]) for i in joints if i in grasp5])

    start_time = rospy.Time.now()

    # Opening hand
    rospy.loginfo("Moving hand to open position")
    trajectory_start_time = 1.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(open_hand_current.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(open_hand_current, 1.0)
    joint_trajectory.points.append(trajectory_point)
    hand_commander.run_joint_trajectory_unsafe(joint_trajectory, True)

    # Closing index and middle fingers. Trajectories are generated from grasp1 - grasp5 and run with hand_commander
    rospy.loginfo("Closing index and ring fingers")
    trajectory_start_time = 4.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(grasp1_current.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(grasp1_current, 1.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp2_current, 4.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp3_current, 6.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp4_current, 8.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp5_current, 10.0)
    joint_trajectory.points.append(trajectory_point)
    hand_commander.run_joint_trajectory_unsafe(joint_trajectory, False)

    # Interrupting trajectory of index using two partial trajectories
    rospy.loginfo("Moving index finger to partial trajectory goals")
    rospy.sleep(2)
    trajectory_start_time = 8.0
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(grasp_partial_1.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(grasp_partial_1, 1.0)
    joint_trajectory.points.append(trajectory_point)
    trajectory_point = construct_trajectory_point(grasp_partial_2, 3.0)
    joint_trajectory.points.append(trajectory_point)
    hand_commander.run_joint_trajectory_unsafe(joint_trajectory, False)

    graphs_finished = False

    rate = rospy.Rate(10)

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

do_fingers_trajectory()

# ####################################################################################
# DO GRASP POSE
#
hand_mapping = hand_parameters.mapping[hand_serial]

# Hand joints are detected
joints = hand_finder.get_hand_joints()[hand_mapping]

open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
             'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
             'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

keys = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_LFJ1', 'rh_LFJ2',
        'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3',
        'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_THJ1',
        'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5']

position = [1.07, 0.26, 0.88, -0.34, 0.85, 0.60,
            0.21, -0.23, 0.15, 1.06, 0.16, 1.04,
            0.05, 1.04, 0.34, 0.68, -0.24, 0.35,
            0.69, 0.18, 1.20, -0.11]

def do_grasp_pose():
    grasp_pose = dict(zip(keys, position))

    # Adjust poses according to the hand loaded
    open_hand_current = dict([(i, open_hand[i]) for i in joints if i in open_hand])
    grasp_pose_current = dict([(i, grasp_pose[i]) for i in joints if i in grasp_pose])

    # Partial list of goals
    grasp_partial_1 = {'rh_FFJ3': 0.50}

    start_time = rospy.Time.now()

    # Move hand using move_to_joint_value_target_unsafe to 1st position
    hand_commander.move_to_joint_value_target_unsafe(open_hand_current, 1.0, True)

    rospy.sleep(2)

    # Move hand using run_joint_trajectory_unsafe to joint angles specified in 'position' list
    hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

    trajectory_start_time = 2.0
    joint_trajectory = JointTrajectory()

    # Construct and send partial trajectory for joint listed in grasp_partial_1
    joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
    joint_trajectory.joint_names = list(grasp_partial_1.keys())
    joint_trajectory.points = []
    trajectory_point = construct_trajectory_point(grasp_partial_1, 1.0)
    joint_trajectory.points.append(trajectory_point)

    hand_commander.run_joint_trajectory_unsafe(joint_trajectory, True)
    rospy.sleep(2)

do_grasp_pose()

# ####################################################################################
# START TO DO FINGER LEARNING
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
                rospy.loginfo("Moving arm to joint states\n" + str(hand_joint_goals_1) + "\n")
                hand_commander.move_to_joint_value_target_unsafe(hand_joint_goals_1, 0.5, True)

deep_keep_moving_fingers()

from my_arm_utils import mk_grasp

mk_grasp(hand_joint_goals_1)
