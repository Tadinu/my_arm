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
