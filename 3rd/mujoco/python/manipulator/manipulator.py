#!/usr/bin/env python3
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print('CURRENT DIR:', currentdir)
parentdir = os.path.dirname(currentdir)
print('PARENT DIR:', parentdir)
os.sys.path.insert(0, parentdir)

import math
import os
import copy
import numpy as np

from robot import Robot # robot.py in parentdir
import rotations, utils # in currentdir

try:
    import mujoco_py
except ImportError as e:
    raise error.DependencyNotInstalled("{}. (HINT: you need to install mujoco_py, and also perform the setup instructions here: https://github.com/openai/mujoco-py/.)".format(e))

CMODEL_PATH   = "/home/brhm/DUC/RobotArm/src/my_arm/3rd/mujoco/python/manipulator/assets/manipulator.xml"
CINITIAL_QPOS = {
                'robot0:slide0': 0.405,
                'robot0:slide1': 0.48,
                'robot0:slide2': 0.0,
                'robot0:torso_lift_joint'       : 0.0,
                'robot0:head_pan_joint'         : 0.0,
                'robot0:head_tilt_joint'        : 0.0,
                'robot0:shoulder_pan_joint'     : 0.0,
                'robot0:shoulder_lift_joint'    : 0.0,
                'robot0:upperarm_roll_joint'    : 0.0,
                'robot0:elbow_flex_joint'       : 0.0,
                'robot0:forearm_roll_joint'     : 0.0,
                'robot0:wrist_flex_joint'       : 0.0,
                'robot0:wrist_roll_joint'       : 0.0,
                'robot0:r_gripper_finger_joint' : 0.0,
                'robot0:l_gripper_finger_joint' : 0.0,

                'table0:slide0': 1.05,
                'table0:slide1': 0.4,
                'table0:slide2': 0.0,
                'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],
                }

CBASE_QPOS = {
             'robot0:shoulder_pan_joint'     : 0.0,
             'robot0:shoulder_lift_joint'    : 0.0,
             'robot0:upperarm_roll_joint'    : 0.0,
             'robot0:elbow_flex_joint'       : 0.0,
             'robot0:forearm_roll_joint'     : 0.0,
             'robot0:wrist_flex_joint'       : 0.0,
             'robot0:wrist_roll_joint'       : 0.0,
             'robot0:r_gripper_finger_joint' : 0.0,
             'robot0:l_gripper_finger_joint' : 0.0,
             }

CCATCH_QPOS = {
              'robot0:shoulder_pan_joint'     : 0.,
              'robot0:shoulder_lift_joint'    : 0.,
              'robot0:upperarm_roll_joint'    : 0.1,
              'robot0:elbow_flex_joint'       : 0.1,
              'robot0:forearm_roll_joint'     : 0.1,
              'robot0:wrist_flex_joint'       : 0.0,
              'robot0:wrist_roll_joint'       : 0.0,
              'robot0:r_gripper_finger_joint' : 0.0,
              'robot0:l_gripper_finger_joint' : 0.0,
              }

class Manipulator(Robot):
    def __init__(self, model_path, initial_qpos, n_substeps):
        super(Manipulator, self).__init__(
            model_path=model_path, initial_qpos=initial_qpos, n_substeps=n_substeps)

        self.gripper_extra_height=0.2

    # ==========================================================================
    # SIMULATOR METHODS ========================================================
    #
    def setup_viewer(self):
        body_id = self.sim.model.body_name2id('robot0:gripper_link')
        lookat = self.sim.data.body_xpos[body_id]
        for idx, value in enumerate(lookat):
            self.viewer.cam.lookat[idx] = value
        self.viewer.cam.distance = 2.5
        self.viewer.cam.azimuth = 132.
        self.viewer.cam.elevation = -14.

    # ==========================================================================
    # ROBOT METHODS ============================================================
    #
    def initialize(self):
        if self.sim.data.qpos is not None and self.sim.model.joint_names:
            self.joint_names = [n for n in self.sim.model.joint_names if n.startswith('robot')]

        for name, value in self.initial_qpos.items():
            self.sim.data.set_joint_qpos(name, value)
        utils.reset_mocap_welds(self.sim)
        self.sim.forward()

    def print_joint_info(self, joint_id):
        name = self.joint_names[joint_id]
        print(joint_id, name,
              self.sim.data.get_joint_qpos(name),
              self.sim.data.get_joint_qvel(name))

    def move_joint_by_id(self, joint_id, qpos, qvel):
        if self.sim.data.qpos is not None and self.joint_names:
            joint_name = self.joint_names[joint_id]
            self.sim.data.set_joint_qpos(joint_name, qpos)
            self.sim.data.set_joint_qvel(joint_name, qvel)
            #self.sim.step()

    def move_joint_by_name(self, joint_name, qpos, qvel):
        self.sim.data.set_joint_qpos(joint_name, qpos)
        self.sim.data.set_joint_qvel(joint_name, qvel)
        #self.sim.step()

    def move_end_effector(self, target):
        # Move end effector into position.
        #gripper_target   = np.array([-0.25, 0., -0.431 + self.gripper_extra_height]) + self.sim.data.get_site_xpos('robot0:grip')
        gripper_target   = target
        gripper_rotation = np.array([1., 0., 0.01, 0.])
        self.sim.data.set_mocap_pos('robot0:mocap', gripper_target)
        self.sim.data.set_mocap_quat('robot0:mocap', gripper_rotation)
        for _ in range(1):
            self.sim.step()

    def move_to_qpos(self, qpos, qvel):
        for name, qpos_val in qpos.items():
            self.move_joint_by_name(name, qpos_val, qvel)
        #self.sim.forward()

    def act(self, action):
        action = action.copy()  # ensure that we don't change the action outside of this scope
        pos_ctrl, gripper_ctrl = action[:3], action[3]

        #pos_ctrl *= 0.05  # limit maximum change in position
        end_tip_rot_ctrl = [1., 0., 0.01, 0.]  # fixed rotation of the end effector, expressed as a quaternion
        gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
        assert gripper_ctrl.shape == (2,)
        action = np.concatenate([pos_ctrl, end_tip_rot_ctrl, gripper_ctrl])

        # Apply action to simulation.
        utils.ctrl_set_action(self.sim, action)  # For gripper torque control
        utils.mocap_set_action(self.sim, action) # For mocap pos & orient

        self.count += 1
        self.sim.step()
        self.viewer.render()

if __name__ == "__main__":
    robot  = Manipulator(CMODEL_PATH, initial_qpos=CINITIAL_QPOS, n_substeps=20)
    target = np.array([-0.25, 0., -0.431 + robot.gripper_extra_height]) + robot.sim.data.get_site_xpos('robot0:grip')

    action = np.array([-0.05, 0., -0.1, 0.5])
    #print('LEN ', len(robot.sim.data.ctrl), len(robot.sim.model.actuator_biastype))

    qvel = 1
    qpos = 0.1

    while True:
        #robot.act(action)
        robot.move_end_effector(target)
        qpos += 0.1
        qvel -= 0.01
        robot.move_joint_by_id(10, qpos, qvel)
        robot.move_joint_by_name('robot0:elbow_flex_joint', qpos, qvel)

        #robot.move_to_qpos(CCATCH_QPOS, qvel)
        #robot.move_to_qpos(CBASE_QPOS, qvel)
        #robot.print_joint_info(9)

        robot.sim.step()
        robot.viewer.render()

        if robot.count > 100 and os.getenv('TESTING') is not None:
            break
