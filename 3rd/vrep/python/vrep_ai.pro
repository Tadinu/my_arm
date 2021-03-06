QT += core
QT -= gui

CONFIG += c++11

TARGET = vrep_ai
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES +=

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

DISTFILES += \
    my_arm.py \
    vrep.py \
    vrepConst.py \
    complexCommandTest.py \
    pathPlanningTest.py \
    simpleSynchronousTest.py \
    simpleTest.py \
    RobotOperationEnv.py \
    menace_object.py \
    RobotOperationEnv.py \
    robot.py \
    robotBot.py \
    robotCommon.py \
    train_robot_operation.py \
    ddpg/ActorNetwork.py \
    ddpg/CriticNetwork.py \
    ddpg/OU.py \
    ddpg/replay_buffer.py \
    ddpg/ReplayBuffer.py \
    train_robot_operation_II.py \
    actormodel.json \
    criticmodel.json \
    actormodel.h5 \
    criticmodel.h5 \
    eigen_grasp.py \
    UR5/ur5.py \
    UR5/ur5_ctrl.py \
    repeated_timer.py \
    her/actor_critic.py \
    her/ddpg.py \
    her/her.py \
    her/normalizer.py \
    her/replay_buffer.py \
    her/rollout.py \
    her/util.py \
    her/experiment/config.py \
    her/experiment/play.py \
    her/experiment/plot.py \
    her/experiment/train.py \
    train_robot_operation_ddpg.py \
    train_robot_operation_ddpg_II.py \
    train_robot_operation_ddpg_her.py \
    RobotOperationGoalEnv.py \
    train_robot_operation_deepq.py \
    dqn/dqn.py \
    train_robot_operation_ppo.py \
    ppo/ppo2/policies.py \
    ppo/ppo2/ppo2.py
