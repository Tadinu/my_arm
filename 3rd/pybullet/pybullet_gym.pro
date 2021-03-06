QT += core
QT -= gui

CONFIG += c++11

TARGET = pybullet_gym
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES +=

INCLUDEPATH += /usr/include/python2.7/ \
               #/usr/include/python3.5m/

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
    envs/kuka.py \
    kukaJointSpaceGymEnvTest.py \
    enjoy_kuka_avoid_fallings_objs.py \
    PG_Pong.py \
    kukaBot.py \
    menace_object.py \
    initialize_qvalues.py \
    envs/kukaFallingObjsGymEnv.py \
    train_kuka_avoid_fall_objs.py \
    data/sphere_5cm.urdf \
    data/table/table.urdf \
    data/plane.urdf \
    data/kuka_iiwa/kuka_with_gripper2.sdf \
    envs/racecar.py \
    envs/racecarGymEnv.py \
    envs/racecarZEDGymEnv.py \
    train_kuka_catch_fall_objs.py \
    ddpg/ActorNetwork.py \
    ddpg/CriticNetwork.py \
    ddpg/OU.py \
    ddpg/replay_buffer.py \
    ddpg/ReplayBuffer.py \
    envs/kukaCatchObjsGymEnv.py

