QT += core
QT -= gui

CONFIG += c++11

TARGET = mujoco_ai
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

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
    python/gym-baxter/gym_baxter/envs/baxter.py \
    python/gym-baxter/gym_baxter/envs/baxter_env.py \
    python/gym-baxter/setup.py \
    python/gym-baxter/README.md \
    python/gym-baxter/gym_baxter/__init__.py \
    python/gym-baxter/gym_baxter/envs/baxter.py \
    python/gym-baxter/gym_baxter/envs/baxter_env.py \
    python/gym-baxter/gym_baxter/envs/__init__.py \
    python/her/README.md \
    python/her/actor_critic.py \
    python/her/ddpg.py \
    python/her/her.py \
    python/her/__init__.py \
    python/her/normalizer.py \
    python/her/replay_buffer.py \
    python/her/rollout.py \
    python/her/util.py \
    python/her/experiment/config.py \
    python/her/experiment/__init__.py \
    python/her/experiment/play.py \
    python/her/experiment/plot.py \
    python/her/experiment/train.py \
    python/gym-baxter/gym_baxter/envs/assets/baxter.xml \
    python/gym-baxter/gym_baxter/envs/assets/baxter_without_table.xml \
    python/gym-baxter/gym_baxter/envs/assets/spring.xml \
    python/gym-baxter/gym_baxter/envs/transformation.py \
    python/gym-baxter/gym_baxter/envs/visualize_mjc.py

