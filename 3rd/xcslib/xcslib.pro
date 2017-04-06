QT += core
QT -= gui

CONFIG += c++11

TARGET = xcslib
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = lib
##CONFIG += staticlib
##DESTDIR = lib

DEFINES += XCS_LIB
INCLUDEPATH+= /usr/include \
              /usr/local/include \
              ./pf

SOURCES += \
    binary_action.cpp \
    binary_inputs.cpp \
    boolean_action.cpp \
    experiment_mgr.cpp \
    fsm_env.cpp \
    generic.cpp \
    grid_env.cpp \
    integer_action.cpp \
    multiplexer_env.cpp \
    parity_env.cpp \
    real_interval_condition.cpp \
    woods_env.cpp \
    woods2_env.cpp \
    xcs_classifier_system.cpp \
    xcs_classifier.cpp \
    xcs_config_mgr2.cpp \
    xcs_main.cpp \
    xcs_random.cpp \
    xcs_utility.cpp \
    pf/base.cpp \
    pf/nlms.cpp \
    pf/utility.cpp \
    utility/mm2.cpp \
    utility/moving_average.cpp

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
    manual.pdf \
    LICENSE \
    makefile \
    makefile.others \
    README \
    utility/average.awk \
    utility/prepare_compact.sh \
    utility/prepare_pop.sh \
    utility/prepare_results.sh \
    utility/prepare_rwd.sh \
    utility/prepare_se.sh \
    utility/prepare_steps.sh \
    utility/prepare_trace.sh \
    utility/xcs_ma.sh \
    utility/xcs_pop.sh \
    utility/xcs_rwd.sh \
    utility/xcs_se.sh \
    utility/xcs_steps.sh \
    utility/makefile \
    make/xcs-docs.ini \
    make/xcs.make \
    make/xcsf.make

HEADERS += \
    action_base.hpp \
    binary_action.hpp \
    binary_inputs.hpp \
    boolean_action.hpp \
    condition_base.hpp \
    environment_base.hpp \
    experiment_mgr.hpp \
    fsm_env.hpp \
    generic.hpp \
    grid_env.hpp \
    inputs_base.hpp \
    integer_action.hpp \
    interval.hpp \
    multiplexer_env.hpp \
    parity_env.hpp \
    prediction_functions.hpp \
    rl_definitions.hpp \
    ternary_condition.hpp \
    woods_env.hpp \
    woods2_env.hpp \
    xcs_classifier_system.hpp \
    xcs_classifier.hpp \
    xcs_config_mgr2.hpp \
    xcs_definitions.hpp \
    xcs_random.hpp \
    xcs_utility.hpp \
    pf/base.hpp \
    pf/nlms.hpp \
    pf/utility.hpp
