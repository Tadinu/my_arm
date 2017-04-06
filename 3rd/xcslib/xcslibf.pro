QT += core
QT -= gui

CONFIG += c++11

TARGET = xcsflib
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = lib
##CONFIG += staticlib
##DESTDIR = lib

DEFINES += XCSF_LIB
INCLUDEPATH+= /usr/include \
              /usr/local/include \
              ./pf

SOURCES += \
    dummy_action.cpp \
    experiment_mgr.cpp \
    fsm_env.cpp \
    generic.cpp \
    grid_env.cpp \
    parity_env.cpp \
    real_functions_env.cpp \
    real_inputs.cpp \
    real_interval_condition.cpp \
    ternary_condition.cpp \
    woods_env.cpp \
    woods2_env.cpp \
    xcs_config_mgr2.cpp \
    xcs_random.cpp \
    xcs_utility.cpp \
    xcsf_classifier_system.cpp \
    xcsf_classifier.cpp \
    xcsf_main.cpp \
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
    condition_base.hpp \
    dummy_action.hpp \
    experiment_mgr.hpp \
    fsm_env.hpp \
    generic.hpp \
    grid_env.hpp \
    inputs_base.hpp \
    interval.hpp \
    parity_env.hpp \
    prediction_functions.hpp \
    real_functions_env.hpp \
    real_inputs.hpp \
    real_interval_condition.hpp \
    rl_definitions.hpp \
    ternary_condition.hpp \
    woods_env.hpp \
    woods2_env.hpp \
    xcs_config_mgr2.hpp \
    xcs_definitions.hpp \
    xcs_random.hpp \
    xcs_utility.hpp \
    xcsf_classifier_system.hpp \
    xcsf_classifier.hpp \
    pf/base.hpp \
    pf/nlms.hpp \
    pf/utility.hpp
