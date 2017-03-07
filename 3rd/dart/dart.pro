QT += core
QT -= gui

CONFIG += c++11

TARGET = dart
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH+= /usr/include \
              /usr/local/include \
              /opt/ros/kinetic/include \
              . \
              ./build \
              ./dart

CONFIG(release, debug|release) {
    message(Release)
    LIBS += -L./build/lib/ \
            -ldart \
            -ldart-gui \
            -ldart-collision \
            -ldart-collision-bullet \
            -ldart-utils \
            -ldart-utils-urdf
}

CONFIG(debug, debug|release) {
    message(Debug)
    LIBS += -L./build/lib/ \
            -ldart \
            -ldart-gui \
            -ldart-collision \
            -ldart-collision-bullet \
            -ldart-utils \
            -ldart-utils-urdf
}

SOURCES += main.cpp \
    examples/softBodies/Main.cpp \
    examples/softBodies/MyWindow.cpp \
    examples/hardcodedDesign/Main.cpp \
    examples/hardcodedDesign/MyWindow.cpp \
    examples/jointConstraints/Controller.cpp \
    examples/jointConstraints/Main.cpp \
    examples/jointConstraints/MyWindow.cpp \
    tutorials/tutorialBiped-Finished.cpp \
    tutorials/tutorialBiped.cpp \
    tutorials/tutorialCollisions-Finished.cpp \
    tutorials/tutorialCollisions.cpp \
    tutorials/tutorialDominoes-Finished.cpp \
    tutorials/tutorialDominoes.cpp \
    tutorials/tutorialMultiPendulum-Finished.cpp \
    tutorials/tutorialMultiPendulum.cpp \
    examples/rigidShapes/Main.cpp \
    examples/rigidShapes/MyWindow.cpp \
    examples/simpleFrames/Main.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    examples/softBodies/MyWindow.hpp \
    examples/hardcodedDesign/MyWindow.hpp \
    examples/jointConstraints/Controller.hpp \
    examples/jointConstraints/MyWindow.hpp \
    examples/rigidShapes/MyWindow.hpp

DISTFILES += \
    examples/softBodies/CMakeLists.txt \
    examples/CMakeLists.txt \
    data/skel/biped.skel \
    data/skel/bullet_collision.skel \
    data/skel/chain.skel \
    data/skel/cube.skel \
    data/skel/cubes.skel \
    data/skel/empty.skel \
    data/skel/freeChain.skel \
    data/skel/fullbody1.skel \
    data/skel/ground.skel \
    data/skel/joint_limit.skel \
    data/skel/mesh_collision.skel \
    data/skel/shapes.skel \
    data/skel/skateboard.skel \
    data/skel/soft_cubes.skel \
    data/skel/soft_open_chain.skel \
    data/skel/softBodies.skel \
    data/skel/sphere.skel \
    data/skel/spheres.skel \
    data/skel/two_cubes.skel \
    data/skel/vehicle.skel \
    examples/hardcodedDesign/README.txt \
    examples/hardcodedDesign/CMakeLists.txt \
    examples/jointConstraints/CMakeLists.txt \
    tutorials/CMakeLists.txt \
    examples/rigidShapes/CMakeLists.txt \
    examples/simpleFrames/CMakeLists.txt \
    CMakeLists.txt
