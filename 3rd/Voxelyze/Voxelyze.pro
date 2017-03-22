QT += core
QT -= gui

CONFIG += c++11

TEMPLATE = lib

TARGET = Voxelyze
##DESTDIR = lib

CONFIG += console
CONFIG -= app_bundle
##CONFIG += staticlib

DEFINES += MY_ARM USE_OPEN_GL GROUND_PLANE_XZ ## PARDISO_5 GROUND_PLANE_XY !NOTE: DART use XZ as plane
INCLUDEPATH+= /usr/include \
              /usr/local/include \
              ./include \
              ./include\rapidjson \
              ./include\rapidjson\error \
              ./include\rapidjson\internal \
              ./include\rapidjson\msinttypes

SOURCES += \
    src/Voxelyze.cpp \
    src/VX_Collision.cpp \
    src/VX_External.cpp \
    src/VX_LinearSolver.cpp \
    src/VX_Link.cpp \
    src/VX_Material.cpp \
    src/VX_MaterialLink.cpp \
    src/VX_MaterialVoxel.cpp \
    src/VX_MeshRender.cpp \
    src/VX_Voxel.cpp

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
    include/Array3D.h \
    include/Quat3D.h \
    include/Vec3D.h \
    include/Voxelyze.h \
    include/VX_Collision.h \
    include/VX_External.h \
    include/VX_LinearSolver.h \
    include/VX_Link.h \
    include/VX_Material.h \
    include/VX_MaterialLink.h \
    include/VX_MaterialVoxel.h \
    include/VX_MeshRender.h \
    include/VX_Utils.h \
    include/VX_Voxel.h \
    include/rapidjson/error/error.h \
    include/rapidjson/error/en.h \
    include/rapidjson/allocators.h \
    include/rapidjson/document.h \
    include/rapidjson/encodedstream.h \
    include/rapidjson/encodings.h \
    include/rapidjson/filereadstream.h \
    include/rapidjson/filestream.h \
    include/rapidjson/filewritestream.h \
    include/rapidjson/memorybuffer.h \
    include/rapidjson/memorystream.h \
    include/rapidjson/prettywriter.h \
    include/rapidjson/rapidjson.h \
    include/rapidjson/reader.h \
    include/rapidjson/stringbuffer.h \
    include/rapidjson/writer.h \
    include/rapidjson/internal/dtoa.h \
    include/rapidjson/internal/itoa.h \
    include/rapidjson/internal/meta.h \
    include/rapidjson/internal/pow10.h \
    include/rapidjson/internal/stack.h \
    include/rapidjson/internal/strfunc.h \
    include/rapidjson/msinttypes/inttypes.h \
    include/rapidjson/msinttypes/stdint.h

DISTFILES += \
    makefile \
    README.md
