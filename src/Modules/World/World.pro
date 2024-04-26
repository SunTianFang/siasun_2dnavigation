QT       += core gui
CONFIG +=  c++14

DEFINES += WORLD_MAKE_LIB #定义此宏将构建库

TARGET = World
TEMPLATE = lib
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################
include($$PWD/../../../Pri/Geometry.pri)
include($$PWD/../../../Pri/Tools.pri)
include($$PWD/../../../Pri/Traj.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/ArcPath.cpp \
    src/GenericPath.cpp \
    src/LinePath.cpp \
    src/Node.cpp \
    src/NodeBase.cpp \
    src/Path.cpp \
    src/PathBase.cpp \
    src/RfId.cpp \
    src/ScpPath.cpp \
    src/SidePath.cpp \
    src/UnknownPath.cpp \
    src/Vertex.cpp \
    src/World.cpp \
    src/GraphicObj.cpp

INCLUDEPATH += \
    ../Tools/include \
    ../Geometry/include \
    ../Traj/include \
    include

HEADERS  += \

Desktop{
DEFINES += DesktopRun
}
