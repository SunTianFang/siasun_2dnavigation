QT       += core gui
CONFIG +=  c++14

DEFINES += TRAJ_MAKE_LIB #定义此宏将构建库

TARGET = Traj
TEMPLATE = lib
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################
include($$PWD/../../../Pri/Geometry.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/ArcTraj.cpp \
    src/LineTraj.cpp \
    src/LzySTraj.cpp \
    src/SideTraj.cpp \
    src/SpinTraj.cpp \
    src/SplnTraj.cpp \
    src/ScpTraj.cpp \
    src/SppTraj.cpp \
    src/SteeTraj.cpp


INCLUDEPATH += \
    ../Tools/include \
    include

HEADERS  += \

Desktop{
DEFINES += DesktopRun
}
