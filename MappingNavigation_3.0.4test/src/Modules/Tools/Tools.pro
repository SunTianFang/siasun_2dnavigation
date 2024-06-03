QT       += core gui
CONFIG +=  c++14

DEFINES += WORLD_MAKE_LIB #定义此宏将构建库

TARGET = Tools
TEMPLATE = lib
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################
include($$PWD/../../../Pri/common.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
            src/Archive.cpp \
            src/Tools.cpp \
            src/time_patch.cpp \
            src/Combination.cpp \
            src/AbstractParameter.cpp \
            src/Gpio.cpp \
            src/LED.cpp \
            src/systemInfo.cpp \
            src/WatchDog.cpp

INCLUDEPATH += \
    ../Tools/include \
    ../Geometry/include \
    include \
    include/RK3399 \
    /home/ymh/ceres-solver-1.14.0/ceres-solver-1.14.0/include \
    /home/ymh/ceres-solver-1.14.0/ceres-solver-1.14.0/include/ceres

HEADERS  += \



Desktop{
DEFINES += DesktopRun
}


RK3399{
DEFINES+=_RK3399_ARM_64
unix{
    target.path = /home/proembed/CarryBoy/
    INSTALLS += target
  }
}

E3845{
DEFINES += _LINUX64
DEFINES += _E3845_LINUX64
unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}
PICM4{
    CONFIG +=  c++17
    DEFINES += _PiCM4_LINUX32
unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}
