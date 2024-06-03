QT       += core gui
CONFIG +=  c++14

DEFINES += CSM_MAKE_LIB #定义此宏将构建库

TARGET = Csm
TEMPLATE = lib

#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################

include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Tools.pri)
include($$PWD/../../../Pri/Geometry.pri)
include($$PWD/../../../Pri/NavBase.pri)


DESTDIR = $$LIFELONG_SLAM_DIR

INCLUDEPATH += \
    ../NavBase/BlackBox/include \
    ../NavBase/Protocol/include

SOURCES += \
    src/csm/CorrList.cpp \
    src/csm/CsmMatcher.cpp \
    src/csm/CsmScan.cpp \
    src/csm/fastMatcher.cpp \
    src/csm/math_utils.cpp \
    src/csm/math_utils_gsl.cpp \
    src/egsl/egsl.cpp \
    src/egsl/egsl_conversions.cpp \
    src/egsl/egsl_misc.cpp \
    src/egsl/egsl_ops.cpp \
    src/gpc/gpc.cpp \
    src/gpc/gpc_utils.cpp

unix {
#INCLUDEPATH += /usr/include/eigen3
#INCLUDEPATH += ../ThirdParty/eigen3/
}

win32 {
INCLUDEPATH += d:/eigen3
}

INCLUDEPATH += \
    ../Scan/include \
    ../Tools/include \
    ../Geometry/include \
    include \
    include/csm \
    src\
    src/csm \
    ../ThirdParty/eigen3/ \
    ../NavBase/Mapping/include


Desktop{
DEFINES += DesktopRun
INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9
LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so
}

RK3399{
DEFINES+=_RK3399_ARM_64
    INCLUDEPATH += /opt/RK3399/boostlib/include/
    LIBS += -L /opt/RK3399/boostlib/lib/ -lboost_system -lboost_thread -lboost_atomic

    INCLUDEPATH += /opt/RK3399/pcl_1_9_0/include/pcl-1.9
    LIBS += /opt/RK3399/pcl_1_9_0/lib/libpcl_*.so

unix{
    target.path = /home/proembed/CarryBoy/
    INSTALLS += target
  }
}

E3845{
DEFINES += _LINUX64
DEFINES += _E3845_LINUX64

INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9

LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so

unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}



PICM4{
CONFIG +=  c++17
DEFINES += _PiCM4_LINUX32

    INCLUDEPATH += /opt/raspberrypi/sysroot/usr/include/pcl-1.9
    LIBS += /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/libpcl_*.so

unix {
    target.path = /home/siasun/CarryBoy/
    INSTALLS += target
 }
}
