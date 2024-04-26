QT       += core gui
CONFIG +=  c++14

#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

DEFINES += METHODS_MAKE_LIB #定义此宏将构建库

TARGET = Methods
TEMPLATE = lib
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################

include($$PWD/../../../Pri/Carto.pri)
include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Geometry.pri)
include($$PWD/../../../Pri/Tools.pri)
include($$PWD/../../../Pri/Scan.pri)
include($$PWD/../../../Pri/ndt_oru.pri)
include($$PWD/../../../Pri/ThirdParty.pri)
#include($$PWD/../../../Pri/Diagnosis.pri)


#By Sam
include($$PWD/../../../Pri/NavBase.pri)


DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    NdtMethod/src/NdtLocArea.cpp \
    NdtMethod/src/NdtMethod.cpp \
    FeatureMethod/src/FeatureLocArea.cpp \
    FeatureMethod/src/FeatureMethod.cpp \
    TemplateMethod/src/TemplateLocArea.cpp \
    TemplateMethod/src/TemplateMethod.cpp \
    Manager/src/LocalizationMethods.cpp \
    Manager/src/LocalizationPlan.cpp \
    Manager/src/LocalizationManager.cpp \
    FeatureMethod/src/FeatureScan.cpp \
    FeatureMethod/src/FeatureLocalization.cpp \
    FastMatchMethod/src/FastMatchMethod.cpp \
    ScanMatchMethod/src/ScanMatchMethod.cpp \
    SlamMethod/src/SlamLocArea.cpp \
    SlamMethod/src/SlamMethod.cpp \
    ScanMatchMethod/src/ScanMatchLocArea.cpp



win32 {
INCLUDEPATH += \
    d:/eigen3 \
    d:/pcl-1.8.1/include/pcl-1.8 \
    d:/pcl-1.8.1/3rdParty/Boost/include/boost-1_64 \
    d:/pcl-1.8.1/3rdParty/FLANN/include
}

INCLUDEPATH +=  \
    ../Tools/include \
    ../Geometry/include \
    ../Scan/include \
    ../Csm/include/csm \
    ../Csm/include \
    ../Csm/src/csm \
    ../Feature/FeatureObject/include \
    ../Feature/FeatureMatch/include \
    ../World/include \
    ../../lib/MiniEigen/include \
    ../../lib/Navigation/Slam/include \
    ../../Modules/Common/Platform/include \
    ../Ndt/ndt_oru/include \
    ../Ndt/ndt_fuser/include \
    NdtMethod/include \
    FeatureMethod/include \
    TemplateMethod/include \
    FastMatchMethod/include \
    ScanMatchMethod/include \
    SlamMethod/include \
    Manager/include \
    ../ThirdParty/eigen3 \
    ../../App/NavApp/include/

Desktop{
DEFINES += DesktopRun
INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9 \
                /opt/glog/glog_h/
LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so
}

RK3399{
DEFINES+=_RK3399_ARM_64

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
INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9 \
#   /usr/include/glog/
LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so
#LIBS+= /usr/lib/x86_64-linux-gnu/libglog.so

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
        target.path = /home/pi/CarryBoy/
        INSTALLS += target
     }
}

HEADERS += \
    FastMatchMethod/include/FastMatchMethod.h \
    FeatureMethod/include/FeatureLocalization.h \
    FeatureMethod/include/FeatureLocArea.h \
    FeatureMethod/include/FeatureMatchInfo.h \
    FeatureMethod/include/FeatureMethod.h \
    Manager/include/LocalizationManager.h \
    Manager/include/LocalizationMethod.h \
    Manager/include/LocalizationMethods.h \
    Manager/include/LocalizationParam.h \
    Manager/include/LocalizationPlan.h \
    NdtMethod/include/NdtLocArea.h \
    NdtMethod/include/NdtMethod.h \
    ScanMatchMethod/include/ScanMatchMethod.h \
    SlamMethod/include/SlamMethod.h \
    SlamMethod/include/SlamLocArea.h \
    TemplateMethod/include/TemplateLocArea.h \
    TemplateMethod/include/TemplateMethod.h

