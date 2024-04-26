QT       += core gui
CONFIG +=  c++14


#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

DEFINES += NDT_ORU_MAKE_LIB #定义此宏将构建库

QMAKE_CXXFLAGS += -fopenmp

win32 {
QMAKE_CXXFLAGS += -BigObj
QMAKE_CXXFLAGS += -Ofast -flto
}

TARGET = ndt_oru
TEMPLATE = lib

#QMAKE_CXXFLAGS_RELEASE += -O0

########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################
include($$PWD/../../../../Pri/Carto.pri)
include($$PWD/../../../../Pri/common.pri)
include($$PWD/../../../../Pri/Geometry.pri)
include($$PWD/../../../../Pri/Tools.pri)
include($$PWD/../../../../Pri/Feature.pri)
include($$PWD/../../../../Pri/Scan.pri)
include($$PWD/../../../../Pri/ThirdParty.pri)
include($$PWD/../../../../Pri/NavBase.pri)
#include($$PWD/../../../../Pri/Diagnosis.pri)


DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/ndt_pointcloud.cpp \
    src/ndt_cell_oru.cpp \
    src/lazy_grid_oru.cpp \
    src/ndt_map_oru.cpp \
    src/ndt_maps_oru.cpp \
    src/ndt_matcher_d2d_2d.cpp \
    src/NdtBaseLocalization_oru.cpp \
    src/NdtExtLocalization_oru.cpp \
    src/ndt_inform_oru.cpp \
#    src/LocalizationParam.cpp \
     ../ndt_fuser/src/NdtCellEditable.cpp \
    ../ndt_fuser/src/NdtMapEditable.cpp \
    ../ndt_fuser/src/NdtMapsEditable.cpp \
    ../ndt_fuser/src/DatasetLocalization.cpp \
#    ../ndt_fuser/src/NdtLocArea.cpp \
#    ../ndt_fuser/src/FeatureLocArea.cpp \
#    ../ndt_fuser/src/TemplateLocArea.cpp \
    ../ndt_fuser/src/MapFuser.cpp \
#    ../ndt_fuser/src/NdtMethod.cpp \
#    ../ndt_fuser/src/FeatureMethod.cpp \
#    ../ndt_fuser/src/TemplateMethod.cpp \
#    ../ndt_fuser/src/LocalizationMethods.cpp \
#    ../ndt_fuser/src/LocalizationPlan.cpp \
#    ../ndt_fuser/src/DualNdtLocalization.cpp \
#    ../ndt_fuser/src/LocalizeFactory.cpp \
    ../ndt_fuser/src/VectPose.cpp

unix {
INCLUDEPATH += \
#    /usr/include/eigen3 \
#    /usr/include/pcl-1.10

#INCLUDEPATH += ../../ThirdParty/eigen3/

}






win32 {
INCLUDEPATH += \
    d:/eigen3 \
    d:/pcl-1.8.1/include/pcl-1.8 \
    d:/pcl-1.8.1/3rdParty/Boost/include/boost-1_64 \
    d:/pcl-1.8.1/3rdParty/FLANN/include
}

INCLUDEPATH +=  \
    ../../Tools/include \
    ../../Geometry/include \
    ../../Scan/include \
    ../../Csm/include/csm \
    ../../Csm/include \
    ../../Csm/src/csm \
    ../../Feature/include \
    ../../World/include \
    ../../../lib/Pcl/include \
    ../../../lib/MiniEigen/include \
    ../../../lib/Navigation/Slam/include \
    ../../../Modules/Common/Platform/include \
    ../ndt_fuser/include \
    include \
    ../ndt_omp/include \
    ../../ThirdParty/eigen3/ \
    ../../../Modules/NavBase/Mapping/include

HEADERS  += \

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

