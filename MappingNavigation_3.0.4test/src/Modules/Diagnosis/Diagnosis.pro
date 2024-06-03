#-------------------------------------------------
#
# Project created by QtCreator 2021-12-29T09:06:41
#
#-------------------------------------------------
QT       += core gui
CONFIG +=  c++14

TARGET = Diagnosis
TEMPLATE = lib

DEFINES += DIAGNOSIS_LIBRARY
DEFINES += MG_ENABLE_HTTP_STREAMING_MULTIPART
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################

QMAKE_CXXFLAGS += -fopenmp \
                  -fPIC

LIBS +=  -lpthread \
         -ldl\
         -lrt\
         -fopenmp

include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Geometry.pri)
include($$PWD/../../../Pri/Tools.pri)
include($$PWD/../../../Pri/Scan.pri)
include($$PWD/../../../Pri/NavBase.pri)
include($$PWD/../../../Pri/ndt_oru.pri)
include($$PWD/../../../Pri/ndt_omp.pri)
include($$PWD/../../../Pri/Methods.pri)
include($$PWD/../../../Pri/Feature.pri)
include($$PWD/../../../Pri/Methods.pri)
include($$PWD/../../../Pri/Csm.pri)
include($$PWD/../../../Pri/ThirdParty.pri)
include($$PWD/../../../Pri/Carto.pri)
#include($$PWD/../../../World.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/LCMTask.cpp \
    src/Localization_Msg.c \
    src/LocalizationTest_Msg.c \
    src/FeatureMatchInfoLcm_.c \
    src/Diagnosis.cpp \
    src/LcmCloudAdjusted.c \
    src/robot_control_t.c \
    src/robot_control_t_new.c \
    src/WebtoolServer/api/archive/ctrlsetting.cpp \
    src/WebtoolServer/api/archive/datalocation.cpp \
    src/WebtoolServer/api/archive/datapcloud.cpp \
    src/WebtoolServer/api/archive/datasetting.cpp \
    src/WebtoolServer/api/apimanager.cpp \
    src/WebtoolServer/api/ctrlapi.cpp \
    src/WebtoolServer/api/dataapi.cpp \
    src/WebtoolServer/api/downloadapi.cpp \
    src/WebtoolServer/server/httpServer.cpp \
    src/WebtoolServer/mongoose/mongoose.c \
    src/WebtoolServer/api/archive/dataparm.cpp \
    src/WebtoolServer/api/archive/datadxscandata.cpp \
    src/WebtoolServer/api/archive/datadxlaserparm.cpp \
    src/WebtoolServer/api/archive/datafinishrecorddx.cpp \
    src/WebtoolServer/api/archive/datastartrecorddx.cpp


INCLUDEPATH += include/ \
                ../Geometry/include \
                ../World/include \
                ../Ndt/ndt_oru/include \
                ../Ndt/ndt_fuser/include \
                ../Common/Platform/include \
                ../Scan/include \
                ../Methods/Manager/include \
                ../Methods/NdtMethod/include \
                ../Methods/FeatureMethod/include \
                ../Methods/TemplateMethod/include \
                ../Methods/SlamMethod/include \
                ../Carto \
                ../../lib/Navigation/Slam/include \
                /src/WebtoolServer \
                ../../App/NavApp/include \
                ../Csm/include \
                ../Csm/include/csm \
                ../Csm/src/csm \
                ../Tools/include \
                ../Methods/FastMatchMethod/include \
                ../ThirdParty  \
		../ThirdParty/eigen3 \
                ../NavBase/Mapping/include \
		../Carto/myceres/include



Desktop{
DEFINES += DesktopRun
unix:!macx: LIBS += -lboost_system \
                    -lboost_filesystem \
                    -lboost_thread \
                    -lboost_atomic

INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9 \
                /opt/glog/glog_h/ \
LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so

INCLUDEPATH +=  /usr/include/glib-2.0 \
                /usr/lib/x86_64-linux-gnu/glib-2.0/include



LIBS += -lglib-2.0
LIBS += -L/opt/glog/glog_so -lglog

}

RK3399{

    DEFINES += _LINUX64
    DEFINES+=_RK3399_ARM_64
    DEFINES += NAV_APP

    INCLUDEPATH += /opt/RK3399/boostlib/include/
    LIBS += -L /opt/RK3399/boostlib/lib/ -lboost_system -lboost_thread -lboost_atomic

    INCLUDEPATH +=  /opt/RK3399/glib_rk3399/glib-2.0 \
                    /opt/RK3399/glib_rk3399/include
    LIBS += /opt/RK3399/glib_rk3399/libglib-2.0*.so

    INCLUDEPATH += /opt/RK3399/pcl_1_9_0/include/pcl-1.9
    LIBS += /opt/RK3399/pcl_1_9_0/lib/libpcl_*.so

    INCLUDEPATH +=/usr/include/eigen3

unix{
    target.path = /home/proembed/CarryBoy/
    INSTALLS += target
  }
}

E3845{
DEFINES += _LINUX64
DEFINES += _E3845_LINUX64
DEFINES += _SERVICE_HARDWARE
DEFINES += NAV_APP

unix:!macx: LIBS += -lboost_system \
                    -lboost_filesystem \
                    -lboost_thread \
                    -lboost_atomic
INCLUDEPATH +=  /usr/include/glib-2.0 \
                /usr/lib/x86_64-linux-gnu/glib-2.0/include




LIBS += -lglib-2.0

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

    INCLUDEPATH += /opt/raspberrypi/sysroot/usr/include/boost/
    LIBS += -L /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/ -lboost_system -lboost_thread -lboost_atomic

    INCLUDEPATH +=  /opt/raspberrypi/sysroot/usr/include/glib-2.0 \
                    /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/glib-2.0/include
    LIBS += /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/libglib-2.0*.so

    INCLUDEPATH += /opt/raspberrypi/sysroot/usr/include/pcl-1.9
    LIBS += /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/libpcl_*.so

unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}


