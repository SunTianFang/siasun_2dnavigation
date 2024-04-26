TEMPLATE = app
CONFIG += console  c++14
CONFIG -= app_bundle
QT       += core gui

#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################

TARGET = robo_localization

DEFINES += MG_ENABLE_HTTP_STREAMING_MULTIPART
DEFINES += NAV_APP

QMAKE_CXXFLAGS += -fopenmp

LIBS +=  -lpthread \
         -ldl\
         -lrt\
         -fopenmp

include($$PWD/../../../Pri/Carto.pri)
include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Geometry.pri)
include($$PWD/../../../Pri/Tools.pri)
include($$PWD/../../../Pri/Scan.pri)
include($$PWD/../../../Pri/NavBase.pri)
include($$PWD/../../../Pri/ndt_oru.pri)
include($$PWD/../../../Pri/ndt_omp.pri)
include($$PWD/../../../Pri/Methods.pri)
include($$PWD/../../../Pri/Feature.pri)
include($$PWD/../../../Pri/Scan.pri)
include($$PWD/../../../Pri/Methods.pri)
include($$PWD/../../../Pri/Csm.pri)
include($$PWD/../../../Pri/ThirdParty.pri)
include($$PWD/../../../Pri/SoftPls.pri)
include($$PWD/../../../Pri/Diagnosis.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/LocalizeFactory.cpp \
    src/main.cpp \
    src/RoboManager.cpp \
    ../../Modules/NavBase/Protocol/src/httpcommunicationglobaldata.cpp

INCLUDEPATH += include/ \
                ../../Modules/Geometry/include \
                ../../Modules/Traj/include \
                ../../Modules/World/include \
                ../../Modules/Feature/FeatureObject/include \
                ../../Modules/Feature/FeatureMatch/include \
                ../../Modules/Csm/include \
                ../../Modules/Csm/include/csm \
                ../../Modules/Csm/src/csm \
                ../../Modules/Ndt/ndt_oru/include \
                ../../Modules/Ndt/ndt_omp/include \
                ../../Modules/Ndt/interface/include \
                ../../Modules/Ndt/ndt_fuser/include \
                ../../Modules/Common/Platform/include \
                ../../Modules/Tools/include \
                ../../Modules/Scan/include \
                ../../Modules/Diagnosis/include \
                ../../Modules/Carto \
                ../../Modules/Methods/Manager/include \
                ../../Modules/Methods/NdtMethod/include \
                ../../Modules/Methods/FeatureMethod/include \
                ../../Modules/Methods/TemplateMethod/include \
                ../../Modules/Methods/FastMatchMethod/include \
                ../../Modules/Methods/ScanMatchMethod/include \
                ../../Modules/Methods/SlamMethod/include \
                ../../Modules/Editable/Feature/include \
                ../../Modules/Editable/Methods/include \
                ../../Modules/Editable/StaticObjs/include \
                ../../Modules/Editable/Manager/include \
                ../../lib/Navigation/Tools/include \
                ../../lib/Navigation/Slam/include \
		../../Modules/SoftPls/include \
                ../../Modules/NavBase/Protocol/include \
                ../../Modules/NavBase/BlackBox/include \
                ../../Modules/Diagnosis/src/WebtoolServer \
                ../../Modules/ThirdParty/eigen3 \
                ../../Modules/NavBase/Mapping/include


Desktop{
DEFINES += DesktopRun
unix:!macx: LIBS += -lboost_system \
                    -lboost_filesystem \
                    -lboost_thread \
                    -lboost_atomic

INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9 \
                /opt/glog/glog_h/
LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so

INCLUDEPATH +=  /usr/include/glib-2.0 \
                /usr/lib/x86_64-linux-gnu/glib-2.0/include
INCLUDEPATH += /usr/include/glog/
LIBS += -lglib-2.0

LIBS +=$$PWD/../../Modules/NavBase/Mapping/lib/libCalibrateTool_x86.so

}
RK3399{
    DEFINES +=_RK3399_ARM_64
    DEFINES += NAV_APP

    INCLUDEPATH += /opt/RK3399/boostlib/include/
    LIBS += -L /opt/RK3399/boostlib/lib/ -lboost_system -lboost_thread -lboost_atomic -lboost_iostreams

    INCLUDEPATH +=  /opt/RK3399/glib_rk3399/glib-2.0 \
                    /opt/RK3399/glib_rk3399/include
    LIBS += /opt/RK3399/glib_rk3399/libglib-2.0*.so

    INCLUDEPATH += /opt/RK3399/pcl_1_9_0/include/pcl-1.9
    LIBS += /opt/RK3399/pcl_1_9_0/lib/libpcl_*.so
LIBS +=$$PWD/../../../Parameters/libCalibrateTool_RK3399.so

    INCLUDEPATH += /usr/include/glog/


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
                /usr/lib/x86_64-linux-gnu/glib-2.0/include #\
               # /usr/local/include/glog/
INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9 #\

LIBS += -lglib-2.0
INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9
LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so

#LIBS +=$$PWD/../../../Parameters/libCalibrateTool_x86.so
LIBS +=$$PWD/../../Modules/NavBase/Mapping/lib/libCalibrateTool_x86.so

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

    LIBS +=$$PWD/../../../Parameters/libCalibrateTool_PI.so

unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}

