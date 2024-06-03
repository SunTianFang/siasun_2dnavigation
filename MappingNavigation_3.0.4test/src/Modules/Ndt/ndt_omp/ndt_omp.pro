QT       += core gui
CONFIG +=  c++14


#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

DEFINES += NDT_PCL_MAKE_LIB #定义此宏将构建库

TARGET = ndt_omp
TEMPLATE = lib
#QMAKE_CXXFLAGS_RELEASE += -O0
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################
include($$PWD/../../../../Pri/common.pri)
include($$PWD/../../../../Pri/Geometry.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/voxel_grid_covariance_omp.cpp \
    src/ndt_omp.cpp \
    src/NdtLocalization_omp.cpp

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
    d:/PCL-1.8.1/3rdParty/Boost/include/boost-1_64 \
    d:/PCL-1.8.1/3rdParty/FLANN/include
}

INCLUDEPATH += \
    ../../../lib/MiniEigen/include \
    ../../../Modules/Tools/include \
    ../../../Modules/Scan/include \
    ../../../lib/Navigation/Slam/include \
    include \
    ../../../Modules/Ndt/ndt_fuser/include \
    ../../../Modules/Ndt/ndt_oru/include \
    ../../../Modules/Common/Platform/include \
    ../../Geometry/include \
    ../../ThirdParty/eigen3/ 
    


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
