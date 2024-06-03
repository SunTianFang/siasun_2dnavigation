QT       += core gui
CONFIG +=  c++14

DEFINES += SCAN_MAKE_LIB #定义此宏将构建库

TARGET = Scan
TEMPLATE = lib
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################
include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Geometry.pri)
include($$PWD/../../../Pri/Csm.pri)
include($$PWD/../../../Pri/ThirdParty.pri)
include($$PWD/../../../Pri/Tools.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/ScanPointCloud.cpp \
    src/Scan.cpp \
    src/ScannerParam.cpp \
    src/misc.cpp \
    src/RawScan.cpp \
    src/LineElement.cpp \
    src/CircleElement.cpp \
    src/BasObject.cpp \
    src/StockedObjects.cpp \
    src/StaticObjects.cpp \
    src/LiveObjects.cpp

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
    ../Csm/include \
    ../Csm/include/csm \
    ../Csm/src\
    ../Csm/src/csm \
    include \
    ../ThirdParty/eigen3/


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
DEFINES +=_E3845_LINUX64




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
