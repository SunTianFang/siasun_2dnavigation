QT       += core gui
CONFIG   +=  c++14
DEFINES  += SOFTPLS_MAKE_LIB #定义此宏将构建库

TARGET   = SoftPls
TEMPLATE = lib

include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Scan.pri)
include($$PWD/../../../Pri/ThirdParty.pri)
include($$PWD/../../../Pri/NavBase.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
            src/AvoidObstacle.cpp \
            src/PLS.cpp

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
    ../ThirdParty/eigen3/ \
    ../NavBase/Mapping/include

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
