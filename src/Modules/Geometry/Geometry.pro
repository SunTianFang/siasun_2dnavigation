QT       += core gui
CONFIG += c++14
DEFINES += GEOMETRY_MAKE_LIB #定义此宏将构建库

TARGET = Geometry
TEMPLATE = lib
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################
include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Tools.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/Angle.cpp \
    src/Arc.cpp \
    src/Bezier.cpp \
    src/Circle.cpp \
    src/Direct.cpp \
    src/Ellipse.cpp \
    src/Frame.cpp \
    src/Line.cpp \
    src/LineBase.cpp \
    src/MultiSegLine.cpp \
    src/NewCurve.cpp \
    src/Pnt.cpp \
    src/PolyRegion.cpp \
    src/Posture.cpp \
    src/DataRange.cpp \
    src/Rectangle.cpp \
    src/ScrnRef.cpp \
    src/Spline.cpp \
    src/Scp.cpp \
    src/Spp.cpp \
    src/Transfor.cpp \
    src/AppArea.cpp \
    src/AffinePosture.cpp \
    src/MatchInfo.cpp


INCLUDEPATH += \
    ../../lib/Tools/include \
    include \
    ../ThirdParty/eigen3/
#    ../../Modules/Tools/include/

unix {
    #INCLUDEPATH += /usr/include/eigen3 \
    #INCLUDEPATH += ../ThirdParty/eigen3/
}



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
