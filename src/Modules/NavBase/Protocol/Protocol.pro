#-------------------------------------------------
#
# Project created by QtCreator 2021-09-01T15:02:54
#
#-------------------------------------------------

TARGET = Protocol
TEMPLATE = lib

DEFINES += PROTOCOL_LIBRARY

include($$PWD/../../../BlackBox.pri)
include($$PWD/../../../common.pri)
include($$PWD/../../../Tools.pri)
include($$PWD/../../../Network.pri)
include($$PWD/../../../ThirdParty.pri)
include($$PWD/../../../LaserSensor.pri)
include($$PWD/../../../LaserDrivers.pri)
include($$PWD/../../../Geometry.pri)
include($$PWD/../../../CanDev.pri)
include($$PWD/../../../Scan.pri)
include($$PWD/../../../Csm.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/AutoOutPutBlackBox.cpp \
    src/CPing.cpp \
    src/ParameterObject.cpp \
    src/RoboLocClnt.cpp \
    src/RoboLocProto.cpp \
    src/BaseOdometry.cpp \
    src/LaserMapping.cpp \
    src/LaserOdometry.cpp \
    src/RawMap.cpp

INCLUDEPATH += \
            include
unix {
INCLUDEPATH += \
    /usr/local/include/eigen3
}
