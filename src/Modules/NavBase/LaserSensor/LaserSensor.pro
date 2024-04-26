#-------------------------------------------------
#
# Project created by QtCreator 2021-09-01T14:07:42
#
#-------------------------------------------------

TARGET = LaserSensor
TEMPLATE = lib

DEFINES += LASERSENSOR_LIBRARY

include($$PWD/../../../common.pri)
include($$PWD/../../../Tools.pri)
include($$PWD/../../../LaserDrivers.pri)
include($$PWD/../../../Geometry.pri)
include($$PWD/../../../Network.pri)
include($$PWD/../../../ThirdParty.pri)
include($$PWD/../../../BlackBox.pri)
include($$PWD/../../../Scan.pri)
include($$PWD/../../../Csm.pri)

DESTDIR = $$LIFELONG_SLAM_DIR
unix {
INCLUDEPATH += \
    /usr/local/include/eigen3
}
SOURCES += \
    src/HokuyoLaserScanner.cpp \
    src/PfR2000LaserScanner.cpp \
    src/RangeScanner.cpp \
    src/SensorFamily.cpp \
    src/Sick581LaserScanner.cpp \
    src/SickSafetyLaserScanner.cpp \
    src/laser_t.c

INCLUDEPATH += include/ \
