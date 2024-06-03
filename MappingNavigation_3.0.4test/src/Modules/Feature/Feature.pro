QT       += core gui
CONFIG +=  c++14

DEFINES += FEATURE_MAKE_LIB #定义此宏将构建库

TARGET = Feature
TEMPLATE = lib
#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################
include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Geometry.pri)
include($$PWD/../../../Pri/ThirdParty.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    FeatureObject/src/PointFeature.cpp \
    FeatureObject/src/ShortLineFeature.cpp \
    FeatureObject/src/FlatReflectorFeature.cpp \
    FeatureObject/src/CornerPointFeature.cpp \
    FeatureObject/src/CylinderFeature.cpp \
    FeatureObject/src/RectFeature.cpp \
    FeatureObject/src/LineFeature.cpp \
    FeatureObject/src/LineFeatureSet.cpp \
    FeatureObject/src/PointFeatureSet.cpp \
    FeatureObject/src/FeatureSet.cpp \
    FeatureObject/src/FeatureMap.cpp \
    FeatureMatch/src/CMatrix.cpp \
    FeatureMatch/src/LeastSquareMethod.cpp \
    FeatureMatch/src/LineMatchPair.cpp \
    FeatureMatch/src/LineMatchList.cpp \
    FeatureMatch/src/PointMatchPair.cpp \
    FeatureMatch/src/PointMatchList.cpp \
    FeatureMatch/src/MatchTabSet.cpp


win32 {
INCLUDEPATH += d:/eigen3 \
}

INCLUDEPATH += \
    ../Tools/include \
    ../Scan/include \
    ../Geometry/include \
    ../Csm \
    ../Csm/src/csm \
    ../Csm/include \
    ../Csm/include/csm \
    ../World/include \
    FeatureObject/include \
    FeatureMatch/include \
    ../ThirdParty/eigen3/


HEADERS  += \



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
