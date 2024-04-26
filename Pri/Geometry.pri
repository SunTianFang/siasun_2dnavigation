INCLUDEPATH += $$PWD/../src/Modules/Geometry/include
DEPENDPATH += $$PWD/../src/Modules/Geometry
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lGeometry

