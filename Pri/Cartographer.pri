INCLUDEPATH += $$PWD/src/Cartographer
DEPENDPATH += $$PWD/src/Cartographer
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lCartographer

