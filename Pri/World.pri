INCLUDEPATH += $$PWD/../src/World/include
DEPENDPATH += $$PWD/../src/World
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lWorld

