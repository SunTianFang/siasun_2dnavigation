INCLUDEPATH += $$PWD/../src/Traj/include
DEPENDPATH += $$PWD/../src/Traj
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lTraj

