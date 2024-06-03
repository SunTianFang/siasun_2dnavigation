INCLUDEPATH += $$PWD/../src/Modules/Ndt/ndt_oru
DEPENDPATH += $$PWD/../src/Modules/Ndt/ndt_oru
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lndt_oru

