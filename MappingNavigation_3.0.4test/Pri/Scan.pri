INCLUDEPATH += $$PWD/../src/Modules/Scan/include
DEPENDPATH += $$PWD/../src/Modules/Scan
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lScan

