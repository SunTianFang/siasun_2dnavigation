INCLUDEPATH += $$PWD/src/Modules/Methods/include
DEPENDPATH += $$PWD/src/Modules/Methods
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lMethods

