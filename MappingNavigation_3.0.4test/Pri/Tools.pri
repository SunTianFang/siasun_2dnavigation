INCLUDEPATH += $$PWD/../src/Modules/Tools/include/ \
               $$PWD/../src/Modules/Tools/include/RK3399

DEPENDPATH += $$PWD/../src/Modules/Tools \
             $$PWD/../src/Modules/Tools/include/RK3399
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lTools

