INCLUDEPATH += $$PWD/src/Modules/Ndt/ndt_omp
DEPENDPATH += $$PWD/src/Modules/Ndt/ndt_omp
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lndt_omp
