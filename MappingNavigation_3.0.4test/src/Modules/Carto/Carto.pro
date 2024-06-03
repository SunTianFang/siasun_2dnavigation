#-------------------------------------------------
#
# Project created by QtCreator 2022-02-13T20:38:02
#
#-------------------------------------------------

QT       -= gui
CONFIG +=  c++14

TARGET = Carto
TEMPLATE = lib

DEFINES += CARTO_LIBRARY

QMAKE_CXXFLAGS += -fopenmp -ffast-math

include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Tools.pri)
include($$PWD/../../../Pri/ThirdParty.pri)

SOURCES += \
    common/src/mymath.cpp \
    common/src/time.cpp \
    mapping/src/correlative_scan_matcher_2d.cpp \
    mapping/src/fast_correlative_scan_matcher_2d.cpp \
    mapping/src/grid_2d.cpp \
    mapping/src/gridmap.cpp \
    mapping/src/probability_grid_range_data_inserter_2d.cpp \
    mapping/src/probability_grid.cpp \
    mapping/src/probability_values.cpp \
    mapping/src/ray_casting.cpp \
    sensor/src/point_cloud.cpp \
    sensor/src/range_data.cpp \
    transform/src/rigid_transform.cpp \
    transform/src/timestamped_transform.cpp \
    myceres/src/block_sparse_matrix.cc \
    myceres/src/block_structure.cc \
    myceres/src/dense_sparse_matrix.cc \
    myceres/src/location.cpp \
    myceres/src/trust_region_step_evaluator.cc \
    mapping/src/ceres_scan_matcher_2d.cpp \
    common/src/thread_pool.cpp \
    common/src/fixed_ratio_sampler.cpp \
    mapping/src/submap_2d.cpp \
    mapping/src/range_data_collator.cpp \
    mapping/src/pose_graph_trimmer.cpp \
    mapping/src/pose_graph_2d.cpp \
    mapping/src/pose_extrapolator.cpp \
    mapping/src/optimization_problem_2d.cpp \
    mapping/src/motion_filter.cpp \
    mapping/src/map_builder.cpp \
    mapping/src/local_trajectory_builder_2d.cpp \
    mapping/src/imu_tracker.cpp \
    mapping/src/global_trajectory_builder.cpp \
    mapping/src/constraint_builder_2d.cpp \
    sensor/src/voxel_filter.cpp \
    sensor/src/ordered_multi_queue.cpp \
    sensor/src/collator.cpp \
    myceres/src/optimize_pose_graph.cpp \
    common/src/task.cpp \
    transform/src/transform_interpolation_buffer.cpp \
    node.cpp \
    navigation.cpp \
    map_builder_bridge.cpp \
    NaviInterface.cpp \
    mapping/src/real_time_correlative_scan_matcher_2d.cpp \
    common/src/mylogging.cpp

HEADERS +=\
        carto_global.h \
        type.h



INCLUDEPATH += \
    common/include \
    transform/include \
    sensor/include \
    mapping/include \
    myceres/include \
    ../ThirdParty/eigen3/



#LIBS += -lgflags



DESTDIR = $$LIFELONG_SLAM_DIR

#unix {
#   target.path = /usr/lib
# #   INSTALLS += target
#}

Desktop{
DEFINES += DesktopRun
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lglog
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lgflags
INCLUDEPATH += /usr/include/glog/
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

#LIBS += -L/usr/lib/x86_64-linux-gnu/ -lgflags
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

DISTFILES +=

