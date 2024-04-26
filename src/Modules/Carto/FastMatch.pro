#-------------------------------------------------
#
# Project created by QtCreator 2022-01-11T22:32:04
#
#-------------------------------------------------





QT       += core gui
CONFIG +=  c++14


QMAKE_CXXFLAGS += -fopenmp -ffast-math



TARGET = FastMatch
TEMPLATE = lib

DEFINES += FASTMATCH_LIBRARY

SOURCES += \
    common/src/logging.cpp \
    common/src/mymath.cpp \
    common/src/time.cpp \
    mapping/src/correlative_scan_matcher_2d.cpp \
    mapping/src/fast_correlative_scan_matcher_2d.cpp \
    mapping/src/grid_2d.cpp \
    mapping/src/probability_grid.cpp \
    mapping/src/probability_grid_range_data_inserter_2d.cpp \
    mapping/src/probability_values.cpp \
    mapping/src/ray_casting.cpp \
    mapping/src/test_fast_match.cpp \
    sensor/src/point_cloud.cpp \
    sensor/src/range_data.cpp \
    transform/src/rigid_transform.cpp \
    transform/src/timestamped_transform.cpp \
    myceres/src/block_sparse_matrix.cc \
    myceres/src/block_structure.cc \
    myceres/src/dense_sparse_matrix.cc \
    myceres/src/location.cpp \
    myceres/src/pose_graph.cpp \
    myceres/src/trust_region_step_evaluator.cc \
    mapping/src/ceres_scan_matcher_2d.cpp \
    mapping/src/gridmap.cpp

HEADERS +=\
    common/include/logging.h \
    common/include/make_unique.h \
    common/include/mutex.h \
    common/include/mymath.h \
    common/include/mytime.h \
    common/include/optional.h \
    common/include/port.h \
    common/include/rate_timer.h \
    mapping/include/correlative_scan_matcher_2d.h \
    mapping/include/fast_correlative_scan_matcher_2d.h \
    mapping/include/grid_2d.h \
    mapping/include/grid_interface.h \
    mapping/include/id.h \
    mapping/include/interface.h \
    mapping/include/map_limits.h \
    mapping/include/options.h \
    mapping/include/probability_grid.h \
    mapping/include/probability_grid_range_data_inserter_2d.h \
    mapping/include/probability_values.h \
    mapping/include/range_data_inserter_interface.h \
    mapping/include/ray_casting.h \
    mapping/include/submaps.h \
    mapping/include/test_fast_match.h \
    mapping/include/trajectory_node.h \
    mapping/include/xy_index.h \
    sensor/include/point_cloud.h \
    sensor/include/range_data.h \
    sensor/include/sensorproto.h \
    sensor/include/timed_point_cloud_data.h \
    transform/include/rigid_transform.h \
    transform/include/timestamped_transform.h \
    transform/include/transform.h \
    transform/include/transformproto.h \
    myceres/include/block_sparse_matrix.h \
    myceres/include/block_structure.h \
    myceres/include/config.h \
    myceres/include/cubic_interpolation.h \
    myceres/include/dense_sparse_matrix.h \
    myceres/include/eigen.h \
    myceres/include/fixed_array.h \
    myceres/include/fpclassify.h \
    myceres/include/Grid2d_T.h \
    myceres/include/jet.h \
    myceres/include/linear_solver.h \
    myceres/include/location.h \
    myceres/include/manual_constructor.h \
    myceres/include/occupied_space_cost_function_2d.h \
    myceres/include/pointCloud_T.h \
    myceres/include/port.h \
    myceres/include/pose_graph.h \
    myceres/include/scoped_ptr.h \
    myceres/include/small_blas.h \
    myceres/include/spa_cost_function_2d.h \
    myceres/include/trust_region_step_evaluator.h \
    myceres/include/type.h \
    mapping/include/submap_2d.h \
    mapping/include/gridmap.h



include($$PWD/../../../common.pri)

DESTDIR = $$LIFELONG_SLAM_DIR



INCLUDEPATH += \
    ../FastMatch/common/include \
    ../FastMatch/mapping/include \
    ../FastMatch/sensor/include \
    ../FastMatch/transform/include \
    ../FastMatch/myceres/include \
    ../ThirdParty/eigen3/


Desktop{
DEFINES += DesktopRun
}


RK3399{
DEFINES+=_RK3399_ARM_64
unix{
    target.path = /home/proembed/CarryBoy/
    INSTALLS += target
  }
}

E3845{
DEFINES += _E3845_LINUX64
unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}

PICM4{
CONFIG +=  c++17
DEFINES += _PiCM4_LINUX32
unix {
        target.path = /home/pi/CarryBoy/
        INSTALLS += target
     }
}



