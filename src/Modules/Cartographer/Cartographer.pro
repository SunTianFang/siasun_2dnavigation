QT       += core gui
CONFIG += c++14

DEFINES += CARTOGRAPHER_MAKE_LIB #定义此宏将构建库

TARGET = Cartographer
TEMPLATE = lib

include($$PWD/../../../common.pri)
DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += \
    src/cartographer/cartographer/common/lua_parameter_dictionary.cc \
    src/cartographer/cartographer/common/internal/ceres_solver_options.cc \
    src/cartographer/cartographer/common/time.cc \
    src/cartographer/cartographer/common/task.cc \
    src/cartographer/cartographer/common/histogram.cc \
    src/cartographer/cartographer/common/thread_pool.cc \
    src/cartographer/cartographer/common/fixed_ratio_sampler.cc \
    src/cartographer/cartographer/common/configuration_file_resolver.cc \
    src/cartographer/cartographer/metrics/counter.cc \
    src/cartographer/cartographer/metrics/gauge.cc \
    src/cartographer/cartographer/metrics/histogram1.cc \
    src/cartographer/cartographer/io/proto_stream_deserializer.cc \
    src/cartographer/cartographer/io/proto_stream.cc \
    src/cartographer/cartographer/io/internal/mapping_state_serialization.cc \
    src/cartographer/cartographer/io/submap_painter.cc \
    src/cartographer/cartographer/io/image.cc \
    src/cartographer/cartographer/transform/timestamped_transform.cc \
    src/cartographer/cartographer/transform/transform.cc \
    src/cartographer/cartographer/mapping/internal/motion_filter.cc \
    src/cartographer/cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.cc \
    src/cartographer/cartographer/mapping/internal/global_trajectory_builder.cc \
    src/cartographer/cartographer/mapping/internal/imu_based_pose_extrapolator.cc \
    src/cartographer/cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.cc \
    src/cartographer/cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_options_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/tsdf_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/tsd_value_converter.cc \
    src/cartographer/cartographer/mapping/internal/2d/normal_estimation_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/ray_to_pixel_mask.cc \
    src/cartographer/cartographer/mapping/internal/2d/pose_graph_2d.cc \
    src/cartographer/cartographer/mapping/internal/3d/local_trajectory_builder_options_3d.cc \
    src/cartographer/cartographer/mapping/internal/3d/local_trajectory_builder_3d.cc \
    src/cartographer/cartographer/mapping/internal/3d/pose_graph_3d.cc \
    src/cartographer/cartographer/mapping/internal/3d/scan_matching/intensity_cost_function_3d.cc \
    src/cartographer/cartographer/mapping/trajectory_node.cc \
    src/cartographer/cartographer/mapping/map_builder.cc \
    src/cartographer/cartographer/mapping/pose_extrapolator.cc \
    src/cartographer/cartographer/mapping/pose_extrapolator_interface.cc \
    src/cartographer/cartographer/mapping/trajectory_builder_interface.cc \
    src/cartographer/cartographer/mapping/imu_tracker.cc \
    src/cartographer/cartographer/mapping/range_data_inserter_interface.cc \
    src/cartographer/cartographer/mapping/pose_graph_trimmer.cc \
    src/cartographer/cartographer/mapping/value_conversion_tables.cc \
    src/cartographer/cartographer/mapping/2d/submap_2d.cc \
    src/cartographer/cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc \
    src/cartographer/cartographer/mapping/3d/submap_3d.cc \
    src/cartographer/cartographer/mapping/3d/range_data_inserter_3d.cc \
    src/cartographer/cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc \
    src/cartographer/cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.cc \
    src/cartographer/cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.cc \
    src/cartographer/cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.cc \
    src/cartographer/cartographer/mapping/internal/3d/scan_matching/low_resolution_matcher.cc \
    src/cartographer/cartographer/mapping/internal/3d/scan_matching/precomputation_grid_3d.cc \
    src/cartographer/cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.cc \
    src/cartographer/cartographer/mapping/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d.cc \
    src/cartographer/cartographer/mapping/internal/optimization/ceres_pose.cc \
    src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_options.cc \
    src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.cc \
    src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_3d.cc \
    src/cartographer/cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.cc \
    src/cartographer/cartographer/mapping/internal/trajectory_connectivity_state.cc \
    src/cartographer/cartographer/mapping/internal/connected_components.cc \
    src/cartographer/cartographer/mapping/internal/range_data_collator.cc \
    src/cartographer/cartographer/mapping/internal/collated_trajectory_builder.cc \
    src/cartographer/cartographer/mapping/internal/constraints/constraint_builder.cc \
    src/cartographer/cartographer/mapping/internal/constraints/constraint_builder_2d.cc \
    src/cartographer/cartographer/mapping/internal/constraints/constraint_builder_3d.cc \
    src/cartographer/cartographer/sensor/point_cloud.cc \
    src/cartographer/cartographer/sensor/compressed_point_cloud.cc \
    src/cartographer/cartographer/sensor/landmark_data.cc \
    src/cartographer/cartographer/sensor/odometry_data.cc \
    src/cartographer/cartographer/sensor/range_data.cc \
    src/cartographer/cartographer/sensor/imu_data.cc \
    src/cartographer/cartographer/sensor/fixed_frame_pose_data.cc \
    src/cartographer/cartographer/sensor/internal/collator.cc \
    src/cartographer/cartographer/sensor/internal/trajectory_collator.cc \
    src/cartographer/cartographer/sensor/internal/ordered_multi_queue.cc \
    src/cartographer/cartographer/sensor/internal/voxel_filter.cc \
    src/cartographer/cartographer/mapping/2d/probability_grid.cc \
    src/cartographer/cartographer/mapping/2d/grid_2d.cc \
    src/cartographer/cartographer/mapping/probability_values.cc \
    src/cartographer/cartographer/mapping/pose_graph.cc \
    src/cartographer/cartographer/mapping/map_builder_interface.cc \
    build_isolated/cartographer/install/cartographer/common/proto/ceres_solver_options.pb.cc \
    build_isolated/cartographer/install/cartographer/sensor/proto/sensor.pb.cc \
    build_isolated/cartographer/install/cartographer/sensor/proto/adaptive_voxel_filter_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/pose_graph.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/connected_components.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/pose_graph_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/serialization.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/trajectory_node_data.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/trajectory_builder_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/submap.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/submap_visualization.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/motion_filter_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/range_data_inserter_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/trajectory.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/map_builder_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/grid_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/tsdf_range_data_inserter_options_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/map_limits.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/cell_limits_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/grid_2d_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/probability_grid.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/submaps_options_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/local_trajectory_builder_options_3d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/tsdf_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/hybrid_grid.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/normal_estimation_options_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/submaps_options_3d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/pose_extrapolator_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/range_data_inserter_options_3d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_3d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/pose_graph/optimization_problem_options.pb.cc \
    build_isolated/cartographer/install/cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.cc \
    build_isolated/cartographer/install/cartographer/transform/proto/transform.pb.cc \
    build_isolated/cartographer/install/cartographer/transform/proto/timestamped_transform.pb.cc

INCLUDEPATH += /usr/include/eigen3 \
    /usr/include/pcl-1.8 \
    /usr/include/vtk-6.3 \
    src/cartographer \
    build_isolated/cartographer/install

HEADERS  += \

LIBS += -lgomp -lpthread
LIBS += -L/usr/local/lib -lyaml-cpp
LIBS += -lceres
LIBS += -lgflags -lglog
LIBS += -llapack -lblas -lcxsparse -lcholmod

LIBS += /usr/lib/x86_64-linux-gnu/libpcl_*.so
LIBS += -lcairo -lboost_system -lboost_iostreams
LIBS += -lprotoc -lprotobuf -llua5.2
LIBS += -labsl

