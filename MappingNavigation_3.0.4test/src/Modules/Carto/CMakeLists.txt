set ( LIBRARY_OUTPUT_PATH ../../../release )

#RK3399
if(PLATFORM_RK3399)
    add_compile_definitions(_RK3399_ARM_64)
    add_compile_definitions(NAV_APP)
endif()


#E3845
if(PLATFORM_E3845)
    add_compile_definitions(_E3845_LINUX64)
    add_compile_definitions(_LINUX6)
    add_compile_definitions(_SERVICE_HARDWARE)
    add_compile_definitions(NAV_APP)
endif()



#MRC3588
if(PLATFORM_MRC3588)
    add_compile_definitions(_RK3588_LINUX64)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)


endif()

#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)


endif()

include_directories( .
        common/include
        transform/include
        sensor/include
        mapping/include
        myceres/include
        ../ThirdParty/eigen3/
        ../Tools/include
        ../ThirdParty/jsoncpp/include

        )

set ( Carto_SRCS
	common/src/mymath.cpp
	common/src/time.cpp
	mapping/src/correlative_scan_matcher_2d.cpp
	mapping/src/fast_correlative_scan_matcher_2d.cpp
	mapping/src/grid_2d.cpp
	mapping/src/gridmap.cpp
	mapping/src/probability_grid_range_data_inserter_2d.cpp
	mapping/src/probability_grid.cpp
	mapping/src/probability_values.cpp
	mapping/src/ray_casting.cpp
	sensor/src/point_cloud.cpp
	sensor/src/range_data.cpp
	transform/src/rigid_transform.cpp
	transform/src/timestamped_transform.cpp
	myceres/src/block_sparse_matrix.cc
	myceres/src/block_structure.cc
	myceres/src/dense_sparse_matrix.cc
	myceres/src/location.cpp
	myceres/src/trust_region_step_evaluator.cc
	mapping/src/ceres_scan_matcher_2d.cpp
	common/src/thread_pool.cpp
	common/src/fixed_ratio_sampler.cpp
	mapping/src/submap_2d.cpp
	mapping/src/range_data_collator.cpp
	mapping/src/pose_graph_trimmer.cpp
	mapping/src/pose_graph_2d.cpp
	mapping/src/pose_extrapolator.cpp
	mapping/src/optimization_problem_2d.cpp
	mapping/src/motion_filter.cpp
	mapping/src/map_builder.cpp
	mapping/src/local_trajectory_builder_2d.cpp
	mapping/src/imu_tracker.cpp
	mapping/src/global_trajectory_builder.cpp
	mapping/src/constraint_builder_2d.cpp
	sensor/src/voxel_filter.cpp
	sensor/src/ordered_multi_queue.cpp
	sensor/src/collator.cpp
	myceres/src/optimize_pose_graph.cpp
	common/src/task.cpp
	transform/src/transform_interpolation_buffer.cpp
	node.cpp
	navigation.cpp
	map_builder_bridge.cpp
	NaviInterface.cpp
	mapping/src/real_time_correlative_scan_matcher_2d.cpp
	common/src/mylogging.cpp
	)




add_library ( Carto SHARED ${Carto_SRCS}  )

