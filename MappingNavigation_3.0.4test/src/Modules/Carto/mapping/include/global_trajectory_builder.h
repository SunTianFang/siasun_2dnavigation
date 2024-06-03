

#ifndef SLAM_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_
#define SLAM_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "local_trajectory_builder_2d.h"
#include "pose_graph_2d.h"

#include "local_slam_result_data.h"
#include "trajectory_builder_interface.h"



namespace mapping {

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback);



}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_
