
#ifndef SLAM_MAPPING_TRAJECTORY_NODE_H_
#define SLAM_MAPPING_TRAJECTORY_NODE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "optional.h"
#include "mytime.h"

#include "range_data.h"
#include "rigid_transform.h"

#include "timed_point_cloud_data.h"

namespace mapping {

struct TrajectoryNodePose {

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct ConstantPoseData {

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    common::Time time;
    transform::Rigid3d local_pose;
  };
  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;

  common::optional<ConstantPoseData> constant_pose_data;
};

struct TrajectoryNode {

	
  struct Data {

	
    common::Time time;

    // Transform to approximately gravity align the tracking frame as
    // determined by local SLAM.
    Eigen::Quaternion<double,Eigen::DontAlign> gravity_alignment;

    // Used for loop closure in 2D: voxel filtered returns in the
    // 'gravity_alignment' frame.
    sensor::PointCloud filtered_gravity_aligned_point_cloud;

    // Used for loop closure in 3D.
    sensor::PointCloud high_resolution_point_cloud;
    sensor::PointCloud low_resolution_point_cloud;
    Eigen::VectorXf rotational_scan_matcher_histogram;

   


    // The node pose in the local SLAM frame.
    transform::Rigid3d local_pose;
    transform::Rigid3d local_pose_() const { return local_pose; }

	// sensor::RangeData synchronized_range_datas;
  };
  transform::Rigid3d global_pose_() const { return global_pose; }
  common::Time time() const { return constant_data->time; }

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  std::shared_ptr<Data> constant_data;

  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;
};



}  // namespace mapping


#endif  // SLAM_MAPPING_TRAJECTORY_NODE_H_
