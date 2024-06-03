
#ifndef SLAM_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define SLAM_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "mytime.h"
#include "submap_2d.h"
#include "ceres_scan_matcher_2d.h"
#include "real_time_correlative_scan_matcher_2d.h"
#include "motion_filter.h"
#include "range_data_collator.h"
#include "pose_extrapolator.h"

#include "imu_data.h"
#include "voxel_filter.h"
#include "odometry_data.h"
#include "range_data.h"
#include "rigid_transform.h"





namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
class LocalTrajectoryBuilder2D {
 public:

//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct InsertionResult {
	
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::shared_ptr<TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<Submap2D>> insertion_submaps;
  };
  struct MatchingResult {

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
   std::shared_ptr<InsertionResult> insertion_result;
   bool in_corridor;
   double realtimescore;

  };

  explicit LocalTrajectoryBuilder2D(
      const proto::LocalTrajectoryBuilderOptions2D& options,
      const std::vector<std::string>& expected_range_sensor_ids,Eigen::Vector2f &initpos,
      std::unique_ptr<mapping::Grid2D> gridMap);
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::myOdometryData& odometry_data);

  // add by lishen
  void SetInitNodePose(const common::Time time,const transform::Rigid3d &pose);

  ActiveSubmaps2D& GetActiveSubmaps();

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(const common::Time msgtime,
      common::Time time, const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& gravity_alignment,sensor::RangeData& synchronized_range_datas);
  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;

  std::unique_ptr<InsertionResult> InsertIntoSubmap(const common::Time msgtime,
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment, sensor::RangeData& synchronized_range_datas);

  // Scan matches 'gravity_aligned_range_data' and returns the observed pose,
  // or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::RangeData& gravity_aligned_range_data,bool &in_corridor, double& realtimescore);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  const proto::LocalTrajectoryBuilderOptions2D options_;
  ActiveSubmaps2D active_submaps_;

  MotionFilter motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  std::unique_ptr<PoseExtrapolator> extrapolator_;


  int num_accumulated_ ;
  sensor::RangeData accumulated_range_data_;

  sensor::RangeData synchronized_range_datas;

  common::Time accumulation_started_;

  RangeDataCollator range_data_collator_;

  bool first_frame;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
