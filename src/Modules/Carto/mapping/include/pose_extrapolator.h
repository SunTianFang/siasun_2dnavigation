

#ifndef SLAM_MAPPING_POSE_EXTRAPOLATOR_H_
#define SLAM_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "mytime.h"
#include "imu_tracker.h"
#include "imu_data.h"
#include "odometry_data.h"
#include "rigid_transform.h"


namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
class PoseExtrapolator {
 public:

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit PoseExtrapolator(const double  pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      const double pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const;
  common::Time GetLastExtrapolatedTime() const;

  void AddPose(common::Time time, const transform::Rigid3d& pose);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData( sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time, int type);
  transform::Rigid3d ExtrapolatePose(common::Time time);
  // Gravity alignment estimate.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time);



  // add by lishen
  void AddPose_new(const common::Time time,
                                 const transform::Rigid3d& pose);
  // add by lishen
  void SetPose_new(const common::Time time,
                                 const transform::Rigid3d& pose);

  // add by lishen
  transform::Rigid3d ExtrapolatePose_new(const common::Time time);


  // add by lishen
  void AddOdometryData_new(sensor::OdometryData& odometry_data);

  // add by lishen
  Eigen::Quaterniond EstimateGravityOrientation_new(common::Time time);






 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time , int type);
  Eigen::Vector3d ExtrapolateTranslation(common::Time time );






  const double pose_queue_duration_;
  struct TimedPose {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    common::Time time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose,Eigen::aligned_allocator<Eigen::Quaterniond>> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ ;
  Eigen::Vector3d angular_velocity_from_poses_ ;

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData ,Eigen::aligned_allocator<Eigen::Quaterniond>> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ ;
  Eigen::Vector3d angular_velocity_from_odometry_ ;



  //lishen
  transform::Rigid3d current_pose;


};

}  // namespace mapping


#endif  // SLAM_MAPPING_POSE_EXTRAPOLATOR_H_
