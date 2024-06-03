

#ifndef SLAM_MAPPING_INTERNAL_MOTION_FILTER_H_
#define SLAM_MAPPING_INTERNAL_MOTION_FILTER_H_

#include <limits>


#include "mytime.h"

#include "rigid_transform.h"
#include "options.h"


namespace mapping {



// Takes poses as input and filters them to get fewer poses.
class MotionFilter {

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  explicit MotionFilter(const proto::MotionFilterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(common::Time time, const transform::Rigid3d& pose);

 private:
  const proto::MotionFilterOptions options_;
  int num_total_ ;
  int num_different_ ;
  common::Time last_time_;
  transform::Rigid3d last_pose_;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_MOTION_FILTER_H_
