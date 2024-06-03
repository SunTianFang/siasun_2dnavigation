

#ifndef SLAM_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
#define SLAM_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_

#include <memory>

#include "make_unique.h"
#include "timed_point_cloud_data.h"
#include <map>

#include <set>



namespace mapping {

using namespace std;

// Synchronizes TimedPointCloudData from different sensors. Input needs only be
// monotonous in 'TimedPointCloudData::time', output is monotonous in per-point
// timing. Up to one message per sensor is buffered, so a delay of the period of
// the slowest sensor may be introduced, which can be alleviated by passing
// subdivisions.
class RangeDataCollator {
 public:

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit RangeDataCollator(
      const std::vector<std::string>& expected_range_sensor_ids)
      : expected_sensor_ids_(expected_range_sensor_ids.begin(),
                             expected_range_sensor_ids.end()) {current_start_ = common::Time::min();
  current_end_ = common::Time::min();}

  sensor::TimedPointCloudOriginData AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data);

 private:
  sensor::TimedPointCloudOriginData CropAndMerge();

  const std::set<std::string> expected_sensor_ids_;
  // Store at most one message for each sensor.
  std::map<std::string, sensor::TimedPointCloudData> id_to_pending_data_;
  common::Time current_start_;
  common::Time current_end_ ;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
