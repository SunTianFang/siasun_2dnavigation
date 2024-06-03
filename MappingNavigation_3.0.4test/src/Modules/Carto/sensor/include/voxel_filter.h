
#ifndef SENSOR_INTERNAL_VOXEL_FILTER_H_
#define SENSOR_INTERNAL_VOXEL_FILTER_H_

#include <bitset>
#include <unordered_set>

#include "point_cloud.h"
#include "timed_point_cloud_data.h"
#include "sensorproto.h"




namespace sensor {

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
class VoxelFilter {
 public:

  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 'size' is the length of a voxel edge.
  explicit VoxelFilter(float size) : resolution_(size) {}

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;

  // Returns a voxel filtered copy of 'point_cloud'.
  PointCloud Filter(const PointCloud& point_cloud);

  // Same for TimedPointCloud.
  TimedPointCloud Filter(const TimedPointCloud& timed_point_cloud);

  // Same for RangeMeasurement.
  std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> Filter(
      const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>&
          range_measurements);

 private:
  typedef std::bitset<3 * 32>  KeyType ;

  static KeyType IndexToKey(const Eigen::Array3i& index);

  Eigen::Array3i GetCellIndex(const Eigen::Vector3f& point) const;

  float resolution_;
  std::unordered_set<KeyType> voxel_set_;
};


class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const proto::AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor


#endif  // SENSOR_INTERNAL_VOXEL_FILTER_H_
