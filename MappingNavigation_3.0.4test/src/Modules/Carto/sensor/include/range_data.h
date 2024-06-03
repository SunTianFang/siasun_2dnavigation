

#ifndef SENSOR_RANGE_DATA_H_
#define SENSOR_RANGE_DATA_H_

#include "port.h"

#include "point_cloud.h"



namespace sensor {

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
struct RangeData {

  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;
};

// Like 'RangeData', but with 'TimedPointClouds'.
struct TimedRangeData {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f origin;
  TimedPointCloud returns;
  TimedPointCloud misses;
};

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);

TimedRangeData TransformTimedRangeData(const TimedRangeData& range_data,
                                       const transform::Rigid3f& transform);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
TimedRangeData CropTimedRangeData(const TimedRangeData& range_data, float min_z,
                                  float max_z);

// Converts 'range_data' to a proto::RangeData.
//proto::RangeData ToProto(const RangeData& range_data);

// Converts 'proto' to RangeData.
//RangeData FromProto(const proto::RangeData& proto);

}  // namespace sensor


#endif  // SENSOR_RANGE_DATA_H_
