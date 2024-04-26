
#ifndef SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "mytime.h"
#include "point_cloud.h"

#include<Eigen/StdVector>

namespace sensor {

struct TimedPointCloudData {
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  common::Time msgtime;
  common::Time time;
  Eigen::Vector3f origin;
  sensor::TimedPointCloud ranges;
};

struct TimedPointCloudOriginData {
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct RangeMeasurement {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector4f point_time;
    size_t origin_index;
  };
  common::Time time;
  std::vector<Eigen::Vector3f> origins;

  std::vector<RangeMeasurement,Eigen::aligned_allocator<Eigen::Vector4f>> ranges;
};


}  // namespace sensor


#endif  // SENSOR_TIMED_POINT_CLOUD_DATA_H_
