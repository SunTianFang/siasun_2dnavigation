
#ifndef SENSOR_IMU_DATA_H_
#define SENSOR_IMU_DATA_H_

#include "Eigen/Core"
#include "mytime.h"



namespace sensor {

struct ImuData {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

// Converts 'imu_data' to a proto::ImuData.
//proto::ImuData ToProto(const ImuData& imu_data);

// Converts 'proto' to an ImuData.
//ImuData FromProto(const proto::ImuData& proto);

}  // namespace sensor


#endif  // SENSOR_IMU_DATA_H_
