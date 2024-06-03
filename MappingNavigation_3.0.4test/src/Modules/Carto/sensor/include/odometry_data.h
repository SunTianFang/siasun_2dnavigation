
#ifndef SENSOR_ODOMETRY_DATA_H_
#define SENSOR_ODOMETRY_DATA_H_

#include "mytime.h"

#include "rigid_transform.h"


namespace sensor {

struct OdometryData {
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::Time time;
  transform::Rigid3d pose;
};


struct myOdometryData {
 
  common::Time time;
  transform::myRigid3d pose;
};

// Converts 'odometry_data' to a proto::OdometryData.
//proto::OdometryData ToProto(const OdometryData& odometry_data);

// Converts 'proto' to an OdometryData.
//OdometryData FromProto(const proto::OdometryData& proto);

}  // namespace sensor


#endif  // SENSOR_ODOMETRY_DATA_H_
