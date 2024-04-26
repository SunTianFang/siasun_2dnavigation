
#ifndef MAPPING_DATA_H_
#define MAPPING_DATA_H_

#include "make_unique.h"
#include "mytime.h"
#include "rigid_transform.h"


namespace mapping {
class TrajectoryBuilderInterface;
}

namespace sensor {

class Data {
 public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
  virtual ~Data() {}

  virtual common::Time GetTime() const = 0;
  const std::string &GetSensorId() const { return sensor_id_; }
  virtual void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *trajectory_builder) = 0;

 protected:
  const std::string sensor_id_;
};

}  // namespace sensor


#endif  // MAPPING_DATA_H_
