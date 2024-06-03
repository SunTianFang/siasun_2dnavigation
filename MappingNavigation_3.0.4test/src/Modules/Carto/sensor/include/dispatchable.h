
#ifndef SENSOR_INTERNAL_DISPATCHABLE_H_
#define SENSOR_INTERNAL_DISPATCHABLE_H_

#include "trajectory_builder_interface.h"
#include "data.h"


namespace sensor {


template <typename DataType>
class Dispatchable : public Data {
 public:
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Dispatchable(const std::string &sensor_id, const DataType &data)
      : Data(sensor_id), data_(data) {}

  common::Time GetTime() const  { return data_.time; }
  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *const trajectory_builder)  {
    trajectory_builder->AddSensorData(sensor_id_, data_);
  }
  const DataType &data() const { return data_; }

 private:
  const DataType data_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
  return common::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor


#endif  // SENSOR_INTERNAL_DISPATCHABLE_H_
