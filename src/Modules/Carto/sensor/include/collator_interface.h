
#ifndef SENSOR_COLLATOR_INTERFACE_H_
#define SENSOR_COLLATOR_INTERFACE_H_

#include <functional>
#include <memory>
#include <unordered_set>
#include <vector>


#include "data.h"


namespace sensor {

class CollatorInterface {
 public:
  typedef std::function<void(const std::string&, std::unique_ptr<Data>)> Callback;

 // typedef void (*Callback)(const std::string&, std::unique_ptr<Data>);

  CollatorInterface() {}
  virtual ~CollatorInterface() {}
  CollatorInterface(const CollatorInterface&) = delete;
  CollatorInterface& operator=(const CollatorInterface&) = delete;

  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  virtual void AddTrajectory(
      int trajectory_id,
      const std::unordered_set<std::string>& expected_sensor_ids,
       const Callback& callback) = 0;

  // Marks 'trajectory_id' as finished.
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'data.sensor_id_' must be added
  // in time order.
  virtual void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) = 0;

  // Dispatches all queued sensor packets. May only be called once.
  // AddSensorData may not be called after Flush.
  virtual void Flush() = 0;


};

}  // namespace sensor


#endif  // SENSOR_COLLATOR_INTERFACE_H_
