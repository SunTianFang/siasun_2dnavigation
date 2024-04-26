

#ifndef SENSOR_INTERNAL_COLLATOR_H_
#define SENSOR_INTERNAL_COLLATOR_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "collator_interface.h"
#include "data.h"
#include "ordered_multi_queue.h"


namespace sensor {

class Collator : public CollatorInterface {
 public:

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

  void AddTrajectory(int trajectory_id,
                     const std::unordered_set<std::string>& expected_sensor_ids,
                     const Callback& callback) ;

  void FinishTrajectory(int trajectory_id) ;

  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) ;

  void Flush() ;


 private:
  // Queue keys are a pair of trajectory ID and sensor identifier.
  OrderedMultiQueue queue_;

  // Map of trajectory ID to all associated QueueKeys.
  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor


#endif  // SENSOR_INTERNAL_COLLATOR_H_
