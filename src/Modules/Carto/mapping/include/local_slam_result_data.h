
#ifndef SLAM_MAPPING_LOCAL_SLAM_RESULT_DATA_H_
#define SLAM_MAPPING_LOCAL_SLAM_RESULT_DATA_H_

#include "pose_graph.h"
#include "data.h"


namespace mapping {

class LocalSlamResultData : public sensor::Data {
 public:
  LocalSlamResultData(const std::string& sensor_id, common::Time time)
      : Data(sensor_id), time_(time) {}

  common::Time GetTime() const  { return time_; }
  virtual void AddToPoseGraph(int trajectory_id,
                              PoseGraph* pose_graph) const = 0;

 private:
  common::Time time_;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_LOCAL_SLAM_RESULT_DATA_H_
