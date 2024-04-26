

#ifndef SLAM_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
#define SLAM_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>


#include "make_unique.h"
#include "port.h"
#include "mytime.h"

#include "submaps.h"
#include "pose_graph_interface.h"

#include "imu_data.h"

#include "odometry_data.h"
#include "timed_point_cloud_data.h"
#include "options.h"



namespace mapping {

//proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
 //   common::LuaParameterDictionary* const parameter_dictionary);

class LocalSlamResultData;

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
class TrajectoryBuilderInterface {
 public:


  struct InsertionResult {

    NodeId node_id;
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  typedef  
      std::function<void(int /* trajectory ID */, common::Time,
                         transform::Rigid3d /* local pose estimate */,
                         sensor::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>,
                         double)> LocalSlamResultCallback;

  struct SensorId {
    enum class SensorType {
      RANGE = 0,
      IMU,
      ODOMETRY,
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };

    SensorType type;
    std::string id;

    bool operator==(const SensorId& other) const {
      return std::forward_as_tuple(type, id) ==
             std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId& other) const {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

  TrajectoryBuilderInterface() {}
  virtual ~TrajectoryBuilderInterface() {}

  TrajectoryBuilderInterface(const TrajectoryBuilderInterface&) = delete;
  TrajectoryBuilderInterface& operator=(const TrajectoryBuilderInterface&) =
      delete;

  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::OdometryData& odometry_data) = 0;

//  virtual void AddSensorData(const std::string& sensor_id,
  //                           const sensor::LandmarkData& landmark_data) = 0;
  // Allows to directly add local SLAM results to the 'PoseGraph'. Note that it
  // is invalid to add local SLAM results for a trajectory that has a
  // 'LocalTrajectoryBuilder2D/3D'.
  virtual void AddLocalSlamResultData(
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;

  virtual void SetLatestAmclPose(transform::Rigid2d latest_tf) = 0;

  virtual int GetLatestSubmapNumRangeData() const = 0;

  virtual int GetNumActiveSubmaps() const = 0;

  virtual void MoveAllSubmaps(transform::Rigid2d amcl_pose) = 0;

  virtual PoseGraphInterface::SubmapData GetLatestSubmapData() = 0;
  virtual   void SetInitNodePose(const common::Time time,const transform::Rigid3d &pose) = 0;
};



}  // namespace mapping


#endif  // SLAM_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
