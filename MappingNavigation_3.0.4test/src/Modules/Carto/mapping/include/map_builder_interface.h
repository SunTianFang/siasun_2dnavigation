
#ifndef SLAM_MAPPING_MAP_BUILDER_INTERFACE_H_
#define SLAM_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "port.h"

#include "id.h"
#include "pose_graph_interface.h"

#include "options.h"
#include "submaps.h"
#include "trajectory_builder_interface.h"
#include "submap_2d.h"
#include "type.h"



namespace mapping {

// This interface is used for both library and RPC implementations.
// Implementations wire up the complete SLAM stack.
class MapBuilderInterface {
 public:
  typedef TrajectoryBuilderInterface::LocalSlamResultCallback LocalSlamResultCallback;

  typedef TrajectoryBuilderInterface::SensorId SensorId;

  MapBuilderInterface() {}
  virtual ~MapBuilderInterface() {}

  MapBuilderInterface(const MapBuilderInterface&) = delete;
  MapBuilderInterface& operator=(const MapBuilderInterface&) = delete;


  virtual int AddTrajectoryBuilder(
      const std::set<SensorId>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options,
      LocalSlamResultCallback local_slam_result_callback,
      Eigen::Vector2f &submap_initpos,bool bExpandMap,
     std::unique_ptr<mapping::Grid2D> gridMap) = 0;


  virtual void FinishTrajectory(int trajectory_id) = 0;

  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::OdometryData& odometry_data) = 0;


  virtual mapping::PoseGraphInterface* pose_graph() = 0;
 
  virtual transform::Rigid2d GetLatestNodePose() = 0;

  virtual void GetSubmapData(const SubmapId submap_id,SubmapTexture *texture)=0;
  virtual void GetSubmapData_upload(const SubmapId submap_id,SubmapTexture_upload *texture, vector<double>& blackcell_index)=0;

  //add by lishen
  virtual void Reset() = 0;
  virtual void SetOptimizeFlag(bool bflag) = 0;

  virtual   MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const=0;
  virtual   MapById<SubmapId, PoseGraphInterface::SubmapData>GetSubmapDataUnderLock() const=0;

  virtual   std::vector<PoseGraphInterface::Constraint> GetConstraints() const=0;

  virtual MapById<SubmapId, PoseGraphInterface::MySubmapData> GetMySubmapData() const = 0;

  virtual int GetNumFinishedNodes() = 0;

  virtual  void RegisterBuildOverCallBack(BuildOverFunc pFunc) = 0;

  virtual  void SetInitNodePose(const common::Time time,const transform::Rigid3d &pose) = 0;
  virtual void ClearSubmapAndNode() = 0;

   virtual bool RotateMap(double angle)=0;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_MAP_BUILDER_INTERFACE_H_
