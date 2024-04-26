
#ifndef SLAM_MAPPING_MAP_BUILDER_H_
#define SLAM_MAPPING_MAP_BUILDER_H_



#include <memory>

#include "thread_pool.h"
#include "pose_graph.h"
#include "options.h"
#include "collator_interface.h"


#include <map>
#include <set>
#include <string>


#include "port.h"
#include "rate_timer.h"
#include "local_slam_result_data.h"

#include "submaps.h"
#include "trajectory_builder_interface.h"
#include "collator_interface.h"
#include "dispatchable.h"
#include "grid_2d.h"

#include "map_builder_interface.h"
#include "pose_graph_2d.h"


namespace mapping {



proto::MapBuilderOptions CreateMapBuilderOptions(LaserNaviConfig config);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class MapBuilder : public MapBuilderInterface {
 public:
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder()  {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback,
      Eigen::Vector2f &submap_initpos,bool bExpandMap,
      std::unique_ptr<mapping::Grid2D> gridMap) ;

 
  void FinishTrajectory(int trajectory_id) ;



 /* void SerializeState(io::ProtoStreamWriterInterface *writer) ;

  void LoadState(io::ProtoStreamReaderInterface *reader,
                 bool load_frozen_state) override;


 int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) ;

*/

  mapping::PoseGraphInterface *pose_graph()  {
    return pose_graph_.get();
  }


  transform::Rigid2d GetLatestNodePose() {
    LOG(FATAL) << "GetLatestNodePose() called from map_builder. Not supported here.";
    return transform::Rigid2d::Identity(); // just to keep the compiler happy
  }

  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data)
  {



 //   AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
    wrapped_trajectory_builder_->AddSensorData(sensor_id,timed_point_cloud_data);

  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data)  {

  //  AddData(sensor::MakeDispatchable(sensor_id, imu_data));

    wrapped_trajectory_builder_->AddSensorData(sensor_id,imu_data);


  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data)  {

    //AddData(sensor::MakeDispatchable(sensor_id, odometry_data));
     wrapped_trajectory_builder_->AddSensorData(sensor_id,odometry_data);
  }

  // add by lishen
  void SetInitNodePose(const common::Time time,const transform::Rigid3d &pose)
  {

      wrapped_trajectory_builder_->SetInitNodePose(time,pose);
  }

  void GetSubmapData(const SubmapId submap_id,SubmapTexture *texture);
  void GetSubmapData_upload(const SubmapId submap_id, SubmapTexture_upload *texture, vector<double>&blackcell_index);

  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const;

  MapById<SubmapId, PoseGraphInterface::SubmapData>GetSubmapDataUnderLock() const;

  // add by lishen
  void RegisterBuildOverCallBack(BuildOverFunc pFunc)
  {
      pose_graph_->RegisterBuildOverCallBack(pFunc);
  }

    // add by lishen
   void Reset();

   // add by lishen
   int GetLatestSubmapNumRangeData();
 
   // add by lishen
   PoseGraphInterface::SubmapData GetLatestSubmapData();
   // add by lishen
   inline void SetOptimizeFlag(bool bflag){pose_graph_->SetOptimizeFlag(bflag);}

   // add by lishen
   MapById<SubmapId, PoseGraphInterface::MySubmapData> GetMySubmapData() const;

   // add by lishen
   std::vector<PoseGraphInterface::Constraint> GetConstraints() const;
   // add by lishen
   int GetNumFinishedNodes();

   // add by lishen
   void ClearSubmapAndNode();
   bool RotateMap(double angle);
  
 private:



  void AddData(std::unique_ptr<sensor::Data> data);

   void HandleCollatedSensorData(const std::string& sensor_id,
                                std::unique_ptr<sensor::Data> data);

  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<PoseGraph> pose_graph_;

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
 

  //int trajectory_id_;
  std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_;

  // Time at which we last logged the rates of incoming sensor data.
  common::Time last_logging_time_;
  std::map<std::string, common::RateTimer> rate_timers_;


};

}  // namespace mapping


#endif  // SLAM_MAPPING_MAP_BUILDER_H_
