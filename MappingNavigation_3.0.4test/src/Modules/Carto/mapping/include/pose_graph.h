

#ifndef SLAM_MAPPING_POSE_GRAPH_H_
#define SLAM_MAPPING_POSE_GRAPH_H_

#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>


#include "id.h"
#include "pose_graph_interface.h"
#include "pose_graph_trimmer.h"



#include "options.h"
#include "submaps.h"
#include "trajectory_node.h"

#include "imu_data.h"
#include "map_by_time.h"
#include "odometry_data.h"


namespace mapping {



class PoseGraph : public PoseGraphInterface {
 public:

    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct InitialTrajectoryPose {
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int to_trajectory_id;
    transform::Rigid3d relative_pose;
    common::Time time;
  };

  PoseGraph() {}
  ~PoseGraph()  {}

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;

  // Inserts an IMU measurement.
  virtual void AddImuData(int trajectory_id,
                          const sensor::ImuData& imu_data) = 0;

  // Inserts an odometry measurement.
  virtual void AddOdometryData(int trajectory_id,
                               const sensor::myOdometryData& odometry_data) = 0;




  // Finishes the given trajectory.
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Freezes a trajectory. Poses in this trajectory will not be optimized.
  virtual void FreezeTrajectory(int trajectory_id) = 0;




  // Adds information that 'node_id' was inserted into 'submap_id'. The submap
  // has to be deserialized first.
  virtual void AddNodeToSubmap(const NodeId& node_id,
                               const SubmapId& submap_id) = 0;

  // Adds serialized constraints. The corresponding trajectory nodes and submaps
  // have to be deserialized before calling this function.
  virtual void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) = 0;

  // Adds a 'trimmer'. It will be used after all data added before it has been
  // included in the pose graph.
  virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;


  // Returns the current optimized transform and submap itself for the given
  // 'submap_id'. Returns 'nullptr' for the 'submap' member if the submap does
  // not exist (anymore).
  virtual SubmapData GetSubmapData(const SubmapId& submap_id) const = 0;

 // proto::PoseGraph ToProto() const ;

  // Returns the IMU data.
  virtual sensor::MapByTime<sensor::ImuData> GetImuData() const = 0;

  // Returns the odometry data.

  //virtual sensor::MapByTime<sensor::OdometryData> GetOdometryData() const = 0;

  virtual int SaveMap(const string filename,double _metersPerPixel, std::vector<sensor_msgs::LaserScan> &vt_laser_msg,double creatmaprange)  = 0;
 

   virtual MapById<SubmapId, PoseGraphInterface::SubmapData> GetSubmapDataUnderLock()const = 0;

  virtual transform::Rigid2d GetLatestNodePose() const = 0;

   virtual  void RegisterBuildOverCallBack(BuildOverFunc pFunc) = 0;

  virtual void RegisterOptOverCallBack(OptOverFunc pFunc) = 0;

   virtual int AddTrajectoryBuilder() = 0;
  
      
   virtual void SetExpandMapFlag(bool bExpand) = 0;

   virtual void  SetFirstSubmap(const transform::Rigid2d &global_submap_pose,std::unique_ptr<mapping::Grid2D> grid) = 0;

    virtual void Clear() = 0;

    virtual bool  LoadBinary(FILE *fp) = 0;
    virtual bool  SaveBinary(FILE *fp) = 0;

    virtual bool RotateMap(double angle) = 0;
    virtual void SetFrozen(bool flag) =0;

    virtual int GetFrozenNodeNum() = 0;

    virtual int GetFrozenSubmapNum() = 0;
};




}  // namespace mapping


#endif  // SLAM_MAPPING_POSE_GRAPH_H_
