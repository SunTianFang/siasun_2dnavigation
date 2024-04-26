
#ifndef NODE_H
#define NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "fixed_ratio_sampler.h"
#include "mutex.h"
#include "map_builder_interface.h"
#include "pose_extrapolator.h"
#include "map_builder_bridge.h"



#include  "submap_2d.h"



class Node {
 public:
  Node(const NodeOptions& node_options,
       ::mapping::MapBuilderInterface *map_builder);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;



  // Finishes a single given trajectory. Returns false if the trajectory did not
  // exist or was already finished.
  bool FinishTrajectory(int trajectory_id);

  // Runs final optimization. All trajectories have to be finished when calling.
  void RunFinalOptimization();


  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const sensor_msgs::OdometryProto& msg);


  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::Imu &msg);

  void HandleLaserScanMessage(bool bUpdatemap,
                               sensor_msgs::LaserScan  &msg,transform::Rigid2d locpos,common::Time loctime,transform::Rigid2d &offset);


  void HandleLaserScanMessage(sensor::PointCloud &pointclouds);

  void GetSubmapList(SubmapList &submap_list) ;
  void GetSubmapData(const mapping::SubmapId submap_id,mapping::SubmapTexture *texture) ;
  void GetSubmapData_upload(const ::mapping::SubmapId submap_id, ::mapping::SubmapTexture_upload *texture, vector<double> &blackcell_index);

  // add by lishen
  //因时间 精力有限 没研究ros是如何在界面显示并保存的，因此自己写的保存地图，因此保存的格式与ros不同
  int SaveMap(const string filename, double _metersPerPixel,double creatmaprange);
  int SaveMap(const string filename,std::vector<sensor_msgs::LaserScan> &vtLasers,double _metersPerPixel,double creatmaprange);

   // add by lishen
  // 得到slam结果，包括位姿，时间戳等
  std::shared_ptr<const MapBuilderBridge::TrajectoryState::LocalSlamData> GetLocalSlamData(int trajectory_id);

  // add by lishen
  //对原有AddTrajectory修改
  int AddTrajectory(const TrajectoryOptions& options,Eigen::Vector2f &submap_initpos,bool bExpandMap,std::unique_ptr<mapping::Grid2D> gridMap);

  // add by lishen
  bool IsTrajectoryActive(){return is_active_trajectory_;}

  // add by lishen
  //资源 回收，内存释放，原ros是一个进程，未看到有资源回收。作为一个线程，为防止内存泄漏，对子图、节点、线程池、约束等等一一进行内存释放
  void Reset();
  // add by lishen
  bool IsInit() {return is_init_ready;}

 // add by lishen
  void SetSlamPoseCbFunc(LocalSlamPoseCbFunc pFunc){  map_builder_bridge_.SetSlamPoseCbFunc(pFunc);}
  // add by lishen
  void RegisterBuildOverCallBack(BuildOverFunc pFunc)
  {
      map_builder_bridge_.RegisterBuildOverCallBack(pFunc);
  }

  void RegisterOptOverCallBack(OptOverFunc pFunc)
  {
         map_builder_bridge_.RegisterOptOverCallBack(pFunc);
  }


  mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> GetNodeData(void);

  // add by lishen
  void GetConstraint();

  // add by lishen
   void AddLaserID(std::string laser_id);


   mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapData> GetSubmapDataUnderLock() const;
  // add by lishen
   std::vector<mapping::PoseGraphInterface::Constraint> GetConstraints() const;

   mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> GetTrajectoryNodePoses() ;


   mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::MySubmapData> GetMySubmapData() const;

   // add by lishen
   int GetNumFinishedNodes();
   // add by lishen
   map<int, vector<int>> GetPossibleConstraintPairs();
   // add by lishen
   void SetInitNodePose(const common::Time time,const transform::Rigid3d &pose);
   // add by lishen
   void SetFirstSubmap(const transform::Rigid2d &global_submap_pose,std::unique_ptr<mapping::Grid2D> grid);
   // add by lishen
   void Clear();

   bool LoadBinary(FILE *fp);

   bool SaveBinary(FILE *fp);

   bool RotateMap(double angle);

   void SetFrozen(bool flag);

   int  GetFrozenNodeNum();

   int GetFrozenSubmapNum();

 private:



  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
 
  bool FinishTrajectoryUnderLock(int trajectory_id) ;

  const NodeOptions node_options_;

 
  common::Mutex mutex_;
  MapBuilderBridge map_builder_bridge_;


  bool is_active_trajectory_ ;

  bool is_init_ready;

  vector<std::string> vtLaserID;

};


#endif  // CNODE_H
