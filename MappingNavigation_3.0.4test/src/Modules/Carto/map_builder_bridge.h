

#ifndef MAP_BUILDER_BRIDGE_H
#define MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "mutex.h"
#include "map_builder_interface.h"

#include "trajectory_builder_interface.h"


#include "make_unique.h"



#include <sys/time.h>

#include "optional.h"
#include "trajectory_builder_interface.h"
#include "imu_data.h"
#include "odometry_data.h"
#include "rigid_transform.h"
#include "transform.h"
#include "msg_conversion.h"


#include "pose_graph_interface.h"

#include  "submap_2d.h"






struct NodeOptions {
  ::proto::MapBuilderOptions map_builder_options;
  std::string map_frame;
  double createmap_range;
};

struct TrajectoryOptions {
  proto::TrajectoryBuilderOptions
      trajectory_builder_options;

  bool use_odometry;
  //bool use_nav_sat;
  //bool use_landmarks;

  int num_laser_scans;
  int num_multi_echo_laser_scans;
  int num_subdivisions_per_laser_scan;
  int num_point_clouds;

};


struct SubmapEntry
{
  /*SubmapEntry_()
    : trajectory_id(0)
    , submap_index(0)
    , submap_version(0)
    , pose()  {
    }

*/
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   typedef int32_t _trajectory_id_type;
  _trajectory_id_type trajectory_id;

   typedef int32_t _submap_index_type;
  _submap_index_type submap_index;

    int submap_version;
    transform::Rigid3d pose;


};

struct SubmapList
{
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	sensor_msgs::Header   header;
	std::vector<SubmapEntry> submap;
};



typedef  void (*LocalSlamPoseCbFunc)(int *isaddnode,const common::Time *ptime,const transform::Rigid3d *plocal_pose,sensor::RangeData *prangedata,bool *bupdatemap,bool *bfirstframe,mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> *pnodepose,double *prealtimescore);
typedef  void (*RunFinalOptCallBack)(mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> *pnodepose);

class MapBuilderBridge {




 public:

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  struct TrajectoryState {

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	////该结构体存储的是local SLAM处理后的结果。由range_data_in_local中已经估计出了在时刻time时的当前local_pose

    // Contains the trajectory state data received from local SLAM, after
    // it had processed accumulated 'range_data_in_local' and estimated
    // current 'local_pose' at 'time'.
    struct LocalSlamData {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      common::Time time;  //时间
      transform::Rigid3d local_pose; //优化匹配出来的local_pose——在submap这个局部坐标系中的位姿
      sensor::RangeData range_data_in_local; //激光数据
    };
    std::shared_ptr<const LocalSlamData> local_slam_data; //局部SLAM的数据
    transform::Rigid3d local_to_map; //submap到global map的坐标变换关系
    std::unique_ptr<transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options; //配置参数

  };

  MapBuilderBridge(
      const NodeOptions& node_options,
      mapping::MapBuilderInterface *map_builder);




  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;



  int AddTrajectory(
      const std::set<
      mapping::TrajectoryBuilderInterface::SensorId>&
      expected_sensor_ids,
      const TrajectoryOptions& trajectory_options,
      Eigen::Vector2f &submap_initpos,
      bool bExpandMap,
      std::unique_ptr<mapping::Grid2D> gridMap);
  void FinishTrajectory(int trajectory_id);
  void RunFinalOptimization();

  int SaveMap(const string filename, double _metersPerPixel,double creatmaprange);

  int SaveMap(const string filename,std::vector<sensor_msgs::LaserScan> &vtLasers,double _metersPerPixel,double creatmaprange);

  bool SerializeState(const std::string& filename);



  std::set<int> GetFrozenTrajectoryIds();





   std::shared_ptr<const TrajectoryState::LocalSlamData> GetLocalSlamData(int trajectory_id);




  std::unique_ptr<sensor::OdometryData> ToOdometryData(
      const sensor_msgs::OdometryProto *msg);
  void HandleOdometryMessage(const std::string& sensor_id,
                             const sensor_msgs::OdometryProto *msg);


 std::unique_ptr<sensor::ImuData> ToImuData(
      const sensor_msgs::Imu &msg);


  void HandleImuMessage(const std::string& sensor_id,
                        const sensor_msgs::Imu &msg);



 //laser

  void HandleLaserScanMessage(sensor_msgs::LaserScan *msg);


  void HandleLaserScanMessage(sensor::PointCloud &pointclouds);


  void Reset();

  void GetSubmapList(SubmapList &submap_list);

  void GetSubmapData(const mapping::SubmapId submap_id,mapping::SubmapTexture *texture);
  void GetSubmapData_upload(const mapping::SubmapId submap_id, mapping::SubmapTexture_upload *texture, vector<double> &blackcell_index);

  mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> GetNodeData(void);

  void SetSlamPoseCbFunc(LocalSlamPoseCbFunc pFunc){m_plocalposCbFunc = pFunc;}
  void SetSlamPoseCbFunc(RunFinalOptCallBack pFunc){m_pRunFinalOptCbFunc = pFunc;}

  bool ShouldMoveSubmaps();
  void MoveSubmaps(const Eigen::Vector3d& amcl_pose);


  inline void SetUpdateMapFlag(bool bflag){ map_builder_->SetOptimizeFlag(!bflag);m_bupdatemap=bflag; }


  void RegisterBuildOverCallBack(BuildOverFunc pFunc)
  {
      map_builder_->RegisterBuildOverCallBack(pFunc);
  }

    void RegisterOptOverCallBack(OptOverFunc pFunc)
    {
         map_builder_->pose_graph()->RegisterOptOverCallBack(pFunc);
    }

 // bool IsTrajectoryActive(){return is_trajectory_active};


    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> GetTrajectoryNodePoses() ;


    mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapData> GetSubmapDataUnderLock() const;

   std::vector<mapping::PoseGraphInterface::Constraint> GetConstraints() const;
   mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::MySubmapData> GetMySubmapData() const;

    int GetNumFinishedNodes();

    map<int, vector<int>> GetPossibleConstraintPairs();

     void SetInitNodePose(const common::Time time,const transform::Rigid3d &pose);

     void SetFirstSubmap(const transform::Rigid2d &global_submap_pose,std::unique_ptr<mapping::Grid2D> grid);

     void Clear();

     bool LoadBinary(FILE *fp);

     bool SaveBinary(FILE *fp);

     bool RotateMap(double angle);

     void SetFrozen(bool flag);

     int  GetFrozenNodeNum();

     int GetFrozenSubmapNum();

private:

  void HandleLaserScan(const common::Time msgtime,
      const std::string& sensor_id, Eigen::Matrix<double, 3, 1> laser_tf, common::Time start_time,
      const sensor::PointCloudWithIntensities& points);
  void HandleRangefinder(const std::string& sensor_id,
						Eigen::Matrix<double, 3, 1> laser_tf,
						 common::Time msgtime,
                         common::Time time,
                         const ::sensor::TimedPointCloud& ranges);



  void OnLocalSlamResult(
      const int trajectory_id, const common::Time time,
      const transform::Rigid3d local_pose,
      sensor::RangeData range_data_in_local,
      const std::unique_ptr<const ::mapping::
                                TrajectoryBuilderInterface::InsertionResult>
          insertion_result,
          double realtimescore) EXCLUDES(mutex_);

  common::Mutex mutex_;
  const NodeOptions node_options_;
  std::unordered_map<int, std::shared_ptr<TrajectoryState::LocalSlamData>>
      trajectory_state_data_ GUARDED_BY(mutex_);

	std::shared_ptr<TrajectoryState::LocalSlamData>  localSlamData;

  mapping::MapBuilderInterface *map_builder_ ;


	bool is_trajectory_active;
  bool last_trajectory_active;

  // These are keyed with 'trajectory_id'.
  TrajectoryOptions trajectory_options_;
 

  int num_subdivisions_per_laser_scan_;
  std::map<std::string, common::Time> sensor_to_previous_subdivision_time_;

 
   LocalSlamPoseCbFunc m_plocalposCbFunc;

   RunFinalOptCallBack m_pRunFinalOptCbFunc;

	bool m_printfwarn;
  
	bool m_bupdatemap; 

        std::vector<sensor_msgs::LaserScan> vt_laser_msg;

};



#endif  // MAP_BUILDER_BRIDGE_H
