
#ifndef SLAM_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
#define SLAM_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "fixed_ratio_sampler.h"
#include "mutex.h"
#include "thread_pool.h"
#include "mytime.h"
#include "submap_2d.h"
#include "constraint_builder_2d.h"
#include "optimization_problem_2d.h"

#include "pose_graph.h"
#include "pose_graph_trimmer.h"


#include "odometry_data.h"
#include "point_cloud.h"
#include "rigid_transform.h"
#include "transform.h"

//dq
#include <queue>


namespace mapping {


//dq 减少匹配计算量
struct submap_sort
{
      double distance;
      int trajectory_id;
      int index; //submap or node

      submap_sort(double distance_value, int trajectory_id_value, int index_value)
      {
              distance = distance_value;
              trajectory_id = trajectory_id_value;
              index = index_value;
      }

      bool operator<(const submap_sort& other) const
      {
              return distance > other.distance;
      }
};
// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.

void LaserScanToPointCloudWithIntensities(const sensor_msgs::LaserScan &msg ,sensor::PointCloudWithIntensities &point_cloud ,common::Time &timestamp);

class PoseGraph2D : public PoseGraph {




 public:

//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseGraph2D(
      const proto::PoseGraphOptions& options,
      std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
      common::ThreadPool* thread_pool);
  ~PoseGraph2D() ;

  PoseGraph2D(const PoseGraph2D&) = delete;
  PoseGraph2D& operator=(const PoseGraph2D&) = delete;

  // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
  // determined by scan matching against 'insertion_submaps.front()' and the
  // node data was inserted into the 'insertion_submaps'. If
  // 'insertion_submaps.front().finished()' is 'true', data was inserted into
  // this submap for the last time.
  NodeId AddNode(
      std::shared_ptr<TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<Submap2D>>& insertion_submaps,bool in_corridor);

  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) ;
  void AddOdometryData(int trajectory_id,
                       const sensor::myOdometryData& odometry_data) ;


  void FinishTrajectory(int trajectory_id) ;
  bool IsTrajectoryFinished(int trajectory_id) const  ;
  void FreezeTrajectory(int trajectory_id) ;
  bool IsTrajectoryFrozen(int trajectory_id) const ;
 
 
  void AddNodeToSubmap(const NodeId& node_id,
                       const SubmapId& submap_id) ;
  void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) ;
  void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) ;
  void RunFinalOptimization() ;



  PoseGraphInterface::SubmapData GetSubmapData(const SubmapId& submap_id) const;

  MapById<SubmapId, PoseGraphInterface::SubmapData> GetAllSubmapData() const;

  MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const ;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const ;
  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const ;
  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const ;


  sensor::MapByTime<sensor::ImuData> GetImuData() const ;
 // sensor::MapByTime<sensor::OdometryData> GetOdometryData() const ;



  int AddTrajectoryBuilder();

 // add by lishen
  MapById<SubmapId, PoseGraphInterface::MySubmapData> GetMySubmapData() const;


  std::vector<Constraint> constraints() const  ;

  int GetNumFinishedNodes();

  transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const ;

  transform::Rigid2d GetLatestNodePose() const ;

  // add by lishen
  int SaveMap(const string filename,double _metersPerPixel, std::vector<sensor_msgs::LaserScan> &vt_laser_msg,double creatmaprange);

  // add by lishen
  transform::Rigid2d GetLastSubmapPose();

  // add by lishen
 void UpdateAllSubmapPoses(transform::Rigid3d offset) ;

 // add by lishen
 void Reset();

 // add by lishen
 void RegisterBuildOverCallBack(BuildOverFunc pFunc){

     std::cout<<"RegisterBuildOverCallBack 3"<<std::endl;
     m_pBuildOverFunc = pFunc;
 }


 // add by lishen
 void SetFirstSubmap(const transform::Rigid2d &global_submap_pose,std::unique_ptr<mapping::Grid2D> grid);

 // add by lishen
 void SetOptimizeFlag(bool bflag){ bOptimize=bflag;}

 // add by lishen
 void SetExpandMapFlag(bool bExpand);

 // add by lishen
 void RegisterOptOverCallBack(OptOverFunc pFunc)
 {
    m_pOptOverFunc = pFunc;
 }

 // add by lishen
 void Clear();

 // add by lishen
 bool LoadBinary(FILE *fp);
 // add by lishen
 bool SaveBinary(FILE *fp);
 // dq 3.6 for save&load node msg
 bool SaveNodes(FILE *fp);
 // dq 3.6 for save&load node msg
 bool Save_constant(FILE *fp, const TrajectoryNode::Data& constant_data);
 // dq 3.6 for save&load node msg
 bool LoadNode(FILE *fp);
 // dq 3.6 for save&load node msg
 bool Load_constant(FILE *fp, TrajectoryNode::Data& constant_data);

  // add by lishen
 bool LoadSubmaps(FILE *fp);
  // add by lishen
 bool SaveSubmaps(FILE *fp);

  // add by lishen
 bool RotateMap(double angle);
 // add by lishen
 void SetFrozen(bool flag);

 // add by lishen
 int GetFrozenNodeNum();

 // add by lishen
 int GetFrozenSubmapNum();

private:
  // The current state of the submap in the background threads. When this
  // transitions to kFinished, all nodes are tried to match against this submap.
  // Likewise, all new nodes are matched against submaps which are finished.
  enum class SubmapState { kActive, kFinished };
  struct InternalSubmapData {
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::shared_ptr<Submap2D> submap;

    // IDs of the nodes that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    std::set<NodeId> node_ids;

    SubmapState state ;

        InternalSubmapData() {state = SubmapState::kActive;}

  };

 private:
  void ClearTask();
  bool SaveConstraints(FILE *fp);
  bool LoadConstraints(FILE *fp);


  MapById<SubmapId, PoseGraphInterface::SubmapData> GetSubmapDataUnderLock()const;

  // Handles a new work item.
  void AddWorkItem(const std::function<void()>& work_item) ;

  // Adds connectivity and sampler for a trajectory if it does not exist.
  void AddTrajectoryIfNeeded(int trajectory_id) ;

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<Submap2D>>& insertion_submaps);

  

  // Adds constraints for a node, and starts scan matching in the background.
  void ComputeConstraintsForNode(
      const NodeId& node_id,
      const std::vector<std::shared_ptr<Submap2D>> &insertion_submaps,
      bool newly_finished_submap,bool in_corridor);

  // Computes constraints for a node and submap pair.
  void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id);

  // Adds constraints for older nodes whenever a new submap is finished.
  void ComputeConstraintsForOldNodes(const SubmapId& submap_id);

  // Runs the optimization, executes the trimmers and processes the work queue.
  void HandleWorkQueue(const constraints::ConstraintBuilder2D::Result& result);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() ;

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() ;

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
      int trajectory_id) const ;

  SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const;


  const proto::PoseGraphOptions options_;
  GlobalSlamOptimizationCallback global_slam_optimization_callback_;
  mutable common::Mutex mutex_;

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  std::unique_ptr<std::deque<std::function<void()>>> work_queue_;

  
  // We globally localize a fraction of the nodes from each trajectory.
  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ ;

  // Number of nodes added since last loop closure.
  int num_nodes_since_last_loop_closure_  ;

  // Whether the optimization has to be run before more data is added.
  bool run_loop_closure_  ;
  bool expand_map;

  // Schedules optimization (i.e. loop closure) to run.
  void DispatchOptimization() ;



  // Current optimization problem.
  std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem_;
  constraints::ConstraintBuilder2D constraint_builder_ ;
  std::vector<Constraint> constraints_ ;

  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  MapById<SubmapId, InternalSubmapData> submap_data_;

  // Data that are currently being shown.
  MapById<NodeId, TrajectoryNode> trajectory_nodes_ ;
  int num_trajectory_nodes_  ;

  // Global submap poses currently used for displaying data.
  MapById<SubmapId, optimization::SubmapSpec2D> global_submap_poses_;



  // List of all trimmers to consult when optimizations finish.
  std::vector<std::unique_ptr<PoseGraphTrimmer>> trimmers_ ;

  // Set of all frozen trajectories not being optimized.
  std::set<int> frozen_trajectories_ ;

  // Set of all finished trajectories.
  std::set<int> finished_trajectories_ ;


  BuildOverFunc       m_pBuildOverFunc;
  OptOverFunc         m_pOptOverFunc;

  bool bOptimize;
  bool isRuningOpt;

  bool finalOpt;

  int m_submap_num_old;
  int m_opt_submap_num_old;

  int m_opt_node_num_old;

  bool m_bfrozen;



  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  class TrimmingHandle : public Trimmable {
   public:
    TrimmingHandle(PoseGraph2D* parent);
    ~TrimmingHandle()  {}

    int num_submaps(int trajectory_id) const ;
    std::vector<SubmapId> GetSubmapIds(int trajectory_id) const ;
    MapById<SubmapId, SubmapData> GetOptimizedSubmapData() const ;
      
    const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const ;
        
    const std::vector<Constraint>& GetConstraints() const ;
      
    void MarkSubmapAsTrimmed(const SubmapId& submap_id);
        
    bool IsFinished(int trajectory_id) const  ;

   private:
    PoseGraph2D* const parent_;
  };
};

}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
