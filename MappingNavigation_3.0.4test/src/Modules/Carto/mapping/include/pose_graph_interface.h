
#ifndef SLAM_MAPPING_POSE_GRAPH_INTERFACE_H_
#define SLAM_MAPPING_POSE_GRAPH_INTERFACE_H_

#include <vector>

#include "id.h"
#include "submaps.h"
#include "rigid_transform.h"
#include "msg_conversion.h"
#include "trajectory_node.h"
#include <set>
#include "type.h"

namespace mapping {

class PoseGraphInterface {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  struct Constraint {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct Pose {
		//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      transform::Rigid3d zbar_ij;
      double translation_weight;
      double rotation_weight;


	Pose(transform::Rigid3d _zbar_ij,double _translation_weight,double _rotation_weight)
	: zbar_ij(_zbar_ij),
	translation_weight(_translation_weight),
	rotation_weight(_rotation_weight) {}

	Pose(const Pose &pos){
	zbar_ij = pos.zbar_ij;
	translation_weight = pos.translation_weight;
	rotation_weight = pos.rotation_weight;
	}
	

    };

    SubmapId submap_id;  // 'i' in the paper.
    NodeId node_id;      // 'j' in the paper.

    // Pose of the node 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where node 'j' was inserted into
    // submap 'i') and inter-submap constraints (where node 'j' was not inserted
    // into submap 'i').
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;

	

   Constraint(SubmapId _submap_id, NodeId _node_id, Pose _pose, Tag  _tag)
      : submap_id(_submap_id),
        node_id(_node_id),
        pose(_pose),
        tag(_tag)
       {}



  };



  struct SubmapPose {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int version;
    transform::Rigid3d pose;
  };

  struct SubmapData {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
  };

  struct MySubmapData {

      std::set<NodeId> node_ids;

  };


  typedef std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,const std::map<int /* trajectory_id */, NodeId>&)> GlobalSlamOptimizationCallback;

  PoseGraphInterface() {}
  virtual ~PoseGraphInterface() {}

  PoseGraphInterface(const PoseGraphInterface&) = delete;
  PoseGraphInterface& operator=(const PoseGraphInterface&) = delete;

  // Waits for all computations to finish and computes optimized poses.
  virtual void RunFinalOptimization() = 0;

  // Returns data for all submaps.
  virtual MapById<SubmapId, SubmapData> GetAllSubmapData() const = 0;

  // Returns the global poses for all submaps.
  virtual MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  virtual transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) const = 0;

  // Returns the current optimized trajectories.
  virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const = 0;

  // Returns the current optimized trajectory poses.
  virtual MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses()
      const = 0;

  virtual MapById<SubmapId, PoseGraphInterface::MySubmapData> GetMySubmapData() const = 0;

  // Checks if the given trajectory is finished.
  virtual bool IsTrajectoryFinished(int trajectory_id) const = 0;

  // Checks if the given trajectory is frozen.
  virtual bool IsTrajectoryFrozen(int trajectory_id) const = 0;

  
  // Returns the collection of constraints.
  virtual std::vector<Constraint> constraints() const = 0;

  virtual void Reset() = 0;

  virtual int SaveMap(const string filename,double _metersPerPixel, std::vector<sensor_msgs::LaserScan> &vt_laser_msg,double creatmaprange)  = 0;

  virtual void SetOptimizeFlag(bool bflag) = 0;

  virtual int GetNumFinishedNodes() = 0;

  virtual  void RegisterBuildOverCallBack(BuildOverFunc pFunc) = 0;

  virtual void RegisterOptOverCallBack(OptOverFunc pFunc) = 0;

  virtual int AddTrajectoryBuilder() = 0;

  virtual void SetExpandMapFlag(bool bExpand) = 0;
  virtual void  SetFirstSubmap(const transform::Rigid2d &global_submap_pose,std::unique_ptr<mapping::Grid2D> grid) = 0;


    virtual void Clear() = 0;

    virtual bool  SaveBinary(FILE *fp) = 0;

    virtual bool  LoadBinary(FILE *fp) = 0;

    virtual bool RotateMap(double angle) = 0;

    virtual void SetFrozen(bool flag) =0;

    virtual int GetFrozenNodeNum() = 0;

     virtual int GetFrozenSubmapNum() = 0;
};



}  // namespace mapping


#endif  // SLAM_MAPPING_POSE_GRAPH_INTERFACE_H_
