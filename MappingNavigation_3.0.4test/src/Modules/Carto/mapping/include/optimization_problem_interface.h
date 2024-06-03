

#ifndef SLAM_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_
#define SLAM_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_

#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "mytime.h"
#include "id.h"
#include "pose_graph_interface.h"

#include "imu_data.h"
#include "map_by_time.h"
#include "odometry_data.h"


namespace mapping {
namespace optimization {

// Implements the SPA loop closure method.
template <typename NodeDataType, typename SubmapDataType,
          typename RigidTransformType>
class OptimizationProblemInterface {
 public:

 //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef PoseGraphInterface::Constraint   Constraint;
 
  OptimizationProblemInterface() {}
  virtual ~OptimizationProblemInterface() {}

  OptimizationProblemInterface(const OptimizationProblemInterface&) = delete;
  OptimizationProblemInterface& operator=(const OptimizationProblemInterface&) =
      delete;

  virtual void AddImuData(int trajectory_id,
                          const sensor::ImuData& imu_data) = 0;
  virtual void AddOdometryData(int trajectory_id,
                               const sensor::myOdometryData& odometry_data) = 0;
  virtual void AddTrajectoryNode(int trajectory_id,
                                 const NodeDataType& node_data) = 0;
  virtual void InsertTrajectoryNode(const NodeId& node_id,
                                    const NodeDataType& node_data) = 0;
  virtual void TrimTrajectoryNode(const NodeId& node_id) = 0;
  virtual void AddSubmap(int trajectory_id,
                         const RigidTransformType& global_submap_pose) = 0;
  virtual void InsertSubmap(const SubmapId& submap_id,
                            const RigidTransformType& global_submap_pose) = 0;
  virtual void TrimSubmap(const SubmapId& submap_id) = 0;
  virtual void SetMaxNumIterations(int32 max_num_iterations) = 0;

  virtual void Reset(void) = 0;


  // Optimizes the global poses.
  virtual void Solve(
      const std::vector<Constraint>& constraints,
      const std::set<int>& frozen_trajectories
          ,int frozen_submapnum,int frozen_nodenum,int nodenumold) = 0;

  virtual const MapById<NodeId, NodeDataType>& node_data() const = 0;
  virtual const MapById<SubmapId, SubmapDataType>& submap_data() const = 0;

  virtual const sensor::MapByTime<sensor::ImuData>& imu_data() const = 0;
  virtual  sensor::MapByTime<sensor::myOdometryData>& odometry_data()
      = 0;

  virtual bool Rotate(double angle) = 0;
};

}  // namespace optimization
}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_
