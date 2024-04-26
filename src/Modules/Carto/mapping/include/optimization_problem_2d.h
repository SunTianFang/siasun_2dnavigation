
#ifndef SLAM_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
#define SLAM_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_

#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "port.h"
#include "mytime.h"
#include "id.h"
#include "optimization_problem_interface.h"
#include "pose_graph_interface.h"
#include "options.h"
#include "imu_data.h"
#include "map_by_time.h"
#include "odometry_data.h"
#include "timestamped_transform.h"


namespace mapping {
namespace optimization {

struct NodeSpec2D {

//	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::Time time;
  transform::Rigid2d local_pose_2d;
  transform::Rigid2d global_pose_2d;
  Eigen::Quaternion<double,Eigen::DontAlign> gravity_alignment;

  transform::Rigid3d icp_relative_pose_last;  //lishen

};

struct SubmapSpec2D {
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  transform::Rigid2d global_pose;
};

class OptimizationProblem2D
    : public OptimizationProblemInterface<NodeSpec2D, SubmapSpec2D,
                                          transform::Rigid2d> {
 public:

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OptimizationProblem2D(
      const ::proto::OptimizationProblemOptions& options);
  ~OptimizationProblem2D();

  OptimizationProblem2D(const OptimizationProblem2D&) = delete;
  OptimizationProblem2D& operator=(const OptimizationProblem2D&) = delete;

  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) ;
  void AddOdometryData(int trajectory_id,
                       const sensor::myOdometryData& odometry_data) ;
  void AddTrajectoryNode(int trajectory_id,
                         const NodeSpec2D& node_data) ;
  void InsertTrajectoryNode(const NodeId& node_id,
                            const NodeSpec2D& node_data) ;
  void TrimTrajectoryNode(const NodeId& node_id) ;
  void AddSubmap(int trajectory_id,
                 const transform::Rigid2d& global_submap_pose) ;
  void InsertSubmap(const SubmapId& submap_id,
                    const transform::Rigid2d& global_submap_pose) ;
  void TrimSubmap(const SubmapId& submap_id) ;
  void SetMaxNumIterations(int32 max_num_iterations) ;

  // applies transform to global pose of all submaps
  void MoveAllSubmaps(transform::Rigid2d offset);
  // void MoveAllNodes(transform::Rigid2d offset);

  void Solve(
      const std::vector<Constraint>& constraints,
      const std::set<int>& frozen_trajectories,int frozen_submapnum,int frozen_nodenum,int nodenumold) ;

  const MapById<NodeId, NodeSpec2D>& node_data() const  {
    return node_data_;
  }
  const MapById<SubmapId, SubmapSpec2D>& submap_data() const  {
    return submap_data_;
  }

  const sensor::MapByTime<sensor::ImuData>& imu_data() const  {
    return imu_data_;
  }
   sensor::MapByTime<sensor::myOdometryData>& odometry_data()
        {
    return odometry_data_;
  }

  bool LoadBinary(FILE *fp);

  bool SaveBinary(FILE *fp);

  bool Rotate(double angle);

  void Reset(void)  {

 
	node_data_.clear();
	submap_data_.clear();
	imu_data_.clear();
	odometry_data_.clear();
 
	}

  void ClearButFirstSubmap(void)  {


        node_data_.clear();
        imu_data_.clear();
        odometry_data_.clear();

        /*for (const auto& submap_id_data :submap_data_) {

            printf("submap_id_data.id.submap_index=%d   \n",submap_id_data.id.submap_index);
            if(submap_id_data.id.submap_index!=0)
            {
                printf("trim submap_id_data.id.submap_index=%d   \n",submap_id_data.id.submap_index);
                submap_data_.Trim(submap_id_data.id);

            }
          }*/

        }

 private:
  std::unique_ptr<transform::Rigid3d> InterpolateOdometry(
      int trajectory_id, common::Time time) const;
  // Computes the relative pose between two nodes based on odometry data.
  std::unique_ptr<transform::Rigid3d> CalculateOdometryBetweenNodes(
      int trajectory_id, const NodeSpec2D& first_node_data,
      const NodeSpec2D& second_node_data) const;




  ::proto::OptimizationProblemOptions options_;
  MapById<NodeId, NodeSpec2D> node_data_;
  MapById<SubmapId, SubmapSpec2D> submap_data_;

  sensor::MapByTime<sensor::ImuData> imu_data_;
  sensor::MapByTime<sensor::myOdometryData> odometry_data_;
};

}  // namespace optimization
}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
