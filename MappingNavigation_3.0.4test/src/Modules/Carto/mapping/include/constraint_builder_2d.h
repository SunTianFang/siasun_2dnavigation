
#ifndef SLAM_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_
#define SLAM_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "fixed_ratio_sampler.h"
#include "mymath.h"
#include "mutex.h"
#include "task.h"
#include "thread_pool.h"
#include "submap_2d.h"
#include "ceres_scan_matcher_2d.h"
#include "fast_correlative_scan_matcher_2d.h"
#include "pose_graph_interface.h"
#include "options.h"

#include "voxel_filter.h"
#include "point_cloud.h"


namespace mapping {
namespace constraints {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
transform::Rigid2d ComputeSubmapPose(const Submap2D& submap);

// Asynchronously computes constraints.
//
// Intermingle an arbitrary number of calls to 'MaybeAddConstraint',
// 'MaybeAddGlobalConstraint', and 'NotifyEndOfNode', then call 'WhenDone' once.
// After all computations are done the 'callback' will be called with the result
// and another MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
//
// This class is thread-safe.
class ConstraintBuilder2D {
 public:

//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef PoseGraphInterface::Constraint Constraint;
  typedef std::vector<Constraint> Result;

  ConstraintBuilder2D(const proto::ConstraintBuilderOptions& options,
                      common::ThreadPoolInterface* thread_pool);
  ~ConstraintBuilder2D();

  ConstraintBuilder2D(const ConstraintBuilder2D&) = delete;
  ConstraintBuilder2D& operator=(const ConstraintBuilder2D&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
  // 'initial_relative_pose' is relative to the 'submap'.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddConstraint(const SubmapId& submap_id, const Submap2D* submap,
                          const NodeId& node_id,
                          const TrajectoryNode::Data* const constant_data,
                          const transform::Rigid2d& initial_relative_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddGlobalConstraint(
      const SubmapId& submap_id, const Submap2D* submap, const NodeId& node_id,
      const TrajectoryNode::Data* const constant_data);

  // Must be called after all computations related to one node have been added.
  void NotifyEndOfNode();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by 'MaybeAdd*Constraint' have finished.
  // 'callback' is executed in the 'ThreadPool'.
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished nodes.
  int GetNumFinishedNodes();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const SubmapId& submap_id);



  // add by lishen
  void SetExpandMapFlag(bool bExpand);

  void SetOptNodeNumOld(int num);

  void SetFinishNodeNum(int num);

  // add by lishen
  void Reset();

  // add by lishen
  void ClearTask();

  // add by lishen
  void Clear();
 

 private:
  struct SubmapScanMatcher {

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const Grid2D* grid;
    std::shared_ptr<scan_matching::FastCorrelativeScanMatcher2D>
        fast_correlative_scan_matcher;
    std::weak_ptr<common::Task> creation_task_handle;



  };

  // The returned 'grid' and 'fast_correlative_scan_matcher' must only be
  // accessed after 'creation_task_handle' has completed.
  const SubmapScanMatcher* DispatchScanMatcherConstruction(
      const SubmapId& submap_id, const Grid2D* grid) REQUIRES(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'compressed_point_cloud' do not change
  // anymore. As output, it may create a new Constraint in 'constraint'.
  void ComputeConstraint(const SubmapId& submap_id, const Submap2D* submap,
                         const NodeId& node_id, bool match_full_submap,
                         const TrajectoryNode::Data* const constant_data,
                         const transform::Rigid2d& initial_relative_pose,
                         const SubmapScanMatcher& submap_scan_matcher,
                         std::unique_ptr<Constraint>* constraint);

  void RunWhenDoneCallback();

  const proto::ConstraintBuilderOptions options_;
  common::ThreadPoolInterface* thread_pool_;
  common::Mutex mutex_;

  // 'callback' set by WhenDone().
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // TODO(gaschler): Use atomics instead of mutex to access these counters.
  // Number of the node in reaction to which computations are currently
  // added. This is always the number of nodes seen so far, even when older
  // nodes are matched against a new submap.
  int num_started_nodes_  ;

  int num_finished_nodes_  ;

  std::unique_ptr<common::Task> finish_node_task_ ;

  std::unique_ptr<common::Task> when_done_task_ ;

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries. Constraint search results
  // with below-threshold scores are also 'nullptr'.
  std::deque<std::unique_ptr<Constraint>> constraints_ ;

  // Map of dispatched or constructed scan matchers by 'submap_id'.
  std::map<SubmapId, SubmapScanMatcher> submap_scan_matchers_
      GUARDED_BY(mutex_);

  common::FixedRatioSampler sampler_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;


  bool is_computeConstrainting;

  bool expand_map;

  int m_opt_node_num_old;

  // Histogram of scan matcher scores.
  //common::Histogram score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace constraints
}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_2D_H_
