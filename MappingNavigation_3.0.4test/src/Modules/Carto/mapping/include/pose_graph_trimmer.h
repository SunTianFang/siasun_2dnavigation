
#ifndef SLAM_MAPPING_POSE_GRAPH_TRIMMER_H_
#define SLAM_MAPPING_POSE_GRAPH_TRIMMER_H_

#include "id.h"
#include "pose_graph_interface.h"


namespace mapping {

// Implemented by the pose graph to provide thread-safe access to functions for
// trimming the graph.
class Trimmable {
 public:
  virtual ~Trimmable() {}

  virtual int num_submaps(int trajectory_id) const = 0;

  virtual std::vector<SubmapId> GetSubmapIds(int trajectory_id) const = 0;
  // Returns finished submaps with optimized poses only.
  virtual MapById<SubmapId, PoseGraphInterface::SubmapData>
  GetOptimizedSubmapData() const = 0;
  virtual const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const = 0;
  virtual const std::vector<PoseGraphInterface::Constraint>& GetConstraints()
      const = 0;

  // Marks 'submap_id' and corresponding intra-submap nodes as trimmed. They
  // will no longer take part in scan matching, loop closure, visualization.
  // Submaps and nodes are only marked, the numbering remains unchanged.
  virtual void MarkSubmapAsTrimmed(const SubmapId& submap_id) = 0;

  // Checks if the given trajectory is finished or not.
  virtual bool IsFinished(int trajectory_id) const = 0;
};

// An interface to implement algorithms that choose how to trim the pose graph.

class PoseGraphTrimmer {
 public:
   ~PoseGraphTrimmer() {}

  // Called once after each pose graph optimization.
  virtual void Trim(Trimmable* pose_graph) = 0;

  // Checks if this trimmer is in a terminatable state.
  virtual bool IsFinished() = 0;
};

// Keeps the last 'num_submaps_to_keep' of the trajectory with 'trajectory_id'
// to implement localization without mapping.
class PureLocalizationTrimmer : public PoseGraphTrimmer {
 public:
  PureLocalizationTrimmer(int trajectory_id, int num_submaps_to_keep);
  ~PureLocalizationTrimmer()  {}

  void Trim(Trimmable* pose_graph) ;
  bool IsFinished() ;

 private:
  const int trajectory_id_;
  int num_submaps_to_keep_;
  bool finished_ ;
};

}  // namespace mapping


#endif  //SLAM_MAPPING_POSE_GRAPH_TRIMMER_H_
