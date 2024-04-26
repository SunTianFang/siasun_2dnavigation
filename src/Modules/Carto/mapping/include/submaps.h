

#ifndef SLAM_MAPPING_SUBMAPS_H_
#define SLAM_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "mymath.h"
#include "port.h"
#include "id.h"
#include "probability_values.h"

//#include "trajectory_node.h"
#include "mylogging.h"

#include "options.h"

#include "grid_2d.h"

using namespace myLogging;

namespace mapping {

// Converts the given probability to log odds.
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

// An individual submap, which has a 'local_pose' in the local map frame, keeps
// track of how many range data were inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.
class Submap {
 public:

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Submap(const transform::Rigid3d& local_submap_pose)
      : local_pose_(local_submap_pose) {   
	num_range_data_ = 0;
  	finished_ = false;}

  Submap() { 
  num_range_data_ = 0;
  finished_ = false;
  }

  virtual ~Submap() {}



  // Fills data into the 'response'.
  virtual void ToResponseSubmapQuery(const transform::Rigid3d& global_submap_pose,
                       SubmapTexture *texture )const = 0;
  virtual void ToResponseSubmapQuery_upload(const transform::Rigid3d& global_submap_pose,
                       SubmapTexture_upload *texture, vector<double>&blackcell_index)const=0;
  // Pose of this submap in the local map frame.
  transform::Rigid3d local_pose() const { return local_pose_; }

  // Number of RangeData inserted.
  int num_range_data() const { return num_range_data_; }
  void set_num_range_data(const int num_range_data) {
    num_range_data_ = num_range_data;
  }

  // Whether the submap is finished or not.
  bool finished() const { return finished_; }
  void set_finished(bool finished) { finished_ = finished; }


  transform::Rigid3d local_pose_;
 private:
  //const transform::Rigid3d local_pose_;
  int num_range_data_ ;
  bool finished_ ;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_SUBMAPS_H_
