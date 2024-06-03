
#ifndef SLAM_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define SLAM_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "grid_2d.h"
#include "options.h"
#include "point_cloud.h"
#include "location.h"







namespace mapping {
namespace scan_matching {



// Align scans with an existing map using Ceres.
class CeresScanMatcher2D {
 public:

//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit CeresScanMatcher2D();
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.


  void SetParam(proto::CeresScanMatcherOptions2D param);

  void Match(const Eigen::Vector2d& target_translation,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const Grid2D& grid,
             transform::Rigid2d* pose_estimate,T_L_SUMMARY &summary) const;

  T_Cere_Weight m_weights;


};

}  // namespace scan_matching
}  // namespace mapping


#endif  
