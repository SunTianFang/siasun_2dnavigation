

#ifndef SLAM_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
#define SLAM_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <vector>

#include "Eigen/Core"

#include "map_limits.h"
#include "xy_index.h"
#include "point_cloud.h"



#include "Eigen/StdVector"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Array2i)


namespace mapping {
namespace scan_matching {

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

// Describes the search space.
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::PointCloud& point_cloud, double resolution,double angular_resolution );

  // For testing.
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // Tightens the search window as much as possible.
  void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                   const CellLimits& cell_limits);

  int num_angular_perturbations;
  double angular_perturbation_step_size;
  double resolution;
  int num_scans;
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
  double  angular_resolution;
};

// Generates a collection of rotated scans.
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
struct Candidate2D {
  Candidate2D(const int init_scan_index, const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {score = 0.f;}

 Candidate2D()
	{
	  scan_index = 0;
	  x_index_offset = 0;
	  y_index_offset = 0;
  	  x = 0.;
	  y = 0.;
	  orientation = 0.;
	  score = 0.f;
	}
   

  // Index into the rotated scans vector.
  int scan_index ;

  // Linear offset from the initial pose.
  int x_index_offset ;
  int y_index_offset ;

  // Pose of this Candidate2D relative to the initial pose.
  double x ;
  double y ;
  double orientation ;

  // Score, higher is better.
  float score ;

  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
