
#ifndef SLAM_MAPPING_2D_MAP_LIMITS_H_
#define SLAM_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "mymath.h"
#include "xy_index.h"

#include "options.h"
//#include "trajectory_node.h"
#include "point_cloud.h"
#include "range_data.h"
#include "rigid_transform.h"
#include "transform.h"
#include "mylogging.h"


using namespace myLogging;

namespace mapping {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
class MapLimits {
 public:

	
  MapLimits(const double resolution, 
            const double max_x,
            const double max_y,
            const CellLimits& cell_limits)
      : resolution_(resolution), 
        max_x_(max_x), 
        max_y_(max_y),
         cell_limits_(cell_limits)  
  {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  MapLimits()
  {

  }



  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
 


	const double max_x() const {return max_x_;}
	const double max_y() const {return max_y_;}
 

  // Returns the limits of the grid in number of cells.
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        common::RoundToInt((max_y_ - point.y()) / resolution_ - 0.5),
        common::RoundToInt((max_x_ - point.x()) / resolution_ - 0.5));
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }
  bool LoadBinary(FILE *fp)
  {
      double tmp;
      if (fread(&tmp, sizeof(double), 1, fp) != 1)
        return false;

      resolution_ = tmp;

      if (fread(&tmp, sizeof(double), 1, fp) != 1)
        return false;
      max_x_ = tmp;

      if (fread(&tmp, sizeof(double), 1, fp) != 1)
        return false;

      max_y_ = tmp;

      int n;
      if (fread(&n, sizeof(int), 1, fp) != 1)
        return false;

      cell_limits_.num_x_cells = n;

      if (fread(&n, sizeof(int), 1, fp) != 1)
        return false;

      cell_limits_.num_y_cells =n;

      return true;
  }

  bool SaveBinary(FILE *fp)
  {
      if (fwrite(&resolution_, sizeof(double), 1, fp) != 1)
        return false;
      if (fwrite(&max_x_, sizeof(double), 1, fp) != 1)
        return false;
      if (fwrite(&max_y_, sizeof(double), 1, fp) != 1)
        return false;
      if (fwrite(&cell_limits_.num_x_cells, sizeof(int), 1, fp) != 1)
        return false;
      if (fwrite(&cell_limits_.num_y_cells, sizeof(int), 1, fp) != 1)
        return false;

      return true;
  }
 public:
  	double resolution_;

	double max_x_;
	double max_y_;
  	CellLimits cell_limits_;
};


}  // namespace mapping


#endif  // SLAM_MAPPING_2D_MAP_LIMITS_H_
