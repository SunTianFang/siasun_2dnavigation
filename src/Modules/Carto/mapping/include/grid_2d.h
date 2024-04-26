

#ifndef SLAM_MAPPING_2D_GRID_2D_H_
#define SLAM_MAPPING_2D_GRID_2D_H_

#include <vector>

#include "map_limits.h"
#include "grid_interface.h"



#include "options.h"


namespace mapping {

struct SubmapTexture {


  std::vector<uint8> pixels;

  int num_y_cells;
  int num_x_cells;
  double resolution;
 

  double max_x;
  double max_y;

  transform::Rigid3d slice_pose;
};
struct SubmapTexture_upload {

    struct Pixels {
      std::vector<char> intensity;  // 地图栅格值
      std::vector<char> alpha;      // 栅格的透明度
    };
    Pixels pixels;
    int width;
    int height;
    double resolution;
    transform::Rigid3d slice_pose;
};

struct SubmapTextures {

  int version;
  std::vector<SubmapTexture> textures;
};


class Grid2D : public GridInterface  {
 public:
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit Grid2D(const MapLimits& limits, float min_correspondence_cost,
                  float max_correspondence_cost);
  explicit Grid2D();

  ~Grid2D(){ }

  // Returns the limits of this Grid2D.
  const MapLimits& limits() const { return limits_; }

  // Finishes the update sequence.
  void FinishUpdate();
  // Returns the correspondence cost of the cell with 'cell_index'.
  float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const;

  // Returns the minimum possible correspondence cost.
  float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

  // Returns the maximum possible correspondence cost.
  float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i& cell_index) const;

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  virtual void GrowLimits(const Eigen::Vector2f& point);

  // add by lishen
  virtual void GrowLimitsNew(const Eigen::Vector2f& point);

  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;



  virtual bool DrawToSubmapTexture(
      SubmapTexture*  texture,
      transform::Rigid3d local_pose) const = 0;

  virtual bool DrawToSubmapTexture_upload(
      SubmapTexture_upload*  texture,
      transform::Rigid3d local_pose,vector<double>&blackcell_index) const = 0;
public:
  const std::vector<uint16>& correspondence_cost_cells() const {
    return correspondence_cost_cells_;
  }
  const std::vector<int>& update_indices() const { return update_indices_; }
  const Eigen::AlignedBox2i& known_cells_box() const {
    return known_cells_box_;
  }

  std::vector<uint16>* mutable_correspondence_cost_cells() {
    return &correspondence_cost_cells_;
  }
  std::vector<int>* mutable_update_indices() { return &update_indices_; }
  Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

  // Converts a 'cell_index' into an index into 'cells_'.
  int ToFlatIndex(const Eigen::Array2i& cell_index) const;

  bool LoadBinary(FILE *fp);

  bool SaveBinary(FILE *fp);

  bool RotateMap(double angle);

  void Transform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr);

 public:
  MapLimits limits_;
  std::vector<uint16> correspondence_cost_cells_;
  float min_correspondence_cost_;
  float max_correspondence_cost_;
  std::vector<int> update_indices_;



  // Bounding box of known cells to efficiently compute cropping limits.
  Eigen::AlignedBox2i known_cells_box_;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_2D_GRID_2D_H_
