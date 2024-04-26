

#ifndef SLAM_MAPPING_2D_PROBABILITY_GRID_H_
#define SLAM_MAPPING_2D_PROBABILITY_GRID_H_

#include <vector>

#include "port.h"
#include "grid_2d.h"
#include "map_limits.h"
#include "xy_index.h"


#include "options.h"


namespace mapping {


// Represents a 2D grid of probabilities.
class ProbabilityGrid : public Grid2D {
 public:
  explicit ProbabilityGrid(const MapLimits& limits);

  explicit ProbabilityGrid();
 
  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability);

  // add by lishen
  void SetProbabilityAnyWay(const Eigen::Array2i& cell_index,
                      const float probability);

  //by DQ
  void SelectGrid( Eigen::Vector2f TLpoint,
                                      Eigen::Vector2f BRpoint);

  //by DQ
  void DeleteGrid(bool entrue = false);

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table);

  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i& cell_index) const;

 // virtual proto::Grid2D ToProto() const ;
  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const ;
  virtual bool DrawToSubmapTexture(
      SubmapTexture*  texture,
      transform::Rigid3d local_pose) const ;
  // add by dq
  virtual bool DrawToSubmapTexture_upload(SubmapTexture_upload *texture,
      transform::Rigid3d local_pose, vector<double> &blackcell_index) const ;


  // add by lishen
  bool Translate(float x, float y);

  // add by lishen
  void MergeOtherMap(std::shared_ptr<ProbabilityGrid> otherMap);


  int MergePgm(std::unique_ptr<ProbabilityGrid> otherMap);

  // add by lishen
  int saveMap(const string fileName);
  int saveBinaryMap(const string fileName);
  //add by lishen
  std::unique_ptr<ProbabilityGrid> CreateLowerResolutionMap(int deci);

  // by DQ
  int GridIndex_size() const { return GridIndex_set.size(); }

 //private:
  std::vector<Eigen::Vector2i> GridIndex_set;


};


std::unique_ptr<ProbabilityGrid> CreateProbabilityGridFromFile(std::string filename);


}  // namespace mapping


#endif  // SLAM_MAPPING_2D_PROBABILITY_GRID_H_
