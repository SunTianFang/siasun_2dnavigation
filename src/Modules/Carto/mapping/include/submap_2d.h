
#ifndef SLAM_MAPPING_2D_SUBMAP_2D_H_
#define SLAM_MAPPING_2D_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "grid_2d.h"
#include "map_limits.h"

#include "range_data_inserter_interface.h"
#include "submaps.h"
#include "trajectory_node.h"
#include "range_data.h"
#include "rigid_transform.h"





namespace mapping {



class Submap2D : public Submap {
 public:

  Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid);
  Submap2D();


  void ToResponseSubmapQuery(const transform::Rigid3d& global_submap_pose,
                       SubmapTexture *texture ) const ;
  void ToResponseSubmapQuery_upload(const transform::Rigid3d& global_submap_pose,
                       SubmapTexture_upload *texture, vector<double>&blackcell_index) const;
  const Grid2D* grid() const { return grid_.get(); }

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const RangeDataInserterInterface* range_data_inserter);
  void Finish();

  bool LoadBinary(FILE *fp) ;

  bool SaveBinary(FILE *fp, bool include_grid_data) const;

  bool RotateMap(double angle);

 private:
  std::unique_ptr<Grid2D> grid_;
};

// Except during initialization when only a single submap exists, there are
// always two submaps into which range data is inserted: an old submap that is
// used for matching, and a new one, which will be used for matching next, that
// is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
class ActiveSubmaps2D {
 public:

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   // add by lishen
  explicit ActiveSubmaps2D(const proto::SubmapsOptions2D& options,Eigen::Vector2f &initpos,std::unique_ptr<mapping::Grid2D> gridMap);

  //ActiveSubmaps2D()  {matching_submap_index_ = 0;}

  ActiveSubmaps2D(const ActiveSubmaps2D&) = delete;
  ActiveSubmaps2D& operator=(const ActiveSubmaps2D&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  int matching_index() const;

  // Inserts 'range_data' into the Submap collection.
  void InsertRangeData(const sensor::RangeData& range_data);

 void AddSubmap(const Eigen::Vector2f& origin);

  std::vector<std::shared_ptr<Submap2D> > submaps() const;

  // add by lishen
  void Clear();


 private:
  std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
  std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f& origin);
  void FinishSubmap();

  const proto::SubmapsOptions2D options_;
  int matching_submap_index_ ;
  std::vector<std::shared_ptr<Submap2D>> submaps_;
  std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
};


}  // namespace mapping


#endif  // SLAM_MAPPING_2D_SUBMAP_2D_H_
