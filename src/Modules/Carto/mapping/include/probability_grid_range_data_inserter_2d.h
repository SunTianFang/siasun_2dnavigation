
#ifndef SLAM_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#define SLAM_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_

#include <utility>
#include <vector>


#include "port.h"
#include "probability_grid.h"
#include "xy_index.h"

#include "options.h"
#include "range_data_inserter_interface.h"
#include "point_cloud.h"
#include "range_data.h"


namespace mapping {



class ProbabilityGridRangeDataInserter2D : public RangeDataInserterInterface {
 public:
  explicit ProbabilityGridRangeDataInserter2D(
      const proto::ProbabilityGridRangeDataInserterOptions2D& options);

  ProbabilityGridRangeDataInserter2D(
      const ProbabilityGridRangeDataInserter2D&) = delete;
  ProbabilityGridRangeDataInserter2D& operator=(
      const ProbabilityGridRangeDataInserter2D&) = delete;

  // Inserts 'range_data' into 'probability_grid'.
  virtual void Insert(const sensor::RangeData& range_data,
                      GridInterface* grid) const ;

 private:
  const proto::ProbabilityGridRangeDataInserterOptions2D options_;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
