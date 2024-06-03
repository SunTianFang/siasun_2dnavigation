

#ifndef SLAM_MAPPING_RANGE_DATA_INSERTER_H_
#define SLAM_MAPPING_RANGE_DATA_INSERTER_H_

#include <utility>
#include <vector>

#include "grid_interface.h"
#include "options.h"
#include "range_data.h"


namespace mapping {


class RangeDataInserterInterface {
 public:
  // Inserts 'range_data' into 'grid'.
  virtual void Insert(const sensor::RangeData& range_data,
                      GridInterface* grid) const = 0;
};

}  // namespace mapping


#endif  // SLAM_MAPPING_RANGE_DATA_INSERTER_H_
