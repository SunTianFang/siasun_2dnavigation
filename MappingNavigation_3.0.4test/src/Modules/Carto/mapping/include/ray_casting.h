
#ifndef SLAM_MAPPING_INTERNAL_2D_RAY_CASTING_H_
#define SLAM_MAPPING_INTERNAL_2D_RAY_CASTING_H_

#include <vector>

#include "port.h"
#include "probability_grid.h"
#include "point_cloud.h"
#include "range_data.h"
#include "transform.h"


namespace mapping {

// For each ray in 'range_data', inserts hits and misses into
// 'probability_grid'. Hits are handled before misses.
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table, bool insert_free_space,
              ProbabilityGrid* probability_grid);

}  // namespace mapping


#endif  // SLAM_MAPPING_INTERNAL_2D_RAY_CASTING_H_
