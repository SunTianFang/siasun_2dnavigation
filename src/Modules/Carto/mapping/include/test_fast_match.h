#ifndef TESTFASTMATCH_H
#define TESTFASTMATCH_H

#include <iostream>
#include <memory>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>

#include "options.h"
#include "probability_grid.h"
#include "probability_grid_range_data_inserter_2d.h"
#include "rigid_transform.h"
#include "transform.h"
#include "point_cloud.h"
#include "make_unique.h"

#include "fast_correlative_scan_matcher_2d.h"
//#include "ceres_scan_matcher_2d.h"

using namespace std;
using namespace mapping;




bool CreateGridMap(sensor::PointCloud &point_cloud,std::string filename,double size_x,double size_y,double max_x,double max_y);



#endif // TESTFASTMATCH_H

