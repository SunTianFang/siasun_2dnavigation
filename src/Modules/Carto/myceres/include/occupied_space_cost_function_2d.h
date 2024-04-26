/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_

#include <limits.h>
#include <iomanip>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Grid2d_T.h"
#include "pointCloud_T.h"
//#include "cartographer/mapping/2d/grid_2d.h"
//#include "cartographer/mapping/probability_values.h"
//#include "cartographer/sensor/point_cloud.h"
//#include "ceres/ceres.h"
#include "cubic_interpolation.h"


class OccupiedSpaceCostFunction2D {
 public:
  template <typename T>
  bool Func1(const T* const pose, T* residual) {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);


    //printf("space_weight_= %f\n",space_weight_);

    double factor_ = space_weight_ / sqrt(point_cloud_.size());

    GridArrayAdapter adapter(grid_);
    myceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
	  int a = 0;
     
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point(T(point_cloud_.xyz[i].x),T(point_cloud_.xyz[i].y), T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;

   	interpolator.Evaluate(
          (grid_.limits_max_x - world[0]) / grid_.limits_resolution - 0.5 ,
          (grid_.limits_max_y - world[1]) / grid_.limits_resolution - 0.5 ,
          &residual[i],a);
	
      residual[i] = factor_ * residual[i];
    }
    return true;
  }

	 template <typename T>
	bool Func2(const T* const pose, T* residual) {
	 

    //printf("translation_weight_= %f\n",translation_weight_);

	 residual[0] = translation_weight_ * (pose[0] - x_);
	 residual[1] = translation_weight_ * (pose[1] - y_);
    return true;
  }
	 template <typename T>
   	bool Func3(const T* const pose, T* residual) {

          //printf("rotation_weight_= %f\n",rotation_weight_);
	
	 residual[0] = rotation_weight_ * (pose[2] - t_);
    return true;
  }


  OccupiedSpaceCostFunction2D( double space_weight,double translation_weight,double rotation_weight,
                               PointCloud_T& point_cloud,
                               Grid2D_T& grid, double x,double y, double t)
      : space_weight_(space_weight),
        translation_weight_(translation_weight),
        rotation_weight_(rotation_weight),
        point_cloud_(point_cloud),
        grid_(grid) ,
		x_(x),y_(y),t_(t){}

 private:
 // int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(Grid2D_T& grid) : grid_(grid) {}

    void GetValue(int row, int column, double* value)  {
	 int kPadding = INT_MAX / 4;
      if (row < 0 || column < 0 || row >= NumRows()  ||
          column >= NumCols() ) {
        *value = 0.9;
      } else {
        *value = (double)(grid_.GetCorrespondenceCost(column , row ));
      }
    }

    int NumRows()  {
      return grid_.num_rows_;
    }

    int NumCols() {
      return grid_.num_cols_;
    }

   private:
     Grid2D_T& grid_;
  };


  double space_weight_;
  double translation_weight_;
  double rotation_weight_;
  PointCloud_T& point_cloud_;
  Grid2D_T& grid_;
  double x_;
  double y_;
  double t_;
};



#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
