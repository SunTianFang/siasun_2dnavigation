/*
 * Copyright 2018 The Cartographer Authors
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

#include "jet.h"


class SpaCostFunction2D {
 public:
  SpaCostFunction2D(double *pose_,double wt,double wr)
  {
	observed_relative_pose_[0] = pose_[0];
	observed_relative_pose_[1] = pose_[1];
	observed_relative_pose_[2] = pose_[2];
	weight_t = wt;
	weight_r = wr;
  }
  template <typename T>
  bool func( T* start_pose,T* end_pose,
                  T* e)  {
  T cos_theta_i = cos(start_pose[2]);
  T sin_theta_i = sin(start_pose[2]);
  T delta_x = end_pose[0] - start_pose[0];
  T delta_y = end_pose[1] - start_pose[1];
  T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  end_pose[2] - start_pose[2]};
  e[0] = (T(observed_relative_pose_[0]) - h[0]) * weight_t;
  e[1] = (T(observed_relative_pose_[1]) - h[1]) * weight_t;
  e[2] = (T(observed_relative_pose_[2]) - h[2]) * weight_r;
    return true;
  }
  
 private:
  double observed_relative_pose_[3];
  double weight_t;
  double weight_r;
};



