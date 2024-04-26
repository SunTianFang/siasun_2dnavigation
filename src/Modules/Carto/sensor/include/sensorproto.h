

#ifndef SENSOR_PROTO_H_
#define SENSOR_PROTO_H_


#include <vector>
#include "port.h"

#include "transformproto.h"


using namespace std;


namespace proto{


// Compressed collection of a 3D point cloud.
struct CompressedPointCloud {
  int32 num_points ;
  vector<int32> point_data ;
};

// Proto representation of ::cartographer::sensor::TimedPointCloudData.
struct TimedPointCloudData {
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64 timestamp ;
  Vector3f origin ;
  vector<Vector4f> point_data ;
};

// Proto representation of ::cartographer::sensor::RangeData.
struct RangeData {
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector3f origin ;
  vector<Vector3f> returns ;
  vector<Vector3f> misses ;
};


/*
// Proto representation of ::cartographer::sensor::LandmarkData.
struct sensorLandmarkData {
  struct LandmarkObservation {
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //bytes id ; ??????
    Rigid3d landmark_to_tracking_transform ;
    double translation_weight ;
    double rotation_weight ;
  };
  int64 timestamp ;
  vector<LandmarkObservation> landmark_observations ;
};*/
struct AdaptiveVoxelFilterOptions {
  // 'max_length' of a voxel edge.
  float max_length ;

  // If there are more points and not at least 'min_num_points' remain, the
  // voxel length is reduced trying to get this minimum number of points.
  float min_num_points ;

  // Points further away from the origin are removed.
  float max_range ;
};




}

#endif
