
#ifndef SENSOR_POINT_CLOUD_H_
#define SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"

#include "rigid_transform.h"
#include "mylogging.h"

#include "Eigen/StdVector"



//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3f)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d)

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaternionf)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaterniond)

using namespace myLogging;

namespace sensor {

// Stores 3D positions of points.
// For 2D points, the third entry is 0.f.
typedef std::vector<Eigen::Vector3f> PointCloud;

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f. For 2D points, the
// third entry is 0.f (and the fourth entry is time).

typedef std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> TimedPointCloud;

struct PointCloudWithIntensities {
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TimedPointCloud points;
  std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud' according to 'transform'.
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
TimedPointCloud CropTimedPointCloud(const TimedPointCloud& point_cloud,
                                    float min_z, float max_z);

}  // namespace sensor


#endif  // SENSOR_POINT_CLOUD_H_
