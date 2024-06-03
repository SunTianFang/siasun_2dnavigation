#pragma once

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include "ndt_omp.h"
#include "ndt_pointcloud.h"
//#include "ndt_omp/ndt_map.h"
//#include "ndt_omp/pointcloud_utils.h"
//#include "ndt_omp/ndt_inform.h"
//#include "LocResult.h"

//#include "ndt_oru/NdtBaseLocalization_oru.h"
#include <Eigen/StdVector>

#define DEFAULT_MAP_RESO 0.2    // 地图缺省分辨率0.2m

using namespace std;

typedef pair<Eigen::Affine3d, int> Candidate_Pose;

namespace ndt_omp
{
class ndtInform_omp;

typedef pcl::PointCloud<pcl::PointXYZ> CPclPointCloud;
typedef NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> NdtOmpRegistration;

class CNdtLocalization
{
  public:
    CNdtLocalization(double map_reso = DEFAULT_MAP_RESO);
    ~CNdtLocalization();

  public:
    // 采用Ndt-omp算法进行点云配准
    bool Localize(ndt_oru::CPointCloud &cloud_in, Eigen::Affine3d &init, Eigen::Affine3d &estimate);

    // 在NDT-OMP算法中，生成TargetCells
    void CreateTargetCells(pcl::PointCloud<pcl::PointXYZ> *cloud);

    // 一致性核查
    void setConsistencyCheck(double x, double y, double r);

  private:
    void LocalizeNdtOmp(const CPclPointCloud &cloud_in, Eigen::Affine3d &initPose, Eigen::Affine3d &estimate);

    // 对输入的点云进行处理，滤除过近的点，并对Z坐标进行初始化
    void FilterCloud(const CPclPointCloud &cloud_in, CPclPointCloud &cloud_filtered);

  public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;
    NdtOmpRegistration::Ptr pOmp;

  public:
    double sensor_max_range;
    double sensor_min_range;
    double resolution;

    // consistency Chenck
    double max_translation_x;    // x方向 最大允许距离偏差
    double max_translation_y;    // y方向 最大允许距离偏差
    double max_rotation_z;       // 最大允许角度偏差

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}    // namespace ndt_omp
