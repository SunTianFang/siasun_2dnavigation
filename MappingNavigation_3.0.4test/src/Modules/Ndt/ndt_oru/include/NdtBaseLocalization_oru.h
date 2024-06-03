#pragma once

#include "ndt_matcher_d2d_2d.h"
#include "VectPose.h"
#include "ndt_pointcloud.h"
#include "ndt_inform_oru.h"
#include "LocResult.h"
#include "ScannerParam.h"  // By Sam

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#else
#include <Eigen/Eigen>
#endif

using namespace std;

#define MAX_SENSOR_RANGE 40     // 传感器最远有效距离
#define MIN_SENSOR_RANGE 0.6    // 传感器最近有效距离

namespace ndt_oru
{
class ndtInform_oru;

///////////////////////////////////////////////////////////////////////////////
//
//   CNdtBaseLocalization封装了利用NDT方法进行基础定位的方法。
//
//   说明：
//      1. 支持Gauss-Newton定位方案
//      2. 仅处理基于机器人的定位(假定仅有一个激光器，且安装姿态与机器人坐标系重合)
//      3. 提供两种定位模式：
//           A.给定估测姿态，快速定位，收敛范围小;
//           B.给定估测姿态，扩展定位(较慢，要求机器人不动)，收敛范围大
//
///////////////////////////////////////////////////////////////////////////////

class DllExport CNdtBaseLocalization
{
  private:
//    double max_translation_norm;    // 定位结果与估测姿态的最大允许距离偏差
//    double max_rotation;            // 定位结果与估测姿态的最大允许角度偏差
    // consistency Chenck
    double max_translation_x;                // x方向 最大允许距离偏差
    double max_translation_y;                // y方向 最大允许距离偏差
    double max_rotation_z;                   // 最大允许角度偏差

  public:
    double map_size_z;
    double resolution;
    double sensor_max_range;    // 扫描最远距离
    double sensor_min_range;    // 扫描最近距离
    NDTMap *map;                // NDT图

  public:
    NDTMatcherD2D_2D matcher2D;    // NDT匹配器
    ndtInform_oru *mp_Informer;
    NDTMatchInfo matchInfo;

  private:
    // 根据里程姿态和扫描点云进行定位
    bool LocalizeNewton(const CPointCloud &cloud_in, Eigen::Affine3d &odometry);

  public:
    // 对输入的点云进行处理，滤除过近的点，并对Z坐标进行初始化
    void FilterCloud(const CPointCloud &cloud_in, CPointCloud &cloud_filtered);

  public:
    CNdtBaseLocalization(double map_reso = DEFAULT_MAP_RESO);
    ~CNdtBaseLocalization();

    // 设置定位所用的NDT图
    bool SetMap(NDTMap *_map);

    // 设置“一致性核查”参数
    void SetConsistencyCheck(double x = 0, double y = 0, double r = 0);

    // 根据新的数据，仅进行定位处理
    virtual bool Localize(const CPointCloud &cloud_in, const Eigen::Affine3d &Tinit, Eigen::Affine3d &Testimate);

    // 在指定的范围内进行扩展定位
    int LocalizeEx(const CPointCloud &cloud_in, Eigen::Affine3d &odometry);

    // 清除匹配状态
    void ClearMatchStatus();

    // By Sam
    void SetLaserParams(CScannerGroupParam *Params);

#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
}    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
