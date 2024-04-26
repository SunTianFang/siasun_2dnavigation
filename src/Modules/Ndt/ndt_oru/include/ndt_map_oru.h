#pragma once

#include <set>
#include "lazy_grid_oru.h"
#include "AppArea.h"
#include "AffinePosture.h"

#ifdef NDT1_USE_MINI_EIGEN
#include <pcl.h>
#define Eigen MiniEigen
#endif

class CPnt;

namespace ndt_oru
{

//
//   NDT图生成参数。
//
struct DllExport CSubmapParam
{
    double cellSize;    // NDT单元格边长
    double mapSizeX;    // 地图X方向长度的一半
    double mapSizeY;    // 地图Y方向长度的一半
    // 应用区域 ...尚未实现
};

class DllExport NDTMap : public LazyGrid
{
  private:
    bool isFirstLoad_;    // 是否是第一次装入
    std::set<NDTCell *> update_set;

  public:
    CAppArea appArea_;

  public:
    bool isTemp_;    // 是否是临时子图
    Eigen::Affine3d homePose_;

    std::vector<ndt_oru::NDTCell *> getAllCells(bool checkGaussian = true) const;  // By Sam: change private to public

  private:
    //
    //   针对update_set中记录的单元(需要更新计算的各单元)，计算它们的均值和协方差，
    //
    void computeNDTCells(Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0), unsigned int maxnumpoints = 1e9,
                         float occupancy_limit = 255, double sensor_noise = 0.1);

    //   为指定点(umean)处的单元添加由ucov表示的分布。
    bool addDistributionToCell(const Eigen::Matrix3d &ucov, const Eigen::Vector3d &umean,
                               unsigned int numpointsindistribution, unsigned int maxnumpoints = 1e9,
                               float max_occupancy = 1024);

    // 取得所有的单元并作为一个向量返回
//    std::vector<ndt_oru::NDTCell *> getAllCells(bool checkGaussian = true) const;

#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  public:
    // 缺省构造函数
    NDTMap(double cellSize = DEFAULT_MAP_RESO) : LazyGrid(cellSize)
    {
        isFirstLoad_ = true;
        isTemp_ = true;    // 只有从文件中装入的子图才是永久的
        homePose_ = PostureToAffine(0, 0, 0);
    }

    // 拷贝构造函数
    NDTMap(const NDTMap &other) { *this = other; }

    virtual ~NDTMap() { Clear(); }

    // 返回对象指针(主要用于继承类中)
    NDTMap *getNDTMapPtr() { return this; }

    virtual void Clear()
    {
        LazyGrid::Clear();
        isFirstLoad_ = true;
        isTemp_ = true;
        homePose_ = PostureToAffine(0, 0, 0);
    }

    // 清除该子图的应用区域表
    void ClearAppArea()
    {
        appArea_.Clear();
    }

    // 增加一个矩形应用区域
    bool AddAppRect(CRectangle *appRect)
    {
        appArea_ += appRect;
        return true;
    }

    // 重载“=”操作符
    virtual void operator=(const NDTMap &other)
    {
        getLazyGrid() = ((NDTMap &)other).getLazyGrid();
        isFirstLoad_ = other.isFirstLoad_;
        isTemp_ = other.isTemp_;

        homePose_ = other.homePose_;
    }

    // 设置NDT图的中心点和尺寸
    bool initialize(const Eigen::Vector3d &center, double sizex, double sizey, double sizez);

    // 根据给定的子图参数和初始位姿，对子图进行初始化
    bool initialize(const Eigen::Affine3d &homePose, const CSubmapParam &submapParam);

    // 根据给定的初始位姿，对子图进行复位(不重新分配空间，清空所有单元，重新设置中心位置)
    virtual void Reset(const Eigen::Affine3d &homePose);

    //   装入第一帧点云
    //   注：
    //     1. 每次调用此方法将删除先前形成的NDT图
    //     2. 此方法将自动计算点云的中心点，并将其作为NDT图的中心点p(0, 0, 0)
    //     3. 全图范围需要预先通过guessSize函数提供
    //     3. 此方法只简单地装入一帧点云，不支持对模型的融合
    bool loadInitialPointCloud(const CPointCloud &pc, double range_limit = -1);

    //
    //   向NDT图中装入一个点云，要求点云的中心点与原NDT图的中心点对准。
    //     - pc : 将要装入的点云
    //     - origin:  The desired origin of the map (will be fitted acording to old_centroid)
    //     - old_centroid: The centroid to which we want to align
    //     - map_size: 新地图的尺寸
    //
    //   注：调用此方法将清除原有NDT图。
    //
    bool loadPointCloudCentroid(const CPointCloud &pc, const Eigen::Vector3d &origin,
                                const Eigen::Vector3d &old_centroid, const Eigen::Vector3d &map_size,
                                double range_limit);

    //
    //   这是NDT图生成的主要方法，它将一个点云加入到NDT图中，同时执行扫描线追踪、
    //   更新冲突单元，并将点云记录到各单元中。
    //
    //    - origin: 生成点云的传感器所在的姿态
    //    - pc:     点云
    //    - maxz:   点云Z坐标的最大值(超过的点会被丢弃)
    //    - sensor_noise: 传感器噪声的标准差
    //
    bool addPointCloud(const Eigen::Vector3d &origin, const CPointCloud &pc, double maxz = 100.0,
                       double sensor_noise = 0.25, double occupancy_limit = 255);

    //
    //   添加一个新点云到NDT图中，利用由一次测量产生的局部图中的均值来更新占据格，
    //   其中需要：进行扫描线追踪、更新冲突单元，并将点添加到单元中
    //
    bool addPointCloudMeanUpdate(const Eigen::Vector3d &origin, const CPointCloud &pc,
                                 const Eigen::Vector3d &localmapsize, unsigned int maxnumpoints = 1e9,
                                 float occupancy_limit = 255, double maxz = 100.0, double sensor_noise = 0.25);

    // 将经过变换的NDT图转为一组NDT单元的向量后返回
    VectNDTCells pseudoTransformNDT(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> T) const;

    // 取得单元的数量
    int getCellsCount(bool checkGaussian = true) const;

    // 取得指定序号的单元中心点的位置
    bool GetCellPnt(int nIdx, CPnt &pt);

    // 清除所有单元的匹配标志字
    void ClearMatchStatus();

    // 对全图进行坐标变换
    void Transform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr);

    // 将另外一个NDT图融合到当前NDT图中
    bool FuseNdtMap(const NDTMap &other, const Eigen::Vector3d &origin, unsigned int maxnumpoints,
                    float occupancy_limit, double maxz, double sensor_noise);

    // 对NDT图进行重采样，生成点云
    void Resample(double reso, CPointCloud &resampledCloud);

    // 判断给定的点是否处于该NDT子图的应用区域中
    bool AppAreaContain(const CPnt &pt);

    // 取得所有的关键单元并作为一个向量返回
    std::vector<ndt_oru::NDTCell *> getAllKeyCells() const;

    // 从文件中装入NDT图
    virtual bool Load(FILE *fp);

    // 将NDT图保存到文件
    virtual bool Save(FILE *fp);

    // By Sam: 拼接
    bool MergeCellsToMap(NDTCell& newCell);
    //By yu
    double getMapResolution(){return cellSize_.x;}
};

// 变换点云的参考系
DllExport void transformPointCloudInPlace(const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T, CPointCloud &pc);

// 结合给定的范围限制值，统计一个点云的重心点
DllExport Eigen::Vector3d getCloudCentroid(const CPointCloud &cloud, double range_limit);

// 结合给定的范围限制值，计算给定点云的范围参数及重心点
DllExport Eigen::Vector3d getCloudDimension(const CPointCloud &cloud, double range_limit, double &maxDist, double &minZ,
                                  double &maxZ);

}    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
