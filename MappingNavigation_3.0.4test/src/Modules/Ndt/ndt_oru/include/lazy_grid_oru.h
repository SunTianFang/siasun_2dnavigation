#pragma once

#include <stdio.h>
#include "spatial_index.h"
#include "ndt_cell_oru.h"
#include "Geometry.h"

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#endif

#define DEFAULT_MAP_SIZE_X 500    // 地图缺省宽度为200m
#define DEFAULT_MAP_SIZE_Y 500    // 地图缺省高度为200m
#define DEFAULT_MAP_RESO 0.2      // 地图缺省分辨率0.2m

namespace ndt_oru
{
///////////////////////////////////////////////////////////////////////////
//   “LazyGrid”用来定义一种栅格化的空间管理方案。
class DllExport LazyGrid : public SpatialIndex
{
  protected:
    NDTCell ****dataArray_;       // NDT单元缓冲区首指针
    NDTCell *protoType;           // 样板NDT单元，用来照此样板生成新单元
    bool cellSizeIsSet_;          // 单元格的尺寸是否已设置
    bool centerIsSet_;            // 区域的中心点是否已设置
    bool sizeIsSet_;              // 区域的范围是否已设置
    CSize3D<double> size_;        // 整个区域的维度
    CSize3D<double> cellSize_;    // Cell的维度
    CSize3D<int> cellsCount_;     // 区域在X/Y/Z方向上含有NDT单元的数量
    Eigen::Vector3d center_;      // 区域中心点
    CRectangle rect_;             // 有效覆盖区域

  protected:
    // 更新边界值
    void UpdateCoveringRect();

  public:
    // 构建立方体栅格(cellSize需要一开始就设置好，但可以后续改变)
    LazyGrid(double cellSize);
    LazyGrid();

    // 根据另外一个对象构造
    LazyGrid(const LazyGrid &another);

    virtual ~LazyGrid();

    // Copy构造函数
    LazyGrid &operator=(const LazyGrid &another);

    LazyGrid &getLazyGrid() { return *this; }

    // 配置过程之一：设置单元格尺寸
    virtual bool setCellSize(double cellSizeX, double cellSizeY, double cellSizeZ);

    // 配置过程之二：设置空间管理范围的中心
    virtual bool setCenter(const Eigen::Vector3d &pt);

    // 配置过程之三：设置空间管理范围
    virtual bool setSize(double sx, double sy, double sz);

    // 清除所有单元格的数据
    void Clear();

    // 将地图重新复位成初始状态(但不释放NDT单元入口表)
    void Reset();

    // 取得指定点处所对应的单元
    virtual NDTCell *getCellForPoint(const Eigen::Vector3d &pt, bool allocIfNeeded = false);

    // 向LazyGrid中加入一个空间点
    virtual NDTCell *addPoint(const Eigen::Vector3d &point);

    /// clone - create an empty object with same type
    virtual SpatialIndex *clone() const;

    // 设置单元(cell)类型，以便进行“工厂式”复制
    virtual void setCellType(NDTCell *type);

    // 删除指定序号的单元
    bool deleteCell(int nIdx);

    // 取得指定序号的单元的指针
    NDTCell *getCell(int nIdx);

    // 计算与给定点的邻近距离不超过n_neigh的所有单元，并返回这些单元的列表
    virtual std::vector<NDTCell *> getClosestNDTCells(const Eigen::Vector3d &pt, int n_neigh) const;

    // 取得在指定索引位置处的NDT单元
    inline NDTCell *getCellAt(int ix, int iy, int iz)
    {
        // 验证索引值范围
        if (ix >= cellsCount_.x || iy >= cellsCount_.y || iz >= cellsCount_.z || ix < 0 || iy < 0 || iz < 0)
            return NULL;

        if (dataArray_ == NULL || dataArray_[ix] == NULL || dataArray_[ix][iy] == NULL)
            return NULL;

        return dataArray_[ix][iy][iz];
    }

    void getCellSize(double &cx, double &cy, double &cz);
    void getGridSize(int &cx, int &cy, int &cz);

    Eigen::Vector3d getCenter() { return center_; }

    // 根据给定的空间点的位置，计算对应的NDT单元的地址。
    virtual void getIndexForPoint(const Eigen::Vector3d &pt, int &idx, int &idy, int &idz) const;

    NDTCell *getProtoType() { return protoType; }

    virtual bool initialize();

    bool traceLine(const Eigen::Vector3d &origin, const Eigen::Vector3d &endpoint, const Eigen::Vector3d &diff,
                   const double &maxz, std::vector<NDTCell *> &cells);

    virtual bool checkCellforNDT(int indX, int indY, int indZ) const;

    // 向图中加入新的NDT单元
    NDTCell *AddNewCell(NDTCell &newCell);

    // 取得覆盖区域
    CRectangle GetCoveringRect() const { return rect_; }

    // 从文件中装入LazyGrid
    virtual bool Load(FILE *fp);

    // 将LazyGrid保存到文件
    virtual bool Save(FILE *fp);

#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
};    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
