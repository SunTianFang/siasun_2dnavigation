#pragma once

#include <vector>
#include "ndt_map_oru.h"

#ifdef NDT1_USE_MINI_EIGEN
#include <pcl.h>
#define Eigen MiniEigen
#endif

#ifdef _MFC_VER
class CDC;
#elif defined QT_VERSION
class QPainter;
class QColor;
#endif

class CScreenReference;
class CPnt;
class CRectangle;

using namespace std;

namespace ndt_oru
{
///////////////////////////////////////////////////////////////////////////////
// 定义可以进行图形显示及交互处理的NDT子图。
class DllExport NDTMapEditable : public NDTMap
{
  public:
    bool isSelected_;    // 该图是否被选中
    bool isVisible_;     // 该图是否显现

  public:
    // 缺省构造函数
    NDTMapEditable(double cellSize) : NDTMap(cellSize)
    {
        isSelected_ = false;
        isVisible_ = true;
    }

    NDTMapEditable() : NDTMap(DEFAULT_MAP_RESO)
    {
        isSelected_ = false;
        isVisible_ = true;
    }

    // 拷贝构造函数
    NDTMapEditable(const NDTMapEditable &other) { *this = other; }

    // 清空该子图
    virtual void Clear()
    {
        NDTMap::Clear();

        isSelected_ = false;
        isVisible_ = true;
    }

    // 重载“=”操作符
    void operator=(const NDTMapEditable &other)
    {
        // 先复制基类成员
        *(getNDTMapPtr()) = other;

        isSelected_ = other.isSelected_;
        isVisible_ = other.isVisible_;
    }

    // 删除指定矩阵区域内的所有单元
    bool DeleteCellsInArea(const CRectangle &r);

    // 取得选中单元的数量
    int CountSelectedCells();

    // 删除所有被选择的(selected)单元
    bool DeleteAllSelectedCells();

#if 0
    // 删除指定的应用区域矩形
    bool DeleteAppRect(int index);
#endif

    // 对图中选中的单元进行坐标变换
    void TransformSelectedCells(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr);

    // 收集所有选中单元的中心点
    vector<CPnt> CollectCenterOfSelectedCells();

    // 判断一个给定的点是否“触碰”到图中的某个单元
    int PointHitCell(const CPnt &pt, float fDistGate, bool &bHitGaussianEllipse);

    // 判断一个给定的点是否“触碰”到图中的参考位姿
    bool PointHitHomePose(const CPnt &pt, float fDistGate);

    // 判断一个给定的点是否“触碰”到NDT图应用区域中的某一个
    int PointHitAppArea(const CPnt &pt, float fDistGate);

    // 选中/取消选中给定矩形中的所有的高斯单元
    void SelCellsInRect(const CRectangle &r, bool bSelect = true);

    // 所有单元都置为不选中
    void UnselAllCells();

#ifdef _MFC_VER
    void Plot(CDC *pDc, CScreenReference &ScrnRef, unsigned long clrCellFill, bool bShowMatched = false,
              unsigned long clrMatched = 0, bool bShowGaussianCells = false);

    // 显示匹配线
    void PlotMatchStatus(CDC *pDc, CScreenReference &ScrnRef, unsigned long clrMatchLine);

#elif defined QT_VERSION
    void Plot(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrCellFill, bool bShowMatched = false,
              QColor clrMatched = Qt::black, bool bShowGaussianCells = false);

    // 显示匹配线
    void PlotMatchStatus(QPainter *pPainter, CScreenReference &ScrnRef, unsigned long clrMatchLine);
#endif
};
}    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
