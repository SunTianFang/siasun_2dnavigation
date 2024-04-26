#pragma once

#include "ndt_maps_oru.h"
#include "NdtMapEditable.h"

#ifdef _MFC_VER
class CDC;
#elif defined QT_VERSION
class QPainter;
class QColor;
#endif

namespace ndt_oru
{
class DllExport NDTMapsEditable : public NDTMaps
{
  protected:
    virtual NDTMap *CreateSubmap() { return new NDTMapEditable; }

  public:
    NDTMapsEditable() {}

    // 判断一个给定的点是否“触碰”到图中的某个单元
    int PointHitCell(const CPnt &pt, float fDistGate, bool &bHitGaussianEllipse);

    // 判断一个给定的点是否“触碰”到图中的某个参考位姿
    int PointHitHomePose(const CPnt &pt, float fDistGate);

    // 选中/不选中指定的子图
    void SetSelected(int subMapId = -1, bool selected = true);

    // 使指定的子图可见/不可见
    void SetVisible(int subMapId = -1, bool visible = true);

    // 判断指定的子图是否可见
    bool IsVisible(int submapId);

    // 选中/取消选中给定矩形中的所有的高斯单元
    void SelCellsInRect(const CRectangle &r, bool bSelect = true);

    // 所有子图的所有单元都置为不选中
    void UnselAllCells();

    // 取得选中单元的数量
    int CountSelectedCells();

    // 删除所有被选择的(selected)单元
    bool DeleteAllSelectedCells();

    // 取得当前唯一被选中的子图编号
    int GetTheOnlySelected();

    // 取得选中子图的数量
    int CountSubmapsSelected();

    // By Sam
    NDTMapEditable *GetMap(int subMapID);

#ifdef _MFC_VER
    void Plot(CDC *pDC, CScreenReference &ScrnRef, unsigned long clrCellFill, bool bShowMatched = false,
              unsigned long clrMatched = 0, bool bShowGaussianCells = false);

    // 显示各子图的ID号
    void ShowId(CDC *pDC, CScreenReference &ScrnRef, int submapId);

    // 显示匹配线
    void PlotMatchStatus(CDC *pDc, CScreenReference &ScrnRef, unsigned long clrMatchLine);

#elif defined QT_VERSION
    void Plot(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrCellFill, bool bShowMatched = false,
              QColor clrMatched = Qt::black, bool bShowGaussianCells = false);

    // 显示各子图的ID号
    void ShowId(QPainter *pPainter, CScreenReference &ScrnRef, int submapId);

    // 显示匹配线
    void PlotMatchStatus(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrMatchLine);
#endif
};
}    // namespace ndt_oru
