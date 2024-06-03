#pragma once

#include "GraphicObj.h"
#include "PointFeatureSet.h"

class DllExport CPointFeatureSetEditable : public CPointFeatureSet
{
  protected:
    // 根据所提供的特征类型分配空间
    virtual CPointFeature *NewPointFeature(int nSubType);

  public:
    CPointFeatureSetEditable();

    // 取得选中特征点的数量
    int GetCountSelected();

    // 判断指定的特征点是否被选中
    bool IsSelected(int index);

    // 选中/取消选中指定的点特征
    void Select(bool sel, int nIdx = -1);

    // 删除所有选中的点特征
    void DeleteAllSelected();

    // 判断指定的点是否触碰到某个点特征
    virtual int PointHit(const CPnt &pt, float thresh);

#ifdef _MFC_VER
    void Dump();

    // 在屏幕上绘制此点特征集合
    void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF cr, COLORREF crSelected, int nLineWidth = 0,
              bool bShowId = false);

#elif defined QT_VERSION
    // 在屏幕上绘制此点特征集合
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor cr, QColor crSelected, int nSize = 1,
              bool bShowId = false, float mmSize = 0);

    // 在屏幕上绘制此集合中所有被选中者
    void PlotSelected(CScreenReference &ScrnRef, QPainter *pPainter, QColor cr, int nSize = 1, float mmSize = 0);
#endif
};
