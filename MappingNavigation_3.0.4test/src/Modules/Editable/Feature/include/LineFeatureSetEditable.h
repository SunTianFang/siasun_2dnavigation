#pragma once

#include "LineFeatureSet.h"

class DllExport CLineFeatureSetEditable : public CLineFeatureSet
{
  public:
    // 根据直线特征类型分配空间
    virtual CLineFeature *NewLineFeature(int nType);

  public:
    CLineFeatureSetEditable() {}

    // 取得选中特征直线的数量
    int GetCountSelected();

    // 选中/取消选中指定的直线特征
    void Select(bool sel, int index = -1);

    // 判断指定的特征直线是否被选中
    bool IsSelected(int index);

    // 删除所有选中的直线特征
    void DeleteAllSelected();

    // 判断一个给定点是否触碰到某一条直线特征(用于屏幕上直线触碰判断)
    int PointHit(const CPnt &pt, float fDistGate);

#ifdef _MFC_VER
    void Dump();

    // 绘制直线特征集合
    void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, COLORREF crSelected, int nSize = 3);

#elif defined QT_VERSION
    // 绘制直线特征集合
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, QColor clrSelected, int nSize = 3);

    // 在屏幕上绘制此集合中所有被选中者
    void PlotSelected(CScreenReference &ScrnRef, QPainter *pPainter, QColor cr, int nSize);
#endif
};
