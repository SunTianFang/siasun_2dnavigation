#pragma once

#include "GraphicObj.h"
#include "PointFeature.h"

class DllExport CPointFeatureEditable : public CPointFeature, public CGraphicObj
{
  public:
    CPointFeatureEditable() {}

    // 生成一个复本
    virtual CPointFeature *Duplicate() const;

    // 判断指定的点是否触碰到该点特征
    virtual bool PointHit(const CPnt &pt, float thresh);

#ifdef _MFC_VER
    // 在屏幕上绘制此点特征
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, COLORREF crSelected,
                      int nPointSize, int nLineWidth = 0);

    // 在屏幕上绘制此点特征的ID号
    virtual void PlotId(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor);

#elif defined QT_VERSION
    // 在屏幕上绘制此点特征
    virtual void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor,
                      QColor crSelected, float fPointSize, int nMinRadius, int nLineWidth = 0);

    // 在屏幕上绘制此点特征的ID号
    virtual void PlotId(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor);
#endif
};
