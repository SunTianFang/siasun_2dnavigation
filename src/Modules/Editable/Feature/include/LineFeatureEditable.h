#pragma once

#include "GraphicObj.h"
#include "LineFeature.h"

class CLineFeatureEditable : public CLineFeature, public CGraphicObj
{
  public:
    CLineFeatureEditable() {}

    // 生成一个复本
    virtual CLineFeature *Duplicate() const;

#ifdef _MFC_VER
    // 在屏幕上显示此直线特征
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF clr, COLORREF clrSelected, int nSize);

#elif defined QT_VERSION
    virtual void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, QColor clrSelected, int nSize);
#endif
};
