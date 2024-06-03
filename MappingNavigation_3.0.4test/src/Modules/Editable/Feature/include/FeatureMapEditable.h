#pragma once

#include "FeatureMap.h"

class CFeatureMapEditable : public CFeatureMap
{
  protected:
    virtual CPointFeatureSet *CreatePointFeatureSet();
    virtual CLineFeatureSet *CreateLineFeatureSet();

  public:
    CFeatureMapEditable() {}

    // 收集特征图中的所有图形对象
    //    void CollectGraphicObjs(CGraphicObjs &GraphicObjs);

#ifdef _MFC_VER
    // 绘制特征图
    void Plot(CScreenReference &ScrnRef, CDC *pDc, COLORREF clrPoint, COLORREF clrLine);
#elif defined QT_VERSION
    // 绘制特征图
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrPoint, QColor clrLine);
#endif
};
