#pragma once

#include <QPainter>
#include "LocalizationMethods.h"
#include "ScrnRef.h"
#include "TreeSelector.h"

///////////////////////////////////////////////////////////////////////////////

class DllExport CLocalizationMethodsEditable : public CLocalizationMethods
{
  public:
    virtual bool Create();

    // 绘制所有定位方法的地图
    void PlotNormalMap(QPainter *painter, CScreenReference &scrnRef);

    // 绘制所有定位方法中的选中对象
    void PlotSelectedObj(QPainter *painter, CScreenReference &scrnRef);

    // 绘制当前定位方法的定位情况
    void PlotLocalization(QPainter *painter, CScreenReference &scrnRef);

    // 更新各种定位方法所用地图的可视属性
    void UpdateVisibility(const CVisibilitySetting &vis);
};
