#pragma once

#include <QPainter>
#include "ScrnRef.h"

class DllExport CMethodEditable
{
  public:
    CMethodEditable() {}

    // 绘制对应于该方法的地图
    virtual void PlotNormalMap(QPainter *painter, CScreenReference &scrnRef) {}

    // 绘制该方法中所有选中对象
    virtual void PlotSelectedObj(QPainter *painter, CScreenReference &scrnRef) {}

    // 绘制对应于该方法的定位情况
    virtual void PlotLocalization(QPainter *painter, CScreenReference &scrnRef) {}

    // 显示对应于该方法的匹配数据
    virtual void ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
                                 int totalSteps)
    {
    }
};
