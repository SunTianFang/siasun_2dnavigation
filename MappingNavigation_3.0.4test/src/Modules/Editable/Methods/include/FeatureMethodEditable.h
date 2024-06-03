#pragma once

#include "FeatureMethod.h"
#include "MethodEditable.h"

///////////////////////////////////////////////////////////////////////////////
//   声明“CFeatureMethodEditable”类，它是CFeatureMethod类的可编辑版本。
class DllExport CFeatureMethodEditable : public CFeatureMethod, public CMethodEditable
{
  private:
    bool lineVisible_;
    bool pointVisible_;
    bool matchInfoShow_;

  protected:
    virtual CFeatureMap *CreateMap();

  public:
    CFeatureMethodEditable();

    // 设置特征图可见/隐藏
    void SetVisible(bool lineVisible, bool pointVisible)
    {
        lineVisible_ = lineVisible;
        pointVisible_ = pointVisible;
    }

    // 绘制对应于该方法的地图
    virtual void PlotNormalMap(QPainter *painter, CScreenReference &scrnRef);

    // 绘制该方法中所有选中对象
    virtual void PlotSelectedObj(QPainter *painter, CScreenReference &scrnRef);

    // 绘制对应于该方法的定位情况
    virtual void PlotLocalization(QPainter *painter, CScreenReference &scrnRef);

    // 显示对应于该方法的匹配数据
    virtual void ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
                                 int totalSteps);

    virtual void ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, CFeatureMatchInfo matchInfoOutSide);

};
