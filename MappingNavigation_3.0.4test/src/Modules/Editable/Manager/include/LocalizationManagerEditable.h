#pragma once

#include "LocalizationManager.h"

class CVisibilitySetting;

///////////////////////////////////////////////////////////////////////////////
//   声明“CLocalizationManagerEditable”类，它是“CLocalizationManager”类的可编辑版本。
class DllExport CLocalizationManagerEditable : public CLocalizationManager
{
  private:
    int methodUsedIdx_;    // 最新一次定位所使用的方法的编号

  public:
    CLocalizationManagerEditable();

    virtual bool Create();

    // 定位过程函数
    virtual CMatchInfo *Localize(Eigen::Affine3d &estimatePose);

    // 绘制所有定位方法的地图
    void PlotNormalMap(QPainter *painter, CScreenReference &scrnRef);

    // 绘制所有定位方法中的选中对象
    void PlotSelectedObj(QPainter *painter, CScreenReference &scrnRef);

    // 绘制当前定位方法的定位情况
    void PlotLocalization(QPainter *painter, CScreenReference &scrnRef);

    // 显示当前定位的匹配数据
    void ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep, int totalSteps);

    // 更新各种定位方法所用地图的可视属性
    void UpdateVisibility(const CVisibilitySetting &vis);
};
