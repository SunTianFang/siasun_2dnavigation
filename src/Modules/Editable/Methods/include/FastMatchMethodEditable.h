#pragma once

#include "FastMatchMethod.h"
#include "MethodEditable.h"

///////////////////////////////////////////////////////////////////////////////
//   声明“CFastMatchMethodEditable”类，它是CFastMatchMethod类的可编辑版本。
class DllExport CFastMatchMethodEditable : public CFastMatchMethod, public CMethodEditable
{
  private:
    bool visible_;
    bool showInitPose;    // 是否要显示匹配的source单元
    bool showResultPose;    // 是否要显示匹配的target单元



  public:
    CFastMatchMethodEditable();

    // 设置可见/隐藏
    void SetVisible(bool visible) { visible_ = visible; }

    // 打开/关闭对匹配的源单元的显示
    void SetShowInitPose(bool show) { showInitPose = show; }

    // 打开/关闭对匹配的目标单元的显示
    void SetShowResultPose(bool show) { showResultPose = show; }

    // 绘制对应于该方法的地图
    virtual void PlotNormalMap(QPainter *painter, CScreenReference &scrnRef);


    // 绘制对应于该方法的定位情况
    virtual void PlotLocalization(QPainter *painter, CScreenReference &scrnRef);

    // 显示对应于该方法的匹配数据
    virtual void ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
                                 int totalSteps);
};
