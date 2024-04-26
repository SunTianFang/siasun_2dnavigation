#pragma once

#include "NdtMethod.h"
#include "MethodEditable.h"

///////////////////////////////////////////////////////////////////////////////
//   声明“CNdtMethodEditable”类，它是CNdtMethod类的可编辑版本。
class DllExport CNdtMethodEditable : public CNdtMethod, public CMethodEditable
{
  private:
    bool visible_;
    bool showSourceMatched_;    // 是否要显示匹配的source单元
    bool showTargetMatched_;    // 是否要显示匹配的target单元

  protected:
    virtual bool Create();

  public:
    CNdtMethodEditable();

    // 设置NDT图可见/隐藏
    void SetVisible(bool visible) { visible_ = visible; }

    // 打开/关闭对匹配的源单元的显示
    void ShowSourceMatched(bool show) { showSourceMatched_ = show; }

    // 打开/关闭对匹配的目标单元的显示
    void ShowTargetMatched(bool show) { showTargetMatched_ = show; }

    // 绘制对应于该方法的地图
    virtual void PlotNormalMap(QPainter *painter, CScreenReference &scrnRef);

    // 绘制该方法中所有选中对象
    virtual void PlotSelectedObj(QPainter *painter, CScreenReference &scrnRef);

    // 绘制对应于该方法的定位情况
    virtual void PlotLocalization(QPainter *painter, CScreenReference &scrnRef);

    // 显示对应于该方法的匹配数据
    virtual void ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
                                 int totalSteps);
};
