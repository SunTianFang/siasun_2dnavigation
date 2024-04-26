#pragma once

#include "TemplateMethod.h"
#include "MethodEditable.h"

///////////////////////////////////////////////////////////////////////////////
//   声明“CTemplateMethodEditable”类，它是CTemplateMethod类的可编辑版本。
class DllExport CTemplateMethodEditable : public CTemplateMethod, public CMethodEditable
{
  private:
    bool visible_;

  protected:
    virtual CStaticObjects *CreateMap();

  public:
    CTemplateMethodEditable();

    // 设置模板图可见/隐藏
    void SetVisible(bool visible) { visible_ = visible; }

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
