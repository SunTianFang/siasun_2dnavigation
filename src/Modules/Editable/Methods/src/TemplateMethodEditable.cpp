#include <stdafx.h>
#include "TemplateMethodEditable.h"
#include "StaticObjectsEditable.h"

///////////////////////////////////////////////////////////////////////////////
//   定义“CTemplateMethodEditable”类，它是CTemplateMethod类的可编辑版本。

CTemplateMethodEditable::CTemplateMethodEditable()
{
    map_ = CreateMap();
    visible_ = true;
}

CStaticObjects *CTemplateMethodEditable::CreateMap()
{
    return new CStaticObjectsEditable;
}

//
//   绘制对应于该方法的地图。
//
void CTemplateMethodEditable::PlotNormalMap(QPainter *painter, CScreenReference &scrnRef)
{
    if (!visible_)
        return;

    CStaticObjectsEditable *objs = dynamic_cast<CStaticObjectsEditable *>(map_);
    if (objs == NULL)
        return;

    objs->Plot(scrnRef, painter, Qt::darkGreen, 2);
}

//
//   绘制该方法中所有选中对象。
//
void CTemplateMethodEditable::PlotSelectedObj(QPainter *painter, CScreenReference &scrnRef)
{
}

//
//   绘制对应于该方法的定位情况。
//
void CTemplateMethodEditable::PlotLocalization(QPainter *painter, CScreenReference &scrnRef)
{
}

void CTemplateMethodEditable::ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
                             int totalSteps)
{
    QString str;
    str.sprintf("第%d帧 (共%d帧) ", curStep, totalSteps);
    painter->setPen(Qt::black);
    painter->drawText(10, 20, str);

    // 显示定位评价数据
    str.sprintf("csm匹配点对,点->线距离(m): %6f", matchInfo_.error);
    painter->drawText(10, 50, str);

    str.sprintf("csm匹配点对,点->点距离(m): %6f", matchInfo_.error2);
    painter->drawText(10, 80, str);

    str.sprintf("覆盖面积(%): %4f% ", matchInfo_.correspondencePercent * 100);
    painter->drawText(10, 110, str);

    str.sprintf("匹配点对个数: %d ", matchInfo_.nvalid);
    painter->drawText(10, 140, str);

    str.sprintf("角度分布相似性: %3f% ", matchInfo_.histCorrelation * 100);
    painter->drawText(10, 170, str);


}
