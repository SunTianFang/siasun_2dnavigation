#include <stdafx.h>
#include "NdtMethodEditable.h"
#include "MapFuser.h"

#define NORNAL_CELL_COLOR QColor(0, 100, 147)

///////////////////////////////////////////////////////////////////////////////
//   定义“CNdtMethodEditable”类，它是CNdtMethod类的可编辑版本。

CNdtMethodEditable::CNdtMethodEditable()
{
    Create();
    visible_ = true;

    // 缺省关闭对匹配源/目标单元的显示
    showSourceMatched_ = false;
    showTargetMatched_ = false;
}

bool CNdtMethodEditable::Create()
{
    // 释放在基类中已分配的maps_和localization_
    if (maps_ != NULL)
        delete maps_;

    if (localization_ != NULL)
        delete localization_;

    // 另行分配editable版本的maps_
    maps_ = new ndt_oru::NDTMapsEditable;
    if (maps_ == NULL)
        return false;

    // 另行分配editable版本的localization_
    localization_ = new ndt_oru::CMapFuser();
    if (localization_ == NULL)
        return false;

    return true;
}

//
//   绘制对应于该方法的地图。
//
void CNdtMethodEditable::PlotNormalMap(QPainter *painter, CScreenReference &scrnRef)
{
    if (!visible_)
        return;

    ndt_oru::CMapFuser *mapFuser = dynamic_cast<ndt_oru::CMapFuser *>(localization_);
    if (mapFuser == NULL)
        return;

    mapFuser->PlotModelMap(painter, scrnRef, NORNAL_CELL_COLOR);
}

//
//   绘制该方法中所有选中对象。
//
void CNdtMethodEditable::PlotSelectedObj(QPainter *painter, CScreenReference &scrnRef)
{
}

//
//   绘制对应于该方法的定位情况。
//
void CNdtMethodEditable::PlotLocalization(QPainter *painter, CScreenReference &scrnRef)
{
    ndt_oru::CMapFuser *mapFuser = dynamic_cast<ndt_oru::CMapFuser *>(localization_);
    if (mapFuser == NULL)
        return;

//    mapFuser->PlotLocalization(painter, scrnRef, showSourceMatched_, showTargetMatched_, false,
//                               false);
    mapFuser->PlotLocalization(painter, scrnRef, true, true, true, false);
}

//
//   显示对应于该方法的匹配数据。
//
void CNdtMethodEditable::ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
                                         int totalSteps)
{
    int submapId = GetCurSubmapId();
    localization_->matcher2D.ShowStatus(painter, scrnRef, curStep, totalSteps, submapId, Qt::black);

    // Sam Add
    painter->setPen(Qt::black);
    QString str;

    str.sprintf("Ndt Match Info: ");
    painter->drawText(10, 95, str);

    str.sprintf("lx: %d, ly: %d, lpx: %d, lpy: %d, mx: %d, my: %d,mo: %d", (int)localization_->mp_Informer->lx,
                localization_->mp_Informer->ly, localization_->mp_Informer->lpx, localization_->mp_Informer->lpy,
                localization_->mp_Informer->mx, localization_->mp_Informer->my,localization_->mp_Informer->mo);//Add By yu. same as blackbox.
    painter->drawText(10, 120, str);

    str.sprintf("ratioX: %f, ratioY: %f", (float)localization_->mp_Informer->lpx / localization_->mp_Informer->lx,
                (float)localization_->mp_Informer->lpy / localization_->mp_Informer->ly);
//    str.sprintf("uG: %d, uN %d",)
    painter->drawText(10, 145, str);

    // By Sam
//    localization_->mp_Informer->laserLinePlot(scrnRef, painter);


#if 0
    if (curStep >= 2)
    {
        int nAveTime, nMinTime, nMaxTime;
        UpdateMatchStatistics(nAveTime, nMaxTime, nMinTime);

        QString str;
        str.sprintf("平均用时: %d, 最小: %d, 最大: %d", nAveTime, nMinTime, nMaxTime);
        painter->setPen(Qt::black);
        painter->drawText(10, 95, str);
    }
#endif
}
