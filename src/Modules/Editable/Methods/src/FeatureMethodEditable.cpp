#include <stdafx.h>
#include "FeatureMethodEditable.h"
#include "FeatureMapEditable.h"
#include "PointFeatureSetEditable.h"
#include "LineFeatureSetEditable.h"

///////////////////////////////////////////////////////////////////////////////
//   定义“CFeatureMethodEditable”类，它是CFeatureMethod类的可编辑版本。

CFeatureMethodEditable::CFeatureMethodEditable()
{
    map_ = CreateMap();
    if (map_ != NULL)
        map_->Init();

    lineVisible_ = true;
    pointVisible_ = true;
    matchInfoShow_ = false;
}

CFeatureMap *CFeatureMethodEditable::CreateMap()
{
    return new CFeatureMapEditable;
}

//
//   绘制对应于该方法的地图。
//
void CFeatureMethodEditable::PlotNormalMap(QPainter *painter, CScreenReference &scrnRef)
{
    // 显示所有直线特征
    if (lineVisible_)
    {
        CLineFeatureSetEditable *lines =
            dynamic_cast<CLineFeatureSetEditable *>(map_->m_pLineFeatures);
        if (lines == NULL)
            return;

        lines->Plot(scrnRef, painter, Qt::magenta, Qt::magenta);
    }

    // 显示所有点特征
    if (pointVisible_)
    {
        CPointFeatureSetEditable *reflectors =
            dynamic_cast<CPointFeatureSetEditable *>(map_->m_pPointFeatures);

        if (reflectors == NULL)
            return;

        reflectors->Plot(scrnRef, painter, Qt::red, Qt::red, 4, false, 40);
    }
}

//
//   绘制该方法中所有选中对象。
//
void CFeatureMethodEditable::PlotSelectedObj(QPainter *painter, CScreenReference &scrnRef)
{
}

//
//   绘制对应于该方法的定位情况。
//
void CFeatureMethodEditable::PlotLocalization(QPainter *painter, CScreenReference &scrnRef)
{
}

// 显示对应于该方法的匹配数据
void CFeatureMethodEditable::ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
                             int totalSteps)
{
    matchInfoShow_ = true;
    QString str;
    str.sprintf("第%d帧 (共%d帧) ", curStep, totalSteps);
    painter->setPen(Qt::black);
    painter->drawText(10, 20, str);
    // 显示定位评价数据
    {
        str.sprintf("当前扫描到的反光板个数 %d(黄色表示)", (int)matchInfo_.vecReflectors_.size());
        painter->drawText(10, 40, str);

        str.sprintf("当前扫描到的直线个数 %d(黄色表示)", (int)matchInfo_.vecLines_.size());
        painter->drawText(10, 60, str);

        str.sprintf("匹配到反光板对数%d(绿色表示),匹配到直线对数%d(绿色表示),总匹配率 %f% ", matchInfo_.vecPointPair_.size(),matchInfo_.vecLinePair_.size(), matchInfo_.matchRatio_);
        painter->drawText(10, 80, str);
    }

    for(int i = 0 ; i < matchInfo_.vecPointPair_.size() ; i++)
    {
        CPnt pLocal =  matchInfo_.vecPointPair_.at(i).m_ptLocalToWorld;
        int nRadius = 30 / 1000.0f * scrnRef.m_fRatio;
        if (nRadius < 3)
            nRadius = 3;
        pLocal.Draw(scrnRef, painter, Qt::green, nRadius);
    }

    for(int i =0 ; i < matchInfo_.vecReflectors_.size(); i++)
    {
         int nRadius = 20 / 1000.0f * scrnRef.m_fRatio;
         if (nRadius < 2)
             nRadius = 2;
         matchInfo_.vecReflectors_.at(i).Draw(scrnRef, painter, Qt::yellow, nRadius );
    }

    for(int i = 0 ; i < matchInfo_.vecLinePair_.size() ; i++)
    {
        CLine lnArrow = matchInfo_.vecLinePair_.at(i).m_lnWorld;
        lnArrow.Draw(scrnRef, painter, Qt::green,2);
    }

    for(int i =0 ; i < matchInfo_.vecLines_.size(); i++)
    {
        CLine lnArrow = matchInfo_.vecLines_.at(i);
        lnArrow.Draw(scrnRef, painter, Qt::yellow,3);
    }

}
void CFeatureMethodEditable::ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef,  CFeatureMatchInfo matchInfoOutSide)
{
    matchInfoShow_ = true;
    QString str;
//    str.sprintf("第%d帧 (共%d帧) ", curStep, totalSteps);
    painter->setPen(Qt::black);
//    painter->drawText(10, 20, str);
    // 显示定位评价数据
    {
        str.sprintf("当前扫描到的反光板个数 %d(黄色表示)", (int)matchInfoOutSide.vecReflectors_.size());
        painter->drawText(10, 40, str);

        str.sprintf("当前扫描到的直线个数 %d(黄色表示)", (int)matchInfoOutSide.vecLines_.size());
        painter->drawText(10, 60, str);

        str.sprintf("匹配到反光板对数%d(绿色表示),匹配到直线对数%d(绿色表示),总匹配率 %f% ", matchInfoOutSide.vecPointPair_.size(),matchInfoOutSide.vecLinePair_.size(), matchInfoOutSide.matchRatio_);
        painter->drawText(10, 80, str);
    }

    for(int i = 0 ; i < matchInfoOutSide.vecPointPair_.size() ; i++)
    {
        CPnt pLocal =  matchInfoOutSide.vecPointPair_.at(i).m_ptLocalToWorld;
        int nRadius = 30 / 1000.0f * scrnRef.m_fRatio;
        if (nRadius < 3)
            nRadius = 3;
        pLocal.Draw(scrnRef, painter, Qt::green, nRadius);
    }

    for(int i =0 ; i < matchInfoOutSide.vecReflectors_.size(); i++)
    {
         int nRadius = 20 / 1000.0f * scrnRef.m_fRatio;
         if (nRadius < 2)
             nRadius = 2;
//       int nRadius = 1;
         matchInfoOutSide.vecReflectors_.at(i).Draw(scrnRef, painter, Qt::yellow, nRadius );
    }

    for(int i = 0 ; i < matchInfoOutSide.vecLinePair_.size() ; i++)
    {
        CLine lnArrow = matchInfoOutSide.vecLinePair_.at(i).m_lnWorld;
        lnArrow.Draw(scrnRef, painter, Qt::green,2);
    }

    for(int i =0 ; i < matchInfoOutSide.vecLines_.size(); i++)
    {
        CLine lnArrow = matchInfoOutSide.vecLines_.at(i);
        lnArrow.Draw(scrnRef, painter, Qt::yellow,3);
    }
}
