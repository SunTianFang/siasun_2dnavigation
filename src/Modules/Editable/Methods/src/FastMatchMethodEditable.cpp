#include <stdafx.h>
#include "FastMatchMethodEditable.h"
#include "MapFuser.h"

#define NORNAL_CELL_COLOR QColor(0, 100, 147)

///////////////////////////////////////////////////////////////////////////////
//   定义“CNdtMethodEditable”类，它是CNdtMethod类的可编辑版本。

CFastMatchMethodEditable::CFastMatchMethodEditable()
{

    visible_ = true;

    // 缺省关闭显示
    showInitPose = false;
    showResultPose = false;

}



//
//   绘制对应于该方法的地图。
//
void CFastMatchMethodEditable::PlotNormalMap(QPainter *painter, CScreenReference &scrnRef)
{
    if (!visible_)
        return;

#if 0
    double max_y = map_->limits_.max_y_;
    double max_x = map_->limits_.max_x_;
    double resolution = map_->limits_.resolution_;


    for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(map_->limits_.cell_limits_)) {

           double prob = map_->GetProbability(xy_index);

           if(prob >= 0.55)
           {

            /*   Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
                 // Index values are row major and the top left has Eigen::Array2i::Zero()
                 // and contains (centered_max_x, centered_max_y). We need to flip and
                 // rotate.
                 return Eigen::Array2i(
                     common::RoundToInt((max_y_ - point.y()) / resolution_ - 0.5),
                     common::RoundToInt((max_x_ - point.x()) / resolution_ - 0.5));
               }*/

               double  yi = max_y - xy_index(0)*resolution;
               double  xi = max_x - xy_index(1)*resolution;


               CPnt  m_ptLeftTop ;
               m_ptLeftTop.x = xi - resolution;
               m_ptLeftTop.y = yi;



               CPnt m_ptRightBottom;
               m_ptRightBottom.x = xi ;
               m_ptRightBottom.y = yi - resolution;

               QPoint pntLeftTop = scrnRef.GetWindowPoint(m_ptLeftTop);
               QPoint pntRightBottom = scrnRef.GetWindowPoint(m_ptRightBottom);

               QRect r(pntLeftTop, pntRightBottom);

               int penStyle = 1;

               QPen pen(QColor(255,255,0));
               pen.setWidth(1);
               pen.setStyle((Qt::PenStyle)penStyle);
               painter->setPen(pen);

               QBrush brush(QColor(200,200,200));
              // if (!bFill)
               //    brush.setStyle(Qt::NoBrush);

               painter->setBrush(brush);
               painter->drawRect(r);


           }

    }
#endif
    return;
}


//
//   绘制对应于该方法的定位情况。
//
void CFastMatchMethodEditable::PlotLocalization(QPainter *painter, CScreenReference &scrnRef)
{
#if 0
    QString str="";

     ////bug  'std::out_of_range'
  //  matchInfo_.initposture.Draw(scrnRef, painter, QColor(255,0,0), QColor(0,255,0), 40, 150, 2, str, Qt::white);

       std::cout<<"fast plot start"<<std::endl;
    matchInfo_.pst_.Draw(scrnRef, painter, QColor(0,0,255), QColor(0,255,0), 40, 150, 2, str, Qt::white);




    const  CFrame curposture(matchInfo_.pst_);
    CScan scan(matchInfo_.cloudIn.size());
    matchInfo_.cloudIn.ToScan(&scan);


    scan.InvTransform(curposture);
    scan.m_pstScanner = curposture;
    scan.Plot(scrnRef, painter, QColor(0,0,255), QColor(0,255,0),true,QColor(255,0,0), 1);

       std::cout<<"fast plot end"<<std::endl;

   /*   const  CFrame initrposture(matchInfo_.initposture);

    CScan initscan(matchInfo_.cloudIn.size());
    matchInfo_.cloudIn.ToScan(&initscan);


    initscan.InvTransform(initrposture);
    initscan.m_pstScanner = initrposture;
    initscan.Plot(scrnRef, painter, QColor(127,127,127), QColor(0,255,0),true,QColor(255,0,0), 1);

*/

#endif

}

//
//   显示对应于该方法的匹配数据。
//
void CFastMatchMethodEditable::ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
                                         int totalSteps)
{
/*
    painter->setPen(Qt::black);
    QString str;

    str.sprintf("Ndt Match Info: ");
    painter->drawText(10, 95, str);

    str.sprintf("lx: %d, ly: %d, lpx: %d, lpy: %d, mx: %d, my: %d", (int)localization_->mp_Informer->lx,
                localization_->mp_Informer->ly, localization_->mp_Informer->lpx, localization_->mp_Informer->lpy,
                localization_->mp_Informer->mx, localization_->mp_Informer->my);
    painter->drawText(10, 120, str);

    str.sprintf("ratioX: %f, ratioY: %f", (float)localization_->mp_Informer->lpx / localization_->mp_Informer->lx,
                (float)localization_->mp_Informer->lpy / localization_->mp_Informer->ly);
    painter->drawText(10, 145, str);*/


}
