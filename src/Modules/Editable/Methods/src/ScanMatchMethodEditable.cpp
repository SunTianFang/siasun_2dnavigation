#include <stdafx.h>
#include "ScanMatchMethodEditable.h"
#include "MapFuser.h"

#define NORNAL_CELL_COLOR QColor(0, 100, 147)

///////////////////////////////////////////////////////////////////////////////
//   定义“CScanMatchMethodEditable”类，它是:CScanMatchMethod类的可编辑版本。

CScanMatchMethodEditable::CScanMatchMethodEditable()
{
    visible_ = true;

    // 缺省关闭显示
    showInitPose = false;
    showResultPose = false;

}

//
//   绘制对应于该方法的地图。
//
void CScanMatchMethodEditable::PlotNormalMap(QPainter *painter, CScreenReference &scrnRef)
{
    if (!visible_)
        return;

    if(map_==nullptr)
        return;

    double max_y = map_->limits_.max_y_;
    double max_x = map_->limits_.max_x_;
    double resolution = map_->limits_.resolution_;


#if 0
    CPnt  m_ptLeftTop ;
    m_ptLeftTop.x = map_->limits_.max_x() - (resolution * map_->limits_.cell_limits_.num_y_cells);
    m_ptLeftTop.y = map_->limits_.max_y();

    CPnt m_ptRightBottom;
    m_ptRightBottom.x = map_->limits_.max_x() ;
    m_ptRightBottom.y = map_->limits_.max_y() - (resolution * map_->limits_.cell_limits_.num_x_cells);

    QPoint pntLeftTop = scrnRef.GetWindowPoint(m_ptLeftTop);
    QPoint pntRightBottom = scrnRef.GetWindowPoint(m_ptRightBottom);

    QRect r(pntLeftTop, pntRightBottom);

    painter->drawRect(r);

#endif

    for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(map_->limits_.cell_limits_))
    {

        double prob = map_->GetProbability(xy_index);

        if(prob>0.55)
        {
           //int  data = 255*(1-prob);
            int data = (1-prob-0.1)*1.25*255;
            if(data>255)
                data = 255;
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

            int penStyle = 0;
            QRect r(pntLeftTop, pntRightBottom);
            QPen pen(QColor(0,0,0));

            pen.setWidth(0);
            pen.setStyle((Qt::PenStyle)penStyle);
            painter->setPen(pen);

//               QBrush brush(QColor(255,255,255));
//               brush.setStyle(Qt::NoBrush);
//               QPen pen(crLineColor);
//               pen.setStyle((Qt::PenStyle)nPenStyle);
//               pen.setWidth(nLineWidth);
//               pPainter->setPen(pen);

            QBrush brush(QColor(data,data,data));
            // if (!bFill)
            //    brush.setStyle(Qt::NoBrush);

            painter->setBrush(brush);
            painter->drawRect(r);
        }
    }
    //by DQ
     if(map_->GridIndex_size() > 0)
     {
         for(int i = 0; i < map_->GridIndex_size(); i++)
         //for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(map_->limits_.cell_limits_))
         {

             double prob = map_->GetProbability(map_->GridIndex_set[i]);

             if(prob>0.55)
             {
                //int  data = 255*(1-prob);
                 //int data = (1-prob-0.1)*1.25*255;
                 //if(data>255)
                 //    data = 255;
                 double  yi = max_y - map_->GridIndex_set[i](0)*resolution;
                 double  xi = max_x - map_->GridIndex_set[i](1)*resolution;

                 CPnt  m_ptLeftTop ;
                 m_ptLeftTop.x = xi - resolution;
                 m_ptLeftTop.y = yi;

                 CPnt m_ptRightBottom;
                 m_ptRightBottom.x = xi ;
                 m_ptRightBottom.y = yi - resolution;

                 QPoint pntLeftTop = scrnRef.GetWindowPoint(m_ptLeftTop);
                 QPoint pntRightBottom = scrnRef.GetWindowPoint(m_ptRightBottom);

                 int penStyle = 0;
                 QRect r(pntLeftTop, pntRightBottom);
                 QPen pen(QColor(0,0,0));

                 pen.setWidth(0);
                 pen.setStyle((Qt::PenStyle)penStyle);
                 painter->setPen(pen);


                 //QBrush brush(QColor(data,data,data));
                 QBrush brush(Qt::green);
                 // if (!bFill)
                 //    brush.setStyle(Qt::NoBrush);

                 painter->setBrush(brush);
                 painter->drawRect(r);
             }
         }
     }
    return;
}


//
//   绘制对应于该方法的定位情况。
//
void CScanMatchMethodEditable::PlotLocalization(QPainter *painter, CScreenReference &scrnRef)
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
void CScanMatchMethodEditable::ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef, int curStep,
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
