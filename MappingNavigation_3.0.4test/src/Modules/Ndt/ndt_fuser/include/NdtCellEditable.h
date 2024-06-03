#pragma once

#include "GraphicObj.h"
#include "ndt_cell_oru.h"

namespace ndt_oru
{
class DllExport NDTCellEditable : public NDTCell, public CGraphicObj
{
  public:
    //   判断一个给定的点是否“触碰”到该单元。
    virtual bool PointHit(const CPnt &pt, float fDistGate);

#ifdef _MFC_VER
    void PlotCell(CScreenReference &ScrnRef, CDC *pDc, unsigned long clrLine, unsigned long clrFill);

    void PlotEllipse(CScreenReference &ScrnRef, CDC *pDc, unsigned long clrLine, unsigned long clrFill,
                     bool bShowMatched = false, unsigned long clrMatched = 0);

    // 显示单元的匹配情况(以直线显示匹配单元对)
    void PlotMatchStatus(CScreenReference &ScrnRef, CDC *pDc, unsigned long clr);

#elif defined QT_VERSION
    void PlotCell(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine, QColor clrFill);

    void PlotEllipse(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine, QColor clrFill,
                     bool bShowMatched = false, QColor clrMatched = Qt::black, int lineWidth = 1);

    // 显示单元的匹配情况(以直线显示匹配单元对)
    void PlotMatchStatus(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr);
#endif
};
}
