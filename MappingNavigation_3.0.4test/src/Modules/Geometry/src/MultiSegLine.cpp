#include <stdafx.h>
#include "MultiSegLine.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CMultiSegLine::CMultiSegLine(const CLine &Line2)
    : CLine(Line2), CDataRangeSet(CDataRange(0, Line2.Length()))
{
}

CMultiSegLine::CMultiSegLine(const CMultiSegLine &another)
{
    *this = another;
}

//
//   重载"="操作符，将对象设置为仅含有一段直线段。
//
void CMultiSegLine::operator =(const CLine &line)
{
    GetLineObject() = line;

    clear();
    *this += CDataRange(0, line.Length());
}

//
//   取得指定的子段。
//
bool CMultiSegLine::GetSegment(int i, CLine &Line) const
{
    // 段号合法性检查
    if (i >= (int)size())
        return false;

    CPnt pt1 = TrajFun(at(i).from_);
    CPnt pt2 = TrajFun(at(i).to_);

    Line.Create(pt1, pt2);

    return true;
}

//
//   判断一个给定的点是否“触碰”到该多段直线。
//
bool CMultiSegLine::PointHit(const CPnt &pt, float fDistGate)
{
    CLine Line;

    for (int i = 0; i < (int)size(); i++)
    {
        GetSegment(i, Line);
        if (Line.PointHit(pt, fDistGate))
            return true;
    }

    return false;
}

#ifdef _MFC_VER

//
//   在屏幕上绘制此直线。
//
void CMultiSegLine::Draw(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nWidth,
                         int nPointSize, bool bBigVertex)
{
    CLine Line;
    for (int i = 0; i < (int)size(); i++)
    {
        GetSegment(i, Line);
        Line.Draw(ScrnRef, pDC, crColor, nWidth, nPointSize, bBigVertex);
    }
}

#elif defined QT_VERSION
//
//   在屏幕上绘制此直线。
//
void CMultiSegLine::Draw(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor, int nWidth,
                         int nPointSize, bool bBigVertex)
{
    CLine Line;
    for (int i = 0; i < (int)size(); i++)
    {
        GetSegment(i, Line);
        Line.Draw(ScrnRef, pPainter, crColor, nWidth, nPointSize, bBigVertex);
    }
}
#endif
