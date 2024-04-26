#include <stdafx.h>
#include "LineFeatureSetEditable.h"
#include "LineFeatureEditable.h"

///////////////////////////////////////////////////////////////////////////////

//
//   根据直线特征类型分配空间。
//
CLineFeature *CLineFeatureSetEditable::NewLineFeature(int nType)
{
    return new CLineFeatureEditable;
}

//
//   取得选中特征直线的数量。
//
int CLineFeatureSetEditable::GetCountSelected()
{
    int count = 0;
    for (int i = 0; i < (int)size(); i++)
    {
        CLineFeatureEditable *p = dynamic_cast<CLineFeatureEditable *>(at(i));
        if (p == NULL)
            continue;

        if (p->IsSelected())
            count++;
    }

    return count;
}

//
//   判断指定的特征直线是否被选中。
//
bool CLineFeatureSetEditable::IsSelected(int index)
{
    if (index < 0 || index >= (int)size())
        return false;

    CLineFeatureEditable *p = dynamic_cast<CLineFeatureEditable *>(at(index));
    if (p == NULL)
        return false;

    return p->IsSelected();
}

//
//   选中/取消选中指定的直线特征。
//
void CLineFeatureSetEditable::Select(bool sel, int index)
{
    for (int i = 0; i < (int)size(); i++)
    {
        if (index >= 0 && i != index)
            continue;

        CLineFeatureEditable *p = dynamic_cast<CLineFeatureEditable *>(at(i));
        if (p == NULL)
            continue;

        p->Select(sel);
    }
}

//
//   删除所有选中的直线特征。
//
void CLineFeatureSetEditable::DeleteAllSelected()
{
    for (int i = (int)size() - 1; i >= 0; i--)
    {
        if (IsSelected(i))
            DeleteAt(i);
    }
}

//
//   判断一个给定点是否触碰到某一条直线特征(用于屏幕上直线触碰判断)。
//   返回值：
//      -1：给定点没有触碰到任何直线特征。
//     >=0: 触碰到的直线特征的编号
//
int CLineFeatureSetEditable::PointHit(const CPnt &pt, float fDistGate)
{
    // 逐个对所有直线特征进行判断
    for (int i = 0; i < (int)size(); i++)
    {
        if (at(i)->PointHit(pt, fDistGate))
            return i;
    }

    return -1;
}

#ifdef _MFC_VER

void CLineFeatureSetEditable::Dump()
{
    //	TRACE("Dumping Line Scan (%d lines):\n", size());

    for (int i = 0; i < (int)size(); i++)
    {
        DebugTrace(_T("Line #%d:\t%.2f\t%.2f\t\t%.2f\t%.2f\n"), i, at(i)->m_ptStart.x,
                   at(i)->m_ptStart.y, at(i)->m_ptEnd.x, at(i)->m_ptEnd.y);
    }

    DebugTrace(_T("\n"));
}

//
//   绘制直线特征集合。
//
void CLineFeatureSetEditable::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor,
                           COLORREF crSelected, int nPointSize, bool bShowActiveSide, bool bShowId,
                           bool bShowRefPoint, int nShowDisabled)
{
    for (int i = 0; i < (int)size(); i++)
    {
        at(i)->SetIntParam(0, (int)bShowActiveSide);
        at(i)->SetIntParam(1, (int)bShowId);
        at(i)->SetIntParam(2, (int)bShowRefPoint);

        at(i)->Plot(ScrnRef, pDC, crColor, crSelected, nPointSize, nShowDisabled);
    }
}

#elif defined QT_VERSION
//
//   绘制直线特征集合。
//
void CLineFeatureSetEditable::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, QColor clrSelected, int nSize)
{
    for (int i = 0; i < (int)size(); i++)
    {
        CLineFeatureEditable *line = dynamic_cast<CLineFeatureEditable *>(at(i));
        if (line == NULL)
            continue;

        line->Plot(ScrnRef, pPainter, clr, clrSelected, nSize);
    }
}

//
//   在屏幕上绘制此集合中所有被选中者。
//
void CLineFeatureSetEditable::PlotSelected(CScreenReference &ScrnRef, QPainter *pPainter, QColor cr, int nSize)
{
    for (int i = 0; i < (int)size(); i++)
    {
        CLineFeatureEditable *line = dynamic_cast<CLineFeatureEditable *>(at(i));
        if (line == NULL)
            continue;

        if (line->IsSelected())
            line->CLineFeature::Draw(ScrnRef, pPainter, cr, nSize);
    }
}

#endif
