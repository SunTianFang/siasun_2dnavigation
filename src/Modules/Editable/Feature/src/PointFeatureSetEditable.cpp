#include <stdafx.h>
#include "PointFeatureSetEditable.h"
#include "PointFeatureEditable.h"

///////////////////////////////////////////////////////////////////////////////

CPointFeatureSetEditable::CPointFeatureSetEditable()
{
}

//
//   取得选中特征点的数量。
//
int CPointFeatureSetEditable::GetCountSelected()
{
    int count = 0;
    for (int i = 0; i < (int)size(); i++)
    {
        CPointFeatureEditable *p = dynamic_cast<CPointFeatureEditable *>(at(i));
        if (p == NULL)
            continue;

        if (p->IsSelected())
            count++;
    }

    return count;
}

//
//   判断指定的特征点是否被选中。
//
bool CPointFeatureSetEditable::IsSelected(int index)
{
    if (index < 0 || index >= (int)size())
        return false;

    CPointFeatureEditable *p = dynamic_cast<CPointFeatureEditable *>(at(index));
    if (p == NULL)
        return false;

    return p->IsSelected();
}

//
//   选中/取消选中指定的点特征。
//
void CPointFeatureSetEditable::Select(bool sel, int nIdx)
{
    for (int i = 0; i < (int)size(); i++)
    {
        if (nIdx >= 0 && i != nIdx)
            continue;

        CPointFeatureEditable *p = dynamic_cast<CPointFeatureEditable *>(at(i));
        if (p == NULL)
            continue;

        p->Select(sel);
    }
}

//
//   删除所有选中的点特征。
//
void CPointFeatureSetEditable::DeleteAllSelected()
{
    for (int i = (int)size() - 1; i >= 0; i--)
    {
        if (IsSelected(i))
            DeleteAt(i);
    }
}

//
//   根据所提供的特征类型分配空间。
//
CPointFeature *CPointFeatureSetEditable::NewPointFeature(int nSubType)
{
    // 目前，只支持CPointFeatureEdit类型
    return new CPointFeatureEditable;
}

//
//   判断指定的点是否触碰到某个点特征。
//
int CPointFeatureSetEditable::PointHit(const CPnt &pt, float thresh)
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i)->PointHit(pt, thresh))
            return i;    // 触碰到这个点特征

    return -1;
}

#ifdef _MFC_VER

void CPointFeatureSetEditable::Dump()
{
    for (int i = 0; i < GetCount(); i++)
    {
        CPointFeature *pFeature = (CPointFeature *)at(i);
        CPnt &pt = pFeature->GetPntObject();
        pt.Dump();
    }
}

//
//   在屏幕上绘制此点特征集合。
//
void CPointFeatureSetEditable::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF cr,
                                    COLORREF crSelected, int nLineWidth, bool bShowId)
{
    for (int i = 0; i < GetCount(); i++)
    {
        CPointFeature *pFeature = (CPointFeature *)at(i);
        pFeature->Plot(ScrnRef, pDC, cr, crSelected, 5, nLineWidth);

        if (bShowId)
            pFeature->PlotId(ScrnRef, pDC, RGB(0, 0, 255));
    }
}

#elif defined QT_VERSION

void CPointFeatureSetEditable::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor cr,
                                    QColor crSelected, int nSize, bool bShowId, float mmSize)
{
    for (int i = 0; i < GetCount(); i++)
    {
        CPointFeatureEditable *pFeature = dynamic_cast<CPointFeatureEditable *>(at(i));
        if (pFeature == NULL)
            continue;

        pFeature->Plot(ScrnRef, pPainter, cr, crSelected, mmSize, nSize);

        if (bShowId)
            pFeature->PlotId(ScrnRef, pPainter, Qt::blue);
    }
}

//
//   在屏幕上绘制此集合中所有被选中者。
//
void CPointFeatureSetEditable::PlotSelected(CScreenReference &ScrnRef, QPainter *pPainter,
                                            QColor cr, int nSize, float mmSize)
{
    for (int i = 0; i < GetCount(); i++)
    {
        CPointFeatureEditable *pFeature = dynamic_cast<CPointFeatureEditable *>(at(i));
        if (pFeature == NULL)
            continue;

        if (pFeature->IsSelected())
            pFeature->Plot(ScrnRef, pPainter, cr, cr, mmSize, nSize);
    }
}

#endif
