#include <stdafx.h>
#include "PointFeatureEditable.h"

///////////////////////////////////////////////////////////////////////////////

#define POINT_FEATURE_RADIUS_MM        25     // 假定点特征半径为25mm

//
//   生成一个复本。
//
CPointFeature *CPointFeatureEditable::Duplicate() const
{
    CPointFeatureEditable *p = new CPointFeatureEditable(*this);
//    *p = *this;
    return p;
}

//
//   判断指定的点是否触碰到该点特征。
//
bool CPointFeatureEditable::PointHit(const CPnt &pt, float thresh)
{
    return IsVisible() && CPointFeature::PointHit(pt, thresh);
}

#ifdef _MFC_VER
//
//   在屏幕上绘制此点特征。
//
void CPointFeatureEditable::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected,
    int nPointSize, int nLineWidth)
{
    int nRadius = POINT_FEATURE_RADIUS_MM / 1000.0f * ScrnRef.m_fRatio;

    if (nRadius < nPointSize)
        nRadius = nPointSize;

    Draw(ScrnRef, pDC, crColor, nRadius, nLineWidth);
}

//
//   在屏幕上绘制此点特征的ID号。
//
void CPointFeatureEditable::PlotId(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor)
{
    CPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());

    CString str;
    str.Format(_T("%d"), id);

    COLORREF crOldColor = pDC->SetTextColor(crColor);
    pDC->TextOut(pnt.x - 20, pnt.y - 20, str);
    pDC->SetTextColor(crOldColor);
}

#elif defined QT_VERSION

//
//   在屏幕上绘制此点特征。
//
void CPointFeatureEditable::Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crSelected,
    float mmPointSize, int nMinRadius, int nLineWidth)
{
    int nRadius = mmPointSize / 1000.0f * ScrnRef.m_fRatio;

    if (nRadius < nMinRadius)
        nRadius = nMinRadius;

    CPointFeature::Draw(ScrnRef, pPainter, crColor, nRadius, nLineWidth);
}

//
//   在屏幕上绘制此点特征的ID号。
//
void CPointFeatureEditable::PlotId(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor)
{
    QPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());

    QString str = QString::number(id);
    pPainter->setPen(crColor);
    pPainter->drawText(pnt.x() - 20, pnt.y() - 20, str);
}
#endif
