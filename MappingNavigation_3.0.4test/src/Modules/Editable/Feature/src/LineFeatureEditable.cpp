#include <stdafx.h>
#include "LineFeatureEditable.h"

///////////////////////////////////////////////////////////////////////////////

//
//   生成一个复本
//
CLineFeature *CLineFeatureEditable::Duplicate() const
{
    return new CLineFeatureEditable(*this);
}

#ifdef _MFC_VER

//
//   绘制直线特征。
//
//   注意：程序中根据特征的扩展参数值确定是否需要显示：1. 直线特征的朝向; 2.特征的ID号
//
void CLineFeatureEditable::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, COLORREF crSelected, int nSize,
                        int nShowDisabled, bool bShowSuggested)
{
    bool bShowActiveSide = m_nParam[0];
    bool bShowId = m_nParam[1];
    bool bShowRefPoint = m_nParam[2];

    // 画出直线特征
    if (!m_bSelected)
    {
        // 正常显示禁止项
        if (IsEnabled() || nShowDisabled == DISABLED_FEATURE_NORMAL)
            Draw(ScrnRef, pDC, crColor, nSize, nSize, false);

        // 淡色显示禁止项
        else if (nShowDisabled == DISABLED_FEATURE_UNDERTONE)
        {
            BYTE r = GetRValue(crColor) / 3;
            BYTE g = GetGValue(crColor) / 3;
            BYTE b = GetBValue(crColor) / 3;
            Draw(ScrnRef, pDC, RGB(r, g, b), nSize, nSize, false);
        }
    }
    else
        Draw(ScrnRef, pDC, crSelected, 3 * nSize, nSize, false);

    // 显示参考投影点
    if (bShowRefPoint && m_bCreateRef)
    {
        m_ptRef.Draw(ScrnRef, pDC, RGB(0, 0, 255), 3);
        m_ptProjectFoot.Draw(ScrnRef, pDC, RGB(255, 255, 0), 3);
    }

    // 显示直线特征朝向
    if (bShowActiveSide)
    {
        CAngle ang = SlantAngle();
        if (m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
            ang += PI / 2;
        else if (m_nWhichSideToUse == FEATURE_DIR_BACK_SIDE_ONLY)
            ang -= PI / 2;

        float fArrowLen = 8 / ScrnRef.m_fRatio;
        CLine lnArrow(GetMidpoint(), ang, fArrowLen);
        lnArrow.Draw(ScrnRef, pDC, RGB(64, 64, 64));
    }

    // 显示直线特征编号
    if (bShowId)
    {
        CString str;
        str.Format(_T("%d"), m_nId);

        pDC->SetTextColor(RGB(0, 255, 0));

        CPoint pnt = ScrnRef.GetWindowPoint(GetMidpoint());
        pDC->TextOut(pnt.x - 20, pnt.y - 20, str);
        pDC->SetTextColor(RGB(0, 0, 0));
    }
}

#elif defined QT_VERSION

//
//   绘制直线特征。
//
//   注意：程序中根据特征的扩展参数值确定是否需要显示：1. 直线特征的朝向; 2.特征的ID号
//
void CLineFeatureEditable::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, QColor clrSelected, int nSize)
{
    bool bShowActiveSide = false;
    bool bShowId = false;

    // 画出直线特征
    if (IsSelected())
        CLineFeature::Draw(ScrnRef, pPainter, clrSelected, 3 * nSize, nSize, false);
    else
        CLineFeature::Draw(ScrnRef, pPainter, clr, nSize, nSize, false);

    // 显示直线特征朝向
    if (bShowActiveSide)
    {
        CAngle ang = SlantAngle();
        if (m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
            ang += PI / 2;
        else if (m_nWhichSideToUse == FEATURE_DIR_BACK_SIDE_ONLY)
            ang -= PI / 2;

        float fArrowLen = 8 / ScrnRef.m_fRatio;
        CLine lnArrow(GetMidpoint(), ang, fArrowLen);
        lnArrow.Draw(ScrnRef, pPainter, QColor(64, 64, 64));
    }

    // 显示直线特征编号
    if (bShowId)
    {
        QString str;
        str.sprintf("%d", m_nId);

        QPen pen(Qt::green);
        pPainter->setPen(pen);
        QPoint pnt = ScrnRef.GetWindowPoint(GetMidpoint());
        pPainter->drawText(pnt.x() - 20, pnt.y() - 20, str);
        pPainter->setPen(Qt::black);
    }
}
#endif
