#include "stdafx.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <algorithm>
#include "Geometry.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPainter>
#include <QColor>
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

using namespace std;

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   CRectangle: The constructor form #1.
//
CRectangle::CRectangle(const CPnt &ptLeftTop, const CPnt &ptRightBottom)
{
    Create(ptLeftTop, ptRightBottom);
}

CRectangle::CRectangle(float fLeft, float fTop, float fRight, float fBottom)
{
    Create(fLeft, fTop, fRight, fBottom);
}

CRectangle::CRectangle(const CRectangle &other)
{
    Create(other.GetLeftTopPoint(), other.GetRightBottomPoint());
}

CRectangle::CRectangle()
{
    Clear();
}

//
//   生成矩形。
//   注：如果给定的“左上”或“右下”点数据大小是颠倒的，程序是能够正常处理的。
//
bool CRectangle::Create(const CPnt &ptLeftTop, const CPnt &ptRightBottom)
{
    Create(ptLeftTop.x, ptLeftTop.y, ptRightBottom.x, ptRightBottom.y);
    return true;
}

//
//   生成矩形。
//   注：如果给定的“左/右”或“上/下”数据大小是颠倒的，程序是能够正常处理的。
//
bool CRectangle::Create(float fLeft, float fTop, float fRight, float fBottom)
{
    if (fLeft > fRight)
        swap(fLeft, fRight);

    if (fBottom > fTop)
        swap(fBottom, fTop);

    m_ptLeftTop.Set(fLeft, fTop);
    m_ptRightBottom.Set(fRight, fBottom);
    m_ptLeftBottom.Set(fLeft, fBottom);
    m_ptRightTop.Set(fRight, fTop);

    m_bInit = true;

    return true;
}

//
//   清除区域。
//
void CRectangle::Clear()
{
    Create(0, 0, 0, 0);
    m_bInit = false;
}

//
//   取得中心点的位置。
//
CPnt CRectangle::GetCenterPoint() const
{
    CPnt ptCenter;
    ptCenter.x = (m_ptLeftTop.x + m_ptRightBottom.x) / 2;
    ptCenter.y = (m_ptLeftTop.y + m_ptRightBottom.y) / 2;

    return ptCenter;
}

//
//   判断一个点是否处于矩形以内。
//
bool CRectangle::Contain(float x, float y) const
{
    if (x >= m_ptLeftTop.x && x <= m_ptRightBottom.x && y >= m_ptRightBottom.y && y <= m_ptLeftTop.y)
        return true;
    else
        return false;
}

//
//   判断一个点是否在此矩形内。
//
bool CRectangle::Contain(const CPnt &pt) const
{
    if (pt.x >= m_ptLeftTop.x && pt.x <= m_ptRightBottom.x && pt.y >= m_ptRightBottom.y && pt.y <= m_ptLeftTop.y)
        return true;
    else
        return false;
}

//
//   判断一线段是否处于矩形以内。
//
bool CRectangle::Contain(const CLine &ln) const
{
    if (Contain(ln.m_ptStart) && Contain(ln.m_ptEnd))
        return true;
    else
        return false;
}

//
//   判断另一个矩形是否处于该矩形以内。
//
bool CRectangle::Contain(const CRectangle &r) const
{
    if (Contain(r.m_ptLeftTop) && Contain(r.m_ptRightBottom))
        return true;
    else
        return false;
}

//
//   取得矩形区域的宽度。
//
float CRectangle::Width() const
{
    return m_ptRightBottom.x - m_ptLeftTop.x;
}

//
//   取得矩形区域的高度。
//
float CRectangle::Height() const
{
    return m_ptLeftTop.y - m_ptRightBottom.y;
}

//
//   取得矩形区域的面积。
//
float CRectangle::Area() const
{
    return Width() * Height();
}

//
//   调整矩形区域大小以便容纳给定的点。
//
void CRectangle::operator+=(const CPnt &pt)
{
    if (!m_bInit)
    {
        m_ptLeftTop = m_ptRightBottom = pt;
        m_bInit = true;
        return;
    }

    if (pt.x < m_ptLeftTop.x)
        m_ptLeftTop.x = pt.x;

    if (pt.x > m_ptRightBottom.x)
        m_ptRightBottom.x = pt.x;

    if (pt.y < m_ptRightBottom.y)
        m_ptRightBottom.y = pt.y;

    if (pt.y > m_ptLeftTop.y)
        m_ptLeftTop.y = pt.y;
}

//
//   调整矩形区域大小以便容纳给定的直线。
//
void CRectangle::operator+=(const CLine &line)
{
    *this += line.m_ptStart;
    *this += line.m_ptEnd;
}

//
//   调整矩形区域大小以便容纳给定的矩形区域。
//
void CRectangle::operator+=(const CRectangle &rect)
{
    *this += rect.GetLeftTopPoint();
    *this += rect.GetRightBottomPoint();
}

bool CRectangle::operator==(const CRectangle &rect)
{
     return (this->m_ptLeftTop == rect.m_ptLeftTop &&
             this->m_ptRightBottom == rect.m_ptRightBottom &&
             this->m_ptLeftBottom == rect.m_ptLeftBottom &&
             this->m_ptRightTop == rect.m_ptRightTop);
}

void CRectangle::operator=(const CRectangle &rect)
{
    this->m_ptLeftTop = rect.m_ptLeftTop ;
    this->m_ptRightBottom =rect.m_ptRightBottom;
    this->m_ptLeftBottom = rect.m_ptLeftBottom ;
    this->m_ptRightTop =rect.m_ptRightTop;
}

//
//   判断一个给定的点是否“触碰”到该矩形。
//   返回值：
//     0 - 未触碰到矩形
//     1 - 触碰到矩形中心点
//     2 - 触碰到矩形左上角顶点
//     3 - 触碰到矩形右上角顶点
//     4 - 触碰到矩形左下角顶点
//     5 - 触碰到矩形右下角顶点
//     6 - 触碰到矩形左边线中点
//     7 - 触碰到矩形右边线中点
//     8 - 触碰到矩形上边线中点
//     9 - 触碰到矩形下边线中点
//    10 - 触碰到矩形边框上的其它点
//    11 - 触碰到矩形内的其它点
//
int CRectangle::PointHit(const CPnt &pt, float fDistGate) const
{
    CLine ln[4] = {CLine(m_ptLeftTop, m_ptRightTop), CLine(m_ptLeftTop, m_ptLeftBottom),
                   CLine(m_ptLeftBottom, m_ptRightBottom), CLine(m_ptRightTop, m_ptRightBottom)};

    // 判断是否触碰到矩形上的特殊点
    if (pt.PointHit(GetCenterPoint(), fDistGate))
        return CENTER_POINT;
    else if (pt.PointHit(m_ptLeftTop, fDistGate))
        return LEFT_TOP_POINT;
    else if (pt.PointHit(m_ptRightTop, fDistGate))
        return RIGHT_TOP_POINT;
    else if (pt.PointHit(m_ptLeftBottom, fDistGate))
        return LEFT_BOTTOM_POINT;
    else if (pt.PointHit(m_ptRightBottom, fDistGate))
        return RIGHT_BOTTOM_POINT;
    else if (pt.PointHit(ln[1].GetMidpoint(), fDistGate))
        return LEFT_MIDDLE_POINT;
    else if (pt.PointHit(ln[3].GetMidpoint(), fDistGate))
        return RIGHT_MIDDLE_POINT;
    else if (pt.PointHit(ln[0].GetMidpoint(), fDistGate))
        return TOP_MIDDLE_POINT;
    else if (pt.PointHit(ln[2].GetMidpoint(), fDistGate))
        return BOTTOM_MIDDLE_POINT;

    // 判断是否触碰到矩形的某条边
    for (int i = 0; i < 4; i++)
        if (ln[i].PointHit(pt, fDistGate))
            return BORDER_POINT;

    // 判断是否落在矩形内部
    if (Contain(pt))
        return OTHER_INSIDE_POINT;

    // 不是以上情况，是矩形外部的点
    return OUTSIDE_POINT;
}

//
//   从文件装入矩形数据。
//
bool CRectangle::LoadText(FILE *fp)
{
    float fLeft, fTop, fRight, fBottom;
    if (fscanf(fp, "%f\t%f\t%f\t%f\n", &fLeft, &fTop, &fRight, &fBottom) != 4)
        return false;

    Create(fLeft, fTop, fRight, fBottom);
    return true;
}

//
//   将矩形数据写入文件。
//
bool CRectangle::SaveText(FILE *fp)
{
    fprintf(fp, "%f\t%f\t%f\t%f\n", m_ptLeftTop.x, m_ptLeftTop.y, m_ptRightBottom.x, m_ptRightBottom.y);
    return true;
}

//
//   从二进制文件装入矩形数据。
//
bool CRectangle::LoadBinary(FILE *fp)
{
    float f[4];
    if (fread(f, sizeof(float), 4, fp) != 4)
        return false;

    Create(f[0], f[1], f[2], f[3]);
    return true;
}

//
//   将矩形数据写入二进制文件。
//
bool CRectangle::SaveBinary(FILE *fp)
{
    float f[4] = {m_ptLeftTop.x, m_ptLeftTop.y, m_ptRightBottom.x, m_ptRightBottom.y};
    if (fwrite(f, sizeof(float), 4, fp) != 4)
        return false;

    return true;
}


#ifdef _MFC_VER

//
//   在屏幕上绘制此矩形。
//
void CRectangle::Draw(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nWidth, bool bFill, COLORREF crFill)
{
    CPnt ptRightTop(m_ptRightBottom.x, m_ptLeftTop.y);
    CPnt ptLeftBottom(m_ptLeftTop.x, m_ptRightBottom.y);

    CLine ln1(m_ptLeftTop, ptRightTop);
    CLine ln2(m_ptLeftTop, ptLeftBottom);
    CLine ln3(ptLeftBottom, m_ptRightBottom);
    CLine ln4(ptRightTop, m_ptRightBottom);

    if (bFill)
    {
        CPoint pntLeftTop = ScrnRef.GetWindowPoint(m_ptLeftTop);
        CPoint pntRightBottom = ScrnRef.GetWindowPoint(m_ptRightBottom);

        CRect r(pntLeftTop, pntRightBottom);

        int saveROP2 = pDC->SetROP2(R2_MERGEPENNOT);
        pDC->FillSolidRect(&r, crFill);
        pDC->SetROP2(saveROP2);
    }

    ln1.Draw(ScrnRef, pDC, crColor, nWidth);
    ln2.Draw(ScrnRef, pDC, crColor, nWidth);
    ln3.Draw(ScrnRef, pDC, crColor, nWidth);
    ln4.Draw(ScrnRef, pDC, crColor, nWidth);
}

#elif defined QT_VERSION

//
//   在屏幕上绘制此矩形。
//
void CRectangle::Draw(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor, int nWidth, int penStyle,
                      bool bFill, QColor crFill)
{
    QPoint pntLeftTop = ScrnRef.GetWindowPoint(m_ptLeftTop);
    QPoint pntRightBottom = ScrnRef.GetWindowPoint(m_ptRightBottom);

    QRect r(pntLeftTop, pntRightBottom);

    QPen pen(crColor);
    pen.setWidth(nWidth);
    pen.setStyle((Qt::PenStyle)penStyle);
    pPainter->setPen(pen);

    QBrush brush(crFill);
    if (!bFill)
        brush.setStyle(Qt::NoBrush);

    pPainter->setBrush(brush);
    pPainter->drawRect(r);
}

#endif
