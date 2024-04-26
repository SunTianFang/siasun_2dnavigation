#include <stdafx.h>
#include "CircleElement.h"

///////////////////////////////////////////////////////////////////////////////
//   “CCircleElement”类的实现。

//
//   复制对象并返回指针。
//
CObjectElement *CCircleElement::Duplicate() const
{
    CCircleElement *circleEle = new CCircleElement(m_circle);
    return circleEle;
}

//
//   判断指定的点是否触碰到该圆形元素。
//
bool CCircleElement::PointHit(const CPnt &pt, float fDistGate)
{
    return (fabs(m_circle.m_ptCenter.DistanceTo(pt) - m_circle.Radius()) < fDistGate);
}

//
//   判断物体元素是否与给定的直线相交。
//
bool CCircleElement::LineHit(const CLine &ray, CPnt &pt, float &dist) const
{
    if (ray.Length() < 0.01f)
        return false;
    else
        return m_circle.IntersectLineAt(ray, pt, dist);
}

//
//   进行坐标正变换。
//
void CCircleElement::Transform(const CFrame &frame)
{
    m_circle.Transform(frame);
}

//
//   进行坐标逆变换。
//
void CCircleElement::InvTransform(const CFrame &frame)
{
    m_circle.InvTransform(frame);
}

//
//   从文本文件中装入数据。
//
bool CCircleElement::LoadText(FILE *fp)
{
    return m_circle.LoadText(fp);
}

//
//   将数据保存到文本文件中。
//
bool CCircleElement::SaveText(FILE *fp)
{
    return m_circle.SaveText(fp);
}

//
//   从二进制文件中装入数据。
//
bool CCircleElement::LoadBinary(FILE *fp)
{
    return m_circle.LoadBinary(fp);
}

//
//   将数据保存到二进制文件中。
//
bool CCircleElement::SaveBinary(FILE *fp)
{
    return m_circle.SaveBinary(fp);
}

#ifdef _MFC_VER
void CCircleElement::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth)
{
    m_circle.Draw(ScrnRef, pDC, crColor, nLineWidth);
}

#elif defined QT_VERSION
void CCircleElement::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor,
                          int nLineWidth)
{
    m_circle.Draw(ScrnRef, pPainter, crColor, nLineWidth);
}
#endif
