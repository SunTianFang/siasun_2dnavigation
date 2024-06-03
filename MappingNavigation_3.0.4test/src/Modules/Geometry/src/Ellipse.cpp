//                         - ELLIPSE.CPP -
//
//   Implementation of class "CEllipse", which depicts ellipse.
//
//   Author: Zhang Lei
//   Date:   2014. 9. 11
//

#include "stdafx.h"
#include <math.h>
#include "Ellipse.h"
#include "ScrnRef.h"
#include "Frame.h"

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#include <QTransform>
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CArc".

//
//   The constructor.
//
CEllipse::CEllipse(const CPnt &ptCenter, float fHalfMajorAxis, float fHalfMiorAxis, const CAngle &angSlant)
{
	 Create(ptCenter, fHalfMajorAxis, fHalfMiorAxis, angSlant);
}

//
//   生成此椭圆圆。
//
void CEllipse::Create(const CPnt &ptCenter, float fHalfMajorAxis, float fHalfMinorAxis, const CAngle &angSlant)
{
	 m_ptCenter = ptCenter;
	 m_fHalfMajorAxis = fHalfMajorAxis;
	 m_fHalfMinorAxis = fHalfMinorAxis;
	 m_angSlant = angSlant;
}

//
//   判断一个点是否处于椭圆以内。
//
bool CEllipse::Contain(const CPnt &pt) const
{
	 float d = m_ptCenter.DistanceTo(pt);

	 // 如果该点距离椭圆中心点距离极近，或小于短半轴，则认为点落于椭圆内
	 if (d < m_fHalfMinorAxis || d < 1E-5)
		  return true;
	 else
	 {
		  // 以椭圆中心处长轴为X轴正轴，构建局部坐标系
		  CFrame frame(m_ptCenter, m_angSlant);

		  // 计算pt在局部坐标系中的位置
		  CPnt pt1 = pt;
		  pt1.Transform(frame);

		  // 取得pt点在局部坐标系中的纵坐标h
		  float h = pt1.y;

		  // phi角为对应于椭圆内切圆的h高度处的圆心角
		  float sin_phi = h / m_fHalfMinorAxis;
		  float cos_phi = sqrt(1 - sin_phi * sin_phi);

		  // 计算对应地此倾角的椭圆点到中心的距离
		  float q1 = m_fHalfMajorAxis * cos_phi;
		  float q2 = m_fHalfMinorAxis * sin_phi;
		  float r = sqrt(q1 * q1 + q2 * q2);
		  return (d <= r);
	 }
}

#ifdef _MFC_VER

//
//   在屏幕上绘制此圆。
//
void CEllipse::Draw(CScreenReference &ScrnRef, CDC *pDC, COLORREF crLineColor, int nLineWidth, COLORREF crFillColor, bool bFill, int nPenStyle)
{
	 CPen pen(nPenStyle, nLineWidth, crLineColor);
	 CPen *pOldPen = pDC->SelectObject(&pen);

	 CBrush Brush(crFillColor);
	 CBrush *pOldBrush;

	 if (bFill)
		  pOldBrush = pDC->SelectObject(&Brush);
	 else
		  pOldBrush = (CBrush *)pDC->SelectStockObject(NULL_BRUSH);

	 CPnt ptLeftTop(m_ptCenter.x - m_fHalfMajorAxis, m_ptCenter.y + m_fHalfMinorAxis);
	 CPnt ptRightBottom(m_ptCenter.x + m_fHalfMajorAxis, m_ptCenter.y - m_fHalfMinorAxis);

	 CPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	 CPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);

	 // 得到未经旋转的矩形外廓
	 CRect r(pnt1, pnt2);
	 float x0 = r.CenterPoint().x;
	 float y0 = r.CenterPoint().y;

	 float c = cos(m_angSlant);
	 float s = sin(m_angSlant);

	 // 设置旋转变换画法
	 XFORM xf;
#if 0
	xf.eDx = x0 - c * x0 + s * y0;
	xf.eDy = y0 - c * y0 - s * x0;
	xf.eM11 =  c;
	xf.eM12 =  s;
	xf.eM21 = -s;
#else
	 xf.eDx = x0 - c * x0 - s * y0;
	 xf.eDy = y0 - c * y0 + s * x0;
	 xf.eM11 = c;
	 xf.eM12 = -s;
	 xf.eM21 = s;
#endif
	 xf.eM22 = c;

	 HDC hdc = pDC->GetSafeHdc();
	 int nGraphicsMode = ::SetGraphicsMode(hdc, GM_ADVANCED);
	 ::SetWorldTransform(hdc, &xf);

	 // 画出椭圆
	 pDC->Ellipse(r);

	 // 画完后恢复正常画法
	 xf.eM11 = (float)1.0;
	 xf.eM12 = (float)0;
	 xf.eM21 = (float)0;
	 xf.eM22 = (float)1.0;
	 xf.eDx = (float)0;
	 xf.eDy = (float)0;

	 SetWorldTransform(hdc, &xf);
	 SetGraphicsMode(hdc, nGraphicsMode);

	 pDC->SelectObject(pOldPen);
	 pDC->SelectObject(pOldBrush);
}

#elif defined QT_VERSION

//
//   在屏幕上绘制此圆。
//
void CEllipse::Draw(CScreenReference &ScrnRef, QPainter *pPainter, QColor crLineColor, int nLineWidth, QColor crFillColor, bool bFill, int nPenStyle)
{
	 QPen pen(crLineColor);
    pen.setStyle((Qt::PenStyle)nPenStyle);
	 pen.setWidth(nLineWidth);
	 pPainter->setPen(pen);

	 QBrush brush(crFillColor);

	 if (!bFill)
		  brush.setStyle(Qt::NoBrush);

	 pPainter->setBrush(brush);

	 CPnt ptLeftTop(m_ptCenter.x - m_fHalfMajorAxis, m_ptCenter.y + m_fHalfMinorAxis);
	 CPnt ptRightBottom(m_ptCenter.x + m_fHalfMajorAxis, m_ptCenter.y - m_fHalfMinorAxis);

	 QPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	 QPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);
	 QPoint pntCenter = (pnt1 + pnt2) / 2;

     // 注意：Qt中的rotate是顺时针旋转！！(所以ang应取相反数)
     float ang = -m_angSlant.m_fRad;
	 float cx = pntCenter.x();
	 float cy = pntCenter.y();
	 int dx = (int)(cos(ang) * cx + sin(ang) * cy);
     int dy = (int)(-sin(ang) * cx + cos(ang) * cy);

	 // 得到未经旋转的矩形外廓
	 QRect r(pnt1, pnt2);
     double rotateAngleDegree = (double)CAngle::ToDegree(ang);

	 // 设置旋转变换画法
	 pPainter->save();
     pPainter->rotate(rotateAngleDegree);
	 pPainter->translate(dx - cx, dy - cy);
	 pPainter->drawEllipse(r);
	 pPainter->restore();
}
#endif
